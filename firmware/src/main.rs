#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::spi::{Operation, SpiDevice};
use esp_backtrace as _;
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, OutputConfig};
use esp_hal::rmt::Rmt;
use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::spi::master::Spi;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_rtos::embassy::Executor;
use log::{info, warn};
use micromath::F32Ext;

extern crate alloc;

use esp_hal_smartled::{RmtSmartLeds, buffer_size, color_order};
use gravity_cube_core::physics_utils::MotionFilter;
use gravity_cube_core::{Color, Config, EffectSwitcher, Vector3D, Vector3DExt};
use smart_leds::RGB8;
use static_cell::{ConstStaticCell, StaticCell};

mod adapters;
use adapters::bmi160::ImuReading;
use adapters::ws2812::Frame;
use adapters::{Bmi160Adapter, FrameBuilder, LED_COUNT, Ws2812Display};

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

/// LED brightness out of 255. Effect colours are full brightness; at full power 384 LEDs
/// would draw far too much current, and dim colours stay clearly visible.
const BRIGHTNESS: u16 = 60;

/// Scale a full-brightness effect colour down to `BRIGHTNESS`.
fn dimmed(color: Color) -> Color {
    let scale = |c: u8| (c as u16 * BRIGHTNESS / 255) as u8;
    Color::new(scale(color.r), scale(color.g), scale(color.b))
}

type LedDriver =
    RmtSmartLeds<'static, { buffer_size::<RGB8>(LED_COUNT) }, Blocking, RGB8, color_order::Grb>;

/// Frames from the physics loop (core 0) to the LED task (core 1). One slot: core 0 computes
/// the next frame while core 1 sends the current one, then waits if core 1 is still busy.
static FRAMES: Channel<CriticalSectionRawMutex, Frame<RGB8>, 1> = Channel::new();

/// Duration of the last LED send in microseconds, for the status log.
static LED_SEND_US: AtomicU32 = AtomicU32::new(0);

/// Sends frames to the LEDs on the second core.
///
/// The RMT driver refills its RAM in a blocking loop while sending (about 11.5 ms for 384
/// LEDs), so giving it its own core keeps that time off the physics loop.
#[embassy_executor::task]
async fn led_task(display: &'static mut Ws2812Display<LedDriver>) {
    loop {
        let frame = FRAMES.receive().await;
        let start = Instant::now();
        if let Err(e) = display.show(&frame) {
            warn!("Display refresh failed: {:?}", e);
        }
        LED_SEND_US.store(start.elapsed().as_micros() as u32, Ordering::Relaxed);
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 98767);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!("╔═══════════════════════════════════════╗");
    info!("║   Gravity Cube - Rust Edition v0.1   ║");
    info!("╚═══════════════════════════════════════╝");
    info!("");

    // ========================================
    // Hardware Configuration
    // ========================================
    info!("Configuring hardware peripherals...");

    // Pin assignments for BMI160 IMU (SPI):
    // - MOSI:  GPIO 11
    // - MISO:  GPIO 13
    // - SCLK:  GPIO 12
    // - CS:    GPIO 10

    // Pin assignment for WS2812 LEDs (RMT):
    // - DATA:  GPIO 16 (via 74AHCT125 level shifter + 330 ohm)

    // Initialize SPI for BMI160 IMU
    info!("  • Setting up SPI for BMI160 IMU...");
    let config = SpiConfig::default().with_frequency(Rate::from_mhz(1));
    let spi_bus = Spi::new(peripherals.SPI2, config) // 1 MHz for BMI160
        .expect("Failed to create SPI")
        .with_sck(peripherals.GPIO12)
        .with_mosi(peripherals.GPIO11)
        .with_miso(peripherals.GPIO13);

    let cs = esp_hal::gpio::Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());
    let mut spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi_bus, cs, Delay::new())
        .expect("Failed to create SPI device");

    // Probe the chip ID before handing the bus to the driver. The first read
    // (BMI160 register 0x7F) only produces the CS edge that switches it into SPI mode.
    let mut read_reg = |reg: u8| -> Option<u8> {
        let mut val = [0u8];
        spi_device
            .transaction(&mut [Operation::Write(&[0x80 | reg]), Operation::Read(&mut val)])
            .ok()
            .map(|_| val[0])
    };
    let _ = read_reg(0x7F);
    Delay::new().delay_millis(2);
    info!(
        "  • SPI probe: BMI160 CHIP_ID(0x00)={:02X?} (expect D1), LSM6DS3 WHO_AM_I(0x0F)={:02X?} (expect 69)",
        read_reg(0x00),
        read_reg(0x0F)
    );

    // Initialize BMI160 IMU; this also calibrates the gyro, so the cube should be still
    let mut imu_opt = match Bmi160Adapter::new(spi_device, &mut Delay::new()) {
        Ok(imu) => {
            info!("  ✓ BMI160 IMU initialized successfully");
            Some(imu)
        }
        Err(_) => {
            warn!("  ✗ Failed to initialize BMI160 IMU");
            warn!("    Check your wiring:");
            warn!("      MOSI → GPIO 11");
            warn!("      MISO → GPIO 13");
            warn!("      SCLK → GPIO 12");
            warn!("      CS   → GPIO 10");
            warn!("    Continuing with default gravity (pointing down), no spin or taps");
            None
        }
    };

    // Initialize RMT for WS2812 LEDs
    info!("  • Setting up RMT for WS2812 LEDs...");
    let rmt_freq = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, rmt_freq).expect("Failed to create RMT");

    // Create RMT channel for WS2812 on GPIO 16, buffer for 384 LEDs.
    // Blocking mode refills the RMT RAM in a tight loop; in async mode the refill waits for
    // the executor and late refills corrupt the stream (visible as LED flicker). Four memory
    // blocks (all TX RAM on the S3) give 8 LEDs of slack instead of 2.
    let led_driver = LedDriver::new_with_memsize(
        esp_hal_smartled::WS2812B_TIMING,
        rmt.channel0,
        peripherals.GPIO16,
        4,
        rmt_freq,
    )
    .expect("Failed to create LED driver");

    // The driver holds a ~36 KB pulse buffer, so it lives in a static rather than on a stack
    static DISPLAY: StaticCell<Ws2812Display<LedDriver>> = StaticCell::new();
    let display = DISPLAY.init(Ws2812Display::new(led_driver));
    info!("  ✓ WS2812 LED driver initialized (384 LEDs on GPIO 16)");

    // Hand the LEDs to the second core, which sends frames while this core runs physics
    static APP_CORE_STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        peripherals.FROM_CPU_INTR1,
        APP_CORE_STACK.take(),
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            EXECUTOR
                .init(Executor::new())
                .run(|spawner| spawner.spawn(led_task(display).unwrap()));
        },
    );
    info!("  ✓ LED output running on core 1");

    info!("");
    info!("Hardware initialization complete!");
    info!("");

    // ========================================
    // Simulation Configuration
    // ========================================
    info!("Initializing physics simulation...");

    // Each effect sets its own particle count and colour
    let sim_config = Config {
        delay_ms: 0,
        ..Config::default()
    };
    let mut simulation = EffectSwitcher::new(sim_config);

    info!(
        "  • Cube size: {}x{}x{}",
        sim_config.cube_size, sim_config.cube_size, sim_config.cube_size
    );
    info!(
        "  • Effect: {} ({} particles)",
        simulation.name(),
        simulation.config().num
    );
    info!("  • Update rate: {}ms", sim_config.delay_ms);
    info!("  • Double-tap the cube to switch effects");
    info!("");

    // Wait for sensors to stabilize
    Timer::after(Duration::from_secs(1)).await;

    info!("Starting main loop...");
    info!("═══════════════════════════════════════");
    info!("");

    // ========================================
    // Main Loop
    // ========================================
    let mut loop_count = 0u32;
    // Track gravity with the gyro and triple anything else, so shaking the cube sloshes the fluid
    let mut motion = MotionFilter::new(0.05, 3.0);
    let mut frame = FrameBuilder::<RGB8>::new();
    let mut last_read = Instant::now();

    loop {
        // No IMU (or a failed read): default gravity towards z == 0, physically the bottom
        let resting = || ImuReading {
            acceleration: Vector3D::new(0.0, 0.0, -1.0),
            angular_velocity: Vector3D::new(0.0, 0.0, 0.0),
        };

        // Read acceleration (with magnitude, so shaking sloshes the fluid) and rotation rate
        let reading = if let Some(ref mut imu) = imu_opt {
            if let Ok(true) = imu.double_tapped() {
                simulation.next_effect();
                info!("Double tap: switched to {} effect", simulation.name());
            }
            imu.read().unwrap_or_else(|_| {
                warn!("Failed to read IMU, using default gravity");
                resting()
            })
        } else {
            resting()
        };
        let acceleration = reading.acceleration;

        // How far the cube turned since the last reading
        let now = Instant::now();
        let dt = (now - last_read).as_micros() as f32 * 1e-6;
        last_read = now;
        let rotation = reading.angular_velocity * dt;

        // Run physics simulation step with gravity plus amplified motion, and the spin
        let frame_start = Instant::now();
        simulation.set_rotation(rotation);
        simulation.step(&motion.apply(acceleration, rotation));
        let physics_time = frame_start.elapsed();

        // Build the frame and hand it to core 1. This only waits if core 1 is still sending
        // the frame before last, so physics and LED output overlap
        let color = dimmed(simulation.config().color);
        frame.clear();
        for pixel in simulation.active_pixels() {
            frame.set_pixel(pixel.position, color);
        }
        FRAMES.send(*frame.frame()).await;

        let frame_time = frame_start.elapsed();

        // Log status periodically (every 100 loops)
        loop_count = loop_count.wrapping_add(1);
        if loop_count.is_multiple_of(100) {
            let active_count = simulation.active_pixels().count();
            let magnitude = (acceleration.x * acceleration.x
                + acceleration.y * acceleration.y
                + acceleration.z * acceleration.z)
                .sqrt();
            info!(
                "Loop {}: {} {} particles, accel: ({:.2}, {:.2}, {:.2}) |{:.2}|g, gyro: ({:.2}, {:.2}, {:.2}) rad/s, physics {}us, frame {}us, LED send {}us",
                loop_count,
                active_count,
                simulation.name(),
                acceleration.x,
                acceleration.y,
                acceleration.z,
                magnitude,
                reading.angular_velocity.x,
                reading.angular_velocity.y,
                reading.angular_velocity.z,
                physics_time.as_micros(),
                frame_time.as_micros(),
                LED_SEND_US.load(Ordering::Relaxed)
            );
        }

        // Delay for next frame
        Timer::after(Duration::from_millis(simulation.config().delay_ms as u64)).await;
    }
}

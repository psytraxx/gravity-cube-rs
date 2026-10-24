#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal::spi::{Operation, SpiDevice};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, OutputConfig};
use esp_hal::rmt::Rmt;
use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use log::{info, warn};
use micromath::F32Ext;

extern crate alloc;

use esp_hal_smartled::{RmtSmartLeds, WS2812_TIMING, buffer_size, color_order};
use gravity_cube_core::{Color, Config, Simulation, SimulationEffect, Vector3D, Vector3DExt};
use smart_leds::RGB8;

mod adapters;
use adapters::{Bmi160Adapter, Ws2812Display};

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

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

    // Initialize BMI160 IMU
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
            warn!("    Continuing with default gravity (pointing down)");
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
    let led_driver =
        RmtSmartLeds::<{ buffer_size::<RGB8>(384) }, _, RGB8, color_order::Grb>::new_with_memsize(
            WS2812_TIMING,
            rmt.channel0,
            peripherals.GPIO16,
            4,
            rmt_freq,
        )
        .expect("Failed to create LED driver");

    let mut display = Ws2812Display::new(led_driver, 384);
    info!("  ✓ WS2812 LED driver initialized (384 LEDs on GPIO 16)");

    info!("");
    info!("Hardware initialization complete!");
    info!("");

    // ========================================
    // Simulation Configuration
    // ========================================
    info!("Initializing physics simulation...");

    // White at 50% brightness: equal channels look the same on every panel and draw
    // less current than the default cyan
    let sim_config = Config {
        color: Color::new(128, 128, 128),
        ..Config::default()
    };

    let mut simulation = Simulation::new(sim_config);
    simulation.populate(sim_config.num_particles);

    info!(
        "  • Cube size: {}x{}x{}",
        sim_config.cube_size, sim_config.cube_size, sim_config.cube_size
    );
    info!("  • Active particles: {}", sim_config.num_particles);
    info!("  • Update rate: {}ms", sim_config.delay_ms);
    info!(
        "  • LED color: RGB({}, {}, {})",
        sim_config.color.r, sim_config.color.g, sim_config.color.b
    );
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

    loop {
        // Read acceleration from IMU (includes magnitude, not just direction)
        // This makes particles respond to shaking and sudden movements
        let acceleration = if let Some(ref mut imu) = imu_opt {
            match imu.read_acceleration() {
                Ok(a) => a,
                Err(_) => {
                    warn!("Failed to read IMU, using default gravity");
                    Vector3D::new(0.0, 0.0, -1.0)
                }
            }
        } else {
            // No IMU available, use default gravity (towards z == 0, physically the bottom)
            Vector3D::new(0.0, 0.0, -1.0)
        };

        // Run physics simulation step with actual acceleration
        simulation.step(&acceleration);

        // Clear display
        if let Err(e) = display.clear().await {
            warn!("Display clear failed: {:?}", e);
        }

        // Update display with active pixels
        for pixel in simulation.active_pixels() {
            if let Err(e) = display.set_pixel(pixel.position, sim_config.color).await {
                warn!("Failed to set pixel at {:?}: {:?}", pixel.position, e);
            }
        }

        // Refresh display to show changes
        if let Err(e) = display.refresh().await {
            warn!("Display refresh failed: {:?}", e);
        }

        // Log status periodically (every 100 loops)
        loop_count = loop_count.wrapping_add(1);
        if loop_count.is_multiple_of(100) {
            let active_count = simulation.active_pixels().count();
            let magnitude = (acceleration.x * acceleration.x
                + acceleration.y * acceleration.y
                + acceleration.z * acceleration.z)
                .sqrt();
            info!(
                "Loop {}: {} particles, accel: ({:.2}, {:.2}, {:.2}) |{:.2}|g",
                loop_count, active_count, acceleration.x, acceleration.y, acceleration.z, magnitude
            );
        }

        // Delay for next frame
        Timer::after(Duration::from_millis(sim_config.delay_ms as u64)).await;
    }
}

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::gpio::Level;
use esp_hal::spi::master::Spi;
use esp_hal::delay::Delay;
use esp_hal::rmt::Rmt;
use log::{info, warn};

extern crate alloc;

use firmware::domain::{Simulation, Config, Color, Vector3D};
use firmware::adapters::{Bmi160Adapter, Ws2812Display};
use firmware::ports::{ImuPort, DisplayPort};
use smart_leds::{RGB8, SmartLedsWrite};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

/// Simple RNG using ESP32 hardware random
struct SimpleRng;

impl SimpleRng {
    fn next_bool(&self) -> bool {
        // Note: In a real implementation, you'd use esp_hal's RNG
        // For now, we'll use a simple alternating pattern
        // This should be replaced with actual hardware RNG
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER = COUNTER.wrapping_add(1);
            (COUNTER & 1) == 0
        }
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 98767);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("╔═══════════════════════════════════════╗");
    info!("║   Gravity Cube - Rust Edition v0.1   ║");
    info!("╚═══════════════════════════════════════╝");
    info!("");

    // ========================================
    // Hardware Configuration
    // ========================================
    info!("Configuring hardware peripherals...");

    // Pin assignments for BMI160 IMU (SPI):
    // - MOSI:  GPIO 23
    // - MISO:  GPIO 19
    // - SCLK:  GPIO 18
    // - CS:    GPIO 5

    // Pin assignment for WS2812 LEDs (RMT):
    // - DATA:  GPIO 37

    // Initialize SPI for BMI160 IMU
    info!("  • Setting up SPI for BMI160 IMU...");
    let spi_bus = Spi::new(peripherals.SPI2, 1_000_000_u32) // 1 MHz for BMI160
        .expect("Failed to create SPI")
        .with_sck(peripherals.GPIO18)
        .with_mosi(peripherals.GPIO23)
        .with_miso(peripherals.GPIO19);

    let cs = esp_hal::gpio::Output::new(
        peripherals.GPIO5,
        Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi_bus, cs, Delay::new());

    // Initialize BMI160 IMU
    let delay = Delay::new();
    let mut imu = match Bmi160Adapter::new(spi_device, delay) {
        Ok(imu) => {
            info!("  ✓ BMI160 IMU initialized successfully");
            imu
        }
        Err(_) => {
            warn!("  ✗ Failed to initialize BMI160 IMU");
            warn!("    Check your wiring:");
            warn!("      MOSI → GPIO 23");
            warn!("      MISO → GPIO 19");
            warn!("      SCLK → GPIO 18");
            warn!("      CS   → GPIO 5");
            panic!("BMI160 initialization failed");
        }
    };

    // Initialize RMT for WS2812 LEDs
    info!("  • Setting up RMT for WS2812 LEDs...");
    let rmt = Rmt::new(peripherals.RMT, 80_000_000_u32)
        .expect("Failed to create RMT");

    // Create RMT channel for WS2812 on GPIO 37
    let rmt_buffer = smartLedBuffer!(384); // 384 LEDs
    let led_driver = SmartLedsAdapter::new(
        rmt.channel0,
        peripherals.GPIO37.degrade(),
        rmt_buffer,
    );

    let mut display = Ws2812Display::new(led_driver, 384);
    info!("  ✓ WS2812 LED driver initialized (384 LEDs on GPIO 37)");

    info!("");
    info!("Hardware initialization complete!");
    info!("");

    // ========================================
    // Simulation Configuration
    // ========================================
    info!("Initializing physics simulation...");

    let sim_config = Config {
        cube_size: 8,
        num_particles: 200,
        velocity: 2.0,
        delay_ms: 35,
        color: Color::new(10, 10, 100), // Blue-ish
    };

    let mut simulation = Simulation::new(sim_config);
    simulation.populate_particles(sim_config.num_particles);

    info!("  • Cube size: {}x{}x{}", sim_config.cube_size, sim_config.cube_size, sim_config.cube_size);
    info!("  • Active particles: {}", sim_config.num_particles);
    info!("  • Velocity: {}", sim_config.velocity);
    info!("  • Update rate: {}ms", sim_config.delay_ms);
    info!("  • LED color: RGB({}, {}, {})", sim_config.color.r, sim_config.color.g, sim_config.color.b);
    info!("");

    let rng = SimpleRng;

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
        // Read gravity from IMU
        let gravity = match imu.read_gravity().await {
            Ok(g) => g,
            Err(_) => {
                warn!("Failed to read IMU, using default gravity");
                Vector3D::new(0.0, 0.0, -1.0)
            }
        };

        // Run physics simulation step
        simulation.step(&gravity, || rng.next_bool());

        // Clear display
        if let Err(_) = display.clear().await {
            warn!("Display clear failed");
        }

        // Update display with active pixels
        for pixel in simulation.active_pixels() {
            if let Err(_) = display.set_pixel(pixel.position, sim_config.color).await {
                // Silently ignore individual pixel errors
            }
        }

        // Refresh display to show changes
        if let Err(_) = display.refresh().await {
            warn!("Display refresh failed");
        }

        // Log status periodically (every 100 loops)
        loop_count = loop_count.wrapping_add(1);
        if loop_count % 100 == 0 {
            let active_count = simulation.active_pixels().count();
            info!("Loop {}: {} active particles, gravity: ({:.2}, {:.2}, {:.2})",
                  loop_count, active_count, gravity.x, gravity.y, gravity.z);
        }

        // Delay for next frame
        Timer::after(Duration::from_millis(sim_config.delay_ms as u64)).await;
    }
}

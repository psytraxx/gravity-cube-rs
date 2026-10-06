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
use gravity_cube_core::{Config, Simulation, SimulationEffect, Vector3D, Vector3DExt};
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

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 98767);

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
    // - MOSI:  GPIO 23
    // - MISO:  GPIO 19
    // - SCLK:  GPIO 18
    // - CS:    GPIO 5

    // Pin assignment for WS2812 LEDs (RMT):
    // - DATA:  GPIO 16 (changed from 37 for ESP32 compatibility)

    // Initialize SPI for BMI160 IMU
    info!("  • Setting up SPI for BMI160 IMU...");
    let config = SpiConfig::default().with_frequency(Rate::from_mhz(1));
    let spi_bus = Spi::new(peripherals.SPI2, config) // 1 MHz for BMI160
        .expect("Failed to create SPI")
        .with_sck(peripherals.GPIO18)
        .with_mosi(peripherals.GPIO23)
        .with_miso(peripherals.GPIO19);

    let cs = esp_hal::gpio::Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi_bus, cs, Delay::new())
        .expect("Failed to create SPI device");

    // Initialize BMI160 IMU
    let mut imu_opt = match Bmi160Adapter::new(spi_device) {
        Ok(imu) => {
            info!("  ✓ BMI160 IMU initialized successfully");
            Some(imu)
        }
        Err(_) => {
            warn!("  ✗ Failed to initialize BMI160 IMU");
            warn!("    Check your wiring:");
            warn!("      MOSI → GPIO 23");
            warn!("      MISO → GPIO 19");
            warn!("      SCLK → GPIO 18");
            warn!("      CS   → GPIO 5");
            warn!("    Continuing with default gravity (pointing down)");
            None
        }
    };

    // Initialize RMT for WS2812 LEDs
    info!("  • Setting up RMT for WS2812 LEDs...");
    let rmt_freq = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, rmt_freq)
        .expect("Failed to create RMT")
        .into_async();

    // Create RMT channel for WS2812 on GPIO 16
    // Buffer size for 384 LEDs
    let led_driver = RmtSmartLeds::<{ buffer_size::<RGB8>(384) }, _, RGB8, color_order::Grb>::new(
        WS2812_TIMING,
        rmt.channel0,
        peripherals.GPIO16,
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

    let sim_config = Config::default();

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
            // No IMU available, use default gravity
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

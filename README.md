# Gravity Cube - Rust Firmware

A physics simulation running on ESP32 that displays gravity-responsive particles on a 3D LED cube, implemented in Rust using Embassy async runtime and hexagonal architecture.

## Architecture

This firmware follows **Hexagonal Architecture** (Ports and Adapters pattern) for clean separation of concerns:

```
┌─────────────────────────────────────────────┐
│              Application                    │
│         (Embassy async tasks)               │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────┴───────────────────────────┐
│              Domain Layer                   │
│  ┌─────────────────────────────────────┐   │
│  │ Simulation Engine                    │   │
│  │ - Physics calculations               │   │
│  │ - Particle movement                  │   │
│  │ - Collision detection                │   │
│  └─────────────────────────────────────┘   │
│  ┌─────────────────────────────────────┐   │
│  │ Domain Types                         │   │
│  │ - Vector3D, Position, Pixel, Color   │   │
│  └─────────────────────────────────────┘   │
└─────────────────┬───────────────────────────┘
                  │
         ┌────────┴─────────┐
         │                  │
    ┌────┴────┐        ┌────┴────┐
    │  Ports  │        │ Adapters│
    │(Traits) │        │ (Impls) │
    └─────────┘        └─────────┘
         │                  │
    ┌────┴────┐        ┌────┴────┐
    │ ImuPort │◄───────┤ BMI160  │
    │DisplayP.│◄───────┤ WS2812  │
    │RandomP. │◄───────┤EspRandom│
    └─────────┘        └─────────┘
```

### Directory Structure

```
firmware/
├── src/
│   ├── domain/              # Core business logic
│   │   ├── types.rs         # Domain types (Vector3D, Pixel, etc.)
│   │   └── simulation.rs    # Physics simulation engine
│   ├── ports/               # Interfaces (traits)
│   │   ├── imu.rs           # IMU sensor port
│   │   ├── display.rs       # LED display port
│   │   └── random.rs        # Random number port
│   ├── adapters/            # Hardware implementations
│   │   ├── bmi160.rs        # BMI160 IMU adapter
│   │   ├── ws2812.rs        # WS2812 LED adapter
│   │   ├── panel_mapper.rs  # 3D→2D panel mapping
│   │   └── esp_random.rs    # ESP32 RNG adapter
│   └── bin/
│       └── main.rs          # Application entry point
```

## Hardware

- **MCU**: ESP32-S3
- **IMU**: BMI160 (via SPI)
- **LEDs**: 384x WS2812B (6 panels of 8x8, controlled via SPI)
- **LED GPIO**: GPIO 37

## Dependencies

### Math Libraries
- **micromath**: Fast floating-point math for embedded systems
- **num-traits**: Generic numeric trait definitions

### Hardware Drivers
- **bmi160**: BMI160 IMU driver
- **ws2812-spi**: WS2812 LED driver using SPI
- **smart-leds**: LED abstraction layer

### Embedded Framework
- **embassy-executor**: Async task executor
- **embassy-time**: Async timers and delays
- **esp-hal**: ESP32 hardware abstraction layer
- **esp-rtos**: RTOS integration for Embassy

## Building

```bash
cd firmware
cargo build --release
```

## Flashing

```bash
cargo run --release
```

## Configuration

The simulation can be configured in `main.rs`:

```rust
let sim_config = Config {
    cube_size: 8,           // 8x8x8 cube
    num_particles: 200,     // Number of active particles
    velocity: 2.0,          // Particle movement speed
    delay_ms: 35,           // Simulation tick rate
    color: Color::new(10, 10, 100), // RGB color
};
```

## Hardware Setup

### BMI160 SPI Connection
You'll need to configure SPI pins for the BMI160 in `main.rs`:

```rust
let spi = /* configure SPI peripheral */;
let delay = /* configure delay provider */;
let mut imu = Bmi160Adapter::new(spi, delay)
    .expect("Failed to initialize BMI160");
```

### WS2812 LED Connection
Configure SPI for WS2812 on GPIO 37:

```rust
let spi = /* configure SPI for WS2812 timing */;
let mut display = Ws2812Display::new(spi, 384);
```

## How It Works

1. **Gravity Sensing**: The BMI160 IMU reads accelerometer data and converts it to a normalized gravity vector
2. **Physics Simulation**: The simulation engine calculates particle movement based on:
   - Gravity direction (down)
   - Perpendicular movement (left/right)
   - Collision detection
   - Velocity scaling
3. **Panel Mapping**: 3D cube coordinates are mapped to 2D LED panel indices accounting for:
   - Panel orientation (0°, 90°, 180°, 270°)
   - Panel direction (N, S, E, W, Up, Down)
   - Axis inversions
4. **Display Update**: Active pixels are rendered to the WS2812 LED strips

## Benefits of Hexagonal Architecture

1. **Testability**: Domain logic can be tested without hardware
2. **Flexibility**: Easy to swap hardware implementations
3. **Maintainability**: Clear separation between business logic and infrastructure
4. **Portability**: Domain layer is hardware-agnostic

## Original C Implementation

This is a port of the original C implementation found in `/main/main.c`. Key improvements:

- ✅ Type safety with Rust's type system
- ✅ Memory safety without GC overhead
- ✅ Async/await with Embassy
- ✅ Hexagonal architecture for testability
- ✅ No-std embedded Rust best practices
- ✅ Using established crates (bmi160, ws2812-spi)

## License

Same as the original Gravity Cube project.

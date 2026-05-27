# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Gravity Cube is a **fluid physics simulation** of gravity-responsive particles on a 3D LED cube (8x8x8). Available in two flavors:

1. **Firmware**: ESP32 hardware with BMI160 IMU and WS2812 LEDs
2. **Desktop**: Interactive 3D visualization on macOS and Linux with GPU-accelerated rendering

Both share the same fluid physics simulation core through a Cargo workspace structure with hexagonal architecture. The simulation uses continuous floating-point particle positions with velocity tracking, collision detection, and fluid-like behavior.

## Workspace Structure

This is a Cargo workspace with three crates:

- **core/**: Platform-agnostic library (no_std compatible)
  - Domain logic: effect.rs, effects/, types.rs, physics_utils.rs

- **firmware/**: ESP32 binary
  - Target: `xtensa-esp32-none-elf`
  - Toolchain: ESP Rust (channel = "esp")
  - Runtime: `esp-rtos` with Embassy async
  - Adapters: BMI160 IMU, WS2812 LEDs, PanelMapper

- **desktop/**: macOS/Linux binary
  - Target: `aarch64-apple-darwin`, `x86_64-apple-darwin`, or `x86_64-unknown-linux-gnu`
  - Toolchain: Rust stable
  - Rendering: wgpu (modern GPU API with Vulkan/Metal/DX12 backends)

## Building and Running

### Desktop (macOS)

```bash
# Build
cargo build -p gravity-cube-desktop --release

# Run
cargo run -p gravity-cube-desktop --release
```

### Desktop (Linux)

```bash
# Build
cargo build -p gravity-cube-desktop --release

# Run (use X11 backend for best compatibility)
WINIT_UNIX_BACKEND=x11 cargo run -p gravity-cube-desktop --release

# Or use the convenience script
./run-desktop.sh
```

### Firmware (ESP32)

```bash
# Navigate to firmware directory
cd firmware

# Build
cargo build --release

# Flash to device
cargo run --release
```

The firmware runner is configured in `firmware/.cargo/config.toml` to use `espflash` with automatic monitoring.

## Hardware Pin Configuration (Firmware)

As documented in `firmware/src/main.rs`:

**BMI160 IMU (SPI):**
- MOSI: GPIO 23
- MISO: GPIO 19
- SCLK: GPIO 18
- CS: GPIO 5

**WS2812 LEDs (RMT):**
- DATA: GPIO 16 (384 LEDs total: 6 panels × 64 LEDs)

## Architecture

The project uses a simple layered architecture with shared core logic:

### 1. Domain Layer (`core/src/`)

Core business logic with no hardware dependencies:
- **types.rs**: Value objects (Vector3D, Position, Pixel, Particle, Color, Config)
  - `Particle`: Continuous f32 position and velocity vectors
  - `Config`: Simulation configuration parameters
- **effect.rs**: `SimulationEffect` trait defining the interface for all simulation implementations
- **effects/**: Simulation effect implementations
  - **fluid.rs**: Fluid physics simulation with particle collision detection
  - **boids.rs**: Flocking/swarming behavior simulation
- **physics_utils.rs**: Shared physics utilities used across effects
  - `apply_axis_boundary()`: Single-axis boundary collision with damping
  - `apply_boundary_3d()`: 3D boundary collision helper
  - `clamp_speed()`: Velocity magnitude limiting
  - `distance_squared()`: Fast distance calculation for collision detection

Key fluid simulation algorithm:
- Continuous floating-point particle positions (not grid-locked)
- Per-particle velocity tracking with smoothing and decay
- Boundary collision with damping (particles bounce off walls)
- O(n²) particle-particle collision detection and response
- Velocity averaging for energy conservation

### 2. Adapters Layer (Platform-Specific)

**Firmware** (`firmware/src/adapters/`):
- **bmi160.rs**: BMI160 IMU adapter (SPI communication)
- **ws2812.rs**: WS2812 LED adapter (RMT-based using esp-hal-smartled)
- **panel_mapper.rs**: Maps 3D cube coordinates to 2D LED panel indices
  - 6 panels with different orientations (0°, 90°, 180°, 270°)
  - Panel directions (N, S, E, W, Up, Down)
  - Axis inversions and coordinate transformations
  - Lookup table for O(1) position-to-panel-mask conversion

**Desktop** (`desktop/src/`):
- **renderer3d.rs**: GPU-accelerated voxel renderer using wgpu
  - Instanced rendering (one draw call per frame)
  - Converts continuous particle positions to discrete grid voxels
  - Keeps fastest particle per grid cell for brightness feedback
  - HSV-based color system with speed-based brightness
- **camera.rs**: Static perspective camera
- **cube_transform.rs**: Handles cube rotation and local gravity transformation
  - Rotates cube based on mouse/keyboard input
  - Transforms world gravity to cube's local coordinate system
- **input.rs**: Mouse drag and keyboard input handling
- **shaders/**: WGSL shaders
  - `vertex3d.wgsl`: Voxel vertex shader with instancing
  - `fragment3d.wgsl`: Simple directional lighting
  - `wireframe.wgsl`: Cube outline rendering

### Application Layer

**Desktop** (`desktop/src/main.rs`)

Event loop with every-frame physics updates:
1. Handle mouse/keyboard input
2. Update cube rotation and gravity transformation
3. Run simulation step (every frame for smooth animation)
4. Render particles to GPU
5. Present frame

**Firmware** (`firmware/src/main.rs`)

Embassy async tasks orchestrating the main loop:
1. Read gravity from IMU
2. Run simulation step
3. Clear display
4. Update pixels (converts continuous positions to discrete grid)
5. Refresh display
6. Delay for frame rate (35ms default)

## Important Implementation Details

### Random Number Generation
Uses ESP32 hardware RNG wrapped in `SimpleRng` struct. The `next_bool()` method uses `is_multiple_of(2)` for random left/right particle movement preferences.

### Memory Allocation
- Custom heap allocator in DRAM2: `esp_alloc::heap_allocator!(size: 98767)`
- No-std environment with alloc crate

### Fluid Physics Simulation

The simulation uses a continuous floating-point particle system with fluid-like behavior:

**Particle System:**
- 256 particles (desktop) or 128 particles (firmware) with continuous f32 positions
- Each particle has position (x, y, z) and velocity (vx, vy, vz) vectors
- Particles move smoothly through continuous space, not locked to grid

**Physics Algorithm (per frame):**
1. **Velocity Update**: `v = v * smoothing + (gravity * strength)`
   - Velocity smoothing: 0.9 (creates smooth acceleration)
   - Gravity strength: 0.08
2. **Velocity Clamping**: Limit to ±0.6 max velocity
3. **Position Update**: `position += velocity`
4. **Boundary Collision**: Reflect velocity with 0.92 damping when hitting walls
5. **Velocity Decay**: `v *= 0.95` (friction/air resistance)
6. **Particle Collision** (O(n²)):
   - Detect particles within distance < 1.0
   - Separate overlapping particles
   - Average velocities for energy conservation

**Display Conversion:**
- Continuous positions rounded to nearest grid cell for LED display
- Desktop renderer keeps fastest particle per cell for brightness feedback

### Simulation Configuration

**Firmware**: Edit in `firmware/src/main.rs`

**Desktop**: Edit in `desktop/src/main.rs`

Both use the same Config struct with fluid physics parameters:
```rust
let sim_config = Config {
    cube_size: 8,
    num_particles: 256,          // 256 (desktop) or 128 (firmware)
    velocity: 2.0,               // Legacy field (unused by fluid sim)
    delay_ms: 35,                // Frame delay (firmware only)
    color: Color::new(100, 180, 255),  // Cyan for water appearance

    // Fluid physics parameters
    gravity: 0.08,               // Gravity strength multiplier
    damping: 0.92,               // Velocity reduction on boundary bounce
    max_velocity: 0.6,           // Maximum velocity clamp
    velocity_smoothing: 0.9,     // Acceleration integration smoothing
    velocity_decay: 0.95,        // Per-frame velocity decay (friction)
    collision_repulsion: 0.5,    // Particle separation force
    collision_damping: 0.3,      // Collision velocity dampening
};
```

## Dependencies

### Core (Shared)
- **micromath**: Fast no-std floating-point math

### Firmware
- **esp-hal**: Hardware abstraction (ESP32 peripherals, SPI, RMT, RNG)
- **esp-rtos**: RTOS integration enabling Embassy
- **embassy-executor**: Async task executor
- **embassy-time**: Async timers
- **bmi160**: IMU driver
- **esp-hal-smartled**: WS2812 LED driver using RMT

### Desktop
- **wgpu**: GPU graphics API
- **winit**: Window and input handling
- **glam**: 3D math library (faster than micromath for desktop)
- **pollster**: Simple async executor for setup
- **rand**: Random number generation
- **bytemuck**: Safe GPU buffer casting

## Build Configuration

### Optimization
- Dev builds: opt-level = "s" (Rust debug is too slow for embedded)
- Release builds: LTO = 'fat', codegen-units = 1, opt-level = 's'

### Linker
Custom linker error handler in `build.rs` provides helpful messages for common issues (missing defmt, esp-rtos, etc.).

## Desktop Controls

- **Mouse Drag**: Rotate cube (changes gravity direction in cube's local space)
- **Arrow Keys**: Alternative rotation controls (Up/Down for pitch, Left/Right for yaw)
- **Escape**: Exit application

The desktop version uses cube rotation with local gravity transformation. World gravity always points down (0, -1, 0), but when you rotate the cube, this gravity vector is transformed into the cube's local coordinate system. This creates the effect of particles flowing in the direction the cube is tilted, matching the behavior of tilting the real hardware.

## Testing Strategy

Core domain layer (effect.rs, effects/, types.rs, physics_utils.rs) is hardware-agnostic and can be unit tested. Firmware adapters require hardware. Desktop adapters can be tested without special hardware.

### Running Tests

```bash
# Test core library only (firmware crate requires ESP toolchain)
cargo test -p gravity-cube-core

# Test desktop crate
cargo test -p gravity-cube-desktop
```

### Important: Micromath Precision Considerations

The core library uses `micromath` for no_std compatibility, which provides fast approximations for floating-point operations. These approximations have lower precision than standard library implementations:

- **Square root**: ~2-3% error vs. exact value (e.g., sqrt(3) returns ~1.75 instead of 1.732)
- **Test tolerances**: Physics tests use tolerances of 0.01-0.02 to account for approximation errors
- **Production impact**: Minimal - the simulation prioritizes speed over precision for real-time performance

When writing new tests for physics calculations, use appropriate tolerances (typically 0.01-0.02) rather than expecting exact floating-point equality.

## Common Issues

### Firmware
- If BMI160 initialization fails, verify SPI connections match the documented GPIO pins
- If WS2812 LEDs don't work, ensure GPIO 16 is correctly connected and 384 LEDs are powered adequately

### Desktop (macOS)
- For Apple Silicon Macs: target is auto-detected as `aarch64-apple-darwin`
- For Intel Macs: target is auto-detected as `x86_64-apple-darwin`

### Desktop (Linux)
- If you encounter EGL initialization errors, use X11 backend: `WINIT_UNIX_BACKEND=x11`
- On Wayland systems, wgpu may work better with X11 compatibility mode
- The provided `./run-desktop.sh` script automatically uses X11 backend

## Important Notes

- The workspace contains both ESP32 and desktop targets
- Desktop builds use auto-detected native target (no need to specify explicitly on macOS/Linux)
- Firmware requires ESP Rust toolchain; desktop requires stable toolchain
- Core library is no_std but can enable "std" feature for desktop
- On Linux with Wayland, use `WINIT_UNIX_BACKEND=x11` for best compatibility

## Original Implementation

This project is a Rust port of a fluid cube simulation. The physics algorithm is ported from a C/Arduino implementation with continuous floating-point particle positions, velocity tracking, and collision detection. The Rust version adds:
- Type safety and memory safety
- Layered architecture with clear separation of concerns
- Platform abstraction (works on both ESP32 firmware and desktop)
- GPU-accelerated rendering for desktop visualization
- no_std compatibility for embedded systems

# Gravity Cube - Rust Edition

A **fluid physics simulation** of gravity-responsive particles on a 3D LED cube, implemented in Rust using hexagonal architecture. Available in two flavors:

- **Firmware**: Runs on ESP32 hardware with BMI160 IMU and WS2812 LEDs
- **Desktop**: Interactive 3D visualization on macOS/Linux with GPU-accelerated rendering

Both versions share the same fluid physics simulation core with continuous particle movement, velocity tracking, and collision detection, demonstrating the power of hexagonal architecture for code reuse across platforms.

**Enclosure:** a 3D-printable translucent case for the cube. You can [view it in 3D and download the STLs](https://psytraxx.github.io/gravity-cube-rs/voxel-shell-65x65/viewer.html). Details are in [Enclosure](#-enclosure).

---

## Project Structure

This project uses a Cargo workspace with three crates:

```
gravity-cube-rs/
├── Cargo.toml              # Workspace root
├── core/                   # Platform-agnostic library
│   ├── src/
│   │   ├── effect.rs       # SimulationEffect trait
│   │   ├── effects/        # Simulation implementations (fluid, boids)
│   │   ├── physics_utils.rs # Shared physics utilities
│   │   └── types.rs        # Domain types (Particle, Vector3D, etc.)
│   └── Cargo.toml
├── firmware/               # ESP32 firmware
│   ├── src/
│   │   ├── main.rs         # Firmware entry point
│   │   └── adapters/       # BMI160 IMU, WS2812 LED adapters
│   ├── build.rs
│   ├── .cargo/config.toml  # ESP32 target configuration
│   └── Cargo.toml
├── case/                   # 3D-printable enclosure (published to GitHub Pages)
│   ├── concepts.html       # Comparison of five frame concepts
│   ├── voxel-shell-65x65/  # STLs, generator, fit checks, 3D viewer
│   └── voxel-shell-80x80/  # same enclosure for 80 x 80 mm flexible 8x8 panels
└── desktop/                # Desktop simulator
    ├── shaders/            # WGSL shaders (vertex, fragment, wireframe)
    ├── src/
    │   ├── main.rs         # Desktop entry point
    │   ├── renderer3d.rs   # GPU-accelerated voxel renderer
    │   ├── camera.rs       # Static perspective camera
    │   ├── cube_transform.rs # Rotation and gravity transform
    │   └── input.rs        # Mouse/keyboard input handling
    └── Cargo.toml
```

---

## Architecture

Both versions follow **Hexagonal Architecture** (Ports and Adapters pattern): the core crate defines the domain and platform adapters feed it gravity and render its particles:

```
┌─────────────────────────────────────────────┐
│          Application Layer                  │
│   (ESP32 async / Desktop event loop)       │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────┴───────────────────────────┐
│          Domain Layer (core crate)          │
│  ┌─────────────────────────────────────┐   │
│  │ SimulationEffect Trait               │   │
│  │ - Common interface for all effects   │   │
│  └─────────────────────────────────────┘   │
│  ┌─────────────────────────────────────┐   │
│  │ Effect Implementations               │   │
│  │ - FluidSimulation (particle physics) │   │
│  │ - BoidsSimulation (flocking behavior)│   │
│  └─────────────────────────────────────┘   │
│  ┌─────────────────────────────────────┐   │
│  │ Physics Utilities                    │   │
│  │ - Boundary collision detection       │   │
│  │ - Speed clamping & distance calc     │   │
│  └─────────────────────────────────────┘   │
│  ┌─────────────────────────────────────┐   │
│  │ Domain Types                         │   │
│  │ - Particle (pos + velocity)          │   │
│  │ - Vector3D, Position, Pixel, Color   │   │
│  └─────────────────────────────────────┘   │
└─────────────────┬───────────────────────────┘
                  │
         ┌────────┴─────────┐
         │                  │
    ┌────┴─────┐      ┌─────┴────┐
    │ Firmware │      │ Desktop  │
    │ adapters │      │ adapters │
    ├──────────┤      ├──────────┤
    │ BMI160   │      │ Mouse/   │
    │ (gravity)│      │ keyboard │
    │ WS2812   │      │ (gravity)│
    │ (LEDs)   │      │ wgpu     │
    │          │      │ renderer │
    └──────────┘      └──────────┘
```

---

## 🎮 Desktop Simulator (macOS/Linux)

Interactive 3D visualization with GPU-accelerated rendering and fluid physics.

### Features

- **GPU Rendering**: wgpu 30-based instanced voxel rendering with WGSL shaders
- **Fluid Simulation**: 256 particles with continuous positions, velocity tracking, and collisions
- **Interactive Gravity**: Rotate cube with mouse/keyboard to change gravity direction
- **Real-time Updates**: Physics runs every frame (~60 FPS) for smooth animation
- **Visual Feedback**: Speed-based brightness using HSV color system

### Requirements

- macOS (Apple Silicon or Intel) or Linux
- Rust stable toolchain
- GPU with Vulkan/Metal/DX12 support

### Building

```bash
# Build desktop version (auto-detects target)
cargo build -p gravity-cube-desktop --release
```

### Running

```bash
# macOS
cargo run -p gravity-cube-desktop --release

# Linux (use X11 backend for best compatibility)
WINIT_UNIX_BACKEND=x11 cargo run -p gravity-cube-desktop --release

```

### Controls

- **Mouse Drag**: Rotate cube (changes gravity in local space; fast drags swirl the fluid)
- **Double Click**: Switch effect (fluid → boids → starfield)
- **Arrow Keys**: Alternative rotation (Up/Down = pitch, Left/Right = yaw)
- **Escape**: Exit

The cube rotates based on your input, and world gravity (always pointing down) is transformed into the cube's local coordinate system. This creates realistic fluid behavior as you tilt the cube!

### Configuration

The desktop uses `Config::default()` (see `core/src/types.rs`):

```rust
Config {
    cube_size: 8,
    num: 256,                    // Particles the effect runs with
    delay_ms: 35,                // Firmware frame delay (desktop updates every frame)
    color: Color::DEFAULT,
}
```

The physics tuning constants (gravity, damping, max velocity, smoothing, decay, collision repulsion/damping) are compile-time constants at the top of `core/src/effects/fluid.rs`.

---

## 🔌 ESP32 Firmware

Runs on ESP32 hardware with real IMU sensor and LED cube.

### Hardware Requirements

- **MCU**: ESP32-S3 (target `xtensa-esp32s3-none-elf`)
- **IMU**: BMI160 accelerometer + gyroscope (SPI); double-tap the cube to switch effect
- **LEDs**: 384× WS2812B (6 panels of 8×8)
- **LED Control**: RMT peripheral via GPIO 16

### Pin Configuration

**BMI160 IMU (SPI)**:
- MOSI: GPIO 11
- MISO: GPIO 13
- SCLK: GPIO 12
- CS: GPIO 10

**WS2812 LEDs (RMT)**:
- DATA: GPIO 16 → 74AHCT125 (5 V) → 330 Ω → IN of panel 0

**Panel chain** (data jumper from each panel's OUT to the next panel's IN; power is dropped to every panel's IN header separately):

```
0 Left → 1 Front → 2 Right → 3 Back → 4 Top → 5 Bottom
```

Each panel has IN at the bottom-left and scans row by row to OUT at the top-right. Front is the North face (`y == 0`); Left/Right/Back/Top/Bottom map to East/West/South/Up/Down. The IMU must be mounted so its axes match these faces.

### Prerequisites

1. Install ESP Rust toolchain:
```bash
# Install espup
cargo install espup

# Install ESP toolchain
espup install

# Source the environment (add to ~/.zshrc or ~/.bashrc)
. $HOME/export-esp.sh
```

2. Install espflash:
```bash
cargo install espflash
```

### Building

```bash
# Make sure the Xtensa toolchain (incl. xtensa-esp32s3-elf-gcc linker) is on PATH
. $HOME/export-esp.sh

# Navigate to firmware directory
cd firmware

# Build firmware
cargo build --release
```

### Flashing to ESP32

```bash
# Flash and monitor (automatically detects USB port)
cargo run --release

# Or specify port explicitly
espflash flash --monitor target/xtensa-esp32s3-none-elf/release/firmware
```

### Configuration

The firmware also uses `Config::default()` in `firmware/src/main.rs` (8×8×8 cube, 256 particles, 35ms frame delay, ~28 FPS). Override fields there if the ESP32 can't keep up, e.g. lower `num`.

Each effect sets its own particle count and colour in one place, `effects()` in
`core/src/effects/switcher.rs`: the fluid runs 256 particles, the boids a flock of 32 and the
starfield 24 stars. To add an effect, implement `SimulationEffect` and add a line there.

The physics tuning constants (gravity, damping, max velocity, smoothing, decay, collision repulsion/damping) are compile-time constants at the top of `core/src/effects/fluid.rs`.

### Troubleshooting

If BMI160 initialization fails:
- Check SPI wiring matches pin configuration above
- Verify 3.3V power supply to IMU
- Check SPI bus speed (1 MHz default)

If LEDs don't light:
- Verify GPIO 16 connection to LED data line
- Check 5V power supply to LED strips (384 LEDs draw significant current)
- Confirm LED count matches configuration (384 LEDs)

---

## 📦 Enclosure

The **Voxel Shell 65×65** is an 89.6 mm cube made of six mitred face shells, printed in translucent resin. Each shell holds one 65 × 66 mm WS2812 8×8 panel behind an 8×8 light grid and a diffuser skin, so every LED shows as a square pixel. A snap-in USB-C socket and an SS12F15 slide switch sit on the bottom front edge.

- [Interactive 3D viewer and STL downloads](https://psytraxx.github.io/gravity-cube-rs/voxel-shell-65x65/viewer.html)
- [Frame concepts that were compared](https://psytraxx.github.io/gravity-cube-rs/concepts.html)
- Source, parts list and assembly steps: [`case/voxel-shell-65x65/README.md`](case/voxel-shell-65x65/README.md)
- **Voxel Shell 80×80** (103.6 mm cube, flexible 80 × 80 mm 8×8 panels, same firmware): [3D viewer](https://psytraxx.github.io/gravity-cube-rs/voxel-shell-80x80/viewer.html), [`case/voxel-shell-80x80/README.md`](case/voxel-shell-80x80/README.md)

The site is published from `case/` by `.github/workflows/pages.yml` on every push to `main` that changes it.

---

## 🔬 How It Works

### Fluid Physics Simulation (Shared Core)

1. **Gravity Input**:
   - Firmware: BMI160 accelerometer reads 3-axis acceleration
   - Desktop: World gravity transformed to cube's local coordinate system based on rotation

2. **Particle System**:
   - 256 particles by default (`Config::default()`)
   - Each particle has continuous f32 position (x, y, z) and velocity (vx, vy, vz) vectors
   - Particles initialized in bottom half of cube (like water filling halfway)

3. **Physics Update (every frame)**:
   - **Velocity Update**: `v = v * smoothing + (gravity * strength)`
     - Smoothing factor: 0.9 (creates smooth acceleration)
     - Gravity strength: 0.08
   - **Velocity Clamping**: Limit to ±0.6 max velocity
   - **Predict Position**: `position += velocity`, clamped to the cube walls
   - **Particle Collision** (spatial grid, 3 solver passes): push overlapping pairs apart in proportion to their overlap
   - **Velocity from Displacement**: `v = (position - previous) * 0.95`, so particles blocked by walls or neighbours come to rest

4. **Display Update**:
   - Continuous particle positions rounded to nearest grid cell, with hysteresis so particles on a cell boundary don't flicker
   - Firmware: Map 3D grid positions to 2D LED panel indices
   - Desktop: Keep fastest particle per cell for brightness feedback, render with GPU instancing

### Panel Mapping (Firmware Only)

The physical LED cube has 6 panels chained in the order Left, Front, Right, Back, Top, Bottom, each with a different orientation on the cube:

- Pre-computed lookup table (512 entries for 8³ positions)
- Handles rotations: 0°, 90°, 180°, 270°
- Applies X/Y axis inversions per panel
- Determines which panels each 3D position touches

### 3D Rendering (Desktop Only)

- **Instanced Voxel Rendering**: Single draw call for all visible voxels
- **Particle-to-Voxel Conversion**: Continuous positions rounded to discrete grid cells
- **Instance Data**: Voxel position (x,y,z), HSV-based color with speed brightness
- **WGSL Shaders** (in `desktop/shaders/`):
  - `vertex3d.wgsl`: Positions voxel instances with model transformation
  - `fragment3d.wgsl`: Directional lighting
  - `wireframe.wgsl`: Cube outline
- **Depth Buffer**: GPU handles depth sorting automatically

---

## 🏗️ Benefits of Hexagonal Architecture

1. **Code Reuse**: Same physics simulation runs on ESP32 and macOS
2. **Testability**: Domain logic can be tested without hardware
3. **Flexibility**: Easy to add new platforms (e.g., Raspberry Pi, Web)
4. **Maintainability**: Clear separation of concerns
5. **Portability**: Domain layer is platform-agnostic (no_std compatible)

---

## 🧪 Testing

The core library includes comprehensive unit tests for physics utilities and simulation effects.

### Running Tests

```bash
# Test core library (platform-agnostic)
cargo test -p gravity-cube-core

# Test desktop simulator
cargo test -p gravity-cube-desktop
```

### Note on Floating-Point Precision

The core library uses `micromath` for no_std compatibility, which provides fast approximations rather than exact floating-point calculations. Physics tests use tolerances of 0.01-0.02 to account for approximation errors (~2-3% for square roots). This prioritizes real-time performance over mathematical precision, which is appropriate for visual simulations.

---

## 🔍 Code Quality

After every code change, run clippy and fmt to keep the code clean:

```bash
# Format core and desktop (skip firmware — requires ESP toolchain)
cargo fmt -p gravity-cube-core -p gravity-cube-desktop

# Lint with warnings as errors
cargo clippy -p gravity-cube-core -p gravity-cube-desktop -- -D warnings
```

---

## 📦 Dependencies

### Shared (Core)
- `micromath`: Fast floating-point math (no_std compatible)

### Firmware
- `esp-hal`: ESP32 hardware abstraction
- `esp-rtos`: RTOS integration for Embassy
- `embassy-executor`, `embassy-time`: Async runtime
- `esp-hal-smartled`: WS2812 LED driver (RMT)

### Desktop
- `wgpu` (v30): GPU graphics API (Metal/Vulkan/DX12)
- `winit` (v0.30): Window and input handling
- `glam` (v0.34): 3D math library
- `pollster`: Async executor for setup
- `rand`: Random number generation
- `bytemuck`: Safe GPU buffer casting
- `log` + `env_logger`: Logging

---

## 🎯 Future Enhancements

- [ ] Web version (WASM + WebGPU)
- [ ] VR support (OpenXR)
- [ ] Network sync (control firmware from desktop)
- [ ] Multiple cube sizes (4×4×4, 16×16×16)
- [ ] Different physics modes (bounce, attract, repel)
- [ ] UI overlay for real-time parameter tweaking

---

## 📝 License

Same as the original Gravity Cube project.

Original C implementation: https://github.com/Oachristensen/Gravity-Cube

---

## 🙏 Credits

- **Original Project**: [Gravity Cube by Oachristensen](https://github.com/Oachristensen/Gravity-Cube)
- **Rust Port**: Implements the same physics with type safety and cross-platform support
- **Architecture**: Hexagonal (Ports & Adapters) pattern for maximum flexibility

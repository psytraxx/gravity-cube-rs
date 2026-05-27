//! Domain types for the gravity cube simulation.
//!
//! This module contains all value objects used throughout the simulation:
//! - [`Vector3D`]: 3D floating-point vector for positions and velocities
//! - [`Particle`]: A particle with continuous position and velocity
//! - [`Position`]: Discrete grid position (u8 coordinates)
//! - [`Config`]: Simulation configuration parameters

use micromath::vector::{F32x3, Vector};

/// Type alias for 3D vector using micromath's `F32x3`.
///
/// Provides basic vector math operations like addition, subtraction,
/// scalar multiplication, and magnitude calculation.
pub type Vector3D = F32x3;

/// Extension trait for Vector3D with additional convenience methods
pub trait Vector3DExt {
    /// Create a new vector
    fn new(x: f32, y: f32, z: f32) -> Self;

    /// Normalize vector to unit length
    fn normalized(&self) -> Self;
}

impl Vector3DExt for Vector3D {
    fn new(x: f32, y: f32, z: f32) -> Self {
        F32x3 { x, y, z }
    }

    fn normalized(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0001 {
            F32x3 {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        } else {
            F32x3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }
        }
    }
}

/// Discrete 3D position in the cube grid.
///
/// Represents a voxel position using unsigned 8-bit coordinates.
/// Valid range for an 8x8x8 cube is 0-7 for each axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Position {
    /// X coordinate (0-7 for standard cube)
    pub x: u8,
    /// Y coordinate (0-7 for standard cube)
    pub y: u8,
    /// Z coordinate (0-7 for standard cube)
    pub z: u8,
}

impl Position {
    /// Create a new position with the given coordinates.
    pub const fn new(x: u8, y: u8, z: u8) -> Self {
        Self { x, y, z }
    }
}

/// A displayable pixel in the 3D LED cube.
///
/// Represents a single lit voxel at a discrete grid position.
/// Used as output from the simulation for rendering on hardware or desktop.
#[derive(Debug, Clone, Copy)]
pub struct Pixel {
    /// The grid position of this pixel.
    pub position: Position,
}

impl Pixel {
    /// Create a new pixel at the given grid coordinates.
    pub fn new(x: u8, y: u8, z: u8) -> Self {
        Self {
            position: Position::new(x, y, z),
        }
    }
}

/// A particle with continuous floating-point position and velocity.
///
/// Particles are the fundamental units of the fluid simulation. Unlike [`Pixel`],
/// which uses discrete grid coordinates, particles have continuous positions
/// that allow for smooth movement and realistic physics.
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::Particle;
///
/// let particle = Particle::new(3.5, 4.2, 1.0);
/// assert!(particle.speed() < 0.001); // Initially at rest
///
/// let grid_pos = particle.grid_position();
/// assert_eq!(grid_pos.x, 3);
/// assert_eq!(grid_pos.y, 4);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Particle {
    /// Continuous 3D position in cube space (typically 0.0 to 7.0).
    pub position: Vector3D,
    /// Velocity vector determining movement per simulation step.
    pub velocity: Vector3D,
}

impl Particle {
    /// Create a new particle at the given position with zero velocity.
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            position: Vector3D { x, y, z },
            velocity: Vector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }

    /// Get the speed (magnitude of velocity).
    pub fn speed(&self) -> f32 {
        self.velocity.magnitude()
    }

    /// Convert continuous position to discrete grid position for display.
    ///
    /// Clamps coordinates to the valid range (0-7) and truncates to integers.
    pub fn grid_position(&self) -> Position {
        Position::new(
            self.position.x.clamp(0.0, 7.0) as u8,
            self.position.y.clamp(0.0, 7.0) as u8,
            self.position.z.clamp(0.0, 7.0) as u8,
        )
    }
}

/// RGB color for LED display.
///
/// Each component is an 8-bit value (0-255).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Color {
    /// Red component (0-255).
    pub r: u8,
    /// Green component (0-255).
    pub g: u8,
    /// Blue component (0-255).
    pub b: u8,
}

impl Color {
    /// Create a new color with the given RGB values.
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Black (all LEDs off).
    pub const BLACK: Color = Color::new(0, 0, 0);
    /// Default blue color for particles.
    pub const DEFAULT: Color = Color::new(100, 180, 255);
}

/// Simulation configuration parameters.
///
/// Common configuration shared across all simulation effects.
/// Each effect can have its own physics parameters as constants.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Cube dimension (typically 8 for an 8x8x8 cube).
    pub cube_size: u8,
    /// Number of particles to simulate.
    pub num_particles: usize,
    /// Delay between simulation steps in milliseconds (for firmware timing).
    pub delay_ms: u32,
    /// Default color for particles.
    pub color: Color,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            cube_size: 8,
            num_particles: 256,
            delay_ms: 35,
            color: Color::DEFAULT,
        }
    }
}

/// Cardinal and vertical directions for LED panel faces.
///
/// Used by [`PanelConfig`] to specify which face of the cube a panel covers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Negative Y direction.
    North = 1,
    /// Positive X direction.
    East = 2,
    /// Positive Y direction.
    South = 3,
    /// Negative X direction.
    West = 4,
    /// Positive Z direction (top).
    Up = 5,
    /// Negative Z direction (bottom).
    Down = 6,
}

/// Configuration for a single LED panel on the cube face.
///
/// Each physical LED panel may be mounted with different orientations
/// and axis inversions. This struct captures the mapping from 3D cube
/// coordinates to the panel's 2D LED indices.
#[derive(Debug, Clone, Copy)]
pub struct PanelConfig {
    /// Panel index in the LED chain (0-5 for a 6-panel cube).
    pub panel_num: u8,
    /// Rotation in degrees (0, 90, 180, or 270).
    pub orientation: u16,
    /// Which face of the cube this panel covers.
    pub direction: Direction,
    /// Whether the X axis is inverted for this panel.
    pub inverted_x: bool,
    /// Whether the Y axis is inverted for this panel.
    pub inverted_y: bool,
}

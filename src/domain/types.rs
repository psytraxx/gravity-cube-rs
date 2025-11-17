use micromath::F32Ext;

/// 3D Vector with magnitude
#[derive(Debug, Clone, Copy)]
pub struct Vector3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3D {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn from_raw(x: i16, y: i16, z: i16) -> Self {
        Self::new(x as f32, y as f32, z as f32)
    }

    pub fn magnitude(&self) -> f32 {
        F32Ext::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Normalize vector to unit length
    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0001 {
            Self {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        } else {
            Self::new(0.0, 0.0, 0.0)
        }
    }

    /// Cross product with another vector
    pub fn cross(&self, other: &Vector3D) -> Vector3D {
        Vector3D::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// Swap X and Z axes (for sensor orientation)
    pub fn swap_xz(&self) -> Self {
        Self::new(self.z, self.y, self.x)
    }
}

impl core::ops::Mul<f32> for Vector3D {
    type Output = Vector3D;

    fn mul(self, scalar: f32) -> Vector3D {
        Vector3D::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

impl core::ops::Neg for Vector3D {
    type Output = Vector3D;

    fn neg(self) -> Vector3D {
        Vector3D::new(-self.x, -self.y, -self.z)
    }
}

pub trait Vector3DExt {
    fn from_raw(x: i16, y: i16, z: i16) -> Self;
    fn magnitude(&self) -> f32;
    fn swap_xz(&self) -> Self;
}

impl Vector3DExt for Vector3D {
    fn from_raw(x: i16, y: i16, z: i16) -> Self {
        Vector3D::from_raw(x, y, z)
    }

    fn magnitude(&self) -> f32 {
        self.magnitude()
    }

    fn swap_xz(&self) -> Self {
        self.swap_xz()
    }
}

/// 3D Pixel position in the cube
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Position {
    pub x: u8,
    pub y: u8,
    pub z: u8,
}

impl Position {
    pub const fn new(x: u8, y: u8, z: u8) -> Self {
        Self { x, y, z }
    }

    pub fn is_valid(&self, size: u8) -> bool {
        self.x < size && self.y < size && self.z < size
    }
}

/// A pixel in the 3D cube
#[derive(Debug, Clone, Copy)]
pub struct Pixel {
    pub position: Position,
    pub active: bool,
}

impl Pixel {
    pub fn new(x: u8, y: u8, z: u8) -> Self {
        Self {
            position: Position::new(x, y, z),
            active: false,
        }
    }

    pub fn activate(&mut self) {
        self.active = true;
    }

    pub fn deactivate(&mut self) {
        self.active = false;
    }
}

/// RGB Color
#[derive(Debug, Clone, Copy)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const BLACK: Color = Color::new(0, 0, 0);
    pub const DEFAULT: Color = Color::new(10, 10, 100);
}

/// Movement parameters for physics simulation
#[derive(Debug, Clone, Copy)]
pub struct MoveParams {
    pub down: Vector3D,
    pub right: Vector3D,
    pub left: Vector3D,
}

impl MoveParams {
    pub fn from_gravity(gravity: &Vector3D, velocity: f32) -> Self {
        let gravity_unit = gravity.normalize();

        // Choose reference vector (avoid parallel to gravity)
        let ref_vec = if gravity_unit.z.abs() > 0.99 {
            Vector3D::new(0.0, 1.0, 0.0)
        } else {
            Vector3D::new(0.0, 0.0, 1.0)
        };

        // Calculate right direction via cross product
        let right_unnorm = gravity_unit.cross(&ref_vec);
        let mag = right_unnorm.magnitude();

        let right = if mag > 0.01 {
            right_unnorm.normalize()
        } else {
            Vector3D::new(1.0, 0.0, 0.0)
        };

        // Left is opposite of right
        let left = -right;

        Self {
            down: gravity_unit * velocity,
            right: right * velocity,
            left: left * velocity,
        }
    }
}

/// Configuration constants
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub cube_size: u8,
    pub num_particles: usize,
    pub velocity: f32,
    pub delay_ms: u32,
    pub color: Color,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            cube_size: 8,
            num_particles: 200,
            velocity: 2.0,
            delay_ms: 35,
            color: Color::DEFAULT,
        }
    }
}

/// Panel directions on the cube
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    North = 1,
    East = 2,
    South = 3,
    West = 4,
    Up = 5,
    Down = 6,
}

/// Panel configuration
#[derive(Debug, Clone, Copy)]
pub struct PanelConfig {
    pub panel_num: u8,
    pub orientation: u16, // 0, 90, 180, 270
    pub direction: Direction,
    pub inverted_x: bool,
    pub inverted_y: bool,
}

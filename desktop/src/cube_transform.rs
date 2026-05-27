use glam::{Mat4, Quat, Vec3};

/// Represents the cube's rotation in 3D space
pub struct CubeTransform {
    pub rotation_x: f32, // Pitch
    pub rotation_y: f32, // Yaw
    pub position: Vec3,
}

impl CubeTransform {
    pub fn new() -> Self {
        Self {
            rotation_x: 0.0,
            rotation_y: 0.0,
            position: Vec3::new(3.5, 3.5, 3.5), // Center of 8x8x8 cube
        }
    }

    pub fn rotate(&mut self, delta_x: f32, delta_y: f32) {
        self.rotation_y += delta_x;
        self.rotation_x += delta_y;

        // No clamping - allow full 360° rotation in all directions
    }

    /// Get the model matrix for rendering
    pub fn model_matrix(&self) -> Mat4 {
        // Translate to origin, rotate, translate back
        let translate_to_origin = Mat4::from_translation(-self.position);
        let rotate_y = Mat4::from_rotation_y(self.rotation_y);
        let rotate_x = Mat4::from_rotation_x(self.rotation_x);
        let translate_back = Mat4::from_translation(self.position);

        translate_back * rotate_x * rotate_y * translate_to_origin
    }

    /// Get gravity in cube's local space
    /// World gravity is always (0, -1, 0), but we transform it to cube's local coordinates
    pub fn get_local_gravity(&self) -> Vec3 {
        // World gravity (always pointing down)
        let world_gravity = Vec3::new(0.0, -1.0, 0.0);

        // Create inverse rotation (from world to local)
        let rotation =
            Quat::from_rotation_y(-self.rotation_y) * Quat::from_rotation_x(-self.rotation_x);

        // Transform gravity to local space
        rotation * world_gravity
    }
}

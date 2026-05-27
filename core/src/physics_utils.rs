//! Shared physics utilities for simulation effects.
//!
//! This module provides common physics operations used across multiple simulation
//! implementations, reducing code duplication and improving maintainability.

use crate::types::Vector3D;
use micromath::vector::Vector;

/// Apply boundary collision for a single axis.
///
/// When a particle hits a boundary (0 or bound), this function:
/// 1. Clamps the position to valid range [0, bound]
/// 2. Reflects the velocity with damping (energy loss)
///
/// # Arguments
///
/// * `pos` - Mutable reference to position component (x, y, or z)
/// * `vel` - Mutable reference to velocity component
/// * `bound` - Maximum valid position (typically cube_size - 1.0)
/// * `damping` - Energy retention on bounce (0.0 = no bounce, 1.0 = perfect reflection)
///
/// # Example
///
/// ```
/// use gravity_cube_core::physics_utils::apply_axis_boundary;
///
/// let mut x = -0.5;  // Past boundary
/// let mut vx = -0.2; // Moving further out
/// apply_axis_boundary(&mut x, &mut vx, 7.0, 0.9);
///
/// assert_eq!(x, 0.0);           // Clamped to boundary
/// assert_eq!(vx, 0.2 * 0.9);    // Velocity reflected and damped
/// ```
#[inline]
pub fn apply_axis_boundary(pos: &mut f32, vel: &mut f32, bound: f32, damping: f32) {
    if *pos < 0.0 {
        *pos = 0.0;
        *vel = vel.abs() * damping;
    } else if *pos >= bound {
        *pos = bound;
        *vel = -vel.abs() * damping;
    }
}

/// Apply boundary collision for all three axes.
///
/// Convenience function that applies boundary collision to all components
/// of a 3D position and velocity vector.
///
/// # Arguments
///
/// * `position` - Mutable reference to 3D position vector
/// * `velocity` - Mutable reference to 3D velocity vector
/// * `bound` - Maximum valid position for all axes
/// * `damping` - Energy retention on bounce
///
/// # Example
///
/// ```
/// use gravity_cube_core::{Vector3D, Vector3DExt};
/// use gravity_cube_core::physics_utils::apply_boundary_3d;
///
/// let mut pos = Vector3D::new(8.0, 3.0, -1.0);  // X and Z out of bounds
/// let mut vel = Vector3D::new(0.5, 0.1, -0.3);
/// apply_boundary_3d(&mut pos, &mut vel, 7.0, 0.9);
///
/// assert_eq!(pos.x, 7.0);  // Clamped
/// assert_eq!(pos.z, 0.0);  // Clamped
/// ```
#[inline]
pub fn apply_boundary_3d(position: &mut Vector3D, velocity: &mut Vector3D, bound: f32, damping: f32) {
    apply_axis_boundary(&mut position.x, &mut velocity.x, bound, damping);
    apply_axis_boundary(&mut position.y, &mut velocity.y, bound, damping);
    apply_axis_boundary(&mut position.z, &mut velocity.z, bound, damping);
}

/// Clamp velocity magnitude to a maximum speed.
///
/// If the velocity vector's magnitude exceeds max_speed, scales it down
/// while preserving direction. This prevents particles from moving too fast.
///
/// # Arguments
///
/// * `velocity` - Mutable reference to velocity vector
/// * `max_speed` - Maximum allowed speed (magnitude)
///
/// # Example
///
/// ```
/// use gravity_cube_core::{Vector3D, Vector3DExt};
/// use gravity_cube_core::physics_utils::clamp_speed;
///
/// let mut vel = Vector3D::new(1.0, 1.0, 1.0);  // Magnitude ~1.73
/// clamp_speed(&mut vel, 1.0);
///
/// assert!(vel.x.abs() < 1.0);  // Each component reduced
/// let speed = (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).sqrt();
/// assert!((speed - 1.0).abs() < 0.02);  // Total speed is ~1.0 (with tolerance for fast math)
/// ```
#[inline]
pub fn clamp_speed(velocity: &mut Vector3D, max_speed: f32) {
    let speed = velocity.magnitude();
    if speed > max_speed {
        let scale = max_speed / speed;
        *velocity *= scale;
    }
}

/// Calculate distance squared between two 3D points.
///
/// Returns the squared Euclidean distance, which is cheaper to compute
/// than the actual distance (avoids sqrt). Useful for proximity checks
/// where you can compare against `threshold * threshold`.
///
/// # Example
///
/// ```
/// use gravity_cube_core::{Vector3D, Vector3DExt};
/// use gravity_cube_core::physics_utils::distance_squared;
///
/// let p1 = Vector3D::new(0.0, 0.0, 0.0);
/// let p2 = Vector3D::new(3.0, 4.0, 0.0);
/// let dist_sq = distance_squared(&p1, &p2);
///
/// assert_eq!(dist_sq, 25.0);  // 3^2 + 4^2 = 25
/// ```
#[inline]
pub fn distance_squared(p1: &Vector3D, p2: &Vector3D) -> f32 {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let dz = p2.z - p1.z;
    dx * dx + dy * dy + dz * dz
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Vector3DExt;

    #[test]
    fn test_axis_boundary_lower() {
        let mut pos = -0.5;
        let mut vel = -0.2;
        apply_axis_boundary(&mut pos, &mut vel, 7.0, 0.9);

        assert_eq!(pos, 0.0);
        assert!((vel - 0.18).abs() < 0.001); // 0.2 * 0.9
    }

    #[test]
    fn test_axis_boundary_upper() {
        let mut pos = 8.0;
        let mut vel = 0.2;
        apply_axis_boundary(&mut pos, &mut vel, 7.0, 0.9);

        assert_eq!(pos, 7.0);
        assert!((vel + 0.18).abs() < 0.001); // -0.2 * 0.9
    }

    #[test]
    fn test_axis_boundary_no_collision() {
        let mut pos = 3.5;
        let mut vel = 0.1;
        apply_axis_boundary(&mut pos, &mut vel, 7.0, 0.9);

        assert_eq!(pos, 3.5); // Unchanged
        assert_eq!(vel, 0.1); // Unchanged
    }

    #[test]
    fn test_boundary_3d() {
        let mut pos = Vector3D::new(8.0, 3.0, -1.0);
        let mut vel = Vector3D::new(0.5, 0.1, -0.3);
        apply_boundary_3d(&mut pos, &mut vel, 7.0, 0.9);

        assert_eq!(pos.x, 7.0);
        assert_eq!(pos.y, 3.0);
        assert_eq!(pos.z, 0.0);
    }

    #[test]
    fn test_clamp_speed_over_limit() {
        let mut vel = Vector3D::new(1.0, 1.0, 1.0);
        clamp_speed(&mut vel, 1.0);

        let speed = vel.magnitude();
        // Use larger tolerance for micromath's fast sqrt approximation
        // (micromath returns ~1.75 for sqrt(3), resulting in ~0.99 after clamping)
        assert!((speed - 1.0).abs() < 0.02);
    }

    #[test]
    fn test_clamp_speed_under_limit() {
        let mut vel = Vector3D::new(0.1, 0.1, 0.1);
        let original = vel;
        clamp_speed(&mut vel, 1.0);

        assert_eq!(vel.x, original.x);
        assert_eq!(vel.y, original.y);
        assert_eq!(vel.z, original.z);
    }

    #[test]
    fn test_distance_squared() {
        let p1 = Vector3D::new(0.0, 0.0, 0.0);
        let p2 = Vector3D::new(3.0, 4.0, 0.0);
        let dist_sq = distance_squared(&p1, &p2);

        assert_eq!(dist_sq, 25.0);
    }
}

//! Core trait definitions for simulation effects.
//!
//! This module defines the common interface that all simulation effects must implement
//! to be compatible with both firmware and desktop platforms.

use super::types::*;

/// Common interface for all simulation effects.
///
/// This trait defines the core operations that all effects must implement
/// to be compatible with both firmware and desktop platforms. All effects
/// use the same [`Config`] struct for common parameters (cube size, particle
/// count, etc.), but may have effect-specific physics constants.
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, SimulationEffect, Vector3D, Vector3DExt};
///
/// # struct MyEffect { particles: Vec<gravity_cube_core::Particle>, config: Config }
/// # impl SimulationEffect for MyEffect {
/// #     fn new(config: Config) -> Self {
/// #         MyEffect { particles: Vec::new(), config }
/// #     }
/// #     fn populate(&mut self, _: usize) {}
/// #     fn step(&mut self, _: &Vector3D) {}
/// #     fn particles(&self) -> &[gravity_cube_core::Particle] { &self.particles }
/// #     fn active_pixels(&self) -> impl Iterator<Item = gravity_cube_core::Pixel> + '_ {
/// #         self.particles.iter().map(|p| {
/// #             let pos = p.grid_position();
/// #             gravity_cube_core::Pixel::new(pos.x, pos.y, pos.z)
/// #         })
/// #     }
/// #     fn config(&self) -> &Config { &self.config }
/// # }
/// // Create and run simulation
/// let config = Config::default();
/// let mut sim = MyEffect::new(config);
/// sim.populate(64);
///
/// let gravity = Vector3D::new(0.0, 1.0, 0.0);
/// sim.step(&gravity);
/// ```
pub trait SimulationEffect {
    /// Create a new simulation instance with the given configuration.
    ///
    /// The simulation starts empty. Call [`populate`](Self::populate) to add particles.
    fn new(config: Config) -> Self;

    /// Initialize particles or other state for the simulation.
    ///
    /// The number and distribution of particles is effect-specific.
    /// For example, fluid simulation fills the bottom half of the cube,
    /// while boids might be distributed randomly throughout.
    ///
    /// # Arguments
    ///
    /// * `num_particles` - Target number of particles to create
    fn populate(&mut self, num_particles: usize);

    /// Advance the simulation by one time step.
    ///
    /// Applies the effect's physics algorithm to update particle positions
    /// and velocities. The acceleration parameter is typically from an IMU
    /// (for firmware) or transformed gravity (for desktop).
    ///
    /// # Arguments
    ///
    /// * `acceleration` - Generic acceleration vector. Interpretation is
    ///   effect-specific (gravity for fluid, wind for boids, etc.).
    fn step(&mut self, acceleration: &Vector3D);

    /// Get access to particles for rendering.
    ///
    /// Returns a slice of all particles in the simulation.
    /// Used by desktop renderer for GPU visualization.
    fn particles(&self) -> &[Particle];

    /// Get an iterator over active pixels for LED display.
    ///
    /// Converts continuous particle positions to discrete grid positions.
    /// Used by firmware for WS2812 LED control.
    fn active_pixels(&self) -> impl Iterator<Item = Pixel> + '_;

    /// Get the effect's configuration.
    fn config(&self) -> &Config;
}

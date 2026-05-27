//! # Gravity Cube Core
//!
//! Platform-agnostic physics simulation library for gravity-responsive particles.
//!
//! This crate provides the core fluid physics simulation that can run on both
//! `std` environments (desktop) and `no_std` environments (embedded ESP32).
//!
//! ## Features
//!
//! - **Continuous particle physics**: Particles have floating-point positions and velocities
//! - **Spatial grid collision detection**: O(n) average-case collision detection using spatial hashing
//! - **Configurable physics**: Gravity, damping, velocity limits, and collision parameters
//! - **`no_std` compatible**: Works on embedded systems with `alloc`
//!
//! ## Example
//!
//! ```rust
//! use gravity_cube_core::{Config, Simulation, SimulationEffect, Vector3D, Vector3DExt};
//!
//! let config = Config::default();
//! let mut sim = Simulation::new(config);
//! sim.populate(64);
//!
//! // Apply gravity pointing down
//! let gravity = Vector3D::new(0.0, 1.0, 0.0);
//! sim.step(&gravity);
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod effect;
pub mod effects;
pub mod physics_utils;
pub mod types;

// Always export traits for extensibility
pub use effect::SimulationEffect;

// Re-export commonly used types
pub use types::{
    Color, Config, Direction, PanelConfig, Particle, Pixel, Position, Vector3D, Vector3DExt,
};

// Re-export simulation implementations for convenience
pub use effects::boids::BoidsSimulation;
pub use effects::fluid::FluidSimulation;

/// Alias for backwards compatibility - defaults to FluidSimulation.
pub type Simulation = FluidSimulation;

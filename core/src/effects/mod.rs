//! Simulation effect implementations.
//!
//! This module contains different physics simulations that implement
//! the [`SimulationEffect`](crate::effect::SimulationEffect) trait.

/// Fluid physics simulation with gravity response and particle collisions.
pub mod fluid;

/// Boids flocking simulation with separation, alignment, and cohesion.
pub mod boids;

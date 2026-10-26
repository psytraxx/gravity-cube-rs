//! Simulation effect implementations.
//!
//! This module contains different physics simulations that implement
//! the [`SimulationEffect`](crate::effect::SimulationEffect) trait.

/// Fluid physics simulation with gravity response and particle collisions.
pub mod fluid;

/// Boids flocking simulation with separation, alignment, and cohesion.
pub mod boids;

/// Starfield streaming out of the centre, fixed in the world as the cube turns.
pub mod starfield;

/// Switches between the effects at runtime.
pub mod switcher;

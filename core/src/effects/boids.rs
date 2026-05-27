//! Boids flocking simulation.
//!
//! Implements Craig Reynolds' Boids algorithm for simulating flocking behavior.
//! Each boid follows three simple rules:
//! 1. Separation: Avoid crowding neighbors
//! 2. Alignment: Steer towards average heading of neighbors
//! 3. Cohesion: Steer towards average position of neighbors

use crate::effect::SimulationEffect;
use crate::physics_utils;
use crate::types::*;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use micromath::F32Ext;

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

use log::info;
use micromath::vector::Vector;

// Boids physics constants
/// Minimum distance boids try to maintain from each other.
const SEPARATION_DISTANCE: f32 = 1.5;

/// Weight for velocity alignment with neighbors.
const ALIGNMENT_WEIGHT: f32 = 0.3;

/// Weight for moving toward flock center.
const COHESION_WEIGHT: f32 = 0.1;

/// Weight for avoiding collisions.
const SEPARATION_WEIGHT: f32 = 0.5;

/// Maximum boid velocity.
const MAX_SPEED: f32 = 0.4;

/// How far boids can "see" neighbors.
const PERCEPTION_RADIUS: f32 = 3.0;

/// Boundary repulsion strength.
const BOUNDARY_FORCE: f32 = 0.3;

/// Margin distance from walls where boundary repulsion begins.
const BOUNDARY_MARGIN: f32 = 1.5;

/// Wind/acceleration responsiveness scale.
///
/// Controls how much external acceleration (e.g., from tilting the cube)
/// affects boid movement. Lower values maintain more autonomous flocking behavior.
const WIND_SCALE: f32 = 0.05;

/// Velocity damping on boundary collision (energy loss when hitting walls).
const BOUNDARY_COLLISION_DAMPING: f32 = 0.5;

/// Speed limiter tolerance factor.
///
/// Slightly below 1.0 (0.9999) to prevent floating-point precision issues
/// where speed might oscillate at exactly MAX_SPEED.
const SPEED_LIMIT_TOLERANCE: f32 = 0.9999;

/// Boids flocking simulation.
///
/// Simulates autonomous agents (boids) that exhibit emergent flocking behavior
/// through simple local rules. The result is realistic bird-like or fish-like
/// swarm motion.
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, SimulationEffect, Vector3D, Vector3DExt};
/// use gravity_cube_core::effects::boids::BoidsSimulation;
///
/// let config = Config::default();
/// let mut sim = BoidsSimulation::new(config);
/// sim.populate(64);
///
/// // Apply wind force
/// let wind = Vector3D::new(0.1, 0.0, 0.0);
/// sim.step(&wind);
///
/// // Get boid positions for rendering
/// for boid in sim.particles() {
///     println!("Boid at ({:.2}, {:.2}, {:.2})",
///         boid.position.x, boid.position.y, boid.position.z);
/// }
/// ```
pub struct BoidsSimulation {
    particles: Vec<Particle>,
    config: Config,
}

impl SimulationEffect for BoidsSimulation {
    fn new(config: Config) -> Self {
        Self {
            particles: Vec::new(),
            config,
        }
    }

    fn populate(&mut self, num_particles: usize) {
        self.particles.clear();

        // Distribute boids throughout the cube
        // Simple approach: distribute evenly with some variation
        let cube_size = self.config.cube_size as f32;

        for i in 0..num_particles {
            // Use index to create pseudo-random but deterministic positions
            let fi = i as f32;
            let x = ((fi * 7.131) % cube_size).clamp(0.5, cube_size - 0.5);
            let y = ((fi * 11.243) % cube_size).clamp(0.5, cube_size - 0.5);
            let z = ((fi * 13.579) % cube_size).clamp(0.5, cube_size - 0.5);

            let mut particle = Particle::new(x, y, z);

            // Give initial random-ish velocity based on position
            let vx = ((x * 7.0) % 1.0 - 0.5) * 0.2;
            let vy = ((y * 11.0) % 1.0 - 0.5) * 0.2;
            let vz = ((z * 13.0) % 1.0 - 0.5) * 0.2;
            particle.velocity = Vector3D::new(vx, vy, vz);
            self.particles.push(particle);
        }
    }

    fn step(&mut self, acceleration: &Vector3D) {
        let perception = PERCEPTION_RADIUS;
        let separation_dist = SEPARATION_DISTANCE;
        let max_speed = MAX_SPEED;
        let bound = (self.config.cube_size - 1) as f32;

        self.log_step_info(acceleration);

        // Calculate steering forces for each boid
        let mut forces: Vec<Vector3D> = Vec::new();
        forces.resize(self.particles.len(), Vector3D::new(0.0, 0.0, 0.0));

        for (i, _) in self.particles.iter().enumerate() {
            let mut separation = Vector3D::new(0.0, 0.0, 0.0);
            let mut alignment = Vector3D::new(0.0, 0.0, 0.0);
            let mut cohesion = Vector3D::new(0.0, 0.0, 0.0);
            let mut neighbor_count = 0;

            // Check all other boids for flocking behavior
            for j in 0..self.particles.len() {
                if i == j {
                    continue;
                }

                let dx = self.particles[j].position.x - self.particles[i].position.x;
                let dy = self.particles[j].position.y - self.particles[i].position.y;
                let dz = self.particles[j].position.z - self.particles[i].position.z;
                let dist_sq = dx * dx + dy * dy + dz * dz;

                if dist_sq < perception * perception && dist_sq > 0.0001 {
                    let dist = dist_sq.sqrt();
                    neighbor_count += 1;

                    // Separation: move away from close neighbors
                    if dist < separation_dist {
                        let repel_strength = (separation_dist - dist) / separation_dist;
                        separation.x -= (dx / dist) * repel_strength;
                        separation.y -= (dy / dist) * repel_strength;
                        separation.z -= (dz / dist) * repel_strength;
                    }

                    // Alignment: average velocity
                    alignment.x += self.particles[j].velocity.x;
                    alignment.y += self.particles[j].velocity.y;
                    alignment.z += self.particles[j].velocity.z;

                    // Cohesion: steer toward average position
                    cohesion.x += dx;
                    cohesion.y += dy;
                    cohesion.z += dz;
                }
            }

            // Average and apply weights
            if neighbor_count > 0 {
                let n = neighbor_count as f32;
                alignment *= ALIGNMENT_WEIGHT / n;
                cohesion *= COHESION_WEIGHT / n;
            }
            separation *= SEPARATION_WEIGHT;

            // Boundary avoidance - repel from walls
            let mut boundary = Vector3D::new(0.0, 0.0, 0.0);
            let pos = &self.particles[i].position;

            if pos.x < BOUNDARY_MARGIN {
                boundary.x += (BOUNDARY_MARGIN - pos.x) * BOUNDARY_FORCE;
            }
            if pos.x > bound - BOUNDARY_MARGIN {
                boundary.x -= (pos.x - (bound - BOUNDARY_MARGIN)) * BOUNDARY_FORCE;
            }
            if pos.y < BOUNDARY_MARGIN {
                boundary.y += (BOUNDARY_MARGIN - pos.y) * BOUNDARY_FORCE;
            }
            if pos.y > bound - BOUNDARY_MARGIN {
                boundary.y -= (pos.y - (bound - BOUNDARY_MARGIN)) * BOUNDARY_FORCE;
            }
            if pos.z < BOUNDARY_MARGIN {
                boundary.z += (BOUNDARY_MARGIN - pos.z) * BOUNDARY_FORCE;
            }
            if pos.z > bound - BOUNDARY_MARGIN {
                boundary.z -= (pos.z - (bound - BOUNDARY_MARGIN)) * BOUNDARY_FORCE;
            }

            // Add wind effect from acceleration
            let wind = *acceleration * WIND_SCALE;

            forces[i] = separation + alignment + cohesion + boundary + wind;
        }

        // Apply forces and update positions
        for (i, particle) in self.particles.iter_mut().enumerate() {
            // Update velocity
            particle.velocity.x += forces[i].x;
            particle.velocity.y += forces[i].y;
            particle.velocity.z += forces[i].z;

            // Clamp speed to prevent excessive velocity
            physics_utils::clamp_speed(&mut particle.velocity, max_speed * SPEED_LIMIT_TOLERANCE);

            // Update position
            particle.position.x += particle.velocity.x;
            particle.position.y += particle.velocity.y;
            particle.position.z += particle.velocity.z;

            // Hard boundary constraints with bounce
            physics_utils::apply_boundary_3d(
                &mut particle.position,
                &mut particle.velocity,
                bound,
                BOUNDARY_COLLISION_DAMPING,
            );
        }
    }

    fn particles(&self) -> &[Particle] {
        &self.particles
    }

    fn active_pixels(&self) -> impl Iterator<Item = Pixel> + '_ {
        self.particles.iter().map(|p| {
            let grid_pos = p.grid_position();
            Pixel::new(grid_pos.x, grid_pos.y, grid_pos.z)
        })
    }

    fn config(&self) -> &Config {
        &self.config
    }
}

impl BoidsSimulation {
    fn log_step_info(&self, acceleration: &Vector3D) {
        let magnitude = acceleration.magnitude();
        info!("=== Boids Step ===");
        info!(
            "Wind: ({:.2}, {:.2}, {:.2}) |{:.2}|",
            acceleration.x, acceleration.y, acceleration.z, magnitude
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect::SimulationEffect;

    fn test_config() -> crate::types::Config {
        crate::types::Config {
            cube_size: 8,
            num_particles: 10,
            delay_ms: 35,
            color: Color::new(255, 200, 100),
        }
    }

    #[test]
    fn test_boids_creation() {
        let config = test_config();
        let sim = BoidsSimulation::new(config);
        assert!(sim.particles.is_empty());
        assert_eq!(sim.config().cube_size, 8);
    }

    #[test]
    fn test_populate_boids() {
        let config = test_config();
        let mut sim = BoidsSimulation::new(config);
        sim.populate(10);
        assert_eq!(sim.particles().len(), 10);

        // Boids should be distributed throughout cube
        for boid in sim.particles() {
            assert!(boid.position.x >= 0.0 && boid.position.x < 8.0);
            assert!(boid.position.y >= 0.0 && boid.position.y < 8.0);
            assert!(boid.position.z >= 0.0 && boid.position.z < 8.0);
        }
    }

    #[test]
    fn test_boids_have_initial_velocity() {
        let config = test_config();
        let mut sim = BoidsSimulation::new(config);
        sim.populate(10);

        // At least some boids should have non-zero initial velocity
        let has_velocity = sim.particles().iter().any(|p| p.speed() > 0.01);
        assert!(has_velocity, "Boids should have initial velocities");
    }

    #[test]
    fn test_boids_move() {
        let config = test_config();
        let mut sim = BoidsSimulation::new(config);
        sim.populate(5);

        let initial_positions: Vec<_> = sim
            .particles()
            .iter()
            .map(|p| (p.position.x, p.position.y, p.position.z))
            .collect();

        // Run simulation
        let wind = Vector3D::new(0.0, 0.0, 0.0);
        for _ in 0..10 {
            sim.step(&wind);
        }

        // At least some boids should have moved
        let moved = sim.particles().iter().enumerate().any(|(i, p)| {
            let (ix, iy, iz) = initial_positions[i];
            (p.position.x - ix).abs() > 0.1
                || (p.position.y - iy).abs() > 0.1
                || (p.position.z - iz).abs() > 0.1
        });
        assert!(moved, "Boids should move over time");
    }

    #[test]
    fn test_boids_stay_in_bounds() {
        let config = test_config();
        let mut sim = BoidsSimulation::new(config);
        sim.populate(10);

        let wind = Vector3D::new(1.0, 1.0, 1.0);
        for _ in 0..100 {
            sim.step(&wind);
        }

        let bound = config.cube_size as f32 - 1.0;
        for boid in sim.particles() {
            assert!(
                boid.position.x >= 0.0 && boid.position.x <= bound,
                "Boid X out of bounds: {}",
                boid.position.x
            );
            assert!(
                boid.position.y >= 0.0 && boid.position.y <= bound,
                "Boid Y out of bounds: {}",
                boid.position.y
            );
            assert!(
                boid.position.z >= 0.0 && boid.position.z <= bound,
                "Boid Z out of bounds: {}",
                boid.position.z
            );
        }
    }

    #[test]
    fn test_boids_speed_limit() {
        let config = test_config();
        let mut sim = BoidsSimulation::new(config);
        sim.populate(5);

        let strong_wind = Vector3D::new(2.0, 2.0, 2.0);
        for _ in 0..20 {
            sim.step(&strong_wind);
        }

        // All boids should respect max speed (with small tolerance for floating point precision)
        for boid in sim.particles() {
            assert!(
                boid.speed() <= MAX_SPEED * 1.05,
                "Boid speed {} exceeds max {} by too much",
                boid.speed(),
                MAX_SPEED
            );
        }
    }

    #[test]
    fn test_active_pixels_count() {
        let config = test_config();
        let mut sim = BoidsSimulation::new(config);
        sim.populate(10);

        let pixels: Vec<_> = sim.active_pixels().collect();
        assert_eq!(pixels.len(), 10);
    }
}

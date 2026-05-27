//! Fluid physics simulation engine.
//!
//! This module contains the fluid simulation logic for gravity-responsive particles.
//! It uses spatial grid hashing for efficient O(n) average-case collision detection.

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

/// Collision detection threshold - particles closer than this distance will repel.
const COLLISION_THRESHOLD: f32 = 1.0;

/// Minimum distance squared to avoid division by zero in collision calculations.
const MIN_DISTANCE_SQUARED: f32 = 0.0001;

/// Grid cell size for spatial hashing (should be >= COLLISION_THRESHOLD).
const GRID_CELL_SIZE: f32 = 1.0;

// Fluid physics constants
/// Gravity acceleration multiplier.
const GRAVITY: f32 = 0.08;

/// Acceleration responsiveness scale factor.
///
/// This factor balances responsiveness to acceleration (e.g., from IMU) with stability.
/// Lower values (0.1-0.2) make fluid more stable but less responsive to tilt/shake.
/// Higher values (0.4-0.5) make it more dynamic but can become unstable.
const ACCELERATION_SCALE: f32 = 0.3;

/// Velocity retention on boundary collision (0.0-1.0).
const DAMPING: f32 = 0.92;

/// Maximum velocity per axis.
const MAX_VELOCITY: f32 = 0.6;

/// Velocity smoothing/inertia factor (0.0-1.0).
const VELOCITY_SMOOTHING: f32 = 0.9;

/// Velocity decay per step - friction (0.0-1.0).
const VELOCITY_DECAY: f32 = 0.95;

/// Particle collision repulsion strength.
const COLLISION_REPULSION: f32 = 0.5;

/// Energy loss during particle collisions.
const COLLISION_DAMPING: f32 = 0.3;

/// Neighbor offsets for spatial grid lookups (27 neighbors including self).
const NEIGHBOR_OFFSETS: [(i32, i32, i32); 27] = [
    (-1, -1, -1),
    (0, -1, -1),
    (1, -1, -1),
    (-1, 0, -1),
    (0, 0, -1),
    (1, 0, -1),
    (-1, 1, -1),
    (0, 1, -1),
    (1, 1, -1),
    (-1, -1, 0),
    (0, -1, 0),
    (1, -1, 0),
    (-1, 0, 0),
    (0, 0, 0),
    (1, 0, 0),
    (-1, 1, 0),
    (0, 1, 0),
    (1, 1, 0),
    (-1, -1, 1),
    (0, -1, 1),
    (1, -1, 1),
    (-1, 0, 1),
    (0, 0, 1),
    (1, 0, 1),
    (-1, 1, 1),
    (0, 1, 1),
    (1, 1, 1),
];

/// Spatial grid for O(n) average-case collision detection.
///
/// Divides the cube into cells and only checks collisions between particles
/// in the same or neighboring cells. For an 8x8x8 cube with cell size 1.0,
/// this creates a 9x9x9 grid (extra cell for boundary).
struct SpatialGrid {
    /// Grid dimensions (cube_size + 1 to handle edge cases).
    size: usize,
    /// Index of the first particle in each cell.
    head: Vec<u16>,
    /// Index of the next particle in the same cell.
    next: Vec<u16>,
}

impl SpatialGrid {
    /// Create a new spatial grid for the given cube size.
    fn new(cube_size: u8) -> Self {
        let size = (cube_size as usize) + 1;
        let total_cells = size * size * size;
        let mut head = Vec::with_capacity(total_cells);
        head.resize(total_cells, u16::MAX);
        Self {
            size,
            head,
            next: Vec::new(),
        }
    }

    /// Clear all cells for a new frame.
    fn clear(&mut self, particle_count: usize) {
        self.head.fill(u16::MAX);
        if self.next.len() < particle_count {
            self.next.resize(particle_count, u16::MAX);
        }
    }

    /// Get the cell index for a position.
    #[inline]
    fn cell_index(&self, x: f32, y: f32, z: f32) -> usize {
        let cx = (x / GRID_CELL_SIZE).clamp(0.0, (self.size - 1) as f32) as usize;
        let cy = (y / GRID_CELL_SIZE).clamp(0.0, (self.size - 1) as f32) as usize;
        let cz = (z / GRID_CELL_SIZE).clamp(0.0, (self.size - 1) as f32) as usize;
        cx + cy * self.size + cz * self.size * self.size
    }

    /// Insert a particle into the grid.
    fn insert(&mut self, particle_index: u16, pos: &Vector3D) {
        let idx = self.cell_index(pos.x, pos.y, pos.z);
        self.next[particle_index as usize] = self.head[idx];
        self.head[idx] = particle_index;
    }

    /// Iterate over all neighboring cells (including the cell itself).
    /// Returns an iterator of cell indices to check for collisions.
    fn neighbor_cells(&self, x: f32, y: f32, z: f32) -> impl Iterator<Item = usize> + '_ {
        let cx = (x / GRID_CELL_SIZE).clamp(0.0, (self.size - 1) as f32) as i32;
        let cy = (y / GRID_CELL_SIZE).clamp(0.0, (self.size - 1) as f32) as i32;
        let cz = (z / GRID_CELL_SIZE).clamp(0.0, (self.size - 1) as f32) as i32;
        let size = self.size as i32;

        // Generate all 27 neighbor offsets (-1, 0, 1) for each axis
        NEIGHBOR_OFFSETS.iter().filter_map(move |&(dx, dy, dz)| {
            let nx = cx + dx;
            let ny = cy + dy;
            let nz = cz + dz;
            if nx >= 0 && nx < size && ny >= 0 && ny < size && nz >= 0 && nz < size {
                Some(
                    (nx as usize)
                        + (ny as usize) * (size as usize)
                        + (nz as usize) * (size as usize) * (size as usize),
                )
            } else {
                None
            }
        })
    }
}

/// Fluid physics simulation implementation.
///
/// Simulates gravity-responsive particles with continuous positions,
/// boundary collisions, and particle-particle interactions.
///
/// # Performance
///
/// Uses spatial grid hashing for collision detection, achieving O(n) average-case
/// complexity instead of O(n²) brute force. With 256 particles in an 8x8x8 cube,
/// this typically reduces collision checks from ~32,000 to ~2,000 per frame.
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, SimulationEffect, Vector3D, Vector3DExt};
/// use gravity_cube_core::effects::fluid::FluidSimulation;
///
/// let config = Config::default();
/// let mut sim = FluidSimulation::new(config);
/// sim.populate(64);
///
/// // Simulate with gravity pointing down
/// let gravity = Vector3D::new(0.0, 1.0, 0.0);
/// sim.step(&gravity);
///
/// // Get particle positions for rendering
/// for particle in sim.particles() {
///     println!("Particle at ({:.2}, {:.2}, {:.2})",
///         particle.position.x, particle.position.y, particle.position.z);
/// }
/// ```
pub struct FluidSimulation {
    /// All particles in the simulation.
    particles: Vec<Particle>,
    config: Config,
    /// Spatial grid for efficient collision detection.
    grid: SpatialGrid,
}

impl SimulationEffect for FluidSimulation {
    fn new(config: Config) -> Self {
        let grid = SpatialGrid::new(config.cube_size);
        Self {
            particles: Vec::new(),
            config,
            grid,
        }
    }

    fn populate(&mut self, num_particles: usize) {
        self.particles.clear();
        let mut index = 0;

        let fill_height = self.config.cube_size / 2;
        for y in (self.config.cube_size - fill_height)..self.config.cube_size {
            for z in 0..self.config.cube_size {
                for x in 0..self.config.cube_size {
                    if index >= num_particles {
                        break;
                    }
                    self.particles
                        .push(Particle::new(x as f32, y as f32, z as f32));
                    index += 1;
                }
                if index >= num_particles {
                    break;
                }
            }
            if index >= num_particles {
                break;
            }
        }
    }

    fn step(&mut self, acceleration: &Vector3D) {
        let size = self.config.cube_size as f32;
        let bound = size - 1.0;

        self.log_step_info(acceleration);

        // Pre-compute scaled acceleration
        let scaled_accel = *acceleration * (ACCELERATION_SCALE * GRAVITY);
        let smoothing = VELOCITY_SMOOTHING;
        let max_vel = MAX_VELOCITY;
        let damping = DAMPING;
        let decay = VELOCITY_DECAY;

        // Update each particle's velocity and position
        for particle in self.particles.iter_mut() {
            // Update velocity with acceleration
            particle.velocity.x = particle.velocity.x * smoothing + scaled_accel.x;
            particle.velocity.y = particle.velocity.y * smoothing + scaled_accel.y;
            particle.velocity.z = particle.velocity.z * smoothing + scaled_accel.z;

            // Constrain velocity
            particle.velocity.x = particle.velocity.x.clamp(-max_vel, max_vel);
            particle.velocity.y = particle.velocity.y.clamp(-max_vel, max_vel);
            particle.velocity.z = particle.velocity.z.clamp(-max_vel, max_vel);

            // Update position
            particle.position.x += particle.velocity.x;
            particle.position.y += particle.velocity.y;
            particle.position.z += particle.velocity.z;

            // Apply boundary collisions with bounce
            physics_utils::apply_boundary_3d(
                &mut particle.position,
                &mut particle.velocity,
                bound,
                damping,
            );

            // Additional velocity decay
            particle.velocity *= decay;
        }

        // Build spatial grid for collision detection
        self.grid.clear(self.particles.len());
        for (i, particle) in self.particles.iter().enumerate() {
            self.grid.insert(i as u16, &particle.position);
        }

        // Particle collision detection using spatial grid
        let repulsion_factor = COLLISION_REPULSION * 0.5 * COLLISION_DAMPING;
        let particle_count = self.particles.len();

        for i in 0..particle_count {
            let pos_i = self.particles[i].position;

            // Only check particles in neighboring cells
            for cell_idx in self.grid.neighbor_cells(pos_i.x, pos_i.y, pos_i.z) {
                let mut neighbor_idx = self.grid.head[cell_idx];
                while neighbor_idx != u16::MAX {
                    let j = neighbor_idx as usize;
                    neighbor_idx = self.grid.next[j];

                    // Only check pairs once (j > i)
                    if j <= i {
                        continue;
                    }

                    let dx = self.particles[j].position.x - self.particles[i].position.x;
                    let dy = self.particles[j].position.y - self.particles[i].position.y;
                    let dz = self.particles[j].position.z - self.particles[i].position.z;
                    let dist_squared = dx * dx + dy * dy + dz * dz;

                    if dist_squared < COLLISION_THRESHOLD && dist_squared > MIN_DISTANCE_SQUARED {
                        let dist = dist_squared.sqrt();
                        let inv_dist = 1.0 / dist;

                        // Normalize direction vector and apply repulsion
                        let rx = dx * inv_dist * repulsion_factor;
                        let ry = dy * inv_dist * repulsion_factor;
                        let rz = dz * inv_dist * repulsion_factor;

                        // Separate particles
                        self.particles[i].position.x -= rx;
                        self.particles[i].position.y -= ry;
                        self.particles[i].position.z -= rz;
                        self.particles[j].position.x += rx;
                        self.particles[j].position.y += ry;
                        self.particles[j].position.z += rz;

                        // Average velocities
                        let avg_vel_x =
                            (self.particles[i].velocity.x + self.particles[j].velocity.x) * 0.5;
                        let avg_vel_y =
                            (self.particles[i].velocity.y + self.particles[j].velocity.y) * 0.5;
                        let avg_vel_z =
                            (self.particles[i].velocity.z + self.particles[j].velocity.z) * 0.5;

                        self.particles[i].velocity.x = avg_vel_x;
                        self.particles[i].velocity.y = avg_vel_y;
                        self.particles[i].velocity.z = avg_vel_z;
                        self.particles[j].velocity.x = avg_vel_x;
                        self.particles[j].velocity.y = avg_vel_y;
                        self.particles[j].velocity.z = avg_vel_z;
                    }
                }
            }
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

impl FluidSimulation {
    fn log_step_info(&self, acceleration: &Vector3D) {
        let magnitude = acceleration.magnitude();
        info!("=== Simulation Step ===");
        info!(
            "Acceleration: ({:.2}, {:.2}, {:.2}) |{:.2}|g",
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
            num_particles: 4,
            delay_ms: 35,
            color: Color::DEFAULT,
        }
    }

    #[test]
    fn test_simulation_creation() {
        let config = test_config();
        let sim = FluidSimulation::new(config);
        assert!(sim.particles.is_empty());
        assert_eq!(sim.config().cube_size, 8);
    }

    #[test]
    fn test_populate_particles() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);
        sim.populate(4);
        assert_eq!(sim.particles().len(), 4);

        // Particles should start in bottom half
        for particle in sim.particles() {
            assert!(particle.position.y >= 4.0);
            assert!(particle.position.y < 8.0);
        }
    }

    #[test]
    fn test_gravity_moves_particles_down() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);

        // Place a particle in the middle
        sim.particles.push(Particle::new(3.5, 3.5, 3.5));
        let initial_y = sim.particles[0].position.y;

        // Apply downward gravity (positive Y is down in this coordinate system)
        let gravity = Vector3D::new(0.0, 1.0, 0.0);

        // Run several steps
        for _ in 0..10 {
            sim.step(&gravity);
        }

        // Particle should have moved down (higher Y value)
        assert!(sim.particles[0].position.y > initial_y);
    }

    #[test]
    fn test_boundary_collision_bottom() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);

        // Place particle at bottom with downward velocity
        let mut particle = Particle::new(3.5, 6.9, 3.5);
        particle.velocity.y = 0.5;
        sim.particles.push(particle);

        // Apply gravity pushing down
        let gravity = Vector3D::new(0.0, 1.0, 0.0);

        // Run simulation
        for _ in 0..20 {
            sim.step(&gravity);
        }

        // Particle should be constrained to bounds
        let bound = config.cube_size as f32 - 1.0;
        assert!(sim.particles[0].position.y <= bound);
        assert!(sim.particles[0].position.y >= 0.0);
    }

    #[test]
    fn test_boundary_collision_all_sides() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);

        // Place particle in center
        sim.particles.push(Particle::new(3.5, 3.5, 3.5));
        let bound = config.cube_size as f32 - 1.0;

        // Test each direction
        let directions = [
            Vector3D::new(1.0, 0.0, 0.0),  // +X
            Vector3D::new(-1.0, 0.0, 0.0), // -X
            Vector3D::new(0.0, 1.0, 0.0),  // +Y
            Vector3D::new(0.0, -1.0, 0.0), // -Y
            Vector3D::new(0.0, 0.0, 1.0),  // +Z
            Vector3D::new(0.0, 0.0, -1.0), // -Z
        ];

        for gravity in directions {
            sim.particles[0] = Particle::new(3.5, 3.5, 3.5);

            for _ in 0..100 {
                sim.step(&gravity);
            }

            // Check all coordinates are within bounds
            let p = &sim.particles[0].position;
            assert!(p.x >= 0.0 && p.x <= bound, "X out of bounds: {}", p.x);
            assert!(p.y >= 0.0 && p.y <= bound, "Y out of bounds: {}", p.y);
            assert!(p.z >= 0.0 && p.z <= bound, "Z out of bounds: {}", p.z);
        }
    }

    #[test]
    fn test_velocity_clamping() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);

        // Place particle with extreme velocity
        let mut particle = Particle::new(3.5, 3.5, 3.5);
        particle.velocity = Vector3D::new(10.0, 10.0, 10.0);
        sim.particles.push(particle);

        // Run one step
        let gravity = Vector3D::new(0.0, 0.0, 0.0);
        sim.step(&gravity);

        // Velocity should be clamped to max_velocity
        let v = &sim.particles[0].velocity;
        assert!(v.x.abs() <= MAX_VELOCITY);
        assert!(v.y.abs() <= MAX_VELOCITY);
        assert!(v.z.abs() <= MAX_VELOCITY);
    }

    #[test]
    fn test_particle_collision_separation() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);

        // Place two particles very close together
        sim.particles.push(Particle::new(3.5, 3.5, 3.5));
        sim.particles.push(Particle::new(3.6, 3.5, 3.5));

        let p1_initial = sim.particles[0].position;
        let p2_initial = sim.particles[1].position;
        let initial_dx = p2_initial.x - p1_initial.x;

        // Run simulation with no gravity for more steps
        let gravity = Vector3D::new(0.0, 0.0, 0.0);
        for _ in 0..50 {
            sim.step(&gravity);
        }

        // Check that particles moved apart from collision repulsion
        let p1 = sim.particles[0].position;
        let p2 = sim.particles[1].position;
        let final_dx = p2.x - p1.x;

        // The separation should have increased
        assert!(
            final_dx > initial_dx,
            "Particles should separate: initial dx={}, final dx={}",
            initial_dx,
            final_dx
        );
    }

    #[test]
    fn test_grid_position_conversion() {
        let particle = Particle::new(2.7, 5.3, 7.9);
        let grid_pos = particle.grid_position();

        assert_eq!(grid_pos.x, 2);
        assert_eq!(grid_pos.y, 5);
        assert_eq!(grid_pos.z, 7);
    }

    #[test]
    fn test_grid_position_clamping() {
        // Test positions outside bounds get clamped
        let particle = Particle::new(-1.0, 10.0, 3.5);
        let grid_pos = particle.grid_position();

        assert_eq!(grid_pos.x, 0); // Clamped from -1
        assert_eq!(grid_pos.y, 7); // Clamped from 10
        assert_eq!(grid_pos.z, 3);
    }

    #[test]
    fn test_active_pixels_returns_correct_count() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);
        sim.populate(10);

        let pixels: Vec<_> = sim.active_pixels().collect();
        assert_eq!(pixels.len(), 10);
    }

    #[test]
    fn test_zero_gravity_velocity_decay() {
        let config = test_config();
        let mut sim = FluidSimulation::new(config);

        // Place particle with initial velocity
        let mut particle = Particle::new(3.5, 3.5, 3.5);
        particle.velocity = Vector3D::new(0.5, 0.5, 0.5);
        sim.particles.push(particle);

        let initial_speed = sim.particles[0].speed();

        // Run with zero gravity
        let gravity = Vector3D::new(0.0, 0.0, 0.0);
        for _ in 0..20 {
            sim.step(&gravity);
        }

        // Velocity should have decayed
        let final_speed = sim.particles[0].speed();
        assert!(
            final_speed < initial_speed,
            "Velocity should decay over time"
        );
    }
}

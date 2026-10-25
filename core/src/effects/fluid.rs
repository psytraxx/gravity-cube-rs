//! Fluid physics simulation engine.
//!
//! This module contains the fluid simulation logic for gravity-responsive particles.
//! It uses spatial grid hashing for efficient O(n) average-case collision detection.

use crate::effect::SimulationEffect;
use crate::types::*;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use micromath::F32Ext;

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

/// Collision detection threshold - particles closer than this distance will repel.
const COLLISION_THRESHOLD: f32 = 1.0;

/// Minimum distance squared to avoid division by zero in collision calculations.
const MIN_DISTANCE_SQUARED: f32 = 0.0001;

/// Grid cell size for spatial hashing (should be >= COLLISION_THRESHOLD).
const GRID_CELL_SIZE: f32 = 1.0;

// Fluid physics constants
/// Gravity acceleration multiplier.
const GRAVITY: f32 = 0.15;

/// Acceleration responsiveness scale factor.
///
/// This factor balances responsiveness to acceleration (e.g., from IMU) with stability.
/// Lower values (0.1-0.2) make fluid more stable but less responsive to tilt/shake.
/// Higher values (0.4-0.5) make it more dynamic but can become unstable.
const ACCELERATION_SCALE: f32 = 0.3;

/// Maximum velocity per axis.
const MAX_VELOCITY: f32 = 0.9;

/// Velocity smoothing/inertia factor (0.0-1.0).
const VELOCITY_SMOOTHING: f32 = 0.9;

/// Velocity decay per step - friction (0.0-1.0).
const VELOCITY_DECAY: f32 = 0.95;

/// Fraction of the overlap between two particles corrected per solver pass (0.0-1.0).
const COLLISION_STIFFNESS: f32 = 0.8;

/// Collision solver passes per step. More passes keep a deep pile from compressing.
const SOLVER_ITERATIONS: usize = 3;

/// How far (in cells) a particle must move from its displayed cell's centre before its LED
/// moves. Values above 0.5 stop particles sitting on a cell boundary from flickering.
const DISPLAY_HYSTERESIS: f32 = 0.65;

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
    /// Displayed cell per particle, updated with hysteresis.
    cells: Vec<Position>,
    /// Positions at the start of the current step, used to derive velocities.
    previous: Vec<Vector3D>,
}

impl SimulationEffect for FluidSimulation {
    fn new(config: Config) -> Self {
        let grid = SpatialGrid::new(config.cube_size);
        Self {
            particles: Vec::new(),
            config,
            grid,
            cells: Vec::new(),
            previous: Vec::new(),
        }
    }

    fn populate(&mut self, num_particles: usize) {
        self.particles.clear();
        self.cells.clear();
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
        let bound = self.config.cube_size as f32 - 1.0;
        let scaled_accel = *acceleration * (ACCELERATION_SCALE * GRAVITY);

        // Integrate velocity and move every particle to its predicted position
        self.previous.clear();
        for particle in self.particles.iter_mut() {
            self.previous.push(particle.position);

            particle.velocity = particle.velocity * VELOCITY_SMOOTHING + scaled_accel;
            particle.velocity.x = particle.velocity.x.clamp(-MAX_VELOCITY, MAX_VELOCITY);
            particle.velocity.y = particle.velocity.y.clamp(-MAX_VELOCITY, MAX_VELOCITY);
            particle.velocity.z = particle.velocity.z.clamp(-MAX_VELOCITY, MAX_VELOCITY);

            particle.position += particle.velocity;
            clamp_to_cube(&mut particle.position, bound);
        }

        // Push overlapping particles apart, keeping them inside the cube
        for _ in 0..SOLVER_ITERATIONS {
            self.resolve_collisions();
            for particle in self.particles.iter_mut() {
                clamp_to_cube(&mut particle.position, bound);
            }
        }

        // Velocity is the distance actually travelled: a particle blocked by a wall or
        // by its neighbours ends up at rest instead of pressing on forever
        for (particle, previous) in self.particles.iter_mut().zip(self.previous.iter()) {
            particle.velocity = (particle.position - *previous) * VELOCITY_DECAY;
        }

        self.update_cells();
    }

    fn particles(&self) -> &[Particle] {
        &self.particles
    }

    fn active_pixels(&self) -> impl Iterator<Item = Pixel> + '_ {
        self.particles.iter().enumerate().map(|(i, p)| {
            let cell = self.cells.get(i).copied().unwrap_or(p.grid_position());
            Pixel::new(cell.x, cell.y, cell.z)
        })
    }

    fn config(&self) -> &Config {
        &self.config
    }
}

impl FluidSimulation {
    /// One solver pass: separate every overlapping pair in proportion to its overlap.
    fn resolve_collisions(&mut self) {
        self.grid.clear(self.particles.len());
        for (i, particle) in self.particles.iter().enumerate() {
            self.grid.insert(i as u16, &particle.position);
        }

        for i in 0..self.particles.len() {
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

                    let delta = self.particles[j].position - self.particles[i].position;
                    let dist_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

                    if dist_squared < COLLISION_THRESHOLD && dist_squared > MIN_DISTANCE_SQUARED {
                        let dist = dist_squared.sqrt();
                        let push = (COLLISION_THRESHOLD - dist) * 0.5 * COLLISION_STIFFNESS;
                        let offset = delta * (push / dist);
                        self.particles[i].position -= offset;
                        self.particles[j].position += offset;
                    }
                }
            }
        }
    }

    /// Move each particle's displayed cell only once it has clearly left it.
    fn update_cells(&mut self) {
        let start = self.cells.len();
        for p in &self.particles[start.min(self.particles.len())..] {
            self.cells.push(p.grid_position());
        }
        for (cell, p) in self.cells.iter_mut().zip(self.particles.iter()) {
            let target = p.grid_position();
            if (p.position.x - cell.x as f32).abs() > DISPLAY_HYSTERESIS {
                cell.x = target.x;
            }
            if (p.position.y - cell.y as f32).abs() > DISPLAY_HYSTERESIS {
                cell.y = target.y;
            }
            if (p.position.z - cell.z as f32).abs() > DISPLAY_HYSTERESIS {
                cell.z = target.z;
            }
        }
    }
}

/// Keep a position inside the cube on every axis.
#[inline]
fn clamp_to_cube(position: &mut Vector3D, bound: f32) {
    position.x = position.x.clamp(0.0, bound);
    position.y = position.y.clamp(0.0, bound);
    position.z = position.z.clamp(0.0, bound);
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

        // Rounds to the nearest cell
        assert_eq!(grid_pos.x, 3);
        assert_eq!(grid_pos.y, 5);
        assert_eq!(grid_pos.z, 7);
    }

    #[test]
    fn test_grid_position_clamping() {
        // Test positions outside bounds get clamped
        let particle = Particle::new(-1.0, 10.0, 3.2);
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

    /// Count LEDs that change between consecutive frames over `frames` steps.
    fn changed_pixels(sim: &mut FluidSimulation, gravity: &Vector3D, frames: usize) -> usize {
        let occupied = |sim: &FluidSimulation| {
            let mut cells = [false; 512];
            for p in sim.active_pixels() {
                let pos = p.position;
                cells[pos.x as usize + pos.y as usize * 8 + pos.z as usize * 64] = true;
            }
            cells
        };
        let mut prev = occupied(sim);
        let mut changed = 0;
        for _ in 0..frames {
            sim.step(gravity);
            let cur = occupied(sim);
            changed += prev.iter().zip(cur.iter()).filter(|(a, b)| a != b).count();
            prev = cur;
        }
        changed
    }

    #[test]
    fn test_particle_on_floor_comes_to_rest() {
        let mut sim = FluidSimulation::new(test_config());
        sim.particles.push(Particle::new(3.0, 3.0, 3.0));
        let gravity = Vector3D::new(0.0, 1.0, 0.0);
        for _ in 0..200 {
            sim.step(&gravity);
        }
        assert_eq!(sim.particles[0].grid_position().y, 7);
        assert!(sim.particles[0].speed() < 0.01);
    }

    #[test]
    fn test_full_cube_settles_without_flicker() {
        let mut config = test_config();
        config.num_particles = 256;
        let mut sim = FluidSimulation::new(config);
        sim.populate(256);
        // Tilted gravity so the pile has to flow before it settles
        let gravity = Vector3D::new(0.3, 1.0, 0.2);
        for _ in 0..600 {
            sim.step(&gravity);
        }
        let changed = changed_pixels(&mut sim, &gravity, 100);
        assert_eq!(changed, 0, "LEDs still changing after settling");
    }
}

//! Starfield: stars stream out of the centre of the cube, fixed in the world.
//!
//! The stars fly through the world, not through the cube: turning the cube turns it around
//! the starfield, which keeps flying the same way, as if the cube were a window onto it.
//! Gravity and shaking do not affect it.

use crate::effect::SimulationEffect;
use crate::physics_utils::rotate;
use crate::types::*;
use micromath::vector::Vector;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use micromath::F32Ext;

#[cfg(not(feature = "std"))]
use alloc::{boxed::Box, vec::Vec};

/// Speed of a star as it leaves the centre, in cells per step.
const START_SPEED: f32 = 0.03;

/// Speed gained per step per cell from the centre, so stars accelerate as they fly out
/// (each step also moves a star this fraction of its distance further out).
const ACCELERATION: f32 = 0.05;

/// Distance from the centre at which a new star appears.
const SPAWN_DISTANCE: f32 = 0.01;

/// Starfield simulation.
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, SimulationEffect, Vector3D, Vector3DExt};
/// use gravity_cube_core::effects::starfield::StarfieldSimulation;
///
/// let mut sim = StarfieldSimulation::new(Config::default());
/// sim.populate(24);
///
/// // The cube turned a little about its Z axis since the last step
/// sim.set_rotation(Vector3D::new(0.0, 0.0, 0.02));
/// sim.step(&Vector3D::new(0.0, 0.0, -1.0)); // Gravity is ignored
/// assert_eq!(sim.particles().len(), 24);
/// ```
pub struct StarfieldSimulation {
    particles: Vec<Particle>,
    config: Config,
    /// Rotation of the cube per step (axis times radians), in cube axes.
    rotation: Vector3D,
    /// Xorshift state for star directions.
    rng: u32,
}

impl SimulationEffect for StarfieldSimulation {
    fn new(config: Config) -> Self {
        Self {
            particles: Vec::new(),
            config,
            rotation: Vector3D::new(0.0, 0.0, 0.0),
            rng: 0x2545_F491,
        }
    }

    fn name(&self) -> &'static str {
        "starfield"
    }

    /// Scatter stars at random distances along random directions, so the field starts full
    /// instead of as a single burst from the centre.
    fn populate(&mut self, num_particles: usize) {
        self.particles.clear();
        let center = self.center();
        for _ in 0..num_particles {
            let distance = SPAWN_DISTANCE + self.random() * center.x;
            let offset = self.random_direction() * distance;
            self.particles.push(star(center + offset));
        }
    }

    fn step(&mut self, _acceleration: &Vector3D) {
        let center = self.center();
        let bound = self.config.cube_size as f32 - 1.0;
        // The starfield is fixed in the world, so in cube axes it turns opposite to the cube
        let unturn = self.rotation * -1.0;

        for i in 0..self.particles.len() {
            let previous = self.particles[i].position;
            let offset = rotate(previous - center, unturn);
            let distance = offset.magnitude();
            let speed = START_SPEED + distance * ACCELERATION;
            let position = center + offset + offset * (speed / distance);

            let outside = [position.x, position.y, position.z]
                .iter()
                .any(|&c| !(-0.5..=bound + 0.5).contains(&c));
            self.particles[i] = if outside {
                star(center + self.random_direction() * SPAWN_DISTANCE)
            } else {
                let mut moved = star(position);
                moved.velocity = position - previous;
                moved
            };
        }
    }

    fn set_rotation(&mut self, rotation: Vector3D) {
        self.rotation = rotation;
    }

    fn particles(&self) -> &[Particle] {
        &self.particles
    }

    fn active_pixels(&self) -> Box<dyn Iterator<Item = Pixel> + '_> {
        Box::new(self.particles.iter().map(|p| {
            let cell = p.grid_position();
            Pixel::new(cell.x, cell.y, cell.z)
        }))
    }

    fn config(&self) -> &Config {
        &self.config
    }
}

/// A resting star at a position.
fn star(position: Vector3D) -> Particle {
    Particle::new(position.x, position.y, position.z)
}

impl StarfieldSimulation {
    fn center(&self) -> Vector3D {
        let middle = (self.config.cube_size as f32 - 1.0) / 2.0;
        Vector3D::new(middle, middle, middle)
    }

    /// Uniform random number in [0, 1).
    fn random(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Random unit vector, evenly spread over all directions.
    fn random_direction(&mut self) -> Vector3D {
        loop {
            let v = Vector3D::new(
                self.random() * 2.0 - 1.0,
                self.random() * 2.0 - 1.0,
                self.random() * 2.0 - 1.0,
            );
            let length_squared = v.dot(v);
            // Points inside the unit ball, away from the centre, give an even spread
            if length_squared > 0.01 && length_squared <= 1.0 {
                return v * (1.0 / length_squared.sqrt());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NO_GRAVITY: Vector3D = Vector3D {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    fn star_at(x: f32, y: f32, z: f32) -> StarfieldSimulation {
        let mut sim = StarfieldSimulation::new(Config::default());
        sim.particles.push(Particle::new(x, y, z));
        sim
    }

    #[test]
    fn stars_fly_outwards_and_speed_up() {
        let mut sim = star_at(4.0, 3.5, 3.5);
        sim.step(&NO_GRAVITY);
        let first = sim.particles[0].velocity.x;
        sim.step(&NO_GRAVITY);
        let second = sim.particles[0].velocity.x;

        assert!(first > 0.0, "star should move away from the centre");
        assert!(second > first, "star should accelerate");
        assert!((sim.particles[0].position.y - 3.5).abs() < 0.001);
    }

    #[test]
    fn starfield_stays_fixed_when_the_cube_turns() {
        // A star on +X; the cube turns +Z, so in cube axes the star turns towards -Y
        let mut sim = star_at(5.5, 3.5, 3.5);
        sim.set_rotation(Vector3D::new(0.0, 0.0, 0.1));
        sim.step(&NO_GRAVITY);

        let offset = sim.particles[0].position - sim.center();
        let angle = offset.y.atan2(offset.x);
        assert!((angle + 0.1).abs() < 0.01, "expected -0.1 rad, got {angle}");
    }

    #[test]
    fn stars_leaving_the_cube_respawn_at_the_centre() {
        let mut sim = star_at(7.4, 3.5, 3.5);
        sim.step(&NO_GRAVITY);

        let offset = sim.particles[0].position - sim.center();
        assert!(
            offset.magnitude() < 0.1,
            "got {:?}",
            sim.particles[0].position
        );
    }

    #[test]
    fn gravity_is_ignored() {
        let mut a = star_at(4.0, 4.0, 4.0);
        let mut b = star_at(4.0, 4.0, 4.0);
        a.step(&NO_GRAVITY);
        b.step(&Vector3D::new(0.0, 0.0, -1.0));
        assert_eq!(a.particles[0].position, b.particles[0].position);
    }

    #[test]
    fn stars_stay_inside_the_cube() {
        let mut sim = StarfieldSimulation::new(Config::default());
        sim.populate(24);
        sim.set_rotation(Vector3D::new(0.03, -0.02, 0.05));
        for _ in 0..500 {
            sim.step(&NO_GRAVITY);
            assert_eq!(sim.particles().len(), 24);
            for p in sim.particles() {
                for c in [p.position.x, p.position.y, p.position.z] {
                    assert!((-0.5..=7.5).contains(&c), "star outside the cube: {p:?}");
                }
            }
        }
    }
}

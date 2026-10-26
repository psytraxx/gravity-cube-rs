//! Cycles between the simulation effects at runtime.

use crate::effect::SimulationEffect;
use crate::effects::boids::BoidsSimulation;
use crate::effects::fluid::FluidSimulation;
use crate::types::*;

/// Runs one effect at a time and switches to the next on request (a double tap on the
/// cube, a double click on desktop).
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, EffectSwitcher, SimulationEffect};
///
/// let config = Config::default();
/// let mut effects = EffectSwitcher::new(config);
/// effects.populate(config.num_particles);
/// assert_eq!(effects.name(), "fluid");
///
/// effects.next_effect();
/// assert_eq!(effects.name(), "boids");
/// assert_eq!(effects.particles().len(), config.num_particles);
/// ```
pub enum EffectSwitcher {
    Fluid(FluidSimulation),
    Boids(BoidsSimulation),
}

impl EffectSwitcher {
    /// Replace the running effect with the next one, with the same number of particles.
    pub fn next_effect(&mut self) {
        let config = *self.config();
        let count = self.particles().len();
        *self = match self {
            Self::Fluid(_) => Self::Boids(BoidsSimulation::new(config)),
            Self::Boids(_) => Self::Fluid(FluidSimulation::new(config)),
        };
        self.populate(count);
    }

    /// Short name of the running effect, for logs.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Fluid(_) => "fluid",
            Self::Boids(_) => "boids",
        }
    }
}

impl SimulationEffect for EffectSwitcher {
    /// Starts with the fluid effect.
    fn new(config: Config) -> Self {
        Self::Fluid(FluidSimulation::new(config))
    }

    fn populate(&mut self, num_particles: usize) {
        match self {
            Self::Fluid(sim) => sim.populate(num_particles),
            Self::Boids(sim) => sim.populate(num_particles),
        }
    }

    fn step(&mut self, acceleration: &Vector3D) {
        match self {
            Self::Fluid(sim) => sim.step(acceleration),
            Self::Boids(sim) => sim.step(acceleration),
        }
    }

    fn set_rotation(&mut self, rotation: Vector3D) {
        match self {
            Self::Fluid(sim) => sim.set_rotation(rotation),
            Self::Boids(sim) => sim.set_rotation(rotation),
        }
    }

    fn particles(&self) -> &[Particle] {
        match self {
            Self::Fluid(sim) => sim.particles(),
            Self::Boids(sim) => sim.particles(),
        }
    }

    fn active_pixels(&self) -> impl Iterator<Item = Pixel> + '_ {
        match self {
            Self::Fluid(sim) => Pixels::Fluid(sim.active_pixels()),
            Self::Boids(sim) => Pixels::Boids(sim.active_pixels()),
        }
    }

    fn config(&self) -> &Config {
        match self {
            Self::Fluid(sim) => sim.config(),
            Self::Boids(sim) => sim.config(),
        }
    }
}

/// Pixel iterator of whichever effect is running.
enum Pixels<F, B> {
    Fluid(F),
    Boids(B),
}

impl<F, B> Iterator for Pixels<F, B>
where
    F: Iterator<Item = Pixel>,
    B: Iterator<Item = Pixel>,
{
    type Item = Pixel;

    fn next(&mut self) -> Option<Pixel> {
        match self {
            Self::Fluid(pixels) => pixels.next(),
            Self::Boids(pixels) => pixels.next(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn next_effect_cycles_back_to_fluid() {
        let config = Config::default();
        let mut effects = EffectSwitcher::new(config);
        effects.populate(32);

        effects.next_effect();
        effects.next_effect();
        assert_eq!(effects.name(), "fluid");
        assert_eq!(effects.particles().len(), 32);
        assert_eq!(effects.active_pixels().count(), 32);
    }
}

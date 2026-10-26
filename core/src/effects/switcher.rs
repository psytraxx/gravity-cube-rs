//! Cycles between the simulation effects at runtime.

use crate::effect::SimulationEffect;
use crate::effects::boids::BoidsSimulation;
use crate::effects::fluid::FluidSimulation;
use crate::types::*;

/// Runs one effect at a time and switches to the next on request (a double tap on the
/// cube, a double click on desktop).
///
/// Each effect has its own [`Config`], so particle count and colour are set per effect.
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, EffectSwitcher, SimulationEffect};
///
/// let fluid = Config { num: 256, ..Config::default() };
/// let boids = Config { num: 32, ..fluid };
/// let mut effects = EffectSwitcher::with_configs(fluid, boids);
/// effects.populate(effects.config().num);
/// assert_eq!(effects.name(), "fluid");
///
/// effects.next_effect();
/// assert_eq!(effects.name(), "boids");
/// assert_eq!(effects.particles().len(), 32);
/// ```
pub struct EffectSwitcher {
    effect: Effect,
    fluid_config: Config,
    boids_config: Config,
}

/// The running effect.
enum Effect {
    Fluid(FluidSimulation),
    Boids(BoidsSimulation),
}

impl EffectSwitcher {
    /// Start with the fluid effect, using each effect's own config whenever it runs.
    pub fn with_configs(fluid_config: Config, boids_config: Config) -> Self {
        Self {
            effect: Effect::Fluid(FluidSimulation::new(fluid_config)),
            fluid_config,
            boids_config,
        }
    }

    /// Replace the running effect with the next one, populated with `num` from its config.
    pub fn next_effect(&mut self) {
        self.effect = match self.effect {
            Effect::Fluid(_) => Effect::Boids(BoidsSimulation::new(self.boids_config)),
            Effect::Boids(_) => Effect::Fluid(FluidSimulation::new(self.fluid_config)),
        };
        self.populate(self.config().num);
    }

    /// Short name of the running effect, for logs.
    pub fn name(&self) -> &'static str {
        match self.effect {
            Effect::Fluid(_) => "fluid",
            Effect::Boids(_) => "boids",
        }
    }
}

impl SimulationEffect for EffectSwitcher {
    /// Starts with the fluid effect; every effect uses the same config.
    fn new(config: Config) -> Self {
        Self::with_configs(config, config)
    }

    fn populate(&mut self, num_particles: usize) {
        match &mut self.effect {
            Effect::Fluid(sim) => sim.populate(num_particles),
            Effect::Boids(sim) => sim.populate(num_particles),
        }
    }

    fn step(&mut self, acceleration: &Vector3D) {
        match &mut self.effect {
            Effect::Fluid(sim) => sim.step(acceleration),
            Effect::Boids(sim) => sim.step(acceleration),
        }
    }

    fn set_rotation(&mut self, rotation: Vector3D) {
        match &mut self.effect {
            Effect::Fluid(sim) => sim.set_rotation(rotation),
            Effect::Boids(sim) => sim.set_rotation(rotation),
        }
    }

    fn particles(&self) -> &[Particle] {
        match &self.effect {
            Effect::Fluid(sim) => sim.particles(),
            Effect::Boids(sim) => sim.particles(),
        }
    }

    fn active_pixels(&self) -> impl Iterator<Item = Pixel> + '_ {
        match &self.effect {
            Effect::Fluid(sim) => Pixels::Fluid(sim.active_pixels()),
            Effect::Boids(sim) => Pixels::Boids(sim.active_pixels()),
        }
    }

    /// Config of the running effect.
    fn config(&self) -> &Config {
        match &self.effect {
            Effect::Fluid(sim) => sim.config(),
            Effect::Boids(sim) => sim.config(),
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
    fn next_effect_uses_each_effects_config() {
        let fluid = Config {
            num: 256,
            ..Config::default()
        };
        let boids = Config {
            num: 32,
            color: Color::new(255, 170, 60),
            ..fluid
        };
        let mut effects = EffectSwitcher::with_configs(fluid, boids);
        effects.populate(effects.config().num);

        effects.next_effect();
        assert_eq!(effects.name(), "boids");
        assert_eq!(effects.particles().len(), 32);
        assert_eq!(effects.config().color, boids.color);

        effects.next_effect();
        assert_eq!(effects.name(), "fluid");
        assert_eq!(effects.particles().len(), 256);
        assert_eq!(effects.active_pixels().count(), 256);
        assert_eq!(effects.config().color, fluid.color);
    }
}

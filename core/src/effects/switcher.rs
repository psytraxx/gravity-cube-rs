//! Cycles between the simulation effects at runtime.

use core::ops::{Deref, DerefMut};

use crate::effect::SimulationEffect;
use crate::effects::{
    boids::BoidsSimulation, fluid::FluidSimulation, starfield::StarfieldSimulation,
};
use crate::types::*;

#[cfg(not(feature = "std"))]
use alloc::{boxed::Box, vec, vec::Vec};

/// Every effect, in the order a double tap cycles through them, with its particle count and
/// full-brightness colour. Adding an effect only needs its module and a line here.
fn effects(base: Config) -> Vec<EffectEntry> {
    let config = |num, color| Config { num, color, ..base };
    vec![
        // Green only: one LED channel draws the least current on the cube
        EffectEntry::new::<FluidSimulation>(config(256, Color::new(0, 255, 0))),
        // A few dozen boids read as a flock; the fluid's hundreds would fill the cube
        EffectEntry::new::<BoidsSimulation>(config(32, Color::new(255, 128, 0))),
        EffectEntry::new::<StarfieldSimulation>(config(24, Color::new(160, 160, 255))),
    ]
}

/// An effect the switcher can run: which effect, and the config it runs with.
struct EffectEntry {
    config: Config,
    create: fn(Config) -> Box<dyn SimulationEffect>,
}

impl EffectEntry {
    /// Run effect `E` with `config`.
    fn new<E: SimulationEffect + 'static>(config: Config) -> Self {
        Self {
            config,
            create: |config| Box::new(E::new(config)),
        }
    }

    /// Create the effect, populated with `num` particles from its config.
    fn start(&self) -> Box<dyn SimulationEffect> {
        let mut effect = (self.create)(self.config);
        effect.populate(self.config.num);
        effect
    }
}

/// Runs one effect at a time and cycles to the next on request (a double tap on the cube, a
/// double click on desktop), wrapping round after the last.
///
/// Dereferences to the running effect, so it is driven like any [`SimulationEffect`].
///
/// # Example
///
/// ```rust
/// use gravity_cube_core::{Config, EffectSwitcher, SimulationEffect};
///
/// let mut effects = EffectSwitcher::new(Config::default());
/// assert_eq!(effects.name(), "fluid");
///
/// effects.next_effect();
/// assert_eq!(effects.name(), "boids");
/// assert_eq!(effects.particles().len(), effects.config().num);
/// ```
pub struct EffectSwitcher {
    entries: Vec<EffectEntry>,
    index: usize,
    current: Box<dyn SimulationEffect>,
}

impl EffectSwitcher {
    /// Start the first effect. `base` sets the fields shared by every effect (cube size,
    /// frame delay); each effect sets its own `num` and `color`.
    pub fn new(base: Config) -> Self {
        let entries = effects(base);
        let current = entries[0].start();
        Self {
            entries,
            index: 0,
            current,
        }
    }

    /// Replace the running effect with the next one, starting it afresh.
    pub fn next_effect(&mut self) {
        self.index = (self.index + 1) % self.entries.len();
        self.current = self.entries[self.index].start();
    }
}

impl Deref for EffectSwitcher {
    type Target = dyn SimulationEffect;

    fn deref(&self) -> &Self::Target {
        self.current.as_ref()
    }
}

impl DerefMut for EffectSwitcher {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.current.as_mut()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn next_effect_cycles_through_every_effect_and_wraps() {
        let base = Config::default();
        let mut switcher = EffectSwitcher::new(base);
        let entries = effects(base);

        for i in 1..=entries.len() {
            switcher.next_effect();
            let expected = &entries[i % entries.len()].config;
            assert_eq!(switcher.particles().len(), expected.num);
            assert_eq!(switcher.active_pixels().count(), expected.num);
            assert_eq!(switcher.config().color, expected.color);
        }
        assert_eq!(switcher.name(), "fluid");
    }
}

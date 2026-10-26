use super::PanelMapper;
use gravity_cube_core::{Color, Position};
use smart_leds::SmartLedsWrite;

/// Number of LEDs in the chain (6 panels * 64 LEDs).
pub const LED_COUNT: usize = 384;

/// Colour of every LED in chain order.
pub type Frame<C> = [C; LED_COUNT];

/// Builds frames by mapping 3D cube positions onto the LED chain.
///
/// Holds no hardware, so it can run on a different core from the `Ws2812Display` that sends
/// its frames.
pub struct FrameBuilder<C = smart_leds::RGB<u8>> {
    panel_mapper: PanelMapper,
    frame: Frame<C>,
}

impl<C> FrameBuilder<C>
where
    C: From<(u8, u8, u8)> + Default + Copy,
{
    pub fn new() -> Self {
        Self {
            panel_mapper: PanelMapper::new(8),
            frame: [C::default(); LED_COUNT],
        }
    }

    /// Turn every LED off.
    pub fn clear(&mut self) {
        self.frame.fill(C::default());
    }

    /// Light every LED that shows a 3D position.
    pub fn set_pixel(&mut self, pos: Position, color: Color) {
        for led_index in self.panel_mapper.position_to_led_indices(pos) {
            if let Some(led) = self.frame.get_mut(led_index as usize) {
                *led = C::from((color.r, color.g, color.b));
            }
        }
    }

    pub fn frame(&self) -> &Frame<C> {
        &self.frame
    }
}

/// WS2812 LED Display adapter using RMT (ESP32's remote control peripheral)
pub struct Ws2812Display<RMT, C = smart_leds::RGB<u8>> {
    driver: RMT,
    /// Last frame sent to the LEDs; `None` until the first frame.
    sent: Option<Frame<C>>,
}

impl<RMT, C> Ws2812Display<RMT, C>
where
    RMT: SmartLedsWrite<Color = C>,
    C: Copy + PartialEq,
{
    pub fn new(driver: RMT) -> Self {
        Self { driver, sent: None }
    }

    /// Send a frame to the LEDs. Unchanged frames are skipped, so a settled cube keeps
    /// the data line quiet instead of re-sending the same picture every cycle.
    pub fn show(&mut self, frame: &Frame<C>) -> Result<(), RMT::Error> {
        if self.sent.as_ref() == Some(frame) {
            return Ok(());
        }
        self.driver.write(frame.iter().copied())?;
        self.sent = Some(*frame);
        Ok(())
    }
}

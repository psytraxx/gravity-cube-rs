use super::PanelMapper;
use gravity_cube_core::{Color, Position};
use smart_leds::SmartLedsWrite;

/// WS2812 LED Display adapter using RMT (ESP32's remote control peripheral)
pub struct Ws2812Display<RMT, C = smart_leds::RGB<u8>>
where
    C: From<(u8, u8, u8)> + Default + Copy + PartialEq,
{
    driver: RMT,
    panel_mapper: PanelMapper,
    buffer: [C; 384], // 6 panels * 64 LEDs
    /// Last frame sent to the LEDs; `None` until the first refresh.
    sent: Option<[C; 384]>,
    max_leds: usize,
}

impl<RMT, C> Ws2812Display<RMT, C>
where
    RMT: SmartLedsWrite<Color = C>,
    C: From<(u8, u8, u8)> + Default + Copy + PartialEq,
{
    pub fn new(driver: RMT, max_leds: usize) -> Self {
        Self {
            driver,
            panel_mapper: PanelMapper::new(8),
            buffer: [C::default(); 384],
            sent: None,
            max_leds,
        }
    }

    fn clear_buffer(&mut self) {
        for led in self.buffer.iter_mut() {
            *led = C::default();
        }
    }
}

impl<RMT, C> Ws2812Display<RMT, C>
where
    RMT: SmartLedsWrite<Color = C>,
    C: From<(u8, u8, u8)> + Default + Copy + PartialEq,
{
    /// Clear the frame buffer. Nothing is sent to the LEDs until `refresh`, so the
    /// cube never shows a blank frame between updates.
    pub async fn clear(&mut self) -> Result<(), RMT::Error> {
        self.clear_buffer();
        Ok(())
    }

    /// Set a pixel at a 3D position (async version for firmware)
    pub async fn set_pixel(&mut self, pos: Position, color: Color) -> Result<(), RMT::Error> {
        // Get all LED indices for this 3D position
        for led_index in self.panel_mapper.position_to_led_indices(pos) {
            if (led_index as usize) < self.max_leds {
                self.buffer[led_index as usize] = C::from((color.r, color.g, color.b));
            }
        }
        Ok(())
    }

    /// Send the frame to the LEDs. Unchanged frames are skipped, so a settled cube keeps
    /// the data line quiet instead of re-sending the same picture every cycle.
    pub async fn refresh(&mut self) -> Result<(), RMT::Error> {
        if self.sent == Some(self.buffer) {
            return Ok(());
        }
        self.driver
            .write(self.buffer[..self.max_leds].iter().cloned())?;
        self.sent = Some(self.buffer);
        Ok(())
    }
}

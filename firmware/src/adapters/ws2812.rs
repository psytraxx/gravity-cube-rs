use super::PanelMapper;
use gravity_cube_core::{Color, Position};
use smart_leds::SmartLedsWriteAsync;

/// WS2812 LED Display adapter using RMT (ESP32's remote control peripheral)
pub struct Ws2812Display<RMT, C = smart_leds::RGB<u8>>
where
    C: From<(u8, u8, u8)> + Default + Copy,
{
    driver: RMT,
    panel_mapper: PanelMapper,
    buffer: [C; 384], // 6 panels * 64 LEDs
    max_leds: usize,
}

impl<RMT, C> Ws2812Display<RMT, C>
where
    RMT: SmartLedsWriteAsync<Color = C>,
    C: From<(u8, u8, u8)> + Default + Copy,
{
    pub fn new(driver: RMT, max_leds: usize) -> Self {
        Self {
            driver,
            panel_mapper: PanelMapper::new(8),
            buffer: [C::default(); 384],
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
    RMT: SmartLedsWriteAsync<Color = C>,
    C: From<(u8, u8, u8)> + Default + Copy,
{
    /// Clear all LEDs (async version for firmware)
    pub async fn clear(&mut self) -> Result<(), RMT::Error> {
        self.clear_buffer();
        self.driver
            .write(self.buffer[..self.max_leds].iter().cloned())
            .await?;
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

    /// Refresh/update the display to show changes (async version for firmware)
    pub async fn refresh(&mut self) -> Result<(), RMT::Error> {
        self.driver
            .write(self.buffer[..self.max_leds].iter().cloned())
            .await?;
        Ok(())
    }
}

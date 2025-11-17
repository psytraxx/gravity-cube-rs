use crate::domain::{Position, Color};
use crate::ports::DisplayPort;
use crate::adapters::PanelMapper;
use smart_leds::SmartLedsWrite;

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
    RMT: SmartLedsWrite<Color = C>,
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

impl<RMT, C> DisplayPort for Ws2812Display<RMT, C>
where
    RMT: SmartLedsWrite<Color = C>,
    C: From<(u8, u8, u8)> + Default + Copy,
{
    type Error = RMT::Error;

    async fn clear(&mut self) -> Result<(), Self::Error> {
        self.clear_buffer();
        self.driver.write(self.buffer[..self.max_leds].iter().cloned())?;
        Ok(())
    }

    async fn set_pixel(&mut self, pos: Position, color: Color) -> Result<(), Self::Error> {
        // Get all LED indices for this 3D position
        for led_index in self.panel_mapper.position_to_led_indices(pos) {
            if (led_index as usize) < self.max_leds {
                self.buffer[led_index as usize] = C::from((color.r, color.g, color.b));
            }
        }
        Ok(())
    }

    async fn refresh(&mut self) -> Result<(), Self::Error> {
        self.driver.write(self.buffer[..self.max_leds].iter().cloned())?;
        Ok(())
    }
}

use crate::domain::{Position, Color};

/// Port for controlling the LED display
pub trait DisplayPort {
    type Error;

    /// Clear all LEDs
    async fn clear(&mut self) -> Result<(), Self::Error>;

    /// Set a pixel at a 3D position
    async fn set_pixel(&mut self, pos: Position, color: Color) -> Result<(), Self::Error>;

    /// Refresh/update the display to show changes
    async fn refresh(&mut self) -> Result<(), Self::Error>;
}

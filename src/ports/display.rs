use crate::domain::{Color, Position};

/// Port for controlling the LED display
pub trait DisplayPort {
    type Error;

    /// Clear all LEDs
    fn clear(&mut self) -> impl Future<Output = Result<(), Self::Error>>;

    /// Set a pixel at a 3D position
    fn set_pixel(
        &mut self,
        pos: Position,
        color: Color,
    ) -> impl Future<Output = Result<(), Self::Error>>;

    /// Refresh/update the display to show changes
    fn refresh(&mut self) -> impl Future<Output = Result<(), Self::Error>>;
}

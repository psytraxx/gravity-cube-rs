/// Port for random number generation
pub trait RandomPort {
    /// Generate a random boolean
    fn next_bool(&mut self) -> bool;
}

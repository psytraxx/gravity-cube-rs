use crate::domain::Vector3D;

/// Port for reading IMU sensor data
pub trait ImuPort {
    type Error;

    /// Read accelerometer data and return as a normalized unit vector
    async fn read_gravity(&mut self) -> Result<Vector3D, Self::Error>;
}

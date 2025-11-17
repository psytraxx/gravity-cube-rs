use crate::domain::Vector3D;

/// Port for reading IMU sensor data
pub trait ImuPort {
    type Error;

    /// Read accelerometer data and return as a normalized unit vector
    fn read_gravity(&mut self) -> impl Future<Output = Result<Vector3D, Self::Error>>;
}

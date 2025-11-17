use crate::domain::{Vector3D, Vector3DExt};
use crate::ports::ImuPort;
use embedded_hal::spi::SpiDevice;
use embedded_hal::delay::DelayNs;

#[derive(Debug)]
pub enum Bmi160Error<E> {
    Sensor(bmi160::Error<E>),
}

/// BMI160 IMU driver wrapper using the bmi160 crate
pub struct Bmi160Adapter<SPI, DELAY> {
    sensor: bmi160::Bmi160<bmi160::interface::SpiInterface<SPI>>,
    delay: DELAY,
    /// Sensor orientation mapping (swap axes based on physical mounting)
    /// Original C code: unit_vector.x = sensor_data.z / magnitude
    ///                  unit_vector.y = sensor_data.y / magnitude
    ///                  unit_vector.z = sensor_data.x / magnitude
    swap_axes: bool,
}

impl<SPI, DELAY, E> Bmi160Adapter<SPI, DELAY>
where
    SPI: SpiDevice<Error = E>,
    DELAY: DelayNs,
{
    pub fn new(spi: SPI, delay: DELAY) -> Result<Self, Bmi160Error<E>> {
        let sensor = bmi160::Bmi160::new_with_spi(spi);

        let adapter = Self {
            sensor,
            delay,
            swap_axes: true, // Match C code orientation
        };

        // Note: Initialization will be done on first read
        // The bmi160 crate v1.1.0 has a simpler API

        Ok(adapter)
    }

    pub fn read_gravity(&mut self) -> Result<Vector3D, Bmi160Error<E>> {
        // Read accelerometer data
        // Use SensorSelector::Accel to get only accelerometer data
        let data = self
            .sensor
            .data(bmi160::SensorSelector::new().accel())
            .map_err(Bmi160Error::Sensor)?;

        // Extract accelerometer values
        let (raw_x, raw_y, raw_z) = if let Some(accel) = data.accel {
            (accel.x, accel.y, accel.z)
        } else {
            // No data available - return zero vector
            (0, 0, 0)
        };

        // Create vector from raw data
        let vec = Vector3D::from_raw(raw_x, raw_y, raw_z);

        // Normalize to unit vector
        let mut normalized = vec.normalize();

        // Apply axis swapping to match physical orientation
        // Original C code mapping:
        // unit_vector.x = sensor_data.z / magnitude
        // unit_vector.y = sensor_data.y / magnitude
        // unit_vector.z = sensor_data.x / magnitude
        if self.swap_axes {
            normalized = normalized.swap_xz();
        }

        Ok(normalized)
    }
}

impl<SPI, DELAY, E> ImuPort for Bmi160Adapter<SPI, DELAY>
where
    SPI: SpiDevice<Error = E>,
    DELAY: DelayNs,
{
    type Error = Bmi160Error<E>;

    async fn read_gravity(&mut self) -> Result<Vector3D, Self::Error> {
        // Call the synchronous version
        // The bmi160 crate doesn't have async support yet
        self.read_gravity()
    }
}

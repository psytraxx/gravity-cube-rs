use bmi160::Bmi160;
use embedded_hal::spi::SpiDevice;
use gravity_cube_core::{Vector3D, Vector3DExt};

/// BMI160 default accelerometer scale: ±2g with 16-bit resolution
/// LSB/g = 32768 / 2 = 16384
const ACCEL_SCALE: f32 = 1.0 / 16384.0;

#[derive(Debug)]
pub enum Bmi160Error<E> {
    Sensor(bmi160::Error<E>),
}

/// BMI160 IMU driver wrapper using the bmi160 crate
pub struct Bmi160Adapter<SPI> {
    sensor: Bmi160<bmi160::interface::SpiInterface<SPI>>,
    /// Sensor orientation mapping (swap axes based on physical mounting)
    /// Original C code: unit_vector.x = sensor_data.z / magnitude
    ///                  unit_vector.y = sensor_data.y / magnitude
    ///                  unit_vector.z = sensor_data.x / magnitude
    swap_axes: bool,
}

impl<SPI, E> Bmi160Adapter<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    pub fn new(spi: SPI) -> Result<Self, Bmi160Error<E>> {
        let mut sensor = Bmi160::new_with_spi(spi);

        // Initialize the sensor with default configuration
        // This sets up power modes and enables the accelerometer
        sensor
            .set_accel_power_mode(bmi160::AccelerometerPowerMode::Normal)
            .map_err(Bmi160Error::Sensor)?;

        // Set accelerometer range to ±2g (matches our ACCEL_SCALE constant)
        sensor
            .set_accel_range(bmi160::AccelerometerRange::G2)
            .map_err(Bmi160Error::Sensor)?;

        let adapter = Self {
            sensor,
            swap_axes: true, // Match C code orientation
        };

        Ok(adapter)
    }

    /// Read acceleration vector in g-force units.
    ///
    /// Returns the actual acceleration including magnitude, not just direction.
    /// At rest, this will return approximately (0, 0, -1) or (0, 0, 1) depending
    /// on orientation (1g from gravity).
    ///
    /// When the device is shaken or moved, the magnitude will exceed 1g.
    pub fn read_acceleration(&mut self) -> Result<Vector3D, Bmi160Error<E>> {
        // Read accelerometer data
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

        // Convert raw i16 to g-force
        let mut accel = Vector3D::new(
            raw_x as f32 * ACCEL_SCALE,
            raw_y as f32 * ACCEL_SCALE,
            raw_z as f32 * ACCEL_SCALE,
        );

        // Apply axis swapping to match physical orientation
        // Original C code mapping:
        // unit_vector.x = sensor_data.z / magnitude
        // unit_vector.y = sensor_data.y / magnitude
        // unit_vector.z = sensor_data.x / magnitude
        if self.swap_axes {
            accel = Vector3D::new(accel.z, accel.y, accel.x);
        }

        Ok(accel)
    }

    /// Read gravity direction as a normalized unit vector.
    ///
    /// This discards the acceleration magnitude and only returns direction.
    /// Use `read_acceleration()` if you need the actual acceleration magnitude.
    #[allow(dead_code)]
    pub fn read_gravity(&mut self) -> Result<Vector3D, Bmi160Error<E>> {
        let accel = self.read_acceleration()?;
        Ok(accel.normalized())
    }
}

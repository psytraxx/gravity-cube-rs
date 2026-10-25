use bmi160::Bmi160;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;
use gravity_cube_core::{Vector3D, Vector3DExt};

/// Accelerometer scale at ±4g with 16-bit resolution: LSB/g = 32768 / 4 = 8192.
/// ±4g leaves headroom so shaking the cube does not clip.
const ACCEL_SCALE: f32 = 1.0 / 8192.0;

/// Sensor-to-cube axis mapping: cube axis `i` = `AXIS_MAP[i].1 * sensor[AXIS_MAP[i].0]`.
///
/// The board is mounted flat on the top face with its Z axis pointing up, so at rest it
/// reads +1 g on Z, which has to pull towards cube -Z (`z == 0`), the bottom face. With
/// panel 0 (`x == 0`) on the floor the sensor reads -1 g on X, so X maps straight through.
/// If tilting moves the fluid the wrong way on an axis, flip that axis' sign here.
const AXIS_MAP: [(usize, f32); 3] = [(0, 1.0), (1, 1.0), (2, -1.0)];

#[derive(Debug)]
pub enum Bmi160Error<E> {
    Sensor(bmi160::Error<E>),
}

/// BMI160 IMU driver wrapper using the bmi160 crate
pub struct Bmi160Adapter<SPI> {
    sensor: Bmi160<bmi160::interface::SpiInterface<SPI>>,
}

impl<SPI, E> Bmi160Adapter<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    pub fn new(spi: SPI, delay: &mut impl DelayNs) -> Result<Self, Bmi160Error<E>> {
        let mut sensor = Bmi160::new_with_spi(spi);
        delay.delay_ms(10);

        // Initialize the sensor with default configuration
        // This sets up power modes and enables the accelerometer
        sensor
            .set_accel_power_mode(bmi160::AccelerometerPowerMode::Normal)
            .map_err(Bmi160Error::Sensor)?;
        // The accelerometer needs a few ms to leave suspend; reading earlier returns zeros.
        delay.delay_ms(80);

        // Set accelerometer range to ±4g (matches our ACCEL_SCALE constant)
        sensor
            .set_accel_range(bmi160::AccelerometerRange::G4)
            .map_err(Bmi160Error::Sensor)?;
        delay.delay_ms(10);

        let power_mode = sensor.power_mode().map_err(Bmi160Error::Sensor)?;
        log::info!("BMI160 power mode: accel={:?}", power_mode.accel);

        Ok(Self { sensor })
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

        // Convert raw i16 to g-force and map sensor axes onto cube axes
        let raw = [
            raw_x as f32 * ACCEL_SCALE,
            raw_y as f32 * ACCEL_SCALE,
            raw_z as f32 * ACCEL_SCALE,
        ];
        let axis = |i: usize| AXIS_MAP[i].1 * raw[AXIS_MAP[i].0];

        Ok(Vector3D::new(axis(0), axis(1), axis(2)))
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

//! Register-level BMI160 driver over SPI: accelerometer, gyroscope and hardware double tap.
//!
//! The `bmi160` crate cannot configure the interrupt engine, which double-tap detection
//! needs, so the few registers used here are written directly.

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};
use gravity_cube_core::{Vector3D, Vector3DExt};

// Registers
const PMU_STATUS: u8 = 0x03;
/// Gyro X, Y, Z then accel X, Y, Z, each a little-endian i16.
const DATA_GYR_X: u8 = 0x0C;
const INT_STATUS_0: u8 = 0x1C;
const ACC_CONF: u8 = 0x40;
const ACC_RANGE: u8 = 0x41;
const GYR_RANGE: u8 = 0x43;
const INT_EN_0: u8 = 0x50;
const INT_LATCH: u8 = 0x54;
const INT_MAP_0: u8 = 0x55;
const INT_TAP_0: u8 = 0x63;
const INT_TAP_1: u8 = 0x64;
const CMD: u8 = 0x7E;

// Commands
const CMD_ACC_NORMAL: u8 = 0x11;
const CMD_GYR_NORMAL: u8 = 0x15;
const CMD_INT_RESET: u8 = 0xB1;

/// Double-tap bit in INT_EN_0, INT_MAP_0 and INT_STATUS_0.
const DOUBLE_TAP: u8 = 1 << 4;

/// 400 Hz, normal filter mode. The tap engine needs at least 200 Hz and no undersampling.
const ACC_CONF_400HZ: u8 = 0x2A;
/// ±4g leaves headroom so shaking the cube does not clip.
const ACC_RANGE_4G: u8 = 0x05;
/// ±1000°/s covers spinning the cube by hand.
const GYR_RANGE_1000DPS: u8 = 0x01;
/// Hold interrupt status until reset with CMD_INT_RESET, so a tap between polls is not lost.
const INT_LATCHED: u8 = 0x0F;

/// Second tap must follow within 375 ms (30 ms quiet, 50 ms shock time).
const TAP_TIMING: u8 = 0b101;
/// Tap threshold in steps of 125 mg at ±4g: 10 = 1.25 g. Raise it if sloshing the fluid
/// triggers effect changes, lower it if taps are missed.
const TAP_THRESHOLD: u8 = 10;

/// Accelerometer scale at ±4g with 16-bit resolution: LSB/g = 32768 / 4 = 8192.
const ACCEL_SCALE: f32 = 1.0 / 8192.0;
/// Gyro scale at ±1000°/s: 32.8 LSB per °/s, converted to rad/s.
const GYRO_SCALE: f32 = core::f32::consts::PI / 180.0 / 32.8;

/// Gyro readings averaged at startup to measure its zero-rate offset.
const CALIBRATION_SAMPLES: u32 = 50;

/// Sensor-to-cube axis mapping: cube axis `i` = `AXIS_MAP[i].1 * sensor[AXIS_MAP[i].0]`.
///
/// The board is mounted flat on the top face with its Z axis pointing up, so at rest it
/// reads +1 g on Z, which has to pull towards cube -Z (`z == 0`), the bottom face. With
/// panel 0 (`x == 0`) on the floor the sensor reads -1 g on X, so X maps straight through.
/// If tilting moves the fluid the wrong way on an axis, flip that axis' sign here.
const AXIS_MAP: [(usize, f32); 3] = [(0, 1.0), (1, 1.0), (2, -1.0)];

/// The accelerometer measures the push opposite to gravity, so `AXIS_MAP` turns it around;
/// the gyro measures the real rotation and needs the opposite signs.
const GYRO_SIGN: f32 = -1.0;

/// One accelerometer and gyro reading, in cube axes.
pub struct ImuReading {
    /// Gravity plus motion in g, pointing the way the fluid should fall.
    pub acceleration: Vector3D,
    /// Rotation rate of the cube in rad/s.
    pub angular_velocity: Vector3D,
}

/// BMI160 IMU on an SPI bus.
pub struct Bmi160Adapter<SPI> {
    spi: SPI,
    /// Gyro zero-rate offset in rad/s (cube axes), subtracted from every reading.
    gyro_bias: Vector3D,
}

impl<SPI, E> Bmi160Adapter<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    /// Power up both sensors, enable double-tap detection and calibrate the gyro.
    ///
    /// The cube should be still while this runs. The chip must already be in SPI mode
    /// (one dummy read after power-up).
    pub fn new(spi: SPI, delay: &mut impl DelayNs) -> Result<Self, E> {
        let mut imu = Self {
            spi,
            gyro_bias: Vector3D::new(0.0, 0.0, 0.0),
        };

        // Each sensor needs time to leave suspend; reading earlier returns zeros
        imu.write_register(CMD, CMD_ACC_NORMAL)?;
        delay.delay_ms(5);
        imu.write_register(CMD, CMD_GYR_NORMAL)?;
        delay.delay_ms(80);

        imu.write_register(ACC_CONF, ACC_CONF_400HZ)?;
        imu.write_register(ACC_RANGE, ACC_RANGE_4G)?;
        imu.write_register(GYR_RANGE, GYR_RANGE_1000DPS)?;

        imu.write_register(INT_TAP_0, TAP_TIMING)?;
        imu.write_register(INT_TAP_1, TAP_THRESHOLD)?;
        imu.write_register(INT_LATCH, INT_LATCHED)?;
        // INT1 is not wired and its output stays disabled; mapping the interrupt only makes
        // sure the engine reports it in INT_STATUS_0
        imu.write_register(INT_MAP_0, DOUBLE_TAP)?;
        imu.write_register(INT_EN_0, DOUBLE_TAP)?;
        imu.write_register(CMD, CMD_INT_RESET)?;
        delay.delay_ms(10);

        let pmu = imu.read_register(PMU_STATUS)?;
        log::info!(
            "BMI160 power mode: accel={} gyro={} (1 = normal)",
            (pmu >> 4) & 0b11,
            (pmu >> 2) & 0b11
        );

        let mut sum = Vector3D::new(0.0, 0.0, 0.0);
        for _ in 0..CALIBRATION_SAMPLES {
            sum += imu.read_raw()?.1;
            delay.delay_ms(5);
        }
        imu.gyro_bias = sum * (1.0 / CALIBRATION_SAMPLES as f32);
        log::info!(
            "BMI160 gyro offset: ({:.3}, {:.3}, {:.3}) rad/s",
            imu.gyro_bias.x,
            imu.gyro_bias.y,
            imu.gyro_bias.z
        );

        Ok(imu)
    }

    /// Read acceleration and rotation rate.
    pub fn read(&mut self) -> Result<ImuReading, E> {
        let (acceleration, gyro) = self.read_raw()?;
        Ok(ImuReading {
            acceleration,
            angular_velocity: gyro - self.gyro_bias,
        })
    }

    /// Whether the cube was double-tapped since the last call.
    pub fn double_tapped(&mut self) -> Result<bool, E> {
        let tapped = self.read_register(INT_STATUS_0)? & DOUBLE_TAP != 0;
        if tapped {
            self.write_register(CMD, CMD_INT_RESET)?;
        }
        Ok(tapped)
    }

    /// Acceleration in g and uncorrected rotation rate in rad/s, both in cube axes.
    fn read_raw(&mut self) -> Result<(Vector3D, Vector3D), E> {
        let mut data = [0u8; 12];
        self.spi.transaction(&mut [
            Operation::Write(&[0x80 | DATA_GYR_X]),
            Operation::Read(&mut data),
        ])?;
        let value = |i: usize| i16::from_le_bytes([data[2 * i], data[2 * i + 1]]) as f32;

        let gyro = [value(0), value(1), value(2)];
        let accel = [value(3), value(4), value(5)];
        let axis = |raw: &[f32; 3], i: usize| AXIS_MAP[i].1 * raw[AXIS_MAP[i].0];

        let acceleration = Vector3D::new(axis(&accel, 0), axis(&accel, 1), axis(&accel, 2));
        let angular_velocity = Vector3D::new(axis(&gyro, 0), axis(&gyro, 1), axis(&gyro, 2));
        Ok((
            acceleration * ACCEL_SCALE,
            angular_velocity * (GYRO_SIGN * GYRO_SCALE),
        ))
    }

    fn read_register(&mut self, register: u8) -> Result<u8, E> {
        let mut value = [0u8];
        self.spi.transaction(&mut [
            Operation::Write(&[0x80 | register]),
            Operation::Read(&mut value),
        ])?;
        Ok(value[0])
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), E> {
        self.spi.write(&[register & 0x7F, value])
    }
}

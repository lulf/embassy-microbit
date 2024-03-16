//! Motion sensor for the micro:bit.
//!
//! The sensor is an LSM303AGR, a 3D accelerometer and 3D magnetometer combined in a single package.

use embassy_nrf::{
    interrupt::typelevel::{Binding, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0},
    peripherals::{P0_08, P0_16, TWISPI0},
    twim::{self, InterruptHandler},
};
use embassy_sync::channel::DynamicSender;
use embassy_time::{Duration, Ticker};
use lsm303agr::{
    interface::I2cInterface, mode::MagOneShot, AccelMode, AccelOutputDataRate, Acceleration, Error as LsmError,
    Lsm303agr, MagMode, MagOutputDataRate, MagneticField, Status,
};

type I2C<'d> = twim::Twim<'d, TWISPI0>;

/// Accelerometer error
pub type Error = LsmError<twim::Error, ()>;

/// Accelerometer and magnetometer chip present on the microbit
pub struct Sensor<'d> {
    sensor: Lsm303agr<I2cInterface<I2C<'d>>, MagOneShot>,
}

/// Create a new lsm303agr sensor
///
/// As an alternative to the [`Sensor`] struct, you can create a new [`Lsm303agr`] sensor using this
/// function. No initialization is performed, which means you will need to perform initialization
/// and configuration yourself.
///
/// # Examples
///
/// ```no_run
/// use microbit_bsp::{motion, Microbit};
/// use embassy_nrf::{bind_interrupts, peripherals::TWISPI0, twim::InterruptHandler};
///
/// bind_interrupts!(
///     struct InterruptRequests {
///         SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => InterruptHandler<TWISPI0>;
///     }
/// );
/// let irq = InterruptRequests{};
/// let board = Microbit::default();
/// let mut lsm = motion::new_lsm303agr(board.twispi0, irq, board.p23, board.p22);
/// lsm.init().unwrap();
/// lsm
///     .set_accel_mode_and_odr(
///         &mut embassy_time::Delay,
///         AccelMode::Normal,
///         AccelOutputDataRate::Hz10,
///     )
///     .unwrap();
/// lsm
///     .set_mag_mode_and_odr(
///         &mut embassy_time::Delay,
///         lsm303agr::MagMode::HighResolution,
///         lsm303agr::MagOutputDataRate::Hz10,
///     )
///     .unwrap();
/// lsm.mag_enable_low_pass_filter().unwrap();
/// lsm.enable_mag_offset_cancellation().unwrap();
/// ```
pub fn new_lsm303agr<'d>(
    twispi0: TWISPI0,
    irq: impl Binding<SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0, InterruptHandler<TWISPI0>> + 'd,
    sda: P0_16,
    scl: P0_08,
) -> Lsm303agr<I2cInterface<I2C<'d>>, MagOneShot> {
    let config = twim::Config::default();
    let twi = twim::Twim::new(twispi0, irq, sda, scl, config);
    Lsm303agr::new_with_i2c(twi)
}

impl<'d> Sensor<'d> {
    /// Create and initialize the motion sensor
    ///
    /// # Errors
    ///
    /// If there is a problem communicating with the sensor, an error is returned.
    pub fn new(
        twispi0: TWISPI0,
        irq: impl Binding<SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0, InterruptHandler<TWISPI0>> + 'd,
        sda: P0_16,
        scl: P0_08,
    ) -> Result<Self, Error> {
        let mut sensor = new_lsm303agr(twispi0, irq, sda, scl);
        sensor.init()?;
        sensor.set_accel_mode_and_odr(&mut embassy_time::Delay, AccelMode::Normal, AccelOutputDataRate::Hz10)?;

        sensor.set_mag_mode_and_odr(
            &mut embassy_time::Delay,
            MagMode::HighResolution,
            MagOutputDataRate::Hz10,
        )?;
        sensor.mag_enable_low_pass_filter()?;
        sensor.enable_mag_offset_cancellation()?;

        Ok(Self { sensor })
    }

    /// Return status of accelerometer
    ///
    /// # Errors
    ///
    /// If there is a problem communicating with the sensor, an error is returned.
    pub fn accel_status(&mut self) -> Result<Status, Error> {
        self.sensor.accel_status()
    }

    /// Return accelerometer data
    ///
    /// Returned in mg (milli-g) where 1g is 9.8m/s².
    ///
    /// # Errors
    ///
    /// If there is a problem communicating with the sensor, an error is returned.
    pub fn accel_data(&mut self) -> Result<Acceleration, Error> {
        self.sensor.acceleration()
    }

    /// Run a continuous task outputing accelerometer data at the configured data rate
    ///
    /// # Errors
    ///
    /// If there is a problem communicating with the sensor, an error is returned.
    pub async fn accel_run(
        &mut self,
        rate: AccelOutputDataRate,
        sender: DynamicSender<'_, Acceleration>,
    ) -> Result<(), Error> {
        let delay = match rate {
            AccelOutputDataRate::Hz1 => Duration::from_millis(1000),
            AccelOutputDataRate::Hz10 => Duration::from_millis(100),
            AccelOutputDataRate::Hz25 => Duration::from_millis(40),
            AccelOutputDataRate::Hz50 => Duration::from_millis(20),
            AccelOutputDataRate::Hz100 => Duration::from_millis(10),
            AccelOutputDataRate::Hz200 => Duration::from_millis(5),
            AccelOutputDataRate::Hz400 => Duration::from_micros(2500),
            AccelOutputDataRate::Khz1_344 => Duration::from_micros(744),
            AccelOutputDataRate::Khz1_620LowPower => Duration::from_micros(617),
            AccelOutputDataRate::Khz5_376LowPower => Duration::from_micros(186),
        };
        let mut ticker = Ticker::every(delay);
        loop {
            ticker.next().await;
            let data = self.accel_data()?;
            let _ = sender.try_send(data);
        }
    }

    /// Returns data from the magnetometer.
    ///
    /// # Errors
    ///
    /// Returns an error if the magnetometer is not ready to provide data, or if there is an error
    /// communicating with the sensor.
    pub fn mag_data(&mut self) -> nb::Result<MagneticField, Error> {
        self.sensor.magnetic_field()
    }

    /// Returns the status of the magnetometer.
    ///
    /// # Errors
    ///
    /// Returns an error if there is an error communicating with the sensor.
    pub fn mag_status(&mut self) -> Result<Status, Error> {
        self.sensor.mag_status()
    }
}

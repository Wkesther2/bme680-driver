#![no_main]
#![no_std]

//! # BME680 Environmental Sensor Driver
//!
//! A type-safe, `no_std` driver for the Bosch BME680.
//! This driver uses the typestate pattern to ensure the sensor is correctly
//! initialized and configured before measurements are taken.
//!
//! ## Features
//! - **Flexible Configuration**: Individually enable/disable Temperature, Humidity,
//!   Pressure, or Gas measurements to save power.
//! - **Fixed-Point Arithmetic**: No FPU required.
//! - **Typestate Pattern**: Prevents measuring before initialization.
//!
//! ## Units
//! - **Temperature**: Centigrade (C * 100) -> 2350 = 23.50 °C
//! - **Humidity**: Milli-percent (RH % * 1000) -> 45123 = 45.123 %
//! - **Pressure**: Pascal (Pa) -> 101325 = 1013.25 hPa
//! - **Gas Resistance**: Ohms (Ω)

mod calc;

use core::marker::PhantomData;
use embedded_hal::{self, delay::DelayNs, i2c};

/// Memory addresses and sizes for calibration data registers.
mod calib_mem {
    pub const ADDR: [u8; 2] = [0x89, 0xE1];
    pub const SIZES: [usize; 2] = [25, 16];
    pub const TOTAL_SIZE: usize = 25 + 16;
}

/// Memory address and size for the measurement data registers.
mod raw_data_mem {
    pub const ADDR: u8 = 0x1F;
    pub const SIZE: usize = 12;
}

// --- Typestates ---

pub struct Idle;
/// Sensor has been created but not yet initialized with calibration data.
pub struct Uninitialized;
/// Sensor is initialized, configured, and ready for measurements.
pub struct Ready;

/// Error types for the BME680 driver.
pub mod error {
    /// Errors that can occur during communication or configuration.
    #[derive(Debug, Clone, Copy)]
    pub enum Bme680Error<E> {
        /// I2C bus error.
        I2CError(E),
        /// Provided wait time exceeds the 4096ms hardware limit.
        InvalidWaitTime,
        /// Provided profile index is out of bounds (0-9).
        InvalidProfileIndex,
        /// Sensor measurement timed out.
        Timeout,
        /// Gas heating plate has not reached a stable temperature.
        HeaterNotStable,
        /// Gas measurement data is not yet valid.
        GasDataNotReady,
    }

    /// Result type alias for BME680 operations.
    pub type Result<T, E> = core::result::Result<T, Bme680Error<E>>;
}

/// Oversampling settings for Temperature, Pressure, and Humidity.
///
/// Higher oversampling rates increase accuracy (reduce noise) but lead to
/// longer measurement times and higher power consumption.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Oversampling {
    /// No measurement performed. Used to disable a specific sensor.
    Skipped = 0,
    /// 1x Oversampling.
    X1 = 1,
    /// 2x Oversampling.
    X2 = 2,
    /// 4x Oversampling.
    X4 = 3,
    /// 8x Oversampling.
    X8 = 4,
    /// 16x Oversampling.
    X16 = 5,
}

impl Oversampling {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Oversampling::Skipped,
            1 => Oversampling::X1,
            2 => Oversampling::X2,
            3 => Oversampling::X4,
            4 => Oversampling::X8,
            5 => Oversampling::X16,
            _ => panic!("Invalid Oversampling Value"),
        }
    }
}

/// Infinite Impulse Response (IIR) filter coefficient.
///
/// Used to filter short-term disturbances (noise) in pressure and temperature.
/// Does not affect humidity or gas measurements.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum IIRFilter {
    IIR0 = 0,
    IIR1 = 1,
    IIR3 = 2,
    IIR7 = 3,
    IIR15 = 4,
    IIR31 = 5,
    IIR63 = 6,
    IIR127 = 7,
}

/// Available heating profile slots (0 to 9) stored in the sensor.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum GasProfileIndex {
    Profile0 = 0,
    Profile1 = 1,
    Profile2 = 2,
    Profile3 = 3,
    Profile4 = 4,
    Profile5 = 5,
    Profile6 = 6,
    Profile7 = 7,
    Profile8 = 8,
    Profile9 = 9,
}

/// Temperature wrapper for type-safety.
/// Value is stored in Centigrade * 100 (e.g., 2350 = 23.50 °C).
#[derive(Clone, Copy)]
pub struct Celsius(pub i32);

/// Duration wrapper for type-safety. Stored in milliseconds.
#[derive(Debug, Clone, Copy)]
pub struct Milliseconds(pub u32);

/// Configuration for the gas sensor heating plate.
#[derive(Clone, Copy)]
pub struct GasProfile {
    /// Slot index in the sensor memory where this profile is stored.
    pub index: GasProfileIndex,
    /// Target temperature in Celsius.
    pub target_temp: Celsius,
    /// Duration to maintain the temperature before measurement.
    pub wait_time: Milliseconds,
}

/// Grouped oversampling settings for all three environmental sensors.
///
/// Use `Oversampling::Skipped` to disable specific measurements.
pub struct OversamplingConfig {
    /// Temperature oversampling.
    pub temp_osrs: Oversampling,
    /// Humidity oversampling.
    pub hum_osrs: Oversampling,
    /// Pressure oversampling.
    pub pres_osrs: Oversampling,
}

impl OversamplingConfig {
    /// Returns `true` if all measurements are set to `Skipped`.
    ///
    /// This is used internally to determine if a forced measurement command
    /// needs to be sent or if the sensor should remain idle.
    pub fn is_all_skipped(&self) -> bool {
        self.temp_osrs == Oversampling::Skipped
            && self.hum_osrs == Oversampling::Skipped
            && self.pres_osrs == Oversampling::Skipped
    }
}

/// Internal struct to track which sensors are enabled for the current measurement cycle.
struct MeasConfig {
    osrs_config: OversamplingConfig,
    gas_enabled: bool,
}

/// Complete sensor configuration used for setup.
pub struct Config {
    /// Oversampling settings for T, P, H.
    pub osrs_config: OversamplingConfig,
    /// IIR Filter settings.
    pub iir_filter: IIRFilter,
    /// Gas heater configuration.
    /// Set to `None` to disable gas measurement entirely (saves significant power).
    pub gas_profile: Option<GasProfile>,
    /// Current ambient temperature estimate (required for heater resistance calculation).
    pub ambient_temp: Celsius,
}

/// Factory-fused calibration coefficients read from the sensor.
/// These are unique to every individual chip and required for compensation formulas.
#[derive(Debug, Default, Copy, Clone)]
pub struct CalibData {
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    pub par_g1: i8,
    pub par_g2: i16,
    pub par_g3: i8,
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    pub res_heat_range: u8,
    pub res_heat_val: u8,
    pub range_sw_err: i8,
}

/// Raw ADC output and status bits read directly from the sensor registers.
///
/// This struct holds the uncompensated data. It is used internally by the driver
/// to calculate the final physical values using the calibration parameters.
#[derive(Debug, Copy, Clone)]
pub struct RawData {
    pub(crate) temp_adc: u32,
    pub(crate) hum_adc: u16,
    pub(crate) press_adc: u32,
    pub(crate) gas_adc: u16,
    /// Range switching error used for gas calculation.
    pub(crate) gas_range: u8,
    /// Indicates if the gas measurement is valid.
    pub(crate) gas_valid_r: bool,
    /// Indicates if the target heater temperature was reached.
    pub(crate) heat_stab_r: bool,
}

/// Intermediate temperature values used for compensation.
///
/// These values are calculated during temperature compensation and are required
/// for the subsequent pressure and humidity compensation formulas (t_fine).
#[derive(Debug, Copy, Clone, Default)]
pub struct CalcTempData {
    pub(crate) temp_fine: i32,
    pub(crate) temp_comp: i32,
}

/// Represents temperature in Centigrade (degrees Celsius * 100).
///
/// This wrapper ensures type safety and prevents mixing units.
/// Use the `.split()` method to easily format this for display.
///
/// # Example
/// A value of `2350` represents **23.50 °C**.
#[derive(Debug, Copy, Clone, Default)]
pub struct Temperature(pub i32);

impl Temperature {
    /// Splits the fixed-point value into integral (degrees) and fractional (decimals) parts.
    ///
    /// # Returns
    /// A tuple `(whole, fraction)`.
    ///
    /// # Example
    /// ```rust
    /// use bme680_driver::Temperature;
    /// let temp = Temperature(2350);
    /// assert_eq!(temp.split(), (23, 50)); // Represents 23.50 °C
    /// ```
    pub fn split(&self) -> (i32, i32) {
        (self.0 / 100, self.0 % 100)
    }
}

/// Represents relative humidity in milli-percent (percent * 1000).
///
/// # Example
/// A value of `45123` represents **45.123 %rH**.
#[derive(Debug, Copy, Clone, Default)]
pub struct Humidity(pub i32);

impl Humidity {
    /// Splits the fixed-point value into integral and fractional parts.
    ///
    /// # Returns
    /// A tuple `(whole, fraction)`. The fraction represents 3 decimal places.
    ///
    /// # Example
    /// ```rust
    /// use bme680_driver::Humidity;
    /// let hum = Humidity(45123);
    /// assert_eq!(hum.split(), (45, 123)); // Represents 45.123 %
    /// ```
    pub fn split(&self) -> (i32, i32) {
        (self.0 / 1000, self.0 % 1000)
    }
}

/// Represents atmospheric pressure in Pascal (Pa).
///
/// # Example
/// A value of `101325` represents **101325 Pa** (or 1013.25 hPa).
#[derive(Debug, Copy, Clone, Default)]
pub struct Pressure(pub u32);

impl Pressure {
    /// Converts the raw Pascal value to Hectopascal (hPa) and splits it into parts.
    ///
    /// Since 1 hPa = 100 Pa, this effectively splits the integer at the hundreds place.
    ///
    /// # Returns
    /// A tuple `(hPa_integral, hPa_decimal)`.
    ///
    /// # Example
    /// ```rust
    /// use bme680_driver::Pressure;
    /// let press = Pressure(101325);
    /// assert_eq!(press.as_hpa(), (1013, 25)); // Represents 1013.25 hPa
    /// ```
    pub fn as_hpa(&self) -> (u32, u32) {
        (self.0 / 100, self.0 % 100)
    }
}

/// Represents gas resistance in Ohms (Ω).
///
/// A higher gas resistance typically indicates cleaner air (fewer VOCs).
#[derive(Debug, Copy, Clone, Default)]
pub struct Gas(pub u32);

/// Compensated measurement result in physical units.
///
/// All fields use strong types (`Temperature`, `Humidity`, etc.) to prevent unit confusion.
/// If a measurement was skipped/disabled, the corresponding field may contain 0 or a stale value.
#[derive(Debug, Copy, Clone, Default)]
pub struct Measurement {
    /// Temperature data.
    pub temp: Temperature,
    /// Humidity data.
    pub hum: Humidity,
    /// Atmospheric pressure data.
    pub pres: Pressure,
    /// Gas resistance data.
    pub gas: Gas,
}

/// The main BME680 driver structure.
///
/// Use `Bme680::new(...)` to start. The `STATE` generic uses the Typestate pattern
/// to track initialization status at compile time.
#[derive(Debug, Copy, Clone)]
pub struct Bme680<I2C, STATE> {
    i2c: I2C,
    address: u8,
    pub(crate) calib_data: CalibData,
    /// Tracks the calculated wait time required for the current configuration.
    current_wait_time: Milliseconds,
    _state: PhantomData<STATE>,
}

impl<I2C, E> Bme680<I2C, Idle>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Creates a new driver instance in the `Uninitialized` state.
    ///
    /// This does not communicate with the sensor yet.
    ///
    /// # Arguments
    /// * `i2c` - The I2C bus object.
    /// * `address` - The I2C address of the sensor (typically `0x76` or `0x77`).
    pub fn new(i2c: I2C, address: u8) -> Bme680<I2C, Uninitialized> {
        Bme680 {
            i2c,
            address,
            calib_data: CalibData::default(),
            current_wait_time: Milliseconds(0),
            _state: PhantomData,
        }
    }
}

impl<I2C, STATE, E> Bme680<I2C, STATE>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Performs a soft-reset of the sensor.
    ///
    /// This resets all internal registers to their default values.
    /// A delay of at least 2ms is required after the reset command.
    fn reset(&mut self, delay: &mut impl DelayNs) -> error::Result<(), E> {
        self.i2c
            .write(self.address, &[0xE0, 0xB6])
            .map_err(|e| error::Bme680Error::I2CError(e))?;

        delay.delay_ms(2);

        Ok(())
    }

    /// Reads data from a starting register address into a provided buffer.
    ///
    /// This is a low-level helper function for I2C communication.
    fn read_into(&mut self, reg_address: u8, buffer: &mut [u8]) -> error::Result<(), E> {
        self.i2c
            .write_read(self.address, &[reg_address], buffer)
            .map_err(|e| error::Bme680Error::I2CError(e))
    }

    /// Reads a single byte from a specific register address.
    fn read_reg_byte(&mut self, reg_address: u8) -> error::Result<u8, E> {
        let mut buffer = [0];

        self.i2c
            .write_read(self.address, &[reg_address], &mut buffer)
            .map_err(|e| error::Bme680Error::I2CError(e))?;

        Ok(buffer[0])
    }

    /// Writes a byte slice (typically `[Register, Value]`) to the sensor.
    fn write_reg(&mut self, data: &[u8]) -> error::Result<(), E> {
        self.i2c
            .write(self.address, data)
            .map_err(|e| error::Bme680Error::I2CError(e))?;
        Ok(())
    }
}

impl<I2C, E> Bme680<I2C, Uninitialized>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Initializes the sensor: performs a soft-reset and loads factory calibration data.
    ///
    /// This transitions the driver state from `Uninitialized` to `Ready`.
    ///
    /// # Errors
    /// Returns an error if the I2C communication fails during reset or calibration reading.
    pub fn init(mut self, delay: &mut impl DelayNs) -> error::Result<Bme680<I2C, Ready>, E> {
        // Sensor requires time to start up before reset
        delay.delay_ms(2);

        self.reset(delay)?;

        // Read the factory calibration data (requires ~25ms I2C traffic)
        let calib_data = self.get_calib_data()?;

        Ok(Bme680 {
            i2c: self.i2c,
            address: self.address,
            calib_data: calib_data,
            current_wait_time: Milliseconds(0),
            _state: PhantomData,
        })
    }

    /// Reads factory-fused calibration coefficients from the sensor's ROM.
    ///
    /// The BME680 stores calibration data in two non-contiguous memory blocks.
    /// These bytes are required to compensate the raw ADC values into physical units.
    fn get_calib_data(&mut self) -> error::Result<CalibData, E> {
        let mut calib_data = CalibData::default();
        let mut buffer = [0u8; calib_mem::TOTAL_SIZE];

        // 1. Read first block (0x89..0xA0)
        self.read_into(calib_mem::ADDR[0], &mut buffer[0..calib_mem::SIZES[0]])?;
        // 2. Read second block (0xE1..0xF0)
        self.read_into(calib_mem::ADDR[1], &mut buffer[calib_mem::SIZES[0]..])?;

        // Mapping raw buffer bytes to compensation parameters (Bosch proprietary logic)
        // See BME680 datasheet, Section 3.11.1
        calib_data.par_t1 = ((buffer[33] as i32) | ((buffer[34] as i32) << 8)) as u16;
        calib_data.par_t2 = ((buffer[1] as i32) | ((buffer[2] as i32) << 8)) as i16;
        calib_data.par_t3 = buffer[3] as i8;
        calib_data.par_p1 = ((buffer[5] as i32) | ((buffer[6] as i32) << 8)) as u16;
        calib_data.par_p2 = ((buffer[7] as i32) | ((buffer[8] as i32) << 8)) as i16;
        calib_data.par_p3 = buffer[9] as i8;
        calib_data.par_p4 = ((buffer[11] as i32) | ((buffer[12] as i32) << 8)) as i16;
        calib_data.par_p5 = ((buffer[14] as i32) | ((buffer[13] as i32) << 8)) as i16;
        calib_data.par_p6 = buffer[16] as i8;
        calib_data.par_p7 = buffer[15] as i8;
        calib_data.par_p8 = ((buffer[19] as i32) | ((buffer[20] as i32) << 8)) as i16;
        calib_data.par_p9 = ((buffer[21] as i32) | ((buffer[22] as i32) << 8)) as i16;
        calib_data.par_p10 = buffer[23];
        calib_data.par_h1 = (((buffer[26] & 0x0F) as i32) | ((buffer[27] as i32) << 4)) as u16;
        calib_data.par_h2 = (((buffer[26] >> 4) as i32) | ((buffer[25] as i32) << 4)) as u16;
        calib_data.par_h3 = buffer[28] as i8;
        calib_data.par_h4 = buffer[29] as i8;
        calib_data.par_h5 = buffer[30] as i8;
        calib_data.par_h6 = buffer[31];
        calib_data.par_h7 = buffer[32] as i8;
        calib_data.par_g1 = buffer[37] as i8;
        calib_data.par_g2 = ((buffer[35] as i32) | ((buffer[36] as i32) << 8)) as i16;
        calib_data.par_g3 = buffer[38] as i8;

        // Additional heater-specific calibration values
        calib_data.res_heat_val = self.read_reg_byte(0x00)?;
        calib_data.res_heat_range = (self.read_reg_byte(0x02)? >> 4) & 0x03;
        calib_data.range_sw_err = (self.read_reg_byte(0x04)? as i8) >> 4;

        Ok(calib_data)
    }
}

impl<I2C, E> Bme680<I2C, Ready>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Applies a full sensor configuration.
    ///
    /// This method sets oversampling, filters, and gas profiles.
    /// If `config.gas_profile` is `None`, the gas sensor is disabled to save power.
    pub fn configure_sensor(&mut self, config: &mut Config) -> error::Result<(), E> {
        self.config_oversampling(&config.osrs_config)?;
        self.config_iir_filter(config.iir_filter)?;

        if let Some(x) = config.gas_profile {
            self.enable_gas_measurement()?;
            self.select_gas_profile(&x)?;
            self.set_gas_heater_profile(x, config.ambient_temp)?;
        } else {
            self.disable_gas_measurement()?;
        }

        Ok(())
    }

    /// Configures heating duration and target temperature for a gas profile.
    ///
    /// This calculates the necessary register value based on the current ambient temperature.
    pub fn set_gas_heater_profile(
        &mut self,
        config: GasProfile,
        amb_temp: Celsius,
    ) -> error::Result<(), E> {
        self.config_heater_on_time(config.wait_time, config.index)?;
        self.config_target_resistance(config.target_temp, amb_temp, config.index)?;

        Ok(())
    }

    /// Triggers a measurement in 'Forced Mode', waits for completion, and returns compensated data.
    ///
    ///
    ///
    /// # Power Saving
    /// If all measurements are set to `Skipped` and gas is disabled, this function
    /// returns immediately with default values, consuming minimal power.
    pub fn read_new_data(&mut self, delay: &mut impl DelayNs) -> error::Result<Measurement, E> {
        // Read config back from sensor to ensure we don't wait unnecessarily
        let meas_config = self.get_meas_config()?;

        // Optimization: Don't trigger a measurement if nothing is enabled.
        if meas_config.gas_enabled == false && meas_config.osrs_config.is_all_skipped() {
            return Ok(Measurement::default());
        }

        // 1. Wake up sensor and start measurement cycle
        self.activate_forced_mode()?;

        // 2. Wait for heating phase (if gas is enabled)
        // The sensor measures T, P, H first, then heats up for gas measurement.
        if meas_config.gas_enabled {
            delay.delay_ms(self.current_wait_time.0);
        }

        // 3. Poll for "New Data" bit and read ADC values
        let raw_data = self.get_raw_data(delay)?;

        let mut temp = CalcTempData::default();
        let mut hum = 0;
        let mut pres = 0;
        let mut gas = 0;

        // 4. Apply mathematical compensation to raw values (if not skipped)
        if meas_config.osrs_config.temp_osrs != Oversampling::Skipped {
            temp = self.calc_temp(raw_data.temp_adc);

            // Humidity and Pressure compensation depends on "fine temperature"
            if meas_config.osrs_config.hum_osrs != Oversampling::Skipped {
                hum = self.calc_hum(temp.temp_comp, raw_data.hum_adc);
            }

            if meas_config.osrs_config.pres_osrs != Oversampling::Skipped {
                pres = self.calc_pres(temp.temp_fine, raw_data.press_adc);
            }
        }

        // 5. Check gas validity bits
        if meas_config.gas_enabled && !raw_data.gas_valid_r {
            return Err(error::Bme680Error::GasDataNotReady);
        } else if meas_config.gas_enabled && !raw_data.heat_stab_r {
            return Err(error::Bme680Error::HeaterNotStable);
        }

        if meas_config.gas_enabled {
            gas = self.calc_gas(raw_data.gas_adc, raw_data.gas_range);
        }

        Ok(Measurement {
            temp: Temperature(temp.temp_comp),
            hum: Humidity(hum),
            pres: Pressure(pres),
            gas: Gas(gas),
        })
    }

    /// Reads the Chip ID from the sensor (expected value: 0x61).
    pub fn read_chip_id(&mut self) -> error::Result<u8, E> {
        Ok(self.read_reg_byte(0xD0)?)
    }

    /// Selects one of the 10 available gas heater profiles.
    pub fn select_gas_profile(&mut self, profile: &GasProfile) -> error::Result<(), E> {
        self.current_wait_time = profile.wait_time;
        let register = self.read_reg_byte(0x71)?;
        // Update bits [3:0] (nb_conv)
        self.write_reg(&[0x71, (register & 0xF0) | (profile.index as u8)])?;
        Ok(())
    }

    /// Polls the sensor until new data is available and reads all ADC values.
    ///
    /// This method includes a timeout mechanism (approx. 5ms).
    fn get_raw_data(&mut self, delay: &mut impl DelayNs) -> error::Result<RawData, E> {
        let mut new_data = false;
        let mut timeout_us = 5000; // 5ms Timeout

        while !new_data {
            if timeout_us <= 0 {
                return Err(error::Bme680Error::Timeout);
            }
            // Check bit 7 in register 0x1D (new_data_0)
            new_data = (self.read_reg_byte(0x1D)? >> 7) != 0;

            delay.delay_us(500);
            timeout_us -= 500;
        }

        let mut buffer = [0u8; raw_data_mem::SIZE + 1];

        // Burst read starting from 0x1F (pressure MSB)
        self.i2c
            .write_read(self.address, &[raw_data_mem::ADDR], &mut buffer)
            .map_err(|e| error::Bme680Error::I2CError(e))?;

        // Reconstruct 20-bit and 16-bit ADC values from register bytes
        let press_adc =
            ((buffer[2] as u32) >> 4) | ((buffer[1] as u32) << 4) | ((buffer[0] as u32) << 12);
        let temp_adc =
            ((buffer[5] as u32) >> 4) | ((buffer[4] as u32) << 4) | ((buffer[3] as u32) << 12);
        let hum_adc = ((buffer[7] as u32) | ((buffer[6] as u32) << 8)) as u16;
        let gas_adc = (((buffer[12] as u32) >> 6) | ((buffer[11] as u32) << 2)) as u16;
        let gas_range = buffer[12] & 0x0F;

        // Status bits for gas measurement
        let gas_valid_r = ((buffer[12] >> 5) & 0x1) != 0;
        let heat_stab_r = ((buffer[12] >> 4) & 0x1) != 0;

        Ok(RawData {
            temp_adc,
            hum_adc,
            press_adc,
            gas_adc,
            gas_range,
            gas_valid_r,
            heat_stab_r,
        })
    }

    /// Sets oversampling rates for Humidity, Temperature, and Pressure.
    ///
    /// Writes to registers `ctrl_hum` (0x72) and `ctrl_meas` (0x74).
    fn config_oversampling(&mut self, osrs_config: &OversamplingConfig) -> error::Result<(), E> {
        // Humidity configuration (Register 0x72)
        let register = self.read_reg_byte(0x72)?;
        let mut new_reg_val = (register & 0xF8) | osrs_config.hum_osrs as u8;
        self.write_reg(&[0x72, new_reg_val])?;

        // Temperature & Pressure configuration (Register 0x74)
        let register = self.read_reg_byte(0x74)?;
        let temp_pres_combined =
            ((osrs_config.temp_osrs as u8) << 0x5) | ((osrs_config.pres_osrs as u8) << 0x2);
        new_reg_val = (register & 0x03) | temp_pres_combined;
        self.write_reg(&[0x74, new_reg_val])?;

        Ok(())
    }

    /// Configures the IIR filter coefficient.
    fn config_iir_filter(&mut self, iir_filter: IIRFilter) -> error::Result<(), E> {
        let register = self.read_reg_byte(0x75)?;
        let new_reg_val = (register & 0xE3) | ((iir_filter as u8) << 0x2);
        self.write_reg(&[0x75, new_reg_val])?;
        Ok(())
    }

    /// Enables the gas sensing functionality in the sensor (ctrl_gas_1).
    fn enable_gas_measurement(&mut self) -> error::Result<(), E> {
        let register = self.read_reg_byte(0x71)?;
        // Bit 4: run_gas
        self.write_reg(&[0x71, (register & 0xEF) | (0b1 << 4)])?;
        Ok(())
    }

    /// Disables the gas sensing functionality in the sensor.
    fn disable_gas_measurement(&mut self) -> error::Result<(), E> {
        let register = self.read_reg_byte(0x71)?;
        self.write_reg(&[0x71, register & 0xEF])?;
        Ok(())
    }

    /// Activates 'Forced Mode' to trigger a single measurement cycle.
    ///
    /// The sensor returns to Sleep mode automatically after the measurement.
    fn activate_forced_mode(&mut self) -> error::Result<(), E> {
        let register = self.read_reg_byte(0x74)?;
        // Mode 01: Forced mode
        self.write_reg(&[0x74, (register & 0xFC) | 0b01])?;
        Ok(())
    }

    /// Reads the current configuration back from the sensor.
    ///
    /// Used internally to verify which sensors are enabled before waiting/reading.
    fn get_meas_config(&mut self) -> error::Result<MeasConfig, E> {
        let mut buffer = [0u8; 4];

        // Burst read starting from 0x71 (ctrl_gas_1)
        self.read_into(0x71, &mut buffer)?;

        let gas_enabled = ((buffer[0] >> 4) & 0x1) != 0;
        let osrs_h = buffer[1] & 0x7;
        let osrs_p = (buffer[3] >> 2) & 0x3;
        let osrs_t = (buffer[3] >> 5) & 0x3;

        let osrs_config = OversamplingConfig {
            temp_osrs: Oversampling::from_u8(osrs_t),
            hum_osrs: Oversampling::from_u8(osrs_h),
            pres_osrs: Oversampling::from_u8(osrs_p),
        };

        Ok(MeasConfig {
            osrs_config,
            gas_enabled,
        })
    }
}

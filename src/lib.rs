#![no_main]
#![no_std]

//! # BME680 Environmental Sensor Driver
//!
//! A type-safe, `no_std` driver for the Bosch BME680.
//! This driver uses the typestate pattern to ensure the sensor is correctly
//! initialized and configured before measurements are taken.
//!
//! ## Units
//! - **Temperature**: Centigrade (C * 100)
//! - **Humidity**: Milli-percent (RH % * 1000)
//! - **Pressure**: Pascal (Pa)
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
/// Higher oversampling rates increase accuracy but lead to longer measurement times.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Oversampling {
    /// No measurement performed.
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

/// Infinite Impulse Response (IIR) filter coefficient.
///
/// Used to filter short-term disturbances in pressure and temperature.
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

/// Available heating profile slots (0 to 9).
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
/// Value is stored in Centigrade (e.g., 2350 = 23.50 °C).
#[derive(Clone, Copy)]
pub struct Celsius(pub i32);

/// Duration wrapper for type-safety. Stored in milliseconds.
#[derive(Debug, Clone, Copy)]
pub struct Milliseconds(pub u32);

/// Configuration for the gas sensor heating plate.
#[derive(Clone, Copy)]
pub struct GasProfile {
    /// Slot index in the sensor memory.
    pub index: GasProfileIndex,
    /// Target temperature in Celsius.
    pub target_temp: Celsius,
    /// Duration to maintain the temperature before measurement.
    pub wait_time: Milliseconds,
}

/// Grouped oversampling settings for all three environmental sensors.
pub struct OversamplingConfig {
    pub temp_osrs: Oversampling,
    pub hum_osrs: Oversampling,
    pub pres_osrs: Oversampling,
}

/// Complete sensor configuration used for setup.
pub struct Config {
    pub osrs_config: OversamplingConfig,
    pub iir_filter: IIRFilter,
    pub gas_profile: GasProfile,
    /// Baseline temperature for heater resistance calculation.
    pub ambient_temp: Celsius,
}

/// Factory-fused calibration coefficients read from the sensor.
/// These are unique to every individual chip.
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

/// Raw ADC output and status bits read directly from the sensor.
#[derive(Debug, Copy, Clone)]
pub struct RawData {
    pub(crate) temp_adc: u32,
    pub(crate) hum_adc: u16,
    pub(crate) press_adc: u32,
    pub(crate) gas_adc: u16,
    pub(crate) gas_range: u8,
    pub(crate) gas_valid_r: bool,
    pub(crate) heat_stab_r: bool,
}

/// Intermediate temperature values used for compensation.
#[derive(Debug, Copy, Clone)]
pub struct CalcTempData {
    pub(crate) temp_fine: i32,
    pub(crate) temp_comp: i32,
}

/// Represents temperature in Centigrade (degrees Celsius * 100).
///
/// Use the `.split()` method to easily format this for display.
///
/// # Example
/// A value of `2350` represents **23.50 °C**.
#[derive(Debug, Copy, Clone)]
pub struct Temperature(pub i32);

impl Temperature {
    /// Splits the fixed-point value into integral (degrees) and fractional (decimals) parts.
    ///
    /// # Returns
    /// A tuple `(whole, fraction)`.
    ///
    /// # Example
    /// ```rust
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
#[derive(Debug, Copy, Clone)]
pub struct Humidity(pub i32);

impl Humidity {
    /// Splits the fixed-point value into integral and fractional parts.
    ///
    /// # Returns
    /// A tuple `(whole, fraction)`. The fraction represents 3 decimal places.
    ///
    /// # Example
    /// ```rust
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
#[derive(Debug, Copy, Clone)]
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
    /// let press = Pressure(101325);
    /// assert_eq!(press.as_hpa(), (1013, 25)); // Represents 1013.25 hPa
    /// ```
    pub fn as_hpa(&self) -> (u32, u32) {
        (self.0 / 100, self.0 % 100)
    }
}

/// Represents gas resistance in Ohms (Ω).
#[derive(Debug, Copy, Clone)]
pub struct Gas(pub u32);

/// Compensated measurement result in physical units.
///
/// All fields use strong types (`Temperature`, `Humidity`, etc.) to prevent unit confusion.
#[derive(Debug, Copy, Clone)]
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
/// Use `Bme680::new(...)` to start. The `STATE` generic tracks initialization.
#[derive(Debug, Copy, Clone)]
pub struct Bme680<I2C, STATE> {
    i2c: I2C,
    address: u8,
    pub(crate) calib_data: CalibData,
    current_wait_time: Milliseconds,
    _state: PhantomData<STATE>,
}

impl<I2C, E> Bme680<I2C, Idle>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Creates a new driver instance in the `Uninitialized` state.
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

    /// Writes a byte slice (typically [Register, Value]) to the sensor.
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
    pub fn init(mut self, delay: &mut impl DelayNs) -> error::Result<Bme680<I2C, Ready>, E> {
        delay.delay_ms(2);

        self.reset(delay)?;

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
    fn get_calib_data(&mut self) -> error::Result<CalibData, E> {
        let mut calib_data = CalibData::default();
        let mut buffer = [0u8; calib_mem::TOTAL_SIZE];

        // Calibration data is stored in two non-contiguous memory blocks
        self.read_into(calib_mem::ADDR[0], &mut buffer[0..calib_mem::SIZES[0]])?;
        self.read_into(calib_mem::ADDR[1], &mut buffer[calib_mem::SIZES[0]..])?;

        // Mapping raw buffer bytes to compensation parameters (Bosch proprietary logic)
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
    pub fn configure_sensor(&mut self, config: &mut Config) -> error::Result<(), E> {
        self.config_oversampling(&config.osrs_config)?;
        self.config_iir_filter(config.iir_filter)?;
        self.enable_gas_measurement()?;
        self.select_gas_profile(&config.gas_profile)?;
        self.set_gas_heater_profile(config.gas_profile, config.ambient_temp)?;

        Ok(())
    }

    /// Configures heating duration and target temperature for a gas profile.
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
    pub fn read_new_data(&mut self, delay: &mut impl DelayNs) -> error::Result<Measurement, E> {
        self.activate_forced_mode()?;

        // Wait for the specified heater time before attempting to read raw data
        delay.delay_ms(self.current_wait_time.0);

        let raw_data = self.get_raw_data(delay)?;

        // Apply mathematical compensation to raw values
        let temp = self.calc_temp(raw_data.temp_adc);
        let hum = self.calc_hum(temp.temp_comp, raw_data.hum_adc);
        let pres = self.calc_pres(temp.temp_fine, raw_data.press_adc);

        if !raw_data.gas_valid_r {
            return Err(error::Bme680Error::GasDataNotReady);
        } else if !raw_data.heat_stab_r {
            return Err(error::Bme680Error::HeaterNotStable);
        }

        let gas = self.calc_gas(raw_data.gas_adc, raw_data.gas_range);

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

    /// Polls the sensor until new data is available and reads all ADC values.
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

    /// Enables the gas sensing functionality in the sensor.
    fn enable_gas_measurement(&mut self) -> error::Result<(), E> {
        let register = self.read_reg_byte(0x71)?;
        self.write_reg(&[0x71, (register & 0xEF) | (0b1 << 0x4)])?;
        Ok(())
    }

    /// Selects one of the 10 available gas heater profiles.
    pub fn select_gas_profile(&mut self, profile: &GasProfile) -> error::Result<(), E> {
        self.current_wait_time = profile.wait_time;
        let register = self.read_reg_byte(0x71)?;
        self.write_reg(&[0x71, (register & 0xF0) | (profile.index as u8)])?;
        Ok(())
    }

    /// Activates 'Forced Mode' to trigger a single measurement cycle.
    fn activate_forced_mode(&mut self) -> error::Result<(), E> {
        let register = self.read_reg_byte(0x74)?;
        self.write_reg(&[0x74, (register & 0xFC) | 0b01])?;
        Ok(())
    }
}

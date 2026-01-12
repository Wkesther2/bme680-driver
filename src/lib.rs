#![no_main]
#![no_std]

//! # BME680 Environmental Sensor Driver
//!
//! A type-safe, `no_std` driver for the Bosch BME680 environmental sensor.
//! This driver is designed for high-reliability embedded applications where
//! floating-point arithmetic is either unavailable or too slow.
//!
//! ## Key Features
//!
//! - **Typestate Architecture**: The driver uses a compile-time state machine (`Uninitialized` -> `Ready`)
//!   to ensure methods like `read_new_data` cannot be called before the sensor is properly set up.
//! - **Fixed-Point Arithmetic**: All calculations (Temperature, Pressure, Humidity, Gas) are performed
//!   using integer arithmetic. This ensures deterministic behavior and high performance on Cortex-M0/M3/M4 cores.
//! - **Power Efficiency**: The driver supports granular control over which measurements are enabled
//!   (e.g., disabling Gas or Pressure sensing to save energy).
//!
//! ## Data Units
//!
//! To avoid floats, this driver uses scaled integer units:
//!
//! - **Temperature**: Centigrade (`°C * 100`). Example: `2350` = 23.50 °C.
//! - **Humidity**: Milli-percent (`%rH * 1000`). Example: `45123` = 45.123 %rH.
//! - **Pressure**: Pascal (`Pa`). Example: `101325` = 1013.25 hPa.
//! - **Gas Resistance**: Ohms (`Ω`). Higher values typically indicate cleaner air.

mod calc;
pub mod settings;

use core::marker::PhantomData;
use embedded_hal::{self, delay::DelayNs, i2c};

pub use settings::{BME680Builder, Config, GasProfile, GasProfileIndex, IIRFilter, Oversampling};

/// Internal register addresses for the BME680.
///
/// These addresses are derived from the Bosch BME680 Datasheet.
pub(crate) mod regs {
    pub const ADDR_RES_HEAT_VAL: u8 = 0x00;
    pub const ADDR_RES_HEAT_RANGE: u8 = 0x02;
    pub const ADDR_RANGE_SW_ERR: u8 = 0x04;
    /// Status register containing the "New Data" bit (Bit 7) and "Measuring" bit (Bit 6).
    pub const ADDR_EAS_STATUS_0: u8 = 0x1D;
    /// Start of the measurement data registers (Pressure MSB).
    pub const ADDR_PRESS_MSB: u8 = 0x1F;
    pub const ADDR_RES_HEAT_0: u8 = 0x5A;
    pub const ADDR_GAS_WAIT_0: u8 = 0x64;
    /// Ctrl Gas 1: Controls RUN_GAS (Bit 4) and NB_CONV (Bits 0-3).
    pub const ADDR_CTRL_GAS_1: u8 = 0x71;
    /// Ctrl Hum: Controls Humidity oversampling (Bits 0-2).
    pub const ADDR_CTRL_HUM: u8 = 0x72;
    /// Ctrl Meas: Controls Temp/Pres oversampling and Mode selection (Sleep/Forced).
    pub const ADDR_CTRL_MEAS: u8 = 0x74;
    /// Config: IIR Filter settings and SPI 3-wire selection.
    pub const ADDR_CONFIG: u8 = 0x75;
    /// Start of the first calibration data block (0x89 - 0xA1).
    pub const ADDR_CALIB_0: u8 = 0x89;
    /// Chip ID register (should read 0x61).
    pub const ADDR_ID: u8 = 0xD0;
    /// Soft Reset register (write 0xB6 to reset).
    pub const ADDR_RESET: u8 = 0xE0;
    /// Start of the second calibration data block (0xE1 - 0xF0).
    pub const ADDR_CALIB_1: u8 = 0xE1;
}

/// Sizes of various data blocks in memory for burst reads.
mod reg_sizes {
    pub const SIZE_CALIB_0: usize = 25;
    pub const SIZE_CALIB_1: usize = 16;
    pub const SIZE_CALIB_TOTAL: usize = SIZE_CALIB_0 + SIZE_CALIB_1;
    pub const SIZE_RAW_DATA: usize = 13;
}

/// Bit masks for register configuration and bitwise operations.
mod msks {
    /// Command to trigger a soft reset.
    pub const MSK_RESET: u8 = 0xB6;
    pub const MSK_RES_HEAT_RANGE: u8 = 0x30;
    pub const MSK_NB_CONV: u8 = 0xF;
    pub const MSK_PAR_H1_LSB: u8 = 0xF;
    pub const MSK_GAS_RANGE: u8 = 0x0F;
    pub const MSK_OSRS_P: u8 = 0x1C;
    pub const MSK_OSRS_T: u8 = 0xE0;
    pub const MSK_OSRS_H: u8 = 0x3;
    pub const MSK_RUN_GAS: u8 = 0x10;
    pub const MSK_MODE: u8 = 0x3;
    pub const MSK_FILTER: u8 = 0x1C;
}

// --- Typestates ---

/// Initial state of the driver. No logical connection established yet.
pub struct Idle;
/// Sensor instance created, but hardware not yet initialized (calibration data missing).
pub struct Uninitialized;
/// Sensor is fully initialized, calibrated, and ready for measurements.
pub struct Ready;

/// Error types for the BME680 driver.
pub mod error {
    /// Errors that can occur during communication, configuration, or measurement.
    #[derive(Debug, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Bme680Error {
        /// Underlying I2C bus error.
        I2CError,
        /// Provided wait time exceeds the 4096ms hardware limit of the sensor.
        InvalidWaitTime,
        /// Provided profile index is out of bounds (valid range: 0-9).
        InvalidProfileIndex,
        /// Sensor did not complete measurement within the expected time window.
        Timeout,
        /// Gas heating plate did not reach the target temperature (check power supply).
        HeaterNotStable,
        /// Gas measurement completed, but data is marked as invalid by the sensor.
        GasDataNotReady,
    }

    /// Result type alias for BME680 operations.
    pub type Result<T> = core::result::Result<T, Bme680Error>;
}

/// Temperature wrapper for type-safety.
///
/// **Unit:** Centigrade * 100.
///
/// # Example
/// `Celsius(2350)` represents **23.50 °C**.
#[derive(Default, Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Celsius(pub i32);

impl Celsius {
    /// Splits the fixed-point value into integral (degrees) and fractional (decimals) parts.
    ///
    /// Useful for logging or displaying data without converting to `f32`.
    ///
    /// # Returns
    /// A tuple `(whole, fraction)`.
    ///
    /// # Example
    /// ```rust
    /// use bme680_driver::Celsius;
    /// let temp = Celsius(2350);
    /// assert_eq!(temp.split(), (23, 50)); // Represents 23.50 °C
    /// ```
    pub fn split(&self) -> (i32, i32) {
        (self.0 / 100, self.0 % 100)
    }
}

/// Duration wrapper for type-safety. Stored in milliseconds.
#[derive(Default, Debug, Clone, Copy)]
pub struct Milliseconds(pub u32);

/// Factory-fused calibration coefficients read from the sensor.
///
/// These parameters are unique to every individual chip and are read from
/// non-volatile memory during `init()`. They are strictly required to compensate
/// the raw ADC values into physical units.
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
/// This struct holds the uncompensated data read from `regs::ADDR_PRESS_MSB` onwards.
/// It is used internally by the driver as input for the compensation formulas.
#[derive(Debug, Copy, Clone)]
pub struct RawData {
    pub(crate) temp_adc: u32,
    pub(crate) hum_adc: u16,
    pub(crate) press_adc: u32,
    pub(crate) gas_adc: u16,
    /// Range switching error used for gas calculation.
    pub(crate) gas_range: u8,
    /// Flag: `true` if gas measurement is valid.
    pub(crate) gas_valid_r: bool,
    /// Flag: `true` if the target heater temperature was reached.
    pub(crate) heat_stab_r: bool,
}

/// Intermediate temperature values used for compensation.
///
/// `temp_fine` is a high-resolution temperature value calculated during temperature
/// compensation. It is carried over to pressure and humidity compensation formulas
/// to account for temperature dependencies.
#[derive(Debug, Copy, Clone, Default)]
pub struct CalcTempData {
    pub(crate) temp_fine: i32,
    pub(crate) temp_comp: i32,
}

/// Represents relative humidity.
///
/// **Unit:** Milli-percent (percent * 1000).
///
/// # Example
/// `Humidity(45123)` represents **45.123 %rH**.
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// Represents atmospheric pressure.
///
/// **Unit:** Pascal (Pa).
///
/// # Example
/// `Pressure(101325)` represents **101325 Pa** (or 1013.25 hPa).
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// Represents gas resistance.
///
/// **Unit:** Ohms (Ω).
///
/// A higher gas resistance typically indicates cleaner air (fewer VOCs).
/// A drastic drop in resistance usually indicates the presence of VOCs.
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gas(pub u32);

/// Compensated measurement result in physical units.
///
/// All fields use strong types (`Temperature`, `Humidity`, etc.) to prevent unit confusion.
/// If a measurement was disabled in the `Config`, the corresponding field will contain `0`.
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    /// Compensated temperature.
    pub temp: Celsius,
    /// Compensated relative humidity.
    pub hum: Humidity,
    /// Compensated atmospheric pressure.
    pub pres: Pressure,
    /// Compensated gas resistance.
    pub gas: Gas,
}

/// The main BME680 driver structure.
///
/// Use `Bme680::new(...)` or `Bme680::with_config(...)` to start.
/// The `STATE` generic uses the Typestate pattern to track initialization status
/// at compile time, preventing invalid API calls.
#[derive(Debug, Copy, Clone)]
pub struct Bme680<I2C, STATE> {
    i2c: I2C,
    address: u8,
    pub(crate) calib_data: CalibData,
    config: Config,
    _state: PhantomData<STATE>,
}

impl<I2C> Bme680<I2C, Idle>
where
    I2C: i2c::I2c,
{
    /// Creates a new driver instance in the `Uninitialized` state.
    ///
    /// This method does not yet communicate with the sensor.
    ///
    /// # Arguments
    /// * `i2c` - The I2C bus object.
    /// * `address` - The I2C address of the sensor (typically `0x76` or `0x77`).
    pub fn new(i2c: I2C, address: u8) -> Bme680<I2C, Uninitialized> {
        Bme680 {
            i2c,
            address,
            calib_data: CalibData::default(),
            config: Config::default(),
            _state: PhantomData,
        }
    }

    /// Creates a new driver instance with a pre-defined configuration.
    ///
    /// Use the `BME680Builder` to create the `config` object.
    pub fn with_config(i2c: I2C, address: u8, config: Config) -> Bme680<I2C, Uninitialized> {
        Bme680 {
            i2c,
            address,
            calib_data: CalibData::default(),
            config,
            _state: PhantomData,
        }
    }
}

impl<I2C, STATE> Bme680<I2C, STATE>
where
    I2C: i2c::I2c,
{
    /// Performs a soft-reset of the sensor.
    ///
    /// This resets all internal registers to their default values (Power-On-Reset).
    /// A delay of at least 2ms is required after the reset command.
    fn reset(&mut self, delay: &mut impl DelayNs) -> error::Result<()> {
        self.write_reg(&[regs::ADDR_RESET, msks::MSK_RESET])?;

        delay.delay_ms(2);

        Ok(())
    }

    /// Reads data from a starting register address into a provided buffer.
    fn read_into(&mut self, reg_address: u8, buffer: &mut [u8]) -> error::Result<()> {
        self.i2c
            .write_read(self.address, &[reg_address], buffer)
            .map_err(|_| error::Bme680Error::I2CError)
    }

    /// Reads a single byte from a specific register address.
    fn read_reg_byte(&mut self, reg_address: u8) -> error::Result<u8> {
        let mut buffer = [0];

        self.i2c
            .write_read(self.address, &[reg_address], &mut buffer)
            .map_err(|_| error::Bme680Error::I2CError)?;

        Ok(buffer[0])
    }

    /// Writes a byte slice (typically `[Register_Address, Value]`) to the sensor.
    fn write_reg(&mut self, data: &[u8]) -> error::Result<()> {
        self.i2c
            .write(self.address, data)
            .map_err(|_| error::Bme680Error::I2CError)?;
        Ok(())
    }

    /// Configures the heater duration and target temperature for the current gas profile.
    ///
    /// This function utilizes the current `ambient_temp` to calculate the correct
    /// heater resistance value needed to reach the target temperature.
    fn set_gas_heater_profile(&mut self) -> error::Result<()> {
        self.config_heater_on_time()?;
        self.config_target_resistance(None)?;

        Ok(())
    }

    /// Selects one of the 10 available gas heater profiles (0-9) in the sensor.
    fn select_gas_profile(&mut self, index: GasProfileIndex) -> error::Result<()> {
        let register = self.read_reg_byte(regs::ADDR_CTRL_GAS_1)?;

        // Clear NB_CONV bits using mask, then set new profile index
        self.write_reg(&[
            regs::ADDR_CTRL_GAS_1,
            (register & !msks::MSK_NB_CONV) | (index as u8),
        ])?;

        Ok(())
    }

    /// Enables the gas sensing functionality (sets `RUN_GAS` bit).
    fn enable_gas_measurement(&mut self) -> error::Result<()> {
        let register = self.read_reg_byte(regs::ADDR_CTRL_GAS_1)?;
        // Set RUN_GAS bit (Bit 4)
        self.write_reg(&[
            regs::ADDR_CTRL_GAS_1,
            (register & !msks::MSK_RUN_GAS) | msks::MSK_RUN_GAS,
        ])?;
        Ok(())
    }

    /// Disables the gas sensing functionality to save power.
    fn disable_gas_measurement(&mut self) -> error::Result<()> {
        let register = self.read_reg_byte(regs::ADDR_CTRL_GAS_1)?;
        // Clear RUN_GAS bit (Bit 4)
        self.write_reg(&[regs::ADDR_CTRL_GAS_1, register & !msks::MSK_RUN_GAS])?;
        Ok(())
    }
}

impl<I2C> Bme680<I2C, Uninitialized>
where
    I2C: i2c::I2c,
{
    /// Initializes the sensor: performs a soft-reset and loads factory calibration data.
    ///
    /// This consumes the `Uninitialized` driver and returns a `Ready` instance.
    ///
    /// # Process
    /// 1. Soft Reset.
    /// 2. Apply configuration (oversampling, IIR filter, gas settings).
    /// 3. Read calibration coefficients from ROM.
    ///
    /// # Errors
    /// Returns an error if I2C communication fails or if the sensor is unresponsive.
    pub fn init(mut self, delay: &mut impl DelayNs) -> error::Result<Bme680<I2C, Ready>> {
        // Sensor requires time to start up before reset
        delay.delay_ms(2);

        self.reset(delay)?;
        self.apply_config()?;

        // Read the factory calibration data (requires ~25ms I2C traffic)
        let calib_data = self.get_calib_data()?;

        Ok(Bme680 {
            i2c: self.i2c,
            address: self.address,
            calib_data: calib_data,
            config: self.config,
            _state: PhantomData,
        })
    }

    /// Applies the full sensor configuration defined in `self.config`.
    ///
    /// This sets up oversampling rates, IIR filters, and activates the gas sensor
    /// if a profile is present.
    fn apply_config(&mut self) -> error::Result<()> {
        self.config_oversampling()?;
        self.config_iir_filter()?;

        if let Some(profile) = self.config.gas_profile {
            self.enable_gas_measurement()?;
            self.select_gas_profile(profile.index)?;
            self.set_gas_heater_profile()?;
        } else {
            self.disable_gas_measurement()?;
        }

        Ok(())
    }

    /// Sets oversampling rates for Humidity, Temperature, and Pressure.
    ///
    /// Writes to registers `CTRL_HUM` and `CTRL_MEAS`.
    fn config_oversampling(&mut self) -> error::Result<()> {
        // Humidity configuration (Register 0x72)
        // We must read first to preserve other bits if they existed
        let ctrl_hum = self.read_reg_byte(regs::ADDR_CTRL_HUM)?;
        let new_hum = (ctrl_hum & !msks::MSK_OSRS_H) | self.config.osrs_config.hum_osrs as u8;
        self.write_reg(&[regs::ADDR_CTRL_HUM, new_hum])?;

        // Temperature & Pressure configuration (Register 0x74)
        let ctrl_meas = self.read_reg_byte(regs::ADDR_CTRL_MEAS)?;

        // Prepare new bits
        let temp_pres_combined = ((self.config.osrs_config.temp_osrs as u8) << 5)
            | ((self.config.osrs_config.pres_osrs as u8) << 2);

        // Clear old bits (Using !Mask) and set new ones
        let new_meas = (ctrl_meas & !(msks::MSK_OSRS_T | msks::MSK_OSRS_P)) | temp_pres_combined;

        self.write_reg(&[regs::ADDR_CTRL_MEAS, new_meas])?;

        Ok(())
    }

    /// Configures the IIR (Infinite Impulse Response) filter coefficient.
    ///
    /// The IIR filter suppresses noise in pressure and temperature measurements.
    fn config_iir_filter(&mut self) -> error::Result<()> {
        let register = self.read_reg_byte(regs::ADDR_CONFIG)?;
        // Clear filter bits and set new value
        let new_reg_val = (register & !msks::MSK_FILTER) | ((self.config.iir_filter as u8) << 2);
        self.write_reg(&[regs::ADDR_CONFIG, new_reg_val])?;
        Ok(())
    }

    /// Reads factory-fused calibration coefficients from the sensor's ROM.
    ///
    /// The BME680 stores calibration data in two non-contiguous memory blocks.
    /// These bytes are required to compensate the raw ADC values into physical units.
    fn get_calib_data(&mut self) -> error::Result<CalibData> {
        let mut calib_data = CalibData::default();
        let mut buffer = [0u8; reg_sizes::SIZE_CALIB_TOTAL];

        // 1. Read first block (0x89..0xA0)
        self.read_into(regs::ADDR_CALIB_0, &mut buffer[0..reg_sizes::SIZE_CALIB_0])?;
        // 2. Read second block (0xE1..0xF0)
        self.read_into(regs::ADDR_CALIB_1, &mut buffer[reg_sizes::SIZE_CALIB_0..])?;

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

        // Use mask constant for bitwise operations
        calib_data.par_h1 =
            (((buffer[26] & msks::MSK_PAR_H1_LSB) as i32) | ((buffer[27] as i32) << 4)) as u16;
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
        calib_data.res_heat_val = self.read_reg_byte(regs::ADDR_RES_HEAT_VAL)?;

        // Use mask for range reading (Bits 4,5)
        calib_data.res_heat_range =
            (self.read_reg_byte(regs::ADDR_RES_HEAT_RANGE)? & msks::MSK_RES_HEAT_RANGE) >> 4;

        calib_data.range_sw_err = (self.read_reg_byte(regs::ADDR_RANGE_SW_ERR)? as i8) >> 4;

        Ok(calib_data)
    }
}

impl<I2C> Bme680<I2C, Ready>
where
    I2C: i2c::I2c,
{
    /// Triggers a full measurement cycle (Forced Mode), waits for completion, and returns the data.
    ///
    /// This is the primary method for retrieving sensor data. It handles the entire sequence:
    /// 1. Wakes the sensor (Forced Mode).
    /// 2. Waits for the TPH measurement + Gas Heating duration.
    /// 3. Reads raw ADC data.
    /// 4. Compensates raw values using factory calibration data.
    ///
    /// # Power Optimization
    /// If all measurements are set to `Skipped` and gas is disabled in the `Config`,
    /// this function returns immediately with a default `Measurement`, avoiding I2C traffic.
    pub fn read_new_data(&mut self, delay: &mut impl DelayNs) -> error::Result<Measurement> {
        // Optimization: Don't trigger a measurement if nothing is enabled.
        if self.config.gas_disabled() && self.config.osrs_config.is_all_skipped() {
            return Ok(Measurement::default());
        }

        // 1. Wake up sensor and start measurement cycle
        self.activate_forced_mode()?;

        // 2. Wait for heating phase (if gas is enabled)
        // The sensor measures T, P, H first, then heats up for gas measurement.
        if let Some(profile) = self.config.gas_profile {
            delay.delay_ms(profile.wait_time.0);
        }

        // 3. Poll for "New Data" bit and read ADC values
        let raw_data = self.get_raw_data(delay)?;

        let mut temp = CalcTempData::default();
        let mut hum = 0;
        let mut pres = 0;
        let mut gas = 0;

        // 4. Apply mathematical compensation to raw values (if not skipped)
        if self.config.osrs_config.temp_osrs != Oversampling::Skipped {
            temp = self.calc_temp(raw_data.temp_adc);

            // Humidity and Pressure compensation depends on "fine temperature" (t_fine)
            if self.config.osrs_config.hum_osrs != Oversampling::Skipped {
                hum = self.calc_hum(temp.temp_comp, raw_data.hum_adc);
            }

            if self.config.osrs_config.pres_osrs != Oversampling::Skipped {
                pres = self.calc_pres(temp.temp_fine, raw_data.press_adc);
            }
        }

        // 5. Check gas validity bits provided by the sensor
        if self.config.gas_enabled() && !raw_data.gas_valid_r {
            return Err(error::Bme680Error::GasDataNotReady);
        } else if self.config.gas_enabled() && !raw_data.heat_stab_r {
            return Err(error::Bme680Error::HeaterNotStable);
        }

        if self.config.gas_enabled() {
            gas = self.calc_gas(raw_data.gas_adc, raw_data.gas_range);
        }

        Ok(Measurement {
            temp: Celsius(temp.temp_comp),
            hum: Humidity(hum),
            pres: Pressure(pres),
            gas: Gas(gas),
        })
    }

    /// Reads the Chip ID from the sensor.
    ///
    /// Used to verify communication. Expected value is usually `0x61`.
    pub fn read_chip_id(&mut self) -> error::Result<u8> {
        Ok(self.read_reg_byte(regs::ADDR_ID)?)
    }

    /// Updates the ambient temperature used for gas heater calculation.
    ///
    /// **Important:** To maintain stability, avoid calling this method after every single measurement.
    /// Only update if the temperature has changed significantly (e.g. > 1°C), as changes
    /// to the heater resistance can cause the gas sensor values to fluctuate temporarily.
    pub fn update_ambient_temp(&mut self, temp: Celsius) -> error::Result<()> {
        self.config_target_resistance(Some(temp))?;

        Ok(())
    }

    /// Switches the active gas profile to a new one.
    ///
    /// This updates the configuration, selects the profile on the sensor, and recalculates
    /// the heater resistance values.
    pub fn switch_profile(&mut self, profile: GasProfile) -> error::Result<()> {
        self.config.gas_profile = Some(profile);

        self.select_gas_profile(profile.index)?;
        self.set_gas_heater_profile()?;

        Ok(())
    }

    /// Polls the sensor until new data is available and reads all ADC values.
    ///
    /// This method includes a timeout mechanism (max. 50ms) to prevent infinite loops
    /// if the sensor becomes unresponsive.
    fn get_raw_data(&mut self, delay: &mut impl DelayNs) -> error::Result<RawData> {
        let mut new_data = false;
        let mut timeout_us = 50000; // 50ms Timeout

        while !new_data {
            if timeout_us <= 0 {
                return Err(error::Bme680Error::Timeout);
            }
            // Check bit 7 (MSB) in EAS_STATUS_0 register
            // Note: 0x80 is implied for Bit 7
            new_data = (self.read_reg_byte(regs::ADDR_EAS_STATUS_0)? >> 7) != 0;

            delay.delay_us(500);
            timeout_us -= 500;
        }

        let mut buffer = [0u8; reg_sizes::SIZE_RAW_DATA];

        // Burst read starting from ADDR_PRESS_MSB Register (0x1F)
        self.read_into(regs::ADDR_PRESS_MSB, &mut buffer)?;

        // Reconstruct 20-bit and 16-bit ADC values from register bytes
        let press_adc =
            ((buffer[2] as u32) >> 4) | ((buffer[1] as u32) << 4) | ((buffer[0] as u32) << 12);
        let temp_adc =
            ((buffer[5] as u32) >> 4) | ((buffer[4] as u32) << 4) | ((buffer[3] as u32) << 12);
        let hum_adc = ((buffer[7] as u32) | ((buffer[6] as u32) << 8)) as u16;
        let gas_adc = (((buffer[12] as u32) >> 6) | ((buffer[11] as u32) << 2)) as u16;

        // Use mask for gas range
        let gas_range = buffer[12] & msks::MSK_GAS_RANGE;

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

    /// Activates 'Forced Mode' to trigger a single measurement cycle.
    ///
    /// In Forced Mode, the sensor performs one TPHG cycle and then automatically
    /// returns to Sleep Mode.
    fn activate_forced_mode(&mut self) -> error::Result<()> {
        let register = self.read_reg_byte(regs::ADDR_CTRL_MEAS)?;
        // Clear Mode bits and set to 01 (Forced)
        self.write_reg(&[regs::ADDR_CTRL_MEAS, (register & !msks::MSK_MODE) | 0b01])?;
        Ok(())
    }
}

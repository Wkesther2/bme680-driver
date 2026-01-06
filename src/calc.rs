use crate::{Bme680, CalcTempData, Celsius, GasProfileIndex, Milliseconds, error, i2c, regs};

/// Constants and lookup tables provided by Bosch for gas resistance compensation.
mod gas_constants {
    /// Lookup table used in the calculation of gas resistance (derived from Bosch API).
    pub static ARRAY1_INT: [u32; 16] = [
        2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2126008810, 2147483647,
        2130303777, 2147483647, 2147483647, 2143188679, 2136746228, 2147483647, 2126008810,
        2147483647, 2147483647,
    ];
    /// Range scaling table for gas resistance calculation.
    pub static ARRAY2_INT: [u32; 16] = [
        4096000000, 2048000000, 1024000000, 512000000, 255744255, 127110228, 64000000, 32258064,
        16016016, 8000000, 4000000, 2000000, 1000000, 500000, 250000, 125000,
    ];
}

impl<I2C, STATE, E> Bme680<I2C, STATE>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Configures the duration (on-time) of the heater for a specific gas profile.
    ///
    /// The BME680 encodes the wait time using a 6-bit mantissa and a 2-bit multiplier.
    /// This function converts a [`Milliseconds`] value into this hardware-specific format.
    ///
    /// # Arguments
    /// * `wait_time` - Duration in ms (Maximum supported: 4096ms).
    /// * `profile_index` - The profile slot (0-9) to store this timing.
    ///
    /// # Errors
    /// Returns [`Bme680Error::InvalidWaitTime`] if the requested duration exceeds hardware limits.
    pub(crate) fn config_heater_on_time(
        &mut self,
        wait_time: Milliseconds,
        profile_index: GasProfileIndex,
    ) -> error::Result<(), E> {
        if wait_time.0 > 4096 {
            return Err(error::Bme680Error::InvalidWaitTime);
        }

        let mut multiplicator: u8 = 0;
        let mut multiplicator_bits = 0b00;

        // The hardware supports 4 different ranges (multipliers) to represent time:
        // x1 (0-63ms), x4 (64-252ms), x16 (253-1008ms), x64 (1009-4032ms)
        if wait_time.0 <= 63 {
            multiplicator = 1;
            multiplicator_bits = 0b00;
        } else if wait_time.0 > 63 && wait_time.0 <= 252 {
            multiplicator = 4;
            multiplicator_bits = 0b01;
        } else if wait_time.0 > 252 && wait_time.0 <= 1008 {
            multiplicator = 16;
            multiplicator_bits = 0b10;
        } else if wait_time.0 > 1008 && wait_time.0 <= 4032 {
            multiplicator = 64;
            multiplicator_bits = 0b11;
        }

        // Calculate the 6-bit base value
        let base_value = (wait_time.0 / (multiplicator as u32)) as u8;

        // The final byte format: [Multiplier Bits (7:6)][Base Value (5:0)]
        let register_value = base_value + (multiplicator_bits << 6);

        // Registers for heater duration start at 0x64 (gas_wait_0)
        self.write_reg(&[
            regs::ADDR_GAS_WAIT_0 + (profile_index as u8),
            register_value,
        ])?;

        Ok(())
    }

    /// Calculates and sets the target resistance for the gas heater.
    ///
    /// This translates a target temperature in [`Celsius`] into a register value,
    /// accounting for ambient temperature and chip-specific calibration data.
    ///
    /// # Arguments
    /// * `target_temp` - The temperature you want the plate to reach (capped at 400°C).
    /// * `amb_temp` - The current surrounding temperature (baseline for the heater).
    /// * `profile_index` - The profile slot (0-9) to store this resistance.
    ///
    /// # Note
    /// The sensor uses an internal heating control loop. Writing the correct 8-bit value
    /// here ensures the plate reaches exactly the requested temperature.

    pub(crate) fn config_target_resistance(
        &mut self,
        target_temp: Celsius,
        amb_temp: Celsius,
        profile_index: GasProfileIndex,
    ) -> error::Result<(), E> {
        // Hardware Safety: Heating above 400°C can damage the sensor membrane.
        let target_temp = if target_temp.0 <= 400 {
            target_temp.0
        } else {
            400
        };

        // Variable compensation based on Bosch API formulas
        let var1 = ((amb_temp.0 * (self.calib_data.par_g3 as i32)) / 10) << 8;
        let var2 = (self.calib_data.par_g1 as i32 + 784)
            * ((((((self.calib_data.par_g2 as i32) + 154009) * target_temp * 5) / 100) + 3276800)
                / 10);
        let var3 = var1 + (var2 >> 1);
        let var4 = var3 / (self.calib_data.res_heat_range as i32 + 4);
        let var5 = 131 * (self.calib_data.res_heat_val as i32) + 65536;

        // Calculate the scaled resistance value
        let res_heat_x100 = ((var4 / var5) - 250) * 34;

        // Convert to the 8-bit value required by the sensor register
        let res_heat_x = ((res_heat_x100 + 50) / 100) as u8;

        // Registers for heater resistance start at 0x5A (res_heat_0)
        self.write_reg(&[regs::ADDR_RES_HEAT_0 + (profile_index as u8), res_heat_x])?;

        Ok(())
    }

    /// Compensates the raw temperature ADC value into physical units.
    ///
    /// This is the first calculation that must be performed, as it produces `temp_fine`,
    /// which is required for pressure and humidity compensation.
    ///
    /// # Arguments
    /// * `temp_adc` - The 20-bit raw temperature value from the sensor registers.
    ///
    /// # Returns
    /// A [`CalcTempData`] struct containing the high-precision `temp_fine` and the
    /// human-readable `temp_comp` in Centigrade (multiplied by 100).
    pub(crate) fn calc_temp(&mut self, temp_adc: u32) -> CalcTempData {
        let var1 = ((temp_adc as i32) >> 3) - ((self.calib_data.par_t1 as i32) << 1);
        let var2 = (var1 * self.calib_data.par_t2 as i32) >> 11;
        let var3 =
            ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((self.calib_data.par_t3 as i32) << 4)) >> 14;
        let temp_fine = var2 + var3;

        CalcTempData {
            temp_fine,
            // Final temperature in Celsius * 100 (e.g., 2500 = 25.00°C)
            temp_comp: ((temp_fine * 5) + 128) >> 8,
        }
    }

    /// Compensates the raw pressure ADC value into Pascal.
    ///
    /// # Arguments
    /// * `t_fine` - The high-resolution temperature intermediate from [`calc_temp`].
    /// * `press_adc` - The 20-bit raw pressure value from the sensor registers.
    ///
    /// # Returns
    /// The atmospheric pressure in **Pascal (Pa)**.
    pub(crate) fn calc_pres(&mut self, t_fine: i32, press_adc: u32) -> u32 {
        let mut var1 = (t_fine >> 1) - 64_000;
        let mut var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * self.calib_data.par_p6 as i32) >> 2;
        var2 += (var1 * self.calib_data.par_p5 as i32) << 1;
        var2 = (var2 >> 2) + ((self.calib_data.par_p4 as i32) << 16);
        var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((self.calib_data.par_p3 as i32) << 5))
            >> 3)
            + (((self.calib_data.par_p2 as i32) * var1) >> 1);
        var1 = var1 >> 18;
        var1 = ((32768 + var1) * (self.calib_data.par_p1 as i32)) >> 15;

        let mut press_comp = 1048576u32.wrapping_sub(press_adc) as i32;
        press_comp = ((press_comp - (var2 >> 12)) as u32).wrapping_mul(3125) as i32;

        // Handle potential overflow for high pressure values
        if press_comp >= (1 << 30) {
            press_comp = ((press_comp as u32).wrapping_div(var1 as u32) << 1) as i32;
        } else {
            press_comp = ((press_comp << 1) as u32).wrapping_div(var1 as u32) as i32;
        }

        let var1 = ((self.calib_data.par_p9 as i32)
            * (((press_comp >> 3) * (press_comp >> 3)) >> 13))
            >> 12;
        let var2 = ((press_comp >> 2) * self.calib_data.par_p8 as i32) >> 13;
        let var3 = ((press_comp >> 8)
            * (press_comp >> 8)
            * (press_comp >> 8)
            * self.calib_data.par_p10 as i32)
            >> 17;

        press_comp += (var1 + var2 + var3 + ((self.calib_data.par_p7 as i32) << 7)) >> 4;
        press_comp as u32
    }

    /// Compensates the raw humidity ADC value into milli-percent.
    ///
    /// # Arguments
    /// * `temp_comp` - The compensated temperature from [`calc_temp`].
    /// * `hum_adc` - The 16-bit raw humidity value from the sensor registers.
    ///
    /// # Returns
    /// Relative humidity in **milli-percent** (e.g., 45000 = 45.000 %rH).
    pub(crate) fn calc_hum(&mut self, temp_comp: i32, hum_adc: u16) -> i32 {
        let temp_scaled = temp_comp;
        let var1 = hum_adc as i32
            - ((self.calib_data.par_h1 as i32) << 4)
            - (((temp_scaled * self.calib_data.par_h3 as i32) / 100) >> 1);
        let var2 = (self.calib_data.par_h2 as i32
            * (((temp_scaled * self.calib_data.par_h4 as i32) / 100)
                + (((temp_scaled * ((temp_scaled * self.calib_data.par_h5 as i32) / 100)) >> 6)
                    / 100
                    + (1 << 14))))
            >> 10;
        let var3 = var1 * var2;
        let var4 = ((self.calib_data.par_h6 as i32) << 7)
            + ((temp_scaled * self.calib_data.par_h7 as i32) / 100)
            >> 4;
        let var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
        let var6 = (var4 * var5) >> 1;

        // Returns value scaled by 1000 for high resolution without floats
        (((var3 + var6) >> 10) * 1000) >> 12
    }

    /// Converts the raw gas resistance ADC value into Ohms.
    ///
    /// # Arguments
    /// * `gas_adc` - Raw 10-bit gas resistance value.
    /// * `gas_range` - Range switching index from the sensor status register.
    ///
    /// # Returns
    /// The gas resistance in **Ohms (Ω)**.
    pub(crate) fn calc_gas(&mut self, gas_adc: u16, gas_range: u8) -> u32 {
        // Calculation uses 64-bit arithmetic to prevent overflow during intermediate steps
        let var1 = ((1340 + (5 * self.calib_data.range_sw_err as i64))
            * (gas_constants::ARRAY1_INT[gas_range as usize]) as i64)
            >> 16;
        let var2 = ((gas_adc as i64) << 15) - (1 << 24) + var1;
        let var3 = (gas_constants::ARRAY2_INT[gas_range as usize] as i64 * var1) >> 9;

        // Final resistance calculation
        let calc_gas_res: u32 = ((var3 + ((var2 as i64) >> 1i64)) / var2 as i64) as u32;

        calc_gas_res
    }
}

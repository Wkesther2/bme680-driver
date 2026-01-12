use crate::{Bme680, CalcTempData, Celsius, error, i2c, regs};

/// Von Bosch bereitgestellte Konstanten und Nachschlagetabellen für die Gaswiderstandsberechnung.
mod gas_constants {
    /// Lookup-Tabelle für die Basis-Kompensation (aus der Bosch-API abgeleitet).
    pub static ARRAY1_INT: [u32; 16] = [
        2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2126008810, 2147483647,
        2130303777, 2147483647, 2147483647, 2143188679, 2136746228, 2147483647, 2126008810,
        2147483647, 2147483647,
    ];
    /// Bereichs-Skalierungstabelle zur Umrechnung der ADC-Werte in Ohm.
    pub static ARRAY2_INT: [u32; 16] = [
        4096000000, 2048000000, 1024000000, 512000000, 255744255, 127110228, 64000000, 32258064,
        16016016, 8000000, 4000000, 2000000, 1000000, 500000, 250000, 125000,
    ];
}

impl<I2C, STATE> Bme680<I2C, STATE>
where
    I2C: i2c::I2c,
{
    /// Konfiguriert die Aufheizdauer für das aktuelle Gasprofil.
    ///
    /// Die Hardware kodiert die Zeit in einem speziellen Format:
    /// 6 Bit Mantisse und 2 Bit Multiplikator.
    ///
    /// # Mechanik
    /// - Multiplikator 00: 1ms Schritte
    /// - Multiplikator 01: 4ms Schritte
    /// - Multiplikator 10: 16ms Schritte
    /// - Multiplikator 11: 64ms Schritte
    ///
    /// # Fehler
    /// Gibt [`Bme680Error::InvalidWaitTime`] zurück, wenn die Zeit > 4096ms ist.
    pub(crate) fn config_heater_on_time(&mut self) -> error::Result<()> {
        if let Some(profile) = self.config.gas_profile {
            let wait_time = profile.wait_time.0;

            if wait_time > 4096 {
                return Err(error::Bme680Error::InvalidWaitTime);
            }

            let multiplicator;
            let multiplicator_bits;

            if wait_time <= 63 {
                multiplicator = 1;
                multiplicator_bits = 0b00;
            } else if wait_time <= 252 {
                multiplicator = 4;
                multiplicator_bits = 0b01;
            } else if wait_time <= 1008 {
                multiplicator = 16;
                multiplicator_bits = 0b10;
            } else {
                multiplicator = 64;
                multiplicator_bits = 0b11;
            }

            let base_value = (wait_time / (multiplicator as u32)) as u8;
            let register_value = (base_value & 0x3F) | (multiplicator_bits << 6);

            self.write_reg(&[
                regs::ADDR_GAS_WAIT_0 + (profile.index as u8),
                register_value,
            ])?;
        }
        Ok(())
    }

    /// Berechnet den digitalen Zielwiderstand für die Heizplatte.
    ///
    /// Wandelt die Zieltemperatur unter Berücksichtigung der Umgebungstemperatur
    /// und der Chip-Kalibrierung in einen 8-Bit-Registerwert um.
    ///
    /// # Sicherheit
    /// Die Temperatur wird intern auf 400°C begrenzt, um die Sensormembran zu schützen.
    pub(crate) fn config_target_resistance(
        &mut self,
        amb_temp: Option<Celsius>,
    ) -> error::Result<()> {
        if let Some(profile) = self.config.gas_profile {
            if let Some(temp) = amb_temp {
                self.config.ambient_temp = temp;
            }
            let ambient_temp = self.config.ambient_temp.0;

            let target_temp = if profile.target_temp.0 <= 400 {
                profile.target_temp.0
            } else {
                400
            };

            // Ganzzahl-Kompensation basierend auf Bosch API Formeln
            let var1 = ((ambient_temp * (self.calib_data.par_g3 as i32)) / 10) << 8;
            let var2 = (self.calib_data.par_g1 as i32 + 784)
                * ((((((self.calib_data.par_g2 as i32) + 154009) * target_temp * 5) / 100)
                    + 3276800)
                    / 10);
            let var3 = var1 + (var2 >> 1);
            let var4 = var3 / (self.calib_data.res_heat_range as i32 + 4);
            let var5 = 131 * (self.calib_data.res_heat_val as i32) + 65536;

            let res_heat_x100 = ((var4 / var5) - 250) * 34;
            let res_heat_x = ((res_heat_x100 + 50) / 100) as u8;

            self.write_reg(&[regs::ADDR_RES_HEAT_0 + (profile.index as u8), res_heat_x])?;
        }
        Ok(())
    }

    /// Rechnet den rohen Temperatur-ADC-Wert in Milli-Grad Celsius um.
    ///
    /// Dies ist die wichtigste Kompensation, da `temp_fine` als Referenz für
    /// Druck und Luftfeuchtigkeit zwingend erforderlich ist.
    pub(crate) fn calc_temp(&mut self, temp_adc: u32) -> CalcTempData {
        let var1 = ((temp_adc as i32) >> 3) - ((self.calib_data.par_t1 as i32) << 1);
        let var2 = (var1 * self.calib_data.par_t2 as i32) >> 11;
        let var3 =
            ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((self.calib_data.par_t3 as i32) << 4)) >> 14;
        let temp_fine = var2 + var3;

        CalcTempData {
            temp_fine,
            temp_comp: ((temp_fine * 5) + 128) >> 8,
        }
    }

    /// Rechnet den rohen Druck-ADC-Wert in Pascal (Pa) um.
    ///
    /// Erfordert `t_fine` aus der Temperaturkompensation, um temperaturbedingte
    /// Driften des Drucksensors auszugleichen.
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

    /// Rechnet den rohen Feuchtigkeits-ADC-Wert in Milli-Prozent (% * 1000) um.
    pub(crate) fn calc_hum(&mut self, temp_comp: i32, hum_adc: u16) -> i32 {
        let var1 = hum_adc as i32
            - ((self.calib_data.par_h1 as i32) << 4)
            - (((temp_comp * self.calib_data.par_h3 as i32) / 100) >> 1);
        let var2 = (self.calib_data.par_h2 as i32
            * (((temp_comp * self.calib_data.par_h4 as i32) / 100)
                + (((temp_comp * ((temp_comp * self.calib_data.par_h5 as i32) / 100)) >> 6)
                    / 100
                    + (1 << 14))))
            >> 10;
        let var3 = var1 * var2;
        let var4 = ((self.calib_data.par_h6 as i32) << 7)
            + ((temp_comp * self.calib_data.par_h7 as i32) / 100)
            >> 4;
        let var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
        let var6 = (var4 * var5) >> 1;

        (((var3 + var6) >> 10) * 1000) >> 12
    }

    /// Wandelt den rohen Gas-ADC-Wert und den gewählten Bereich in Ohm (Ω) um.
    pub(crate) fn calc_gas(&mut self, gas_adc: u16, gas_range: u8) -> u32 {
        // Nutzt 64-Bit Arithmetik für Zwischendaten, um Overflows bei extremen Widerständen zu vermeiden.
        let var1 = ((1340 + (5 * self.calib_data.range_sw_err as i64))
            * (gas_constants::ARRAY1_INT[gas_range as usize]) as i64)
            >> 16;
        let var2 = ((gas_adc as i64) << 15) - (1 << 24) + var1;
        let var3 = (gas_constants::ARRAY2_INT[gas_range as usize] as i64 * var1) >> 9;

        ((var3 + (var2 >> 1)) / var2) as u32
    }
}

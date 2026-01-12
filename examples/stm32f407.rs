//! Example: Basic environmental monitoring with the BME680 on an STM32F407.
//!
//! This example demonstrates:
//! 1. **Initialization**: Setting up I2C and the BME680 driver using the Type-State pattern.
//! 2. **Flexible Configuration**: Using the `BME680Builder` to customize measurements.
//! 3. **Fixed-Point Handling**: Utilizing the `.split()` method for easy logging without floats.
//! 4. **Gas Sensing**: Configuring a heater profile for VOC detection.
//! 5. **Stability Management**: Implementing a temperature hysteresis to prevent gas value jumps.

#![no_main]
#![no_std]
#![deny(unsafe_code)]

use bme680_driver::*;
use defmt_rtt as _;
use panic_probe as _;
use stm32f4xx_hal::{self as hal, prelude::*};

#[cortex_m_rt::entry]
fn main() -> ! {
    // --- 1. Hardware Setup ---
    let dp = hal::pac::Peripherals::take().unwrap();
    let clock_cfg = hal::rcc::Config::default().sysclk(168.MHz());
    let mut rcc = dp.RCC.freeze(clock_cfg);

    // Setup I2C1 (SCL on PB6, SDA on PB7)
    let gpiob = dp.GPIOB.split(&mut rcc);
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();

    let i2c = hal::i2c::I2c1::new(
        dp.I2C1,
        (scl, sda),
        hal::i2c::Mode::Standard {
            frequency: 100.kHz().into(),
        },
        &mut rcc,
    );

    let mut delay = dp.TIM6.delay_us(&mut rcc);

    // --- 2. Sensor Configuration ---
    // Define a gas heater profile: Target 300°C with 300ms duration.
    let my_gas_profile = GasProfile {
        index: GasProfileIndex::Profile0,
        target_temp: Celsius(300),
        wait_time: Milliseconds(300),
    };

    // Use the Builder to define initial state.
    // Note: ambient_temp is given in milli-Celsius (2000 = 20.00 °C).
    let config = BME680Builder::new()
        .ambient_temp(Celsius(2000))
        .temp_oversampling(Oversampling::X2)
        .hum_oversampling(Oversampling::X1)
        .pres_oversampling(Oversampling::Skipped) // Pressure disabled to save power/time
        .gas_profile(Some(my_gas_profile))
        .build();

    // --- 3. Driver Initialization ---
    // The driver starts 'Uninitialized' and consumes itself into a 'Ready' state via .init()
    let mut bme680 = Bme680::with_config(i2c, 0x76, config)
        .init(&mut delay)
        .expect("Failed to initialize BME680");

    defmt::info!("BME680 initialized successfully");

    // Track the last temperature used for heater calculation to implement hysteresis.
    let mut last_updated_temp = config.ambient_temp;

    // --- 4. Measurement Loop ---
    loop {
        // read_new_data triggers 'Forced Mode', waits for the TPHG duration, and reads results.
        match bme680.read_new_data(&mut delay) {
            Ok(data) => {
                // Formatting: .split() returns (integer, fractional) for easy no-float logging.
                let t = data.temp.split();
                let h = data.hum.split();

                defmt::println!("Temp: {}.{} °C", t.0, t.1);
                defmt::println!("Hum:  {}.{} %", h.0, h.1);
                defmt::println!("Pres: {} Pa (Skipped)", data.pres.0);

                if data.gas.0 > 0 {
                    defmt::println!("Gas Resistance: {} Ohm", data.gas.0);
                }

                // --- Dynamic Heater Compensation with Hysteresis ---
                // Problem: Updating the heater resistance after every measurement causes
                // thermal oscillations, leading to massive jumps in gas resistance values.
                //
                // Solution: Only update the heater profile if the ambient temperature
                // changes by more than 2.00°C (2000 m°C). This balances accuracy and stability.
                if (data.temp.0 - last_updated_temp.0).abs() >= 2000 {
                    defmt::info!("Significant temp change detected. Recalibrating heater...");
                    bme680
                        .update_ambient_temp(data.temp)
                        .expect("Failed to update ambient temperature!");
                    last_updated_temp = data.temp;
                }
            }
            Err(e) => defmt::error!("Measurement error: {:?}", e),
        }

        // Wait 5 seconds between samples.
        // Constant sampling helps the MOX sensor maintain a stable chemical state.
        delay.delay_ms(5000);
    }
}

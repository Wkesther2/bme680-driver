//! Example: Basic environmental monitoring with the BME680 on an STM32F407.
//!
//! This example demonstrates:
//! 1. **Initialization**: Setting up I2C and the BME680 driver.
//! 2. **Configuration**: Setting oversampling, filters, and gas heater profiles.
//! 3. **Data Processing**: Reading measurements and manually formatting the
//!    fixed-point data for logging.
//! 4. **Dynamic Compensation**: Updating the gas heater profile based on the
//!    current ambient temperature to ensure accurate gas resistance readings.

#![no_main]
#![no_std]
#![deny(unsafe_code)]

// The driver is now independent of logging frameworks.
// We use defmt explicitly in the example code only.
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

    // Setup a delay provider (TIM6) used by the driver for internal timings
    let mut delay = dp.TIM6.delay_us(&mut rcc);

    // --- 2. Driver Initialization ---
    // Instantiate driver with the default I2C address (0x76 or 0x77)
    let bme680 = Bme680::new(i2c, 0x76);

    // Initialize the sensor (performs Soft-reset and reads calibration data)
    let mut bme680 = bme680
        .init(&mut delay)
        .expect("Failed to initialize BME680");

    // --- 3. Sensor Configuration ---
    // Define a gas heater profile: Target 300°C with a 300ms heating duration
    let gas_profile0 = GasProfile {
        index: GasProfileIndex::Profile0,
        target_temp: Celsius(300),
        wait_time: Milliseconds(300),
    };

    // Configure sensor oversampling settings (noise reduction vs. speed)
    let osrs_config = OversamplingConfig {
        temp_osrs: Oversampling::X1,
        hum_osrs: Oversampling::X1,
        pres_osrs: Oversampling::X1,
    };

    // Assemble the full sensor configuration
    let mut config = Config {
        osrs_config,
        iir_filter: IIRFilter::IIR0,
        gas_profile: gas_profile0,
        // Initial ambient temperature estimate for the heater algorithm
        ambient_temp: Celsius(2300), // 23.00 °C
    };

    // Apply configuration. This puts the sensor into the appropriate state.
    bme680
        .configure_sensor(&mut config)
        .expect("Failed to configure sensor");

    // --- 4. Measurement Loop ---
    loop {
        // Trigger a measurement and wait for completion.
        // This function handles the necessary delays for measurement and heating.
        let data = bme680
            .read_new_data(&mut delay)
            .expect("Failed to read data");

        // --- Data Formatting & Logging ---
        // Since the driver uses fixed-point arithmetic, we use helper methods
        // to separate integral and decimal parts for human-readable output.

        let temp = data.temp.split();
        let hum = data.hum.split();
        let pres = data.pres.as_hpa(); // Explicitly converts Pa to hPa

        // Log using defmt (or any other logging framework/UART)
        defmt::println!("Temperature:    {}.{} °C", temp.0, temp.1);
        defmt::println!("Humidity:       {}.{} %", hum.0, hum.1);
        defmt::println!("Pressure:       {}.{} hPa", pres.0, pres.1);
        defmt::println!("Gas Resistance: {}  Ohm", data.gas.0); // Raw value access
        defmt::println!("");

        // --- Dynamic Heater Compensation ---
        // The gas sensor's heating plate resistance relies on the ambient temperature.
        // We update the profile using the just-measured temperature (`data.temp.0`)
        // to ensure the plate hits exactly 300°C in the next cycle.
        bme680
            .set_gas_heater_profile(gas_profile0, Celsius(data.temp.0))
            .expect("Failed to update heater profile");

        // Wait 5 seconds before the next measurement cycle
        delay.delay_ms(5000);
    }
}

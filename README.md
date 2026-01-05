# bme680-driver

[![Crates.io](https://img.shields.io/crates/v/bme680-driver.svg)](https://crates.io/crates/bme680-driver)
[![Documentation](https://docs.rs/bme680-driver/badge.svg)](https://docs.rs/bme680-driver)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](https://github.com/Wkesther2/bme680-driver)

## Table of Contents
- [Overview](#-overview)
- [Features](#-features)
- [Repository Structure](#-repository-structure)
- [Installation](#-installation)
- [Quick Start](#-quick-start-stm32-example)
- [Units & Precision](#-units-&-precision)
- [License](#-license)

---

## Overview

A type-safe, `no_std` Rust driver for the **Bosch BME680** environmental sensor. It provides high-level access to temperature, humidity, atmospheric pressure, and gas resistance measurements using the `embedded-hal` traits.

---

## Features

- **Typestate Pattern**: Prevents illegal sensor states and ensures correct initialization.
- **Fixed-Point Arithmetic**: High-performance compensation formulas without the need for a floating-point unit (FPU).
- **Type-Safe Units**: Uses custom types like `Celsius` and `Milliseconds` to prevent unit-mixing errors.
- **No-Std**: Suitable for all bare-metal microcontrollers (STM32, ESP32, AVR, etc.).

---

## Repository Structure

```sh
└── bme680-driver/
    ├── src/
    │   ├── lib.rs
    │   └── calc.rs
    ├── .examples/
    │   └── stm32f407.rs
    ├── .cargo/
    │   └── config.toml
    └── Cargo.toml
```

---

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
bme680-driver = "0.1.0"
```

---

## Quick Start (STM32 Example)

```rust
use bme680_driver::*;

// Initialize I2C and Delay from your HAL...
let bme = Bme680::new(i2c, 0x76);
let mut bme = bme.init(&mut delay).expect("Failed to init BME680");

let mut config = Config {
    osrs_config: OversamplingConfig {
        temp_osrs: Oversampling::X1,
        hum_osrs: Oversampling::X1,
        pres_osrs: Oversampling::X1,
    },
    iir_filter: IIRFilter::IIR0,
    gas_profile: GasProfile {
        index: GasProfileIndex::Profile0,
        target_temp: Celsius(300),
        wait_time: Milliseconds(300),
    },
    ambient_temp: Celsius(2300),
};

bme.configure_sensor(&mut config).unwrap();

loop {
    let data = bme.read_new_data(&mut delay).unwrap();
    
    // Use helper methods to format data (no floats needed!)
    let (temp_int, temp_frac) = data.temp.split();
    let (pres_hpa, pres_dec) = data.pres.as_hpa();
    
    // Log formatted data (e.g., via defmt)
    defmt::println!("Temperature:    {}.{} °C", temp.0, temp.1);
    defmt::println!("Humidity:       {}.{} %", hum.0, hum.1);
    defmt::println!("Pressure:       {}.{} hPa", pres.0, pres.1);
    defmt::println!("Gas Resistance: {}  Ohm", data.gas.0);
    defmt::println!("");
    
    // Dynamically update heater profile with current ambient temperature.
    // We access the raw value (.0) to wrap it into Celsius.
    bme.set_gas_heater_profile(config.gas_profile, Celsius(data.temp.0)).unwrap();
    
    delay.delay_ms(5000);
}
```

---

## Units & Precision

| Measurement | Unit | Scaling | Example |
| :--- | :--- | :--- | :--- |
| **Temperature** | **Celsius** | T * 100 | 2350 = 23.50 °C |
| **Humidity** | **% Relative Humidity** | H * 1000 | 45123 = 45.123 % |
| **Pressure** | **Pascal (Pa)** | 1.0 | 101325 Pa = 1013.25 hPa |
| **Gas Resistance** | **Ohm (Ω)** | 1.0 | 45000 = 45 kΩ |

---

## License
Licensed under either of:

* Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)

* MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)

at your option.
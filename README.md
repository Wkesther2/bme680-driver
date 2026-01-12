# bme680-driver

[![Crates.io](https://img.shields.io/crates/v/bme680-driver.svg)](https://crates.io/crates/bme680-driver)
[![Documentation](https://docs.rs/bme680-driver/badge.svg)](https://docs.rs/bme680-driver)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](https://github.com/Wkesther2/bme680-driver)

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Quick Start](#quick-start-stm32-example)
- [Units & Precision](#units-and-precision)
- [License](#license)

---

## Overview

A type-safe, `no_std` Rust driver for the **Bosch BME680** environmental sensor. It provides high-level access to temperature, humidity, atmospheric pressure, and gas resistance measurements using the `embedded-hal` traits.

---

## Features

- **Power Saving**: Individually enable/disable gas, pressure, temperature, or humidity measurements to reduce power consumption.
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
    │   ├── calc.rs
    │   └── settings.rs
    ├── examples/
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
bme680-driver = "0.3.0"
```

---

## Quick Start (STM32 Example)

```rust
use bme680_driver::*;

// --- 1. Sensor-Konfiguration via Builder ---
// Wir definieren ein Profil für die Gassensor-Heizplatte.
let gas_profile = GasProfile {
    index: GasProfileIndex::Profile0,
    target_temp: Celsius(300),      // Ziel: 300°C
    wait_time: Milliseconds(300),   // 300ms Aufheizzeit
};

let config = BME680Builder::new()
    .temp_oversampling(Oversampling::X2)
    .hum_oversampling(Oversampling::X1)
    .pres_oversampling(Oversampling::Skipped) // Druckmessung deaktiviert
    .gas_profile(Some(gas_profile))
    .ambient_temp(Celsius(2100))    // Erste Schätzung: 21.00°C
    .build();

// --- 2. Initialisierung (Typestate: Uninitialized -> Ready) ---
// I2C und Delay kommen von deinem HAL (z.B. stm32f4xx-hal)
let mut bme = Bme680::with_config(i2c, 0x76, config)
    .init(&mut delay)
    .expect("BME680 Init fehlgeschlagen");

// Tracking für die Heizungskompensation
let mut last_heater_update_temp = Celsius(2100);

loop {
    // Messung triggern (wartet automatisch auf die Heizphase)
    let data = bme.read_new_data(&mut delay).unwrap();
    
    // Daten formatieren ohne Floats (Festkomma-Arithmetik)
    let (t_int, t_frac) = data.temp.split();
    let (h_int, h_frac) = data.hum.split();
    
    defmt::println!("Temp: {}.{} °C", t_int, t_frac);
    defmt::println!("Hum:  {}.{} %", h_int, h_frac);
    
    if data.gas.0 > 0 {
        defmt::println!("Gas:  {} Ohm", data.gas.0);
    }

    // --- Dynamische Heizungskompensation ---
    // Wir aktualisieren die Heizparameter nur bei signifikanten Temp-Änderungen (> 2°C),
    // um chemische Instabilitäten (Sprünge) im Gassensor zu vermeiden.
    if (data.temp.0 - last_heater_update_temp.0).abs() >= 2000 {
        bme.update_ambient_temp(data.temp).unwrap();
        last_heater_update_temp = data.temp;
        defmt::info!("Heizprofil an neue Umgebungstemperatur angepasst.");
    }
    
    delay.delay_ms(5000);
}
```

---

## Units and Precision

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

# nRF-HAL

[![Crate](https://img.shields.io/crates/v/nrf-hal.svg)](https://crates.io/crates/nrf-hal)
[![Docs](https://docs.rs/nrf-hal/badge.svg)](https://docs.rs/nrf-hal)

This library provides high-level access to nRF peripherals. WIP. Uses
a similar API to [STM32-HAL](https://github.com/David-OConnor/stm32-hal);
designed to be interchangeable when able.

## Most features missing!

## Only tested on nRF-5280

## Currently based on [nrf-hal](https://github.com/nrf-rs/nrf-hal), with much code taken directly from it.
Uses [Embassy's nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice)
for Bluetooth functionality.

## Getting started
Review the [syntax overview example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/syntax_overview)
for example uses of many of this library's features. Copy and paste its whole folder (It's set up
using [Knurling's app template](https://github.com/knurling-rs/app-template)), or copy parts of `Cargo.toml` 
and `main.rs` as required.

The [conductivity module example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/conductivity_module)
is a complete example of simple firmware. It uses the DAC, I2C, Timer, and UART peripherals,
with a simple interupt-based control flow.

When specifying this crate as a dependency in `Cargo.toml`, you need to specify a feature
representing your MCU. If this is for code that runs on an MCU directly (ie not a library), also
 include a run-time feature, following the template `l4rt`. For example: 
```toml
cortex-m = "0.7.3"
cortex-m-rt = "0.6.13"
stm32-hal2 = { version = "^0.2.10", features = ["l4x3", "l4rt"]}
```

If you need `embedded-hal` traits, include the `embedded-hal` feature.

You can review [this section of Cargo.toml](https://github.com/David-OConnor/stm32-hal/blob/main/Cargo.toml#L61)
to see which MCU and runtime features are available.

### Example highlights:
```rust
use cortex_m;
use cortex_m_rt::entry;
use nrf_hal::{
    clocks::Clocks,
    gpio::{Pin, PinMode, OutputType},
    i2c::{I2c, I2cDevice},
    low_power,
    timer::{Timer, TimerInterrupt},
};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();

    let mut p15 = Pin::new(15, PinMode::Output);
    pb15.set_high();

    let mut timer = Timer::new_tim3(dp.TIM3, 0.2, &clock_cfg);
    timer.enable_interrupt(TimerInterrupt::Update);

    let mut scl = Pin::new(6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);

    let mut sda = Pin::new(7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    let twim = Twim::new(dp.I2C1, 100_000, &clock_cfg);

    loop {
        low_power::sleep_now(&mut cp.SCB);
    }
}
```
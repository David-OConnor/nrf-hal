# nRF-HAL

[![Crate](https://img.shields.io/crates/v/nrf-hal.svg)](https://crates.io/crates/nrf-hal)
[![Docs](https://docs.rs/nrf-hal/badge.svg)](https://docs.rs/nrf-hal)

This library provides high-level access to nRF-52 and nRF-53 peripherals. WIP. Uses
a similar API to [STM32-HAL](https://github.com/David-OConnor/stm32-hal);
designed to be interchangeable when able.

## Most features missing!
This is currently intended to use the specific features required by AnyLeaf projects
that use nrf-52. It may be expanded to be more general at some point in the future,
but that's not on the near-term road map. Please use the [nrf-rs](https://github.com/nrf-rs) libraries instead.

## Currently based on [nrf-rs](https://github.com/nrf-rs/nrf-hal), with much code taken directly from it.
Uses [Embassy's nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice)
for Bluetooth functionality.

## Getting started
Review the [syntax overview example](https://github.com/David-OConnor/stm32-hal/tree/main/examples/syntax_overview)
for example uses of many of this library's features. Copy and paste its whole folder (It's set up
using [Knurling's app template](https://github.com/knurling-rs/app-template)), or copy parts of `Cargo.toml` 
and `main.rs` as required.

When specifying this crate as a dependency in `Cargo.toml`, you need to specify a feature
representing your MCU. If this is for code that runs on an MCU directly (ie not a library), also
 include a run-time feature, following the template `52840-rt`. For example: 
```toml
cortex-m = "0.7.3"
cortex-m-rt = "0.6.13"
nrf-hal = { version = "^0.1.0", features = ["52840", "52840-rt"]}
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
    gpio::{Pin, Port, Dir, Drive},
    twim::{Twim, TwimFreq},
    low_power,
    timer::{Timer, TimerInterrupt},
};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    let _clock_cfg = Clocks::new(dp.CLOCK);

    let mut p15 = Pin::new(Port::P0, 15, Dir::Output);
    p15.set_high();

    // let mut timer = Timer::new_tim3(dp.TIM3, 0.2, &clock_cfg);
    // timer.enable_interrupt(TimerInterrupt::Update);

    let mut scl = Pin::new(Port::P0, 6, Dir::Output);
    scl.drive(Drive::S0D1);

    let mut sda = Pin::new(Port::P0, 7, Dir::Output);
    sda.drive(Drive::S0D1);

    let twim = Twim::new(dp.TWIM0, &scl, &sda, TwimFreq::K100);

    loop {
        low_power::sleep_now(&mut cp.SCB);
    }
}
```
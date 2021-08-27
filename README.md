# nRF-HAL

[![Crate](https://img.shields.io/crates/v/nrf-hal.svg)](https://crates.io/crates/nrf-hal)
[![Docs](https://docs.rs/nrf-hal/badge.svg)](https://docs.rs/nrf-hal)

This library provides high-level access to nRF-52 and nRF-53 peripherals. Uses
a similar API to [STM32-HAL](https://github.com/David-OConnor/stm32-hal);
designed to be interchangeable when able.

## Legacy features are unsupported
This library does not support the TWI, SPI, and UART peripherals; it only supports
their newer counterparts: TWIM[S], SPIM[S], and UARTE.

## Many features are missing
This is currently intended to use the specific features required by AnyLeaf projects
that use nRF-52. It may be expanded to be more general at some point in the future,
but that's not on the near-term road map. Please use the [nrf-rs](https://github.com/nrf-rs) libraries instead.

## Currently based on [nrf-rs](https://github.com/nrf-rs/nrf-hal), with much code taken directly from it.

## Uses the [ESB library](https://github.com/thalesfragoso/esb) for Nordic ShockBurst wireless communications.
(Currently unimplemented, but compatible if set up in user code.)

## Getting started
The [IR temperature transmitter example](https://github.com/David-OConnor/nrf-hal/tree/main/examples/temperature_transmitter)
is a complete example of simple production firmware. It uses the TWIM, Timer, RTC, and Radio peripherals,
using the ESB protocol to periodically transmit temperature. It's suitable for long-running, battery-powered
use.

When specifying this crate as a dependency in `Cargo.toml`, you need to specify a feature
representing your MCU. If this is for code that runs on an MCU directly (ie not a library), also
 include a run-time feature, following the template `52840-rt`. For example: 
```toml
cortex-m = "0.7.3"
cortex-m-rt = "0.6.13"
nrf-hal = { version = "^0.1.0", features = ["52840", "52840-rt"]}
```

If you need `embedded-hal` traits, include the `embedded-hal` feature.

The [IR temperature transmitter example](https://github.com/David-OConnor/nrf-hal/tree/main/examples/temperature_transmitter)
is a complete example of simple production firmware. It uses the TWIM, Timer, RTC, and Radio peripherals,
using the ESB protocol to periodically transmit temperature. It's suitable for long-running, battery-powered
use.

### Example highlights:
```rust
use cortex_m::{self, asm};
use cortex_m_rt::entry;
use nrf_hal::{
    clocks::Clocks,
    gpio::{Pin, Port, Dir, Drive},
    twim::{Twim, TwimFreq},
    pac,
    timer::{Timer, TimerMode},
};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

    let _clocks = Clocks::new(dp.CLOCK);

    let mut p15 = Pin::new(Port::P0, 15, Dir::Output);
    p15.set_high();

    let mut timer = Timer::new(dp.TIMER1, TimerMode::Timer, 1., 0);
    timer.enable_interrupt(0);

    let mut scl = Pin::new(Port::P0, 0, Dir::Input);
    scl.drive(Drive::S0D1);

    let mut sda = Pin::new(Port::P0, 1, Dir::Input);
    sda.drive(Drive::S0D1);

    let twim = Twim::new(dp.TWIM0, &scl, &sda, TwimFreq::K100);

    loop {
        asm::wfi();
    }
}
```
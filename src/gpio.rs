//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIO register functions.

#[cfg(feature = "51")]
use crate::pac::{gpio, GPIO as P0};

#[cfg(feature = "9160")]
use crate::pac::{p0_ns as gpio, P0_NS as P0};

#[cfg(not(any(feature = "9160", feature = "51")))]
use crate::pac::{p0 as gpio, P0};

#[cfg(any(feature = "52833", feature = "52840"))]
use crate::pac::P1;

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};

#[cfg(feature = "embedded-hal")]
use core::convert::Infallible;

use void::Void;

/// A GPIO port with up to 32 pins.
#[derive(Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Port {
    /// Port 0, available on all nRF52 and nRF51 MCUs.
    Port0 = 0x00,

    /// Port 1, only available on some nRF52 MCUs.
    #[cfg(any(feature = "52833", feature = "52840"))]
    Port1 = 0x20,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Pin direction; values for `DIR`
pub enum Dir {
    Input = 0,
    Output = 1,
}

// #[derive(Copy, Clone)]
// #[repr(u8)]
// /// Values for `GPIOx_MODER`
// pub enum PinMode {
//     Input,
//     Output,
//     Alt(u8),
//     Analog,
// }

// impl PinMode {
//     /// We use this function to find the value bits due to being unable to repr(u8) with
//     /// the wrapped `AltFn` value.
//     fn val(&self) -> u8 {
//         match self {
//             Self::Input => 0b00,
//             Self::Output => 0b01,
//             Self::Alt(_) => 0b10,
//             Self::Analog => 0b11,
//         }
//     }
// }

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `_`
pub enum OutputType {
    PushPull = 0,
    OpenDrain = 1,
}

// #[derive(Copy, Clone)]
// #[repr(u8)]
// /// Values for `GPIOx_OSPEEDR`. This configures I/O output speed. See the user manual
// /// for your MCU for what speeds these are. Note that Fast speed (0b10) is not
// /// available on all STM32 families.
// pub enum OutputSpeed {
//     Low = 0b00,
//     Medium = 0b01,
//     #[cfg(not(feature = "f3"))]
//     Fast = 0b10,
//     High = 0b11,
// }

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `_`
pub enum Pull {
    Floating = 0b00,
    Up = 0b01,
    Dn = 0b10,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `_`.
pub enum PinState {
    High = 1,
    Low = 0,
}

/// Represents a single GPIO pin. Allows configuration, and reading/setting state.
pub struct Pin {
    /// 00AB BBBB
    // pin_port: u8,
    port: Port,
    pin: u8,
}

impl Pin {
    /// Create a new pin, with a specific direction.
    fn new(port: Port, pin: u8, dir: Dir) -> Self {
        let mut result = Self {
            // pin_port: pin | port as u8,
            port,
            pin,
        };

        result.dir(dir);
        result
    }

    // /// Disconnects the pin.
    // ///
    // /// In disconnected mode the pin cannot be used as input or output.
    // /// It is primarily useful to reduce power usage.
    // pub fn disconnect(&mut self) {
    //     // Reset value is disconnected.
    //     unsafe { &(*$PX::ptr()).pin_cnf[$i] }.reset();
    // }

    /// Internal function used to access the register block.
    fn regs(&self) -> &gpio::RegisterBlock {
        let ptr = match self.port {
            Port::Port0 => P0::ptr(),
            #[cfg(any(feature = "52833", feature = "52840"))]
            Port::Port1 => P1::ptr(),
        };

        unsafe { &*ptr }
    }

    // todo: COnfirm how you << the pin is correct.

    /// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
    /// `set_low()`. Sets the `OUTSET` register. Atomic.
    pub fn set_state(&mut self, value: PinState) {
        match value {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    /// Set the pin's output voltage to high. Sets the `OUTSET` register. Atomic.
    pub fn set_high(&mut self) {
        self.regs()
            .outset
            .write(|w| unsafe { w.bits(1 << self.pin) });
    }

    /// Set the pin's output voltage to low. Sets the `OUTCLR` register. Atomic.
    pub fn set_low(&mut self) {
        self.regs()
            .outclr
            .write(|w| unsafe { w.bits(1 << self.pin) });
    }

    /// Check if the pin's input voltage is high. Reads from the `IN` register.
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    /// Check if the pin's input voltage is low. Reads from the `IN` register.
    pub fn is_low(&self) -> bool {
        self.regs().in_.read().bits() & (1 << self.pin) == 0
    }

    /// Set the pin's output direction. Sets the `DIRSET` or `DIRCLR` register.
    pub fn dir(&mut self, dir: Dir) {
        match dir {
            Dir::Input => self
                .regs()
                .dirclr
                .write(|w| unsafe { w.bits(1 << self.pin) }),
            Dir::Output => self
                .regs()
                .dirset
                .write(|w| unsafe { w.bits(1 << self.pin) }),
        }
    }

    // /// Set pin mode. Eg, Output, Input, Analog, or Alt. Sets the `MODER` register.
    // pub fn mode(&mut self) {
    //     // asdf
    // }

    /// Set internal pull resistor: Pull up, pull down, or floating. Sets the `_` register.
    pub fn pull(&mut self) {
        // self.
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl InputPin for Pin {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(self))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(Pin::is_low(self))
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl OutputPin for Pin {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(self);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl ToggleableOutputPin for Pin {
    type Error = Infallible;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        if self.is_high() {
            Pin::set_low(self);
        } else {
            Pin::set_high(self);
        }
        Ok(())
    }
}

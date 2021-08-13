//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIO register functions.

#[cfg(feature = "51")]
use crate::pac::{gpio, GPIO as P0};

#[cfg(feature = "9160")]
use crate::pac::{p0_ns as gpio, P0_NS as P0};

#[cfg(not(any(feature = "9160", feature = "51")))]
use crate::pac::{p0 as gpio, P0};

#[cfg(any(feature = "52833", feature = "52840", feature = "53"))]
use crate::pac::P1;

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

#[cfg(feature = "embedded-hal")]
use core::convert::Infallible;

/// A GPIO port with up to 32 pins.
#[derive(Debug, Eq, PartialEq)]
pub enum Port {
    /// Port 0, available on all nRF52 and nRF51 MCUs.
    Port0,
    /// Port 1, only available on some nRF52 MCUs.
    #[cfg(any(feature = "52833", feature = "52840", feature = "53"))]
    Port1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Pin direction; values for `DIR`
pub enum Dir {
    Input = 0,
    Output = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `PIN_CONF` reg, `INPUT` field.
pub enum InputBuf {
    /// Connect input buffer
    Connect = 0,
    /// Disconnect input buffer
    Disconnect = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `PIN_CONF` reg, `PULL` field.
pub enum Pull {
    /// No pull
    Disabled = 0,
    /// Pull down on pin
    Dn = 1,
    /// Pull up on pin
    Up = 3,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `PIN_CONF` reg, `SENSE` field.
pub enum Sense {
    /// Disabled
    Disabled = 0,
    /// Sense for high level
    High = 2,
    /// Sense for low level
    Low = 3,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Drive configuration. Values for `PIN_CONF` reg, `DRIVE` field.
pub enum Drive {
    /// Standard '0', standard '1'
    S0S1 = 0,
    /// High drive '0', standard '1
    H0S1 = 1,
    /// Standard '0', high drive '1'
    S0H1 = 2,
    /// High drive '0', high 'drive '1'
    H0H1 = 3,
    /// Disconnect '0' standard '1' (normally used for wired-or-connections
    D0S1 = 4,
    /// Disconnect '0' high drive '1' (normally used for wired-or-connections
    D0H1 = 5,
    /// Standard '0'. disconnect '1' (normally used for wired-or-connections
    S0D1 = 6,
    /// High drive '0' disconnect '1' (normally used for wired-or-connections
    H0D1 = 7,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `OUT`, `OUTSET`, `IN` etc.
pub enum PinState {
    High = 1,
    Low = 0,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `Latch` register.
pub enum Latch {
    /// Criteria has not been met
    NotLatched = 0,
    /// Criteria has been met
    Latched = 1,
}

/// Represents a single GPIO pin. Allows configuration, and reading/setting state.
pub struct Pin {
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

    /// Latch register indicating what GPIO pins that have met the criteria set in the PIN_CNF[n].SENSE registers.
    /// Reads from the the `LATCH` register.
    pub fn is_latched(&mut self, latch: Latch) -> bool {
        !(self.regs().latch.read().bits() & (1 << self.pin) == 0)
    }

    // /// Select between default DETECT signal behavior and LDETECT mode. Sets the `DETECTMODE` register.
    // pub fn detect_mode(&mut self, det_mode: DetectMode) {

    // }

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

    /// Set internal pull resistor: Pull up, pull down, or disabled. Sets the `PIN_CNF`
    /// register, `PULL` field.
    pub fn pull(&mut self, pull: Pull) {
        self.regs().pin_cnf[self.pin as usize].modify(|_, w| unsafe { w.pull().bits(pull as u8) })
    }

    /// Connect or disconnect input buffer. Sets `PIN_CNF` register, `INPUT` field.
    pub fn input_buf(&mut self, input: InputBuf) {
        self.regs().pin_cnf[self.pin as usize].modify(|_, w| w.input().bit(input as u8 != 0))
    }

    /// Set drive configuration. Sets `PIN_CNF` register, `DRIVE` field.
    pub fn drive(&mut self, drive: Drive) {
        self.regs().pin_cnf[self.pin as usize].modify(|_, w| w.drive().bits(drive as u8))
    }

    /// Pin sensing mechanism. Sets `PIN_CNF` register, `SENSE` field.
    pub fn sense(&mut self, sense: Sense) {
        self.regs().pin_cnf[self.pin as usize].modify(|_, w| unsafe { w.sense().bits(sense as u8) })
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

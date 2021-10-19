//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIO register functions. It also includes GPIO tasks and events (GPIOTE)
//! functionality.

// todo: Add more GPIOTE functionality, eg task mode and port interrupts.

#[cfg(any(feature = "9160", feature = "53"))]
use crate::pac::{p0_ns as gpio, P0_NS as P0};

#[cfg(not(any(feature = "9160", feature = "53")))]
use crate::pac::{p0 as gpio, P0};

#[cfg(any(feature = "52833", feature = "52840", feature = "53"))]
use crate::pac::P1;

#[cfg(feature = "53")]
use crate::pac::{p1_NS as P1, P0_S as P2, P1_S as P3};

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

#[cfg(feature = "embedded-hal")]
use core::convert::Infallible;

use crate::pac::GPIOTE;

/// Represents a GPIOTE channel.
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum GpioteChannel {
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4,
    C5 = 5,
    C6 = 6,
    C7 = 7,
}

/// The pulse edge used to trigger interrupts. (Polarity). Sets `GPIOTE_CONFIG`
/// register, `POLARITY` field. When In task mode: Operation to be performed on output
/// when OUT[n] task is triggered. When In event mode.Operation on input that shall trigger IN[n] event.
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum Edge {
    /// Task mode: No effect on pin from OUT[n] task. Event mode:
    /// no IN[n] event generated on pin activity.
    None = 0,
    /// "LoToHi". Task mode: Set pin from OUT[n] task. Event mode: Generate
    /// IN[n] event when rising edge on pin.
    Rising = 1,
    /// "HiToLo". Task mode: Clear pin from OUT[n] task. Event mode:
    /// Generate IN[n] event when falling edge on pin.
    Falling = 2,
    /// Task mode: Toggle pin from OUT[n]. Event mode: Generate
    /// IN[n] when any change on pin.
    Toggle = 3,
}

/// GPIOTE mode of operation for a specific pin. Sets `GPIOTE_CONFIG[n]` register.
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum GpioteMode {
    /// Disabled. Pin specified by PSEL will not be acquired by the
    /// GPIOTE module.
    Disabled = 0,
    /// The pin specified by PSEL will be configured as an input and
    /// the IN[n] event will be generated if operation specified in
    /// POLARITY occurs on the pin.
    Event = 1,
    /// The GPIO specified by PSEL will be configured as an output
    /// and triggering the SET[n], CLR[n] or OUT[n] task will
    /// perform the operation specified by POLARITY on the pin.
    /// When enabled as a task the GPIOTE module will acquire the
    /// pin and the pin can no longer be written as a regular output
    /// pin from the GPIO module.
    Task = 3,
}

/// A GPIO port with up to 32 pins.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Port {
    /// Port 0, available on all nRF52 and nRF51 MCUs.
    P0 = 0,
    /// Port 1, only available on some nRF52 MCUs.
    #[cfg(any(feature = "52833", feature = "52840", feature = "53"))]
    P1 = 1,
    #[cfg(feature = "53")]
    P2 = 2,
    #[cfg(feature = "53")]
    P3 = 3,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for the `DETECTMODE` register
pub enum DetectMode {
    /// DETECT directly connected to PIN DETECT signals
    Default = 0,
    /// Use the latched LDETECT behavior
    Ldetect = 1,
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
/// Drive configuration. Values for `PIN_CONF` reg, `DRIVE` field.
pub enum Drive {
    /// Standard '0', standard '1'
    S0S1 = 0,
    /// High drive '0', standard '1
    H0S1 = 1,
    /// Standard '0', high drive '1'
    S0H1 = 2,
    /// High drive '0', high drive '1'
    H0H1 = 3,
    /// Disconnect '0' standard '1' (normally used for wired-or-connections)
    D0S1 = 4,
    /// Disconnect '0' high drive '1' (normally used for wired-or-connections)
    D0H1 = 5,
    /// Standard '0'. disconnect '1' (normally used for wired-or-connections)
    /// Use this setting for "open drain" outputs.
    S0D1 = 6,
    /// High drive '0' disconnect '1' (normally used for wired-or-connections)
    H0D1 = 7,
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
    pub port: Port,
    pub pin: u8,
}

impl Pin {
    /// Create a new pin, with a specific direction.
    pub fn new(port: Port, pin: u8, dir: Dir) -> Self {
        let mut result = Self { port, pin };

        result.dir(dir);
        result
    }

    /// Internal function used to access the register block.
    fn regs(&self) -> &gpio::RegisterBlock {
        let ptr = match self.port {
            Port::P0 => P0::ptr(),
            #[cfg(any(feature = "52833", feature = "52840", feature = "53"))]
            Port::P1 => P1::ptr(),
            #[cfg(feature = "53")]
            Port::P2 => P2::ptr(),
            #[cfg(feature = "53")]
            Port::P3 => P3::ptr(),
        };

        unsafe { &*ptr }
    }

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
    pub fn is_latched(&mut self) -> bool {
        !(self.regs().latch.read().bits() & (1 << self.pin) != 0)
    }

    /// Select between default DETECT signal behavior and LDETECT mode. Sets the `DETECTMODE` register.
    pub fn detect_mode(&mut self, det_mode: DetectMode) {
        match det_mode {
            DetectMode::Default => self
                .regs()
                .detectmode
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin)) }),
            DetectMode::Ldetect => self
                .regs()
                .detectmode
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin)) }),
        }
    }

    /// Set the pin's output direction. Sets the `DIRSET` or `DIRCLR` register.
    pub fn dir(&mut self, dir: Dir) {
        match dir {
            Dir::Input => {
                self.regs()
                    .dirclr
                    .write(|w| unsafe { w.bits(1 << self.pin) });

                self.input_buf(InputBuf::Connect);
            }
            Dir::Output => {
                self.regs()
                    .dirset
                    .write(|w| unsafe { w.bits(1 << self.pin) });

                self.input_buf(InputBuf::Disconnect);
            }
        }
    }

    /// Set internal pull resistor: Pull up, pull down, or disabled. Sets the `PIN_CNF`
    /// register, `PULL` field.
    pub fn pull(&mut self, pull: Pull) {
        self.regs().pin_cnf[self.pin as usize].modify(|_, w| unsafe { w.pull().bits(pull as u8) })
    }

    /// Connect or disconnect input buffer.
    /// A GPIO pin input buffer can be disconnected from the pin to enable power savings when the pin is not
    /// used as an input, see GPIO port and the GPIO pin details on page 148. Input buffers must be connected
    /// to get a valid input value in the IN register, and for the sense mechanism to get access to the pin.
    /// Sets `PIN_CNF` register, `INPUT` field.
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

    /// Enable a GPIO pin interrupt. Sets `GPIOTE_INTENSET` register.
    pub fn enable_interrupt(&mut self, edge: Edge, channel: GpioteChannel) {
        let gpiote_regs = unsafe { &*GPIOTE::ptr() };

        gpiote_regs.config[channel as usize].modify(|_, w| unsafe {
            w.mode().bits(GpioteMode::Event as u8);
            w.psel().bits(self.pin);
            w.port().bit(self.port as u8 != 0);
            w.polarity().bits(edge as u8)
        });

        gpiote_regs
            .intenset
            .write(|w| unsafe { w.bits(1 << channel as usize) })
    }

    /// Disable a GPIO pin interrupt. Sets `GPIOTE_INTENSET` register.
    pub fn disable_interrupt(&mut self, channel: GpioteChannel) {
        let gpiote_regs = unsafe { &*GPIOTE::ptr() };

        gpiote_regs
            .intenclr
            .write(|w| unsafe { w.bits(1 << channel as usize) })
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

/// Check if an event was generatred from the pin specified in `GPIOTE_CONFIG[n].PSTEL`.
/// Reads from `GPIOTE_EVENTS_IN[channel]` register.
pub fn event_triggered(channel: GpioteChannel) -> bool {
    let gpiote_regs = unsafe { &*GPIOTE::ptr() };
    gpiote_regs.events_in[channel as usize].read().bits() != 0
}

/// Resets GPIOTE channel interrupt. Sets `GPIOTE_EVENTS_IN[channel]` register.
pub fn clear_interrupt(channel: GpioteChannel) {
    let gpiote_regs = unsafe { &*GPIOTE::ptr() };
    gpiote_regs.events_in[channel as usize].write(|w| unsafe { w.bits(0) });
}

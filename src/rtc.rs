//! Taken directly from [nrf-hal](https://github.com/nrf-rs/nrf-hal)
//!
//! A high level interface for RTC peripherals.

use core::ops::Deref;

#[cfg(feature = "9160")]
use crate::pac::{rtc0_ns as rtc0, Interrupt, RTC0_NS as RTC0, RTC1_NS as RTC1};

#[cfg(not(feature = "9160"))]
use crate::pac::{rtc0, Interrupt, RTC0, RTC1};

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use crate::pac::RTC2;

/// An opaque high level interface to an RTC peripheral.
pub struct Rtc<T> {
    periph: T,
}

/// Interrupts/Events that can be generated by the RTCn peripheral.
pub enum RtcInterrupt {
    Tick,
    Overflow,
    Compare0,
    Compare1,
    Compare2,
    Compare3,
}

/// Compare registers available on the RTCn.
pub enum RtcCompareReg {
    Compare0,
    Compare1,
    Compare2,
    Compare3,
}

impl<T> Rtc<T>
    where
        T: Instance,
{
    /// Creates a new RTC peripheral instance with a 12 bits prescaler.
    /// fRTC = 32_768 / (`prescaler` + 1 )
    pub fn new(rtc: T, prescaler: u32) -> Result<Self, Error> {
        if prescaler >= (1 << 12) {
            return Err(Error::PrescalerOutOfRange);
        }

        unsafe { rtc.prescaler.write(|w| w.bits(prescaler)) };

        Ok(Rtc { periph: rtc })
    }

    /// Enable/start the Real Time Counter.
    pub fn enable_counter(&self) {
        unsafe {
            self.periph.tasks_start.write(|w| w.bits(1));
        }
    }

    /// Disable/stop the Real Time Counter.
    pub fn disable_counter(&self) {
        unsafe {
            self.periph.tasks_stop.write(|w| w.bits(1));
        }
    }

    /// Enable the generation of a hardware interrupt from a given stimulus.
    pub fn enable_interrupt(&mut self, int: RtcInterrupt) {
        match int {
            RtcInterrupt::Tick => self.periph.intenset.write(|w| w.tick().set()),
            RtcInterrupt::Overflow => self.periph.intenset.write(|w| w.ovrflw().set()),
            RtcInterrupt::Compare0 => self.periph.intenset.write(|w| w.compare0().set()),
            RtcInterrupt::Compare1 => self.periph.intenset.write(|w| w.compare1().set()),
            RtcInterrupt::Compare2 => self.periph.intenset.write(|w| w.compare2().set()),
            RtcInterrupt::Compare3 => self.periph.intenset.write(|w| w.compare3().set()),
        }
    }

    /// Disable the generation of a hardware interrupt from a given stimulus.
    pub fn disable_interrupt(&mut self, int: RtcInterrupt) {
        match int {
            RtcInterrupt::Tick => self.periph.intenclr.write(|w| w.tick().clear()),
            RtcInterrupt::Overflow => self.periph.intenclr.write(|w| w.ovrflw().clear()),
            RtcInterrupt::Compare0 => self.periph.intenclr.write(|w| w.compare0().clear()),
            RtcInterrupt::Compare1 => self.periph.intenclr.write(|w| w.compare1().clear()),
            RtcInterrupt::Compare2 => self.periph.intenclr.write(|w| w.compare2().clear()),
            RtcInterrupt::Compare3 => self.periph.intenclr.write(|w| w.compare3().clear()),
        }
    }

    /// Enable the generation of a hardware event from a given stimulus.
    pub fn enable_event(&mut self, evt: RtcInterrupt) {
        match evt {
            RtcInterrupt::Tick => self.periph.evtenset.write(|w| w.tick().set()),
            RtcInterrupt::Overflow => self.periph.evtenset.write(|w| w.ovrflw().set()),
            RtcInterrupt::Compare0 => self.periph.evtenset.write(|w| w.compare0().set()),
            RtcInterrupt::Compare1 => self.periph.evtenset.write(|w| w.compare1().set()),
            RtcInterrupt::Compare2 => self.periph.evtenset.write(|w| w.compare2().set()),
            RtcInterrupt::Compare3 => self.periph.evtenset.write(|w| w.compare3().set()),
        }
    }

    /// Disables the generation of a hardware event from a given stimulus.
    pub fn disable_event(&mut self, evt: RtcInterrupt) {
        match evt {
            RtcInterrupt::Tick => self.periph.evtenclr.write(|w| w.tick().clear()),
            RtcInterrupt::Overflow => self.periph.evtenclr.write(|w| w.ovrflw().clear()),
            RtcInterrupt::Compare0 => self.periph.evtenclr.write(|w| w.compare0().clear()),
            RtcInterrupt::Compare1 => self.periph.evtenclr.write(|w| w.compare1().clear()),
            RtcInterrupt::Compare2 => self.periph.evtenclr.write(|w| w.compare2().clear()),
            RtcInterrupt::Compare3 => self.periph.evtenclr.write(|w| w.compare3().clear()),
        }
    }

    /// Checks if the given event has been triggered.
    pub fn is_event_triggered(&self, evt: RtcInterrupt) -> bool {
        let orig = match evt {
            RtcInterrupt::Tick => self.periph.events_tick.read().bits(),
            RtcInterrupt::Overflow => self.periph.events_ovrflw.read().bits(),
            RtcInterrupt::Compare0 => self.periph.events_compare[0].read().bits(),
            RtcInterrupt::Compare1 => self.periph.events_compare[1].read().bits(),
            RtcInterrupt::Compare2 => self.periph.events_compare[2].read().bits(),
            RtcInterrupt::Compare3 => self.periph.events_compare[3].read().bits(),
        };
        orig == 1
    }

    /// Resets the given event.
    pub fn reset_event(&self, evt: RtcInterrupt) {
        match evt {
            RtcInterrupt::Tick => {
                self.periph.events_tick.write(|w| unsafe { w.bits(0) });
            }
            RtcInterrupt::Overflow => {
                self.periph.events_ovrflw.write(|w| unsafe { w.bits(0) });
            }
            RtcInterrupt::Compare0 => {
                self.periph.events_compare[0].write(|w| unsafe { w.bits(0) });
            }
            RtcInterrupt::Compare1 => {
                self.periph.events_compare[1].write(|w| unsafe { w.bits(0) });
            }
            RtcInterrupt::Compare2 => {
                self.periph.events_compare[2].write(|w| unsafe { w.bits(0) });
            }
            RtcInterrupt::Compare3 => {
                self.periph.events_compare[3].write(|w| unsafe { w.bits(0) });
            }
        };
    }

    /// Set a timeout for the RTC, in seconds.
    pub fn set_timeout(&mut self, reg: RtcCompareReg, timeout: f32) -> Result<(), Error> {
        self.set_compare(reg, (2_768 as f32 * timeout) as u32)
    }

    /// Set the compare value of a given register. The compare registers have a width
    /// of 24 bits.
    pub fn set_compare(&mut self, reg: RtcCompareReg, val: u32) -> Result<(), Error> {
        if val >= (1 << 24) {
            return Err(Error::CompareOutOfRange);
        }

        let reg = match reg {
            RtcCompareReg::Compare0 => 0,
            RtcCompareReg::Compare1 => 1,
            RtcCompareReg::Compare2 => 2,
            RtcCompareReg::Compare3 => 3,
        };

        unsafe {
            self.periph.cc[reg].write(|w| w.bits(val));
        }

        Ok(())
    }

    /// Obtain the current value of the Real Time Counter, 24 bits of range.
    pub fn get_counter(&self) -> u32 {
        self.periph.counter.read().bits()
    }

    /// Clear the Real Time Counter.
    pub fn clear_counter(&self) {
        unsafe {
            self.periph.tasks_clear.write(|w| w.bits(1));
        }
    }

    /// Sets the Real Time Counter value to 0xFFFFF0, to allow tests of the overflow condition.
    /// The overflow event occurs when the Real Time Counter overflows from 0xFFFFFF to 0.
    pub fn trigger_overflow(&self) {
        unsafe {
            self.periph.tasks_trigovrflw.write(|w| w.bits(1));
        }
    }

    /// Destructure the high level interface. Does not reset any configuration made
    /// to the given RTC peripheral.
    pub fn release(self) -> T {
        self.periph
    }
}

/// Error types associated with the RTC peripheral interface.
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    PrescalerOutOfRange,
    CompareOutOfRange,
}

/// Implemented by all RTC instances.
pub trait Instance: Deref<Target = rtc0::RegisterBlock> + sealed::Sealed {
    /// The interrupt associated with this RTC instance.
    const INTERRUPT: Interrupt;
}

mod sealed {
    pub trait Sealed {}
}

macro_rules! impl_instance {
    ($($name:ident,)*) => {
        $(
            impl Instance for $name {
                const INTERRUPT: Interrupt = Interrupt::$name;
            }
            impl sealed::Sealed for $name {}
        )*
    }
}

impl_instance!(RTC0, RTC1,);

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
impl_instance!(RTC2,);

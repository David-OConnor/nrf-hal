//! Support for the Timer Peripheral.
//! See nRF52840 Product Specification, section 6.30

// todo: PWM, Compare beyond 3, NRF53-only timers, (eg past 0, 1, 2)

use core::ops::Deref;

#[cfg(feature = "9160")]
use crate::pac::{
    generic::Reg, timer0_ns::RegisterBlock as RegBlock0, Interrupt, TIMER0_NS as TIMER0,
    TIMER1_NS as TIMER1, TIMER2_NS as TIMER2,
};

#[cfg(not(feature = "9160"))]
use crate::pac::{timer0::RegisterBlock as RegBlock0, Interrupt, TIMER0, TIMER1, TIMER2};

#[cfg(not(any(feature = "52810", feature = "52811")))]
use crate::pac::{TIMER3, TIMER4};

#[cfg(any(feature = "52832", feature = "52840"))]
use crate::pac::timer3::{
    RegisterBlock as RegBlock3, EVENTS_COMPARE as EVENTS_COMPARE3, TASKS_CAPTURE as TASKS_CAPTURE3,
};

#[cfg(feature = "embedded-hal")]
use embedded_hal::timer;

#[cfg(feature = "embedded-hal")]
use nb::{self, block};

#[cfg(feature = "embedded-hal")]
use void::Void;

/// 52840 RM, section 6.30: TIMER can operate in two modes: Timer mode and Counter mode. In both modes, TIMER is started by
/// triggering the START task, and stopped by triggering the STOP task. After the timer is stopped the timer
/// can resume timing/counting by triggering the START task again. When timing/counting is resumed, the
/// timer will continue from the value it had prior to being stopped
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum TimerMode {
    /// Select Timer mode
    Timer = 0,
    /// Select Low Power Counter mode
    Counter = 2,
}

/// Configure the number of bits used by the TIMER.
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum TimerBitMode {
    /// 16 bit timer bit width
    B16 = 0,
    /// 8 bit timer bit width
    B8 = 1,
    /// 24 bit timer bit width
    B24 = 2,
    /// 32 bit timer bit width
    B32 = 3,
}

/// Configure timer shortcuts between events and tasks. This can allow
/// a timer event to automatically stop or clear the timer.
#[derive(Copy, Clone)]
pub enum TimerShortcut {
    /// Stop the timer on compare event.
    Stop,
    /// Clear the timer on compare event.
    Clear,
    /// Stop and clear the timer on compare event.
    StopClear,
    /// Don't stop or clear the timer on compare event.
    Disabled,
}

/// Represents a Timer peripheral.
pub struct Timer<R> {
    pub regs: R,
    // mode: TimerMode,
}

impl<R> Timer<R>
where
    R: Deref<Target = RegBlock0>,
{
    /// Initialize a Timer peripheral. `freq` is in Hz. By default, configures in 32-bit mode.
    pub fn new(regs: R, mode: TimerMode, freq: f32, compare_num: usize) -> Self {
        regs.mode.write(|w| unsafe { w.bits(mode as u8 as u32) });
        let mut result = Self { regs };
        result.bit_mode(TimerBitMode::B32);

        result.set_freq(freq, compare_num);

        result
    }

    /// Configure the number of bits used by the TIMER.
    pub fn bit_mode(&mut self, mode: TimerBitMode) {
        self.regs
            .bitmode
            .write(|w| unsafe { w.bits(mode as u8 as u32) });
    }

    /// Start the timer.
    pub fn start(&mut self) {
        self.regs.tasks_start.write(|w| unsafe { w.bits(1) });
    }

    /// Stop the timer. Note that you may be able to use the `shortcut` method instead of
    /// calling this explicitly.
    pub fn stop(&mut self) {
        self.regs.tasks_stop.write(|w| unsafe { w.bits(1) });
    }

    /// RM, section 6.30: The Counter register can be cleared by triggering the CLEAR task.
    /// This will explicitly set the internal value to zero. Note that you may be able to
    /// use the `shortcut` method instead of calling this explicitly.
    pub fn clear(&mut self) {
        self.regs.tasks_clear.write(|w| unsafe { w.bits(1) });
    }

    /// Set the timer period, in seconds. Overrides the period or frequency set
    /// in the constructor.
    pub fn set_period(&mut self, time: f32, compare_num: usize) {
        assert!(time > 0.);
        self.set_freq(1. / time, compare_num);
    }

    /// Set the timer frequency, in Hz. Overrides the period or frequency set
    /// in the constructor.
    pub fn set_freq(&mut self, freq: f32, compare_num: usize) {
        // RM: f_TIMER = 16 MHz / 2^PRESCALER
        // Taking CC into account: f_TIMER = 16 MHz / (2^PRESCALER * cc)

        // todo: Refine these ranges.
        // There are multiple valid prescaler values for each frequency, but in general,
        // we get more precision by picking an appropriate one.
        // Note that this algorithm is subjective, and there may be ways to get
        // more precise settings.
        // let prescaler = match freq {
        //     0.0..1. => 9,
        //     0.0.. => 8,
        //     0.0.. => 7,
        //     0.0.. => 6,
        //     0.0.. => 5,
        //     0.0.. => 4,
        //     0.0.. => 3,
        //     0.0.. => 2,
        //     0.0.. => 1,
        //     0.0.. => 0,
        //     _ => panic!("Frequency must be between 0. and asdf."),
        // };
        let prescaler = 1; // todo temp hardset.

        // 16Mhz / timer = 2*psc * cc
        let compare_val = (16_000_000 as f32 / (freq * 2_u32.pow(prescaler) as f32)) as u32;

        self.set_prescaler(prescaler);
        self.set_capture_compare(compare_num, compare_val);
    }

    /// Directly set the prescaler. Use this as a faster method for changing frequency,
    /// with the prescaler pre-calculated.
    pub fn set_prescaler(&mut self, prescaler: u32) {
        assert!(prescaler <= 9);
        // PRESCALER on page 464 and BITMODE on page 463 must only be updated when the timer is stopped.
        // If these registers are updated while the timer is started, unpredictable behavior may occur.
        // Ideally, we'd re-renable after, but we have no way of checking if it's running.
        self.stop();

        self.regs.prescaler.write(|w| unsafe { w.bits(prescaler) });
    }

    /// Enable the Timer interrupt for a given compar number.
    pub fn enable_interrupt(&mut self, compare_num: u8) {
        self.regs.intenset.write(|w| match compare_num {
            0 => w.compare0().set(),
            1 => w.compare1().set(),
            2 => w.compare2().set(),
            3 => w.compare3().set(),
            // 4 => w.compare4().set(),
            // 5=> w.compare5().set(),
            _ => panic!("compare number must be 0 - 5"),
            // todo: compare4 and 5 for the other timers??
        });
    }

    /// Disable an interrupt. (Note: This calls the INTENCLR register).
    pub fn disable_interrupt(&mut self, compare_num: usize) {
        self.regs.intenclr.write(|w| match compare_num {
            0 => w.compare0().clear(),
            1 => w.compare1().clear(),
            2 => w.compare2().clear(),
            3 => w.compare3().clear(),
            _ => panic!("compare number must be 0 - 5"),
        });
        // todo: compare4 and 5 for the other timers??
    }

    /// Clears interrupt associated with this timer for a given compare number.
    /// Note: This does not affect the INTENCLR register; it resets the appropriate `EVENTS_COMPARE`
    /// register.
    /// Clears both the INTENCLR register, and resets the appropriate EVENTS_COMPARE
    /// register.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished. For example, place this at the top of your timer's
    /// interrupt handler.
    pub fn clear_interrupt(&mut self, compare_num: usize) {
        self.regs.events_compare[compare_num].reset();
        // todo: compare4 and 5 for the other timers??
    }

    /// Set up a shortcut associated with an event. This can allow a timer event to
    /// automatically stop or clear the timer.
    pub fn shortcut(&mut self, shortcut: TimerShortcut, compare_num: usize) {
        // todo: DRY
        self.regs.shorts.modify(|_, w| match compare_num {
            0 => match shortcut {
                TimerShortcut::Stop => {
                    w.compare0_clear().disabled();
                    w.compare0_stop().enabled()
                }
                TimerShortcut::Clear => {
                    w.compare0_clear().enabled();
                    w.compare0_stop().disabled()
                }
                TimerShortcut::StopClear => {
                    w.compare0_clear().enabled();
                    w.compare0_stop().enabled()
                }
                TimerShortcut::Disabled => {
                    w.compare0_clear().disabled();
                    w.compare0_stop().disabled()
                }
            },
            1 => match shortcut {
                TimerShortcut::Stop => {
                    w.compare1_clear().disabled();
                    w.compare1_stop().enabled()
                }
                TimerShortcut::Clear => {
                    w.compare1_clear().enabled();
                    w.compare1_stop().disabled()
                }
                TimerShortcut::StopClear => {
                    w.compare1_clear().enabled();
                    w.compare1_stop().enabled()
                }
                TimerShortcut::Disabled => {
                    w.compare1_clear().disabled();
                    w.compare1_stop().disabled()
                }
            },
            2 => match shortcut {
                TimerShortcut::Stop => {
                    w.compare2_clear().disabled();
                    w.compare2_stop().enabled()
                }
                TimerShortcut::Clear => {
                    w.compare2_clear().enabled();
                    w.compare2_stop().disabled()
                }
                TimerShortcut::StopClear => {
                    w.compare2_clear().enabled();
                    w.compare2_stop().enabled()
                }
                TimerShortcut::Disabled => {
                    w.compare2_clear().disabled();
                    w.compare2_stop().disabled()
                }
            },
            3 => match shortcut {
                TimerShortcut::Stop => {
                    w.compare3_clear().disabled();
                    w.compare3_stop().enabled()
                }
                TimerShortcut::Clear => {
                    w.compare3_clear().enabled();
                    w.compare3_stop().disabled()
                }
                TimerShortcut::StopClear => {
                    w.compare3_clear().enabled();
                    w.compare3_stop().enabled()
                }
                TimerShortcut::Disabled => {
                    w.compare3_clear().disabled();
                    w.compare3_stop().disabled()
                }
            },
            _ => panic!("compare number must be 0 - 5"),
        });
        // todo: compare4 and 5 for the other timers??
    }

    /// Set capture task. Enable the trigger on a specific CC[n] register.
    /// TIMER implements one capture task for every available capture/compare register.
    /// Every time the CAPTURE[n] task is triggered, the Counter value is copied to the CC[n] register.
    pub fn capture_task(&mut self, compare_num: usize) {
        // assert!(cc_num <= 5);
        self.regs.tasks_capture[compare_num].write(|w| unsafe { w.bits(1) });
    }

    // /// Set compare event.
    // /// TIMER implements one COMPARE event for every available capture/compare register.
    // /// A COMPARE event is generated when the Counter is incremented and then becomes equal to the
    // /// value specified in one of the capture compare registers. When the Counter value becomes equal to the
    // /// value specified in a capture compare register CC[n], the corresponding compare event COMPARE[n] is
    // /// generated.
    // /// BITMODE on page 463 specifies how many bits of the Counter register and the capture/compare
    // /// register that are used when the comparison is performed. Other bits will be ignored.
    // pub fn compare_event(&mut self, compare_num_num: usize) {
    //     assert!(cc_num <= 5);
    //     self.regs.events_compare[cc_num].write(|w| unsafe { w.bits(1) });
    // }

    /// Set the Capture/Compare register for a given value. Only the number of bits indicated by
    /// BITMODE will be used by the TIMER. (Our constructor always sets 32 bits).
    pub fn set_capture_compare(&mut self, compare_num: usize, value: u32) {
        // assert!(cc_num <= 5);
        self.regs.cc[compare_num].write(|w| unsafe { w.bits(value) });
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> timer::Periodic for Timer<R> {}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> timer::CountDown for Timer<R>
where
    R: Deref<Target = RegBlock0>,
{
    type Time = f32;

    fn start<F: Into<f32>>(&mut self, freq: F) {
        self.set_freq(freq.into(), 0);
        Timer::start(self);
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.regs.events_compare[0].read().bits() == 0 {
            // EVENTS_COMPARE has not been triggered yet
            return Err(nb::Error::WouldBlock);
        }

        // Reset the event, otherwise it will always read `1` from now on.
        self.clear_interrupt(0);

        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<R> timer::Cancel for Timer<R>
where
    R: Deref<Target = RegBlock0>,
{
    type Error = ();

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.stop();
        self.clear_interrupt(0);

        Ok(())
    }
}

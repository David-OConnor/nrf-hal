//! HAL interface to the PWM peripheral.
//!
//! The pulse with modulation (PWM) module enables the generation of pulse width modulated signals on GPIO.

#[cfg(not(any(feature = "9160")))]
use crate::pac::pwm0::*;
#[cfg(any(feature = "9160"))]
use crate::pac::pwm0_ns::*;

use crate::{
    gpio::Pin,
    pac::Interrupt,
    target_constants::{SRAM_LOWER, SRAM_UPPER},
};
use core::{
    cell::Cell,
    ops::Deref,
    sync::atomic::{compiler_fence, Ordering},
};
use embedded_dma::*;

const MAX_SEQ_LEN: usize = 0x7FFF;

/// A safe wrapper around the raw peripheral.
pub struct Pwm<T: Instance> {
    pub regs: T,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Selects operating mode of the wave counter. 52833 PS, section 6.16.4.14
pub enum PwmMode {
    /// Up counter, edge-aligned PWM duty cycle
    Up = 0,
    /// Up and down counter, center-aligned PWM duty cycle
    UpAndDown = 1,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// See 52833 PS, section 6.16.5.16
pub enum Prescaler {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div32 = 5,
    Div64 = 6,
    Div128 = 7,
}

impl Prescaler {
    /// Return the PWM clock speed, based on a 16Mhz system clock.
    fn clock_speed(&self) -> u32 {
        match self {
            Self::Div1 => 16_000_000,
            Self::Div2 => 8_000_000,
            Self::Div4 => 4_000_000,
            Self::Div8 => 2_000_000,
            Self::Div16 => 1_000_000,
            Self::Div32 => 500_000,
            Self::Div64 => 250_000,
            Self::Div128 => 125_000,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// How a sequence is read from RAM and spread to the compare registerSee 52833 PS,
/// section 6.16.5.17: Decoder. Sets DECODER register, LOAD field.
pub enum DecoderLoad {
    /// 1st half word (16-bit) used in all PWM channels 0..3
    Common = 0,
    /// 1st half word (16-bit) used in channel 0..1; 2nd word in
    /// channel 2..3
    Grouped = 1,
    /// 1st half word (16-bit) in ch.0; 2nd in ch.1; ...; 4th in ch.3
    Individual = 2,
    /// 1st half word (16-bit) in ch.0; 2nd in ch.1; ...; 4th in
    /// COUNTERTOP
    WaveForm = 3,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Selects source for advancing the active sequence. See 52833 PS, section 6.16.5.17: Decoder.
/// Sets DECODER register, MODE field.
pub enum DecoderMode {
    /// SEQ[n].REFRESH is used to determine loading internal
    /// compare registers
    RefreshCount = 0,
    /// NEXTSTEP task causes a new value to be loaded to internal
    /// compare registers
    NextStep = 1,
}

pub struct PwmConfig {
    /// Selects operating mode of the wave counter. Defaults to Up.
    pub mode: PwmMode,
    /// Prescaler of the system clock. Defaults to 1.
    pub prescaler: Prescaler,
    /// Number of playbacks of a loop. Defaults to 0.
    pub loop_: u16,
    /// How a sequence is read from RAM and spread to the
    /// compare register. Defaults to individual.
    pub decoder_load: DecoderLoad,
    /// Selects source for advancing the active sequence. Deafults to refresh count.
    pub decoder_mode: DecoderMode,
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            mode: PwmMode::Up,
            prescaler: Prescaler::Div1,
            loop_: 0,
            decoder_load: DecoderLoad::Individual,
            decoder_mode: DecoderMode::RefreshCount,
        }
    }
}

impl<T> Pwm<T>
where
    T: Instance,
{
    /// Takes ownership of the peripheral and applies sane defaults.
    /// See PS, section: 6.16.1 Wave counter, for similar C code.
    pub fn new(regs: T, freq: f32, cfg: PwmConfig) -> Pwm<T> {
        regs.enable.write(|w| w.enable().enabled());
        regs.mode.write(|w| w.updown().bit(cfg.mode as u8 != 0));
        regs.prescaler.write(|w| w.prescaler().bits(cfg.prescaler as u8));
        regs.loop_.write(|w| unsafe { w.cnt().bits(cfg.loop_) });

        regs.decoder.write(|w| {
            w.load().bits(cfg.decoder_load as u8);
            w.mode().bit(cfg.decoder_mode as u8 != 0)
        });

        regs.seq0.refresh.write(|w| unsafe { w.bits(0) });
        regs.seq0.enddelay.write(|w| unsafe { w.bits(0) });
        regs.seq1.refresh.write(|w| unsafe { w.bits(0) });
        regs.seq1.enddelay.write(|w| unsafe { w.bits(0) });

        let mut result = Self { regs };

        result.set_freq(freq);
        result
    }

    /// Sets the PWM clock prescaler.
    #[inline(always)]
    pub fn set_prescaler(&mut self, div: Prescaler) {
        self.regs.prescaler.write(|w| w.prescaler().bits(div as u8));
    }

    /// Gets the PWM clock prescaler.
    #[inline(always)]
    pub fn prescaler(&self) -> Prescaler {
        match self.regs.prescaler.read().prescaler().bits() {
            0 => Prescaler::Div1,
            1 => Prescaler::Div2,
            2 => Prescaler::Div4,
            3 => Prescaler::Div8,
            4 => Prescaler::Div16,
            5 => Prescaler::Div32,
            6 => Prescaler::Div64,
            7 => Prescaler::Div128,
            _ => unreachable!(),
        }
    }

    /// Sets the maximum duty cycle value.
    #[inline(always)]
    pub fn set_counter_top(&mut self, duty: u16) {
        self.regs
            .countertop
            .write(|w| unsafe { w.countertop().bits(duty.min(32767u16)) });
    }
    /// Returns the maximum duty cycle value.
    #[inline(always)]
    pub fn counter_top(&self) -> u16 {
        self.regs.countertop.read().countertop().bits()
    }

    /// Sets the PWM output frequency, in Hz.
    #[inline(always)]
    pub fn set_freq(&mut self, freq: f32) {
        // PWM period: TPWM(Up)= TPWM_CLK * COUNTERTOP (From PS)
        // countertop = clock x period = clk / freq

        let counter_top = self.prescaler().clock_speed() as f32 / freq;

        match self.counter_mode() {
            CounterMode::Up => self.set_counter_top((counter_top as u32).min(32767) as u16),
            CounterMode::UpAndDown => self.set_counter_top((counter_top as u32 / 2).min(32767) as u16),
        };
    }

    /// Sets the PWM output period, in seconds.
    pub fn set_period(&mut self, period: f32) {
        self.set_freq(1. / period);
    }

    // /// Returns the PWM output frequency.
    // #[inline(always)]
    // pub fn period(&self) -> f32 {
    //     let max_duty = self.max_duty() as u32;
    //     let freq = match self.prescaler() {
    //         Prescaler::Div1 => 16_000_000 / max_duty,
    //         Prescaler::Div2 => 8_000_000 / max_duty,
    //         Prescaler::Div4 => 4_000_000 / max_duty,
    //         Prescaler::Div8 => 2_000_000 / max_duty,
    //         Prescaler::Div16 => 1_000_000 / max_duty,
    //         Prescaler::Div32 => 500_000 / max_duty,
    //         Prescaler::Div64 => 250_000 / max_duty,
    //         Prescaler::Div128 => 125_000 / max_duty,
    //     } as f32;
    //     match self.counter_mode() {
    //         CounterMode::Up => freq,
    //         CounterMode::UpAndDown => (freq / 2.),
    //     }
    // }

    /// Sets the associated output pin for the PWM channel.
    ///
    /// Modifying the pin configuration while the PWM instance is enabled is not recommended.
    #[inline(always)]
    pub fn set_output_pin(&self, channel: PwmChannel, pin: &Pin) -> &Self {
        self.regs.psel.out[channel as usize].write(|w| unsafe {
            w.port().bit(pin.port as u8 != 0);
            #[cfg(feature = "53")]
            w.port().bits(pin.port as u8);
            w.pin().bits(pin.pin);
            w.connect().connected()
        });
        self
    }

    /// Enables the PWM generator.
    #[inline(always)]
    pub fn enable(&self) {
        self.regs.enable.write(|w| w.enable().enabled());
    }

    /// Disables the PWM generator.
    #[inline(always)]
    pub fn disable(&self) {
        self.regs.enable.write(|w| w.enable().disabled());
    }

    /// Loads the first PWM value on all enabled channels from sequence n, and starts playing that sequence
    /// at the rate defined in SEQ[n]REFRESH and/or DECODER.MODE. Causes PWM generation to start if not
    /// running.
    #[inline(always)]
    pub fn start_seq_(&self, seq: usize) {
        self.regs.tasks_seqstart[seq].write(|w| w.tasks_seqstart().set_bit());
    }

    /// Enables a PWM channel.
    #[inline(always)]
    pub fn enable_channel(&self, channel: PwmChannel) -> &Self {
        self.regs.psel.out[channel as usize].modify(|_r, w| w.connect().connected());
        self
    }

    /// Disables a PWM channel.
    #[inline(always)]
    pub fn disable_channel(&self, channel: PwmChannel) -> &Self {
        self.regs.psel.out[channel as usize].modify(|_r, w| w.connect().disconnected());
        self
    }

    /// Enables a PWM group.
    #[inline(always)]
    pub fn enable_group(&self, group: Group) -> &Self {
        match group {
            Group::G0 => {
                self.regs.psel.out[0].modify(|_r, w| w.connect().connected());
                self.regs.psel.out[1].modify(|_r, w| w.connect().connected());
            }
            Group::G1 => {
                self.regs.psel.out[2].modify(|_r, w| w.connect().connected());
                self.regs.psel.out[3].modify(|_r, w| w.connect().connected());
            }
        }
        self
    }

    /// Disables a PWM group.
    #[inline(always)]
    pub fn disable_group(&self, group: Group) -> &Self {
        match group {
            Group::G0 => {
                self.regs.psel.out[0].modify(|_r, w| w.connect().disconnected());
                self.regs.psel.out[1].modify(|_r, w| w.connect().disconnected());
            }
            Group::G1 => {
                self.regs.psel.out[2].modify(|_r, w| w.connect().disconnected());
                self.regs.psel.out[3].modify(|_r, w| w.connect().disconnected());
            }
        }
        self
    }

    /// Cofigures how a sequence is read from RAM and is spread to the compare register.
    #[inline(always)]
    pub fn set_load_mode(&self, mode: DecoderLoad) -> &Self {
        self.regs.decoder.modify(|_, w| w.load().bits(mode as u8));
        if mode == DecoderLoad::WaveForm {
            self.disable_channel(PwmChannel::C3);
        } else {
            self.enable_channel(PwmChannel::C3);
        }
        self
    }

    /// Returns how a sequence is read from RAM and is spread to the compare register.
    #[inline(always)]
    pub fn load_mode(&self) -> DecoderLoad {
        match self.regs.decoder.read().load().bits() {
            0 => DecoderLoad::Common,
            1 => DecoderLoad::Grouped,
            2 => DecoderLoad::Individual,
            3 => DecoderLoad::WaveForm,
            _ => unreachable!(),
        }
    }

    /// Selects operating mode of the wave counter.
    #[inline(always)]
    pub fn set_counter_mode(&self, mode: CounterMode) -> &Self {
        self.regs.mode.write(|w| w.updown().bit(mode.into()));
        self
    }

    /// Returns selected operating mode of the wave counter.
    #[inline(always)]
    pub fn counter_mode(&self) -> CounterMode {
        match self.regs.mode.read().updown().bit() {
            false => CounterMode::Up,
            true => CounterMode::UpAndDown,
        }
    }

    /// Selects source for advancing the active sequence.
    #[inline(always)]
    pub fn set_step_mode(&self, mode: StepMode) -> &Self {
        self.regs.decoder.modify(|_r, w| w.mode().bit(mode.into()));
        self
    }

    /// Returns selected source for advancing the active sequence.
    #[inline(always)]
    pub fn step_mode(&self) -> StepMode {
        match self.regs.decoder.read().mode().bit() {
            false => StepMode::Auto,
            true => StepMode::NextStep,
        }
    }

    // Internal helper function that returns 15 bit duty cycle value.
    #[inline(always)]
    fn duty_on_value(&self, index: usize) -> u16 {
        let val = T::buffer().get()[index];
        let is_inverted = (val >> 15) & 1 == 0;
        match is_inverted {
            false => val,
            true => self.counter_top() - (val & 0x7FFF),
        }
    }

    // Internal helper function that returns 15 bit inverted duty cycle value.
    #[inline(always)]
    fn duty_off_value(&self, index: usize) -> u16 {
        let val = T::buffer().get()[index];
        let is_inverted = (val >> 15) & 1 == 0;
        match is_inverted {
            false => self.counter_top() - val,
            true => val & 0x7FFF,
        }
    }

    /// Sets duty cycle (15 bit) for all PWM channels.
    /// Will replace any ongoing sequence playback.
    pub fn set_duty_on_common(&self, duty: u16) {
        let mut buffer = T::buffer().get();
        buffer.copy_from_slice(&[duty.min(self.counter_top()) & 0x7FFF; 4][..]);
        T::buffer().set(buffer);
        self.one_shot();
        self.set_load_mode(DecoderLoad::Common);
        self.regs
            .seq0
            .ptr
            .write(|w| unsafe { w.bits(T::buffer().as_ptr() as u32) });
        self.regs.seq0.cnt.write(|w| unsafe { w.bits(1) });
        self.start_seq(Seq::Seq0);
    }

    /// API bandaid to deal with `set_duty_on_common` being the previous way to start.
    pub fn start(&mut self, duty: f32) {
        self.set_duty_on_common(
            (self.counter_top() as f32 * duty) as u16
        );
    }

    /// Sets inverted duty cycle (15 bit) for all PWM channels.
    /// Will replace any ongoing sequence playback.
    pub fn set_duty_off_common(&self, duty: u16) {
        let mut buffer = T::buffer().get();
        buffer.copy_from_slice(&[duty.min(self.counter_top()) | 0x8000; 4][..]);
        T::buffer().set(buffer);
        self.one_shot();
        self.set_load_mode(DecoderLoad::Common);
        self.regs
            .seq0
            .ptr
            .write(|w| unsafe { w.bits(T::buffer().as_ptr() as u32) });
        self.regs.seq0.cnt.write(|w| unsafe { w.bits(1) });
        self.start_seq(Seq::Seq0);
    }

    /// Returns the common duty cycle value for all PWM channels in `Common` load mode.
    #[inline(always)]
    pub fn duty_on_common(&self) -> u16 {
        self.duty_on_value(0)
    }

    /// Returns the inverted common duty cycle value for all PWM channels in `Common` load mode.
    #[inline(always)]
    pub fn duty_off_common(&self) -> u16 {
        self.duty_off_value(0)
    }

    /// Sets duty cycle (15 bit) for a PWM group.
    /// Will replace any ongoing sequence playback.
    pub fn set_duty_on_group(&self, group: Group, duty: u16) {
        let mut buffer = T::buffer().get();
        buffer[usize::from(group)] = duty.min(self.counter_top()) & 0x7FFF;
        T::buffer().set(buffer);
        self.one_shot();
        self.set_load_mode(DecoderLoad::Grouped);
        self.regs
            .seq0
            .ptr
            .write(|w| unsafe { w.bits(T::buffer().as_ptr() as u32) });
        self.regs.seq0.cnt.write(|w| unsafe { w.bits(2) });
        self.start_seq(Seq::Seq0);
    }

    /// Sets inverted duty cycle (15 bit) for a PWM group.
    /// Will replace any ongoing sequence playback.
    pub fn set_duty_off_group(&self, group: Group, duty: u16) {
        let mut buffer = T::buffer().get();
        buffer[usize::from(group)] = duty.min(self.counter_top()) | 0x8000;
        T::buffer().set(buffer);
        self.one_shot();
        self.set_load_mode(DecoderLoad::Grouped);
        self.regs
            .seq0
            .ptr
            .write(|w| unsafe { w.bits(T::buffer().as_ptr() as u32) });
        self.regs.seq0.cnt.write(|w| unsafe { w.bits(2) });
        self.start_seq(Seq::Seq0);
    }

    /// Returns duty cycle value for a PWM group.
    #[inline(always)]
    pub fn duty_on_group(&self, group: Group) -> u16 {
        self.duty_on_value(usize::from(group))
    }

    /// Returns inverted duty cycle value for a PWM group.
    #[inline(always)]
    pub fn duty_off_group(&self, group: Group) -> u16 {
        self.duty_off_value(usize::from(group))
    }

    /// Sets duty cycle (15 bit) for a PWM channel.
    /// Will replace any ongoing sequence playback and the other channels will return to their previously set value.
    pub fn set_duty_on(&self, channel: PwmChannel, duty: u16) {
        let mut buffer = T::buffer().get();
        buffer[channel as usize] = duty.min(self.counter_top()) & 0x7FFF;
        T::buffer().set(buffer);
        self.one_shot();
        self.set_load_mode(DecoderLoad::Individual);
        self.regs
            .seq0
            .ptr
            .write(|w| unsafe { w.bits(T::buffer().as_ptr() as u32) });
        self.regs.seq0.cnt.write(|w| unsafe { w.bits(4) });
        self.start_seq(Seq::Seq0);
    }

    /// Sets inverted duty cycle (15 bit) for a PWM channel.
    /// Will replace any ongoing sequence playback and the other channels will return to their previously set value.
    pub fn set_duty_off(&self, channel: PwmChannel, duty: u16) {
        let mut buffer = T::buffer().get();
        buffer[channel as usize] = duty.min(self.counter_top()) | 0x8000;
        T::buffer().set(buffer);
        self.one_shot();
        self.set_load_mode(DecoderLoad::Individual);
        self.regs
            .seq0
            .ptr
            .write(|w| unsafe { w.bits(T::buffer().as_ptr() as u32) });
        self.regs.seq0.cnt.write(|w| unsafe { w.bits(4) });
        self.start_seq(Seq::Seq0);
    }

    /// Returns the duty cycle value for a PWM channel.
    #[inline(always)]
    pub fn duty_on(&self, channel: PwmChannel) -> u16 {
        self.duty_on_value(channel as usize)
    }

    /// Returns the inverted duty cycle value for a PWM group.
    #[inline(always)]
    pub fn duty_off(&self, channel: PwmChannel) -> u16 {
        self.duty_off_value(channel as usize)
    }

    /// Sets number of playbacks of sequences.
    #[inline(always)]
    pub fn set_loop(&self, mode: Loop) {
        self.regs.loop_.write(|w| match mode {
            Loop::Disabled => w.cnt().disabled(),
            Loop::Times(n) => unsafe { w.cnt().bits(n) },
            Loop::Inf => unsafe { w.cnt().bits(2) },
        });
        self.regs.shorts.write(|w| match mode {
            Loop::Inf => w.loopsdone_seqstart0().enabled(),
            _ => w.loopsdone_seqstart0().disabled(),
        });
    }

    /// Looping disabled (stop at the end of the sequence).
    #[inline(always)]
    pub fn one_shot(&self) -> &Self {
        self.set_loop(Loop::Disabled);
        self
    }

    /// Loops playback of sequences indefinitely.
    #[inline(always)]
    pub fn loop_inf(&self) -> &Self {
        self.set_loop(Loop::Inf);
        self
    }

    /// Sets number of playbacks of sequences.
    #[inline(always)]
    pub fn repeat(&self, times: u16) -> &Self {
        self.set_loop(Loop::Times(times));
        self
    }

    /// Sets number of additional PWM periods between samples loaded into compare register.
    #[inline(always)]
    pub fn set_seq_refresh(&self, seq: Seq, periods: u32) -> &Self {
        match seq {
            Seq::Seq0 => self.regs.seq0.refresh.write(|w| unsafe { w.bits(periods) }),
            Seq::Seq1 => self.regs.seq1.refresh.write(|w| unsafe { w.bits(periods) }),
        }
        self
    }

    /// Sets number of additional PWM periods after the sequence ends.
    #[inline(always)]
    pub fn set_seq_end_delay(&self, seq: Seq, periods: u32) -> &Self {
        match seq {
            Seq::Seq0 => self.regs.seq0.enddelay.write(|w| unsafe { w.bits(periods) }),
            Seq::Seq1 => self.regs.seq1.enddelay.write(|w| unsafe { w.bits(periods) }),
        }
        self
    }

    /// Loads the first PWM value on all enabled channels from a sequence and starts playing that sequence.
    /// Causes PWM generation to start if not running.
    #[inline(always)]
    pub fn start_seq(&self, seq: Seq) {
        compiler_fence(Ordering::SeqCst);
        self.regs.enable.write(|w| w.enable().enabled());
        self.regs.tasks_seqstart[seq as usize].write(|w| unsafe { w.bits(1) });
        while self.regs.events_seqstarted[seq as usize].read().bits() == 0 {}
        self.regs.events_seqend[0].write(|w| w);
        self.regs.events_seqend[1].write(|w| w);
    }

    /// Steps by one value in the current sequence on all enabled channels, if the `NextStep` step mode is selected.
    /// Does not cause PWM generation to start if not running.
    #[inline(always)]
    pub fn next_step(&self) {
        self.regs.tasks_nextstep.write(|w| unsafe { w.bits(1) });
    }

    /// Stops PWM pulse generation on all channels at the end of current PWM period, and stops sequence playback.
    #[inline(always)]
    pub fn stop(&self) {
        compiler_fence(Ordering::SeqCst);
        self.regs.tasks_stop.write(|w| unsafe { w.bits(1) });
        while self.regs.events_stopped.read().bits() == 0 {}
    }

    /// Loads the given sequence buffers and optionally (re-)starts sequence playback.
    /// Returns a `PemSeq`, containing `Pwm<T>` and the buffers.
    #[allow(unused_mut)]
    pub fn load<B0, B1>(
        mut self,
        seq0_buffer: Option<B0>,
        seq1_buffer: Option<B1>,
        start: bool,
    ) -> Result<PwmSeq<T, B0, B1>, (Error, Pwm<T>, Option<B0>, Option<B1>)>
    where
        B0: ReadBuffer<Word = u16> + 'static,
        B1: ReadBuffer<Word = u16> + 'static,
    {
        if let Some(buf) = &seq0_buffer {
            let (ptr, len) = unsafe { buf.read_buffer() };
            if (ptr as usize) < SRAM_LOWER || (ptr as usize) > SRAM_UPPER {
                return Err((
                    Error::DMABufferNotInDataMemory,
                    self,
                    seq0_buffer,
                    seq1_buffer,
                ));
            }
            if len > MAX_SEQ_LEN {
                return Err((Error::BufferTooLong, self, seq0_buffer, seq1_buffer));
            }
            compiler_fence(Ordering::SeqCst);
            self.regs.seq0.ptr.write(|w| unsafe { w.bits(ptr as u32) });
            self.regs.seq0.cnt.write(|w| unsafe { w.bits(len as u32) });
            if start {
                self.start_seq(Seq::Seq0);
            }
        } else {
            self.regs.seq0.cnt.write(|w| unsafe { w.bits(0) });
        }

        if let Some(buf) = &seq1_buffer {
            let (ptr, len) = unsafe { buf.read_buffer() };
            if (ptr as usize) < SRAM_LOWER || (ptr as usize) > SRAM_UPPER {
                return Err((
                    Error::DMABufferNotInDataMemory,
                    self,
                    seq0_buffer,
                    seq1_buffer,
                ));
            }
            if len > MAX_SEQ_LEN {
                return Err((Error::BufferTooLong, self, seq0_buffer, seq1_buffer));
            }
            compiler_fence(Ordering::SeqCst);
            self.regs.seq1.ptr.write(|w| unsafe { w.bits(ptr as u32) });
            self.regs.seq1.cnt.write(|w| unsafe { w.bits(len as u32) });
            if start {
                self.start_seq(Seq::Seq1);
            }
        } else {
            self.regs.seq1.cnt.write(|w| unsafe { w.bits(0) });
        }

        Ok(PwmSeq {
            inner: Some(Inner {
                seq0_buffer,
                seq1_buffer,
                pwm: self,
            }),
        })
    }

    /// Enables interrupt triggering on the specified event.
    #[inline(always)]
    pub fn enable_interrupt(&self, event: PwmEvent) -> &Self {
        match event {
            PwmEvent::Stopped => self.regs.intenset.modify(|_r, w| w.stopped().set()),
            PwmEvent::LoopsDone => self.regs.intenset.modify(|_r, w| w.loopsdone().set()),
            PwmEvent::PwmPeriodEnd => self.regs.intenset.modify(|_r, w| w.pwmperiodend().set()),
            PwmEvent::SeqStarted(seq) => match seq {
                Seq::Seq0 => self.regs.intenset.modify(|_r, w| w.seqstarted0().set()),
                Seq::Seq1 => self.regs.intenset.modify(|_r, w| w.seqstarted1().set()),
            },
            PwmEvent::SeqEnd(seq) => match seq {
                Seq::Seq0 => self.regs.intenset.modify(|_r, w| w.seqend0().set()),
                Seq::Seq1 => self.regs.intenset.modify(|_r, w| w.seqend1().set()),
            },
        };
        self
    }

    /// Disables interrupt triggering on the specified event.
    #[inline(always)]
    pub fn disable_interrupt(&self, event: PwmEvent) -> &Self {
        match event {
            PwmEvent::Stopped => self.regs.intenclr.modify(|_r, w| w.stopped().clear()),
            PwmEvent::LoopsDone => self.regs.intenclr.modify(|_r, w| w.loopsdone().clear()),
            PwmEvent::PwmPeriodEnd => self.regs.intenclr.modify(|_r, w| w.pwmperiodend().clear()),
            PwmEvent::SeqStarted(seq) => match seq {
                Seq::Seq0 => self.regs.intenclr.modify(|_r, w| w.seqstarted0().clear()),
                Seq::Seq1 => self.regs.intenclr.modify(|_r, w| w.seqstarted1().clear()),
            },
            PwmEvent::SeqEnd(seq) => match seq {
                Seq::Seq0 => self.regs.intenclr.modify(|_r, w| w.seqend0().clear()),
                Seq::Seq1 => self.regs.intenclr.modify(|_r, w| w.seqend1().clear()),
            },
        };
        self
    }

    /// Checks if an event has been triggered.
    #[inline(always)]
    pub fn is_event_triggered(&self, event: PwmEvent) -> bool {
        match event {
            PwmEvent::Stopped => self.regs.events_stopped.read().bits() != 0,
            PwmEvent::LoopsDone => self.regs.events_loopsdone.read().bits() != 0,
            PwmEvent::PwmPeriodEnd => self.regs.events_pwmperiodend.read().bits() != 0,
            PwmEvent::SeqStarted(seq) => {
                self.regs.events_seqstarted[seq as usize].read().bits() != 0
            }
            PwmEvent::SeqEnd(seq) => self.regs.events_seqend[seq as usize].read().bits() != 0,
        }
    }

    /// Marks event as handled.
    #[inline(always)]
    pub fn reset_event(&self, event: PwmEvent) {
        match event {
            PwmEvent::Stopped => self.regs.events_stopped.write(|w| w),
            PwmEvent::LoopsDone => self.regs.events_loopsdone.write(|w| w),
            PwmEvent::PwmPeriodEnd => self.regs.events_pwmperiodend.write(|w| w),
            PwmEvent::SeqStarted(seq) => self.regs.events_seqstarted[seq as usize].write(|w| w),
            PwmEvent::SeqEnd(seq) => self.regs.events_seqend[seq as usize].write(|w| w),
        }
    }

    // /// Returns reference to `Stopped` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_stopped(&self) -> &EVENTS_STOPPED {
    //     &self.regs.events_stopped
    // }
    //
    // /// Returns reference to `LoopsDone` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_loops_done(&self) -> &EVENTS_LOOPSDONE {
    //     &self.regs.events_loopsdone
    // }
    //
    // /// Returns reference to `PwmPeriodEnd` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_pwm_period_end(&self) -> &EVENTS_PWMPERIODEND {
    //     &self.regs.events_pwmperiodend
    // }
    //
    // /// Returns reference to `Seq0 End` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_seq0_end(&self) -> &EVENTS_SEQEND {
    //     &self.regs.events_seqend[0]
    // }
    //
    // /// Returns reference to `Seq1 End` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_seq1_end(&self) -> &EVENTS_SEQEND {
    //     &self.regs.events_seqend[1]
    // }
    //
    // /// Returns reference to `Seq0 Started` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_seq0_started(&self) -> &EVENTS_SEQSTARTED {
    //     &self.regs.events_seqstarted[0]
    // }
    //
    // /// Returns reference to `Seq1 Started` event endpoint for PPI.
    // #[inline(always)]
    // pub fn event_seq1_started(&self) -> &EVENTS_SEQSTARTED {
    //     &self.regs.events_seqstarted[1]
    // }
    //
    // /// Returns reference to `Seq0 Start` task endpoint for PPI.
    // #[inline(always)]
    // pub fn task_start_seq0(&self) -> &TASKS_SEQSTART {
    //     &self.regs.tasks_seqstart[0]
    // }
    //
    // /// Returns reference to `Seq1 Started` task endpoint for PPI.
    // #[inline(always)]
    // pub fn task_start_seq1(&self) -> &TASKS_SEQSTART {
    //     &self.regs.tasks_seqstart[1]
    // }
    //
    // /// Returns reference to `NextStep` task endpoint for PPI.
    // #[inline(always)]
    // pub fn task_next_step(&self) -> &TASKS_NEXTSTEP {
    //     &self.regs.tasks_nextstep
    // }
    //
    // /// Returns reference to `Stop` task endpoint for PPI.
    // #[inline(always)]
    // pub fn task_stop(&self) -> &TASKS_STOP {
    //     &self.regs.tasks_stop
    // }
}

/// A Pwm sequence wrapper
pub struct PwmSeq<T: Instance, B0, B1> {
    inner: Option<Inner<T, B0, B1>>,
}

struct Inner<T: Instance, B0, B1> {
    seq0_buffer: Option<B0>,
    seq1_buffer: Option<B1>,
    pwm: Pwm<T>,
}

impl<T: Instance, B0, B1> PwmSeq<T, B0, B1>
where
    B0: ReadBuffer<Word = u16> + 'static,
    B1: ReadBuffer<Word = u16> + 'static,
{
    /// Returns the wrapped contents.
    #[inline(always)]
    pub fn split(mut self) -> (Option<B0>, Option<B1>, Pwm<T>) {
        compiler_fence(Ordering::SeqCst);
        let inner = self
            .inner
            .take()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        (inner.seq0_buffer, inner.seq1_buffer, inner.pwm)
    }

    /// Stops PWM generation.
    #[inline(always)]
    pub fn stop(&self) {
        let inner = self
            .inner
            .as_ref()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        inner.pwm.stop();
    }

    /// Starts playing the given sequence.
    #[inline(always)]
    pub fn start_seq(&self, seq: Seq) {
        let inner = self
            .inner
            .as_ref()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        inner.pwm.start_seq(seq);
    }

    /// Checks if the given event has been triggered.
    #[inline(always)]
    pub fn is_event_triggered(&self, event: PwmEvent) -> bool {
        let inner = self
            .inner
            .as_ref()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        inner.pwm.is_event_triggered(event)
    }

    /// Marks the given event as handled.
    #[inline(always)]
    pub fn reset_event(&self, event: PwmEvent) {
        let inner = self
            .inner
            .as_ref()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        inner.pwm.reset_event(event)
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<T: Instance> embedded_hal::Pwm for Pwm<T> {
    type Channel = PwmChannel;
    type Duty = u16;
    type Time = Hertz;

    fn enable(&mut self, channel: Self::Channel) {
        self.enable_channel(channel);
    }

    fn disable(&mut self, channel: Self::Channel) {
        self.disable_channel(channel);
    }

    fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
        self.duty_on(channel)
    }

    fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
        self.set_duty_on(channel, duty);
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.counter_top()
    }

    fn get_period(&self) -> Self::Time {
        self.period()
    }

    fn set_period<P>(&mut self, period: P)
    where
        P: Into<Self::Time>,
    {
        Self::set_period(self, period.into());
    }
}

#[derive(PartialEq, Clone, Copy)]
#[repr(usize)]
pub enum PwmChannel {
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
}

#[derive(PartialEq, Clone, Copy)]
pub enum Group {
    G0,
    G1,
}
impl From<Group> for usize {
    fn from(variant: Group) -> Self {
        variant as _
    }
}

#[derive(PartialEq, Clone, Copy)]
#[repr(usize)]
pub enum Seq {
    Seq0 = 0,
    Seq1 = 1,
}

#[derive(PartialEq, Clone, Copy)]
pub enum CounterMode {
    Up,
    UpAndDown,
}
impl From<CounterMode> for bool {
    fn from(variant: CounterMode) -> Self {
        match variant {
            CounterMode::Up => false,
            CounterMode::UpAndDown => true,
        }
    }
}

#[derive(Eq, PartialEq, Clone, Copy)]
pub enum Loop {
    Disabled,
    Times(u16),
    Inf,
}

#[derive(PartialEq, Clone, Copy)]
pub enum PwmEvent {
    Stopped,
    LoopsDone,
    PwmPeriodEnd,
    SeqStarted(Seq),
    SeqEnd(Seq),
}

#[derive(PartialEq, Clone, Copy)]
pub enum StepMode {
    Auto,
    NextStep,
}
impl From<StepMode> for bool {
    fn from(variant: StepMode) -> Self {
        match variant {
            StepMode::Auto => false,
            StepMode::NextStep => true,
        }
    }
}

pub enum Error {
    DMABufferNotInDataMemory,
    BufferTooLong,
}

pub trait Instance: sealed::Sealed + Deref<Target = RegisterBlock> {
    const INTERRUPT: Interrupt;

    /// Provides access to the associated internal duty buffer for the instance.
    fn buffer() -> &'static Cell<[u16; 4]>;
}

// Internal static duty buffers. One per instance.
static mut BUF0: Cell<[u16; 4]> = Cell::new([0; 4]);
#[cfg(not(any(feature = "52810", feature = "52811")))]
static mut BUF1: Cell<[u16; 4]> = Cell::new([0; 4]);
#[cfg(not(any(feature = "52810", feature = "52811")))]
static mut BUF2: Cell<[u16; 4]> = Cell::new([0; 4]);
#[cfg(not(any(feature = "52810", feature = "52811", feature = "52832")))]
static mut BUF3: Cell<[u16; 4]> = Cell::new([0; 4]);

#[cfg(not(any(feature = "9160", feature = "5340-app")))]
impl Instance for crate::pac::PWM0 {
    const INTERRUPT: Interrupt = Interrupt::PWM0;
    #[inline(always)]
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF0 }
    }
}

#[cfg(not(any(
    feature = "52810",
    feature = "52811",
    feature = "9160",
    feature = "5340-app"
)))]
impl Instance for crate::pac::PWM1 {
    const INTERRUPT: Interrupt = Interrupt::PWM1;
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF1 }
    }
}

#[cfg(not(any(
    feature = "52810",
    feature = "52811",
    feature = "9160",
    feature = "5340-app"
)))]
impl Instance for crate::pac::PWM2 {
    const INTERRUPT: Interrupt = Interrupt::PWM2;
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF2 }
    }
}

#[cfg(not(any(
    feature = "52810",
    feature = "52811",
    feature = "52832",
    feature = "5340-app",
    feature = "9160"
)))]
impl Instance for crate::pac::PWM3 {
    const INTERRUPT: Interrupt = Interrupt::PWM3;
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF3 }
    }
}

#[cfg(any(feature = "9160", feature = "5340-app"))]
impl Instance for crate::pac::PWM0_NS {
    const INTERRUPT: Interrupt = Interrupt::PWM0;
    #[inline(always)]
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF0 }
    }
}

#[cfg(any(feature = "9160", feature = "5340-app"))]
impl Instance for crate::pac::PWM1_NS {
    const INTERRUPT: Interrupt = Interrupt::PWM1;
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF1 }
    }
}

#[cfg(any(feature = "9160", feature = "5340-app"))]
impl Instance for crate::pac::PWM2_NS {
    const INTERRUPT: Interrupt = Interrupt::PWM2;
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF2 }
    }
}

#[cfg(any(feature = "9160", feature = "5340-app"))]
impl Instance for crate::pac::PWM3_NS {
    const INTERRUPT: Interrupt = Interrupt::PWM3;
    fn buffer() -> &'static Cell<[u16; 4]> {
        unsafe { &BUF3 }
    }
}
mod sealed {
    pub trait Sealed {}

    #[cfg(not(any(feature = "5340-app", feature = "9160")))]
    impl Sealed for crate::pac::PWM0 {}

    #[cfg(not(any(
        feature = "52810",
        feature = "52811",
        feature = "5340-app",
        feature = "9160"
    )))]
    impl Sealed for crate::pac::PWM1 {}

    #[cfg(not(any(
        feature = "52810",
        feature = "52811",
        feature = "5340-app",
        feature = "9160"
    )))]
    impl Sealed for crate::pac::PWM2 {}

    #[cfg(not(any(
        feature = "52810",
        feature = "52811",
        feature = "52832",
        feature = "5340-app",
        feature = "9160"
    )))]
    impl Sealed for crate::pac::PWM3 {}

    #[cfg(any(feature = "9160", feature = "5340-app"))]
    impl Sealed for crate::pac::PWM0_NS {}

    #[cfg(any(feature = "9160", feature = "5340-app"))]
    impl Sealed for crate::pac::PWM1_NS {}

    #[cfg(any(feature = "9160", feature = "5340-app"))]
    impl Sealed for crate::pac::PWM2_NS {}

    #[cfg(any(feature = "9160", feature = "5340-app"))]
    impl Sealed for crate::pac::PWM3_NS {}
}

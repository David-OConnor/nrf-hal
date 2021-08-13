//! Taken directly from [nrf-hal](https://github.com/nrf-rs/nrf-hal)
//!
//! HAL interface for the GPIOTE peripheral.
//!
//! The GPIO tasks and events (GPIOTE) module provides functionality for accessing GPIO pins using
//! tasks and events.

#[cfg(feature = "51")]
use crate::pac::GPIO as P0;

#[cfg(not(feature = "51"))]
use crate::pac::P0;

#[cfg(any(feature = "52833", feature = "52840", feature = "53"))]
use crate::pac::P1;

use {
    crate::gpio::{Pin, PinState, Port},
    crate::pac::gpiote::{EVENTS_IN, EVENTS_PORT, TASKS_OUT},
    crate::pac::GPIOTE,
};

#[cfg(not(feature = "51"))]
use crate::pac::gpiote::{TASKS_CLR, TASKS_SET};

#[cfg(not(feature = "51"))]
const NUM_CHANNELS: usize = 8;
#[cfg(feature = "51")]
const NUM_CHANNELS: usize = 4;

/// A safe wrapper around the GPIOTE peripheral.
pub struct Gpiote {
    gpiote: GPIOTE,
}

impl Gpiote {
    /// Takes ownership of the `GPIOTE` peripheral, returning a safe wrapper.
    pub fn new(gpiote: GPIOTE) -> Self {
        Self { gpiote }
    }

    fn channel(&self, channel: usize) -> GpioteChannel {
        GpioteChannel {
            gpiote: &self.gpiote,
            channel,
        }
    }
    pub fn channel0(&self) -> GpioteChannel {
        self.channel(0)
    }
    pub fn channel1(&self) -> GpioteChannel {
        self.channel(1)
    }
    pub fn channel2(&self) -> GpioteChannel {
        self.channel(2)
    }
    pub fn channel3(&self) -> GpioteChannel {
        self.channel(3)
    }
    #[cfg(not(feature = "51"))]
    pub fn channel4(&self) -> GpioteChannel {
        self.channel(4)
    }
    #[cfg(not(feature = "51"))]
    pub fn channel5(&self) -> GpioteChannel {
        self.channel(5)
    }
    #[cfg(not(feature = "51"))]
    pub fn channel6(&self) -> GpioteChannel {
        self.channel(6)
    }
    #[cfg(not(feature = "51"))]
    pub fn channel7(&self) -> GpioteChannel {
        self.channel(7)
    }

    pub fn port(&self) -> GpiotePort {
        GpiotePort {
            gpiote: &self.gpiote,
        }
    }

    /// Marks all GPIOTE events as handled
    pub fn reset_events(&self) {
        (0..NUM_CHANNELS).for_each(|ch| self.gpiote.events_in[ch].write(|w| w));
        self.gpiote.events_port.write(|w| w);
    }

    /// Consumes `self` and return back the raw `GPIOTE` peripheral.
    pub fn free(self) -> GPIOTE {
        self.gpiote
    }
}

pub struct GpioteChannel<'a> {
    gpiote: &'a GPIOTE,
    channel: usize,
}

impl<'a> GpioteChannel<'_> {
    /// Configures the channel as an event input with associated pin.
    pub fn input_pin(&'a self, pin: &'a Pin) -> GpioteChannelEvent<'a> {
        GpioteChannelEvent {
            gpiote: &self.gpiote,
            pin: pin,
            channel: self.channel,
        }
    }
    /// Configures the channel as a task output with associated pin.
    pub fn output_pin(&'a self, pin: Pin) -> GpioteTask<'a> {
        GpioteTask {
            gpiote: &self.gpiote,
            pin: pin,
            channel: self.channel,
            task_out_polarity: TaskOutPolarity::Toggle,
        }
    }

    /// Checks if the channel event has been triggered.
    pub fn is_event_triggered(&self) -> bool {
        self.gpiote.events_in[self.channel].read().bits() != 0
    }
    /// Resets channel events.
    pub fn reset_events(&self) {
        self.gpiote.events_in[self.channel].write(|w| w);
    }

    /// Triggers `task out` (as configured with task_out_polarity, defaults to Toggle).
    pub fn out(&self) {
        self.gpiote.tasks_out[self.channel].write(|w| unsafe { w.bits(1) });
    }
    /// Triggers `task set` (set associated pin high).
    #[cfg(not(feature = "51"))]
    pub fn set(&self) {
        self.gpiote.tasks_set[self.channel].write(|w| unsafe { w.bits(1) });
    }
    /// Triggers `task clear` (set associated pin low).
    #[cfg(not(feature = "51"))]
    pub fn clear(&self) {
        self.gpiote.tasks_clr[self.channel].write(|w| unsafe { w.bits(1) });
    }

    /// Returns reference to channel event endpoint for PPI.
    pub fn event(&self) -> &EVENTS_IN {
        &self.gpiote.events_in[self.channel]
    }

    /// Returns reference to task_out endpoint for PPI.
    pub fn task_out(&self) -> &TASKS_OUT {
        &self.gpiote.tasks_out[self.channel]
    }

    /// Returns reference to task_clr endpoint for PPI.
    #[cfg(not(feature = "51"))]
    pub fn task_clr(&self) -> &TASKS_CLR {
        &self.gpiote.tasks_clr[self.channel]
    }

    /// Returns reference to task_set endpoint for PPI.
    #[cfg(not(feature = "51"))]
    pub fn task_set(&self) -> &TASKS_SET {
        &self.gpiote.tasks_set[self.channel]
    }
}

pub struct GpiotePort<'a> {
    gpiote: &'a GPIOTE,
}

impl<'a> GpiotePort<'_> {
    /// Configures associated pin as port event trigger.
    pub fn input_pin(&'a self, pin: &'a Pin) -> GpiotePortEvent<'a> {
        GpiotePortEvent { pin }
    }
    /// Enables GPIOTE interrupt for port events.
    pub fn enable_interrupt(&self) {
        self.gpiote.intenset.write(|w| w.port().set());
    }
    /// Disables GPIOTE interrupt for port events.
    pub fn disable_interrupt(&self) {
        self.gpiote.intenclr.write(|w| w.port().set_bit());
    }
    /// Checks if port event has been triggered.
    pub fn is_event_triggered(&self) -> bool {
        self.gpiote.events_port.read().bits() != 0
    }
    /// Marks port events as handled.
    pub fn reset_events(&self) {
        self.gpiote.events_port.write(|w| w);
    }
    /// Returns reference to port event endpoint for PPI.
    pub fn event(&self) -> &EVENTS_PORT {
        &self.gpiote.events_port
    }
}

pub struct GpioteChannelEvent<'a> {
    gpiote: &'a GPIOTE,
    pin: &'a Pin,
    channel: usize,
}

impl<'a> GpioteChannelEvent<'_> {
    /// Generates event on falling edge.
    pub fn hi_to_lo(&self) -> &Self {
        config_channel_event_pin(self.gpiote, self.channel, self.pin, EventPolarity::HiToLo);
        self
    }
    /// Generates event on rising edge.
    pub fn lo_to_hi(&self) -> &Self {
        config_channel_event_pin(self.gpiote, self.channel, self.pin, EventPolarity::LoToHi);
        self
    }
    /// Generates event on any pin activity.
    pub fn toggle(&self) -> &Self {
        config_channel_event_pin(self.gpiote, self.channel, self.pin, EventPolarity::Toggle);
        self
    }
    /// No event is generated on pin activity.
    pub fn none(&self) -> &Self {
        config_channel_event_pin(self.gpiote, self.channel, self.pin, EventPolarity::None);
        self
    }
    /// Enables GPIOTE interrupt for channel.
    pub fn enable_interrupt(&self) -> &Self {
        unsafe { self.gpiote.intenset.write(|w| w.bits(1 << self.channel)) }
        self
    }
    /// Disables GPIOTE interrupt for channel.
    pub fn disable_interrupt(&self) -> &Self {
        unsafe { self.gpiote.intenclr.write(|w| w.bits(1 << self.channel)) }
        self
    }
}

fn config_channel_event_pin(
    gpiote: &GPIOTE,
    channel: usize,
    pin: &Pin,
    trigger_mode: EventPolarity,
) {
    // Config pin as event-triggering input for specified edge transition trigger mode.
    gpiote.config[channel].write(|w| {
        match trigger_mode {
            EventPolarity::HiToLo => w.mode().event().polarity().hi_to_lo(),
            EventPolarity::LoToHi => w.mode().event().polarity().lo_to_hi(),
            EventPolarity::None => w.mode().event().polarity().none(),
            EventPolarity::Toggle => w.mode().event().polarity().toggle(),
        };

        #[cfg(any(feature = "52833", feature = "52840"))]
        {
            match pin.port {
                Port::P0 => w.port().clear_bit(),
                Port::P1 => w.port().set_bit(),
                #[cfg(feature = "53")]
                Port::P2 => unimplemented!(), // todo
                #[cfg(feature = "53")]
                Port::P2 => unimplemented!(), // todo
            };
        }

        unsafe { w.psel().bits(pin.pin) }
    });
}

pub struct GpiotePortEvent<'a> {
    pin: &'a Pin,
}

impl<'a> GpiotePortEvent<'_> {
    /// Generates event on pin low.
    pub fn low(&self) {
        config_port_event_pin(self.pin, PortEventSense::Low);
    }
    /// Generates event on pin high.
    pub fn high(&self) {
        config_port_event_pin(self.pin, PortEventSense::High);
    }
    /// No event is generated on pin activity.
    pub fn disabled(&self) {
        config_port_event_pin(self.pin, PortEventSense::Disabled);
    }
}

fn config_port_event_pin(pin: &Pin, sense: PortEventSense) {
    // Set pin sense to specified mode to trigger port events.
    unsafe {
        &(*{
            match pin.port {
                Port::P0 => P0::ptr(),
                #[cfg(any(feature = "52833", feature = "52840"))]
                Port::P1 => P1::ptr(),
            }
        })
        .pin_cnf[pin.pin as usize]
    }
    .modify(|_r, w| match sense {
        PortEventSense::Disabled => w.sense().disabled(),
        PortEventSense::High => w.sense().high(),
        PortEventSense::Low => w.sense().low(),
    });
}

pub struct GpioteTask<'a> {
    gpiote: &'a GPIOTE,
    pin: Pin,
    channel: usize,
    task_out_polarity: TaskOutPolarity,
}

impl<'a> GpioteTask<'_> {
    /// Sets initial task output pin state to high.
    pub fn init_high(&self) {
        config_channel_task_pin(
            self.gpiote,
            self.channel,
            &self.pin,
            &self.task_out_polarity,
            PinState::High,
        );
    }
    /// Sets initial task output pin state to low.
    pub fn init_low(&self) {
        config_channel_task_pin(
            self.gpiote,
            self.channel,
            &self.pin,
            &self.task_out_polarity,
            PinState::Low,
        );
    }
    /// Configures polarity of the `task out` operation.
    pub fn task_out_polarity(&mut self, polarity: TaskOutPolarity) -> &mut Self {
        self.task_out_polarity = polarity;
        self
    }
}

fn config_channel_task_pin(
    gpiote: &GPIOTE,
    channel: usize,
    pin: &Pin,
    task_out_polarity: &TaskOutPolarity,
    init_out: PinState,
) {
    // Config pin as task output with specified initial state and task out polarity.
    gpiote.config[channel].write(|w| {
        match init_out {
            PinState::High => w.mode().task().outinit().high(),
            PinState::Low => w.mode().task().outinit().low(),
        };
        match task_out_polarity {
            TaskOutPolarity::Set => w.polarity().lo_to_hi(),
            TaskOutPolarity::Clear => w.polarity().hi_to_lo(),
            TaskOutPolarity::Toggle => w.polarity().toggle(),
        };

        #[cfg(any(feature = "52833", feature = "52840"))]
        {
            match pin.port {
                Port::P0 => w.port().clear_bit(),
                Port::P1 => w.port().set_bit(),
                #[cfg(feature = "53")]
                Port::P2 => unimplemented!(), // todo
                #[cfg(feature = "53")]
                Port::P3 => unimplemented!(), // todo
            };
        }

        unsafe { w.psel().bits(pin.pin) }
    });
}

/// Polarity of the `task out` operation.
pub enum TaskOutPolarity {
    Set,
    Clear,
    Toggle,
}

pub enum EventPolarity {
    None,
    HiToLo,
    LoToHi,
    Toggle,
}

pub enum PortEventSense {
    Disabled,
    High,
    Low,
}

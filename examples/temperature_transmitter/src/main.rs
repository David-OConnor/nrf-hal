//! This module listens for requests over Bluetooth, and upon reception, wakes the device from a lower power
//! mode, takes a reading, and transmits it over Bluetooth.

// todo: Once working, make this an stm32-hal example.

#![no_main]
#![no_std]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use cortex_m::{
    asm,
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use num_traits::float::Float; // float absolute value

use defmt_rtt as _;
use panic_probe as _;

use nrf_hal::{
    // bluetooth::Bluetooth,
    clocks::{Clocks, LFCLK_FREQ},
    gpio::{Dir, Drive, Pin, Port, Pull},
    pac::{self, interrupt, RTC0, TIMER0, TIMER1, TIMER2, TWIM0},
    prelude::*,
    rtc::{Rtc, RtcCompareReg, RtcInterrupt},
    timer::{Timer, TimerMode, TimerShortcut},
    twim::{Twim, TwimFreq},
};

use esb::{
    consts::*, irq::StatePTX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
    EsbBuffer, EsbHeader, EsbIrq, IrqTimer,
};

mod sensor;

// Global state variables, accessible in interrupts.
make_globals!(
    (SDA, Pin),
    (SCL, Pin),
    (RTC, Rtc<RTC0>),
    (TWIM, Twim<TWIM0>),
    (ESB_APP, EsbApp<U1024, U1024>),
    (ESB_IRQ, EsbIrq<U1024, U1024, TIMER0, StatePTX>),
    (ESB_TIMER, IrqTimer<TIMER0>),
    (SENSOR_WAKE_TIMER, Timer<TIMER1>),
    (COOL_THRESH_TIMER, Timer<TIMER2>)
);

make_simple_globals!((OP_MODE, OpMode, OpMode::Active));

static SENSOR_SLEEPING: AtomicBool = AtomicBool::new(false);
// Note: Nordic provides no way to check if the timer's running, so we
// use this variable to keep track.
static COOL_THRESH_TIMER_RUNNING: AtomicBool = AtomicBool::new(false);

// todo: Consider changing SRT to sleep_thresh or something. And change comments
// todo and how it's used etc.

// In °C. If temp is above this, enable the display and show it. Make sure it's low enough
// to let users know they won't get burned if the display's off.
const SHOW_READING_THRESH: f32 = 26.; // todo: 50.

// If temp is below `SHOW_READING_THRESH` for this long in seconds,  hide readings,
// and go to idle mode.
// Make sure BELOW_SRT_LIMIT is enough so that a minor adjustment won't turn the device off.
// 20s or so should be good.
const COOL_THRESH_PERIOD: f32 = 10.;

const MAX_PAYLOAD_SIZE: u8 = 64;
// The first byte we send (status byte) can either be this success byte, or this fault byte. These
// must match those used by the base firmware.
const SUCCESS_BYTE: u8 = 20;

// SLEEP_TIME_IDLE can be 60s for battery operations
const SLEEP_TIME_IDLE: f32 = 45.; // seconds
                                  // Perhaps the active sleep time can be short, since most of the power is
                                  // used by the display? Or not. Note that the sensor will not produce
                                  // reliable readings without a pause between.

// The time to sleep between readings while in active mode.
const SLEEP_TIME_ACTIVE: f32 = 1.; // seconds

// todo: Disable TWIM while sleeping!
// todo: Disable Radio (Take out of Tx and/or Rx mode?) when sleeping!

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum OpMode {
    Active,
    Idle,
    SurfaceMenu,
}

/// Transmit a sensor reading over ESB, with our 5-byte message format.
fn transmit(reading: f32) {
    let reading_bytes = reading.to_bits().to_be_bytes();
    let msg = [
        SUCCESS_BYTE,
        reading_bytes[0],
        reading_bytes[1],
        reading_bytes[2],
        reading_bytes[3],
    ];

    // todo: Store this header globally; don't recreate it each time.
    // todo: We can't use a static var due to the function call.
    let esb_header = EsbHeader::build()
        .max_payload(MAX_PAYLOAD_SIZE)
        .pid(0)
        .pipe(0)
        .no_ack(false)
        .check()
        .unwrap();

    // todo: The ability to send fault codes etc.

    free(|cs| {
        access_global!(ESB_APP, esb_app, cs);
        if let Some(response) = esb_app.read_packet() {
            // let rssi = response.get_header().rssi();
            response.release();
        }

        let mut packet = esb_app.grant_packet(esb_header).unwrap();
        let length = msg.len();
        &packet[..length].copy_from_slice(&msg);
        packet.commit(length);
        esb_app.start_tx();
    });
}

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let dp = pac::Peripherals::take().unwrap();

    let clocks = Clocks::new(dp.CLOCK).enable_ext_hfosc();

    // To prevent extra power use from the temp sensor, we don't use a hardware PU on
    // SCL, and drive it low when the sensor sleeps. In fact, we use software pull
    // ups on both SCL and SDA. See `sensor::sleep` and `sensor::wake` for more details.
    let mut scl = Pin::new(Port::P0, 26, Dir::Input);
    scl.pull(Pull::Up);
    scl.drive(Drive::S0D1);

    let mut sda = Pin::new(Port::P0, 27, Dir::Input);
    sda.pull(Pull::Up);
    sda.drive(Drive::S0D1);

    let twim = Twim::new(dp.TWIM0, &scl, &sda, TwimFreq::K100);

    // Start the low frequency clock, for use with the RTC.
    clocks.start_lfclk();

    // Run RTC for 1 second (1hz == LFCLK_FREQ)
    // todo: High-level API in `nrf-hal` to make setting this more intuitive, and explicit re
    // todo timeout period.
    let mut rtc = Rtc::new(dp.RTC0, 0).unwrap();
    rtc.set_timeout(RtcCompareReg::Compare0, SLEEP_TIME_ACTIVE).unwrap();
    rtc.enable_event(RtcInterrupt::Compare0);
    rtc.enable_interrupt(RtcInterrupt::Compare0);
    rtc.enable_counter();

    // ESB RF setup
    static ESB_BUFER: EsbBuffer<U1024, U1024> = EsbBuffer {
        app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
        radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
        timer_flag: AtomicBool::new(false),
    };

    let addresses = Addresses::default();

    let esb_cfg = ConfigBuilder::default()
        .maximum_transmit_attempts(0)
        .max_payload_size(MAX_PAYLOAD_SIZE)
        .check()
        .unwrap();

    let (esb_app, esb_irq, esb_timer) = ESB_BUFER
        .try_split(dp.TIMER0, dp.RADIO, addresses, esb_cfg)
        .unwrap();

    // The Cool Thresh timer is used to check if we've been below the active threshhold
    // long enough to put the device into idle mode.
    // We use 1.01 / BELOW_SRT_LIMIT instead of 1 / to prevent this timer from syncing with the
    // RTC wakeup, and the RTC wakeup triggering before the main loop can turn off the display. (?)
    let mut cool_thresh_timer =
        Timer::new(dp.TIMER2, TimerMode::Timer, 1.01 / COOL_THRESH_PERIOD, 0);

    cool_thresh_timer.shortcut(TimerShortcut::StopClear, 0);
    cool_thresh_timer.enable_interrupt(0);

    // We use the sensor_wake timer to sleep the core while the sensor wakes, instead
    // of a blocking delay. Datasheet says 250ms, which we pad.
    let mut sensor_wake_timer = Timer::new(dp.TIMER1, TimerMode::Timer, 1. / 0.27, 0);

    sensor_wake_timer.shortcut(TimerShortcut::StopClear, 0);
    sensor_wake_timer.enable_interrupt(0);

    free(|cs| {
        TWIM.borrow(cs).replace(Some(twim));
        // SDA and SCL are global for use in waking the sensor from its sleep mode.
        SDA.borrow(cs).replace(Some(sda));
        SCL.borrow(cs).replace(Some(scl));
        RTC.borrow(cs).replace(Some(rtc));

        ESB_APP.borrow(cs).replace(Some(esb_app));
        ESB_IRQ.borrow(cs).replace(Some(esb_irq.into_ptx()));
        ESB_TIMER.borrow(cs).replace(Some(esb_timer));
        SENSOR_WAKE_TIMER
            .borrow(cs)
            .replace(Some(sensor_wake_timer));
        COOL_THRESH_TIMER
            .borrow(cs)
            .replace(Some(cool_thresh_timer));
    });

    unsafe {
        NVIC::unmask(pac::Interrupt::RTC0);
        NVIC::unmask(pac::Interrupt::RADIO);
        NVIC::unmask(pac::Interrupt::TIMER0);
        NVIC::unmask(pac::Interrupt::TIMER1);
        NVIC::unmask(pac::Interrupt::TIMER2);

        // "If you look at the cortex m4 manual, each interrupt has a byte in a 32 bit register5"
        cp.NVIC.set_priority(pac::Interrupt::RTC0, 32);
        cp.NVIC.set_priority(pac::Interrupt::RADIO, 3);
        cp.NVIC.set_priority(pac::Interrupt::TIMER0, 3);
        cp.NVIC.set_priority(pac::Interrupt::TIMER1, 3);
        cp.NVIC.set_priority(pac::Interrupt::TIMER2, 3);
    }

    loop {
        // todo: Is it wfe we want? Also, NRF_POWER-> SYSTEMOFF = 1?
        asm::wfi();
    }
}

#[interrupt]
fn RADIO() {
    free(|cs| {
        access_global!(ESB_IRQ, esb_irq, cs);
        match esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {
                // ATTEMPTS_FLAG.store(true, Ordering::Release);
            }
            Err(e) => panic!("Found radio error {:?}", e),
            Ok(_) => {}
        }
    });
}

#[interrupt]
/// ISR for the radio IRQ timer, for use with the ESB protocol and library.
fn TIMER0() {
    free(|cs| {
        access_global!(ESB_TIMER, esb_timer, cs);
        esb_timer.timer_interrupt();
    });
}

#[interrupt]
/// ISR for the sensor wake timer, used to sleep the CPU while the sensor is waking up,
/// in a non-blocking way.
fn TIMER1() {
    free(|cs| {
        access_global!(SENSOR_WAKE_TIMER, sensor_wake_timer, cs);
        sensor_wake_timer.clear_interrupt(0);
    });
}

#[interrupt]
/// ISR for the cool thresh timer. When a temperature reading is below the threshhold, this timer
/// starts. When it times out, this ISR runs, putting the sensor and MCU in low-power modes.
fn TIMER2() {
    free(|cs| {
        access_global!(COOL_THRESH_TIMER, cool_thresh_timer, cs);
        cool_thresh_timer.clear_interrupt(0);
        COOL_THRESH_TIMER_RUNNING.store(false, Ordering::Release);

        OP_MODE.borrow(cs).set(OpMode::Idle);
        access_global!(RTC, rtc, cs);
        rtc.set_compare(
            RtcCompareReg::Compare0,
            (LFCLK_FREQ as f32 * SLEEP_TIME_IDLE) as u32,
        )
        .unwrap();

        if SENSOR_SLEEPING.swap(true, Ordering::Relaxed) {
            access_global!(TWIM, twim, cs);
            access_global!(SCL, scl, cs);
            sensor::sleep(twim, scl);
        } else {
            SENSOR_SLEEPING.store(false, Ordering::Relaxed);
        }
    });
}

#[interrupt]
/// Wake the device up, take a reading, and transmit it. Change between active and
/// idle modes based on if the reading indicates the stove it on and/or hot.
fn RTC0() {
    let mut reading = 0.;

    free(|cs| {
        access_global!(RTC, rtc, cs);
        rtc.reset_event(RtcInterrupt::Compare0);
        rtc.clear_counter();

        access_global!(TWIM, twim, cs);

        if SENSOR_SLEEPING.load(Ordering::Acquire) {
            SENSOR_SLEEPING.store(false, Ordering::Release);
            // todo: Swap etc
            access_global!(SDA, sda, cs);
            access_global!(SCL, scl, cs);
            access_global!(SENSOR_WAKE_TIMER, sensor_wake_timer, cs);
            sensor::wake(twim, scl, sda, sensor_wake_timer);
        }

        reading = sensor::read_temp(twim).unwrap_or(3.);
    });

    // Sometimes after sensor wake, our readings are -273.15. Unknown cause. Note that this is the same
    // reading we get if attempting to take a reading while the sensor is asleep.
    let temp_valid = (reading - 3.).abs() > 0.01 && (reading - -273.15).abs() > 0.01;

    if !temp_valid {
        return;
    }

    // If temperature is above a thresh, set the active sleep time. Otherwise,
    // command idle, which will execute power-saving steps, and us a longer sleep time,
    // after a timer expires.
    free(|cs| {
        let mode = OP_MODE.borrow(cs).get();

        access_global!(COOL_THRESH_TIMER, cool_thresh_timer, cs);

        if reading >= SHOW_READING_THRESH {
            cool_thresh_timer.stop();

            if let OpMode::Idle = mode {
                OP_MODE.borrow(cs).set(OpMode::Active);
                access_global!(RTC, rtc, cs);
                rtc.set_compare(
                    RtcCompareReg::Compare0,
                    (LFCLK_FREQ as f32 * SLEEP_TIME_ACTIVE) as u32,
                )
                .unwrap();
            }
        } else {
            match mode {
                OpMode::Active => {
                    // If we were active but are now below the thresh, and haven't already,
                    // start the cool thresh timer.
                    if !COOL_THRESH_TIMER_RUNNING.load(Ordering::Acquire) {
                        // todo: Swap etc
                        COOL_THRESH_TIMER_RUNNING.store(true, Ordering::Release);
                        cool_thresh_timer.start();
                    }
                }
                OpMode::Idle => {
                    // We just woke up the sensor, but don't need to go active; go back to sleep,
                    // and don't show anything on the display.
                    access_global!(TWIM, twim, cs);
                    access_global!(SCL, scl, cs);
                    sensor::sleep(twim, scl);
                    SENSOR_SLEEPING.store(true, Ordering::Release);
                }
                // We should never enter the RTC from the menu; only Active and Idle should show up.
                OpMode::SurfaceMenu => {} // todo: Role of SurfaceMenu on this sensor firmware?
            }
        }
    });

    transmit(reading); // `transmit` has it's own CS; don't nest.
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

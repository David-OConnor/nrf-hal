//! This module listens for requests over Bluetooth, and upon reception, wakes the device from a lower power
//! mode, takes a reading, and transmits it over Bluetooth.

#![no_main]
#![no_std]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::{
    asm,
    delay::Delay,
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use cortex_m_rt::entry;

use num_traits::float::Float; // float absolute value

use defmt_rtt as _;
use panic_probe as _;

use nrf_hal::{
    clocks::Clocks,
    gpio::{Dir, Drive, Pin, Port, Pull},
    pac::{self, interrupt, RTC0, TIMER0, TIMER1, TIMER2, TWIM0},
    prelude::*,
    rtc::{Rtc, RtcCompare, RtcInterrupt},
    timer::{Timer, TimerMode, TimerShortcut},
    twim::{Twim, TwimFreq},
};

use esb::{
    consts::*, irq::StatePTX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
    EsbBuffer, EsbHeader, EsbIrq, IrqTimer, TxPower,
};

mod sensor;

// Global state variables, accessible in interrupts.
make_globals!(
    (DELAY, Delay),
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

static RF_PID: AtomicU8 = AtomicU8::new(0);

// todo from thalesfrago: "keep the esb_irq on the radio interrupt and the esb_app somewhere else"

// todo: 2021-10-18: First time it tries to wake up, it reads -273. Next time it works.

// In °C. If temp is above this, enable the display and show it. Make sure it's low enough
// to let users know they won't get burned if the display's off.
const SHOW_READING_THRESH: f32 = 26.; // todo: 50.

// If temp is below `SHOW_READING_THRESH` for this long in seconds,  hide readings,
// and go to idle mode.
// Make sure BELOW_SRT_LIMIT is enough so that a minor adjustment won't turn the device off.
// 20s or so should be good.
const COOL_THRESH_PERIOD: f32 = 10.;

const MAX_PAYLOAD_SIZE: u8 = 16;
// The first byte we send (status byte) can either be this success byte, or this fault byte. These
// must match those used by the base firmware.
const SUCCESS_BYTE: u8 = 20;

// SLEEP_TIME_IDLE can be 60s for battery operations
const SLEEP_TIME_IDLE: f32 = 6.; // seconds // todo ~45
// Perhaps the active sleep time can be short, since most of the power is
// used by the display? Or not. Note that the sensor will not produce
// reliable readings without a pause between.

const TEMP_ERROR_FLAG: f32 = 3.; // Value we unwrap an i2c error on reading to.

// The time to sleep between readings while in active mode.
const SLEEP_TIME_ACTIVE: f32 = 2.; // seconds

// These flags are used in the radio packet to tell the base unit what type of message is being sent.
// They must match their equiv in the base unit firmware.
const TEMP_XMIT_FLAG: u8 = 0;
// const IDLE_XMIT_FLAG: u8 = 1;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum OpMode {
    Active,
    Idle,
    // SurfaceMenu,
}

/// Transmit a sensor reading over ESB, with our 5-byte message format.
fn transmit(msg: &[u8]) {
    // todo: Way to do this truly atomically (in one go?)
    if RF_PID.compare_exchange(3, 0, Ordering::Release, Ordering::Relaxed).is_err() {
        RF_PID.fetch_add(1, Ordering::Release);
    }

    // todo: Combine this load with the above atomic logic.
    let pid = RF_PID.load(Ordering::Acquire);

    let esb_header = EsbHeader::build()
        .max_payload(MAX_PAYLOAD_SIZE)
        .pid(pid)
        .pipe(0)
        .no_ack(false)
        .check()
        .unwrap();

    // todo: The ability to send fault codes etc.

    free(|cs| {
        access_global!(ESB_APP, esb_app, cs);
        if let Some(response) = esb_app.read_packet() {

            if response.len() < 3 {
                defmt::error!("ERROR RESPONSE LEN");
                // todo: Handle this, but not with a return!
                // return; // todo: What causes this?
            } else {
                defmt::info!("RESPONSE: {} {} {}", response[0], response[1], response[2]);

                // Change emissivity if requested by the base unit.
                if response[0] == SUCCESS_BYTE && response[1] == 1 {
                    let emis = match response[2] {
                        0 => 0.90,
                        1 => 0.85,
                        2 => 0.6,
                        _ => panic!()
                    };

                    defmt::info!("SETTING EMIS");
                    access_global!(TWIM, twim, cs);
                    access_global!(DELAY, delay, cs);
                    sensor::set_emissivity(emis, twim, delay).ok();
                }
            }
            // defmt::info!("RESPONSE: {}", response[0]);

            response.release();
        }

        let mut packet = esb_app.grant_packet(esb_header).unwrap();
        let length = msg.len();
        packet[..length].copy_from_slice(&msg);
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

    // let mut scl = Pin::new(Port::P0, 20, Dir::Output);
    // let mut sda = Pin::new(Port::P0, 24, Dir::Output);
    // scl.set_low();
    // sda.set_low();
    // defmt::info!("LOOPING");
    //
    // loop {}

    // To prevent extra power use from the temp sensor, we don't use a hardware PU on
    // SCL, and drive it low when the sensor sleeps. In fact, we use software pull
    // ups on both SCL and SDA. See `sensor::sleep` and `sensor::wake` for more details.
    // This is fine, since we don't need clock stretching.
    let mut scl = Pin::new(Port::P0, 20, Dir::Input);

    let mut sda = Pin::new(Port::P0, 24, Dir::Input);
    sda.pull(Pull::Up);
    sda.drive(Drive::S0D1);

    let twim = Twim::new(dp.TWIM0, &scl, &sda, TwimFreq::K100);

    // Start the low frequency clock, for use with the RTC.
    clocks.start_lfclk();

    // We use this blocking delay for setting emissivity.
    let delay = Delay::new(cp.SYST, 64_000_000);

    let mut rtc = Rtc::new(dp.RTC0, 0).unwrap();
    rtc.set_timeout(RtcCompare::Compare0, SLEEP_TIME_ACTIVE)
        .unwrap();
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
        // .tx_power(TxPower::POS4DBM) // ts. defaults to 0.
        .maximum_transmit_attempts(3)
        .max_payload_size(MAX_PAYLOAD_SIZE)
        .wait_for_ack_timeout(300) // TS. defaults to 120us.
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
        Timer::new(dp.TIMER2, TimerMode::Timer, 1.01 / COOL_THRESH_PERIOD, 2);

    cool_thresh_timer.shortcut(TimerShortcut::StopClear, 2);
    cool_thresh_timer.enable_interrupt(2);

    // We use the sensor_wake timer to sleep the core while the sensor wakes, instead
    // of a blocking delay. Datasheet says 250ms, which we pad. Note that we change the period depending on
    // how we use this timer in the sensor wake and set_emissivitiy functions.
    let mut sensor_wake_timer = Timer::new(dp.TIMER1, TimerMode::Timer, 1. / 0.3, 1);
    sensor_wake_timer.shortcut(TimerShortcut::StopClear, 1);
    sensor_wake_timer.enable_interrupt(1);

    free(|cs| {
        DELAY.borrow(cs).replace(Some(delay));
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

        // It appears interrupt priority is chunked in sets of 32, with low numbers being higher
        // priority.
        // Set. High priority for the RF interrupts.
        cp.NVIC.set_priority(pac::Interrupt::RADIO, 1);
        cp.NVIC.set_priority(pac::Interrupt::TIMER0, 1);
        // We have to allow Timer 1 to interrupt the RTC, since we use it for nonblocking
        // waits from a fn called from the RTC ISR.
        cp.NVIC.set_priority(pac::Interrupt::TIMER1, 32);
        cp.NVIC.set_priority(pac::Interrupt::RTC0, 64);
        cp.NVIC.set_priority(pac::Interrupt::TIMER2, 64);
    }

    loop {
        // Note that it's not possible to use the RTC with SYSTEMOFF enabled.
        asm::wfi();
    }
}

#[interrupt]
fn RADIO() {
    // defmt::info!("RADIO ISR");
    free(|cs| {
        access_global!(ESB_IRQ, esb_irq, cs);
        match esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {}
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
/// in a non-blocking way. Also used when setting emissivity.
fn TIMER1() {
    free(|cs| {
        access_global!(SENSOR_WAKE_TIMER, sensor_wake_timer, cs);
        sensor_wake_timer.clear_interrupt(1);
    });
}

#[interrupt]
/// ISR for the cool thresh timer. When a temperature reading is below the threshhold, this timer
/// starts. When it times out, this ISR runs, putting the sensor in a low-power mode, and disables
/// TWIM and the radio. Code in this ISR enables power-saving functionality for several peripherals.
fn TIMER2() {
    // defmt::warn!("Going to sleep.");
    free(|cs| {
        access_global!(COOL_THRESH_TIMER, cool_thresh_timer, cs);
        cool_thresh_timer.clear_interrupt(2);

        COOL_THRESH_TIMER_RUNNING.store(false, Ordering::Release);

        OP_MODE.borrow(cs).set(OpMode::Idle);
        access_global!(RTC, rtc, cs);
        rtc.set_timeout(RtcCompare::Compare0, SLEEP_TIME_IDLE)
            .unwrap();

        access_global!(TWIM, twim, cs);

        if SENSOR_SLEEPING
            .compare_exchange(false, true, Ordering::Release, Ordering::Relaxed)
            .is_ok()
        {
            access_global!(SCL, scl, cs);
            sensor::sleep(twim, scl);
            // defmt::warn!("setting sensor to sleep mode");
        }

        twim.disable();

        // Transmit a message to the base unit, so it knows to turn off the display etc.
        // transmit(&[SUCCESS_BYTE, IDLE_XMIT_FLAG, 0, 0, 0, 0]); // `transmit` has it's own CS; don't nest.

        // todo: Not sure we need to mask, but troubleshooting.
        // todo: Mask after a short timer, eg masking immediately appears to block the xmission.
        // NVIC::mask(pac::Interrupt::RADIO);
        // NVIC::mask(pac::Interrupt::TIMER0);


        // todo: Figure out how to re-enable, then put this back. Or does the ESB
        // todo lib handle this for you?
        // unsafe { (*pac::RADIO::ptr()).tasks_disable.write(|w| w.bits(1))}
        // todo: Set POWER register to 0??
    });
}

#[interrupt]
/// Wake the device up, take a reading, and transmit it. Change between active and
/// idle modes based on if the reading indicates the stove is on and/or hot. This ISR
/// contains the bulk of our execution logic.
fn RTC0() {
    free(|cs| {
        access_global!(RTC, rtc, cs);
        rtc.reset_event(RtcInterrupt::Compare0);
        rtc.clear_counter();

        access_global!(TWIM, twim, cs);
        if !twim.is_enabled() {
            twim.enable();
        }
    });

    if SENSOR_SLEEPING
        .compare_exchange(true, false, Ordering::Release, Ordering::Relaxed)
        .is_ok()
    {
        sensor::wake();
    }

    let mut reading = 0.;
    free(|cs| {
        access_global!(TWIM, twim, cs);
        reading = sensor::read_temp(twim).unwrap_or(TEMP_ERROR_FLAG);
    });

    // Sometimes after sensor wake, our readings are -273.15. Unknown cause. Note that this is the same
    // reading we get if attempting to take a reading while the sensor is asleep.
    let temp_valid = (reading - TEMP_ERROR_FLAG).abs() > 0.01 && (reading - -273.15).abs() > 0.01;

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
            cool_thresh_timer.clear();
            COOL_THRESH_TIMER_RUNNING.store(false, Ordering::Release);

            if let OpMode::Idle = mode {

                // todo: Not sure we need to mask, but troubleshooting.
                unsafe {
                    NVIC::unmask(pac::Interrupt::RADIO);
                    // NVIC::unmask(pac::Interrupt::TIMER0);
                }

                OP_MODE.borrow(cs).set(OpMode::Active);
                access_global!(RTC, rtc, cs);
                rtc.set_timeout(RtcCompare::Compare0, SLEEP_TIME_ACTIVE)
                    .unwrap();
            } else {

            }
        } else {
            match mode {
                OpMode::Active => {
                    // defmt::warn!("Temp below thresh.");
                    // If we were active but are now below the thresh, and haven't already,
                    // start the cool thresh timer.

                    if COOL_THRESH_TIMER_RUNNING
                        .compare_exchange(false, true, Ordering::Release, Ordering::Relaxed)
                        .is_ok()
                    {
                        // defmt::warn!("STARTING THE SLEEP TIMER");
                        cool_thresh_timer.start();
                    }
                }
                OpMode::Idle => {
                    // defmt::warn!("Stay asleep");
                    // We just woke up the sensor, but don't need to go active; go back to sleep.
                    access_global!(TWIM, twim, cs);
                    access_global!(SCL, scl, cs);
                    sensor::sleep(twim, scl);
                    SENSOR_SLEEPING.store(true, Ordering::Release);
                } // We should never enter the RTC from the menu; only Active and Idle should show up.
                // OpMode::SurfaceMenu => {} // todo: Role of SurfaceMenu on this sensor firmware?
            }
        }
    });

    let mut mode = OpMode::Active;
    free(|cs| {
       mode = OP_MODE.borrow(cs).get()
    });

    // Transmit the reading if we're in active mode: Either from already being active, or
    // just entering active mode.
    if mode == OpMode::Active {
        let reading_bytes = reading.to_bits().to_be_bytes();
        let msg = [
            SUCCESS_BYTE,
            TEMP_XMIT_FLAG,
            reading_bytes[0],
            reading_bytes[1],
            reading_bytes[2],
            reading_bytes[3],
        ];

        // transmit(msg, *PID); // `transmit` has it's own CS; don't nest.
        transmit(&msg); // `transmit` has it's own CS; don't nest.
    }

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

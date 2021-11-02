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

static SENSOR_SLEEPING: AtomicBool = AtomicBool::new(false);
// Note: Nordic provides no way to check if the timer's running, so we
// use this variable to keep track.
static COOL_THRESH_TIMER_RUNNING: AtomicBool = AtomicBool::new(false);

// From thalesfrago (ESB lib author): "keep the esb_irq on the radio interrupt and the esb_app somewhere else"

// In °C. If temp is above this, enable the display and show it. Make sure it's low enough
// to let users know they won't get burned if the display's off.
const SHOW_READING_THRESH: f32 = 26.; // todo: 50.

// If temp is below `SHOW_READING_THRESH` for this long in seconds, go to idle mode, where we increase
// the wakeup interval to save power.

// Make sure COOL_THRESH_PERIOD is high enough so that a minor adjustment won't turn the device off.
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

static OP_MODE: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// This enum tracks system state, and is stored as an atomic variable: `OP_MODE`.
pub enum OpMode {
    Active = 0,
    Idle = 1,
}

impl From<u8> for OpMode {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Active,
            1 => Self::Idle,
            _ => panic!(),
        }
    }
}

impl OpMode {
    pub fn load(ordering: Ordering) -> Self {
        OP_MODE.load(ordering).into()
    }

    pub fn store(&self, ordering: Ordering) {
        OP_MODE.store(*self as u8, ordering);
    }
}

/// Transmit a sensor reading over ESB, with our 5-byte message format.
fn transmit(
    msg: &[u8],
    twim: &mut Twim<TWIM0>,
    esb_app: &mut EsbApp<U1024, U1024>,
    delay: &mut Delay,
    pid: &mut u8,
) {
    if *pid == 3 {
        *pid = 0;
    } else {
        *pid += 1;
    }

    let esb_header = EsbHeader::build()
        .max_payload(MAX_PAYLOAD_SIZE)
        .pid(*pid)
        .pipe(0)
        .no_ack(false)
        .check()
        .unwrap();

    // todo: The ability to send fault codes etc.

    if let Some(response) = esb_app.read_packet() {
        if response.len() < 3 {
            defmt::error!("ERROR RESPONSE LEN");
        } else {
            defmt::info!("RESPONSE: {} {} {}", response[0], response[1], response[2]);

            // Change emissivity if requested by the base unit.
            if response[0] == SUCCESS_BYTE && response[1] == 1 {
                let emis = match response[2] {
                    0 => 0.90,
                    1 => 0.85,
                    2 => 0.6,
                    _ => panic!(),
                };

                defmt::info!("SETTING EMIS");
                sensor::set_emissivity(emis, twim, delay).ok();
            }
        }

        response.release();
    }

    let mut packet = esb_app.grant_packet(esb_header).unwrap();
    let length = msg.len();
    packet[..length].copy_from_slice(&msg);
    packet.commit(length);
    esb_app.start_tx();
}

#[rtic::app(device = pac)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        sda: Pin,
        scl: Pin,
        rtc: Rtc<RTC0>,
        twim: Twim<TWIM0>,
        cool_thresh_timer: Timer<TIMER2>,
        sensor_wake_timer: Timer<TIMER1>,
    }

    #[local]
    struct Local {
        delay: Delay,
        esb_app: EsbApp<U1024, U1024>,
        esb_irq: EsbIrq<U1024, U1024, TIMER0, StatePTX>,
        esb_timer: IrqTimer<TIMER0>,
        pid: u8,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let cp = cx.core;
        // Set up microcontroller peripherals
        let dp = cx.device;

        let clocks = Clocks::new(dp.CLOCK).enable_ext_hfosc();

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

        (
            Shared {
                sda,
                scl,
                rtc,
                twim,
                cool_thresh_timer,
                sensor_wake_timer,
            },
            Local {
                delay,
                esb_app,
                esb_irq: esb_irq.into_ptx(),
                esb_timer,
                pid: 0,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        // Note that it's not possible to use the RTC with SYSTEMOFF enabled.
        loop {
            asm::wfi();
        }
    }

    #[task(binds = RADIO, local = [esb_irq], priority = 3)]
    fn radio_isr(cx: radio_isr::Context) {
        match cx.local.esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {}
            Err(e) => panic!("Found radio error {:?}", e),
            Ok(_) => {}
        }
    }

    #[task(binds = TIMER0, local = [esb_timer], priority = 3)]
    /// ISR for the radio IRQ timer, for use with the ESB protocol and library.
    fn irq_timer_isr(cx: irq_timer_isr::Context) {
        cx.local.esb_timer.timer_interrupt();
    }

    #[task(binds = TIMER1, priority = 2)]
    /// ISR for the sensor wake timer, used to sleep the CPU while the sensor is waking up,
    /// in a non-blocking way. Also used when setting emissivity.
    /// We have to allow Timer 1 to interrupt the RTC, since we use it for nonblocking, hence
    /// its elevanted priority.
    fn wake_timer_isr(_cx: wake_timer_isr::Context) {
        // Don't use the timer as a resource here, or it won't be able to fire through the lock in
        // the RTC ISR, which uses this timer.
        unsafe {
            (*TIMER1::ptr()).events_compare[1].reset(); // clear the interrupt flag
        }
    }

    #[task(binds = TIMER2, shared = [cool_thresh_timer, rtc, twim, scl], priority = 1)]
    /// ISR for the cool thresh timer. When a temperature reading is below the threshhold, this timer
    /// starts. When it times out, this ISR runs, putting the sensor in a low-power mode, and disables
    /// TWIM and the radio. Code in this ISR enables power-saving functionality for several peripherals.
    fn cool_timer_isr(cx: cool_timer_isr::Context) {
        let cool_thresh_timer = cx.shared.cool_thresh_timer;
        let twim = cx.shared.twim;
        let rtc = cx.shared.rtc;
        let scl = cx.shared.scl;

        defmt::info!("SLEEPING");

        (cool_thresh_timer, twim, rtc, scl).lock(|cool_thresh_timer, twim, rtc, scl| {
            cool_thresh_timer.clear_interrupt(2);

            COOL_THRESH_TIMER_RUNNING.store(false, Ordering::Release);

            OpMode::Idle.store(Ordering::Release);
            rtc.set_timeout(RtcCompare::Compare0, SLEEP_TIME_IDLE)
                .unwrap();

            if SENSOR_SLEEPING
                .compare_exchange(false, true, Ordering::Release, Ordering::Relaxed)
                .is_ok()
            {
                sensor::sleep(twim, scl);
            }

            twim.disable();
        });
    }

    #[task(binds = RTC0, shared = [cool_thresh_timer, rtc, twim, scl, sda, sensor_wake_timer], local = [esb_app, delay, pid], priority = 1)]
    /// Wake the device up, take a reading, and transmit it. Change between active and
    /// idle modes based on if the reading indicates the stove is on and/or hot. This ISR
    /// contains the bulk of our execution logic.
    fn rtc_isr(cx: rtc_isr::Context) {
        let cool_thresh_timer = cx.shared.cool_thresh_timer;
        let twim = cx.shared.twim;
        let rtc = cx.shared.rtc;
        let scl = cx.shared.scl;
        let sda = cx.shared.sda;
        let sensor_wake_timer = cx.shared.sensor_wake_timer;

        (cool_thresh_timer, twim, rtc, scl, sda, sensor_wake_timer).lock(
            |cool_thresh_timer, twim, rtc, scl, sda, sensor_wake_timer| {
                rtc.reset_event(RtcInterrupt::Compare0);
                rtc.clear_counter();

                if !twim.is_enabled() {
                    twim.enable();
                }

                if SENSOR_SLEEPING
                    .compare_exchange(true, false, Ordering::Release, Ordering::Relaxed)
                    .is_ok()
                {
                    sensor::wake(scl, sda, sensor_wake_timer);
                }

                let reading = sensor::read_temp(twim).unwrap_or(TEMP_ERROR_FLAG);
                // defmt::info!("READING: {}", reading);

                // Sometimes after sensor wake, our readings are -273.15. Unknown cause. Note that this is the same
                // reading we get if attempting to take a reading while the sensor is asleep.
                let temp_valid =
                    (reading - TEMP_ERROR_FLAG).abs() > 0.01 && (reading - -273.15).abs() > 0.01;

                if !temp_valid {
                    return;
                }

                // If temperature is above a thresh, set the active sleep time. Otherwise,
                // command idle, which will execute power-saving steps, and us a longer sleep time,
                // after a timer expires.
                let mode = OpMode::load(Ordering::Acquire);

                if reading >= SHOW_READING_THRESH {
                    cool_thresh_timer.stop();
                    cool_thresh_timer.clear();
                    COOL_THRESH_TIMER_RUNNING.store(false, Ordering::Release);

                    if let OpMode::Idle = mode {
                        OpMode::Active.store(Ordering::Release);
                        rtc.set_timeout(RtcCompare::Compare0, SLEEP_TIME_ACTIVE)
                            .unwrap();
                    }
                } else {
                    match mode {
                        OpMode::Active => {
                            // If we were active but are now below the thresh, and haven't already,
                            // start the cool thresh timer.

                            if COOL_THRESH_TIMER_RUNNING
                                .compare_exchange(false, true, Ordering::Release, Ordering::Relaxed)
                                .is_ok()
                            {
                                cool_thresh_timer.start();
                            }
                        }
                        OpMode::Idle => {
                            // We just woke up the sensor, but don't need to go active; go back to sleep.
                            sensor::sleep(twim, scl);

                            SENSOR_SLEEPING.store(true, Ordering::Release);
                        } // We should never enter the RTC from the menu; only Active and Idle should show up.
                    }
                }

                let mode = OpMode::load(Ordering::Acquire);

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

                    transmit(&msg, twim, cx.local.esb_app, cx.local.delay, cx.local.pid);
                }
            },
        );
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

//! Code related to reading from the MLX90614 sensor, setting its emissivity,
//! and enter and waking from its low power mode.

use cortex_m::{asm::wfi, delay::Delay};

use nrf_hal::{
    gpio::{Dir, Pin},
    pac::{P0, TIMER1, TWIM0},
    twim::Twim,
};

const SENSOR_ADDR: u8 = 0x5A;
const SLEEP_CMD: u8 = 0xFF;
// MLX90614 Datasheet, section 8.4.5. Read flags:
const OBJ1_CMD: u8 = 0x07;
const EMIS_EEPROM_CMD: u8 = 0x04 | 0b0010_0000;

pub enum SensorError {
    /// I²C bus error
    _I2C, // currently unused. Ie if we add error handling to `stm_hal2::i2c`.
    /// CRC checksum mismatch (PEC)
    _ChecksumMismatch,
}

/// Read the temperature from the sensor.
pub fn read_temp(twim: &mut Twim<TWIM0>) -> Result<f32, SensorError> {
    let mut data = [0; 3];
    twim.write_read(SENSOR_ADDR, &[OBJ1_CMD], &mut data).ok();

    let reading = u16::from_le_bytes([data[0], data[1]]);
    Ok(reading as f32 * 0.02 - 273.15)
}

/// Read emissivity from the sensor.
pub fn read_emissivity(twim: &mut Twim<TWIM0>) -> Result<f32, SensorError> {
    let mut data = [0; 3];
    twim.write_read(SENSOR_ADDR, &[EMIS_EEPROM_CMD], &mut data)
        .ok();

    let reading = u16::from_le_bytes([data[0], data[1]]);
    Ok(reading as f32 / 65_535.)
}

/// Write emissivity to the sensor. Emissivity is set using the ERPROM, and stays
/// set even when powered down.
pub fn set_emissivity(
    emis: f32,
    twim: &mut Twim<TWIM0>,
    delay: &mut Delay,
) -> Result<(), SensorError> {
    if !(0.1..=1.).contains(&emis) {
        panic!("Emissivity must be between 0.1 and 1.0.")
    }

    defmt::warn!("SETTING EMIS: {}", emis);

    let eps = (emis * 65535.) as u16;

    let pec = smbus_pec::pec(&[SENSOR_ADDR << 1, EMIS_EEPROM_CMD, 0, 0]); // ?

    // Datasheet, section 8.3.3.1 - When EEPROM cell content is to be changed Melexis recommends following procedure:
    // 1. Power up the device
    // 2. Write 0x0000 into the cell of interest (effectively erasing the cell)
    twim.write(SENSOR_ADDR, &[EMIS_EEPROM_CMD, 0, 0, pec]).ok();

    // 3. Wait at least 5ms (10ms to be on the safe side)
    delay.delay_ms(10);

    // 4. Write the new value
    let pec = smbus_pec::pec(&[
        SENSOR_ADDR << 1,
        EMIS_EEPROM_CMD,
        eps as u8,
        (eps >> 8) as u8,
    ]); // todo TS
    twim.write(
        SENSOR_ADDR,
        &[EMIS_EEPROM_CMD, eps as u8, (eps >> 8) as u8, pec],
    )
    .ok();
    // 5. Wait at least 5ms (10ms to be on the safe side)
    // 6. Read back and compare if the write was successful
    // let mut buf = [0, 0];
    // i2c.write_read(SENSOR_ADDR, &[EMIS_EEPROM_CMD], &mut buf);
    // 7. Power down (to be sure that when power up next time the changes will take place)
    // (We're skipping steps 6 and 7.)

    Ok(())
}

pub fn sleep(twim: &mut Twim<TWIM0>, scl: &mut Pin) {
    let pec = smbus_pec::pec(&[SENSOR_ADDR << 1, SLEEP_CMD]);

    twim.write(SENSOR_ADDR, &[SLEEP_CMD, pec]).ok();

    // Set SCL low to reduce parasitic power loss in sleep mode, as directed by the
    // MLX90614 datasheet.
    // "NOTE: In order to limit the current consumption to the typical 2.5μA Melexis
    // recommends that the SCL pin is kept low during sleep as there is leakage current
    // trough the internal synthesized zener diode connected to SCL pin. This may be
    // achieved by configuring the MD driver of SCL pin as Push-Pull and not having
    // Pull-Up resistor connected on SCL line."
    // We have it configured this way.

    scl.dir(Dir::Output);
    scl.set_low();
}

/// Wake from sleep mode. See datasheet, section 8.4.8.
/// "SCL pin high and then PWM/SDA pin low for at least t_(DDQ) > 33ms"
/// Note: "On-chip IIR filter is skipped for the very first measurement (post-wake)"
// pub fn wake(scl: &mut Pin, sda: &mut Pin, timer: &mut Timer<TIMER1>) {
pub fn wake() {
    // Note that this function uses the PAC directly, since its async delays using timers
    // preclude normal resource sharing in a CS. (CSs stop the timer ASMs from working properly)

    // Set SCL and SDA to output pins so we can set them manually.
    // todo: We probably don't need to set SCL to be an output pin here, since
    // todo it's already set that way during the sleep procedure.

    // Don't allow RTC interrupts to fire here, eg during our WFI delays. They shouldn't, but
    // just in case.

    unsafe {
        // scl.dir(Dir::Output);
        (*P0::ptr()).dirset.write(|w| w.bits(1 << 20));

        // sda.dir(Dir::Output);
        (*P0::ptr()).dirset.write(|w| w.bits(1 << 24));

        (*P0::ptr()).pin_cnf[24].modify(|_, w| {
            // sda.drive(Drive::S0S1);
            w.drive().bits(0);

            // sda.pull(Pull::Disabled);
            w.pull().bits(0)
        });
    }

    // Disconnect TWIM from the pins as well, or we won't be able to manually
    // set the pins.
    unsafe {
        (*TWIM0::ptr())
            .psel
            .scl
            .modify(|_, w| w.connect().set_bit());
        (*TWIM0::ptr())
            .psel
            .sda
            .modify(|_, w| w.connect().set_bit());
    }

    // scl.set_high();
    unsafe { (*P0::ptr()).outset.write(|w| w.bits(1 << 20)) }
    // sda.set_low();
    unsafe { (*P0::ptr()).outclr.write(|w| w.bits(1 << 24)) }

    unsafe {
        // timer.set_period(0.040, 1); // > 33ms
        (*TIMER1::ptr()).cc[1].write(|w| w.bits(320_000));

        // timer.start();
        (*TIMER1::ptr()).tasks_start.write(|w| w.bits(1));
    }

    wfi();

    // sda.set_high();
    unsafe { (*P0::ptr()).outset.write(|w| w.bits(1 << 24)) }

    // Reset previous pin settings for TWIM
    unsafe {
        // scl.dir(Dir::Input);
        (*P0::ptr()).dirclr.write(|w| w.bits(1 << 20));

        // sda.dir(Dir::Input);
        (*P0::ptr()).dirset.write(|w| w.bits(1 << 24));

        (*P0::ptr()).pin_cnf[24].modify(|_, w| {
            // sda.drive(Drive::S0D1);
            w.drive().bits(6);

            // sda.pull(Pull::Up);
            w.pull().bits(3)
        });
    }

    // Datasheet: After wake up the first data is available after 0.25 seconds (typ).

    unsafe {
        // timer.set_period(0.3, 1);
        (*TIMER1::ptr()).cc[1].write(|w| w.bits(2_400_000));

        // timer.start();
        (*TIMER1::ptr()).tasks_start.write(|w| w.bits(1));
    }

    wfi();

    unsafe {
        (*TWIM0::ptr())
            .psel
            .scl
            .modify(|_, w| w.connect().clear_bit());
        (*TWIM0::ptr())
            .psel
            .sda
            .modify(|_, w| w.connect().clear_bit());
    }
}

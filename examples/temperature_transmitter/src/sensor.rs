//! Code related to reading from the MLX90614 sensor, setting its emissivity,
//! and enter and waking from its low power mode.

use cortex_m::{asm::wfi, delay::Delay};

use nrf_hal::{
    gpio::{Dir, Drive, Pin, Pull},
    pac::{P0, TIMER1, TWIM0},
    timer::Timer,
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
pub fn _read_emissivity(twim: &mut Twim<TWIM0>) -> Result<f32, SensorError> {
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

    unsafe {
        (*TWIM0::ptr())
            .psel
            .scl
            .modify(|_, w| w.connect().set_bit());
    }

    scl.dir(Dir::Output);
    scl.set_low();
}

/// Wake from sleep mode. See datasheet, section 8.4.8.
/// "SCL pin high and then PWM/SDA pin low for at least t_(DDQ) > 33ms"
/// Note: "On-chip IIR filter is skipped for the very first measurement (post-wake)"
pub fn wake(scl: &mut Pin, sda: &mut Pin, timer: &mut Timer<TIMER1>) {
    // Set SCL and SDA to output pins so we can set them manually.
    // todo: We probably don't need to set SCL to be an output pin here, since
    // todo it's already set that way during the sleep procedure.

    // Don't allow RTC interrupts to fire here, eg during our WFI delays. They shouldn't, but
    // just in case.

    // Disconnect TWIM from the pins as well, or we won't be able to manually
    // set them pins.
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

    scl.dir(Dir::Output);
    sda.dir(Dir::Output);

    sda.drive(Drive::S0S1);
    sda.pull(Pull::Disabled);

    scl.set_high();
    sda.set_low();

    timer.set_period(0.040, 1); // > 33ms
    timer.start();

    wfi();

    sda.set_high();

    // Reset previous pin settings for TWIM
    scl.dir(Dir::Input);
    sda.dir(Dir::Input);
    sda.drive(Drive::S0D1);
    sda.pull(Pull::Up);

    // Datasheet: After wake up the first data is available after 0.25 seconds (typ).

    timer.set_period(0.3, 1);
    timer.start();

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

//! This library provides high-level access to nRF-52 and nRF-53 peripherals. WIP. Uses
//! a similar API to [STM32-HAL](https://github.com/David-OConnor/stm32-hal);
//! designed to be interchangeable when able.
//!
//! See the Readme on crates.io or Github for more details.

#![no_std]

// Re-export of the [svd2rust](https://crates.io/crates/svd2rust) auto-generated API for
// stm32 peripherals.

#[cfg(feature = "52810")]
pub use nrf52810_pac as pac;

#[cfg(feature = "52811")]
pub use nrf52811_pac as pac;

#[cfg(feature = "52832")]
pub use nrf52832_pac as pac;

#[cfg(feature = "52833")]
pub use nrf52833_pac as pac;

#[cfg(feature = "52840")]
pub use nrf52840_pac as pac;

#[cfg(feature = "5340")]
pub use nrf5340_app_pac as pac;

#[cfg(feature = "5340")]
pub use nrf5340_net_pac as net_pac;

pub mod clocks;
pub mod gpio;
pub mod gpiote;
pub mod rtc;
pub mod spim;
pub mod timer;
pub mod twim;
// pub mod uarte;  // todo Relies on timer functionality we changed

#[cfg(feature = "usb")]
pub mod usb;

// Modules, functions, and structs below taken directly from `nrf-rs` `nrf-hal`.

/// Length of Nordic EasyDMA differs for MCUs
pub mod target_constants {
    #[cfg(feature = "52805")]
    pub const EASY_DMA_SIZE: usize = (1 << 14) - 1;
    #[cfg(feature = "52810")]
    pub const EASY_DMA_SIZE: usize = (1 << 10) - 1;
    #[cfg(feature = "52811")]
    pub const EASY_DMA_SIZE: usize = (1 << 14) - 1;
    #[cfg(feature = "52820")]
    pub const EASY_DMA_SIZE: usize = (1 << 15) - 1;
    #[cfg(feature = "52832")]
    pub const EASY_DMA_SIZE: usize = (1 << 8) - 1;
    #[cfg(feature = "52833")]
    pub const EASY_DMA_SIZE: usize = (1 << 16) - 1;
    #[cfg(feature = "52840")]
    pub const EASY_DMA_SIZE: usize = (1 << 16) - 1;
    #[cfg(feature = "5340")]
    pub const EASY_DMA_SIZE: usize = (1 << 16) - 1;
    #[cfg(feature = "9160")]
    pub const EASY_DMA_SIZE: usize = (1 << 12) - 1;

    // Limits for Easy DMA - it can only read from data ram
    pub const SRAM_LOWER: usize = 0x2000_0000;
    pub const SRAM_UPPER: usize = 0x3000_0000;

    #[cfg(any(feature = "52810", feature = "52832"))]
    pub const FORCE_COPY_BUFFER_SIZE: usize = 255;
    #[cfg(not(any(feature = "52810", feature = "52832")))]
    pub const FORCE_COPY_BUFFER_SIZE: usize = 1024;
    const _CHECK_FORCE_COPY_BUFFER_SIZE: usize = EASY_DMA_SIZE - FORCE_COPY_BUFFER_SIZE;
    // ERROR: FORCE_COPY_BUFFER_SIZE must be <= EASY_DMA_SIZE
}

/// Does this slice reside entirely within RAM?
pub(crate) fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= target_constants::SRAM_LOWER && (ptr + slice.len()) < target_constants::SRAM_UPPER
}

/// Return an error if slice is not in RAM.
pub(crate) fn slice_in_ram_or<T>(slice: &[u8], err: T) -> Result<(), T> {
    if slice_in_ram(slice) {
        Ok(())
    } else {
        Err(err)
    }
}

/// A handy structure for converting rust slices into ptr and len pairs
/// for use with EasyDMA. Care must be taken to make sure mutability
/// guarantees are respected
pub(crate) struct DmaSlice {
    ptr: u32,
    len: u32,
}

impl DmaSlice {
    pub fn null() -> Self {
        Self { ptr: 0, len: 0 }
    }

    pub fn from_slice(slice: &[u8]) -> Self {
        Self {
            ptr: slice.as_ptr() as u32,
            len: slice.len() as u32,
        }
    }
}

// todo: should these helper macros be removed from this library? It has nothing to do with nRF.
// todo: Copy+paste from stm32-hal.

/// Syntax helper for getting global variables of the form `Mutex<RefCell<Option>>>` from an interrupt-free
/// context - eg in interrupt handlers.
///
/// Example: `access_global!(DELAY, delay, cs)`
#[macro_export]
macro_rules! access_global {
    ($NAME_GLOBAL:ident, $name_local:ident, $cs:expr) => {
        let mut part1 = $NAME_GLOBAL.borrow($cs).borrow_mut();
        let $name_local = part1.as_mut().unwrap();
    };
}

/// Syntax helper for setting global variables of the form `Mutex<RefCell<Option>>>`.
/// eg in interrupt handlers. Ideal for non-copy-type variables that can't be initialized
/// immediatiately.
///
/// Example: `make_globals!(
///     (USB_SERIAL, SerialPort<UsbBusType>),
///     (DELAY, Delay),
/// )`
#[macro_export]
macro_rules! make_globals {
    ($(($NAME:ident, $type:ty)),+) => {
        $(
            static $NAME: Mutex<RefCell<Option<$type>>> = Mutex::new(RefCell::new(None));
        )+
    };
}

/// Syntax helper for setting global variables of the form `Mutex<Cell<>>>`.
/// eg in interrupt handlers. Ideal for copy-type variables.
///
/// Example: `make_simple_globals!(
///     (VALUE, f32, 2.),
///     (SETTING, Setting, Setting::A),
/// )`
#[macro_export]
macro_rules! make_simple_globals {
    ($(($NAME:ident, $type:ty, $val:expr)),+) => {
        $(
            static $NAME: Mutex<Cell<$type>> = Mutex::new(Cell::new($val));
        )+
    };
}

/// In the prelude, we export helper macros.
pub mod prelude {
    pub use access_global;
    pub use make_globals;
    pub use make_simple_globals;
}

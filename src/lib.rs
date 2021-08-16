//! This library provides high-level access to nRF-52 and nRF-53 peripherals. WIP. Uses
//! a similar API to [STM32-HAL](https://github.com/David-OConnor/stm32-hal);
//! designed to be interchangeable when able.

//! ## Most features missing!
//! This is currently intended to use the specific features required by AnyLeaf projects
//! that use nrf-52. It may be expanded to be more general at some point in the future,
//! but that's not on the near-term road map. Please use the [nrf-rs](https://github.com/nrf-rs) libraries instead.

//! Please see the [Readme](https://github.com/David-OConnor/nrf-hal/blob/main/README.md) for a detailed overview,
//! and the [examples folder on Github](https://github.com/David-OConnor/nrf-hal/tree/main/examples)
//! for example code and project structure.

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
pub mod spim;
pub mod rtc;
pub mod twim;

#[cfg(feature = "usb")]
pub mod usb;

// Modules, functions, and structs below taken directly from `nrf-rs` `nrf-hal`.

/// Length of Nordic EasyDMA differs for MCUs
pub mod target_constants {
    #[cfg(feature = "51")]
    pub const EASY_DMA_SIZE: usize = (1 << 8) - 1;
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

    #[cfg(any(feature = "51", feature = "52810", feature = "52832"))]
    pub const FORCE_COPY_BUFFER_SIZE: usize = 255;
    #[cfg(not(any(feature = "51", feature = "52810", feature = "52832")))]
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
#[cfg(not(feature = "51"))]
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
#[cfg(not(feature = "51"))]
pub(crate) struct DmaSlice {
    ptr: u32,
    len: u32,
}

#[cfg(not(feature = "51"))]
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

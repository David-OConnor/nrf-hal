//! Taken directly from [nrf-hal](https://github.com/nrf-rs/nrf-hal)
//!
//! HAL interface to the SPIM peripheral.
//!
//! See product specification, chapter 31.

use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

#[cfg(feature = "9160")]
use crate::pac::{spim0_ns as spim0, SPIM0_NS as SPIM0};

#[cfg(not(feature = "9160"))]
use crate::pac::{spim0, SPIM0};

pub use spim0::frequency::FREQUENCY_A as SpimFreq;

use core::iter::repeat_with;

#[cfg(feature = "52811")]
use crate::pac::SPIM1;

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use crate::pac::{SPIM1, SPIM2};

#[cfg(any(feature = "52833", feature = "52840"))]
use crate::pac::SPIM3;

use crate::gpio::Pin;
use crate::target_constants::{EASY_DMA_SIZE, FORCE_COPY_BUFFER_SIZE};
use crate::{slice_in_ram, slice_in_ram_or, DmaSlice};

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::OutputPin;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Clock polarity
pub enum SpiPolarity {
    /// Clock signal low when idle
    IdleLow = 0,
    /// Clock signal high when idle
    IdleHigh = 1,
}

// SpiPhase, and SpiMode copy+pasted from `stm32-hal`, with addition of PartialEq.

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Clock phase
pub enum SpiPhase {
    /// Data in "captured" on the first clock transition
    CaptureOnFirstTransition = 0,
    /// Data in "captured" on the second clock transition
    CaptureOnSecondTransition = 1,
}

/// SPI mode
#[derive(Clone, Copy, PartialEq)]
pub struct SpiMode {
    /// Clock polarity
    pub polarity: SpiPolarity,
    /// Clock phase
    pub phase: SpiPhase,
}

impl SpiMode {
    /// Set Spi Mode 0: Idle low, capture on first transition.
    pub fn mode0() -> Self {
        Self {
            polarity: SpiPolarity::IdleLow,
            phase: SpiPhase::CaptureOnFirstTransition,
        }
    }

    /// Set Spi Mode 1: Idle low, capture on second transition.
    pub fn mode1() -> Self {
        Self {
            polarity: SpiPolarity::IdleLow,
            phase: SpiPhase::CaptureOnSecondTransition,
        }
    }

    /// Set Spi Mode 2: Idle high, capture on first transition.
    pub fn mode2() -> Self {
        Self {
            polarity: SpiPolarity::IdleHigh,
            phase: SpiPhase::CaptureOnFirstTransition,
        }
    }

    /// Set Spi Mode 3: Idle high, capture on second transition.
    pub fn mode3() -> Self {
        Self {
            polarity: SpiPolarity::IdleHigh,
            phase: SpiPhase::CaptureOnSecondTransition,
        }
    }
}

/// Interface to a SPIM instance.
///
/// This is a very basic interface that comes with the following limitations:
/// - The SPIM instances share the same address space with instances of SPIS,
///   SPI, TWIM, TWIS, and TWI. You need to make sure that conflicting instances
///   are disabled before using `Spim`. See product specification, section 15.2.
pub struct Spim<T>(T);

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<T> embedded_hal::blocking::spi::Transfer<u8> for Spim<T>
where
    T: Instance,
{
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        // If the slice isn't in RAM, we can't write back to it at all
        slice_in_ram_or(words, Error::DMABufferNotInDataMemory)?;

        words.chunks(EASY_DMA_SIZE).try_for_each(|chunk| {
            self.do_spi_dma_transfer(DmaSlice::from_slice(chunk), DmaSlice::from_slice(chunk))
        })?;

        Ok(words)
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl<T> embedded_hal::blocking::spi::Write<u8> for Spim<T>
where
    T: Instance,
{
    type Error = Error;

    fn write<'w>(&mut self, words: &'w [u8]) -> Result<(), Error> {
        // Mask on segment where Data RAM is located on nrf52840 and nrf52832
        // Upper limit is choosen to entire area where DataRam can be placed
        let needs_copy = !slice_in_ram(words);

        let chunk_sz = if needs_copy {
            FORCE_COPY_BUFFER_SIZE
        } else {
            EASY_DMA_SIZE
        };

        let step = if needs_copy {
            Self::spi_dma_copy
        } else {
            Self::spi_dma_no_copy
        };

        words.chunks(chunk_sz).try_for_each(|c| step(self, c))
    }
}
impl<T> Spim<T>
where
    T: Instance,
{
    fn spi_dma_no_copy(&mut self, chunk: &[u8]) -> Result<(), Error> {
        self.do_spi_dma_transfer(DmaSlice::from_slice(chunk), DmaSlice::null())
    }

    fn spi_dma_copy(&mut self, chunk: &[u8]) -> Result<(), Error> {
        let mut buf = [0u8; FORCE_COPY_BUFFER_SIZE];
        buf[..chunk.len()].copy_from_slice(chunk);

        self.do_spi_dma_transfer(DmaSlice::from_slice(&buf[..chunk.len()]), DmaSlice::null())
    }

    pub fn new(
        spim: T,
        sck: &Pin,
        mosi: &Pin,
        miso: &Pin,
        frequency: SpimFreq,
        mode: SpiMode,
        orc: u8,
    ) -> Self {
        // Select pins.

        // todo: You may need to feature gate the port setting for variants with more
        // todo than P0 and P1 (ie 53)

        spim.psel.sck.write(|w| unsafe {
            w.port().bit(sck.port as u8 != 0);
            w.pin().bits(sck.pin);
            w.connect().set_bit()
        });

        spim.psel.mosi.write(|w| unsafe {
            w.port().bit(mosi.port as u8 != 0);
            w.pin().bits(mosi.pin);
            w.connect().set_bit()
        });

        spim.psel.miso.write(|w| unsafe {
            w.port().bit(miso.port as u8 != 0);
            w.pin().bits(miso.pin);
            w.connect().set_bit()
        });

        // Enable SPIM instance.
        spim.enable.write(|w| w.enable().enabled());

        // Configure mode.
        spim.config.write(|w| {
            // Can't match on `mode` due to embedded-hal, see https://github.com/rust-embedded/embedded-hal/pull/126
            if mode == SpiMode::mode1() {
                w.order().msb_first();
                w.cpol().active_high();
                w.cpha().leading();
            } else if mode == SpiMode::mode1() {
                w.order().msb_first();
                w.cpol().active_high();
                w.cpha().trailing();
            } else if mode == SpiMode::mode2() {
                w.order().msb_first();
                w.cpol().active_low();
                w.cpha().leading();
            } else {
                w.order().msb_first();
                w.cpol().active_low();
                w.cpha().trailing();
            }
            w
        });

        // Configure frequency.
        spim.frequency.write(|w| w.frequency().variant(frequency));

        // Set over-read character to `0`.
        spim.orc.write(|w|
            // The ORC field is 8 bits long, so `0` is a valid value to write
            // there.
            unsafe { w.orc().bits(orc) });

        Spim(spim)
    }

    /// Internal helper function to setup and execute SPIM DMA transfer.
    fn do_spi_dma_transfer(&mut self, tx: DmaSlice, rx: DmaSlice) -> Result<(), Error> {
        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started.
        compiler_fence(SeqCst);

        // Set up the DMA write.
        self.0.txd.ptr.write(|w| unsafe { w.ptr().bits(tx.ptr) });

        self.0.txd.maxcnt.write(|w|
            // Note that that nrf52840 maxcnt is a wider.
            // type than a u8, so we use a `_` cast rather than a `u8` cast.
            // The MAXCNT field is thus at least 8 bits wide and accepts the full
            // range of values that fit in a `u8`.
            unsafe { w.maxcnt().bits(tx.len as _ ) });

        // Set up the DMA read.
        self.0.rxd.ptr.write(|w|
            // This is safe for the same reasons that writing to TXD.PTR is
            // safe. Please refer to the explanation there.
            unsafe { w.ptr().bits(rx.ptr) });
        self.0.rxd.maxcnt.write(|w|
            // This is safe for the same reasons that writing to TXD.MAXCNT is
            // safe. Please refer to the explanation there.
            unsafe { w.maxcnt().bits(rx.len as _) });

        // Start SPI transaction.
        self.0.tasks_start.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed.
        compiler_fence(SeqCst);

        // Wait for END event.
        //
        // This event is triggered once both transmitting and receiving are
        // done.
        while self.0.events_end.read().bits() == 0 {}

        // Reset the event, otherwise it will always read `1` from now on.
        self.0.events_end.write(|w| w);

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed.
        compiler_fence(SeqCst);

        if self.0.txd.amount.read().bits() != tx.len {
            return Err(Error::Transmit);
        }
        if self.0.rxd.amount.read().bits() != rx.len {
            return Err(Error::Receive);
        }
        Ok(())
    }

    /// Read and write from a SPI slave, using a single buffer.
    ///
    /// This method implements a complete read transaction, which consists of
    /// the master transmitting what it wishes to read, and the slave responding
    /// with the requested data.
    ///
    /// Uses the provided chip select pin to initiate the transaction. Transmits
    /// all bytes in `buffer`, then receives an equal number of bytes.
    pub fn transfer(&mut self, chip_select: &mut Pin, buffer: &mut [u8]) -> Result<(), Error> {
        slice_in_ram_or(buffer, Error::DMABufferNotInDataMemory)?;

        chip_select.set_low();

        // Don't return early, as we must reset the CS pin.
        let res = buffer.chunks(EASY_DMA_SIZE).try_for_each(|chunk| {
            self.do_spi_dma_transfer(DmaSlice::from_slice(chunk), DmaSlice::from_slice(chunk))
        });

        chip_select.set_high();

        res
    }

    /// Read and write from a SPI slave, using separate read and write buffers.
    ///
    /// This method implements a complete read transaction, which consists of
    /// the master transmitting what it wishes to read, and the slave responding
    /// with the requested data.
    ///
    /// Uses the provided chip select pin to initiate the transaction. Transmits
    /// all bytes in `tx_buffer`, then receives bytes until `rx_buffer` is full.
    ///
    /// If `tx_buffer.len() != rx_buffer.len()`, the transaction will stop at the
    /// smaller of either buffer.
    pub fn transfer_split_even(
        &mut self,
        chip_select: &mut Pin,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
    ) -> Result<(), Error> {
        // NOTE: RAM slice check for `rx_buffer` is not necessary, as a mutable
        // slice can only be built from data located in RAM.
        slice_in_ram_or(tx_buffer, Error::DMABufferNotInDataMemory)?;

        let txi = tx_buffer.chunks(EASY_DMA_SIZE);
        let rxi = rx_buffer.chunks_mut(EASY_DMA_SIZE);

        chip_select.set_low();

        // Don't return early, as we must reset the CS pin
        let res = txi.zip(rxi).try_for_each(|(t, r)| {
            self.do_spi_dma_transfer(DmaSlice::from_slice(t), DmaSlice::from_slice(r))
        });

        chip_select.set_high();

        res
    }

    /// Read and write from a SPI slave, using separate read and write buffers.
    ///
    /// This method implements a complete read transaction, which consists of
    /// the master transmitting what it wishes to read, and the slave responding
    /// with the requested data.
    ///
    /// Uses the provided chip select pin to initiate the transaction. Transmits
    /// all bytes in `tx_buffer`, then receives bytes until `rx_buffer` is full.
    ///
    /// This method is more complicated than the other `transfer` methods because
    /// it is allowed to perform transactions where `tx_buffer.len() != rx_buffer.len()`.
    /// If this occurs, extra incoming bytes will be discarded, OR extra outgoing bytes
    /// will be filled with the `orc` value.
    pub fn transfer_split_uneven(
        &mut self,
        chip_select: &mut Pin,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
    ) -> Result<(), Error> {
        // NOTE: RAM slice check for `rx_buffer` is not necessary, as a mutable
        // slice can only be built from data located in RAM.
        slice_in_ram_or(tx_buffer, Error::DMABufferNotInDataMemory)?;

        // For the tx and rx, we want to return Some(chunk)
        // as long as there is data to send. We then chain a repeat to
        // the end so once all chunks have been exhausted, we will keep
        // getting Nones out of the iterators.
        let txi = tx_buffer
            .chunks(EASY_DMA_SIZE)
            .map(Some)
            .chain(repeat_with(|| None));

        let rxi = rx_buffer
            .chunks_mut(EASY_DMA_SIZE)
            .map(Some)
            .chain(repeat_with(|| None));

        chip_select.set_low();

        // We then chain the iterators together, and once BOTH are feeding
        // back Nones, then we are done sending and receiving.
        //
        // Don't return early, as we must reset the CS pin.
        let res = txi
            .zip(rxi)
            .take_while(|(t, r)| t.is_some() || r.is_some())
            // We also turn the slices into either a DmaSlice (if there was data), or a null
            // DmaSlice (if there is no data).
            .map(|(t, r)| {
                (
                    t.map(|t| DmaSlice::from_slice(t))
                        .unwrap_or_else(DmaSlice::null),
                    r.map(|r| DmaSlice::from_slice(r))
                        .unwrap_or_else(DmaSlice::null),
                )
            })
            .try_for_each(|(t, r)| self.do_spi_dma_transfer(t, r));

        chip_select.set_high();

        res
    }

    /// Write to an SPI slave.
    ///
    /// This method uses the provided chip select pin to initiate the
    /// transaction, then transmits all bytes in `tx_buffer`. All incoming
    /// bytes are discarded.
    pub fn write(&mut self, chip_select: &mut Pin, tx_buffer: &[u8]) -> Result<(), Error> {
        slice_in_ram_or(tx_buffer, Error::DMABufferNotInDataMemory)?;
        self.transfer_split_uneven(chip_select, tx_buffer, &mut [0u8; 0])
    }

    /// Return the raw interface to the underlying SPIM peripheral.
    pub fn free(self) -> T {
        self.0
    }
}

#[derive(Debug)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    /// EasyDMA can only read from data memory, read only buffers in flash will fail.
    DMABufferNotInDataMemory,
    Transmit,
    Receive,
}

/// Implemented by all SPIM instances.
pub trait Instance: Deref<Target = spim0::RegisterBlock> + sealed::Sealed {}

mod sealed {
    pub trait Sealed {}
}

impl sealed::Sealed for SPIM0 {}
impl Instance for SPIM0 {}

#[cfg(any(
    feature = "52832",
    feature = "52833",
    feature = "52840",
    feature = "52811"
))]
mod _spim1 {
    use super::*;
    impl Instance for SPIM1 {}
    impl sealed::Sealed for SPIM1 {}
}

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
mod _spim2 {
    use super::*;
    impl Instance for SPIM2 {}
    impl sealed::Sealed for SPIM2 {}
}

#[cfg(any(feature = "52833", feature = "52840"))]
mod _spim3 {
    use super::*;
    impl Instance for SPIM3 {}
    impl sealed::Sealed for SPIM3 {}
}

use crate::pac::USBD;

pub use nrf_usbd::Usbd;

pub struct UsbPeripheral<'a> {
    regs: USBD,
}

impl<'a> UsbPeripheral<'a> {
    pub fn new(regs: USBD) -> Self {
        Self { _usbd: regs }
    }
}

unsafe impl<'a> nrf_usbd::UsbPeripheral for UsbPeripheral<'a> {
    const REGISTERS: *const () = USBD::ptr() as *const _;
}

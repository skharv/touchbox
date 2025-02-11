use core::default::Default;
use fugit::ExtU32;
use mcp230xx::Level;
use packed_struct::prelude::*;
use usb_device::bus::UsbBus;
use usb_device::class_prelude::UsbBusAllocator;
use usbd_human_interface_device::{
    descriptor::InterfaceProtocol,
    device::DeviceClass,
    interface::{
        InBytes8, Interface, InterfaceBuilder, InterfaceConfig, OutNone, ReportSingle,
        UsbAllocatable,
    },
    UsbHidError,
};

#[rustfmt::skip]
pub const CLASSIC_DESCRIPTOR: &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x18,        //   Usage Maximum (0x18)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x18,        //   Report Count (24)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
];

#[derive(Clone, Copy, Debug, Eq, PartialEq, Default, PackedStruct)]
#[packed_struct(endian = "lsb", size_bytes = "3")]
pub struct TwinStickReport {
    pub a1: u8,
    pub b1: u8,
    pub a2: u8,
}

pub struct TwinStick<'a, B: UsbBus> {
    interface: Interface<'a, B, InBytes8, OutNone, ReportSingle>,
}

impl<'a, B: UsbBus> TwinStick<'a, B> {
    pub fn write_report(&mut self, report: &TwinStickReport) -> Result<(), UsbHidError> {
        let data = report.pack().map_err(|_| UsbHidError::SerializationError)?;
        self.interface
            .write_report(&data)
            .map(|_| ())
            .map_err(UsbHidError::from)
    }
}

impl<'a, B: UsbBus> DeviceClass<'a> for TwinStick<'a, B> {
    type I = Interface<'a, B, InBytes8, OutNone, ReportSingle>;

    fn interface(&mut self) -> &mut Self::I {
        &mut self.interface
    }

    fn reset(&mut self) {}

    fn tick(&mut self) -> Result<(), UsbHidError> {
        Ok(())
    }
}

pub struct TwinStickConfig<'a> {
    interface: InterfaceConfig<'a, InBytes8, OutNone, ReportSingle>,
}

impl<'a> Default for TwinStickConfig<'a> {
    #[must_use]
    fn default() -> Self {
        let builder = InterfaceBuilder::new(CLASSIC_DESCRIPTOR)
            .unwrap()
            .boot_device(InterfaceProtocol::None)
            .description("Joystick")
            .in_endpoint(1.millis())
            .unwrap()
            .without_out_endpoint()
            .build();

        Self::new(builder)
    }
}

impl<'a> TwinStickConfig<'a> {
    #[must_use]
    pub fn new(interface: InterfaceConfig<'a, InBytes8, OutNone, ReportSingle>) -> Self {
        Self { interface }
    }
}

impl<'a, B: UsbBus + 'a> UsbAllocatable<'a, B> for TwinStickConfig<'a> {
    type Allocated = TwinStick<'a, B>;

    fn allocate(self, usb_alloc: &'a UsbBusAllocator<B>) -> Self::Allocated {
        Self::Allocated {
            interface: Interface::new(usb_alloc, self.interface),
        }
    }
}

pub fn get_twin_stick_report(
    bank_a1: &mut [Level; 8],
    bank_b1: &mut [Level; 8],
    bank_a2: &mut [Level; 8],
) -> TwinStickReport {
    let mut a1 = 0;
    for (idx, pressed) in bank_a1[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            a1 |= 1 << idx;
        }
    }

    let mut b1 = 0;
    for (idx, pressed) in bank_b1[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            b1 |= 1 << idx;
        }
    }

    let mut a2 = 0;
    for (idx, pressed) in bank_a2[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            a2 |= 1 << idx;
        }
    }

    TwinStickReport { a1, b1, a2 }
}

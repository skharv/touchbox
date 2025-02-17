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
pub const ALL_BUTTON_DESCRIPTOR: &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x1A,        //   Usage Maximum (24)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x1A,        //   Report Count (24)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0xC0,              // End Collection
];

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default, PackedStruct)]
#[packed_struct(endian = "lsb", size_bytes = "4")]
pub struct AllButtonReport {
    #[packed_field]
    pub a1: u8,
    #[packed_field]
    pub b1: u8,
    #[packed_field]
    pub a2: u8,
    #[packed_field]
    pub b2: u8,
}

pub struct AllButton<'a, B: UsbBus> {
    interface: Interface<'a, B, InBytes8, OutNone, ReportSingle>,
}

impl<'a, B: UsbBus> AllButton<'a, B> {
    pub fn write_report(&mut self, report: &AllButtonReport) -> Result<(), UsbHidError> {
        let data = report.pack().map_err(|_| UsbHidError::SerializationError)?;
        self.interface
            .write_report(&data)
            .map(|_| ())
            .map_err(UsbHidError::from)
    }
}

impl<'a, B: UsbBus> DeviceClass<'a> for AllButton<'a, B> {
    type I = Interface<'a, B, InBytes8, OutNone, ReportSingle>;

    fn interface(&mut self) -> &mut Self::I {
        &mut self.interface
    }

    fn reset(&mut self) {}

    fn tick(&mut self) -> Result<(), UsbHidError> {
        Ok(())
    }
}

pub struct AllButtonConfig<'a> {
    interface: InterfaceConfig<'a, InBytes8, OutNone, ReportSingle>,
}

impl<'a> Default for AllButtonConfig<'a> {
    #[must_use]
    fn default() -> Self {
        let builder = InterfaceBuilder::new(ALL_BUTTON_DESCRIPTOR)
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

impl<'a> AllButtonConfig<'a> {
    #[must_use]
    pub fn new(interface: InterfaceConfig<'a, InBytes8, OutNone, ReportSingle>) -> Self {
        Self { interface }
    }
}

impl<'a, B: UsbBus + 'a> UsbAllocatable<'a, B> for AllButtonConfig<'a> {
    type Allocated = AllButton<'a, B>;

    fn allocate(self, usb_alloc: &'a UsbBusAllocator<B>) -> Self::Allocated {
        Self::Allocated {
            interface: Interface::new(usb_alloc, self.interface),
        }
    }
}

pub fn get_all_button_report(
    bank_a1: &mut [Level; 8],
    bank_b1: &mut [Level; 8],
    bank_a2: &mut [Level; 8],
    bank_b2: &mut [Level; 8],
) -> AllButtonReport {
    // Reorganise the inputs
    let mut ro_a1: [Level; 8] = [Level::Low; 8];
    let mut ro_b1: [Level; 8] = [Level::Low; 8];
    let mut ro_a2: [Level; 8] = [Level::Low; 8];
    let mut ro_b2: [Level; 8] = [Level::Low; 8];

    ro_a1[..7].copy_from_slice(&bank_a1[..7]); //0-6
    ro_a1[7] = bank_b1[2]; //7
    ro_b1[..5].copy_from_slice(&bank_b1[3..8]); //8-12
    ro_b1[5..8].copy_from_slice(&bank_a2[..3]); //13-15
    ro_a2[..2].copy_from_slice(&bank_a2[3..5]); //16-17
    ro_a2[2..8].copy_from_slice(&bank_b2[..6]); //18-23
    ro_b2[..2].copy_from_slice(&bank_b2[6..8]); //24-25

    let mut a1 = 0;
    for (idx, pressed) in ro_a1[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            a1 |= 1 << idx;
        }
    }

    let mut b1 = 0;
    for (idx, pressed) in ro_b1[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            b1 |= 1 << idx;
        }
    }

    let mut a2 = 0;
    for (idx, pressed) in ro_a2[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            a2 |= 1 << idx;
        }
    }

    let mut b2 = 0;
    for (idx, pressed) in ro_b2[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            b2 |= 1 << idx;
        }
    }
    AllButtonReport { a1, b1, a2, b2 }
}

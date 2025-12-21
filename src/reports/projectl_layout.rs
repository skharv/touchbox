use core::{default::Default, ptr::null};
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
pub const PROJECTL_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x05, // Usage (Gamepad)
    0xA1, 0x01, // Collection (Application)
    // Hat 
    0x05, 0x01, //   Usage Page (Generic Desktop)
    0x09, 0x39, //   Usage Hat Switch   
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x07, //   Logical Maximum (7)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x42, //   Input (Data,Var,Abs,Null)
    // Buttons
    0x05, 0x09, //   Usage Page (Button)
    0x19, 0x01, //   Usage Minimum (1)
    0x29, 0x16, //   Usage Maximum (22)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x16, //   Report Count (22)
    0x81, 0x02, //   Input (Data,Var,Abs)
    // Padding
    0x75, 0x01, //   Report Size (1)
    0x95, 0x07, //   Report Count (07)
    0x81, 0x01, //   Input (Cnst,Ary,Abs)
    // Done
    0xC0,       // End Collection
];

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default, PackedStruct)]
#[packed_struct(endian = "lsb", size_bytes = "4")]
pub struct ProjectLReport {
    #[packed_field]
    pub hat: u8,
    #[packed_field]
    pub buttons1: u8,
    #[packed_field]
    pub buttons2: u8,
    #[packed_field]
    pub buttons3: u8,
}

pub struct ProjectL<'a, B: UsbBus> {
    interface: Interface<'a, B, InBytes8, OutNone, ReportSingle>,
}

impl<'a, B: UsbBus> ProjectL<'a, B> {
    pub fn write_report(&mut self, report: &ProjectLReport) -> Result<(), UsbHidError> {
        let data = report.pack().map_err(|_| UsbHidError::SerializationError)?;
        self.interface
            .write_report(&data)
            .map(|_| ())
            .map_err(UsbHidError::from)
    }
}

impl<'a, B: UsbBus> DeviceClass<'a> for ProjectL<'a, B> {
    type I = Interface<'a, B, InBytes8, OutNone, ReportSingle>;

    fn interface(&mut self) -> &mut Self::I {
        &mut self.interface
    }

    fn reset(&mut self) {}

    fn tick(&mut self) -> Result<(), UsbHidError> {
        Ok(())
    }
}

pub struct ProjectLConfig<'a> {
    interface: InterfaceConfig<'a, InBytes8, OutNone, ReportSingle>,
}

impl<'a> Default for ProjectLConfig<'a> {
    #[must_use]
    fn default() -> Self {
        let builder = InterfaceBuilder::new(PROJECTL_DESCRIPTOR)
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

impl<'a> ProjectLConfig<'a> {
    #[must_use]
    pub fn new(interface: InterfaceConfig<'a, InBytes8, OutNone, ReportSingle>) -> Self {
        Self { interface }
    }
}

impl<'a, B: UsbBus + 'a> UsbAllocatable<'a, B> for ProjectLConfig<'a> {
    type Allocated = ProjectL<'a, B>;

    fn allocate(self, usb_alloc: &'a UsbBusAllocator<B>) -> Self::Allocated {
        Self::Allocated {
            interface: Interface::new(usb_alloc, self.interface),
        }
    }
}

pub fn get_projectl_report(
    bank_a1: &mut [Level; 8],
    bank_b1: &mut [Level; 8],
    bank_a2: &mut [Level; 8],
    bank_b2: &mut [Level; 8],
) -> ProjectLReport {
    let mut hat = 8;
    let mut buttons1 = 0;
    let mut buttons2 = 0;
    let mut buttons3 = 0;

    let left = bank_a1[1] == Level::Low;
    let right = bank_a1[3] == Level::Low;
    let up = bank_b1[6] == Level::Low;
    let down = bank_a1[2] == Level::Low;

    let ol1 = bank_a1[4] == Level::Low;
    let ol2 = bank_a1[5] == Level::Low;
    let ol3 = bank_a1[6] == Level::Low;

    // b1 [0] and [1] are not connected
    let or1 = bank_b1[2] == Level::Low;
    let or2 = bank_b1[3] == Level::Low;
    let or3 = bank_b1[4] == Level::Low;
    // b1[6] is used for up
    let modh = bank_b1[5] == Level::Low;
    let mody = bank_b1[7] == Level::Low;
    let moda = bank_a1[0] == Level::Low;

    let rmid = bank_a2[0] == Level::Low;
    let rright = bank_a2[1] == Level::Low;
    let rup = bank_a2[2] == Level::Low;
    let rdown = bank_a2[4] == Level::Low;
    let rleft = bank_a2[3] == Level::Low;

    let ba = bank_b2[0] == Level::Low;
    let bx = bank_b2[1] == Level::Low;
    let by = bank_b2[2] == Level::Low;
    let bb = bank_b2[3] == Level::Low;
    let rt = bank_b2[4] == Level::Low;
    let rb = bank_b2[5] == Level::Low;
    let lb = bank_b2[6] == Level::Low;
    let lt = bank_b2[7] == Level::Low;

    // Hat
    if up && !down {
        hat = 0;
        if right && !left {
            hat = 1;
        }
        if left && !right {
            hat = 7;
        }
    }
    if down && !up {
        hat = 4;
        if right && !left {
            hat = 3;
        }
        if left && !right {
            hat = 5;
        }
    }
    if left && !right {
        hat = 6;
        if up && !down {
            hat = 7;
        }
        if down && !up {
            hat = 5;
        }
    }
    if right && !left {
        hat = 2;
        if up && !down {
            hat = 1;
        }
        if down && !up {
            hat = 3;
        }
    }

    if ba {
        buttons1 |= 1 << 0;
    }
    if bb {
        buttons1 |= 1 << 1;
    }
    if bx {
        buttons1 |= 1 << 2;
    }
    if by {
        buttons1 |= 1 << 3;
    }
    if lb {
        buttons1 |= 1 << 4;
    }
    if rb {
        buttons1 |= 1 << 5;
    }
    if ol1 {
        buttons1 |= 1 << 6;
    }
    if or1 {
        buttons1 |= 1 << 7;
    }

    if ol3 {
        buttons2 |= 1 << 0;
    }
    if or3 {
        buttons2 |= 1 << 1;
    }
    if lt {
        buttons2 |= 1 << 2;
    }
    if rt {
        buttons2 |= 1 << 3;
    }
    if ol2 {
        buttons2 |= 1 << 4;
    }
    if or2 {
        buttons2 |= 1 << 5;
    }
    if moda {
        buttons2 |= 1 << 6;
    }
    if modh {
        buttons2 |= 1 << 7;
    }

    if mody {
        buttons3 |= 1 << 0;
    }
    if rup {
        buttons3 |= 1 << 1;
    }
    if rdown {
        buttons3 |= 1 << 2;
    }
    if rleft {
        buttons3 |= 1 << 3;
    }
    if rright {
        buttons3 |= 1 << 4;
    }
    if rmid {
        buttons3 |= 1 << 5;
    }

    ProjectLReport {
        hat,
        buttons1,
        buttons2,
        buttons3,
    }
}

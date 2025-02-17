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
pub const TWIN_STICK_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x05, // Usage (Gamepad)
    0xA1, 0x01, // Collection (Application)
    // Analogs
    0x05, 0x01, //   Usage Page (Generic Desktop)     
    0x09, 0x30, //   Usage (X)
    0x09, 0x31, //   Usage (Y)
    0x09, 0x32, //   Usage (Z)
    0x09, 0x35, //   Usage (Rz)
    0x15, 0x81, //   Logical Minimum (-127)
    0x25, 0x7f, //   Logical Maximum (127)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x04, //   Report Count (4)
    0x81, 0x02, //   Input (Data,Var,Abs)
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
    0x29, 0x0f, //   Usage Maximum (15)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x0f, //   Report Count (15)
    0x81, 0x02, //   Input (Data,Var,Abs)
    // Padding
    0x75, 0x01, //   Report Size (1)
    0x95, 0x09, //   Report Count (9)
    0x81, 0x01, //   Input (Cnst,Ary,Abs)
    // Done
    0xC0,       // End Collection
];

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default, PackedStruct)]
#[packed_struct(endian = "lsb", size_bytes = "8")]
pub struct TwinStickReport {
    #[packed_field]
    pub x: i8,
    #[packed_field]
    pub y: i8,
    #[packed_field]
    pub z: i8,
    #[packed_field]
    pub rz: i8,
    #[packed_field]
    pub hat: u8,
    #[packed_field]
    pub buttons1: u8,
    #[packed_field]
    pub buttons2: u8,
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
        let builder = InterfaceBuilder::new(TWIN_STICK_DESCRIPTOR)
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
    bank_b2: &mut [Level; 8],
) -> TwinStickReport {
    let mut x = 0;
    let mut y = 0;
    let mut z = 0;
    let mut rz = 0;
    let mut hat = 255;
    let mut buttons1 = 0;
    let mut buttons2 = 0;

    let modh = bank_b1[5] == Level::High;
    let modx = bank_b1[6] == Level::High;
    let mody = bank_b1[7] == Level::High;

    let left = bank_a1[1] == Level::High;
    let right = bank_a1[3] == Level::High;
    let up = bank_b2[7] == Level::High;
    let down = bank_a1[2] == Level::High;

    let rup = bank_a2[2] == Level::High;
    let rdown = bank_a2[4] == Level::High;
    let rleft = bank_a2[3] == Level::High;
    let rright = bank_a2[1] == Level::High;

    let lb = bank_a1[0] == Level::High;
    let ba = bank_a2[0] == Level::High;

    let ol1 = bank_a1[4] == Level::High;
    let ol2 = bank_a1[5] == Level::High;
    let ol3 = bank_a1[6] == Level::High;

    let or1 = bank_b1[2] == Level::High;
    let or2 = bank_b1[3] == Level::High;
    let or3 = bank_b1[4] == Level::High;

    let bb = bank_b2[0] == Level::High;
    let rb = bank_b2[1] == Level::High;
    let by = bank_b2[2] == Level::High;
    let bx = bank_b2[3] == Level::High;
    let lt = bank_b2[4] == Level::High;
    let rt = bank_b2[5] == Level::High;
    let bs = bank_b2[6] == Level::High;

    // X & Y axis
    // No Modifiers
    if !modh {
        if !(modx || mody) || (modx && mody) {
            if !(left || right) || (left && right) {
                if up {
                    y -= 127;
                }
                if down {
                    y += 127;
                }
            } else {
                if up {
                    y -= 89;
                }
                if down {
                    y += 89;
                }
            }
            if !(up || down) || (up && down) {
                if left {
                    x -= 127;
                }
                if right {
                    x += 127;
                }
            } else {
                if left {
                    x -= 89;
                }
                if right {
                    x += 89;
                }
            }
        }

        // X Modifier
        if modx && !mody {
            if !(left || right) || (left && right) {
                if up {
                    y -= 67;
                }
                if down {
                    y += 67;
                }
            } else {
                if up {
                    y -= 39;
                }
                if down {
                    y += 39;
                }
            }
            if !(up || down) || (up && down) {
                if left {
                    x -= 83;
                }
                if right {
                    x += 83;
                }
            } else {
                if left {
                    x -= 92;
                }
                if right {
                    x += 92;
                }
            }
        }

        // Y Modifier
        if !modx && mody {
            if up {
                y -= 92;
            }
            if down {
                y += 92;
            }
            if !(up || down) || (up && down) {
                if left {
                    x -= 41;
                }
                if right {
                    x += 41;
                }
            } else {
                if left {
                    x -= 39;
                }
                if right {
                    x += 39;
                }
            }
        }
    } else {
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
    }
    // Z & Rz Axis
    if !(rleft || rright) || (rleft && rright) {
        if rup {
            rz -= 127;
        }
        if rdown {
            rz += 127;
        }
    } else {
        if rup {
            rz -= 107;
        }
        if rdown {
            rz += 107;
        }
    }
    if !(rup || rdown) || (rup && rdown) {
        if rleft {
            z -= 127;
        }
        if rright {
            z += 127;
        }
    } else {
        if rleft {
            z -= 52;
        }
        if rright {
            z += 52;
        }
    }

    if ba {
        buttons1 |= 1 << 0;
    }
    if bb {
        buttons1 |= 1 << 1;
    }
    if lt {
        buttons1 |= 1 << 2;
    }
    if bx {
        buttons1 |= 1 << 3;
    }
    if by {
        buttons1 |= 1 << 4;
    }
    if rt {
        buttons1 |= 1 << 5;
    }
    if lb {
        buttons1 |= 1 << 6;
    }
    if rb {
        buttons1 |= 1 << 7;
    }

    if ol2 {
        buttons2 |= 1 << 0;
    }
    if bs {
        buttons2 |= 1 << 1;
    }
    if ol1 {
        buttons2 |= 1 << 2;
    }
    if or1 {
        buttons2 |= 1 << 3;
    }
    if or2 {
        buttons2 |= 1 << 4;
    }
    if ol3 {
        buttons2 |= 1 << 5;
    }
    if or3 {
        buttons2 |= 1 << 6;
    }

    TwinStickReport {
        x,
        y,
        z,
        rz,
        hat,
        buttons1,
        buttons2,
    }
}

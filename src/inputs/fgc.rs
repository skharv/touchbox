use crate::{hal::I2C, pac::I2C0};
use defmt::warn;
use mcp230xx::{Level, Mcp23017, Mcp230xx};
use rp_pico::hal::gpio::bank0::{Gpio17, Gpio28};
use rp_pico::hal::gpio::PullUp;
use rp_pico::hal::gpio::{FunctionI2C, Pin};

#[allow(clippy::type_complexity)]
pub fn read_bank_a1(
    mcp: &mut Mcp230xx<
        I2C<
            I2C0,
            (
                Pin<Gpio28, FunctionI2C, PullUp>,
                Pin<Gpio17, FunctionI2C, PullUp>,
            ),
        >,
        Mcp23017,
    >,
) -> [Level; 8] {
    let mut buttons: [Level; 8] = [Level::Low; 8];
    let Ok(a4) = mcp.gpio(Mcp23017::A4) else {
        warn!("failed to read A4");
        return [Level::Low; 8];
    };

    if a4 == Level::Low {
        if let Ok(a0) = mcp.gpio(Mcp23017::A0) {
            buttons[0] = a0;
        } else {
            buttons[0] = Level::Low;
        }

        if let Ok(a1) = mcp.gpio(Mcp23017::A1) {
            buttons[1] = a1;
        } else {
            buttons[1] = Level::Low;
        }

        if let Ok(a2) = mcp.gpio(Mcp23017::A2) {
            buttons[2] = a2;
        } else {
            buttons[2] = Level::Low;
        }

        if let Ok(a3) = mcp.gpio(Mcp23017::A3) {
            buttons[3] = a3;
        } else {
            buttons[3] = Level::Low;
        }

        buttons[4] = Level::Low;
        buttons[5] = Level::Low;
        buttons[6] = Level::Low;
        buttons[7] = Level::Low;
    } else {
        buttons[0] = Level::Low;
        buttons[1] = Level::Low;
        buttons[2] = Level::Low;
        buttons[3] = Level::Low;

        if let Ok(a0) = mcp.gpio(Mcp23017::A0) {
            buttons[4] = a0;
        } else {
            buttons[4] = Level::Low;
        }

        if let Ok(a1) = mcp.gpio(Mcp23017::A1) {
            buttons[5] = a1;
        } else {
            buttons[5] = Level::Low;
        }

        if let Ok(a2) = mcp.gpio(Mcp23017::A2) {
            buttons[6] = a2;
        } else {
            buttons[6] = Level::Low;
        }

        if let Ok(a3) = mcp.gpio(Mcp23017::A3) {
            buttons[7] = a3;
        } else {
            buttons[7] = Level::Low;
        }
    };

    buttons
}

#[allow(clippy::type_complexity)]
pub fn read_bank_b1(
    mcp: &mut Mcp230xx<
        I2C<
            I2C0,
            (
                Pin<Gpio28, FunctionI2C, PullUp>,
                Pin<Gpio17, FunctionI2C, PullUp>,
            ),
        >,
        Mcp23017,
    >,
) -> [Level; 8] {
    let mut buttons: [Level; 8] = [Level::Low; 8];

    if let Ok(b0) = mcp.gpio(Mcp23017::B0) {
        buttons[0] = b0;
    } else {
        buttons[0] = Level::Low;
    };

    if let Ok(b1) = mcp.gpio(Mcp23017::B1) {
        buttons[1] = b1;
    } else {
        buttons[1] = Level::Low;
    };

    if let Ok(b2) = mcp.gpio(Mcp23017::B2) {
        buttons[2] = b2;
    } else {
        buttons[2] = Level::Low;
    };

    if let Ok(b3) = mcp.gpio(Mcp23017::B3) {
        buttons[3] = b3;
    } else {
        buttons[3] = Level::Low;
    };

    if let Ok(b4) = mcp.gpio(Mcp23017::B4) {
        buttons[4] = b4;
    } else {
        buttons[4] = Level::Low;
    };

    if let Ok(b5) = mcp.gpio(Mcp23017::B5) {
        buttons[5] = b5;
    } else {
        buttons[5] = Level::Low;
    };

    if let Ok(b6) = mcp.gpio(Mcp23017::B6) {
        buttons[6] = b6;
    } else {
        buttons[6] = Level::Low;
    };

    if let Ok(b7) = mcp.gpio(Mcp23017::B7) {
        buttons[7] = b7;
    } else {
        buttons[7] = Level::Low;
    };
    buttons
}

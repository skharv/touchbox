use crate::Debounce;
use crate::{hal::I2C, pac::I2C0};
use mcp230xx::{Level, Mcp23017, Mcp230xx};
use rp_pico::hal::gpio::bank0::{Gpio17, Gpio28};
use rp_pico::hal::gpio::PullUp;
use rp_pico::hal::gpio::{FunctionI2C, Pin};

pub const BANK_A: u8 = 0x12;
pub const BANK_B: u8 = 0x13;

fn bit_to_level(v: u8, bit: usize) -> Level {
    if (v & (1 << bit)) != 0 {
        Level::High
    } else {
        Level::Low
    }
}

#[allow(clippy::type_complexity)]
pub fn read_bank(
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
    bank: u8,
    debounce: &mut [Debounce; 8],
    now: u64,
    debounce_us: u64,
) -> [Level; 8] {
    let mut buttons: [Level; 8] = [Level::High; 8];

    let raw = mcp.read(bank).unwrap();

    for i in 0..8 {
        let level = bit_to_level(raw, i);
        buttons[i] = debounce[i].update(level, now, debounce_us);
    }

    buttons
}

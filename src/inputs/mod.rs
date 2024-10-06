use crate::{hal::I2C, pac::I2C0};
use mcp230xx::{Mcp23017, Mcp230xx, Polarity};
use rp_pico::hal::gpio::bank0::{Gpio17, Gpio28};
use rp_pico::hal::gpio::PullUp;
use rp_pico::hal::gpio::{FunctionI2C, Pin};

pub mod fgc;
pub mod smash;

#[allow(clippy::type_complexity)]
pub fn init_button(
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
    map: Mcp23017,
) {
    mcp.set_direction(map, mcp230xx::Direction::Input).unwrap();
    mcp.set_pull_up(map, mcp230xx::PullUp::Enabled).unwrap();
    mcp.set_input_polarity(map, Polarity::Inverted).unwrap();
}

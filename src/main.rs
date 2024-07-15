#![no_std]
#![no_main]

use cst816s::TouchGesture;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico as bsp;
use rp_pico::hal::gpio::FunctionI2C;
use rp_pico::hal::I2C;
use rp_pico::pac::i2c0;

use bsp::entry;
use fugit::RateExtU32;

use display_interface_spi::SPIInterface;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    pixelcolor::Rgb565,
    primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle},
};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, i2c, pac, pwm,
    sio::Sio,
    spi,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio9.into_push_pull_output();

    // Create an SPI driver instance for the SPI1 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let dc_pin = pins.gpio8.into_push_pull_output();
    let rst_pin = pins.gpio13.into_push_pull_output();

    let spi_interface = SPIInterface::new(spi, dc_pin, spi_cs);

    // initialize PWM for backlight
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM6
    let mut pwm = pwm_slices.pwm6;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM6 to GPIO 13
    let mut channel = pwm.channel_a;
    channel.output_to(pins.gpio12);

    // Create display driver
    let mut display = gc9a01a::GC9A01A::new(spi_interface, rst_pin, channel);
    // Bring out of reset
    display.reset(&mut delay).unwrap();
    // Initialize registers
    display.initialize(&mut delay).unwrap();
    // Fill screen with single color
    display.clear(Rgb565::CSS_IVORY).unwrap();
    // Turn on backlight
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    let yoffset = 100;

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(5)
        .stroke_color(Rgb565::CSS_ORCHID)
        .build();

    let gpio21 = pins.gpio21.into_pull_up_input();
    let gpio22 = pins.gpio22.into_push_pull_output();

    // Setup I2C for touchpad
    let sda_pin = pins.gpio6.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<FunctionI2C>();

    let i2c0_pins = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Touchpad setup
    let mut touchpad = cst816s::CST816S::new(i2c0_pins, gpio21, gpio22);
    touchpad.setup(&mut delay).unwrap();

    // screen outline for the round 1.28 inch Waveshare display
    Circle::new(Point::new(1, 1), 238)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // triangle
    Triangle::new(
        Point::new(50, 32 + yoffset),
        Point::new(50 + 32, 32 + yoffset),
        Point::new(50 + 8, yoffset),
    )
    .into_styled(style)
    .draw(&mut display)
    .unwrap();

    // square
    Rectangle::new(Point::new(110, yoffset), Size::new_equal(32))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // circle
    Circle::new(Point::new(170, yoffset), 32)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    let gpio28 = pins.gpio28.into_pull_down_input();

    let mut refresh_count = 0;

    loop {
        if let Some(evt) = touchpad.read_one_touch_event(true) {
            refresh_count += 1;

            let touch_time = match evt.gesture {
                TouchGesture::LongPress => {
                    refresh_count = 1000;
                    50_000
                }
                TouchGesture::SingleClick => 5_000,
                _ => 0,
            };

            if touch_time > 0 {
                led_pin.set_high().unwrap();
            }

            if refresh_count > 40 {
                refresh_count = 0;
            }
        }

        if gpio28.is_high().unwrap() {
            led_pin.set_low().unwrap();
        }
    }
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

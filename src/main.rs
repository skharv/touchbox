#![no_std]
#![no_main]

use crate::{hal::I2C, pac::I2C0};
use cortex_m::prelude::_embedded_hal_timer_CountDown;
use cst816s::TouchGesture;
#[allow(clippy::wildcard_imports)]
use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::text::Text;
use mcp230xx::{Level, Mcp23017, Mcp230xx};
use panic_halt as _;
use reports::all_button_layout::{AllButtonReport, *};
use rp_pico as bsp;
use rp_pico::hal::gpio::bank0::{Gpio17, Gpio28};
use rp_pico::hal::gpio::{FunctionI2C, FunctionPwm, Pin};
use usb_device::{class_prelude::*, prelude::*};
use usbd_human_interface_device::prelude::*;

use bsp::entry;
use fugit::{ExtU32, RateExtU32};

use display_interface_spi::SPIInterface;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    pixelcolor::Rgb565,
    primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle},
};

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac, pwm,
    sio::Sio,
    spi,
    watchdog::Watchdog,
};

use crate::hal::usb::UsbBus;

mod inputs;
mod reports;

use inputs::fgc;

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

    let timer = bsp::hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // USB
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut joy = UsbHidClassBuilder::new()
        .add_device(AllButtonConfig::default())
        .build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default()
            .manufacturer("skharv")
            .product("touchbox")
            .serial_number("TEST")])
        .unwrap()
        .build();

    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let spi_mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let spi_miso = pins.gpio12.into_function::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio9.into_push_pull_output();

    let valid_pinout = (spi_mosi, spi_miso, spi_sclk);

    // Create an SPI driver instance for the SPI1 device
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI1, valid_pinout);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let dc_pin = pins.gpio8.into_push_pull_output();
    let rst_pin = pins.gpio13.into_push_pull_output();

    let spi_interface = SPIInterface::new(spi, dc_pin, spi_cs);

    // initialize PWM for backlight
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM6
    let mut pwm = pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to GPIO 25
    let mut channel = pwm.channel_b;
    let led_pin = pins.led.into_function::<FunctionPwm>();

    channel.output_to(led_pin);

    // Create display driver
    let mut display = gc9a01a::GC9A01A::new(spi_interface, rst_pin, channel);
    display.reset(&mut delay).unwrap();
    display.initialize(&mut delay).unwrap();
    display.clear(Rgb565::CSS_IVORY).unwrap();
    display.set_backlight(55000);

    // Draw Stuff
    let yoffset = 100;
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);
    let style = PrimitiveStyleBuilder::new()
        .stroke_width(5)
        .stroke_color(Rgb565::CSS_ORCHID)
        .build();

    Circle::new(Point::new(1, 1), 238)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    Triangle::new(
        Point::new(50, 32 + yoffset),
        Point::new(50 + 32, 32 + yoffset),
        Point::new(50 + 16, yoffset),
    )
    .into_styled(style)
    .draw(&mut display)
    .unwrap();

    Rectangle::new(Point::new(110, yoffset), Size::new_equal(32))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    Text::new("qwer", Point::new(30, 80), text_style)
        .draw(&mut display)
        .unwrap();

    // Touchpad setup
    let sda_pin = pins.gpio6.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_function::<FunctionI2C>();

    let i2c1_pins = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin.reconfigure(),
        scl_pin.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let gpio21 = pins.gpio21.into_pull_up_input();
    let gpio22 = pins.gpio22.into_push_pull_output();

    let mut touchpad = cst816s::CST816S::new(i2c1_pins, gpio21, gpio22);
    touchpad.setup(&mut delay).unwrap();

    // Setup MCP23017
    let sda_pin0 = pins.gpio28.into_function::<FunctionI2C>();
    let scl_pin0 = pins.gpio17.into_function::<FunctionI2C>();

    let i2c0_pins = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin0.reconfigure(),
        scl_pin0.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let mut mcp = Mcp230xx::<
        I2C<
            I2C0,
            (
                Pin<Gpio28, FunctionI2C, gpio::PullUp>,
                Pin<Gpio17, FunctionI2C, gpio::PullUp>,
            ),
        >,
        Mcp23017,
    >::new(i2c0_pins, 0x27)
    .unwrap();

    // Initialize all the buttons
    inputs::init_button(&mut mcp, Mcp23017::A0);
    inputs::init_button(&mut mcp, Mcp23017::A1);
    inputs::init_button(&mut mcp, Mcp23017::A2);
    inputs::init_button(&mut mcp, Mcp23017::A3);
    inputs::init_button(&mut mcp, Mcp23017::A4);

    // Input Polling rate
    let mut input_count_down = timer.count_down();
    input_count_down.start(1.millis());

    // Begin Loop
    loop {
        if input_count_down.wait().is_ok() {
            // Touchpad Gestures
            if let Some(evt) = touchpad.read_one_touch_event(true) {
                match evt.gesture {
                    TouchGesture::LongPress => {
                        display.set_backlight(55000);
                    }
                    TouchGesture::SingleClick => {
                        display.set_backlight(1000);
                    }
                    _ => (),
                };
            }

            let mut bank_a1 = fgc::read_bank_a1(&mut mcp);
            let mut bank_b1 = fgc::read_bank_b1(&mut mcp);
            let mut bank_a2 = [Level::Low; 8];
            let mut bank_b2 = [Level::Low; 8];

            match joy.device().write_report(&get_report(
                &mut bank_a1,
                &mut bank_b1,
                &mut bank_a2,
                &mut bank_b2,
            )) {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write joystick report: {:?}", e)
                }
            }
        }

        if usb_dev.poll(&mut [&mut joy]) {};
    }
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

fn get_report(
    bank_a1: &mut [Level; 8],
    bank_b1: &mut [Level; 8],
    bank_a2: &mut [Level; 8],
    bank_b2: &mut [Level; 8],
) -> AllButtonReport {
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

    let mut b2 = 0;
    for (idx, pressed) in bank_b2[..8].iter_mut().enumerate() {
        if *pressed == Level::High {
            b2 |= 1 << idx;
        }
    }
    AllButtonReport { a1, b1, a2, b2 }
}

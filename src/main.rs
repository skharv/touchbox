#![no_std]
#![no_main]

use core::cell::RefCell;
use core::{u8, usize};

use crate::{hal::I2C, pac::I2C0};
use cortex_m::{delay::Delay, prelude::_embedded_hal_timer_CountDown};
use cst816s::TouchGesture;
use display_interface_spi::SPIInterface;
use embedded_graphics::image::{Image, ImageRaw, ImageRawBE, ImageRawLE};
#[allow(clippy::wildcard_imports)]
use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use gc9a01a_driver::{FrameBuffer, Orientation};
use mcp230xx::{Level, Mcp23017, Mcp230xx};
use panic_halt as _;
use rp_pico as bsp;
use rp_pico::hal::gpio::bank0::{Gpio17, Gpio28};
use rp_pico::hal::gpio::{FunctionI2C, FunctionPwm, Pin};
use tinybmp::{Bmp, RawBmp};
use tinytga::Tga;
use usb_device::{class_prelude::*, prelude::*};
use usbd_human_interface_device::prelude::*;

use bsp::entry;
use fugit::{ExtU32, RateExtU32};

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

use inputs::allbtn;

use reports::{all_button_layout, all_button_layout::get_all_button_report};
use reports::{classic_layout, classic_layout::get_classic_report};
use reports::{twin_stick_layout, twin_stick_layout::get_twin_stick_report};

const SCREEN_WIDTH: u32 = 240;
const SCREEN_HEIGHT: u32 = 240;
const BUFFER_SIZE: usize = (SCREEN_WIDTH * SCREEN_HEIGHT * 2) as usize;

struct DelayWrapper<'a> {
    delay: &'a mut Delay,
}

impl<'a> DelayWrapper<'a> {
    pub fn new(delay: &'a mut Delay) -> Self {
        DelayWrapper { delay }
    }
}

impl<'a> DelayNs for DelayWrapper<'a> {
    fn delay_ns(&mut self, ns: u32) {
        let us = (ns + 999) / 1000;
        self.delay.delay_us(us);
    }
}

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

    let i2c_ref_cell = RefCell::new(i2c0_pins);

    let mut mcp1 = Mcp230xx::<
        I2C<
            I2C0,
            (
                Pin<Gpio28, FunctionI2C, gpio::PullUp>,
                Pin<Gpio17, FunctionI2C, gpio::PullUp>,
            ),
        >,
        Mcp23017,
    >::new(&i2c_ref_cell, 0x27)
    .unwrap();

    let mut mcp2 = Mcp230xx::<
        I2C<
            I2C0,
            (
                Pin<Gpio28, FunctionI2C, gpio::PullUp>,
                Pin<Gpio17, FunctionI2C, gpio::PullUp>,
            ),
        >,
        Mcp23017,
    >::new(&i2c_ref_cell, 0x26)
    .unwrap();

    // Initialize all the buttons
    inputs::init_button(&mut mcp1, Mcp23017::A0);
    inputs::init_button(&mut mcp1, Mcp23017::A1);
    inputs::init_button(&mut mcp1, Mcp23017::A2);
    inputs::init_button(&mut mcp1, Mcp23017::A3);
    inputs::init_button(&mut mcp1, Mcp23017::A4);
    inputs::init_button(&mut mcp1, Mcp23017::A5);
    inputs::init_button(&mut mcp1, Mcp23017::A6);
    inputs::init_button(&mut mcp1, Mcp23017::A7);

    inputs::init_button(&mut mcp1, Mcp23017::B0);
    inputs::init_button(&mut mcp1, Mcp23017::B1);
    inputs::init_button(&mut mcp1, Mcp23017::B2);
    inputs::init_button(&mut mcp1, Mcp23017::B3);
    inputs::init_button(&mut mcp1, Mcp23017::B4);
    inputs::init_button(&mut mcp1, Mcp23017::B5);
    inputs::init_button(&mut mcp1, Mcp23017::B6);
    inputs::init_button(&mut mcp1, Mcp23017::B7);

    inputs::init_button(&mut mcp2, Mcp23017::A0);
    inputs::init_button(&mut mcp2, Mcp23017::A1);
    inputs::init_button(&mut mcp2, Mcp23017::A2);
    inputs::init_button(&mut mcp2, Mcp23017::A3);
    inputs::init_button(&mut mcp2, Mcp23017::A4);
    inputs::init_button(&mut mcp2, Mcp23017::A5);
    inputs::init_button(&mut mcp2, Mcp23017::A6);
    inputs::init_button(&mut mcp2, Mcp23017::A7);

    inputs::init_button(&mut mcp2, Mcp23017::B0);
    inputs::init_button(&mut mcp2, Mcp23017::B1);
    inputs::init_button(&mut mcp2, Mcp23017::B2);
    inputs::init_button(&mut mcp2, Mcp23017::B3);
    inputs::init_button(&mut mcp2, Mcp23017::B4);
    inputs::init_button(&mut mcp2, Mcp23017::B5);
    inputs::init_button(&mut mcp2, Mcp23017::B6);
    inputs::init_button(&mut mcp2, Mcp23017::B7);

    // Create display driver
    let mut clear_colour = Rgb565::CSS_BLACK;

    let mut delay_wrapper = DelayWrapper::new(&mut delay);

    let mut display = gc9a01a_driver::GC9A01A::new(
        spi,
        dc_pin,
        spi_cs,
        rst_pin,
        false,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
    );
    //display.hard_reset(&mut delay_wrapper).unwrap();
    display.init(&mut delay_wrapper).unwrap();
    display
        .set_orientation(&Orientation::PortraitSwapped)
        .unwrap();
    display.clear_screen(clear_colour.into_storage()).unwrap();

    channel.set_duty_cycle_percent(50).unwrap();

    //let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
    //let mut framebuffer = FrameBuffer::new(&mut buffer, SCREEN_WIDTH, SCREEN_HEIGHT);

    //let spi_interface = SPIInterface::new(spi, dc_pin, spi_cs);

    //let mut display = gc9a01a::GC9A01A::new(spi_interface, rst_pin, channel);
    //display.reset(&mut delay).unwrap();
    //display.initialize(&mut delay).unwrap();
    //display.clear(clear_colour).unwrap();
    //display.set_backlight(10000);

    // Draw Stuff
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::CSS_LIGHT_GOLDENROD_YELLOW);
    let style = PrimitiveStyleBuilder::new()
        .stroke_width(5)
        .stroke_color(Rgb565::CSS_ORCHID)
        .build();

    // Input Polling rate
    let mut input_count_down = timer.count_down();
    input_count_down.start(1.millis());
    let mut cleared = false;

    loop {
        if !cleared {
            let bmp_data = include_bytes!("../assets/menu.raw");
            display.draw_image(bmp_data).unwrap();
            cleared = true;
        }

        if mcp1.gpio(Mcp23017::A5).unwrap() == Level::High {
            //display.clear_screen(clear_colour.into_storage()).unwrap();

            let mut all_button_joy = UsbHidClassBuilder::new()
                .add_device(all_button_layout::AllButtonConfig::default())
                .build(&usb_bus);

            let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
                .strings(&[StringDescriptors::default()
                    .manufacturer("skharv")
                    .product("touchbox(buttonbox)")
                    .serial_number("00001")])
                .unwrap()
                .build();

            let bmp_data = include_bytes!("../assets/ab.raw");
            display.draw_image(bmp_data).unwrap();

            // Begin Loop
            loop {
                // Touchpad Gestures
                if let Some(evt) = touchpad.read_one_touch_event(true) {
                    match evt.gesture {
                        TouchGesture::LongPress => {
                            //display.set_backlight(55000);
                        }
                        TouchGesture::SingleClick => {
                            usb_dev.force_reset().unwrap();
                            break;
                        }
                        _ => (),
                    };
                }

                if input_count_down.wait().is_ok() {
                    let mut bank_a1 = allbtn::read_bank_a(&mut mcp1);
                    let mut bank_b1 = allbtn::read_bank_b(&mut mcp1);
                    let mut bank_a2 = allbtn::read_bank_a(&mut mcp2);
                    let mut bank_b2 = allbtn::read_bank_b(&mut mcp2);

                    match all_button_joy.device().write_report(&get_all_button_report(
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

                if usb_dev.poll(&mut [&mut all_button_joy]) {}
            }
        }
        if mcp1.gpio(Mcp23017::A4).unwrap() == Level::High {
            let bmp_data = include_bytes!("../assets/ssbm.raw");
            display.draw_image(bmp_data).unwrap();

            let mut twin_stick_joy = UsbHidClassBuilder::new()
                .add_device(twin_stick_layout::TwinStickConfig::default())
                .build(&usb_bus);

            let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
                .strings(&[StringDescriptors::default()
                    .manufacturer("skharv")
                    .product("touchbox(ssbmbox)")
                    .serial_number("00001")])
                .unwrap()
                .build();

            // Begin Loop
            loop {
                // Touchpad Gestures
                if let Some(evt) = touchpad.read_one_touch_event(true) {
                    match evt.gesture {
                        TouchGesture::LongPress => {
                            //display.set_backlight(55000);
                        }
                        TouchGesture::SingleClick => {
                            usb_dev.force_reset().unwrap();
                            break;
                        }

                        _ => (),
                    };
                }

                if input_count_down.wait().is_ok() {
                    let mut bank_a1 = allbtn::read_bank_a(&mut mcp1);
                    let mut bank_b1 = allbtn::read_bank_b(&mut mcp1);
                    let mut bank_a2 = allbtn::read_bank_a(&mut mcp2);
                    let mut bank_b2 = allbtn::read_bank_b(&mut mcp2);

                    match twin_stick_joy.device().write_report(&get_twin_stick_report(
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

                if usb_dev.poll(&mut [&mut twin_stick_joy]) {}
            }
        }
        if mcp1.gpio(Mcp23017::B2).unwrap() == Level::High {
            let bmp_data = include_bytes!("../assets/sf.raw");
            display.draw_image(bmp_data).unwrap();

            let mut classic_joy = UsbHidClassBuilder::new()
                .add_device(classic_layout::ClassicConfig::default())
                .build(&usb_bus);

            let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
                .strings(&[StringDescriptors::default()
                    .manufacturer("skharv")
                    .product("touchbox(fightbox)")
                    .serial_number("00001")])
                .unwrap()
                .build();

            // Begin Loop
            loop {
                // Touchpad Gestures
                if let Some(evt) = touchpad.read_one_touch_event(true) {
                    match evt.gesture {
                        TouchGesture::LongPress => {
                            //display.set_backlight(55000);
                        }
                        TouchGesture::SingleClick => {
                            break;
                        }

                        _ => (),
                    };
                }

                if input_count_down.wait().is_ok() {
                    let mut bank_a1 = allbtn::read_bank_a(&mut mcp1);
                    let mut bank_b1 = allbtn::read_bank_b(&mut mcp1);
                    let mut bank_a2 = allbtn::read_bank_a(&mut mcp2);
                    let mut bank_b2 = allbtn::read_bank_b(&mut mcp2);

                    match classic_joy.device().write_report(&get_classic_report(
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

                if usb_dev.poll(&mut [&mut classic_joy]) {}
            }
        }
    }
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

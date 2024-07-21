#![no_std]
#![no_main]

use cst816s::TouchGesture;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::_embedded_hal_timer_CountDown;
use panic_halt as _;
use rp_pico as bsp;
use rp_pico::hal::gpio::{
    DynPinId, FunctionI2C, FunctionPwm, FunctionSioInput, Pin, PullDown, PullUp,
};
use rp_pico::hal::I2C;
#[allow(clippy::wildcard_imports)]
use usb_device::{class_prelude::*, prelude::*};
use usbd_human_interface_device::device::joystick::JoystickReport;
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
use usbd_human_interface_device::usb_class::UsbHidBuilderError;

use crate::hal::usb::UsbBus;

pub mod reports;

use reports::*;

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
        &embedded_hal::spi::MODE_0,
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
    // Bring out of reset
    display.reset(&mut delay).unwrap();
    // Initialize registers
    display.initialize(&mut delay).unwrap();
    // Fill screen with single color
    display.clear(Rgb565::CSS_IVORY).unwrap();
    display.set_backlight(55000);
    // Turn on backlight
    //let mut led_pin = pins.led.into_push_pull_output();
    //led_pin.set_high().unwrap();

    let yoffset = 100;

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(5)
        .stroke_color(Rgb565::CSS_ORCHID)
        .build();

    let gpio21 = pins.gpio21.into_pull_up_input();
    let gpio22 = pins.gpio22.into_push_pull_output();

    // Setup I2C for touchpad
    let sda_pin = pins.gpio6.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_function::<FunctionI2C>();

    let i2c0_pins = I2C::i2c1(
        pac.I2C1,
        sda_pin.reconfigure(),
        scl_pin.reconfigure(),
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

    let mut input_pins: [Pin<DynPinId, FunctionSioInput, PullUp>; 3] = [
        pins.gpio26.into_pull_up_input().into_dyn_pin(),
        pins.gpio27.into_pull_up_input().into_dyn_pin(),
        pins.gpio28.into_pull_up_input().into_dyn_pin(),
    ];

    let mut input_count_down = timer.count_down();
    input_count_down.start(1.millis());

    loop {
        if input_count_down.wait().is_ok() {
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

            match joy.device().write_report(&get_report(&mut input_pins)) {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write joystick report: {:?}", e)
                }
            }
        }

        if usb_dev.poll(&mut [&mut joy]) {}
    }
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

fn get_report(pins: &mut [Pin<DynPinId, FunctionSioInput, PullUp>; 3]) -> AllButtonReport {
    // Read out 8 buttons first
    let mut buttons = 0;
    for (idx, pin) in pins[..3].iter_mut().enumerate() {
        if pin.is_low().unwrap() {
            buttons |= 1 << idx;
        }
    }

    AllButtonReport { buttons }
}

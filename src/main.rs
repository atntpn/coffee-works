#![no_std]
#![no_main]

use esp_idf_hal::gpio;
use esp_idf_hal::prelude::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_println::println;
use sh1106::{mode::BufferedGraphicsMode, builder::Builder, prelude::*};
use embedded_graphics::{mono_font::{ascii::FONT_6X10, MonoTextStyle}, pixelcolor::BinaryColor, text::Text};
use rotary_encoder_hal::Rotary;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();

    let mut i2c = peripherals.i2c0
        .with_pins(
            peripherals.pins.gpio8,  // SDA
            peripherals.pins.gpio9,  // SCL
        )
        .with_config(<esp_idf_hal::i2c::Config as Default>::default())
        .build();

    let mut display = Builder::new()
        .with_size(sh1106::Size::Display128x64)
        .connect_i2c(i2c)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear();

    let mut pump_pin = peripherals.pins.gpio16.into_output().unwrap();
    let mut pump_on = false;

    let clk = peripherals.pins.gpio1.into_input().unwrap();
    let dt = peripherals.pins.gpio2.into_input().unwrap();
    let sw = peripherals.pins.gpio3.into_input().unwrap();

    let mut encoder = Rotary::new(clk, dt);

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    loop {
        if sw.is_low().unwrap() {
            pump_on = !pump_on;
            if pump_on {
                pump_pin.set_high().unwrap();
            } else {
                pump_pin.set_low().unwrap();
            }
            std::thread::sleep(std::time::Duration::from_millis(200));
        }

        encoder.update().unwrap();
        if encoder.direction() == rotary_encoder_hal::Direction::Clockwise {
            println!("Rotated CW");
        } else if encoder.direction() == rotary_encoder_hal::Direction::CounterClockwise {
            println!("Rotated CCW");
        }

        display.clear();
        Text::new(
            if pump_on { "Pompe: ON" } else { "Pompe: OFF" },
            embedded_graphics::Point::new(10, 20),
            text_style,
        )
        .into_styled(text_style)
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();

        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}

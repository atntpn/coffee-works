use core::fmt::Write;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::i2c::I2c;
use esp_hal::delay::Delay;
use esp_println::println;
use heapless::String;
use mini_oled::{
    error::MiniOledError,
    prelude::{I2cInterface, Sh1106},
};

const DISPLAY_ADDRESS: u8 = 0x3C;
const INIT_ATTEMPTS: u8 = 10;

pub struct DisplayManager<I2C: I2c> {
    display: Sh1106<I2cInterface<I2C>>,
}

impl<I2C: I2c> DisplayManager<I2C> {
    pub fn new(i2c: I2C, delay: &Delay) -> Result<Self, MiniOledError> {
        delay.delay_millis(500);
        let mut display = Sh1106::new(I2cInterface::new(i2c, DISPLAY_ADDRESS));
        for attempt in 1..=INIT_ATTEMPTS {
            match display.init() {
                Ok(()) => {
                    println!("OLED initialise a la tentative {attempt}");
                    return Ok(Self { display });
                }
                Err(error) if attempt == INIT_ATTEMPTS => return Err(error),
                Err(error) => {
                    println!("Tentative OLED {attempt}/{INIT_ATTEMPTS}: {:?}", error);
                    delay.delay_millis(200);
                }
            }
        }
        unreachable!()
    }

    pub fn show_telemetry(
        &mut self,
        steam: Option<f32>,
        extraction: Option<f32>,
        temperature: Option<f32>,
    ) -> Result<(), MiniOledError> {
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
        let canvas = self.display.get_mut_canvas();
        canvas.clear(BinaryColor::Off).ok();
        let mut steam_text: String<22> = String::new();
        let mut extraction_text: String<22> = String::new();
        let mut temperature_text: String<22> = String::new();
        match steam {
            Some(v) => write!(&mut steam_text, "VAPEUR  {:.2} bar", v).ok(),
            None => write!(&mut steam_text, "VAPEUR  --.-- bar").ok(),
        };
        match extraction {
            Some(v) => write!(&mut extraction_text, "EXTRACT {:.2} bar", v).ok(),
            None => write!(&mut extraction_text, "EXTRACT --.-- bar").ok(),
        };
        match temperature {
            Some(v) => write!(&mut temperature_text, "GROUPE  {:.2} C", v).ok(),
            None => write!(&mut temperature_text, "GROUPE  --.-- C").ok(),
        };
        for (text, y) in [
            ("LA MILLARZOCCO", 1),
            (steam_text.as_str(), 18),
            (extraction_text.as_str(), 32),
            (temperature_text.as_str(), 46),
        ] {
            Text::with_baseline(
                text,
                Point::new(if y == 1 { 20 } else { 3 }, y),
                style,
                Baseline::Top,
            )
            .draw(canvas)
            .ok();
        }
        self.display.flush()
    }
}

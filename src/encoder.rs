use esp_hal::{
    gpio::{Input, Pull},
    peripheral::Peripheral,
    time,
};

const BUTTON_DEBOUNCE_US: u64 = 5_000;
const ROTATION_DEBOUNCE_US: u64 = 1_500;

#[derive(Clone, Copy, Debug)]
pub enum EncoderEvent {
    Clockwise,
    CounterClockwise,
    Button,
}

pub struct EncoderManager<'d> {
    clk: Input<'d>,
    dt: Input<'d>,
    switch: Input<'d>,
    last_clk_high: bool,
    last_rotation_at: u64,
    last_raw_button: bool,
    stable_button: bool,
    button_changed_at: u64,
}

impl<'d> EncoderManager<'d> {
    pub fn new(
        clk_pin: impl Peripheral<P = impl esp_hal::gpio::InputPin> + 'd,
        dt_pin: impl Peripheral<P = impl esp_hal::gpio::InputPin> + 'd,
        switch_pin: impl Peripheral<P = impl esp_hal::gpio::InputPin> + 'd,
    ) -> Self {
        let clk = Input::new(clk_pin, Pull::Up);
        let dt = Input::new(dt_pin, Pull::Up);
        let switch = Input::new(switch_pin, Pull::Up);
        let last_clk_high = clk.is_high();
        let raw_button = switch.is_low();
        let now = time::now().ticks();

        Self {
            clk,
            dt,
            switch,
            last_clk_high,
            last_rotation_at: now.wrapping_sub(ROTATION_DEBOUNCE_US),
            last_raw_button: raw_button,
            stable_button: raw_button,
            button_changed_at: now,
        }
    }

    pub fn poll(&mut self) -> Option<EncoderEvent> {
        let now = time::now().ticks();
        let raw_button = self.switch.is_low();
        if raw_button != self.last_raw_button {
            self.last_raw_button = raw_button;
            self.button_changed_at = now;
        } else if raw_button != self.stable_button
            && now.wrapping_sub(self.button_changed_at) >= BUTTON_DEBOUNCE_US
        {
            let was_pressed = self.stable_button;
            self.stable_button = raw_button;
            if self.stable_button && !was_pressed {
                self.last_clk_high = self.clk.is_high();
                return Some(EncoderEvent::Button);
            }
        }

        let clk_high = self.clk.is_high();
        if self.last_clk_high
            && !clk_high
            && now.wrapping_sub(self.last_rotation_at) >= ROTATION_DEBOUNCE_US
        {
            self.last_rotation_at = now;
            self.last_clk_high = clk_high;
            return Some(if self.dt.is_high() {
                EncoderEvent::Clockwise
            } else {
                EncoderEvent::CounterClockwise
            });
        }
        self.last_clk_high = clk_high;

        None
    }
}

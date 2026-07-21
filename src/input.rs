use esp_hal::{
    gpio::{Input, Pull},
    peripheral::Peripheral,
    time,
};

const DEBOUNCE_US: u64 = 50_000;

pub struct InputManager<'d> {
    button: Input<'d>,
    last_raw_pressed: bool,
    stable_pressed: bool,
    raw_changed_at: u64,
}

impl<'d> InputManager<'d> {
    pub fn new(pin: impl Peripheral<P = impl esp_hal::gpio::InputPin> + 'd) -> Self {
        Self {
            button: Input::new(pin, Pull::Up),
            last_raw_pressed: false,
            stable_pressed: false,
            raw_changed_at: time::now().ticks(),
        }
    }

    pub fn poll_toggle(&mut self) -> bool {
        let raw_pressed = self.button.is_low();
        let now = time::now().ticks();

        if raw_pressed != self.last_raw_pressed {
            self.last_raw_pressed = raw_pressed;
            self.raw_changed_at = now;
            return false;
        }

        if raw_pressed != self.stable_pressed
            && now.wrapping_sub(self.raw_changed_at) >= DEBOUNCE_US
        {
            self.stable_pressed = raw_pressed;
            return raw_pressed;
        }

        false
    }
}

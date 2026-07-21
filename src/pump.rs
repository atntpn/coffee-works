use esp_hal::{
    gpio::{Level, Output},
    peripheral::Peripheral,
};

pub struct PumpController<'d> {
    command: Output<'d>,
    is_on: bool,
}

impl<'d> PumpController<'d> {
    pub fn new(pin: impl Peripheral<P = impl esp_hal::gpio::OutputPin> + 'd) -> Self {
        // Le module de puissance est actif à l'état bas : HIGH = pompe arrêtée.
        Self {
            command: Output::new(pin, Level::High),
            is_on: false,
        }
    }

    pub fn set_on(&mut self, on: bool) {
        if self.is_on != on {
            self.is_on = on;
            self.apply_state();
        }
    }

    pub fn is_on(&self) -> bool {
        self.is_on
    }

    fn apply_state(&mut self) {
        if self.is_on {
            self.command.set_low();
        } else {
            self.command.set_high();
        }
    }
}

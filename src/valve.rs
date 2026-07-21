use esp_hal::{
    gpio::{Level, Output},
    peripheral::Peripheral,
};

/// Commande logique d'un optocoupleur d'électrovanne.
///
/// L'optocoupleur est actif à l'état bas : LOW = ouverte, HIGH = fermée.
pub struct ValveController<'d> {
    output: Output<'d>,
    open: bool,
}

impl<'d> ValveController<'d> {
    pub fn new(pin: impl Peripheral<P = impl esp_hal::gpio::OutputPin> + 'd) -> Self {
        Self {
            // La sortie est placée immédiatement dans l'état de sécurité.
            output: Output::new(pin, Level::High),
            open: false,
        }
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    pub fn set_open(&mut self, open: bool) {
        self.open = open;
        if open {
            self.output.set_low();
        } else {
            self.output.set_high();
        }
    }

    pub fn force_closed(&mut self) {
        self.set_open(false);
    }
}

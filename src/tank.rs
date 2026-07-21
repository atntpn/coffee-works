use esp_hal::{
    gpio::{Input, Pull},
    peripheral::Peripheral,
    time,
};

// L'absence d'eau coupe la pompe immédiatement. Le retour de l'eau doit être
// stable avant d'autoriser un nouveau démarrage.
const WATER_PRESENT_CONFIRM_US: u64 = 500_000;

pub struct TankWaterSensor<'d> {
    signal: Input<'d>,
    last_raw_has_water: bool,
    stable_has_water: bool,
    raw_changed_at: u64,
}

impl<'d> TankWaterSensor<'d> {
    pub fn new(pin: impl Peripheral<P = impl esp_hal::gpio::InputPin> + 'd) -> Self {
        let signal = Input::new(pin, Pull::Up);
        let raw_has_water = signal.is_low();

        Self {
            signal,
            last_raw_has_water: raw_has_water,
            // Etat sûr au démarrage : pompe interdite jusqu'à confirmation.
            stable_has_water: false,
            raw_changed_at: time::now().ticks(),
        }
    }

    pub fn has_water(&self) -> bool {
        self.stable_has_water
    }

    pub fn poll(&mut self) -> Option<bool> {
        let raw_has_water = self.signal.is_low();
        let now = time::now().ticks();

        if raw_has_water != self.last_raw_has_water {
            self.last_raw_has_water = raw_has_water;
            self.raw_changed_at = now;
        }

        // Avec le pull-up, un fil de signal débranché signifie aussi bac vide.
        if !raw_has_water && self.stable_has_water {
            self.stable_has_water = false;
            return Some(false);
        }

        if raw_has_water
            && !self.stable_has_water
            && now.wrapping_sub(self.raw_changed_at) >= WATER_PRESENT_CONFIRM_US
        {
            self.stable_has_water = true;
            return Some(true);
        }

        None
    }
}

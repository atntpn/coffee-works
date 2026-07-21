use esp_hal::{
    gpio::{Input, Pull},
    peripheral::Peripheral,
    time,
};

// On coupe rapidement la pompe quand l'eau atteint la sonde.
const FULL_CONFIRM_US: u64 = 200_000;
// On attend davantage avant de remplir afin d'ignorer les vagues et parasites.
const NOT_FULL_CONFIRM_US: u64 = 2_000_000;

pub struct WaterLevelSensor<'d> {
    relay_contact: Input<'d>,
    last_raw_full: bool,
    stable_full: bool,
    confirmed: bool,
    raw_changed_at: u64,
}

impl<'d> WaterLevelSensor<'d> {
    pub fn new(pin: impl Peripheral<P = impl esp_hal::gpio::InputPin> + 'd) -> Self {
        let relay_contact = Input::new(pin, Pull::Up);
        let raw_full = relay_contact.is_low();

        Self {
            relay_contact,
            last_raw_full: raw_full,
            // Etat sûr au démarrage : la pompe reste arrêtée jusqu'à ce que
            // "PAS PLEIN" ait été confirmé pendant deux secondes.
            stable_full: true,
            confirmed: false,
            raw_changed_at: time::now().ticks(),
        }
    }

    pub fn is_full(&self) -> bool {
        self.stable_full
    }

    pub fn poll(&mut self) -> Option<bool> {
        let raw_full = self.relay_contact.is_low();
        let now = time::now().ticks();

        if raw_full != self.last_raw_full {
            self.last_raw_full = raw_full;
            self.raw_changed_at = now;
            return None;
        }

        let confirmation_us = if raw_full {
            FULL_CONFIRM_US
        } else {
            NOT_FULL_CONFIRM_US
        };

        if !self.confirmed && now.wrapping_sub(self.raw_changed_at) >= confirmation_us {
            self.confirmed = true;
            self.stable_full = raw_full;
            return Some(self.stable_full);
        }

        if raw_full != self.stable_full && now.wrapping_sub(self.raw_changed_at) >= confirmation_us
        {
            self.stable_full = raw_full;
            return Some(self.stable_full);
        }

        None
    }
}

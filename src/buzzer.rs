use esp_hal::{
    ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::TimerIFace,
        LowSpeed,
    },
    time,
};

const TANK_ALARM_INTERVAL_US: u64 = 4_000_000;

#[derive(Clone, Copy)]
enum Sequence {
    Marseillaise,
    Click,
    Rotate,
    TankEmpty,
}

pub struct BuzzerController<'d> {
    channel: Channel<'d, LowSpeed>,
    timers: [&'d dyn TimerIFace<LowSpeed>; 4],
    startup_enabled: bool,
    click_enabled: bool,
    rotation_enabled: bool,
    tank_alarm_enabled: bool,
    sequence: Option<Sequence>,
    step: usize,
    step_started_at: u64,
    last_tank_alarm_at: u64,
}

impl<'d> BuzzerController<'d> {
    pub fn new(
        mut channel: Channel<'d, LowSpeed>,
        timers: [&'d dyn TimerIFace<LowSpeed>; 4],
    ) -> Result<Self, channel::Error> {
        channel.configure(channel::config::Config {
            timer: timers[0],
            // Le module est déclenché au niveau bas : 100 % maintient I/O à HIGH.
            duty_pct: 100,
            pin_config: channel::config::PinConfig::PushPull,
        })?;
        let now = time::now().ticks();
        Ok(Self {
            channel,
            timers,
            startup_enabled: true,
            click_enabled: true,
            rotation_enabled: true,
            tank_alarm_enabled: true,
            sequence: None,
            step: 0,
            step_started_at: now,
            last_tank_alarm_at: now.wrapping_sub(TANK_ALARM_INTERVAL_US),
        })
    }

    pub fn startup_enabled(&self) -> bool {
        self.startup_enabled
    }

    pub fn click_enabled(&self) -> bool {
        self.click_enabled
    }

    pub fn rotation_enabled(&self) -> bool {
        self.rotation_enabled
    }

    pub fn tank_alarm_enabled(&self) -> bool {
        self.tank_alarm_enabled
    }

    pub fn toggle_startup(&mut self) {
        self.startup_enabled = !self.startup_enabled;
    }

    pub fn toggle_click(&mut self) {
        self.click_enabled = !self.click_enabled;
    }

    pub fn toggle_rotation(&mut self) {
        self.rotation_enabled = !self.rotation_enabled;
    }

    pub fn toggle_tank_alarm(&mut self) {
        self.tank_alarm_enabled = !self.tank_alarm_enabled;
    }

    pub fn set_sound_settings(
        &mut self,
        startup_enabled: bool,
        click_enabled: bool,
        rotation_enabled: bool,
        tank_alarm_enabled: bool,
    ) {
        self.startup_enabled = startup_enabled;
        self.click_enabled = click_enabled;
        self.rotation_enabled = rotation_enabled;
        self.tank_alarm_enabled = tank_alarm_enabled;
    }

    pub fn play_marseillaise(&mut self) {
        if self.startup_enabled {
            self.start(Sequence::Marseillaise);
        }
    }

    pub fn play_click(&mut self) {
        if self.click_enabled {
            self.start(Sequence::Click);
        }
    }

    pub fn play_rotate(&mut self) {
        if self.rotation_enabled {
            self.start(Sequence::Rotate);
        }
    }

    pub fn tick(&mut self, system_enabled: bool, tank_has_water: bool) {
        if !system_enabled {
            self.sequence = None;
            self.apply_note(None);
            return;
        }

        let now = time::now().ticks();
        if let Some(sequence) = self.sequence {
            let (_, duration_us) = sequence_step(sequence, self.step);
            if now.wrapping_sub(self.step_started_at) >= duration_us {
                self.step += 1;
                if let Some((note, _)) = valid_sequence_step(sequence, self.step) {
                    self.step_started_at = now;
                    self.apply_note(note);
                } else {
                    self.sequence = None;
                    self.apply_note(None);
                }
            }
            return;
        }

        if self.tank_alarm_enabled
            && !tank_has_water
            && now.wrapping_sub(self.last_tank_alarm_at) >= TANK_ALARM_INTERVAL_US
        {
            self.last_tank_alarm_at = now;
            self.start(Sequence::TankEmpty);
        }
    }

    fn start(&mut self, sequence: Sequence) {
        self.sequence = Some(sequence);
        self.step = 0;
        self.step_started_at = time::now().ticks();
        self.apply_note(sequence_step(sequence, 0).0);
    }

    fn apply_note(&mut self, note: Option<(usize, u8)>) {
        if let Some((index, duty_pct)) = note {
            let _ = self.channel.configure(channel::config::Config {
                timer: self.timers[index],
                duty_pct,
                pin_config: channel::config::PinConfig::PushPull,
            });
        } else {
            // Niveau HIGH permanent : silence pour le module actif LOW.
            let _ = self.channel.set_duty(100);
        }
    }
}

fn valid_sequence_step(sequence: Sequence, step: usize) -> Option<(Option<(usize, u8)>, u64)> {
    let count = match sequence {
        Sequence::Marseillaise => 22,
        Sequence::Click => 2,
        Sequence::Rotate => 2,
        Sequence::TankEmpty => 6,
    };
    (step < count).then(|| sequence_step(sequence, step))
}

fn sequence_step(sequence: Sequence, step: usize) -> (Option<(usize, u8)>, u64) {
    match sequence {
        // Ouverture de La Marseillaise, adaptée aux quatre fréquences
        // disponibles du buzzer : C, D, E et G.
        Sequence::Marseillaise => match step {
            0 => (Some((3, 50)), 90_000),
            1 => (Some((2, 50)), 170_000),
            2 => (Some((3, 50)), 90_000),
            3 => (Some((0, 50)), 210_000),
            4 => (None, 30_000),
            5 => (Some((0, 50)), 210_000),
            6 => (Some((1, 50)), 210_000),
            7 => (None, 30_000),
            8 => (Some((1, 50)), 210_000),
            9 => (Some((3, 50)), 310_000),
            10 => (Some((2, 50)), 90_000),
            11 => (Some((2, 50)), 170_000),
            12 => (None, 45_000),
            13 => (Some((0, 50)), 90_000),
            14 => (Some((2, 50)), 170_000),
            15 => (Some((0, 50)), 90_000),
            16 => (Some((3, 50)), 210_000),
            17 => (Some((2, 50)), 360_000),
            18 => (Some((1, 50)), 170_000),
            19 => (Some((3, 50)), 90_000),
            20 => (Some((0, 50)), 480_000),
            _ => (None, 100_000),
        },
        Sequence::Click => match step {
            0 => (Some((0, 8)), 26_000),
            _ => (None, 22_000),
        },
        Sequence::Rotate => match step {
            0 => (Some((0, 5)), 13_000),
            _ => (None, 10_000),
        },
        Sequence::TankEmpty => match step {
            0 | 2 | 4 => (Some((0, 50)), 400_000),
            _ => (None, 180_000),
        },
    }
}

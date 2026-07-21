use esp_hal::{
    gpio::{Level, Output},
    peripheral::Peripheral,
    time,
};

pub const TARGET_MIN_C: f32 = 80.0;
pub const TARGET_MAX_C: f32 = 105.0;
pub const TARGET_STEP_C: f32 = 1.0;

const HARD_MAX_TEMPERATURE_C: f32 = 115.0;
const CONTROL_WINDOW_US: u64 = 1_000_000;
const CONTROL_TIMEOUT_US: u64 = 1_500_000;

// Valeurs initiales pour le groupe. Elles devront être affinées sur la machine réelle.
const KP: f32 = 0.07;
const KI: f32 = 0.002;
const KD: f32 = 0.02;

pub struct GroupHeaterController<'d> {
    command: Output<'d>,
    target_c: f32,
    duty: f32,
    integral: f32,
    last_error: f32,
    last_pid_at: Option<u64>,
    last_control_at: u64,
    window_started_at: u64,
    safety_ok: bool,
    is_on: bool,
}

impl<'d> GroupHeaterController<'d> {
    pub fn new(pin: impl Peripheral<P = impl esp_hal::gpio::OutputPin> + 'd) -> Self {
        let now = time::now().ticks();
        Self {
            command: Output::new(pin, Level::Low),
            target_c: 93.0,
            duty: 0.0,
            integral: 0.0,
            last_error: 0.0,
            last_pid_at: None,
            last_control_at: now,
            window_started_at: now,
            safety_ok: false,
            is_on: false,
        }
    }

    pub fn target_c(&self) -> f32 {
        self.target_c
    }

    pub fn adjust_target(&mut self, delta_c: f32) {
        self.set_target(self.target_c + delta_c);
    }

    pub fn set_target(&mut self, target_c: f32) {
        self.target_c = target_c.clamp(TARGET_MIN_C, TARGET_MAX_C);
        self.last_pid_at = None;
        self.last_error = 0.0;
    }

    pub fn update_control(&mut self, system_enabled: bool, temperature_c: Option<f32>) {
        let now = time::now().ticks();
        self.last_control_at = now;

        let Some(temperature) = temperature_c else {
            self.force_off();
            return;
        };

        self.safety_ok = system_enabled && (-20.0..HARD_MAX_TEMPERATURE_C).contains(&temperature);
        if !self.safety_ok {
            self.force_off();
            return;
        }

        let error = self.target_c - temperature;
        let (dt, derivative) = match self.last_pid_at {
            Some(previous) => {
                let dt = (now.wrapping_sub(previous) as f32 / 1_000_000.0).clamp(0.05, 1.0);
                (dt, (error - self.last_error) / dt)
            }
            None => (0.0, 0.0),
        };

        if dt > 0.0 {
            let candidate_integral = (self.integral + error * dt).clamp(-200.0, 200.0);
            let candidate_output = KP * error + KI * candidate_integral + KD * derivative;
            if (0.0..=1.0).contains(&candidate_output)
                || (candidate_output > 1.0 && error < 0.0)
                || (candidate_output < 0.0 && error > 0.0)
            {
                self.integral = candidate_integral;
            }
        }

        self.duty = (KP * error + KI * self.integral + KD * derivative).clamp(0.0, 1.0);
        self.last_error = error;
        self.last_pid_at = Some(now);
    }

    pub fn tick(&mut self) {
        let now = time::now().ticks();
        if !self.safety_ok || now.wrapping_sub(self.last_control_at) > CONTROL_TIMEOUT_US {
            self.force_off();
            return;
        }

        if now.wrapping_sub(self.window_started_at) >= CONTROL_WINDOW_US {
            self.window_started_at = now;
        }

        let on_time_us = (self.duty * CONTROL_WINDOW_US as f32) as u64;
        self.set_output(now.wrapping_sub(self.window_started_at) < on_time_us);
    }

    pub fn force_off(&mut self) {
        self.safety_ok = false;
        self.duty = 0.0;
        self.integral = 0.0;
        self.last_pid_at = None;
        self.set_output(false);
    }

    pub fn duty_percent(&self) -> u8 {
        (self.duty * 100.0 + 0.5) as u8
    }

    fn set_output(&mut self, on: bool) {
        if self.is_on == on {
            return;
        }
        self.is_on = on;
        if on {
            self.command.set_high();
        } else {
            self.command.set_low();
        }
    }
}

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

// Régulation adaptée à la forte inertie de la masse en laiton. La température
// est projetée à court terme à partir de sa vitesse de montée afin de couper
// la chauffe avant que la sonde atteigne physiquement la consigne.
const KP: f32 = 0.08;
const KI: f32 = 0.002;
const KD: f32 = 0.01;
const PREDICTION_HORIZON_S: f32 = 20.0;
const RATE_FILTER_ALPHA: f32 = 0.25;
const INTEGRAL_ZONE_C: f32 = 4.0;
const INTEGRAL_LIMIT: f32 = 30.0;
const COAST_CUTOFF_MARGIN_C: f32 = 0.2;

pub struct GroupHeaterController<'d> {
    command: Output<'d>,
    target_c: f32,
    duty: f32,
    integral: f32,
    last_error: f32,
    last_temperature_c: Option<f32>,
    filtered_rise_c_per_s: f32,
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
            last_temperature_c: None,
            filtered_rise_c_per_s: 0.0,
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
        self.integral = 0.0;
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

        let (dt, raw_rise_rate) = match (self.last_pid_at, self.last_temperature_c) {
            (Some(previous_at), Some(previous_temperature)) => {
                let dt = (now.wrapping_sub(previous_at) as f32 / 1_000_000.0).clamp(0.05, 1.5);
                (dt, (temperature - previous_temperature) / dt)
            }
            _ => (0.0, 0.0),
        };
        if dt > 0.0 {
            self.filtered_rise_c_per_s +=
                RATE_FILTER_ALPHA * (raw_rise_rate - self.filtered_rise_c_per_s);
        }

        // Une baisse de température ne doit pas provoquer une anticipation
        // inverse. Seule l'énergie déjà engagée pendant la montée est projetée.
        let predicted_temperature =
            temperature + self.filtered_rise_c_per_s.max(0.0) * PREDICTION_HORIZON_S;
        let error = self.target_c - predicted_temperature;
        let derivative = match self.last_pid_at {
            Some(previous) => {
                let derivative_dt =
                    (now.wrapping_sub(previous) as f32 / 1_000_000.0).clamp(0.05, 1.5);
                (error - self.last_error) / derivative_dt
            }
            None => 0.0,
        };

        if predicted_temperature >= self.target_c - COAST_CUTOFF_MARGIN_C {
            self.duty = 0.0;
            self.integral = 0.0;
        } else if dt > 0.0 && error.abs() <= INTEGRAL_ZONE_C {
            let candidate_integral =
                (self.integral + error * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            let candidate_output = KP * error + KI * candidate_integral + KD * derivative;
            if (0.0..=1.0).contains(&candidate_output)
                || (candidate_output > 1.0 && error < 0.0)
                || (candidate_output < 0.0 && error > 0.0)
            {
                self.integral = candidate_integral;
            }
            self.duty = limited_warmup_duty(
                (KP * error + KI * self.integral + KD * derivative).clamp(0.0, 1.0),
                self.target_c - temperature,
            );
        } else {
            // Hors de la zone intégrale, aucune énergie résiduelle ne doit
            // s'accumuler pendant la longue phase de montée en température.
            self.integral = 0.0;
            self.duty = limited_warmup_duty(
                (KP * error + KD * derivative).clamp(0.0, 1.0),
                self.target_c - temperature,
            );
        }

        self.last_error = error;
        self.last_temperature_c = Some(temperature);
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
        self.last_temperature_c = None;
        self.filtered_rise_c_per_s = 0.0;
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

fn limited_warmup_duty(requested: f32, actual_error_c: f32) -> f32 {
    let maximum = if actual_error_c <= 3.0 {
        0.12
    } else if actual_error_c <= 6.0 {
        0.25
    } else if actual_error_c <= 10.0 {
        0.45
    } else {
        1.0
    };
    requested.min(maximum)
}

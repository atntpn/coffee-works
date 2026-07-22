use esp_hal::time;

pub const EXTRACTION_TARGET_MIN_BAR: f32 = 1.0;
pub const EXTRACTION_TARGET_MAX_BAR: f32 = 4.0;
pub const EXTRACTION_TARGET_STEP_BAR: f32 = 0.1;
const EXTRACTION_DEFAULT_TARGET_BAR: f32 = 4.0;
const EXTRACTION_RESTART_HYSTERESIS_BAR: f32 = 0.05;
// Le PID peut se stabiliser quelques centièmes sous la consigne. Cette marge
// permet de considérer la chaudière prête sans attendre un franchissement
// exact rendu peu probable par le bruit de mesure.
const STEAM_READY_TOLERANCE_BAR: f32 = 0.05;
const STEAM_FILL_TIMEOUT_US: u64 = 180_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutomationPhase {
    Standby,
    TankEmpty,
    SteamFill,
    SteamHeating,
    ExtractionPressure,
    Ready,
    SensorFault,
    FillTimeout,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct AutomationOutputs {
    pub pump_on: bool,
    pub steam_valve_open: bool,
    pub extraction_valve_open: bool,
}

impl AutomationOutputs {
    fn all_closed() -> Self {
        Self {
            pump_on: false,
            steam_valve_open: false,
            extraction_valve_open: false,
        }
    }
}

pub struct MachineAutomation {
    phase: AutomationPhase,
    steam_ready: bool,
    extraction_pressurized: bool,
    extraction_target_bar: f32,
    phase_started_at: u64,
    steam_fill_fault: bool,
}

impl MachineAutomation {
    pub fn new() -> Self {
        let now = time::now().ticks();
        Self {
            phase: AutomationPhase::Standby,
            steam_ready: false,
            extraction_pressurized: false,
            extraction_target_bar: EXTRACTION_DEFAULT_TARGET_BAR,
            phase_started_at: now,
            steam_fill_fault: false,
        }
    }

    pub fn phase(&self) -> AutomationPhase {
        self.phase
    }

    pub fn extraction_target_bar(&self) -> f32 {
        self.extraction_target_bar
    }

    pub fn extraction_ready(&self) -> bool {
        self.extraction_pressurized
    }

    pub fn adjust_extraction_target(&mut self, delta_bar: f32) {
        self.set_extraction_target(self.extraction_target_bar + delta_bar);
    }

    pub fn set_extraction_target(&mut self, target_bar: f32) {
        self.extraction_target_bar =
            target_bar.clamp(EXTRACTION_TARGET_MIN_BAR, EXTRACTION_TARGET_MAX_BAR);
        // La prochaine mise à jour revalide immédiatement la pression par
        // rapport à la nouvelle consigne.
        self.extraction_pressurized = false;
    }

    pub fn update(
        &mut self,
        system_enabled: bool,
        tank_has_water: bool,
        steam_boiler_full: bool,
        steam_pressure_bar: Option<f32>,
        steam_target_bar: f32,
        extraction_pressure_bar: Option<f32>,
    ) -> AutomationOutputs {
        if !system_enabled {
            self.set_phase(AutomationPhase::Standby);
            self.steam_ready = false;
            self.extraction_pressurized = false;
            self.steam_fill_fault = false;
            return AutomationOutputs::all_closed();
        }

        if !tank_has_water {
            self.set_phase(AutomationPhase::TankEmpty);
            return AutomationOutputs::all_closed();
        }

        // L'état prêt suit la pression réelle, même lorsqu'un remplissage
        // vapeur suspend momentanément la pressurisation extraction.
        if let Some(extraction_pressure) = extraction_pressure_bar {
            if !self.extraction_pressurized {
                if extraction_pressure >= self.extraction_target_bar {
                    self.extraction_pressurized = true;
                }
            } else if extraction_pressure
                <= self.extraction_target_bar - EXTRACTION_RESTART_HYSTERESIS_BAR
            {
                self.extraction_pressurized = false;
            }
        }

        // Le remplissage vapeur est toujours prioritaire.
        if !steam_boiler_full {
            if self.steam_fill_fault {
                self.set_phase(AutomationPhase::FillTimeout);
                return AutomationOutputs::all_closed();
            }
            self.set_phase(AutomationPhase::SteamFill);
            // Une fois la pression vapeur validée au premier démarrage, cette
            // autorisation reste acquise jusqu'à la mise en veille. Un appoint
            // d'eau suspend seulement l'extraction pendant le remplissage.
            if self.phase_elapsed_us() >= STEAM_FILL_TIMEOUT_US {
                self.steam_fill_fault = true;
                self.set_phase(AutomationPhase::FillTimeout);
                return AutomationOutputs::all_closed();
            }
            return AutomationOutputs {
                pump_on: true,
                steam_valve_open: true,
                extraction_valve_open: false,
            };
        }

        let Some(steam_pressure) = steam_pressure_bar else {
            self.set_phase(AutomationPhase::SensorFault);
            return AutomationOutputs::all_closed();
        };

        // Au démarrage, la chaudière extraction reste isolée jusqu'à ce que
        // la chaudière vapeur soit arrivée à sa consigne.
        if !self.steam_ready {
            if steam_pressure < steam_target_bar - STEAM_READY_TOLERANCE_BAR {
                self.set_phase(AutomationPhase::SteamHeating);
                return AutomationOutputs::all_closed();
            }
            self.steam_ready = true;
        }

        if extraction_pressure_bar.is_none() {
            self.set_phase(AutomationPhase::SensorFault);
            return AutomationOutputs::all_closed();
        }

        if self.extraction_pressurized {
            self.set_phase(AutomationPhase::Ready);
            AutomationOutputs::all_closed()
        } else {
            self.set_phase(AutomationPhase::ExtractionPressure);
            AutomationOutputs {
                pump_on: true,
                steam_valve_open: false,
                extraction_valve_open: true,
            }
        }
    }

    fn set_phase(&mut self, phase: AutomationPhase) {
        if self.phase != phase {
            self.phase = phase;
            self.phase_started_at = time::now().ticks();
        }
    }

    fn phase_elapsed_us(&self) -> u64 {
        time::now().ticks().wrapping_sub(self.phase_started_at)
    }
}

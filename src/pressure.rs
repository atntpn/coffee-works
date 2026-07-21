use embedded_hal::i2c::I2c;
use esp_hal::time;

const ADS1115_ADDRESS: u8 = 0x48;
const CONFIG_REGISTER: u8 = 0x01;
const CONVERSION_REGISTER: u8 = 0x00;

// Mesures simples A0/A1, plage +/-6,144 V, 128 echantillons/s, comparateur coupe.
const SINGLE_A0_CONFIG: u16 = 0xC183;
const SINGLE_A1_CONFIG: u16 = 0xD183;
const CONVERSION_TIME_US: u64 = 10_000;
const SAMPLE_INTERVAL_US: u64 = 250_000;

const ADC_LSB_VOLTS: f32 = 0.0001875;
const SENSOR_SUPPLY_VOLTS: f32 = 5.12;
// Zéros relevés à pression atmosphérique sur la machine assemblée.
const STEAM_SENSOR_ZERO_VOLTS: f32 = 0.5131;
// Zéro relevé sur A1 à pression atmosphérique. Seul l'offset est corrigé :
// la sensibilité volts/bar reste strictement identique à celle de A0.
const EXTRACTION_SENSOR_ZERO_VOLTS: f32 = 0.4971;
const SENSOR_VOLTAGE_MAX: f32 = SENSOR_SUPPLY_VOLTS * 0.9;
// Gain nominal commun des capteurs 0,5-4,5 V / 0-16 bar.
const VOLTS_PER_BAR: f32 = 0.256;
// Une tension légèrement inférieure au zéro nominal représente une pression
// négative issue du décalage du capteur. Une sortie proche de 0 V reste en
// revanche considérée comme un fil débranché ou un capteur défectueux.
const SENSOR_VOLTAGE_LOW_LIMIT: f32 = 0.25;
const SENSOR_VOLTAGE_HIGH_MARGIN: f32 = 0.08;
const MIN_PRESSURE_BAR: f32 = -1.0;
const MAX_PRESSURE_BAR: f32 = 16.0;
const ZERO_DEADBAND_BAR: f32 = 0.05;

#[derive(Debug)]
pub enum PressureError<E> {
    I2c(E),
    InvalidConversionState,
}

pub struct PressureSample {
    pub raw: i16,
    pub voltage: f32,
    pub bars: Option<f32>,
    pub config: u16,
}

pub struct PressureReadings {
    pub steam: PressureSample,
    pub extraction: PressureSample,
}

#[derive(Clone, Copy)]
enum ConversionState {
    Waiting,
    Steam,
    Extraction,
}

pub struct PressureSensor<I2C> {
    i2c: I2C,
    state: ConversionState,
    pending_steam: Option<PressureSample>,
    action_at: u64,
}

impl<I2C: I2c> PressureSensor<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            state: ConversionState::Waiting,
            pending_steam: None,
            action_at: time::now().ticks(),
        }
    }

    pub fn poll(&mut self) -> Result<Option<PressureReadings>, PressureError<I2C::Error>> {
        let now = time::now().ticks();

        match self.state {
            ConversionState::Waiting => {
                if now.wrapping_sub(self.action_at) < SAMPLE_INTERVAL_US {
                    return Ok(None);
                }

                self.start_conversion(SINGLE_A0_CONFIG)?;
                self.state = ConversionState::Steam;
                self.action_at = now;
                Ok(None)
            }
            ConversionState::Steam => {
                if now.wrapping_sub(self.action_at) < CONVERSION_TIME_US {
                    return Ok(None);
                }

                self.pending_steam = Some(self.read_sample(STEAM_SENSOR_ZERO_VOLTS)?);
                self.start_conversion(SINGLE_A1_CONFIG)?;
                self.state = ConversionState::Extraction;
                self.action_at = now;
                Ok(None)
            }
            ConversionState::Extraction => {
                if now.wrapping_sub(self.action_at) < CONVERSION_TIME_US {
                    return Ok(None);
                }

                let extraction = self.read_sample(EXTRACTION_SENSOR_ZERO_VOLTS)?;
                let steam = self
                    .pending_steam
                    .take()
                    .ok_or(PressureError::InvalidConversionState)?;
                self.state = ConversionState::Waiting;
                self.action_at = now;
                Ok(Some(PressureReadings { steam, extraction }))
            }
        }
    }

    fn start_conversion(&mut self, config: u16) -> Result<(), PressureError<I2C::Error>> {
        let bytes = config.to_be_bytes();
        self.i2c
            .write(ADS1115_ADDRESS, &[CONFIG_REGISTER, bytes[0], bytes[1]])
            .map_err(PressureError::I2c)
    }

    fn read_sample(
        &mut self,
        sensor_zero_volts: f32,
    ) -> Result<PressureSample, PressureError<I2C::Error>> {
        let mut bytes = [0_u8; 2];
        self.i2c
            .write_read(ADS1115_ADDRESS, &[CONVERSION_REGISTER], &mut bytes)
            .map_err(PressureError::I2c)?;
        let mut config_bytes = [0_u8; 2];
        self.i2c
            .write_read(ADS1115_ADDRESS, &[CONFIG_REGISTER], &mut config_bytes)
            .map_err(PressureError::I2c)?;

        let raw = i16::from_be_bytes(bytes);
        let config = u16::from_be_bytes(config_bytes);
        let voltage = raw as f32 * ADC_LSB_VOLTS;
        let allowed_max = SENSOR_VOLTAGE_MAX + SENSOR_VOLTAGE_HIGH_MARGIN;
        let bars = if voltage < SENSOR_VOLTAGE_LOW_LIMIT || voltage > allowed_max {
            None
        } else {
            let bars = (voltage - sensor_zero_volts) / VOLTS_PER_BAR;
            let calibrated = if bars.abs() <= ZERO_DEADBAND_BAR {
                0.0
            } else {
                bars
            };
            Some(calibrated.clamp(MIN_PRESSURE_BAR, MAX_PRESSURE_BAR))
        };

        Ok(PressureSample {
            raw,
            voltage,
            bars,
            config,
        })
    }
}

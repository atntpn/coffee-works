use ads1x1x::{
    channel, ic, mode, Ads1x1x, DataRate16Bit, Error as AdsError, FullScaleRange, TargetAddr,
};
use embedded_hal::i2c::I2c;
use esp_hal::time;

// Mesures simples A0/A1, plage +/-6,144 V, 128 echantillons/s, comparateur coupe.
const SINGLE_A0_CONFIG: u16 = 0xC183;
const SINGLE_A1_CONFIG: u16 = 0xD183;
const CONVERSION_TIME_US: u64 = 10_000;
const SAMPLE_INTERVAL_US: u64 = 250_000;

const ADC_LSB_VOLTS: f32 = 0.0001875;
const SENSOR_SUPPLY_VOLTS: f32 = 5.12;
// Référence zéro commune aux deux capteurs, relevée sur A0 à pression
// atmosphérique. A0 et A1 utilisent strictement la même calibration.
const SENSOR_ZERO_VOLTS: f32 = 0.5131;
const SENSOR_VOLTAGE_MAX: f32 = SENSOR_SUPPLY_VOLTS * 0.9;
// Gain nominal commun des capteurs 0,5-4,5 V / 0-16 bar.
const VOLTS_PER_BAR: f32 = 0.256;
// Une tension légèrement inférieure au zéro nominal représente une pression
// négative issue du décalage du capteur. Une sortie proche de 0 V reste en
// revanche considérée comme un fil débranché ou un capteur défectueux.
const SENSOR_VOLTAGE_LOW_LIMIT: f32 = 0.25;
const SENSOR_VOLTAGE_HIGH_MARGIN: f32 = 0.08;
// Les deux transducteurs mesurent une pression manométrique 0-16 bar. Un
// léger décalage électrique sous le zéro ne doit donc jamais être présenté
// comme une pression négative.
const MIN_PRESSURE_BAR: f32 = 0.0;
const MAX_PRESSURE_BAR: f32 = 16.0;
const ZERO_DEADBAND_BAR: f32 = 0.05;

#[derive(Debug)]
pub enum PressureError<E> {
    I2c(E),
    InvalidDriverConfiguration,
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
    adc: Ads1x1x<I2C, ic::Ads1115, ic::Resolution16Bit, mode::OneShot>,
    state: ConversionState,
    pending_steam: Option<PressureSample>,
    action_at: u64,
}

impl<I2C: I2c> PressureSensor<I2C> {
    pub fn new(i2c: I2C) -> Result<Self, PressureError<I2C::Error>> {
        let mut adc = Ads1x1x::new_ads1115(i2c, TargetAddr::Gnd);
        adc.set_full_scale_range(FullScaleRange::Within6_144V)
            .map_err(map_driver_error)?;
        adc.set_data_rate(DataRate16Bit::Sps128)
            .map_err(map_driver_error)?;

        Ok(Self {
            adc,
            state: ConversionState::Waiting,
            pending_steam: None,
            action_at: time::now().ticks(),
        })
    }

    pub fn poll(&mut self) -> Result<Option<PressureReadings>, PressureError<I2C::Error>> {
        let now = time::now().ticks();

        match self.state {
            ConversionState::Waiting => {
                if now.wrapping_sub(self.action_at) < SAMPLE_INTERVAL_US {
                    return Ok(None);
                }

                match self.adc.read(channel::SingleA0) {
                    Err(nb::Error::WouldBlock) => {
                        self.state = ConversionState::Steam;
                        self.action_at = now;
                        Ok(None)
                    }
                    Err(nb::Error::Other(error)) => Err(map_driver_error(error)),
                    Ok(raw) => {
                        self.pending_steam =
                            Some(convert_sample(raw, SENSOR_ZERO_VOLTS, SINGLE_A0_CONFIG));
                        self.start_extraction_conversion(now)
                    }
                }
            }
            ConversionState::Steam => {
                if now.wrapping_sub(self.action_at) < CONVERSION_TIME_US {
                    return Ok(None);
                }

                match self.adc.read(channel::SingleA0) {
                    Ok(raw) => {
                        self.pending_steam =
                            Some(convert_sample(raw, SENSOR_ZERO_VOLTS, SINGLE_A0_CONFIG));
                        self.start_extraction_conversion(now)
                    }
                    Err(nb::Error::WouldBlock) => Ok(None),
                    Err(nb::Error::Other(error)) => Err(map_driver_error(error)),
                }
            }
            ConversionState::Extraction => {
                if now.wrapping_sub(self.action_at) < CONVERSION_TIME_US {
                    return Ok(None);
                }

                match self.adc.read(channel::SingleA1) {
                    Ok(raw) => {
                        let extraction = convert_sample(raw, SENSOR_ZERO_VOLTS, SINGLE_A1_CONFIG);
                        let steam = self
                            .pending_steam
                            .take()
                            .ok_or(PressureError::InvalidConversionState)?;
                        self.state = ConversionState::Waiting;
                        self.action_at = now;
                        Ok(Some(PressureReadings { steam, extraction }))
                    }
                    Err(nb::Error::WouldBlock) => Ok(None),
                    Err(nb::Error::Other(error)) => Err(map_driver_error(error)),
                }
            }
        }
    }

    fn start_extraction_conversion(
        &mut self,
        now: u64,
    ) -> Result<Option<PressureReadings>, PressureError<I2C::Error>> {
        match self.adc.read(channel::SingleA1) {
            Err(nb::Error::WouldBlock) => {
                self.state = ConversionState::Extraction;
                self.action_at = now;
                Ok(None)
            }
            Err(nb::Error::Other(error)) => Err(map_driver_error(error)),
            Ok(_) => Err(PressureError::InvalidConversionState),
        }
    }
}

fn map_driver_error<E>(error: AdsError<E>) -> PressureError<E> {
    match error {
        AdsError::I2C(error) => PressureError::I2c(error),
        AdsError::InvalidInputData => PressureError::InvalidDriverConfiguration,
    }
}

fn convert_sample(raw: i16, sensor_zero_volts: f32, config: u16) -> PressureSample {
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

    PressureSample {
        raw,
        voltage,
        bars,
        config,
    }
}

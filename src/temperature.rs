use embedded_hal::spi::SpiBus;
use esp_hal::{
    gpio::{Level, Output},
    peripheral::Peripheral,
    time,
};

const CONFIG_REGISTER: u8 = 0x00;
const RTD_MSB_REGISTER: u8 = 0x01;
const FAULT_STATUS_REGISTER: u8 = 0x07;
const CONFIG_3WIRE_AUTO: u8 = 0xD2;

const REFERENCE_RESISTOR_OHMS: f32 = 430.0;
const RTD_NOMINAL_OHMS: f32 = 100.0;
// Coefficients IEC 60751 de l'équation Callendar-Van Dusen pour une PT100.
const PT100_A: f32 = 3.9083e-3;
const PT100_B: f32 = -5.775e-7;
const PT100_C: f32 = -4.183e-12;

// Calibration à un point : le montage affichait 35,4 °C quand la température
// de référence était 31,7 °C. L'erreur est convertie en résistance parasite
// et retirée avant la conversion, au lieu d'appliquer un ratio de température.
const CALIBRATION_RESISTANCE_OFFSET_OHMS: f32 = 1.299_668;
const MIN_VALID_TEMP_C: f32 = -30.0;
const MAX_VALID_TEMP_C: f32 = 200.0;
const SAMPLE_INTERVAL_US: u64 = 500_000;

#[derive(Debug)]
pub enum TemperatureError<E> {
    Spi(E),
    Configuration(u8),
    Fault(u8),
    MissingProbe { config: u8, rtd_value: u16 },
    InvalidTemperature(f32),
}

pub struct TemperatureSensor<'d, SPI> {
    spi: SPI,
    cs: Output<'d>,
    initialized: bool,
    last_action_at: u64,
}

impl<'d, SPI: SpiBus<u8>> TemperatureSensor<'d, SPI> {
    pub fn new(spi: SPI, cs_pin: impl Peripheral<P = impl esp_hal::gpio::OutputPin> + 'd) -> Self {
        Self {
            spi,
            cs: Output::new(cs_pin, Level::High),
            initialized: false,
            last_action_at: time::now().ticks().wrapping_sub(SAMPLE_INTERVAL_US),
        }
    }

    pub fn poll(&mut self) -> Result<Option<f32>, TemperatureError<SPI::Error>> {
        let now = time::now().ticks();
        if now.wrapping_sub(self.last_action_at) < SAMPLE_INTERVAL_US {
            return Ok(None);
        }
        self.last_action_at = now;

        if !self.initialized {
            self.write_register(CONFIG_REGISTER, CONFIG_3WIRE_AUTO)?;
            let config = self.read_register(CONFIG_REGISTER)?;
            // Le bit d'effacement des défauts (0x02) revient tout seul à zéro.
            if config != (CONFIG_3WIRE_AUTO & !0x02) {
                return Err(TemperatureError::Configuration(config));
            }
            self.initialized = true;
            return Ok(None);
        }

        let mut data = [RTD_MSB_REGISTER, 0x00, 0x00];
        self.transfer(&mut data)?;

        let rtd_value = u16::from_be_bytes([data[1], data[2]]);
        if (rtd_value & 0x0001) != 0 {
            let fault_status = self.read_register(FAULT_STATUS_REGISTER)?;
            self.write_register(CONFIG_REGISTER, CONFIG_3WIRE_AUTO)?;
            return Err(TemperatureError::Fault(fault_status));
        }

        let rtd_raw = rtd_value >> 1;
        if rtd_raw == 0 {
            let config = self.read_register(CONFIG_REGISTER)?;
            return Err(TemperatureError::MissingProbe { config, rtd_value });
        }

        let measured_resistance = rtd_raw as f32 * REFERENCE_RESISTOR_OHMS / 32768.0;
        let calibrated_resistance = measured_resistance - CALIBRATION_RESISTANCE_OFFSET_OHMS;
        let temperature_c = pt100_temperature_from_resistance(calibrated_resistance);

        if !temperature_c.is_finite()
            || !(MIN_VALID_TEMP_C..=MAX_VALID_TEMP_C).contains(&temperature_c)
        {
            return Err(TemperatureError::InvalidTemperature(temperature_c));
        }

        Ok(Some(temperature_c))
    }

    fn write_register(
        &mut self,
        register: u8,
        value: u8,
    ) -> Result<(), TemperatureError<SPI::Error>> {
        self.cs.set_low();
        let result = self
            .spi
            .write(&[register | 0x80, value])
            .and_then(|_| self.spi.flush())
            .map_err(TemperatureError::Spi);
        core::hint::spin_loop();
        self.cs.set_high();
        result
    }

    fn read_register(&mut self, register: u8) -> Result<u8, TemperatureError<SPI::Error>> {
        let mut data = [register & 0x7F, 0x00];
        self.transfer(&mut data)?;
        Ok(data[1])
    }

    fn transfer(&mut self, data: &mut [u8]) -> Result<(), TemperatureError<SPI::Error>> {
        self.cs.set_low();
        let result = self
            .spi
            .transfer_in_place(data)
            .and_then(|_| self.spi.flush())
            .map_err(TemperatureError::Spi);
        core::hint::spin_loop();
        self.cs.set_high();
        result
    }
}

fn pt100_temperature_from_resistance(resistance_ohms: f32) -> f32 {
    // Approximation initiale, puis résolution de Callendar-Van Dusen par Newton.
    // Cette méthode fonctionne aussi sous 0 °C sans nécessiter de racine carrée.
    let mut temperature = (resistance_ohms / RTD_NOMINAL_OHMS - 1.0) / PT100_A;

    for _ in 0..5 {
        let (modeled_resistance, derivative) = if temperature >= 0.0 {
            (
                RTD_NOMINAL_OHMS
                    * (1.0 + PT100_A * temperature + PT100_B * temperature * temperature),
                RTD_NOMINAL_OHMS * (PT100_A + 2.0 * PT100_B * temperature),
            )
        } else {
            let t2 = temperature * temperature;
            let t3 = t2 * temperature;
            (
                RTD_NOMINAL_OHMS
                    * (1.0
                        + PT100_A * temperature
                        + PT100_B * t2
                        + PT100_C * (temperature - 100.0) * t3),
                RTD_NOMINAL_OHMS
                    * (PT100_A + 2.0 * PT100_B * temperature + PT100_C * (4.0 * t3 - 300.0 * t2)),
            )
        };

        temperature -= (modeled_resistance - resistance_ohms) / derivative;
    }

    temperature
}

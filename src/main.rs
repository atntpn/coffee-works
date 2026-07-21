#![no_std]
#![no_main]

mod automation;
mod buzzer;
mod display;
mod encoder;
mod group_heater;
mod heater;
mod pressure;
mod pump;
mod settings;
mod tank;
mod temperature;
mod tft;
mod valve;
mod water;

use automation::{AutomationOutputs, MachineAutomation, EXTRACTION_TARGET_STEP_BAR};
use buzzer::BuzzerController;
use display::DisplayManager;
use encoder::{EncoderEvent, EncoderManager};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    i2c::master::{BusTimeout, Config as I2cConfig, I2c},
    ledc::{
        channel,
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::{self, RateExtU32},
};
use esp_println::println;
use group_heater::{GroupHeaterController, TARGET_STEP_C};
use heater::{HeaterController, TARGET_STEP_BAR};
use pressure::{PressureError, PressureSample, PressureSensor};
use pump::PumpController;
use settings::{MachineSettings, SettingsStore};
use tank::TankWaterSensor;
use temperature::{TemperatureError, TemperatureSensor};
use tft::{DisplayPage, MenuItem, SoundSetting, TftManager, UiState};
use valve::ValveController;
use water::WaterLevelSensor;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();

    println!("WeAct Studio ESP32-S3 demarree !");

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default()
            .with_frequency(400.kHz())
            .with_timeout(BusTimeout::Maximum),
    )
    .unwrap()
    .with_sda(peripherals.GPIO8)
    .with_scl(peripherals.GPIO9);

    let pressure_i2c = I2c::new(
        peripherals.I2C1,
        I2cConfig::default()
            .with_frequency(100.kHz())
            .with_timeout(BusTimeout::Maximum),
    )
    .unwrap()
    .with_sda(peripherals.GPIO11)
    .with_scl(peripherals.GPIO12);

    let temperature_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(100.kHz())
            .with_mode(Mode::_1),
    )
    .unwrap()
    .with_sck(peripherals.GPIO15)
    .with_mosi(peripherals.GPIO14)
    .with_miso(peripherals.GPIO13);

    let tft_spi = Spi::new(
        peripherals.SPI3,
        SpiConfig::default()
            .with_frequency(40.MHz())
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO36)
    .with_mosi(peripherals.GPIO35);

    let mut tft_transfer_buffer = [0_u8; 512];
    let mut tft = match TftManager::new(
        tft_spi,
        peripherals.GPIO37,
        peripherals.GPIO40,
        peripherals.GPIO41,
        &mut delay,
        &mut tft_transfer_buffer,
    ) {
        Ok(tft) => tft,
        Err(()) => {
            println!("TFT ST7796 indisponible");
            loop {
                delay.delay_millis(1_000);
            }
        }
    };

    let mut display = match DisplayManager::new(i2c, &delay) {
        Ok(display) => display,
        Err(error) => {
            println!("Ecran SH1106 indisponible: {:?}", error);
            loop {
                delay.delay_millis(1_000);
            }
        }
    };

    let mut encoder = EncoderManager::new(peripherals.GPIO1, peripherals.GPIO2, peripherals.GPIO3);
    let mut pressure = PressureSensor::new(pressure_i2c);
    let mut temperature = TemperatureSensor::new(temperature_spi, peripherals.GPIO10);
    let mut pump = PumpController::new(peripherals.GPIO16);
    let mut boiler_heater = HeaterController::new(peripherals.GPIO17);
    let mut group_heater = GroupHeaterController::new(peripherals.GPIO18);
    let mut water = WaterLevelSensor::new(peripherals.GPIO7);
    let mut tank = TankWaterSensor::new(peripherals.GPIO5);
    let mut steam_valve = ValveController::new(peripherals.GPIO6);
    let mut extraction_valve = ValveController::new(peripherals.GPIO21);
    let mut automation = MachineAutomation::new();
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut buzzer_timer_0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let mut buzzer_timer_1 = ledc.timer::<LowSpeed>(timer::Number::Timer1);
    let mut buzzer_timer_2 = ledc.timer::<LowSpeed>(timer::Number::Timer2);
    let mut buzzer_timer_3 = ledc.timer::<LowSpeed>(timer::Number::Timer3);
    for (timer, frequency) in [
        (&mut buzzer_timer_0, 900.Hz()),
        (&mut buzzer_timer_1, 1_010.Hz()),
        (&mut buzzer_timer_2, 1_135.Hz()),
        (&mut buzzer_timer_3, 1_350.Hz()),
    ] {
        timer
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency,
            })
            .unwrap();
    }
    let buzzer_channel = ledc.channel::<LowSpeed>(channel::Number::Channel0, peripherals.GPIO4);
    let mut buzzer = BuzzerController::new(
        buzzer_channel,
        [
            &buzzer_timer_0,
            &buzzer_timer_1,
            &buzzer_timer_2,
            &buzzer_timer_3,
        ],
    )
    .unwrap();
    let mut settings_store = SettingsStore::new();
    match settings_store.load() {
        Ok(Some(settings)) => {
            boiler_heater.set_target(settings.steam_target_bar);
            automation.set_extraction_target(settings.extraction_target_bar);
            group_heater.set_target(settings.group_target_c);
            buzzer.set_sound_settings(
                settings.startup_sound_enabled,
                settings.click_sound_enabled,
                settings.rotation_sound_enabled,
                settings.tank_alarm_sound_enabled,
            );
            println!(
                "Reglages restaures: vapeur={:.2} bar, extraction={:.2} bar, groupe={:.1} C",
                boiler_heater.target_bar(),
                automation.extraction_target_bar(),
                group_heater.target_c()
            );
        }
        Ok(None) => println!("Aucun reglage sauvegarde: valeurs par defaut utilisees"),
        Err(error) => println!("Lecture des reglages impossible: {:?}", error),
    }
    // La machine démarre en veille. Un clic ouvre ensuite la page d'accueil.
    let mut system_enabled = false;
    let mut water_is_full = water.is_full();
    let mut tank_has_water = tank.has_water();
    let mut steam_pressure_bars = None;
    let mut extraction_pressure_bars = None;
    let mut temperature_c = None;
    let mut current_page = DisplayPage::Standby;
    let mut menu_item = MenuItem::SteamBoiler;
    let mut sound_setting = SoundSetting::Startup;
    let mut pressure_error_reported = false;
    let mut temperature_error_reported = false;
    let mut oled_dirty = true;
    let mut tft_dirty = true;
    let mut last_display_refresh_at = time::now().ticks().wrapping_sub(200_000);
    let mut last_automation_phase = automation.phase();
    let mut last_automation_outputs = AutomationOutputs {
        pump_on: false,
        steam_valve_open: false,
        extraction_valve_open: false,
    };
    let mut settings_dirty = false;
    let mut settings_changed_at = time::now().ticks();

    println!("Encodeur configure sur CLK=GPIO1, DT=GPIO2, SW=GPIO3");
    println!("Buzzer MH-FMD configure sur GPIO4, VCC=5V, actif LOW");
    println!("Electrovanne vapeur configuree sur GPIO6, LOW=OUVERTE, HIGH=FERMEE");
    println!("Electrovanne extraction configuree sur GPIO21, LOW=OUVERTE, HIGH=FERMEE");
    println!("Commande pompe configuree sur GPIO16");
    println!("SSR chaudiere configure sur GPIO17, active HIGH");
    println!("SSR groupe configure sur GPIO18, active HIGH");
    println!("Controleur niveau eau configure sur GPIO7, contact sec NO vers GND ESP32");
    println!("Capteur du bac configure sur GPIO5, LOW = eau presente");
    println!("ADS1115 configure sur SDA=GPIO11, SCL=GPIO12: vapeur=A0, extraction=A1");
    println!("MAX31865 groupe configure sur CS=GPIO10, SDO=GPIO13, SDI=GPIO14, CLK=GPIO15");
    println!("TFT ST7796 configure sur CS=GPIO37, DC=GPIO40, RES=GPIO41, SDA=GPIO35, SCL=GPIO36");
    println!(
        "Eau initiale: {}",
        if water_is_full { "PLEIN" } else { "PAS PLEIN" }
    );
    println!("Machine en VEILLE - cliquer sur l'encodeur pour demarrer");

    pump.set_on(false);
    boiler_heater.force_off();
    group_heater.force_off();

    macro_rules! refresh_display {
        () => {
            tft_dirty = true;
        };
    }
    macro_rules! refresh_live_data_display {
        () => {
            oled_dirty = true;
            if matches!(
                current_page,
                DisplayPage::Home
                    | DisplayPage::SteamBoiler
                    | DisplayPage::Group
                    | DisplayPage::ExtractionPressure
                    | DisplayPage::Status
            ) {
                tft_dirty = true;
            }
        };
    }
    macro_rules! mark_settings_dirty {
        () => {
            settings_dirty = true;
            settings_changed_at = time::now().ticks();
        };
    }
    loop {
        if let Some(event) = encoder.poll() {
            match event {
                EncoderEvent::Button if current_page != DisplayPage::Standby => buzzer.play_click(),
                EncoderEvent::Clockwise | EncoderEvent::CounterClockwise
                    if current_page != DisplayPage::Standby =>
                {
                    buzzer.play_rotate()
                }
                _ => {}
            }
            match (current_page, event) {
                (DisplayPage::Standby, EncoderEvent::Button) => {
                    system_enabled = true;
                    buzzer.play_marseillaise();
                    boiler_heater.update_control(true, steam_pressure_bars);
                    group_heater.update_control(true, temperature_c);
                    current_page = DisplayPage::Home;
                    println!("Machine mise en SERVICE");
                }
                (DisplayPage::Standby, _) => {}
                (DisplayPage::Home, EncoderEvent::Button) => current_page = DisplayPage::Menu,
                (DisplayPage::Home, _) => {}
                (DisplayPage::Menu, EncoderEvent::Clockwise) => menu_item.next(),
                (DisplayPage::Menu, EncoderEvent::CounterClockwise) => menu_item.previous(),
                (DisplayPage::Menu, EncoderEvent::Button) => match menu_item {
                    MenuItem::Home => current_page = DisplayPage::Home,
                    MenuItem::SteamBoiler => current_page = DisplayPage::SteamBoiler,
                    MenuItem::Group => current_page = DisplayPage::Group,
                    MenuItem::ExtractionPressure => current_page = DisplayPage::ExtractionPressure,
                    MenuItem::Status => current_page = DisplayPage::Status,
                    MenuItem::Settings => current_page = DisplayPage::Settings,
                    MenuItem::Standby => {
                        system_enabled = false;
                        pump.set_on(false);
                        boiler_heater.force_off();
                        group_heater.force_off();
                        steam_valve.force_closed();
                        extraction_valve.force_closed();
                        current_page = DisplayPage::Standby;
                        println!("Machine mise en VEILLE");
                    }
                },
                (DisplayPage::SteamBoiler, EncoderEvent::Clockwise) => {
                    boiler_heater.adjust_target(TARGET_STEP_BAR);
                    mark_settings_dirty!();
                    boiler_heater.update_control(system_enabled, steam_pressure_bars);
                    println!("Consigne pression: {:.2} bar", boiler_heater.target_bar());
                }
                (DisplayPage::SteamBoiler, EncoderEvent::CounterClockwise) => {
                    boiler_heater.adjust_target(-TARGET_STEP_BAR);
                    mark_settings_dirty!();
                    boiler_heater.update_control(system_enabled, steam_pressure_bars);
                    println!("Consigne pression: {:.2} bar", boiler_heater.target_bar());
                }
                (DisplayPage::Group, EncoderEvent::Clockwise) => {
                    group_heater.adjust_target(TARGET_STEP_C);
                    mark_settings_dirty!();
                    group_heater.update_control(system_enabled, temperature_c);
                    println!("Consigne groupe: {:.0} C", group_heater.target_c());
                }
                (DisplayPage::Group, EncoderEvent::CounterClockwise) => {
                    group_heater.adjust_target(-TARGET_STEP_C);
                    mark_settings_dirty!();
                    group_heater.update_control(system_enabled, temperature_c);
                    println!("Consigne groupe: {:.0} C", group_heater.target_c());
                }
                (DisplayPage::ExtractionPressure, EncoderEvent::Clockwise) => {
                    automation.adjust_extraction_target(EXTRACTION_TARGET_STEP_BAR);
                    mark_settings_dirty!();
                    println!(
                        "Consigne extraction: {:.2} bar",
                        automation.extraction_target_bar()
                    );
                }
                (DisplayPage::ExtractionPressure, EncoderEvent::CounterClockwise) => {
                    automation.adjust_extraction_target(-EXTRACTION_TARGET_STEP_BAR);
                    mark_settings_dirty!();
                    println!(
                        "Consigne extraction: {:.2} bar",
                        automation.extraction_target_bar()
                    );
                }
                (DisplayPage::Settings, EncoderEvent::Clockwise) => sound_setting.next(),
                (DisplayPage::Settings, EncoderEvent::CounterClockwise) => sound_setting.previous(),
                (DisplayPage::Settings, EncoderEvent::Button) => match sound_setting {
                    SoundSetting::Startup => {
                        buzzer.toggle_startup();
                        mark_settings_dirty!();
                    }
                    SoundSetting::Click => {
                        buzzer.toggle_click();
                        mark_settings_dirty!();
                    }
                    SoundSetting::Rotation => {
                        buzzer.toggle_rotation();
                        mark_settings_dirty!();
                    }
                    SoundSetting::TankAlarm => {
                        buzzer.toggle_tank_alarm();
                        mark_settings_dirty!();
                    }
                    SoundSetting::Back => current_page = DisplayPage::Menu,
                },
                (
                    DisplayPage::SteamBoiler
                    | DisplayPage::Group
                    | DisplayPage::ExtractionPressure
                    | DisplayPage::Status,
                    EncoderEvent::Button,
                ) => {
                    current_page = DisplayPage::Menu;
                }
                (DisplayPage::Status, _) => {}
            }
            refresh_display!();
        }

        if let Some(new_water_is_full) = water.poll() {
            water_is_full = new_water_is_full;
            boiler_heater.update_control(system_enabled, steam_pressure_bars);

            println!(
                "Eau: {}, pompe automatique {}",
                if water_is_full { "PLEIN" } else { "PAS PLEIN" },
                if pump.is_on() { "ON" } else { "OFF" }
            );

            refresh_live_data_display!();
        }

        if let Some(new_tank_has_water) = tank.poll() {
            tank_has_water = new_tank_has_water;
            println!(
                "Bac: {}, pompe {}",
                if tank_has_water {
                    "EAU PRESENTE"
                } else {
                    "VIDE"
                },
                if pump.is_on() { "ON" } else { "OFF" }
            );

            refresh_live_data_display!();
        }

        match pressure.poll() {
            Ok(Some(readings)) => {
                print_pressure_sample("A0 vapeur", &readings.steam);
                print_pressure_sample("A1 extraction", &readings.extraction);
                steam_pressure_bars = readings.steam.bars;
                extraction_pressure_bars = readings.extraction.bars;
                boiler_heater.update_control(system_enabled, steam_pressure_bars);
                pressure_error_reported = false;
                refresh_live_data_display!();
            }
            Ok(None) => {}
            Err(error) => {
                if !pressure_error_reported {
                    match error {
                        PressureError::I2c(i2c_error) => {
                            println!("ADS1115 indisponible: {:?}", i2c_error)
                        }
                        PressureError::InvalidConversionState => {
                            println!("Cycle de conversion ADS1115 incoherent")
                        }
                    }
                    pressure_error_reported = true;
                }

                if steam_pressure_bars.is_some() || extraction_pressure_bars.is_some() {
                    steam_pressure_bars = None;
                    extraction_pressure_bars = None;
                    boiler_heater.force_off();
                    refresh_live_data_display!();
                }
            }
        }

        match temperature.poll() {
            Ok(Some(new_temperature_c)) => {
                temperature_c = Some(new_temperature_c);
                group_heater.update_control(system_enabled, temperature_c);
                println!("Temperature groupe PT100: {:.2} C", new_temperature_c);
                temperature_error_reported = false;
                refresh_live_data_display!();
            }
            Ok(None) => {}
            Err(error) => {
                if !temperature_error_reported {
                    match error {
                        TemperatureError::Spi(spi_error) => {
                            println!("MAX31865 indisponible: {:?}", spi_error)
                        }
                        TemperatureError::Configuration(config) => {
                            println!("Configuration MAX31865 invalide: 0x{:02X}", config)
                        }
                        TemperatureError::Fault(status) => {
                            println!("Defaut sonde MAX31865: status=0x{:02X}", status)
                        }
                        TemperatureError::MissingProbe { config, rtd_value } => {
                            println!(
                                "MAX31865 mesure nulle: config=0x{:02X}, rtd=0x{:04X}",
                                config, rtd_value
                            )
                        }
                        TemperatureError::InvalidTemperature(value) => {
                            println!("Temperature PT100 incoherente: {:.2} C", value)
                        }
                    }
                    temperature_error_reported = true;
                }

                if temperature_c.is_some() {
                    temperature_c = None;
                    group_heater.force_off();
                    refresh_live_data_display!();
                }
            }
        }

        let automation_outputs = automation.update(
            system_enabled,
            tank_has_water,
            water_is_full,
            steam_pressure_bars,
            boiler_heater.target_bar(),
            extraction_pressure_bars,
        );
        if automation_outputs.pump_on {
            // Au démarrage, la pompe prend son régime avant qu'une vanne ne
            // soit éventuellement ouverte par l'automatisme 500 ms plus tard.
            pump.set_on(true);
            steam_valve.set_open(automation_outputs.steam_valve_open);
            extraction_valve.set_open(automation_outputs.extraction_valve_open);
        } else {
            // Invariant de sécurité : aucune électrovanne ne reste ouverte
            // lorsque la pompe est arrêtée. On ferme avant de couper la pompe.
            steam_valve.force_closed();
            extraction_valve.force_closed();
            pump.set_on(false);
        }
        if automation_outputs != last_automation_outputs {
            last_automation_outputs = automation_outputs;
            println!(
                "Sorties automatiques: pompe={} | vanne vapeur={} | vanne extraction={}",
                if automation_outputs.pump_on {
                    "ON"
                } else {
                    "OFF"
                },
                if automation_outputs.steam_valve_open {
                    "OUVERTE"
                } else {
                    "FERMEE"
                },
                if automation_outputs.extraction_valve_open {
                    "OUVERTE"
                } else {
                    "FERMEE"
                },
            );
            refresh_live_data_display!();
        }
        if automation.phase() != last_automation_phase {
            last_automation_phase = automation.phase();
            println!(
                "Phase automatique: {:?} | vapeur={:?}/{:.2} bar | extraction={:?}/{:.2} bar | pompe={} | vanne vapeur={} | vanne extraction={}",
                last_automation_phase,
                steam_pressure_bars,
                boiler_heater.target_bar(),
                extraction_pressure_bars,
                automation.extraction_target_bar(),
                if automation_outputs.pump_on { "ON" } else { "OFF" },
                if automation_outputs.steam_valve_open {
                    "OUVERTE"
                } else {
                    "FERMEE"
                },
                if automation_outputs.extraction_valve_open {
                    "OUVERTE"
                } else {
                    "FERMEE"
                },
            );
            refresh_live_data_display!();
        }

        boiler_heater.tick();
        group_heater.tick();
        buzzer.tick(system_enabled, tank_has_water);

        let settings_now = time::now().ticks();
        if settings_dirty
            && (!system_enabled || settings_now.wrapping_sub(settings_changed_at) >= 2_000_000)
        {
            let settings = MachineSettings {
                steam_target_bar: boiler_heater.target_bar(),
                extraction_target_bar: automation.extraction_target_bar(),
                group_target_c: group_heater.target_c(),
                startup_sound_enabled: buzzer.startup_enabled(),
                click_sound_enabled: buzzer.click_enabled(),
                rotation_sound_enabled: buzzer.rotation_enabled(),
                tank_alarm_sound_enabled: buzzer.tank_alarm_enabled(),
            };
            match settings_store.save(settings) {
                Ok(()) => {
                    settings_dirty = false;
                    println!("Reglages sauvegardes dans la flash");
                }
                Err(error) => {
                    settings_changed_at = settings_now;
                    println!("Sauvegarde des reglages impossible: {:?}", error);
                }
            }
        }

        let now = time::now().ticks();
        if (oled_dirty || tft_dirty) && now.wrapping_sub(last_display_refresh_at) >= 200_000 {
            last_display_refresh_at = now;
            if oled_dirty {
                match display.show_telemetry(
                    steam_pressure_bars,
                    extraction_pressure_bars,
                    temperature_c,
                ) {
                    Ok(()) => oled_dirty = false,
                    Err(error) => {
                        println!("Erreur de communication avec l'OLED: {:?}", error);
                    }
                }
            }
            if tft_dirty {
                match tft.show_page(
                    current_page,
                    UiState {
                        pump_is_on: pump.is_on(),
                        boiler_is_full: water_is_full,
                        tank_has_water,
                        steam_pressure_bars,
                        extraction_pressure_bars,
                        extraction_target_pressure_bar: automation.extraction_target_bar(),
                        extraction_ready: automation.extraction_ready(),
                        temperature_c,
                        boiler_heater_duty_percent: boiler_heater.duty_percent(),
                        target_pressure_bar: boiler_heater.target_bar(),
                        group_heater_duty_percent: group_heater.duty_percent(),
                        target_group_temperature_c: group_heater.target_c(),
                        system_enabled,
                        startup_sound_enabled: buzzer.startup_enabled(),
                        click_sound_enabled: buzzer.click_enabled(),
                        rotation_sound_enabled: buzzer.rotation_enabled(),
                        tank_alarm_sound_enabled: buzzer.tank_alarm_enabled(),
                        steam_valve_open: steam_valve.is_open(),
                        extraction_valve_open: extraction_valve.is_open(),
                        sound_setting,
                        menu_item,
                    },
                ) {
                    Ok(()) => tft_dirty = false,
                    Err(()) => println!("Erreur de communication avec le TFT"),
                }
            }
        }
    }
}

fn print_pressure_sample(name: &str, sample: &PressureSample) {
    println!(
        "ADS1115 {}: brut={} tension={:.4} V config=0x{:04X}{}",
        name,
        sample.raw,
        sample.voltage,
        sample.config,
        if sample.bars.is_some() {
            ""
        } else {
            " (hors plage pression)"
        }
    );
}

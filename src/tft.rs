use core::fmt::Write;

use embedded_graphics::{
    image::{Image, ImageRawBE},
    mono_font::{
        ascii::{FONT_10X20, FONT_6X10, FONT_7X13_BOLD, FONT_9X15_BOLD},
        MonoFont, MonoTextStyle, MonoTextStyleBuilder,
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{
        Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
    },
    text::{Alignment, Baseline, Text},
};
use embedded_hal::{delay::DelayNs, spi::SpiBus};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{gpio::Output, peripheral::Peripheral};
use esp_println::println;
use heapless::String;
use mipidsi::{
    interface::SpiInterface,
    models::ST7796,
    options::{ColorOrder, Orientation, Rotation},
    Builder, Display,
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DisplayPage {
    Standby,
    Home,
    Menu,
    SteamBoiler,
    Group,
    ExtractionPressure,
    Status,
    Settings,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum MenuItem {
    Home,
    SteamBoiler,
    Group,
    ExtractionPressure,
    Status,
    Settings,
    Standby,
}

impl MenuItem {
    pub fn next(&mut self) {
        *self = match self {
            Self::Home => Self::SteamBoiler,
            Self::SteamBoiler => Self::Group,
            Self::Group => Self::ExtractionPressure,
            Self::ExtractionPressure => Self::Status,
            Self::Status => Self::Settings,
            Self::Settings => Self::Standby,
            Self::Standby => Self::Home,
        };
    }

    pub fn previous(&mut self) {
        *self = match self {
            Self::Home => Self::Standby,
            Self::SteamBoiler => Self::Home,
            Self::Group => Self::SteamBoiler,
            Self::ExtractionPressure => Self::Group,
            Self::Status => Self::ExtractionPressure,
            Self::Settings => Self::Status,
            Self::Standby => Self::Settings,
        };
    }

    fn index(self) -> usize {
        match self {
            Self::Home => 0,
            Self::SteamBoiler => 1,
            Self::Group => 2,
            Self::ExtractionPressure => 3,
            Self::Status => 4,
            Self::Settings => 5,
            Self::Standby => 6,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SoundSetting {
    Startup,
    Click,
    Rotation,
    TankAlarm,
    Back,
}

impl SoundSetting {
    pub fn next(&mut self) {
        *self = match self {
            Self::Startup => Self::Click,
            Self::Click => Self::Rotation,
            Self::Rotation => Self::TankAlarm,
            Self::TankAlarm => Self::Back,
            Self::Back => Self::Startup,
        };
    }

    pub fn previous(&mut self) {
        *self = match self {
            Self::Startup => Self::Back,
            Self::Click => Self::Startup,
            Self::Rotation => Self::Click,
            Self::TankAlarm => Self::Rotation,
            Self::Back => Self::TankAlarm,
        };
    }

    fn index(self) -> usize {
        match self {
            Self::Startup => 0,
            Self::Click => 1,
            Self::Rotation => 2,
            Self::TankAlarm => 3,
            Self::Back => 4,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub struct UiState {
    pub pump_is_on: bool,
    pub boiler_is_full: bool,
    pub tank_has_water: bool,
    pub steam_pressure_bars: Option<f32>,
    pub extraction_pressure_bars: Option<f32>,
    pub extraction_target_pressure_bar: f32,
    pub extraction_ready: bool,
    pub temperature_c: Option<f32>,
    pub boiler_heater_duty_percent: u8,
    pub target_pressure_bar: f32,
    pub group_heater_duty_percent: u8,
    pub target_group_temperature_c: f32,
    pub system_enabled: bool,
    pub startup_sound_enabled: bool,
    pub click_sound_enabled: bool,
    pub rotation_sound_enabled: bool,
    pub tank_alarm_sound_enabled: bool,
    pub steam_valve_open: bool,
    pub extraction_valve_open: bool,
    pub sound_setting: SoundSetting,
    pub menu_item: MenuItem,
}

type Interface<'a, 'd, SPI> =
    SpiInterface<'a, ExclusiveDevice<SPI, Output<'d>, NoDelay>, Output<'d>>;

pub struct TftManager<'a, 'd, SPI>
where
    SPI: SpiBus<u8>,
{
    display: Display<Interface<'a, 'd, SPI>, ST7796, Output<'d>>,
    current_page: Option<DisplayPage>,
    last_state: Option<UiState>,
}

impl<'a, 'd, SPI> TftManager<'a, 'd, SPI>
where
    SPI: SpiBus<u8>,
{
    pub fn new<CS, DC, RST>(
        spi: SPI,
        cs_pin: impl Peripheral<P = CS> + 'd,
        dc_pin: impl Peripheral<P = DC> + 'd,
        rst_pin: impl Peripheral<P = RST> + 'd,
        delay: &mut impl DelayNs,
        transfer_buffer: &'a mut [u8],
    ) -> Result<Self, ()>
    where
        CS: esp_hal::gpio::OutputPin,
        DC: esp_hal::gpio::OutputPin,
        RST: esp_hal::gpio::OutputPin,
    {
        let cs = Output::new(cs_pin, esp_hal::gpio::Level::High);
        let dc = Output::new(dc_pin, esp_hal::gpio::Level::Low);
        let rst = Output::new(rst_pin, esp_hal::gpio::Level::High);
        let spi_device = ExclusiveDevice::new_no_delay(spi, cs).map_err(|_| ())?;
        let interface = SpiInterface::new(spi_device, dc, transfer_buffer);
        let display = Builder::new(ST7796, interface)
            .reset_pin(rst)
            .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
            .color_order(ColorOrder::Bgr)
            .init(delay)
            .map_err(|_| ())?;
        println!("TFT ST7796 initialise: interface principale active");
        Ok(Self {
            display,
            current_page: None,
            last_state: None,
        })
    }

    pub fn show_page(&mut self, page: DisplayPage, state: UiState) -> Result<(), ()> {
        // Un effacement complet rend le balayage du ST7796 visible. Il n'est
        // nécessaire qu'au changement de page : chaque carte repeint ensuite
        // son propre fond lors des mises à jour de valeurs.
        let page_changed = self.current_page != Some(page);
        if page_changed {
            self.display.clear(BG).map_err(|_| ())?;
            self.current_page = Some(page);
        }
        if page_changed || self.last_state.is_none() {
            match page {
                DisplayPage::Standby => self.draw_standby(),
                DisplayPage::Home => self.draw_home(state),
                DisplayPage::Menu => self.draw_menu(state),
                DisplayPage::SteamBoiler => self.draw_steam(state),
                DisplayPage::Group => self.draw_group(state),
                DisplayPage::ExtractionPressure => self.draw_extraction(state),
                DisplayPage::Status => self.draw_status(state),
                DisplayPage::Settings => self.draw_settings(state),
            }
        } else if let Some(previous) = self.last_state {
            self.update_page(page, previous, state);
        }
        self.last_state = Some(state);
        Ok(())
    }

    fn update_page(&mut self, page: DisplayPage, previous: UiState, state: UiState) {
        match page {
            DisplayPage::Standby => {}
            DisplayPage::Home => {
                if value_changed(previous.steam_pressure_bars, state.steam_pressure_bars) {
                    self.update_metric_value(Point::new(18, 69), state.steam_pressure_bars);
                }
                if value_changed(previous.temperature_c, state.temperature_c) {
                    self.update_metric_value(Point::new(170, 69), state.temperature_c);
                }
                if value_changed(
                    previous.extraction_pressure_bars,
                    state.extraction_pressure_bars,
                ) {
                    self.update_metric_value(Point::new(322, 69), state.extraction_pressure_bars);
                }
                if previous.extraction_ready != state.extraction_ready {
                    self.home_status_card(
                        Point::new(351, 205),
                        "EXTRACTION",
                        if state.extraction_ready {
                            "PRETE"
                        } else {
                            "EN CHARGE"
                        },
                        state.extraction_ready,
                        false,
                    );
                }
                if previous.pump_is_on != state.pump_is_on {
                    self.home_status_card(
                        Point::new(18, 205),
                        "POMPE",
                        if state.pump_is_on {
                            "ACTIVE"
                        } else {
                            "ARRETEE"
                        },
                        state.pump_is_on,
                        false,
                    );
                }
                if previous.boiler_is_full != state.boiler_is_full {
                    self.home_status_card(
                        Point::new(129, 205),
                        "CHAUDIERE",
                        if state.boiler_is_full {
                            "PLEINE"
                        } else {
                            "NIVEAU BAS"
                        },
                        state.boiler_is_full,
                        !state.boiler_is_full,
                    );
                }
                if previous.tank_has_water != state.tank_has_water {
                    self.home_status_card(
                        Point::new(240, 205),
                        "RESERVOIR",
                        if state.tank_has_water {
                            "EAU PRESENTE"
                        } else {
                            "A REMPLIR"
                        },
                        state.tank_has_water,
                        !state.tank_has_water,
                    );
                }
            }
            DisplayPage::Menu => {
                if previous.menu_item != state.menu_item {
                    self.draw_menu(state);
                }
            }
            DisplayPage::SteamBoiler => {
                if value_changed(previous.steam_pressure_bars, state.steam_pressure_bars) {
                    self.update_large_value(Point::new(34, 100), state.steam_pressure_bars, "bar");
                }
                if value_changed(
                    Some(previous.target_pressure_bar),
                    Some(state.target_pressure_bar),
                ) {
                    self.update_target_value(
                        Point::new(268, 82),
                        state.target_pressure_bar,
                        "bar",
                        2,
                    );
                }
                if previous.boiler_heater_duty_percent != state.boiler_heater_duty_percent {
                    self.update_duty(Point::new(24, 211), state.boiler_heater_duty_percent);
                }
            }
            DisplayPage::Group => {
                if value_changed(previous.temperature_c, state.temperature_c) {
                    self.update_large_value(Point::new(34, 100), state.temperature_c, "C");
                }
                if value_changed(
                    Some(previous.target_group_temperature_c),
                    Some(state.target_group_temperature_c),
                ) {
                    self.update_target_value(
                        Point::new(268, 82),
                        state.target_group_temperature_c,
                        "C",
                        0,
                    );
                }
                if previous.group_heater_duty_percent != state.group_heater_duty_percent {
                    self.update_duty(Point::new(24, 211), state.group_heater_duty_percent);
                }
            }
            DisplayPage::ExtractionPressure => {
                if value_changed(
                    previous.extraction_pressure_bars,
                    state.extraction_pressure_bars,
                ) {
                    self.update_large_value(
                        Point::new(34, 100),
                        state.extraction_pressure_bars,
                        "bar",
                    );
                }
                if value_changed(
                    Some(previous.extraction_target_pressure_bar),
                    Some(state.extraction_target_pressure_bar),
                ) {
                    self.update_target_value(
                        Point::new(268, 82),
                        state.extraction_target_pressure_bar,
                        "bar",
                        2,
                    );
                }
                if previous.extraction_ready != state.extraction_ready {
                    self.badge(
                        Point::new(202, 215),
                        if state.extraction_ready {
                            "PRETE"
                        } else {
                            "CHARGE"
                        },
                        state.extraction_ready,
                    );
                }
            }
            DisplayPage::Status => {
                let status_changed = previous.pump_is_on != state.pump_is_on
                    || previous.boiler_is_full != state.boiler_is_full
                    || previous.tank_has_water != state.tank_has_water
                    || previous.boiler_heater_duty_percent != state.boiler_heater_duty_percent
                    || previous.group_heater_duty_percent != state.group_heater_duty_percent
                    || previous.steam_valve_open != state.steam_valve_open
                    || previous.extraction_valve_open != state.extraction_valve_open
                    || previous.system_enabled != state.system_enabled;
                if status_changed {
                    self.draw_status(state);
                }
            }
            DisplayPage::Settings => {
                if previous.startup_sound_enabled != state.startup_sound_enabled
                    || previous.click_sound_enabled != state.click_sound_enabled
                    || previous.rotation_sound_enabled != state.rotation_sound_enabled
                    || previous.tank_alarm_sound_enabled != state.tank_alarm_sound_enabled
                    || previous.sound_setting != state.sound_setting
                {
                    self.draw_settings(state);
                }
            }
        }
    }

    fn draw_standby(&mut self) {
        const IMAGE_BYTES: &[u8] = include_bytes!("../assets/standby.rgb565");
        let raw = ImageRawBE::<Rgb565>::new(IMAGE_BYTES, 480);
        let _ = Image::new(&raw, Point::zero()).draw(&mut self.display);
        self.button(
            Point::new(137, 271),
            Size::new(206, 37),
            "CLIQUER POUR ALLUMER",
        );
    }

    fn draw_home(&mut self, state: UiState) {
        self.header("TABLEAU DE BORD", state.system_enabled);
        self.metric_card(
            Point::new(18, 69),
            "VAPEUR",
            state.steam_pressure_bars,
            "bar",
        );
        self.metric_card(Point::new(170, 69), "GROUPE", state.temperature_c, "C");
        self.metric_card(
            Point::new(322, 69),
            "EXTRACTION",
            state.extraction_pressure_bars,
            "bar",
        );
        self.home_status_card(
            Point::new(18, 205),
            "POMPE",
            if state.pump_is_on {
                "ACTIVE"
            } else {
                "ARRETEE"
            },
            state.pump_is_on,
            false,
        );
        self.home_status_card(
            Point::new(129, 205),
            "CHAUDIERE",
            if state.boiler_is_full {
                "PLEINE"
            } else {
                "NIVEAU BAS"
            },
            state.boiler_is_full,
            !state.boiler_is_full,
        );
        self.home_status_card(
            Point::new(240, 205),
            "RESERVOIR",
            if state.tank_has_water {
                "EAU PRESENTE"
            } else {
                "A REMPLIR"
            },
            state.tank_has_water,
            !state.tank_has_water,
        );
        self.home_status_card(
            Point::new(351, 205),
            "EXTRACTION",
            if state.extraction_ready {
                "PRETE"
            } else {
                "EN CHARGE"
            },
            state.extraction_ready,
            false,
        );
        self.footer("Appuyer pour ouvrir le menu");
    }

    fn draw_menu(&mut self, state: UiState) {
        self.header("MENU", state.system_enabled);
        let labels = [
            ("ACCUEIL", "Vue d'ensemble"),
            ("CHAUDIERE VAPEUR", "Pression et chauffe"),
            ("GROUPE", "Temperature et chauffe"),
            ("CHAUDIERE EXTRACTION", "Pression et consigne"),
            ("ETAT DE LA MACHINE", "Eau, pompe, vannes et chauffe"),
            ("REGLAGES", "Sons et preferences"),
        ];
        for (index, (title, subtitle)) in labels.iter().enumerate() {
            let position = Point::new(18 + (index % 2) as i32 * 227, 70 + (index / 2) as i32 * 52);
            self.menu_card(position, title, subtitle, index == state.menu_item.index());
        }
        self.menu_action_card(
            Point::new(18, 230),
            "MISE EN VEILLE",
            "Arreter la machine en toute securite",
            state.menu_item == MenuItem::Standby,
        );
        self.footer("Tourner pour naviguer  -  Appuyer pour ouvrir");
    }

    fn draw_steam(&mut self, state: UiState) {
        self.page_header(
            "CHAUDIERE VAPEUR",
            "Pression et regulation",
            state.system_enabled,
        );
        self.card(Point::new(18, 82), Size::new(230, 108), false);
        self.large_value(Point::new(34, 100), state.steam_pressure_bars, "bar");
        self.target_panel(Point::new(268, 82), state.target_pressure_bar, "bar", 2);
        self.duty_panel(
            Point::new(24, 211),
            "PUISSANCE DE CHAUFFE",
            state.boiler_heater_duty_percent,
        );
        self.footer("Tourner: regler  -  Appuyer: valider");
    }

    fn draw_group(&mut self, state: UiState) {
        self.page_header("GROUPE", "Temperature et regulation", state.system_enabled);
        self.card(Point::new(18, 82), Size::new(230, 108), false);
        self.large_value(Point::new(34, 100), state.temperature_c, "C");
        self.target_panel(
            Point::new(268, 82),
            state.target_group_temperature_c,
            "C",
            0,
        );
        self.duty_panel(
            Point::new(24, 211),
            "PUISSANCE DE CHAUFFE",
            state.group_heater_duty_percent,
        );
        self.footer("Tourner: regler  -  Appuyer: valider");
    }

    fn draw_extraction(&mut self, state: UiState) {
        self.page_header(
            "CHAUDIERE EXTRACTION",
            "Pression geree par la pompe",
            state.system_enabled,
        );
        self.card(Point::new(18, 82), Size::new(230, 108), false);
        self.large_value(Point::new(34, 100), state.extraction_pressure_bars, "bar");
        self.target_panel(
            Point::new(268, 82),
            state.extraction_target_pressure_bar,
            "bar",
            2,
        );
        self.badge(
            Point::new(202, 215),
            if state.extraction_ready {
                "PRETE"
            } else {
                "CHARGE"
            },
            state.extraction_ready,
        );
        self.footer("Tourner: modifier la consigne  -  Appuyer: retour");
    }

    fn draw_status(&mut self, state: UiState) {
        self.page_header(
            "ETAT DE LA MACHINE",
            "Diagnostic en temps reel",
            state.system_enabled,
        );
        let items = [
            (
                "SYSTEME",
                if state.system_enabled {
                    "EN SERVICE"
                } else {
                    "EN VEILLE"
                },
                state.system_enabled,
                false,
            ),
            (
                "POMPE",
                if state.pump_is_on {
                    "ACTIVE"
                } else {
                    "ARRETEE"
                },
                state.pump_is_on,
                false,
            ),
            (
                "CHAUDIERE",
                if state.boiler_is_full {
                    "PLEINE"
                } else {
                    "NIVEAU BAS"
                },
                state.boiler_is_full,
                !state.boiler_is_full,
            ),
            (
                "RESERVOIR",
                if state.tank_has_water {
                    "EAU PRESENTE"
                } else {
                    "VIDE"
                },
                state.tank_has_water,
                !state.tank_has_water,
            ),
            (
                "VANNE VAPEUR",
                if state.steam_valve_open {
                    "OUVERTE"
                } else {
                    "FERMEE"
                },
                state.steam_valve_open,
                false,
            ),
            (
                "VANNE EXTRACTION",
                if state.extraction_valve_open {
                    "OUVERTE"
                } else {
                    "FERMEE"
                },
                state.extraction_valve_open,
                false,
            ),
            (
                "SSR VAPEUR",
                if state.boiler_heater_duty_percent > 0 {
                    "EN CHAUFFE"
                } else {
                    "ARRETE"
                },
                state.boiler_heater_duty_percent > 0,
                false,
            ),
            (
                "SSR GROUPE",
                if state.group_heater_duty_percent > 0 {
                    "EN CHAUFFE"
                } else {
                    "ARRETE"
                },
                state.group_heater_duty_percent > 0,
                false,
            ),
        ];
        for (index, (label, value, active, warning)) in items.iter().enumerate() {
            let position = Point::new(18 + (index % 2) as i32 * 227, 70 + (index / 2) as i32 * 49);
            self.status_wide(position, label, value, *active, *warning);
        }
        self.footer("Appuyer pour revenir au menu");
    }

    fn draw_settings(&mut self, state: UiState) {
        self.page_header(
            "REGLAGES",
            "Gestion individuelle des sons",
            state.system_enabled,
        );
        let rows = [
            ("SON DE DEMARRAGE", Some(state.startup_sound_enabled)),
            ("RETOUR DES CLICS", Some(state.click_sound_enabled)),
            ("RETOUR DE ROTATION", Some(state.rotation_sound_enabled)),
            ("ALARME RESERVOIR", Some(state.tank_alarm_sound_enabled)),
            ("RETOUR AU MENU", None),
        ];
        for (index, (label, enabled)) in rows.iter().enumerate() {
            let position = Point::new(30, 67 + index as i32 * 43);
            let selected = index == state.sound_setting.index();
            self.card(position, Size::new(420, 36), selected);
            self.text(
                label,
                position + Point::new(15, 11),
                FONT_7X13_BOLD,
                if selected { PANEL } else { TEXT },
            );
            if let Some(enabled) = enabled {
                self.badge(
                    position + Point::new(326, 6),
                    if *enabled { "ACTIF" } else { "COUPE" },
                    *enabled,
                );
            }
        }
        self.footer("Tourner: selectionner  -  Appuyer: modifier");
    }

    fn header(&mut self, title: &str, enabled: bool) {
        self.text("LA MILLARZOCCO", Point::new(18, 17), FONT_9X15_BOLD, TEXT);
        self.text(title, Point::new(18, 39), FONT_6X10, MUTED);
        self.badge(
            Point::new(385, 20),
            if enabled { "SERVICE" } else { "VEILLE" },
            enabled,
        );
        self.separator(58);
    }

    fn page_header(&mut self, title: &str, subtitle: &str, enabled: bool) {
        self.text(title, Point::new(18, 15), FONT_9X15_BOLD, TEXT);
        self.text(subtitle, Point::new(18, 38), FONT_6X10, MUTED);
        self.badge(
            Point::new(385, 20),
            if enabled { "SERVICE" } else { "VEILLE" },
            enabled,
        );
        self.separator(58);
    }

    fn metric_card(&mut self, position: Point, label: &str, value: Option<f32>, unit: &str) {
        self.card(position, Size::new(140, 116), false);
        self.text(label, position + Point::new(12, 13), FONT_6X10, MUTED);
        self.update_metric_value(position, value);
        self.centered(unit, position + Point::new(70, 91), FONT_7X13_BOLD, ACCENT);
    }

    fn update_metric_value(&mut self, position: Point, value: Option<f32>) {
        let mut value_text: String<16> = String::new();
        match value {
            Some(v) => write!(&mut value_text, "{:5.2}", v).ok(),
            None => write!(&mut value_text, "--.--").ok(),
        };
        self.centered_bg(
            value_text.as_str(),
            position + Point::new(70, 58),
            FONT_10X20,
            TEXT,
            PANEL,
        );
    }

    fn home_status_card(
        &mut self,
        position: Point,
        label: &str,
        value: &str,
        active: bool,
        warning: bool,
    ) {
        self.card(position, Size::new(105, 62), false);
        self.dot(position + Point::new(10, 14), active, warning);
        self.text(label, position + Point::new(24, 9), FONT_6X10, MUTED);
        self.text(
            value,
            position + Point::new(10, 37),
            FONT_6X10,
            if warning { WARNING } else { TEXT },
        );
    }

    fn menu_card(&mut self, position: Point, title: &str, subtitle: &str, selected: bool) {
        self.card(position, Size::new(217, 42), selected);
        self.text(
            title,
            position + Point::new(14, 5),
            FONT_7X13_BOLD,
            if selected { BG } else { TEXT },
        );
        self.text(
            subtitle,
            position + Point::new(14, 23),
            FONT_6X10,
            if selected { PANEL } else { MUTED },
        );
    }

    fn menu_action_card(&mut self, position: Point, title: &str, subtitle: &str, selected: bool) {
        self.card(position, Size::new(444, 42), selected);
        self.text(
            title,
            position + Point::new(14, 5),
            FONT_7X13_BOLD,
            if selected { BG } else { TEXT },
        );
        self.text(
            subtitle,
            position + Point::new(210, 14),
            FONT_6X10,
            if selected { PANEL } else { MUTED },
        );
    }

    fn large_value(&mut self, position: Point, value: Option<f32>, unit: &str) {
        self.text("MESURE ACTUELLE", position, FONT_6X10, MUTED);
        self.update_large_value(position, value, unit);
    }

    fn update_large_value(&mut self, position: Point, value: Option<f32>, unit: &str) {
        let mut value_text: String<20> = String::new();
        match value {
            Some(v) => write!(&mut value_text, "{:5.2} {}", v, unit).ok(),
            None => write!(&mut value_text, "--.-- {}", unit).ok(),
        };
        self.text_bg(
            value_text.as_str(),
            position + Point::new(0, 31),
            FONT_10X20,
            TEXT,
            PANEL,
        );
    }

    fn target_panel(&mut self, position: Point, target: f32, unit: &str, decimals: usize) {
        self.card(position, Size::new(188, 108), false);
        self.text("CONSIGNE", position + Point::new(15, 14), FONT_6X10, MUTED);
        let mut target_text: String<20> = String::new();
        if decimals == 0 {
            write!(&mut target_text, "{:.0} {}", target, unit).ok();
        } else {
            write!(&mut target_text, "{:.2} {}", target, unit).ok();
        }
        self.centered(
            target_text.as_str(),
            position + Point::new(94, 61),
            FONT_10X20,
            TEXT,
        );
        self.centered(
            "<  ENCODEUR  >",
            position + Point::new(94, 91),
            FONT_6X10,
            ACCENT,
        );
    }

    fn update_target_value(&mut self, position: Point, target: f32, unit: &str, decimals: usize) {
        let mut target_text: String<20> = String::new();
        if decimals == 0 {
            write!(&mut target_text, "{:3.0} {}", target, unit).ok();
        } else {
            write!(&mut target_text, "{:4.2} {}", target, unit).ok();
        }
        self.centered_bg(
            target_text.as_str(),
            position + Point::new(94, 61),
            FONT_10X20,
            TEXT,
            PANEL,
        );
    }

    fn duty_panel(&mut self, position: Point, label: &str, duty: u8) {
        self.card(position, Size::new(432, 63), false);
        self.text(label, position + Point::new(14, 10), FONT_6X10, MUTED);
        let _ = Rectangle::new(position + Point::new(14, 35), Size::new(356, 10))
            .into_styled(PrimitiveStyle::with_fill(BORDER))
            .draw(&mut self.display);
        let width = 356 * duty.min(100) as u32 / 100;
        if width > 0 {
            let _ = Rectangle::new(position + Point::new(14, 35), Size::new(width, 10))
                .into_styled(PrimitiveStyle::with_fill(ACCENT))
                .draw(&mut self.display);
        }
        let mut duty_text: String<8> = String::new();
        write!(&mut duty_text, "{}%", duty).ok();
        self.centered(
            duty_text.as_str(),
            position + Point::new(405, 43),
            FONT_7X13_BOLD,
            TEXT,
        );
    }

    fn update_duty(&mut self, position: Point, duty: u8) {
        let _ = Rectangle::new(position + Point::new(14, 35), Size::new(356, 10))
            .into_styled(PrimitiveStyle::with_fill(BORDER))
            .draw(&mut self.display);
        let width = 356 * duty.min(100) as u32 / 100;
        if width > 0 {
            let _ = Rectangle::new(position + Point::new(14, 35), Size::new(width, 10))
                .into_styled(PrimitiveStyle::with_fill(ACCENT))
                .draw(&mut self.display);
        }
        let mut duty_text: String<8> = String::new();
        write!(&mut duty_text, "{:3}%", duty).ok();
        self.centered_bg(
            duty_text.as_str(),
            position + Point::new(405, 43),
            FONT_7X13_BOLD,
            TEXT,
            PANEL,
        );
    }

    fn status_wide(
        &mut self,
        position: Point,
        label: &str,
        value: &str,
        active: bool,
        warning: bool,
    ) {
        self.card(position, Size::new(217, 43), false);
        self.dot(position + Point::new(15, 15), active, warning);
        self.text(label, position + Point::new(34, 6), FONT_6X10, MUTED);
        self.text(
            value,
            position + Point::new(34, 23),
            FONT_6X10,
            if warning { WARNING } else { TEXT },
        );
    }

    fn card(&mut self, position: Point, size: Size, selected: bool) {
        let card_style = PrimitiveStyleBuilder::new()
            .fill_color(if selected { ACCENT } else { PANEL })
            .stroke_color(if selected { ACCENT } else { BORDER })
            .stroke_width(1)
            .build();
        let _ =
            RoundedRectangle::with_equal_corners(Rectangle::new(position, size), Size::new(12, 12))
                .into_styled(card_style)
                .draw(&mut self.display);
    }

    fn badge(&mut self, position: Point, text: &str, enabled: bool) {
        let color = if enabled { SAGE } else { BORDER };
        let size = Size::new(77, 24);
        let _ =
            RoundedRectangle::with_equal_corners(Rectangle::new(position, size), Size::new(12, 12))
                .into_styled(PrimitiveStyle::with_fill(color))
                .draw(&mut self.display);
        // La coordonnée Y de Text est la ligne de base, pas le haut du texte.
        self.centered(
            text,
            position + Point::new(size.width as i32 / 2, size.height as i32 / 2 + 4),
            FONT_6X10,
            if enabled { PANEL } else { TEXT },
        );
    }

    fn button(&mut self, position: Point, size: Size, text: &str) {
        let _ =
            RoundedRectangle::with_equal_corners(Rectangle::new(position, size), Size::new(18, 18))
                .into_styled(
                    PrimitiveStyleBuilder::new()
                        .fill_color(ACCENT)
                        .stroke_color(PANEL)
                        .stroke_width(1)
                        .build(),
                )
                .draw(&mut self.display);
        self.centered(
            text,
            position + Point::new(size.width as i32 / 2, size.height as i32 / 2 + 5),
            FONT_7X13_BOLD,
            PANEL,
        );
    }

    fn dot(&mut self, position: Point, active: bool, warning: bool) {
        let color = if warning {
            WARNING
        } else if active {
            SAGE
        } else {
            MUTED
        };
        let _ = Circle::new(position, 9)
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(&mut self.display);
    }

    fn separator(&mut self, y: i32) {
        let _ = Line::new(Point::new(18, y), Point::new(462, y))
            .into_styled(PrimitiveStyle::with_stroke(BORDER, 1))
            .draw(&mut self.display);
    }

    fn footer(&mut self, text: &str) {
        self.separator(292);
        self.centered(text, Point::new(240, 307), FONT_6X10, MUTED);
    }

    fn text(&mut self, text: &str, position: Point, font: MonoFont<'static>, color: Rgb565) {
        let _ = Text::with_baseline(
            text,
            position,
            MonoTextStyle::new(&font, color),
            Baseline::Top,
        )
        .draw(&mut self.display);
    }

    fn centered(&mut self, text: &str, position: Point, font: MonoFont<'static>, color: Rgb565) {
        let _ = Text::with_alignment(
            text,
            position,
            MonoTextStyle::new(&font, color),
            Alignment::Center,
        )
        .draw(&mut self.display);
    }

    fn text_bg(
        &mut self,
        text: &str,
        position: Point,
        font: MonoFont<'static>,
        color: Rgb565,
        background: Rgb565,
    ) {
        let text_style = MonoTextStyleBuilder::new()
            .font(&font)
            .text_color(color)
            .background_color(background)
            .build();
        let _ =
            Text::with_baseline(text, position, text_style, Baseline::Top).draw(&mut self.display);
    }

    fn centered_bg(
        &mut self,
        text: &str,
        position: Point,
        font: MonoFont<'static>,
        color: Rgb565,
        background: Rgb565,
    ) {
        let text_style = MonoTextStyleBuilder::new()
            .font(&font)
            .text_color(color)
            .background_color(background)
            .build();
        let _ = Text::with_alignment(text, position, text_style, Alignment::Center)
            .draw(&mut self.display);
    }
}

const BG: Rgb565 = Rgb565::new(27, 55, 27);
const PANEL: Rgb565 = Rgb565::new(31, 63, 31);
const BORDER: Rgb565 = Rgb565::new(20, 41, 20);
const TEXT: Rgb565 = Rgb565::new(1, 2, 1);
const MUTED: Rgb565 = Rgb565::new(10, 21, 10);
const ACCENT: Rgb565 = Rgb565::new(1, 2, 1);
const SAGE: Rgb565 = Rgb565::new(5, 37, 10);
const WARNING: Rgb565 = Rgb565::new(26, 10, 7);

fn value_changed(previous: Option<f32>, current: Option<f32>) -> bool {
    match (previous, current) {
        (Some(a), Some(b)) => (a * 100.0) as i32 != (b * 100.0) as i32,
        (None, None) => false,
        _ => true,
    }
}

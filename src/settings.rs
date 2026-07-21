use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_storage::{FlashStorage, FlashStorageError};

use crate::{
    automation::{EXTRACTION_TARGET_MAX_BAR, EXTRACTION_TARGET_MIN_BAR},
    group_heater::{TARGET_MAX_C, TARGET_MIN_C},
    heater::{TARGET_MAX_BAR, TARGET_MIN_BAR},
};

const SETTINGS_MAGIC: u32 = 0x4D49_4C41; // "MILA"
const SETTINGS_VERSION: u16 = 1;
const RECORD_SIZE: usize = 32;
const CRC_OFFSET: usize = 28;

// La table de partitions actuelle réserve 0x9000..0xF000 pour les données
// NVS. Le firmware bare-metal n'utilise pas NVS : les six secteurs servent
// donc de journal circulaire de réglages.
const STORAGE_START: u32 = 0x9000;
const SECTOR_SIZE: u32 = 0x1000;
const SLOT_COUNT: usize = 6;

#[derive(Clone, Copy)]
pub struct MachineSettings {
    pub steam_target_bar: f32,
    pub extraction_target_bar: f32,
    pub group_target_c: f32,
    pub startup_sound_enabled: bool,
    pub click_sound_enabled: bool,
    pub rotation_sound_enabled: bool,
    pub tank_alarm_sound_enabled: bool,
}

pub struct SettingsStore {
    flash: FlashStorage,
    current_slot: Option<usize>,
    sequence: u32,
}

impl SettingsStore {
    pub fn new() -> Self {
        Self {
            flash: FlashStorage::new(),
            current_slot: None,
            sequence: 0,
        }
    }

    pub fn load(&mut self) -> Result<Option<MachineSettings>, FlashStorageError> {
        let mut newest: Option<(usize, u32, MachineSettings)> = None;

        for slot in 0..SLOT_COUNT {
            let mut record = [0_u8; RECORD_SIZE];
            self.flash.read(slot_address(slot), &mut record)?;
            let Some((sequence, settings)) = decode_record(&record) else {
                continue;
            };

            let is_newer = newest
                .as_ref()
                .map(|(_, current, _)| sequence_is_newer(sequence, *current))
                .unwrap_or(true);
            if is_newer {
                newest = Some((slot, sequence, settings));
            }
        }

        if let Some((slot, sequence, settings)) = newest {
            self.current_slot = Some(slot);
            self.sequence = sequence;
            Ok(Some(settings))
        } else {
            Ok(None)
        }
    }

    pub fn save(&mut self, settings: MachineSettings) -> Result<(), FlashStorageError> {
        let next_slot = self
            .current_slot
            .map(|slot| (slot + 1) % SLOT_COUNT)
            .unwrap_or(0);
        let next_sequence = self.sequence.wrapping_add(1);
        let address = slot_address(next_slot);
        let record = encode_record(settings, next_sequence);

        self.flash.erase(address, address + SECTOR_SIZE)?;
        self.flash.write(address, &record)?;

        // Relire l'enregistrement avant de le considérer comme valide en RAM.
        let mut verification = [0_u8; RECORD_SIZE];
        self.flash.read(address, &mut verification)?;
        if decode_record(&verification).is_none() {
            return Err(FlashStorageError::Other(-1));
        }

        self.current_slot = Some(next_slot);
        self.sequence = next_sequence;
        Ok(())
    }
}

fn slot_address(slot: usize) -> u32 {
    STORAGE_START + slot as u32 * SECTOR_SIZE
}

fn encode_record(settings: MachineSettings, sequence: u32) -> [u8; RECORD_SIZE] {
    let mut record = [0_u8; RECORD_SIZE];
    record[0..4].copy_from_slice(&SETTINGS_MAGIC.to_le_bytes());
    record[4..6].copy_from_slice(&SETTINGS_VERSION.to_le_bytes());
    record[6..8].copy_from_slice(&(RECORD_SIZE as u16).to_le_bytes());
    record[8..12].copy_from_slice(&sequence.to_le_bytes());
    record[12..14].copy_from_slice(&to_scaled_u16(settings.steam_target_bar, 100.0).to_le_bytes());
    record[14..16]
        .copy_from_slice(&to_scaled_u16(settings.extraction_target_bar, 100.0).to_le_bytes());
    record[16..18].copy_from_slice(&to_scaled_u16(settings.group_target_c, 10.0).to_le_bytes());
    record[18] = (settings.startup_sound_enabled as u8)
        | ((settings.click_sound_enabled as u8) << 1)
        | ((settings.rotation_sound_enabled as u8) << 2)
        | ((settings.tank_alarm_sound_enabled as u8) << 3);
    let crc = crc32(&record[..CRC_OFFSET]);
    record[CRC_OFFSET..RECORD_SIZE].copy_from_slice(&crc.to_le_bytes());
    record
}

fn decode_record(record: &[u8; RECORD_SIZE]) -> Option<(u32, MachineSettings)> {
    if read_u32(record, 0) != SETTINGS_MAGIC
        || read_u16(record, 4) != SETTINGS_VERSION
        || read_u16(record, 6) as usize != RECORD_SIZE
        || read_u32(record, CRC_OFFSET) != crc32(&record[..CRC_OFFSET])
    {
        return None;
    }

    let steam_target_bar = read_u16(record, 12) as f32 / 100.0;
    let extraction_target_bar = read_u16(record, 14) as f32 / 100.0;
    let group_target_c = read_u16(record, 16) as f32 / 10.0;
    if !(TARGET_MIN_BAR..=TARGET_MAX_BAR).contains(&steam_target_bar)
        || !(EXTRACTION_TARGET_MIN_BAR..=EXTRACTION_TARGET_MAX_BAR).contains(&extraction_target_bar)
        || !(TARGET_MIN_C..=TARGET_MAX_C).contains(&group_target_c)
    {
        return None;
    }

    let flags = record[18];
    Some((
        read_u32(record, 8),
        MachineSettings {
            steam_target_bar,
            extraction_target_bar,
            group_target_c,
            startup_sound_enabled: flags & 0x01 != 0,
            click_sound_enabled: flags & 0x02 != 0,
            rotation_sound_enabled: flags & 0x04 != 0,
            tank_alarm_sound_enabled: flags & 0x08 != 0,
        },
    ))
}

fn read_u16(record: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([record[offset], record[offset + 1]])
}

fn read_u32(record: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        record[offset],
        record[offset + 1],
        record[offset + 2],
        record[offset + 3],
    ])
}

fn to_scaled_u16(value: f32, scale: f32) -> u16 {
    (value * scale + 0.5) as u16
}

fn sequence_is_newer(candidate: u32, current: u32) -> bool {
    candidate != current && candidate.wrapping_sub(current) < 0x8000_0000
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for byte in bytes {
        crc ^= *byte as u32;
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

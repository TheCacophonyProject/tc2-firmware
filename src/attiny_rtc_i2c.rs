use crate::device_config::get_datetime_utc;
use crate::formatted_time::FormattedNZTime;
use crate::re_exports::bsp::pac::I2C1;
use crate::re_exports::log::{debug, error, info, warn};
use crate::synced_date_time::SyncedDateTime;
use byteorder::{BigEndian, ByteOrder};
use chrono::{Datelike, Months, NaiveDate, NaiveDateTime, NaiveTime, Timelike, Utc};
use core::cmp::PartialEq;
use crc::{Algorithm, Crc};

use crate::constants::EXPECTED_ATTINY_FIRMWARE_VERSION;
use crate::re_exports::bsp::hal::gpio::bank0::{Gpio3, Gpio6, Gpio7};
use crate::re_exports::bsp::hal::gpio::{
    FunctionI2C, FunctionSio, Pin, PullNone, PullUp, SioInput, SioOutput,
};
use crate::re_exports::bsp::hal::{I2C, Timer};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{Error, ErrorKind, I2c};

pub type I2CConfig = I2C<
    I2C1,
    (
        Pin<Gpio6, FunctionI2C, PullUp>,
        Pin<Gpio7, FunctionI2C, PullUp>,
    ),
>;

type I2cUnlockedPin = Pin<Gpio3, FunctionSio<SioInput>, PullUp>;
type I2cLockedPin = Pin<Gpio3, FunctionSio<SioOutput>, PullNone>;

// Attiny + RTC comms
// NOTE: Early on in development we got strange errors when the raspberry pi was accessing the
//  attiny-provided i2c interface at the same time as we wanted to.  The hacky?/ingenious?
//  solution was to allocate a gpio pin that would determine who has the 'lock' on the i2c bus.
//  This is handled by this `SharedI2C` abstraction which mediates comms with the attiny/RTC.
pub struct MainI2C {
    unlocked_pin: Option<I2cUnlockedPin>,
    i2c: I2CConfig,
    delay: Timer,
}

pub mod tc2_agent_state {
    pub const _NOT_READY: u8 = 0x00;
    pub const READY: u8 = 1 << 1;
    pub const RECORDING: u8 = 1 << 2;
    pub const SHORT_TEST_RECORDING: u8 = 1 << 3;
    pub const AUDIO_MODE: u8 = 1 << 4;
    pub const OFFLOAD: u8 = 1 << 5;
    pub const THERMAL_MODE: u8 = 1 << 6;
    pub const LONG_TEST_RECORDING: u8 = 1 << 7;
}

#[derive(Default, Copy, Clone)]
pub struct Tc2AgentState(u8);

#[cfg(feature = "std")]
impl core::fmt::Display for Tc2AgentState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Tc2AgentState(0b{:08b})", self.0)
    }
}

impl From<u8> for Tc2AgentState {
    fn from(value: u8) -> Self {
        Tc2AgentState(value)
    }
}

impl From<Tc2AgentState> for u8 {
    fn from(value: Tc2AgentState) -> Self {
        value.0
    }
}

impl Tc2AgentState {
    pub(crate) fn flag_is_set(self, flag: u8) -> bool {
        self.0 & flag != 0
    }

    pub fn set_flag(&mut self, flag: u8) {
        self.0 |= flag;
    }

    pub fn unset_flag(&mut self, flag: u8) {
        self.0 &= !flag;
    }

    pub fn is_not_ready(self) -> bool {
        self.0 == 0
    }

    pub fn is_ready(self) -> bool {
        self.flag_is_set(tc2_agent_state::READY)
    }

    pub fn recording_in_progress(self) -> bool {
        self.flag_is_set(tc2_agent_state::RECORDING)
    }

    pub fn test_audio_recording_requested(self) -> bool {
        self.short_test_audio_recording_requested() || self.long_test_audio_recording_requested()
    }

    pub fn test_thermal_recording_requested(self) -> bool {
        self.short_test_thermal_recording_requested()
            || self.long_test_thermal_recording_requested()
    }

    pub fn short_test_audio_recording_requested(self) -> bool {
        self.requested_audio_mode() && self.flag_is_set(tc2_agent_state::SHORT_TEST_RECORDING)
    }

    pub fn long_test_audio_recording_requested(self) -> bool {
        self.requested_audio_mode() && self.flag_is_set(tc2_agent_state::LONG_TEST_RECORDING)
    }

    pub fn short_test_thermal_recording_requested(self) -> bool {
        self.requested_thermal_mode() && self.flag_is_set(tc2_agent_state::SHORT_TEST_RECORDING)
    }

    pub fn long_test_thermal_recording_requested(self) -> bool {
        self.requested_thermal_mode() && self.flag_is_set(tc2_agent_state::LONG_TEST_RECORDING)
    }

    pub fn test_recording_requested(self) -> bool {
        self.test_audio_recording_requested() || self.test_thermal_recording_requested()
    }

    pub fn is_offloading_files(self) -> bool {
        self.flag_is_set(tc2_agent_state::OFFLOAD)
    }

    pub fn requested_thermal_mode(self) -> bool {
        self.flag_is_set(tc2_agent_state::THERMAL_MODE)
    }
    pub fn requested_audio_mode(self) -> bool {
        self.flag_is_set(tc2_agent_state::AUDIO_MODE)
    }
}

#[repr(u8)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum AlarmMode {
    Audio = 0,
    Thermal = 1,
}

#[cfg(feature = "std")]
impl core::fmt::Display for AlarmMode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl TryFrom<u8> for AlarmMode {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(AlarmMode::Audio),
            1 => Ok(AlarmMode::Thermal),
            _ => Err("invalid audio mode"),
        }
    }
}

#[repr(u8)]
#[derive(PartialEq, Clone, Copy)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
pub enum CameraState {
    PoweringOn = 0x00,
    PoweredOn = 0x01,
    PoweringOff = 0x02,
    PoweredOff = 0x03,
    PowerOnTimeout = 0x04,
    InvalidState = 0x05,
}

impl CameraState {
    pub fn pi_is_powered_on(self) -> bool {
        self == CameraState::PoweredOn
    }
    pub fn pi_is_powered_off(self) -> bool {
        self == CameraState::PoweredOff
    }

    pub fn pi_is_waking_or_awake(self) -> bool {
        self.pi_is_powering_on() || self.pi_is_powered_on()
    }

    pub fn pi_is_powering_on(self) -> bool {
        self == CameraState::PoweringOn
    }
}

impl From<u8> for CameraState {
    fn from(value: u8) -> Self {
        match value {
            0x00 => CameraState::PoweringOn,
            0x01 => CameraState::PoweredOn,
            0x02 => CameraState::PoweringOff,
            0x03 => CameraState::PoweredOff,
            0x04 => CameraState::PowerOnTimeout,
            0x05 => CameraState::InvalidState,
            x => {
                warn!("Invalid camera state {:x}", x);
                CameraState::InvalidState
            }
        }
    }
}

impl From<u8> for CameraConnectionState {
    fn from(value: u8) -> Self {
        match value {
            0x00 => CameraConnectionState::NoConnection,
            0x01 => CameraConnectionState::ConnectedToWifi,
            0x02 => CameraConnectionState::HostingHotspot,
            0x03 => CameraConnectionState::WifiSetup,
            0x04 => CameraConnectionState::HotspotSetup,
            x => {
                warn!("Invalid camera connection state {:x}", x);
                CameraConnectionState::NoConnection
            }
        }
    }
}

#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
#[derive(Copy, Clone, PartialEq)]
pub enum RecordingMode {
    Audio(RecordingRequestType),
    Thermal(RecordingRequestType),
    None,
}

impl RecordingMode {
    #[allow(dead_code)]
    pub fn record_audio(&self) -> bool {
        matches!(self, RecordingMode::Audio(_))
    }

    pub fn record_thermal(&self) -> bool {
        matches!(self, RecordingMode::Thermal(_))
    }
}

#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
pub struct RecordingTypeDetail {
    user_requested: bool,
    pub duration_seconds: u32,
}

#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
pub enum RecordingRequestType {
    Test(RecordingTypeDetail),
    Scheduled(RecordingTypeDetail),
}

impl RecordingRequestType {
    pub fn test_recording(duration_seconds: u32) -> Self {
        RecordingRequestType::Test(RecordingTypeDetail {
            user_requested: true,
            duration_seconds,
        })
    }

    pub fn scheduled_recording() -> Self {
        RecordingRequestType::Scheduled(RecordingTypeDetail {
            user_requested: false,
            duration_seconds: 60,
        })
    }

    pub fn is_user_requested(&self) -> bool {
        match self {
            RecordingRequestType::Test(RecordingTypeDetail { user_requested, .. })
            | RecordingRequestType::Scheduled(RecordingTypeDetail { user_requested, .. }) => {
                *user_requested
            }
        }
    }

    pub fn duration_seconds(&self) -> u32 {
        match self {
            RecordingRequestType::Test(RecordingTypeDetail {
                duration_seconds, ..
            })
            | RecordingRequestType::Scheduled(RecordingTypeDetail {
                duration_seconds, ..
            }) => *duration_seconds,
        }
    }
}

static I2C_ATTINY_CRC_MISMATCH_ERROR: &str = "CRC mismatch";
static I2C_ATTINY_WRITE_FAILED: &str = "Attiny write failed";
static I2C_UNKNOWN_ERROR: &str = "i2c unknown error";

fn map_i2c_err(e: ErrorKind) -> &'static str {
    match e {
        ErrorKind::Bus => "i2c bus error",
        ErrorKind::ArbitrationLoss => "i2c arbitration loss error",
        ErrorKind::NoAcknowledge(src) => match src {
            embedded_hal::i2c::NoAcknowledgeSource::Data => "i2c no acknowledge (data) error",
            embedded_hal::i2c::NoAcknowledgeSource::Address => "i2c no acknowledge (address) error",
            embedded_hal::i2c::NoAcknowledgeSource::Unknown => "i2c no acknowledge (unknown) error",
        },
        ErrorKind::Overrun => "i2c overrun error",
        ErrorKind::Other => "i2c other error",
        _ => I2C_UNKNOWN_ERROR,
    }
}

#[repr(u8)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
pub enum CameraConnectionState {
    NoConnection = 0x00,
    ConnectedToWifi = 0x01,
    HostingHotspot = 0x02,
    WifiSetup = 0x03,
    HotspotSetup = 0x04,
}

pub const ATTINY_ADDRESS: u8 = 0x25;
pub const RTC_ADDRESS: u8 = 0x51;
pub const ATTINY_REG_VERSION: u8 = 0x01;
pub const ATTINY_REG_CAMERA_STATE: u8 = 0x02;
pub const ATTINY_REG_CAMERA_CONNECTION: u8 = 0x03;
pub const ATTINY_REG_RP2040_PI_POWER_CTRL: u8 = 0x05;
pub const ATTINY_REG_KEEP_ALIVE: u8 = 0x0e;
// const REG_PI_WAKEUP: u8 = 0x06;
pub const ATTINY_REG_TC2_AGENT_STATE: u8 = 0x07;

pub const RTC_REG_ALARM_CONTROL: u8 = 0x01;
pub const RTC_REG_DATETIME_SECONDS: u8 = 0x02;
pub const RTC_REG_ALARM_MINUTES: u8 = 0x09;
const DEFAULT_MAX_I2C_ATTEMPTS: u8 = 100;

pub const CRC_AUG_CCITT: Algorithm<u16> = Algorithm {
    width: 16,
    poly: 0x1021,
    init: 0x1D0F,
    refin: false,
    refout: false,
    xorout: 0x0000,
    check: 0x0000,
    residue: 0x0000,
};

pub fn decode_bcd(input: u8) -> u8 {
    let digits: u8 = input & 0xf;
    let tens: u8 = (input >> 4) & 0x7;
    10 * tens + digits
}

/// Convert the decimal value to Binary Coded Decimal.
pub(crate) fn encode_bcd(input: u8) -> u8 {
    let digits: u8 = input % 10;
    let tens: u8 = input / 10;
    let tens = tens << 4;
    tens + digits
}

pub struct ScheduledAlarmTime {
    pub time: chrono::DateTime<Utc>,
    pub mode: AlarmMode,
    pub already_triggered: bool,
}

#[cfg(feature = "std")]
impl core::fmt::Display for ScheduledAlarmTime {
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            fmt,
            "ScheduledAlarmTime: {}, mode: {:?}, already_triggered: {}",
            FormattedNZTime(self.date_time()),
            self.mode,
            self.already_triggered
        )
    }
}

impl ScheduledAlarmTime {
    pub fn has_triggered(&self) -> bool {
        self.already_triggered
    }
    pub fn date_time(&self) -> chrono::DateTime<Utc> {
        self.time
    }
    pub fn is_audio_alarm(&self) -> bool {
        self.mode == AlarmMode::Audio
    }
    pub fn is_thermal_alarm(&self) -> bool {
        !self.is_audio_alarm()
    }
}

#[cfg(feature = "no-std")]
impl defmt::Format for ScheduledAlarmTime {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "ScheduledAlarmTime: {}, mode: {}, already_triggered: {}",
            FormattedNZTime(self.date_time()),
            self.mode,
            self.already_triggered
        );
    }
}

impl MainI2C {
    pub fn new(
        i2c_config: I2CConfig,
        unlocked_pin: I2cUnlockedPin,
        delay: Timer,
    ) -> Result<MainI2C, &'static str> {
        let mut i2c = MainI2C {
            unlocked_pin: Some(unlocked_pin),
            i2c: i2c_config,
            delay,
        };
        let version = i2c.get_attiny_firmware_version()?;
        if version != EXPECTED_ATTINY_FIRMWARE_VERSION {
            error!(
                "Mismatched Attiny firmware version – expected {}, got {}",
                EXPECTED_ATTINY_FIRMWARE_VERSION, version
            );
            return Err("Mismatched Attiny firmware version");
        }
        Ok(i2c)
    }

    pub fn get_scheduled_alarm(&mut self, now: &SyncedDateTime) -> Option<ScheduledAlarmTime> {
        self.with_i2c_retrying(|i2c| {
            let mut data = [0];
            i2c.write_read(RTC_ADDRESS, &[RTC_REG_ALARM_CONTROL], &mut data)
                .map_err(|e| map_i2c_err(e.kind()))?;
            // Alarm interrupt active, but alarm flag (0b0000_1000) not triggered yet
            let alarm_not_active = data[0] & 0b0000_0010 == 0;
            if alarm_not_active {
                return Err("no alarm scheduled");
            }
            let alarm_active_but_already_triggered = data[0] & 0b0000_1010 == 0b0000_1010;

            let mut data = [0, 0, 0, 0];
            let minute_alarm: u8 = RTC_REG_ALARM_MINUTES;
            i2c.write_read(RTC_ADDRESS, &[minute_alarm], &mut data)
                .map_err(|e| map_i2c_err(e.kind()))?;
            let minutes = decode_bcd(data[0] & 0b0111_1111);
            if data[0] & 0b1000_0000 != 0 {
                return Err("minutes not enabled");
            }
            let hours = decode_bcd(data[1] & 0b0011_1111);
            if data[1] & 0b1000_0000 != 0 {
                return Err("hours not enabled");
            }
            let day = decode_bcd(data[2] & 0b0011_1111);
            if data[2] & 0b1000_0000 != 0 {
                return Err("day not enabled");
            }
            let weekday = decode_bcd(data[3] & 0b0000_0111);
            let alarm_mode = if weekday == 0 {
                AlarmMode::Thermal
            } else {
                AlarmMode::Audio
            };
            if data[3] & 0b1000_0000 == 0 {
                return Err("weekday enabled");
            }
            let mut naive_date = NaiveDate::from_ymd_opt(
                now.date_time().year(),
                now.date_time().month(),
                u32::from(day),
            )
            .unwrap();
            let naive_time =
                NaiveTime::from_hms_opt(u32::from(hours), u32::from(minutes), 0).unwrap();
            if alarm_active_but_already_triggered {
                // Time is in the past, make sure we get the month correct
                if now.date_time().day() < u32::from(day) {
                    naive_date = naive_date.checked_sub_months(Months::new(1)).unwrap();
                }
            } else {
                // Time is in the future, check if day < current day, and if so, increment month
                if u32::from(day) < now.date_time().day() {
                    naive_date = naive_date.checked_add_months(Months::new(1)).unwrap();
                }
            }
            let utc_datetime = NaiveDateTime::new(naive_date, naive_time).and_utc();
            Ok(ScheduledAlarmTime {
                time: utc_datetime,
                mode: alarm_mode,
                already_triggered: alarm_active_but_already_triggered,
            })
        })
        .ok()
    }

    fn try_attiny_write_command(&mut self, command: u8, value: u8) -> Result<(), &str> {
        let request_crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[command, value]);
        self.with_i2c_retrying(|i2c| {
            let mut payload = [command, value, 0, 0];
            BigEndian::write_u16(&mut payload[2..=3], request_crc);
            // Write the value
            let result = i2c
                .write(ATTINY_ADDRESS, &payload)
                .map_err(|e| map_i2c_err(e.kind()));
            while i2c.tx_fifo_used() != 0 {}
            if result.is_ok() {
                // Verify that it actually got written:
                payload[0] = 0;
                let mut request = [command, 0x00, 0x00];
                let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&request[0..1]);
                BigEndian::write_u16(&mut request[1..=2], crc);
                let result = i2c
                    .write_read(ATTINY_ADDRESS, &request, &mut payload[0..=2])
                    .map_err(|e| map_i2c_err(e.kind()));
                return if result.is_ok() {
                    let returned_value = payload[0];
                    let returned_crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[returned_value]);
                    if BigEndian::read_u16(&payload[1..=2]) == returned_crc {
                        if returned_value == value {
                            Ok(())
                        } else {
                            warn!(
                                "Failed writing command {} with value {} to attiny, got {}",
                                command, value, returned_value
                            );
                            Err(I2C_ATTINY_WRITE_FAILED)
                        }
                    } else {
                        Err(I2C_ATTINY_CRC_MISMATCH_ERROR)
                    }
                } else {
                    result
                };
            }
            result
        })
    }

    #[allow(dead_code)]
    pub fn check_device_present(&mut self, addr: u8) -> Result<bool, &str> {
        self.with_i2c(
            |i2c| {
                let payload = &mut [0];
                i2c.write_read(addr, &[0], payload)
                    .map(|()| true)
                    .map_err(|e| map_i2c_err(e.kind()))
            },
            Some(0),
        )
    }

    #[allow(dead_code)]
    fn attiny_write_read_command(
        &mut self,
        command: u8,
        value: Option<u8>,
        payload: &mut [u8; 3],
    ) -> Result<(), &str> {
        self.with_i2c_retrying(|i2c| {
            if let Some(v) = value {
                let mut request = [command, v, 0x00, 0x00];
                let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&request[0..=1]);
                BigEndian::write_u16(&mut request[2..=3], crc);
                i2c.write_read(ATTINY_ADDRESS, &request, payload)
                    .map_err(|e| map_i2c_err(e.kind()))
            } else {
                let mut request = [command, 0x00, 0x00];
                let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&request[0..1]);
                BigEndian::write_u16(&mut request[1..=2], crc);
                i2c.write_read(ATTINY_ADDRESS, &request, payload)
                    .map_err(|e| map_i2c_err(e.kind()))
            }
        })
    }

    fn try_attiny_read_command(&mut self, command: u8) -> Result<u8, &'static str> {
        let mut payload = [0u8; 3];
        let request_crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[command]);
        self.with_i2c_retrying(|i2c| {
            payload[0] = 0;
            let mut request = [command, 0x00, 0x00];
            BigEndian::write_u16(&mut request[1..=2], request_crc);
            let result = i2c
                .write_read(ATTINY_ADDRESS, &request, &mut payload)
                .map(|()| 0u8)
                .map_err(|e| map_i2c_err(e.kind()));
            if result.is_ok() {
                let value = payload[0];
                let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[value]);
                return if BigEndian::read_u16(&payload[1..=2]) == crc {
                    Ok(value)
                } else {
                    Err(I2C_ATTINY_CRC_MISMATCH_ERROR)
                };
            }
            result
        })
    }

    pub fn get_attiny_firmware_version(&mut self) -> Result<u8, &'static str> {
        self.try_attiny_read_command(ATTINY_REG_VERSION)
    }

    pub fn tell_pi_to_shutdown(&mut self) -> Result<(), &str> {
        self.try_attiny_write_command(ATTINY_REG_RP2040_PI_POWER_CTRL, 0x00)
    }

    pub fn advise_raspberry_pi_it_may_shutdown(&mut self) -> Result<(), &str> {
        let result = self.tell_pi_to_shutdown();
        if result.is_err() {
            error!("Error sending power-down advice to raspberry pi");
        } else {
            info!("Sent power-down advice to raspberry pi");
        }
        result
    }

    pub fn tell_pi_to_wakeup(&mut self) -> Result<(), &str> {
        self.try_attiny_write_command(ATTINY_REG_RP2040_PI_POWER_CTRL, 0x01)
    }

    pub fn get_is_recording(&mut self) -> Result<bool, &str> {
        let state = self.get_tc2_agent_state()?;
        Ok(state.recording_in_progress())
    }

    pub fn attiny_keep_alive(&mut self) -> Result<(), &str> {
        self.try_attiny_write_command(ATTINY_REG_KEEP_ALIVE, 0x01)
    }

    fn set_recording_flag(&mut self, is_recording: bool) -> Result<(), &str> {
        let mut state = self.get_tc2_agent_state()?;
        if is_recording {
            state.set_flag(tc2_agent_state::RECORDING);
        } else {
            state.unset_flag(tc2_agent_state::RECORDING);
        }
        self.try_attiny_write_command(ATTINY_REG_TC2_AGENT_STATE, state.into())
    }

    fn set_offload_flag(&mut self, is_offloading: bool) -> Result<(), &str> {
        let mut state = self.get_tc2_agent_state()?;
        if is_offloading {
            state.set_flag(tc2_agent_state::OFFLOAD);
        } else {
            state.unset_flag(tc2_agent_state::OFFLOAD);
        }
        self.try_attiny_write_command(ATTINY_REG_TC2_AGENT_STATE, state.into())
    }

    pub fn started_recording(&mut self) -> Result<(), &str> {
        self.set_recording_flag(true)
    }

    pub fn stopped_recording(&mut self) -> Result<(), &str> {
        self.set_recording_flag(false)
    }

    pub fn begin_offload(&mut self) -> Result<(), &str> {
        self.set_offload_flag(true)
    }

    pub fn end_offload(&mut self) -> Result<(), &str> {
        self.set_offload_flag(false)
    }

    pub fn offload_flag_is_set(&mut self) -> Result<bool, &str> {
        Ok(self.get_tc2_agent_state()?.is_offloading_files())
    }

    pub fn check_if_pi_is_awake_and_tc2_agent_is_ready(&mut self) -> Result<bool, &str> {
        if self.get_camera_state()?.pi_is_powered_on() {
            Ok(self.get_tc2_agent_state()?.is_ready())
        } else {
            Ok(false)
        }
    }

    pub fn get_tc2_agent_state(&mut self) -> Result<Tc2AgentState, &'static str> {
        let state = self.try_attiny_read_command(ATTINY_REG_TC2_AGENT_STATE)?;
        Ok(Tc2AgentState::from(state))
    }

    pub fn enter_thermal_mode(&mut self) -> Result<(), &str> {
        let mut state = self.get_tc2_agent_state()?;
        state.unset_flag(tc2_agent_state::AUDIO_MODE);
        state.set_flag(tc2_agent_state::THERMAL_MODE);
        self.try_attiny_write_command(ATTINY_REG_TC2_AGENT_STATE, state.into())
    }

    pub fn enter_audio_mode(&mut self) -> Result<(), &str> {
        let mut state = self.get_tc2_agent_state()?;
        state.set_flag(tc2_agent_state::AUDIO_MODE);
        state.unset_flag(tc2_agent_state::THERMAL_MODE);
        self.try_attiny_write_command(ATTINY_REG_TC2_AGENT_STATE, state.into())
    }

    pub fn tc2_agent_clear_mode_flags(&mut self) -> Result<(), &str> {
        let mut state = self.get_tc2_agent_state()?;
        state.unset_flag(tc2_agent_state::LONG_TEST_RECORDING);
        state.unset_flag(tc2_agent_state::SHORT_TEST_RECORDING);
        state.unset_flag(tc2_agent_state::THERMAL_MODE);
        state.unset_flag(tc2_agent_state::RECORDING);
        state.unset_flag(tc2_agent_state::AUDIO_MODE);
        self.try_attiny_write_command(ATTINY_REG_TC2_AGENT_STATE, state.into())
    }

    pub fn get_camera_state(&mut self) -> Result<CameraState, &'static str> {
        let state = self.try_attiny_read_command(ATTINY_REG_CAMERA_STATE)?;
        Ok(CameraState::from(state))
    }

    pub fn get_camera_connection_state(&mut self) -> Result<CameraConnectionState, &'static str> {
        let state = self.try_attiny_read_command(ATTINY_REG_CAMERA_CONNECTION)?;
        Ok(CameraConnectionState::from(state))
    }

    pub fn get_datetime(
        &mut self,
        timer: Timer,
        print: bool,
    ) -> Result<SyncedDateTime, &'static str> {
        self.with_i2c_retrying(|i2c| {
            let mut data = [0; 7];
            i2c.write_read(RTC_ADDRESS, &[RTC_REG_DATETIME_SECONDS], &mut data)
                .map_err(|e| map_i2c_err(e.kind()))?;

            if data[0] & 0b1000_0000 == 0b1000_0000 {
                // Time integrity compromised
                return Err("Time integrity compromised");
            }

            let year = decode_bcd(data[6]);
            let month = decode_bcd(data[5] & 0x1f);
            // let weekday = decode_bcd(data[4] & 0x07);
            let day = decode_bcd(data[3] & 0x3f);
            let hours = decode_bcd(data[2] & 0x3f);
            let minutes = decode_bcd(data[1] & 0x7f);
            let seconds = decode_bcd(data[0]);
            if year < 25 || day == 0 || day > 31 || month == 0 || month > 12 {
                Err("Invalid datetime output from RTC")
            } else {
                let synced_time = SyncedDateTime::new(
                    get_datetime_utc(year, month, day, hours, minutes, seconds),
                    timer,
                );
                if print {
                    debug!("Synced DateTime with RTC {}", synced_time);
                }

                Ok(synced_time)
            }
        })
    }

    pub fn set_wakeup_alarm(
        &mut self,
        wakeup_datetime_utc: &chrono::DateTime<Utc>,
        mode: AlarmMode,
        now: &SyncedDateTime,
    ) -> Result<(), &'static str> {
        #[allow(clippy::cast_possible_truncation)]
        let wake_hour = wakeup_datetime_utc.time().hour() as u8;
        #[allow(clippy::cast_possible_truncation)]
        let wake_min = wakeup_datetime_utc.time().minute() as u8;
        #[allow(clippy::cast_possible_truncation)]
        let wake_day = wakeup_datetime_utc.date_naive().day() as u8;

        debug!(
            "Set alarm in {} mins,  mode {}, Current time: {}, Next alarm: {}",
            (*wakeup_datetime_utc - now.date_time_utc).num_minutes(),
            mode,
            now,
            FormattedNZTime(*wakeup_datetime_utc)
        );

        self.set_wakeup_alarm_days_hours_mins(wake_day, wake_hour, wake_min, now, mode)
    }

    pub fn set_wakeup_alarm_days_hours_mins(
        &mut self,
        wake_day: u8,
        wake_hour: u8,
        wake_min: u8,
        now: &SyncedDateTime,
        alarm_mode: AlarmMode,
    ) -> Result<(), &'static str> {
        let result = self.with_i2c_retrying(|i2c| {
            let alarm_mins = encode_bcd(wake_min);
            let alarm_hours = encode_bcd(wake_hour);
            let alarm_days = encode_bcd(wake_day); // Not enabled
            // NOTE: Store alarm mode in unused weekdays field.
            let alarm_weekdays = if alarm_mode == AlarmMode::Thermal {
                0b1000_0000
            } else {
                0b1000_0001
            }; // Not enabled
            let payload = [
                RTC_REG_ALARM_MINUTES,
                alarm_mins,
                alarm_hours,
                alarm_days,
                alarm_weekdays,
            ];
            i2c.write(RTC_ADDRESS, &payload)
                .map_err(|e| map_i2c_err(e.kind()))?;
            while i2c.tx_fifo_used() != 0 {}
            i2c.write(RTC_ADDRESS, &[RTC_REG_ALARM_CONTROL, 0b0000_0010])
                .map_err(|e| map_i2c_err(e.kind()))?;
            while i2c.tx_fifo_used() != 0 {}
            Ok(())
        });
        if let Err(e) = result {
            error!("Failed to set wake alarm: {}", e);
            result
        } else {
            match self.get_scheduled_alarm(now) {
                Some(ScheduledAlarmTime {
                    time,
                    mode,
                    already_triggered,
                }) => {
                    if already_triggered {
                        Err("Alarm already triggered")
                    } else if time.minute() != u32::from(wake_min) {
                        Err("read mins didn't match set mins")
                    } else if time.hour() != u32::from(wake_hour) {
                        Err("read hours didn't match set hours")
                    } else if time.day() != u32::from(wake_day) {
                        Err("read days didn't match set days")
                    } else if alarm_mode != mode {
                        Err("alarm mode didn't match set alarm mode")
                    } else {
                        debug!("Got scheduled alarm.");
                        Ok(())
                    }
                }
                None => Err("No scheduled alarm set"),
            }
        }
    }

    fn take_i2c_lock(&mut self) -> Option<I2cLockedPin> {
        let mut lock_pin = self.unlocked_pin.take().unwrap();
        if lock_pin.is_high().unwrap_or(false) {
            let mut pin = lock_pin.reconfigure();
            pin.set_low().unwrap();
            Some(pin)
        } else {
            self.unlocked_pin = Some(lock_pin);
            None
        }
    }

    fn restore_i2c_lock_pin(&mut self, pin: I2cLockedPin) {
        self.unlocked_pin = Some(pin.reconfigure());
    }

    fn with_i2c<SUCCESS>(
        &mut self,
        mut func: impl FnMut(&mut I2CConfig) -> Result<SUCCESS, &'static str>,
        max_attempts: Option<u8>,
    ) -> Result<SUCCESS, &'static str> {
        let mut attempts = 0;
        let max_attempts = max_attempts.unwrap_or(DEFAULT_MAX_I2C_ATTEMPTS);
        loop {
            assert!(self.unlocked_pin.is_some(), "Should have unlocked pin");
            let result = if let Some(pin) = self.take_i2c_lock() {
                let result = func(&mut self.i2c);
                self.restore_i2c_lock_pin(pin);
                result
            } else {
                Err("Failed to get i2c lock")
            };
            if result.is_ok() {
                return result;
            }
            if attempts >= max_attempts {
                return result;
            }
            attempts += 1;
            self.delay.delay_us(500);
        }
    }

    fn with_i2c_retrying<SUCCESS>(
        &mut self,
        func: impl FnMut(&mut I2CConfig) -> Result<SUCCESS, &'static str>,
    ) -> Result<SUCCESS, &'static str> {
        self.with_i2c(func, None)
    }

    pub fn clear_and_disable_alarm(&mut self, now: &SyncedDateTime) -> Result<(), &str> {
        let result = self.with_i2c_retrying(|i2c| {
            // Clear the alarm flag and the alarm interrupt enable flag
            i2c.write(RTC_ADDRESS, &[RTC_REG_ALARM_CONTROL, 0])
                .map_err(|e| map_i2c_err(e.kind()))?;
            while i2c.tx_fifo_used() != 0 {}
            // Set all alarm parts (minutes, hours, etc) to disabled.
            let payload = [
                RTC_REG_ALARM_MINUTES,
                0b1000_0000,
                0b1000_0000,
                0b1000_0000,
                0b1000_0000,
            ];
            i2c.write(RTC_ADDRESS, &payload)
                .map_err(|e| map_i2c_err(e.kind()))?;
            while i2c.tx_fifo_used() != 0 {}
            Ok(())
        });
        if result.is_err() {
            result
        } else if self.get_scheduled_alarm(now).is_some() {
            Err("Failed to clear and disable alarm")
        } else {
            Ok(())
        }
    }

    pub fn tell_attiny_to_power_down_rp2040(&mut self) -> Result<(), &str> {
        self.try_attiny_write_command(ATTINY_REG_RP2040_PI_POWER_CTRL, 0x02)
    }

    fn try_read_eeprom_command(&mut self, command: u8, payload: &mut [u8]) -> Result<(), &str> {
        self.with_i2c_retrying(|i2c| {
            i2c.write_read(EEPROM_I2C_ADDRESS, &[command], payload)
                .map_err(|e| map_i2c_err(e.kind()))
        })
    }

    pub fn check_if_is_audio_device(&mut self) -> Result<bool, ()> {
        let page_length: usize = 16;
        assert!(EEPROM_LENGTH < usize::from(u8::MAX));
        let mut eeprom = Eeprom::from_bytes([0u8; EEPROM_LENGTH]);
        for (chunk_num, chunk) in eeprom.as_mut().chunks_mut(page_length).enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            if let Err(e) = self.try_read_eeprom_command(chunk_num as u8, chunk) {
                warn!("Couldn't read eeprom data: {}", e);
                return Err(());
            }
        }
        if !eeprom.has_valid_data() {
            return Err(());
        }
        Ok(eeprom.audio_only())
    }
}

const EEPROM_LENGTH: usize = 1 + 1 + 3 * 4 + 1 + 8 + 4 + 2;
pub const EEPROM_I2C_ADDRESS: u8 = 0x50;

pub struct Eeprom {
    inner: [u8; EEPROM_LENGTH],
}

impl Eeprom {
    pub fn from_bytes(bytes: [u8; EEPROM_LENGTH]) -> Self {
        Self { inner: bytes }
    }

    pub fn as_mut(&mut self) -> &mut [u8] {
        &mut self.inner
    }

    pub fn version(&self) -> u8 {
        self.inner[0]
    }

    pub fn hardware_version(&self) -> [u8; 3] {
        self.inner[1..4].try_into().unwrap()
    }

    pub fn audio_only(&self) -> bool {
        self.inner[13] != 0
    }

    pub fn crc_16(&self) -> u16 {
        BigEndian::read_u16(&self.inner[self.inner.len() - 2..])
    }

    pub fn has_valid_data(&self) -> bool {
        let has_data = self.inner.iter().any(|&x| x != 0xff);
        if !has_data {
            info!("No EEPROM data");
            false
        } else if self.version() != 0xca {
            debug!(
                "Incorrect first byte got {} should be {}",
                self.version(),
                0xca
            );
            false
        } else if !self.crc_is_valid() {
            false
        } else if self.hardware_version()[0] == 1 {
            info!("Need eeprom version 2");
            false
        } else {
            true
        }
    }

    pub fn crc_is_valid(&self) -> bool {
        let embedded_crc =
            Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&self.inner[..self.inner.len() - 2]);
        let calculated_crc = self.crc_16();
        let valid = embedded_crc == calculated_crc;
        if !valid {
            // FIXME?
            // warn!(
            //     "Eeprom CRC failed expected {} got {}",
            //     calculated_crc, embedded_crc
            // );
        }
        valid
    }
}

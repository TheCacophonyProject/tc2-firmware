use crate::EXPECTED_ATTINY_FIRMWARE_VERSION;
use crate::bsp::pac::I2C1;
use byteorder::{BigEndian, ByteOrder};
use chrono::{Timelike, Utc};
use cortex_m::delay::Delay;
use crc::{Algorithm, Crc};
use defmt::{Format, error, info, warn};
use embedded_hal::digital::InputPin;
use embedded_hal::i2c::{Error, ErrorKind, I2c};
use pcf8563::{Control, DateTime, PCF8563};
use rp2040_hal::I2C;
use rp2040_hal::gpio::bank0::{Gpio3, Gpio6, Gpio7};
use rp2040_hal::gpio::{FunctionI2C, FunctionSio, Pin, PullDown, PullUp, SioInput};

pub type I2CConfig = I2C<
    I2C1,
    (
        Pin<Gpio6, FunctionI2C, PullUp>,
        Pin<Gpio7, FunctionI2C, PullUp>,
    ),
>;

type I2cUnlockedPin = Pin<Gpio3, FunctionSio<SioInput>, PullDown>;
type I2cLockedPin = Pin<Gpio3, FunctionSio<SioInput>, PullUp>;

pub struct SharedI2C {
    unlocked_pin: Option<I2cUnlockedPin>,
    i2c: Option<I2CConfig>,
    rtc: Option<PCF8563<I2CConfig>>,
}

pub mod tc2_agent_state {
    pub const NOT_READY: u8 = 0x00;
    pub const READY: u8 = 1 << 1;
    pub const RECORDING: u8 = 1 << 2;
    pub const TEST_AUDIO_RECORDING: u8 = 1 << 3;
    pub const TAKE_AUDIO: u8 = 1 << 4;
    pub const OFFLOAD: u8 = 1 << 5;
    pub const THERMAL_MODE: u8 = 1 << 6;
    pub const LONG_AUDIO_RECORDING: u8 = 1 << 7;
}

#[repr(u8)]
#[derive(Format)]
enum CameraState {
    PoweringOn = 0x00,
    PoweredOn = 0x01,
    PoweringOff = 0x02,
    PoweredOff = 0x03,
    PowerOnTimeout = 0x04,
    InvalidState = 0x05,
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

#[derive(Format, Copy, Clone)]
pub struct RecordingTypeDetail {
    user_requested: bool,
    thermal_requested: bool,
    duration_seconds: usize,
}

#[derive(Format, Copy, Clone)]
pub enum RecordingMode {
    Audio(AudioRecordingType),
    Thermal,
}

impl RecordingMode {
    pub fn record_audio(&self) -> bool {
        match self {
            RecordingMode::Audio(_) => true,
            RecordingMode::Thermal => false,
        }
    }

    pub fn record_thermal(&self) -> bool {
        !self.record_audio()
    }
}

#[derive(Format, Copy, Clone)]
pub enum AudioRecordingType {
    Test(RecordingTypeDetail),
    Long(RecordingTypeDetail),
    Scheduled(RecordingTypeDetail),
    ThermalRequestedScheduled(RecordingTypeDetail),
}

impl AudioRecordingType {
    pub fn test_recording() -> Self {
        AudioRecordingType::Test(RecordingTypeDetail {
            user_requested: true,
            thermal_requested: false,
            duration_seconds: 10,
        })
    }

    pub fn long_recording() -> Self {
        AudioRecordingType::Long(RecordingTypeDetail {
            user_requested: true,
            thermal_requested: false,
            duration_seconds: 60 * 5,
        })
    }

    pub fn scheduled_recording() -> Self {
        AudioRecordingType::Scheduled(RecordingTypeDetail {
            user_requested: false,
            thermal_requested: false,
            duration_seconds: 60,
        })
    }

    pub fn thermal_scheduled_recording() -> Self {
        AudioRecordingType::ThermalRequestedScheduled(RecordingTypeDetail {
            user_requested: false,
            thermal_requested: true,
            duration_seconds: 60,
        })
    }

    pub fn is_user_requested(&self) -> bool {
        match self {
            AudioRecordingType::Test(RecordingTypeDetail { user_requested, .. })
            | AudioRecordingType::Long(RecordingTypeDetail { user_requested, .. })
            | AudioRecordingType::Scheduled(RecordingTypeDetail { user_requested, .. })
            | AudioRecordingType::ThermalRequestedScheduled(RecordingTypeDetail {
                user_requested,
                ..
            }) => *user_requested,
        }
    }

    pub fn is_scheduled(&self) -> bool {
        !self.is_user_requested()
    }

    pub fn is_thermal_requested(&self) -> bool {
        matches!(self, AudioRecordingType::ThermalRequestedScheduled(_))
    }

    pub fn duration_seconds(&self) -> usize {
        match self {
            AudioRecordingType::Test(RecordingTypeDetail {
                duration_seconds, ..
            })
            | AudioRecordingType::Long(RecordingTypeDetail {
                duration_seconds, ..
            })
            | AudioRecordingType::Scheduled(RecordingTypeDetail {
                duration_seconds, ..
            })
            | AudioRecordingType::ThermalRequestedScheduled(RecordingTypeDetail {
                duration_seconds,
                ..
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

fn map_rtc_err(e: pcf8563::Error<rp2040_hal::i2c::Error>) -> &'static str {
    match e {
        pcf8563::Error::I2C(e) => match e {
            rp2040_hal::i2c::Error::Abort(err_code) => {
                error!("rtc/i2c abort {}", err_code);
                "rtc/i2c abort error"
            }
            rp2040_hal::i2c::Error::InvalidReadBufferLength => {
                "rtc/i2c zero length read buffer passed"
            }
            rp2040_hal::i2c::Error::InvalidWriteBufferLength => {
                "rtc/i2c zero length write buffer passed"
            }
            rp2040_hal::i2c::Error::AddressOutOfRange(address) => {
                error!("rtc/i2c address out of range: {}", address);
                "rtc/i2c address out of range"
            }
            rp2040_hal::i2c::Error::AddressReserved(address) => {
                error!("rtc/i2c address reserved : {}", address);
                "rtc/i2c address reserved "
            }
            _ => I2C_UNKNOWN_ERROR,
        },
        pcf8563::Error::InvalidInputData => "rtc/i2c invalid input data",
    }
}

#[repr(u8)]
enum CameraConnectionState {
    NoConnection = 0x00,
    ConnectedToWifi = 0x01,
    HostingHotspot = 0x02,
}

const ATTINY_ADDRESS: u8 = 0x25;
const REG_VERSION: u8 = 0x01;
const REG_CAMERA_STATE: u8 = 0x02;
const REG_CAMERA_CONNECTION: u8 = 0x03;
const REG_RP2040_PI_POWER_CTRL: u8 = 0x05;
const REG_KEEP_ALIVE: u8 = 0x0e;
//const REG_PI_WAKEUP: u8 = 0x06;
const REG_TC2_AGENT_STATE: u8 = 0x07;
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
impl SharedI2C {
    pub fn new(
        i2c_config: I2CConfig,
        unlocked_pin: I2cUnlockedPin,
        delay: &mut Delay,
    ) -> Result<SharedI2C, &str> {
        let mut i2c = SharedI2C {
            unlocked_pin: Some(unlocked_pin),
            i2c: Some(i2c_config),
            rtc: None,
        };
        let version = i2c.get_attiny_firmware_version(delay)?;
        if version != EXPECTED_ATTINY_FIRMWARE_VERSION {
            error!(
                "Mismatched Attiny firmware version – expected {}, got {}",
                EXPECTED_ATTINY_FIRMWARE_VERSION, version
            );
            return Err("Mismatched Attiny firmware version");
        }
        Ok(i2c)
    }

    // Dead code
    pub fn free(&mut self) -> (I2CConfig, I2cUnlockedPin) {
        if let Some(unlocked_pin) = self.unlocked_pin.take() {
            if let Some(device) = self.rtc.take() {
                let dev = device.destroy();
                (dev, unlocked_pin)
            } else if let Some(device) = self.i2c.take() {
                (device, unlocked_pin)
            } else {
                unreachable!("Can't get here")
            }
        } else {
            unreachable!("Can't get here")
        }
    }

    pub fn get_scheduled_alarm_time(&mut self, delay: &mut Delay) -> (Option<u8>, Option<u8>) {
        let result = self.with_rtc(delay, |rtc| {
            let hours = rtc.get_alarm_hours().map_err(map_rtc_err)?;
            let mins = rtc.get_alarm_minutes().map_err(map_rtc_err)?;
            Ok::<(Option<u8>, Option<u8>), &str>((Some(hours), Some(mins)))
        });
        match result {
            Ok((hours, minutes)) => (hours, minutes),
            Err(e) => {
                error!("Failed to get scheduled alarm time from RTC: {}", e);
                (None, None)
            }
        }
    }

    fn try_attiny_write_command(
        &mut self,
        delay: &mut Delay,
        command: u8,
        value: u8,
    ) -> Result<(), &str> {
        let request_crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[command, value]);
        self.with_i2c_retrying(delay, |i2c| {
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

    pub fn check_device_present(&mut self, addr: u8, delay: &mut Delay) -> Result<bool, &str> {
        self.with_i2c(
            delay,
            |i2c| {
                let payload = &mut [0];
                i2c.write_read(addr, &[0], payload)
                    .map(|v| true)
                    .map_err(|e| map_i2c_err(e.kind()))
            },
            Some(0),
        )
    }

    fn attiny_write_read_command(
        &mut self,
        delay: &mut Delay,
        command: u8,
        value: Option<u8>,
        payload: &mut [u8; 3],
    ) -> Result<(), &str> {
        self.with_i2c_retrying(delay, |i2c| {
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

    fn try_attiny_read_command(
        &mut self,
        delay: &mut Delay,
        command: u8,
    ) -> Result<u8, &'static str> {
        let mut payload = [0u8; 3];
        let request_crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[command]);
        self.with_i2c_retrying(delay, |i2c| {
            payload[0] = 0;
            let mut request = [command, 0x00, 0x00];
            BigEndian::write_u16(&mut request[1..=2], request_crc);
            let result = i2c
                .write_read(ATTINY_ADDRESS, &request, &mut payload)
                .map(|r| 0u8)
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

    pub fn get_attiny_firmware_version(&mut self, delay: &mut Delay) -> Result<u8, &'static str> {
        self.try_attiny_read_command(delay, REG_VERSION)
    }

    pub fn tell_pi_to_shutdown(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.try_attiny_write_command(delay, REG_RP2040_PI_POWER_CTRL, 0x00)
    }

    pub fn tell_pi_to_wakeup(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.try_attiny_write_command(delay, REG_RP2040_PI_POWER_CTRL, 0x01)
    }

    pub fn power_ctrl_status(&mut self, delay: &mut Delay) -> Result<u8, &str> {
        self.try_attiny_read_command(delay, REG_RP2040_PI_POWER_CTRL)
    }

    pub fn get_is_recording(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        Ok((state & tc2_agent_state::RECORDING) == tc2_agent_state::RECORDING)
    }

    pub fn attiny_keep_alive(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.try_attiny_write_command(delay, REG_KEEP_ALIVE, 0x01)
    }

    fn set_recording_flag(&mut self, delay: &mut Delay, is_recording: bool) -> Result<(), &str> {
        let mut state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        if is_recording {
            state |= tc2_agent_state::RECORDING;
        } else {
            state &= !tc2_agent_state::RECORDING;
        }
        self.try_attiny_write_command(delay, REG_TC2_AGENT_STATE, state)
    }

    fn set_offload_flag(&mut self, delay: &mut Delay, is_offloading: bool) -> Result<(), &str> {
        let mut state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        if is_offloading {
            state |= tc2_agent_state::OFFLOAD;
        } else {
            state &= !tc2_agent_state::OFFLOAD;
        }
        self.try_attiny_write_command(delay, REG_TC2_AGENT_STATE, state)
    }

    pub fn started_recording(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.set_recording_flag(delay, true)
    }

    pub fn stopped_recording(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.set_recording_flag(delay, false)
    }

    pub fn begin_offload(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.set_offload_flag(delay, true)
    }

    pub fn end_offload(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.set_offload_flag(delay, false)
    }

    pub fn offload_flag_is_set(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        Ok(state & tc2_agent_state::OFFLOAD == tc2_agent_state::OFFLOAD)
    }

    pub fn pi_is_waking_or_awake(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_CAMERA_STATE)?;
        let camera_state = CameraState::from(state);
        match camera_state {
            CameraState::PoweredOn | CameraState::PoweringOn => Ok(true),
            _ => Ok(false),
        }
    }
    pub fn pi_is_awake_and_tc2_agent_is_ready(
        &mut self,
        delay: &mut Delay,
        print: bool,
    ) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_CAMERA_STATE)?;
        let camera_state = CameraState::from(state);
        let pi_is_awake = match camera_state {
            CameraState::PoweredOn => Ok(true),
            _ => Ok(false),
        };
        // FIXME: Tidy control flow
        pi_is_awake.and_then(|is_awake| {
            if is_awake {
                // If the agent is ready, make sure the REG_RP2040_PI_POWER_CTRL is set to 1
                let _ = self.tell_pi_to_wakeup(delay);
                self.tc2_agent_is_ready(delay, print)
            } else {
                if print {
                    info!("Camera state {:?}", camera_state);
                }
                Ok(false)
            }
        })
    }

    pub fn tc2_agent_is_ready(&mut self, delay: &mut Delay, print: bool) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        if print {
            if state == tc2_agent_state::NOT_READY {
                info!("tc2-agent not ready");
            } else if state == tc2_agent_state::READY {
                // 2
                info!("tc2-agent ready");
            } else if state == tc2_agent_state::NOT_READY | tc2_agent_state::RECORDING {
                info!("tc2-agent not ready, rp2040 recording",);
            } else if state == tc2_agent_state::READY | tc2_agent_state::RECORDING {
                info!("tc2-agent ready and rp2040 recording",);
            } else if state == tc2_agent_state::READY | tc2_agent_state::TEST_AUDIO_RECORDING {
                info!("tc2-agent ready and wanting test audio recording");
            } else {
                info!("tc2-agent unknown state {:08b}({})", state, state);
            }
        }
        Ok((state & tc2_agent_state::READY) == tc2_agent_state::READY)
    }

    pub fn tc2_agent_state(&mut self, delay: &mut Delay) -> Result<u8, &str> {
        self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)
    }

    pub fn tc2_agent_requested_audio_recording(
        &mut self,
        delay: &mut Delay,
    ) -> Result<Option<AudioRecordingType>, &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        let rec_state = state & tc2_agent_state::READY == tc2_agent_state::READY;
        if rec_state {
            if state & tc2_agent_state::TEST_AUDIO_RECORDING
                == tc2_agent_state::TEST_AUDIO_RECORDING
            {
                Ok(Some(AudioRecordingType::test_recording()))
            } else if state & tc2_agent_state::LONG_AUDIO_RECORDING
                == tc2_agent_state::LONG_AUDIO_RECORDING
            {
                Ok(Some(AudioRecordingType::long_recording()))
            } else if state & tc2_agent_state::TAKE_AUDIO == tc2_agent_state::TAKE_AUDIO {
                Ok(Some(AudioRecordingType::thermal_scheduled_recording()))
            } else {
                Ok(None)
            }
        } else {
            Ok(None)
        }
    }

    pub fn tc2_agent_clear_test_audio_rec(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::TEST_AUDIO_RECORDING, false)
    }

    pub fn tc2_agent_clear_mode_flags(&mut self, delay: &mut Delay) -> Result<(), &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        let mut val = state & !tc2_agent_state::LONG_AUDIO_RECORDING;
        val &= !tc2_agent_state::TEST_AUDIO_RECORDING;
        val &= !tc2_agent_state::THERMAL_MODE;
        val &= !tc2_agent_state::TAKE_AUDIO;
        self.try_attiny_write_command(delay, REG_TC2_AGENT_STATE, val)
    }

    pub fn tc2_agent_write_flag(
        &mut self,
        delay: &mut Delay,
        flag: u8,
        set: bool,
    ) -> Result<(), &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        let val = if set { state | flag } else { state & !flag };
        self.try_attiny_write_command(delay, REG_TC2_AGENT_STATE, val)
    }

    pub fn tc2_agent_clear_and_set_flag(
        &mut self,
        delay: &mut Delay,
        clear_flag: u8,
        set_flag: Option<u8>,
    ) -> Result<(), &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        let mut val = state & !clear_flag;
        if let Some(flag) = set_flag {
            val |= flag;
        }
        self.try_attiny_write_command(delay, REG_TC2_AGENT_STATE, val)
    }

    pub fn tc2_agent_requested_thermal_mode(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_TC2_AGENT_STATE)?;
        Ok((state & tc2_agent_state::THERMAL_MODE) == tc2_agent_state::THERMAL_MODE)
    }
    pub fn tc2_agent_request_thermal_mode(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::THERMAL_MODE, true)
    }

    pub fn tc2_agent_clear_thermal_mode(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::THERMAL_MODE, false)
    }

    pub fn tc2_agent_take_audio_rec(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::TAKE_AUDIO, true)
    }

    pub fn tc2_agent_clear_take_audio_rec(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::TAKE_AUDIO, false)
    }

    pub fn pi_is_powered_down(&mut self, delay: &mut Delay, print: bool) -> Result<bool, &str> {
        let state = self.try_attiny_read_command(delay, REG_CAMERA_STATE)?;
        let camera_state = CameraState::from(state);
        if print {
            info!("Pi camera state {}", camera_state);
        }
        match camera_state {
            CameraState::PoweredOff => Ok(true),
            _ => Ok(false),
        }
    }

    pub fn get_datetime(&mut self, delay: &mut Delay) -> Result<DateTime, &str> {
        self.with_rtc(delay, |rtc| {
            let datetime = rtc.get_datetime().map_err(map_rtc_err)?;
            if datetime.day == 0 || datetime.day > 31 || datetime.month == 0 || datetime.month > 12
            {
                Err("Invalid datetime output from RTC")
            } else {
                Ok(datetime)
            }
        })
    }

    pub fn enable_alarm(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.with_rtc(delay, |rtc| {
            // FIXME: Also verify each one of these steps
            rtc.clear_alarm_flag().map_err(map_rtc_err)?;
            rtc.control_alarm_interrupt(Control::On)
                .map_err(map_rtc_err)?;
            rtc.control_alarm_day(Control::Off).map_err(map_rtc_err)?;
            rtc.control_alarm_hours(Control::On).map_err(map_rtc_err)?;
            rtc.control_alarm_minutes(Control::On)
                .map_err(map_rtc_err)?;
            rtc.control_alarm_weekday(Control::Off)
                .map_err(map_rtc_err)?;
            Ok(())
        })
    }

    pub fn disable_alarm(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.with_rtc(delay, |rtc| {
            // FIXME: Also verify each one of these steps
            rtc.clear_alarm_flag().map_err(map_rtc_err)?;
            rtc.control_alarm_interrupt(Control::Off)
                .map_err(map_rtc_err)?;
            rtc.disable_all_alarms().map_err(map_rtc_err)?;
            Ok(())
        })
    }

    pub fn set_wakeup_alarm(
        &mut self,
        datetime_utc: &chrono::DateTime<Utc>,
        delay: &mut Delay,
    ) -> Result<(), &str> {
        #[allow(clippy::cast_possible_truncation)]
        let wake_hour = datetime_utc.time().hour() as u8;
        #[allow(clippy::cast_possible_truncation)]
        let wake_min = datetime_utc.time().minute() as u8;
        self.set_wakeup_alarm_hours_mins(delay, wake_hour, wake_min)
    }

    pub fn set_wakeup_alarm_hours_mins(
        &mut self,
        delay: &mut Delay,
        wake_hour: u8,
        wake_min: u8,
    ) -> Result<(), &str> {
        info!("Setting wake alarm for UTC {}:{}", wake_hour, wake_min);
        // TODO: Get to the bottom of the finicky behaviour we're seeing.
        //  Maybe just do the alarms etc in terms of our own i2c abstraction.

        // For some reason doing this first seems to prime the alarm so that it actually succeeds!?
        let result = self.with_i2c_retrying(delay, |i2c| {
            let mut data = [0];
            let minute_alarm: u8 = 0x09;
            let hour_alarm: u8 = 0x0A;
            i2c.write_read(0x51u8, &[hour_alarm], &mut data)
                .map_err(|e| map_i2c_err(e.kind()))?;
            let hours = data[0];
            i2c.write_read(0x51u8, &[minute_alarm], &mut data)
                .map_err(|e| map_i2c_err(e.kind()))?;
            let minutes = data[0];
            info!("Hours {:08b}, minutes {:08b}", hours, minutes);
            Ok(())
        });

        // FIXME: Maybe have a separate retry loop for all of these steps?
        let result = self.with_rtc(delay, |rtc| {
            rtc.control_alarm_minutes(Control::On)
                .map_err(map_rtc_err)?;
            rtc.control_alarm_hours(Control::On).map_err(map_rtc_err)?;
            let alarm_hours_enabled = rtc.is_alarm_hours_enabled().map_err(map_rtc_err)?;
            if !alarm_hours_enabled {
                return Err("alarm hours not enabled");
            }
            let alarm_minutes_enabled = rtc.is_alarm_minutes_enabled().map_err(map_rtc_err)?;
            if !alarm_minutes_enabled {
                return Err("alarm minutes not enabled");
            }
            // let interrupt_enabled = rtc.is_alarm_interrupt_enabled().map_err(map_rtc_err)?;
            // if !interrupt_enabled {
            //     return Err("alarm interrupt not enabled");
            // }
            rtc.set_alarm_hours(wake_hour).map_err(map_rtc_err)?;
            rtc.set_alarm_minutes(wake_min).map_err(map_rtc_err)?;
            let set_hour = rtc.get_alarm_hours().map_err(map_rtc_err)?;
            if set_hour != wake_hour {
                return Err("wake hour didn't match set hour");
            }
            let set_min = rtc.get_alarm_minutes().map_err(map_rtc_err)?;
            if set_min != wake_min {
                return Err("wake minute didn't match set minute");
            }
            Ok(())
        });
        if let Err(e) = result {
            error!("Failed to set wake alarm: {}", e);
        } else {
            info!("Set alarm {:?}", self.get_scheduled_alarm_time(delay));
        }
        result
    }

    pub fn alarm_triggered(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        self.with_rtc(delay, |rtc| rtc.get_alarm_flag().map_err(map_rtc_err))
    }

    fn restore_i2c_rtc_lock_pin(&mut self, pin: I2cLockedPin, rtc: PCF8563<I2CConfig>) {
        self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
        self.rtc = Some(rtc);
    }

    fn take_i2c_rtc_lock(&mut self) -> Option<(I2cLockedPin, PCF8563<I2CConfig>)> {
        if let Some(config) = self.i2c.take() {
            self.rtc = Some(PCF8563::new(config));
        }
        let mut lock_pin = self.unlocked_pin.take().unwrap();
        let is_low = lock_pin.is_low().unwrap_or(false);
        if is_low {
            let pin = lock_pin.into_pull_type::<PullUp>();
            Some((pin, self.rtc.take().unwrap()))
        } else {
            self.unlocked_pin = Some(lock_pin);
            None
        }
    }

    fn take_i2c_attiny_lock(&mut self) -> Option<(I2cLockedPin, I2CConfig)> {
        if let Some(device) = self.rtc.take() {
            let dev = device.destroy();
            self.i2c = Some(dev);
        }
        let mut lock_pin = self.unlocked_pin.take().unwrap();
        let is_low = lock_pin.is_low().unwrap_or(false);
        if is_low {
            let pin = lock_pin.into_pull_type::<PullUp>();
            Some((pin, self.i2c.take().unwrap()))
        } else {
            self.unlocked_pin = Some(lock_pin);
            None
        }
    }

    fn restore_i2c_attiny_lock_pin(&mut self, pin: I2cLockedPin, i2c: I2CConfig) {
        self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
        self.i2c = Some(i2c);
    }

    fn with_rtc<SUCCESS>(
        &mut self,
        delay: &mut Delay,
        mut func: impl FnMut(&mut PCF8563<I2CConfig>) -> Result<SUCCESS, &'static str>,
    ) -> Result<SUCCESS, &'static str> {
        let mut attempts = 0;
        loop {
            let result = if let Some((pin, mut rtc)) = self.take_i2c_rtc_lock() {
                let result = func(&mut rtc);
                self.restore_i2c_rtc_lock_pin(pin, rtc);
                result
            } else {
                Err("Failed to get i2c lock")
            };
            if result.is_ok() {
                return result;
            }
            if attempts >= DEFAULT_MAX_I2C_ATTEMPTS {
                return result;
            }
            attempts += 1;
            delay.delay_us(500);
        }
    }

    fn with_i2c<SUCCESS>(
        &mut self,
        delay: &mut Delay,
        mut func: impl FnMut(&mut I2CConfig) -> Result<SUCCESS, &'static str>,
        max_attempts: Option<u8>,
    ) -> Result<SUCCESS, &'static str> {
        let mut attempts = 0;
        let max_attempts = max_attempts.unwrap_or(DEFAULT_MAX_I2C_ATTEMPTS);
        loop {
            let result = if let Some((pin, mut i2c)) = self.take_i2c_attiny_lock() {
                let result = func(&mut i2c);
                self.restore_i2c_attiny_lock_pin(pin, i2c);
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
            delay.delay_us(500);
        }
    }

    fn with_i2c_retrying<SUCCESS>(
        &mut self,
        delay: &mut Delay,
        func: impl FnMut(&mut I2CConfig) -> Result<SUCCESS, &'static str>,
    ) -> Result<SUCCESS, &'static str> {
        self.with_i2c(delay, func, None)
    }

    pub fn clear_alarm(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.with_rtc(delay, |rtc| rtc.clear_alarm_flag().map_err(map_rtc_err))
    }

    pub fn alarm_interrupt_enabled(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        self.with_rtc(delay, |rtc| {
            rtc.is_alarm_interrupt_enabled().map_err(map_rtc_err)
        })
    }

    pub fn tell_attiny_to_power_down_rp2040(&mut self, delay: &mut Delay) -> Result<(), &str> {
        self.try_attiny_write_command(delay, REG_RP2040_PI_POWER_CTRL, 0x02)
    }

    fn try_read_eeprom_command(
        &mut self,
        delay: &mut Delay,
        command: u8,
        payload: &mut [u8],
    ) -> Result<(), &str> {
        self.with_i2c_retrying(delay, |i2c| {
            i2c.write_read(EEPROM_I2C_ADDRESS, &[command], payload)
                .map_err(|e| map_i2c_err(e.kind()))
        })
    }

    pub fn check_if_is_audio_device(&mut self, delay: &mut Delay) -> Result<bool, ()> {
        let page_length: usize = 16;
        assert!(EEPROM_LENGTH < usize::from(u8::MAX));
        let mut eeprom = Eeprom::from_bytes([0u8; EEPROM_LENGTH]);
        for (chunk_num, chunk) in eeprom.as_mut().chunks_mut(16).enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            if let Err(e) = self.try_read_eeprom_command(delay, chunk_num as u8, chunk) {
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
const EEPROM_I2C_ADDRESS: u8 = 0x50;

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

    pub fn power_version(&self) -> [u8; 3] {
        self.inner[4..7].try_into().unwrap()
    }

    pub fn touch_version(&self) -> [u8; 3] {
        self.inner[7..10].try_into().unwrap()
    }

    pub fn mic_version(&self) -> [u8; 3] {
        self.inner[10..13].try_into().unwrap()
    }

    pub fn audio_only(&self) -> bool {
        self.inner[13] != 0
    }

    pub fn id(&self) -> u64 {
        BigEndian::read_u64(&self.inner[4..12])
    }

    pub fn timestamp(&self) -> chrono::DateTime<Utc> {
        // FIXME: If we want to use this, need to know what increments the timestamp was stored in.
        let timestamp = i64::from(BigEndian::read_u32(&self.inner[12..16]));
        chrono::DateTime::<Utc>::from_timestamp_micros(timestamp).unwrap()
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
            info!(
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
            warn!(
                "Eeprom CRC failed expected {} got {}",
                calculated_crc, embedded_crc
            );
        }
        valid
    }
}

use crate::EXPECTED_ATTINY_FIRMWARE_VERSION;
use crate::bsp::pac::I2C1;
use byteorder::{BigEndian, ByteOrder};
use chrono::{Timelike, Utc};
use cortex_m::delay::Delay;
use crc::{Algorithm, Crc};
use defmt::{Format, error, info, warn};
use embedded_hal::digital::InputPin;
use embedded_hal::i2c::I2c;
use pcf8563::{Control, DateTime, PCF8563};
use rp2040_hal::I2C;
use rp2040_hal::gpio::bank0::{Gpio3, Gpio6, Gpio7};
use rp2040_hal::gpio::{FunctionI2C, FunctionSio, Pin, PullDown, PullUp, SioInput};
use rp2040_hal::i2c::Error;

pub type I2CConfig = I2C<
    I2C1,
    (
        Pin<Gpio6, FunctionI2C, PullUp>,
        Pin<Gpio7, FunctionI2C, PullUp>,
    ),
>;
pub struct SharedI2C {
    unlocked_pin: Option<Pin<Gpio3, FunctionSio<SioInput>, PullDown>>,
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

#[repr(u8)]
#[derive(Format)]

pub enum RecordingType {
    Test = 0,
    Long = 1,
    Scheduled = 2,
    ThermalRequestedScheduled = 3,
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
        i2c: I2CConfig,
        unlocked_pin: Pin<Gpio3, FunctionSio<SioInput>, PullDown>,
        delay: &mut Delay,
    ) -> SharedI2C {
        let mut shared_i2c = SharedI2C {
            unlocked_pin: Some(unlocked_pin),
            i2c: Some(i2c),
            rtc: None,
        };

        let mut attempts = 0;
        loop {
            match shared_i2c.get_attiny_firmware_version(delay) {
                Ok(version) => match version {
                    EXPECTED_ATTINY_FIRMWARE_VERSION => {
                        break;
                    }
                    version => {
                        error!(
                            "Mismatched Attiny firmware version – expected {}, got {}",
                            EXPECTED_ATTINY_FIRMWARE_VERSION, version
                        );
                        break;
                    }
                },
                Err(e) => {
                    warn!("Error communicating with i2c, attempt #{}", attempts);
                    attempts += 1;
                    if attempts > 100 {
                        crate::panic!("Unable to communicate with Attiny over i2c: {:?}", e);
                    } else {
                        delay.delay_us(500);
                    }
                }
            }
        }
        shared_i2c
    }

    pub fn free(&mut self) -> (I2CConfig, Pin<Gpio3, FunctionSio<SioInput>, PullDown>) {
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

    fn i2c(&mut self) -> &mut I2CConfig {
        if let Some(device) = self.rtc.take() {
            let dev = device.destroy();
            self.i2c = Some(dev);
        }
        self.i2c.as_mut().unwrap()
    }

    fn write_i2c(&mut self, data: &[u8]) -> Result<(), Error> {
        let result = self.i2c().write(ATTINY_ADDRESS, data);
        while self.i2c().tx_fifo_used() != 0 {}
        result
    }

    fn rtc(&mut self) -> &mut PCF8563<I2CConfig> {
        if let Some(config) = self.i2c.take() {
            self.rtc = Some(PCF8563::new(config));
        }
        self.rtc.as_mut().unwrap()
    }

    fn attiny_write_command(&mut self, command: u8, value: u8, crc: u16) -> Result<(), Error> {
        let mut lock_pin = self.unlocked_pin.take().unwrap();
        let is_low = lock_pin.is_low().unwrap_or(false);
        if is_low {
            let pin = lock_pin.into_pull_type::<PullUp>();
            let mut payload = [command, value, 0, 0];
            BigEndian::write_u16(&mut payload[2..=3], crc);
            let result = self.write_i2c(&payload);
            self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
            result
        } else {
            self.unlocked_pin = Some(lock_pin);
            Err(Error::Abort(1))
        }
    }

    // fn attiny_read_command(&mut self, command: u8, payload: &mut [u8; 3]) -> Result<(), Error> {
    //     match self.i2c().write(ATTINY_ADDRESS, &[command]) {
    //         Ok(_) => self.i2c().read(ATTINY_ADDRESS, payload),
    //         Err(e) => Err(e),
    //     }
    //
    //     let mut response = [0u8; 3];
    //     let mut payload = [command, 0x00, 0x00];
    //     let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&payload[0..1]);
    //     BigEndian::write_u16(&mut payload[1..=2], crc);
    //     match self.i2c()
    //         .write_read(ATTINY_ADDRESS, &payload, &mut response) {
    //         Ok(_) => {
    //
    //         },
    //         Err(e) => Err(e),
    //     }
    // }

    fn attiny_write_read_command(
        &mut self,
        command: u8,
        value: Option<u8>,
        payload: &mut [u8; 3],
    ) -> Result<(), Error> {
        let mut lock_pin = self.unlocked_pin.take().unwrap();
        let is_low = lock_pin.is_low().unwrap_or(false);
        if is_low {
            let pin = lock_pin.into_pull_type::<PullUp>();
            let result = if let Some(v) = value {
                let mut request = [command, v, 0x00, 0x00];
                let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&request[0..=1]);
                BigEndian::write_u16(&mut request[2..=3], crc);
                self.i2c().write_read(ATTINY_ADDRESS, &request, payload)
            } else {
                let mut request = [command, 0x00, 0x00];
                let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&request[0..1]);
                BigEndian::write_u16(&mut request[1..=2], crc);
                self.i2c().write_read(ATTINY_ADDRESS, &request, payload)
            };
            self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
            result
        } else {
            self.unlocked_pin = Some(lock_pin);
            Err(Error::Abort(1))
        }
    }

    // fn try_attiny_write_read_command(
    //     &mut self,
    //     command: u8,
    //     value: Option<u8>,
    //     delay: &mut Delay,
    // ) -> Result<u8, Error> {
    //     let mut payload = [0u8; 3];
    //     let mut num_attempts = 0;
    //     loop {
    //         match self.attiny_write_read_command(command, value, &mut payload) {
    //             Ok(_) => {
    //                 return Ok(payload[0]);
    //             }
    //             Err(e) => {
    //                 if num_attempts == 100 {
    //                     return Err(e);
    //                 }
    //                 num_attempts += 1;
    //                 delay.delay_us(500);
    //             }
    //         }
    //     }
    // }

    fn try_attiny_read_command(
        &mut self,
        command: u8,
        delay: &mut Delay,
        attempts: Option<i32>,
    ) -> Result<u8, Error> {
        let mut payload = [0u8; 3];
        let max_attempts = attempts.unwrap_or(100);
        let mut num_attempts = 0;
        loop {
            payload[0] = 0;
            match self.attiny_write_read_command(command, None, &mut payload) {
                Ok(()) => {
                    let value = payload[0];
                    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&payload[0..1]);
                    if BigEndian::read_u16(&payload[1..=2]) == crc {
                        return Ok(payload[0]);
                    }
                    num_attempts += 1;
                    if num_attempts == max_attempts {
                        return Err(Error::Abort(1));
                    }
                    delay.delay_us(500);
                }
                Err(e) => {
                    num_attempts += 1;
                    if num_attempts == max_attempts {
                        return Err(e);
                    }
                    delay.delay_us(500);
                }
            }
        }
    }

    fn try_attiny_write_command(
        &mut self,
        command: u8,
        value: u8,
        delay: &mut Delay,
    ) -> Result<(), Error> {
        let mut num_attempts = 0;
        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[command, value]);
        loop {
            match self.attiny_write_command(command, value, crc) {
                Ok(()) => {
                    // Now immediately read it back to see that it actually got set properly.
                    let result = self.try_attiny_read_command(command, delay, None);
                    if let Ok(set_val) = result {
                        if set_val == value {
                            return Ok(());
                        }
                        warn!(
                            "Failed writing command {} with value {} to attiny, got {}",
                            command, value, set_val
                        );
                    } else {
                        warn!(
                            "Failed reading back value from attiny for command {}",
                            command
                        );
                        // Continue
                    }
                }
                Err(e) => {
                    if num_attempts == 100 {
                        error!(
                            "Failed writing command {} with value {} to attiny",
                            command, value
                        );
                        return Err(e);
                    }
                    warn!("ATTiny write error, retrying in 500µs");
                    num_attempts += 1;
                    delay.delay_us(500);
                }
            }
        }
    }

    pub fn get_attiny_firmware_version(&mut self, delay: &mut Delay) -> Result<u8, Error> {
        self.try_attiny_read_command(REG_VERSION, delay, None)
    }

    pub fn tell_pi_to_shutdown(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_RP2040_PI_POWER_CTRL, 0x00, delay)
    }

    pub fn tell_pi_to_wakeup(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_RP2040_PI_POWER_CTRL, 0x01, delay)
    }

    pub fn power_ctrl_status(&mut self, delay: &mut Delay) -> Result<u8, Error> {
        self.try_attiny_read_command(REG_RP2040_PI_POWER_CTRL, delay, None)
    }

    pub fn get_is_recording(&mut self, delay: &mut Delay) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => Ok((state & tc2_agent_state::RECORDING) == tc2_agent_state::RECORDING),
            Err(e) => Err(e),
        }
    }

    pub fn attiny_keep_alive(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_KEEP_ALIVE, 0x01, delay)
    }

    pub fn set_recording_flag(
        &mut self,
        delay: &mut Delay,
        is_recording: bool,
    ) -> Result<(), Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(mut state) => {
                if is_recording {
                    state |= tc2_agent_state::RECORDING;
                } else {
                    state &= !tc2_agent_state::RECORDING;
                }
                match self.try_attiny_write_command(REG_TC2_AGENT_STATE, state, delay) {
                    Ok(()) => Ok(()),
                    Err(x) => Err(x),
                }
            }
            Err(x) => Err(x),
        }
    }

    pub fn tc2_agent_is_recording(&mut self, delay: &mut Delay) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => Ok((state & 0x04) == 0x04),
            Err(e) => Err(e),
        }
    }

    pub fn pi_is_waking_or_awake(&mut self, delay: &mut Delay) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_CAMERA_STATE, delay, None) {
            Ok(state) => {
                let camera_state = CameraState::from(state);
                match camera_state {
                    CameraState::PoweredOn | CameraState::PoweringOn => Ok(true),
                    _ => Ok(false),
                }
            }
            Err(e) => Err(e),
        }
    }
    pub fn pi_is_awake_and_tc2_agent_is_ready(
        &mut self,
        delay: &mut Delay,
        print: bool,
    ) -> Result<bool, Error> {
        let mut recorded_camera_state = None;
        let pi_is_awake = match self.try_attiny_read_command(REG_CAMERA_STATE, delay, None) {
            Ok(state) => {
                recorded_camera_state = Some(CameraState::from(state));
                let camera_state = CameraState::from(state);
                match camera_state {
                    CameraState::PoweredOn => Ok(true),
                    _ => Ok(false),
                }
            }
            Err(e) => Err(e),
        };
        pi_is_awake.and_then(|is_awake| {
            if is_awake {
                // If the agent is ready, make sure the REG_RP2040_PI_POWER_CTRL is set to 1
                let _ = self.tell_pi_to_wakeup(delay);
                self.tc2_agent_is_ready(delay, print, None)
            } else {
                if print {
                    if let Some(state) = &recorded_camera_state {
                        info!("Camera state {:?}", state);
                    }
                }
                Ok(false)
            }
        })
    }

    pub fn tc2_agent_is_ready(
        &mut self,
        delay: &mut Delay,
        print: bool,
        max_attempts: Option<i32>,
    ) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, max_attempts) {
            Ok(state) => {
                if print {
                    if state == 0 {
                        info!("tc2-agent not ready");
                    } else if state == 2 {
                        info!("tc2-agent ready");
                    } else if state == 4 {
                        info!("tc2-agent not ready, rp2040 recording",);
                    } else if state == 6 {
                        info!("tc2-agent ready and rp2040 recording",);
                    } else if state == 10 {
                        info!("tc2-agent ready and wanting test recording {}", state);
                    } else {
                        info!("tc2-agent unknown state {}", state);
                    }
                }
                Ok((state & tc2_agent_state::READY) == tc2_agent_state::READY)
            }
            Err(e) => Err(e),
        }
    }

    pub fn tc2_agent_state(&mut self, delay: &mut Delay) -> Result<u8, Error> {
        self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None)
    }

    pub fn tc2_agent_requested_audio_recording(
        &mut self,
        delay: &mut Delay,
    ) -> Result<Option<RecordingType>, Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => {
                let rec_state: bool = state & tc2_agent_state::READY == tc2_agent_state::READY;
                if rec_state {
                    if state & tc2_agent_state::TEST_AUDIO_RECORDING
                        == tc2_agent_state::TEST_AUDIO_RECORDING
                    {
                        return Ok(Some(RecordingType::Test));
                    } else if state & tc2_agent_state::LONG_AUDIO_RECORDING
                        == tc2_agent_state::LONG_AUDIO_RECORDING
                    {
                        return Ok(Some(RecordingType::Long));
                    } else if state & tc2_agent_state::TAKE_AUDIO == tc2_agent_state::TAKE_AUDIO {
                        return Ok(Some(RecordingType::ThermalRequestedScheduled));
                    }
                }
                Ok(None)
            }
            Err(e) => Err(e),
        }
    }

    pub fn tc2_agent_clear_test_audio_rec(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::TEST_AUDIO_RECORDING, false)
    }

    pub fn tc2_agent_write_flag(
        &mut self,
        delay: &mut Delay,
        flag: u8,
        set: bool,
    ) -> Result<(), Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => {
                let val = if set { state | flag } else { state & !flag };
                match self.try_attiny_write_command(REG_TC2_AGENT_STATE, val, delay) {
                    Ok(()) => Ok(()),
                    Err(x) => Err(x),
                }
            }
            Err(e) => Err(e),
        }
    }

    pub fn tc2_agent_clear_and_set_flag(
        &mut self,
        delay: &mut Delay,
        clear_flag: u8,
        set_flag: Option<u8>,
    ) -> Result<(), Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => {
                let mut val = state & !clear_flag;
                if let Some(flag) = set_flag {
                    val |= flag;
                }

                match self.try_attiny_write_command(REG_TC2_AGENT_STATE, val, delay) {
                    Ok(()) => Ok(()),
                    Err(x) => Err(x),
                }
            }
            Err(e) => Err(e),
        }
    }

    pub fn tc2_agent_requested_thermal_mode(&mut self, delay: &mut Delay) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => {
                Ok((state & tc2_agent_state::THERMAL_MODE) == tc2_agent_state::THERMAL_MODE)
            }
            Err(e) => Err(e),
        }
    }
    pub fn tc2_agent_request_thermal_mode(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::THERMAL_MODE, true)
    }

    pub fn tc2_agent_clear_thermal_mode(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::THERMAL_MODE, false)
    }

    pub fn tc2_agent_take_audio_rec(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::TAKE_AUDIO, true)
    }

    pub fn tc2_agent_clear_take_audio_rec(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.tc2_agent_write_flag(delay, tc2_agent_state::TAKE_AUDIO, false)
    }

    pub fn pi_is_powered_down(&mut self, delay: &mut Delay, print: bool) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_CAMERA_STATE, delay, None) {
            Ok(state) => {
                let camera_state = CameraState::from(state);
                if print {
                    info!("Pi camera state {}", camera_state);
                }
                match camera_state {
                    CameraState::PoweredOff => Ok(true),
                    _ => Ok(false),
                }
            }
            Err(e) => Err(e),
        }
    }

    pub fn get_datetime(&mut self, delay: &mut Delay) -> Result<DateTime, &str> {
        let mut num_attempts = 0;
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let result = match self.rtc().get_datetime() {
                    Ok(datetime) => {
                        if num_attempts != 0 {
                            info!("Getting datetime took {} attempts", num_attempts);
                        }
                        if datetime.day == 0
                            || datetime.day > 31
                            || datetime.month == 0
                            || datetime.month > 12
                        {
                            Err("Invalid datetime input from RTC")
                        } else {
                            Ok(datetime)
                        }
                    }
                    Err(pcf8563::Error::I2C(e)) => Err("I2C error to RTC"),
                    Err(pcf8563::Error::InvalidInputData) => {
                        unreachable!("Should never get here")
                    }
                };

                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if result.is_ok() || num_attempts == 100 {
                    return result;
                }
                num_attempts += 1;
                delay.delay_us(500);
            } else {
                num_attempts += 1;
                delay.delay_us(500);
                self.unlocked_pin = Some(lock_pin);
                if num_attempts == 100 {
                    return Err("I2C in use");
                }
            }
        }
    }

    pub fn enable_alarm(&mut self, delay: &mut Delay) -> Result<(), &str> {
        let mut attempts = 0;
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let mut success = true;
                success = success && self.rtc().clear_alarm_flag().is_ok();
                success = success && self.rtc().control_alarm_interrupt(Control::On).is_ok();
                success = success && self.rtc().control_alarm_interrupt(Control::On).is_ok();
                success = success && self.rtc().control_alarm_day(Control::Off).is_ok();
                success = success && self.rtc().control_alarm_hours(Control::On).is_ok();
                success = success && self.rtc().control_alarm_minutes(Control::On).is_ok();
                success = success && self.rtc().control_alarm_weekday(Control::Off).is_ok();
                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if success {
                    return Ok(());
                }
                attempts += 1;
                if attempts == 100 {
                    return Err("Failed to enable alarm");
                }
                delay.delay_us(500);
            } else {
                attempts += 1;
                self.unlocked_pin = Some(lock_pin);
                if attempts == 100 {
                    return Err("Failed to access I2C to enable alarm");
                }
                delay.delay_us(500);
            }
        }
    }

    pub fn disable_alarm(&mut self, delay: &mut Delay) -> Result<(), &str> {
        let mut attempts = 0;
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let mut success = true;
                success = success && self.rtc().clear_alarm_flag().is_ok();
                success = success && self.rtc().control_alarm_interrupt(Control::Off).is_ok();
                success = success && self.rtc().disable_all_alarms().is_ok();

                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if success {
                    return Ok(());
                }
                attempts += 1;
                if attempts == 100 {
                    return Err("Failed to enable alarm");
                }
                delay.delay_us(500);
            } else {
                attempts += 1;
                self.unlocked_pin = Some(lock_pin);
                if attempts == 100 {
                    return Err("Failed to access I2C to enable alarm");
                }
                delay.delay_us(500);
            }
        }
    }
    pub fn set_wakeup_alarm(
        &mut self,
        datetime_utc: &chrono::DateTime<Utc>,
        delay: &mut Delay,
    ) -> Result<(), &str> {
        let wake_hour = datetime_utc.time().hour();
        let wake_min = datetime_utc.time().minute();
        info!("Setting wake alarm for UTC {}h:{}m", wake_hour, wake_min);
        let mut num_attempts = 0;
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            #[allow(clippy::cast_possible_truncation)]
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let mut success = true;
                success = success && self.rtc().set_alarm_hours(wake_hour as u8).is_ok();
                success = success && self.rtc().set_alarm_minutes(wake_min as u8).is_ok();
                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if success {
                    return Ok(());
                }
                if num_attempts == 100 {
                    return Err("Failed to set wakeup alarm");
                }
                num_attempts += 1;
                delay.delay_us(500);
            } else {
                num_attempts += 1;
                self.unlocked_pin = Some(lock_pin);
                if num_attempts == 100 {
                    return Err("Failed to access I2C to set wakeup alarm");
                }
                delay.delay_us(500);
            }
        }
    }

    pub fn alarm_triggered(&mut self, delay: &mut Delay) -> bool {
        // NOTE: This only returns on success, otherwise it blocks indefinitely.
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let result = self.rtc().get_alarm_flag();
                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if let Ok(result) = result {
                    return result;
                }
            } else {
                self.unlocked_pin = Some(lock_pin);
            }
            delay.delay_us(500);
        }
    }

    pub fn clear_alarm(&mut self, delay: &mut Delay) {
        // FIXME: deal with all this duplicate logic
        // NOTE: This only returns on success, otherwise it blocks indefinitely.
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let result = self.rtc().clear_alarm_flag();
                let success = result.is_ok();
                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if success {
                    return;
                }
            } else {
                self.unlocked_pin = Some(lock_pin);
            }
            delay.delay_us(500);
        }
    }

    pub fn alarm_interrupt_enabled(&mut self, delay: &mut Delay) -> Result<bool, &str> {
        let mut attempts = 0;
        loop {
            let mut lock_pin = self.unlocked_pin.take().unwrap();
            let is_low = lock_pin.is_low().unwrap_or(false);
            if is_low {
                let pin = lock_pin.into_pull_type::<PullUp>();
                let result = self.rtc().is_alarm_interrupt_enabled();
                let success = result.is_ok();
                self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
                if success {
                    return Ok(result.unwrap());
                }
                attempts += 1;
                if attempts == 100 {
                    return Err("Failed to enable alarm");
                }
                delay.delay_us(500);
            } else {
                attempts += 1;
                self.unlocked_pin = Some(lock_pin);
                if attempts == 100 {
                    return Err("Failed to access I2C to enable alarm");
                }
                delay.delay_us(500);
            }
        }
    }

    pub fn tell_attiny_to_power_down_rp2040(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_RP2040_PI_POWER_CTRL, 0x02, delay)
    }

    fn read_eeprom_command(&mut self, command: u8, payload: &mut [u8]) -> Result<(), Error> {
        let mut lock_pin = self.unlocked_pin.take().unwrap();
        let is_low = lock_pin.is_low().unwrap_or(false);
        if is_low {
            let pin = lock_pin.into_pull_type::<PullUp>();
            let result = self.i2c().write_read(EEPROM_ADDRESS, &[command], payload);
            self.unlocked_pin = Some(pin.into_pull_type::<PullDown>());
            result
        } else {
            self.unlocked_pin = Some(lock_pin);
            Err(Error::Abort(1))
        }
    }

    fn try_read_eeprom_command(
        &mut self,
        command: u8,
        delay: &mut Delay,
        attempts: Option<i32>,
        payload: &mut [u8],
    ) -> Result<(), Error> {
        let max_attempts = attempts.unwrap_or(100);
        let mut num_attempts = 0;
        loop {
            match self.read_eeprom_command(command, payload) {
                Ok(()) => {
                    return Ok(());
                }
                Err(e) => {
                    num_attempts += 1;
                    if num_attempts == max_attempts {
                        return Err(e);
                    }
                    delay.delay_us(500);
                }
            }
        }
    }

    pub fn check_if_is_audio_device(&mut self, delay: &mut Delay) -> Result<bool, ()> {
        let page_length: usize = 16;

        let mut read_length: usize;
        assert!(EEPROM_LENGTH < usize::from(u8::MAX));
        let mut eeprom_data = [0u8; EEPROM_LENGTH];
        for i in (0..EEPROM_LENGTH).step_by(page_length) {
            read_length = if EEPROM_LENGTH - i < page_length {
                EEPROM_LENGTH - i
            } else {
                page_length
            };
            #[allow(clippy::cast_possible_truncation)]
            if let Err(e) = self.try_read_eeprom_command(
                i as u8,
                delay,
                None,
                &mut eeprom_data[i..read_length + i],
            ) {
                warn!("Couldn't read eeprom data {}", e);
                return Err(());
            }
            if i == 0 && eeprom_data[1] == 1 {
                // don't think any need to parse as it has no audio info
                info!("Need eeprom version 2 ");
                return Err(());
            }
        }
        let has_data = eeprom_data.iter().any(|&x| x != 0xff);
        if !has_data {
            info!("No EEPROM data");
            return Err(());
        }

        if eeprom_data[0] != 0xCA {
            info!(
                "Incorrect first byte got {} should be {}",
                eeprom_data[0], 0xCA
            );
            return Err(());
        }
        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&eeprom_data[..eeprom_data.len() - 2]);
        let gotcrc = BigEndian::read_u16(&eeprom_data[eeprom_data.len() - 2..]);
        if gotcrc == crc {
            return Ok(eeprom_data[14] > 0);
        }
        info!("CRC failed expected {} got {}", crc, gotcrc);
        Err(())
    }
}

const EEPROM_LENGTH: usize = 1 + 1 + 3 * 4 + 1 + 8 + 4 + 2;
const EEPROM_ADDRESS: u8 = 0x50;

//not used to save memory
// #[derive(Format)]
// pub struct EEPROM {
//     pub version: u8,
//     pub hardware_version: [u8; 3],
//     pub power_version: [u8; 3],
//     pub touch_version: [u8; 3],
//     pub mic_version: [u8; 3],
//     pub audio_only: bool,
//     pub id: u64,
//     pub timestamp: u32,
// }

// impl EEPROM {
//     pub fn new(payload: &[u8]) -> EEPROM {
//         EEPROM {
//             version: payload[0],
//             hardware_version: payload[1..4].try_into().unwrap(),
//             power_version: payload[4..7].try_into().unwrap(),
//             touch_version: payload[7..10].try_into().unwrap(),
//             mic_version: payload[10..13].try_into().unwrap(),
//             audio_only: payload[13] > 0,
//             id: BigEndian::read_u64(&payload[4..12]),
//             timestamp: BigEndian::read_u32(&payload[12..16]),
//         }
//     }
// }

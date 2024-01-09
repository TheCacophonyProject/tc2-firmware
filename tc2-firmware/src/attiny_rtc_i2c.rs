use crate::bsp::pac::I2C1;
use chrono::{Datelike, NaiveDateTime, Timelike};
use core::ops::BitAnd;
use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use embedded_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};
use pcf8563::{Control, DateTime, PCF8563};
use rp2040_hal::gpio::bank0::{Gpio6, Gpio7};
use rp2040_hal::gpio::{FunctionI2C, Pin, PullDown};
use rp2040_hal::i2c::Error;
use rp2040_hal::I2C;

pub type I2CConfig = I2C<
    I2C1,
    (
        Pin<Gpio6, FunctionI2C, PullDown>,
        Pin<Gpio7, FunctionI2C, PullDown>,
    ),
>;
pub struct SharedI2C {
    i2c: Option<I2CConfig>,
    rtc: Option<PCF8563<I2CConfig>>,
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

#[repr(u8)]
#[derive(Format)]
enum Tc2AgentReadyState {
    NotReady = 0x00,
    Ready = 0x02,
    Recording = 0x04,
}

impl Into<u8> for CameraState {
    fn into(self) -> u8 {
        self as u8
    }
}

impl From<u8> for CameraState {
    fn from(value: u8) -> Self {
        match value {
            0x00 => CameraState::PoweringOn,
            0x01 => CameraState::PoweredOn,
            0x02 => CameraState::PoweredOff,
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
//const REG_PI_WAKEUP: u8 = 0x06;
const REG_TC2_AGENT_STATE: u8 = 0x07;
impl SharedI2C {
    pub fn new(i2c: I2CConfig, delay: &mut Delay) -> SharedI2C {
        let mut shared_i2c = SharedI2C {
            i2c: Some(i2c),
            rtc: None,
        };

        let _ = match shared_i2c.get_attiny_firmware_version(delay) {
            Ok(version) => match version {
                8 => {}
                version => {
                    error!(
                        "Mismatched Attiny firmware version – expected {}, got {}",
                        8, version
                    );
                }
            },
            Err(e) => {
                crate::panic!("Unable to communicate with Attiny over i2c: {:?}", e);
            }
        };

        shared_i2c
    }

    pub fn free(&mut self) -> I2CConfig {
        if let Some(device) = self.rtc.take() {
            let dev = device.destroy();
            dev
        } else if let Some(device) = self.i2c.take() {
            device
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
    fn rtc(&mut self) -> &mut PCF8563<I2CConfig> {
        if let Some(config) = self.i2c.take() {
            self.rtc = Some(PCF8563::new(config))
        }
        self.rtc.as_mut().unwrap()
    }

    fn attiny_write_command(&mut self, command: u8, value: u8) -> Result<(), Error> {
        self.i2c().write(ATTINY_ADDRESS, &[command, value])
    }

    fn attiny_read_command(&mut self, command: u8, payload: &mut [u8; 1]) -> Result<(), Error> {
        match self.i2c().write(ATTINY_ADDRESS, &[command]) {
            Ok(_) => self.i2c().read(ATTINY_ADDRESS, payload),
            Err(e) => Err(e),
        }
    }

    fn attiny_write_read_command(
        &mut self,
        command: u8,
        value: Option<u8>,
        payload: &mut [u8; 1],
    ) -> Result<(), Error> {
        if let Some(v) = value {
            self.i2c()
                .write_read(ATTINY_ADDRESS, &[command, v], payload)
        } else {
            self.i2c().write_read(ATTINY_ADDRESS, &[command], payload)
        }
    }

    fn try_attiny_write_read_command(
        &mut self,
        command: u8,
        value: Option<u8>,
        delay: &mut Delay,
    ) -> Result<u8, Error> {
        let mut payload = [0u8; 1];
        let mut num_attempts = 0;
        loop {
            match self.attiny_write_read_command(command, value, &mut payload) {
                Ok(_) => {
                    return Ok(payload[0]);
                }
                Err(e) => {
                    if num_attempts == 100 {
                        return Err(e);
                    }
                    num_attempts += 1;
                    delay.delay_us(500);
                }
            }
        }
    }

    fn try_attiny_read_command(
        &mut self,
        command: u8,
        delay: &mut Delay,
        attempts: Option<i32>,
    ) -> Result<u8, Error> {
        let mut payload = [0u8; 1];
        let max_attempts = attempts.unwrap_or(100);
        let mut num_attempts = 0;
        loop {
            payload[0] = 0;
            match self.attiny_read_command(command, &mut payload) {
                Ok(_) => {
                    return Ok(payload[0]);
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
        loop {
            match self.attiny_write_command(command, value) {
                Ok(_) => return Ok(()),
                Err(e) => {
                    if num_attempts == 100 {
                        return Err(e);
                    }
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

    pub fn set_recording_flag(
        &mut self,
        delay: &mut Delay,
        is_recording: bool,
    ) -> Result<(), Error> {
        let state = match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay, None) {
            Ok(state) => {
                info!("Read raw tc2-agent state {}", state);
                Ok(state & 1 << 1 == 2)
            }
            Err(e) => Err(e),
        };
        match state {
            Ok(state) => {
                let mut val = if state { 2 } else { 0 };
                let flag = if is_recording { 4 } else { 0 };
                val |= flag;
                info!("Set tc2-agent state {}", val);
                match self.try_attiny_write_command(REG_TC2_AGENT_STATE, val, delay) {
                    Ok(_) => Ok(()),
                    Err(x) => Err(x),
                }
            }
            Err(x) => Err(x),
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
        if let Ok(is_awake) = pi_is_awake {
            if is_awake {
                // If the agent is ready, make sure the REG_RP2040_PI_POWER_CTRL is set to 1
                let ctrl_state = self.power_ctrl_status(delay);
                info!("Power ctrl state {:?}", ctrl_state);
                let _ = self.tell_pi_to_wakeup(delay);
                let agent_ready = self.tc2_agent_is_ready(delay, print, None);
                agent_ready
            } else {
                if print {
                    if let Some(state) = &recorded_camera_state {
                        info!("Camera state {:?}", state);
                    }
                }
                Ok(false)
            }
        } else {
            pi_is_awake
        }
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
                    } else {
                        info!("tc2-agent unknown state {}", state);
                    }
                }
                Ok(state & 1 << 1 == 2)
            }
            Err(e) => Err(e),
        }
    }

    pub fn pi_is_powered_down(&mut self, delay: &mut Delay) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_CAMERA_STATE, delay, None) {
            Ok(state) => {
                let camera_state = CameraState::from(state);
                info!("Pi camera state {}", camera_state);
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
            match self.rtc().get_datetime() {
                Ok(datetime) => {
                    if num_attempts != 0 {
                        info!("Getting datetime took {} attempts", num_attempts);
                    }
                    if datetime.day == 0
                        || datetime.day > 31
                        || datetime.month == 0
                        || datetime.month > 12
                    {
                        return Err("Invalid datetime input from RTC");
                    }
                    return Ok(datetime);
                }
                Err(pcf8563::Error::I2C(e)) => {
                    if num_attempts == 100 {
                        return Err("I2C error to RTC");
                    }
                    num_attempts += 1;
                    delay.delay_us(500);
                }
                Err(pcf8563::Error::InvalidInputData) => {
                    unreachable!("Should never get here")
                }
            }
        }
    }

    pub fn set_current_time(&mut self) {
        let date_time = DateTime {
            year: 23,
            month: 11,
            weekday: 3,
            day: 22,
            hours: 21,
            minutes: 10,
            seconds: 0,
        };
        self.rtc().set_datetime(&date_time).unwrap();
    }

    pub fn enable_alarm(&mut self, delay: &mut Delay) {
        self.rtc().clear_alarm_flag().unwrap_or(());
        self.rtc()
            .control_alarm_interrupt(Control::On)
            .unwrap_or(());
        self.rtc().control_alarm_day(Control::Off).unwrap_or(());
        self.rtc().control_alarm_hours(Control::On).unwrap_or(());
        self.rtc().control_alarm_minutes(Control::On).unwrap_or(());
        self.rtc().control_alarm_weekday(Control::Off).unwrap_or(());
    }

    pub fn disable_alarm(&mut self, delay: &mut Delay) {
        self.rtc().clear_alarm_flag().unwrap_or(());
        self.rtc()
            .control_alarm_interrupt(Control::Off)
            .unwrap_or(());
        self.rtc().control_alarm_day(Control::Off).unwrap_or(());
        self.rtc().control_alarm_hours(Control::Off).unwrap_or(());
        self.rtc().control_alarm_minutes(Control::Off).unwrap_or(());
        self.rtc().control_alarm_weekday(Control::Off).unwrap_or(());
    }

    pub fn print_alarm_status(&mut self, delay: &mut Delay) {
        info!(
            "Alarm interrupt enabled: {}",
            self.rtc().is_alarm_interrupt_enabled().unwrap_or(false)
        );
        info!(
            "Alarm day enabled: {}",
            self.rtc().is_alarm_day_enabled().unwrap_or(false)
        );
        info!(
            "Alarm weekday enabled: {}",
            self.rtc().is_alarm_weekday_enabled().unwrap_or(false)
        );
        info!(
            "Alarm hour enabled: {}",
            self.rtc().is_alarm_hours_enabled().unwrap_or(false)
        );
        info!(
            "Alarm minute enabled: {}",
            self.rtc().is_alarm_minutes_enabled().unwrap_or(false)
        );
    }
    pub fn set_wakeup_alarm(
        &mut self,
        datetime_utc: &NaiveDateTime,
        delay: &mut Delay,
    ) -> Result<(), Error> {
        let wake_hour = datetime_utc.time().hour();
        let wake_min = datetime_utc.time().minute();
        info!("Setting wake alarm for UTC {}h:{}m", wake_hour, wake_min);
        let mut num_attempts = 0;
        loop {
            match self.rtc().set_alarm_hours(wake_hour as u8) {
                Ok(_) => {
                    num_attempts = 0;
                    loop {
                        match self.rtc().set_alarm_minutes(wake_min as u8) {
                            Ok(_) => return Ok(()),
                            Err(pcf8563::Error::I2C(e)) => {
                                if num_attempts == 100 {
                                    return Err(e);
                                }
                                num_attempts += 1;
                                delay.delay_us(500);
                            }
                            Err(pcf8563::Error::InvalidInputData) => {
                                unreachable!("Should never get here")
                            }
                        }
                    }
                }
                Err(pcf8563::Error::I2C(e)) => {
                    if num_attempts == 100 {
                        return Err(e);
                    }
                    num_attempts += 1;
                    delay.delay_us(500);
                }
                Err(pcf8563::Error::InvalidInputData) => {
                    unreachable!("Should never get here")
                }
            }
        }
    }

    pub fn alarm_triggered(&mut self, delay: &mut Delay) -> bool {
        let mut num_attempts = 0;
        loop {
            match self.rtc().get_alarm_flag() {
                Ok(val) => {
                    return val;
                }
                Err(e) => {
                    if num_attempts == 100 {
                        error!("Failed reading alarm_triggered from RTC");
                        return false;
                    }
                    num_attempts += 1;
                    delay.delay_us(500);
                }
            }
        }
    }

    pub fn clear_alarm(&mut self) -> () {
        self.rtc().clear_alarm_flag().unwrap_or(())
    }

    pub fn alarm_interrupt_enabled(&mut self) -> bool {
        self.rtc().is_alarm_interrupt_enabled().unwrap_or(false)
    }

    pub fn tell_attiny_to_power_down_rp2040(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_RP2040_PI_POWER_CTRL, 0x02, delay)
    }
}

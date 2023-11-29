use crate::bsp::pac::I2C1;
use chrono::{Datelike, NaiveDateTime, Timelike};
use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use embedded_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};
use pcf8563::{DateTime, PCF8563};
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
const REG_TRIGGER_SLEEP: u8 = 0x05;
const REG_PI_WAKEUP: u8 = 0x06;
const REG_TC2_AGENT_STATE: u8 = 0x07;
impl SharedI2C {
    pub fn new(i2c: I2CConfig, delay: &mut Delay) -> SharedI2C {
        let mut shared_i2c = SharedI2C {
            i2c: Some(i2c),
            rtc: None,
        };

        let _ = match shared_i2c.get_attiny_firmware_version(delay) {
            Ok(version) => match version {
                6 => {}
                version => {
                    error!(
                        "Mismatched Attiny firmware version – expected {}, got {}",
                        6, version
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

    fn try_attiny_read_command(&mut self, command: u8, delay: &mut Delay) -> Result<u8, Error> {
        let mut payload = [0u8; 1];
        let mut num_attempts = 0;
        loop {
            match self.attiny_read_command(command, &mut payload) {
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
        self.try_attiny_read_command(REG_VERSION, delay)
    }

    pub fn tell_pi_to_shutdown(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_TRIGGER_SLEEP, 0x01, delay)
    }

    pub fn tell_pi_to_wakeup(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_PI_WAKEUP, 0x01, delay)
    }

    pub fn pi_is_awake_and_tc2_agent_is_ready(
        &mut self,
        delay: &mut Delay,
        print: bool,
    ) -> Result<bool, Error> {
        let mut recorded_camera_state = None;
        let pi_is_awake = match self.try_attiny_read_command(REG_CAMERA_STATE, delay) {
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
                match self.try_attiny_read_command(REG_TC2_AGENT_STATE, delay) {
                    Ok(state) => {
                        if print {
                            info!(
                                "Camera state {:?}, tc2-agent ready: {}",
                                recorded_camera_state.as_ref().unwrap(),
                                state == 2
                            );
                        }
                        Ok(state == 2)
                    }
                    Err(e) => Err(e),
                }
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

    pub fn pi_is_powered_down(&mut self, delay: &mut Delay) -> Result<bool, Error> {
        match self.try_attiny_read_command(REG_CAMERA_STATE, delay) {
            Ok(state) => {
                let camera_state = CameraState::from(state);
                match camera_state {
                    CameraState::PoweredOff => Ok(true),
                    _ => Ok(false),
                }
            }
            Err(e) => Err(e),
        }
    }

    pub fn get_datetime(&mut self, delay: &mut Delay) -> Result<DateTime, Error> {
        let mut num_attempts = 0;
        loop {
            match self.rtc().get_datetime() {
                Ok(datetime) => {
                    if num_attempts != 0 {
                        info!("Getting datetime took {} attempts", num_attempts);
                    }
                    return Ok(datetime);
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

    pub fn set_wakeup_alarm(
        &mut self,
        datetime_utc: &NaiveDateTime,
        delay: &mut Delay,
    ) -> Result<(), Error> {
        let wake_hour = datetime_utc.time().hour();
        let wake_min = datetime_utc.time().minute();
        info!("Setting wake alarm for {}h:{}m", wake_hour, wake_min);
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

    pub fn alarm_triggered(&mut self) -> bool {
        self.rtc().get_alarm_flag().unwrap_or(false)
    }

    pub fn clear_alarm(&mut self) -> () {
        self.rtc().clear_alarm_flag().unwrap_or(())
    }

    pub fn alarm_interrupt_enabled(&mut self) -> bool {
        self.rtc().is_alarm_interrupt_enabled().unwrap_or(false)
    }

    pub fn tell_attiny_to_power_down_rp2040(&mut self, delay: &mut Delay) -> Result<(), Error> {
        self.try_attiny_write_command(REG_TRIGGER_SLEEP, 0x02, delay)
    }
}

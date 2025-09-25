use crate::attiny_rtc_i2c::{CRC_AUG_CCITT, CameraState, tc2_agent_state};
use crate::tests::stubs::fake_timer::Timer;
use crate::tests::test_global_state::{
    ATTINY_FIRMWARE_VERSION, ATTINY_KEEP_ALIVE, ATTINY_POWER_CTRL_STATE, CAMERA_STATE,
    CURRENT_TIME, RTC_ALARM_STATE, TC2_AGENT_STATE,
};
use byteorder::{BigEndian, ByteOrder};
use chrono::{Datelike, Timelike};
use crc::Crc;
use log::debug;
use std::time::Instant;

pub struct I2CConfig;

pub struct RtcAlarm {
    pub minutes: u8,
    pub(crate) hours: u8,
    pub(crate) day: u8,
    pub(crate) weekday_alarm_mode: u8,
    pub enabled: u8,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[non_exhaustive]
pub enum ErrorKind {
    /// Bus error occurred. e.g. A START or a STOP condition is detected and is not
    /// located after a multiple of 9 SCL clock pulses.
    Bus,
    /// The arbitration was lost, e.g. electrical problems with the clock signal.
    ArbitrationLoss,
    /// A bus operation was not acknowledged, e.g. due to the addressed device not
    /// being available on the bus or the device not being ready to process requests
    /// at the moment.
    NoAcknowledge(NoAcknowledgeSource),
    /// The peripheral receive buffer was overrun.
    Overrun,
    /// A different error occurred. The original error may contain more information.
    Other,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum NoAcknowledgeSource {
    /// The device did not acknowledge its address. The device may be missing.
    Address,
    /// The device did not acknowledge the data. It may not be ready to process
    /// requests at the moment.
    Data,
    /// Either the device did not acknowledge its address or the data, but it is
    /// unknown which.
    Unknown,
}

pub struct I2cError {}
impl I2cError {
    pub fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl I2CConfig {
    pub fn new() -> I2CConfig {
        I2CConfig {}
    }
    pub fn write(&mut self, address: u8, payload: &[u8]) -> Result<(), I2cError> {
        // NOTE: In between writes we might want to advance the state of the external simulated devices.

        // We might actually want to write a state to some external device (rpi power on state)
        match address {
            ATTINY_ADDRESS => match payload[0] {
                ATTINY_REG_KEEP_ALIVE => match payload[1] {
                    0x01 => {
                        // FIXME: Make this be time from our synced clock.
                        *ATTINY_KEEP_ALIVE.lock().unwrap() = Instant::now();
                    }
                    _ => panic!("Unsupported attiny keep alive 0x{:02x?}", payload[1]),
                },
                ATTINY_REG_VERSION => {
                    // nop
                }
                ATTINY_REG_CAMERA_STATE => {
                    // nop
                }
                ATTINY_REG_RP2040_PI_POWER_CTRL => {
                    // Set power control
                    *ATTINY_POWER_CTRL_STATE.lock().unwrap() = payload[1];
                    match payload[1] {
                        0x00 => {
                            // Tell rPi it can shutdown.
                            let mut camera_state = CAMERA_STATE.lock().unwrap();
                            if *camera_state == CameraState::PoweredOn {
                                *camera_state = CameraState::PoweringOff;
                            }
                        }
                        0x01 => {
                            // Tell rPi to wake up.
                            let mut camera_state = CAMERA_STATE.lock().unwrap();
                            if *camera_state == CameraState::PoweredOff {
                                *camera_state = CameraState::PoweringOn;
                            }
                        }
                        0x02 => {
                            // Tell attiny to shut US down.  Can we simulate this?
                            // Maybe it's the same as a restart, with the time advanced?
                            //*CAMERA_STATE.lock().unwrap() = CameraState::PoweringOf;
                        }
                        _ => panic!(
                            "Unsupported attiny power control state 0x{:02x?}",
                            payload[1]
                        ),
                    }
                }
                ATTINY_REG_TC2_AGENT_STATE => {
                    *TC2_AGENT_STATE.lock().unwrap() = payload[1].into();
                }
                _ => panic!("Unsupported attiny command 0x{:02x?}", payload[0]),
            },
            RTC_ADDRESS => match payload[0] {
                RTC_REG_ALARM_MINUTES => {
                    let mut alarm = RTC_ALARM_STATE.lock().unwrap();
                    alarm.minutes = payload[1];
                    alarm.hours = payload[2];
                    alarm.day = payload[3];
                    alarm.weekday_alarm_mode = payload[4];
                }
                RTC_REG_ALARM_CONTROL => {
                    let mut alarm = RTC_ALARM_STATE.lock().unwrap();
                    alarm.enabled = payload[1];
                }
                _ => panic!("Unsupported RTC command"),
            },
            _ => panic!("Unsupported i2c peripheral 0x{:02x?}", address),
        }
        Ok(())
    }

    pub fn write_read(
        &mut self,
        address: u8,
        request: &[u8],
        payload: &mut [u8],
    ) -> Result<(), I2cError> {
        // Maybe time, camera state etc can be mutable static vars that we can changes from outside?
        // Advance external CAMERA_STATE
        match address {
            EEPROM_I2C_ADDRESS => match request[0] {
                _ => panic!("Unsupported EEPROM write: {:02x?}", request[0]),
            },
            RTC_ADDRESS => {
                let current_time = CURRENT_TIME.lock().unwrap();
                match request[0] {
                    RTC_REG_DATETIME_SECONDS => {
                        // FIXME: If we want to compromise time integrity:
                        // if payload[0] & 0b1000_0000 == 0b1000_0000 {
                        //     // Time integrity compromised
                        //     return Err("Time integrity compromised");
                        // }
                        let seconds = current_time.second() as u8;
                        let minutes = current_time.minute() as u8;
                        let hours = current_time.hour() as u8;
                        let day = current_time.day() as u8;
                        let month = current_time.month() as u8;
                        let year = (current_time.year() - 2000) as u8;

                        // info!("Setting RTC to: {:?} ", current_time);
                        // info!("Seconds: {:?} ", seconds);
                        // info!("Minutes: {:?} ", minutes);
                        // info!("Hours: {:?} ", hours);
                        // info!("Day: {:?} ", day);
                        // info!("Month: {:?} ", month);
                        // info!("Year: {:?} ", year);

                        let year = crate::attiny_rtc_i2c::encode_bcd(year);
                        let month = crate::attiny_rtc_i2c::encode_bcd(month);
                        // let weekday = decode_bcd(data[4] & 0x07);
                        let day = crate::attiny_rtc_i2c::encode_bcd(day);
                        let hours = crate::attiny_rtc_i2c::encode_bcd(hours);
                        let minutes = crate::attiny_rtc_i2c::encode_bcd(minutes);
                        let seconds = crate::attiny_rtc_i2c::encode_bcd(seconds);
                        payload[0] = seconds;
                        payload[1] = minutes;
                        payload[2] = hours;
                        payload[3] = day;
                        payload[5] = month;
                        payload[6] = year;
                    }
                    RTC_REG_ALARM_CONTROL => {
                        let alarm = RTC_ALARM_STATE.lock().unwrap();
                        payload[0] = alarm.enabled;
                        debug!(
                            "RTC alarm enabled: {:?}",
                            alarm.enabled & 0b0000_0010 == 0b0000_0010
                        );
                    }
                    RTC_REG_ALARM_MINUTES => {
                        let alarm = RTC_ALARM_STATE.lock().unwrap();
                        payload[0] = alarm.minutes;
                        payload[1] = alarm.hours;
                        payload[2] = alarm.day;
                        payload[3] = alarm.weekday_alarm_mode;
                    }
                    _ => {
                        panic!("Unsupported RTC write: {:02x?}", request);
                    }
                }
            }
            ATTINY_ADDRESS => match request[0] {
                ATTINY_REG_KEEP_ALIVE => {
                    payload[0] = 0x01; // Always return that the keep alive was set.
                    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                    BigEndian::write_u16(&mut payload[1..=2], crc);
                }
                ATTINY_REG_VERSION => {
                    let attiny_firmware_version = ATTINY_FIRMWARE_VERSION.lock().unwrap();
                    payload[0] = *attiny_firmware_version; // Set ATTINY FIRMWARE_VERSION to 1
                    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                    BigEndian::write_u16(&mut payload[1..=2], crc);
                }
                ATTINY_REG_CAMERA_STATE => {
                    // Write the camera state:
                    {
                        let camera_state = CAMERA_STATE.lock().unwrap();
                        payload[0] = *camera_state as u8;
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    // Maybe advance the simulation?
                    {
                        let mut camera_state = CAMERA_STATE.lock().unwrap();
                        debug!("Got camera state {:?}", camera_state);
                        if *camera_state == CameraState::PoweringOn {
                            *camera_state = CameraState::PoweredOn;
                        }
                        if *camera_state == CameraState::PoweringOff {
                            *camera_state = CameraState::PoweredOff;
                            TC2_AGENT_STATE
                                .lock()
                                .unwrap()
                                .unset_flag(tc2_agent_state::READY);
                        }
                    }
                    {
                        let camera_state = CAMERA_STATE.lock().unwrap();
                        debug!("Got updated camera state {:?}", camera_state);
                    }
                }
                ATTINY_REG_RP2040_PI_POWER_CTRL => {
                    let power_state = ATTINY_POWER_CTRL_STATE.lock().unwrap();
                    payload[0] = *power_state;
                    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                    BigEndian::write_u16(&mut payload[1..=2], crc);
                }
                ATTINY_REG_TC2_AGENT_STATE => {
                    // Once we're powered on, we care about the tc2-agent state.
                    {
                        let agent_state = TC2_AGENT_STATE.lock().unwrap();
                        payload[0] = u8::from(*agent_state);
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    {
                        //Advance state
                        let mut agent_state = TC2_AGENT_STATE.lock().unwrap();
                        debug!("Got tc2-agent state {}", agent_state);
                        if !agent_state.flag_is_set(tc2_agent_state::READY) {
                            agent_state.set_flag(tc2_agent_state::READY);
                        }
                    }
                }
                _ => panic!("Unsupported attiny command 0x{:02x?}", request[0]),
            },
            _ => panic!("Unsupported i2c peripheral 0x{:02x?}", address),
        }
        Ok(())
    }
    pub fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), I2cError> {
        Ok(())
    }

    pub fn tx_fifo_used(&self) -> u32 {
        0
    }
}

pub enum PullType {
    PullUp,
    PullDown,
}

pub struct Pin {
    pull_type: PullType,
}

impl Pin {
    pub fn new(pull_type: PullType) -> Pin {
        Pin { pull_type }
    }

    pub fn is_low(&self) -> Result<bool, ()> {
        // FIXME: May want the ability for the pin to not be low?
        Ok(true)
    }

    pub fn into_pull_type(self, pull_type: PullType) -> Self {
        Self { pull_type }
    }
}

pub type I2cUnlockedPin = Pin;
pub type I2cLockedPin = Pin;

// Attiny + RTC comms
// NOTE: Early on in development we got strange errors when the raspberry pi was accessing the
//  attiny-provided i2c interface at the same time as we wanted to.  The hacky?/ingenious?
//  solution was to allocate a gpio pin that would determine who has the 'lock' on the i2c bus.
//  This is handled by this `SharedI2C` abstraction which mediates comms with the attiny/RTC.
pub struct MainI2C {
    pub unlocked_pin: Option<I2cUnlockedPin>,
    pub i2c: I2CConfig,
    pub delay: Timer,
}

use crate::attiny_rtc_i2c::{
    ATTINY_ADDRESS, ATTINY_REG_CAMERA_STATE, ATTINY_REG_KEEP_ALIVE,
    ATTINY_REG_RP2040_PI_POWER_CTRL, ATTINY_REG_TC2_AGENT_STATE, ATTINY_REG_VERSION, CRC_AUG_CCITT,
    CameraState, EEPROM_I2C_ADDRESS, RTC_ADDRESS, RTC_REG_ALARM_CONTROL, RTC_REG_ALARM_MINUTES,
    RTC_REG_DATETIME_SECONDS, decode_bcd, tc2_agent_state,
};
use crate::re_exports::bsp::pac::{I2C0, I2C1, RESETS};
use crate::re_exports::log::{debug, error, info, warn};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use byteorder::{BigEndian, ByteOrder};
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};
use crc::Crc;
use fugit::{HertzU32, Rate};
use std::time::Instant;

#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// I2C abort with error
    Abort(u32),
    /// User passed in a read buffer that was 0 length
    ///
    /// This is a limitation of the RP2040 I2C peripheral.
    /// If the slave ACKs its address, the I2C peripheral must read
    /// at least one byte before sending the STOP condition.
    InvalidReadBufferLength,
    /// User passed in a write buffer that was 0 length
    ///
    /// This is a limitation of the RP2040 I2C peripheral.
    /// If the slave ACKs its address, the I2C peripheral must write
    /// at least one byte before sending the STOP condition.
    InvalidWriteBufferLength,
    /// Target i2c address is out of range
    AddressOutOfRange(u16),
    /// Target i2c address is reserved
    AddressReserved(u16),
}

pub struct I2C<A, B>((A, B));

#[derive(Debug)]
pub struct I2cError;
impl I2cError {
    pub fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl<A, B> I2C<I2C0, (A, B)> {
    pub(crate) fn i2c0(
        p0: I2C0,
        p1: A,
        p2: B,
        _p3: Rate<u32, 1, 1>,
        _p4: &mut RESETS,
        _p5: HertzU32,
    ) -> I2C<I2C0, (A, B)> {
        I2C((p0, (p1, p2)))
    }

    pub fn write(&mut self, _address: u8, _payload: &[u8]) -> Result<(), Error> {
        Ok(())
    }

    pub fn write_read(
        &mut self,
        _address: u8,
        _payload: &[u8],
        _dst: &mut [u8],
    ) -> Result<(), Error> {
        Ok(())
    }

    pub fn tx_fifo_used(&self) -> u32 {
        0
    }
}

impl<A, B> I2C<I2C1, (A, B)> {
    pub(crate) fn i2c1(
        p0: I2C1,
        p1: A,
        p2: B,
        _p3: Rate<u32, 1, 1>,
        _p4: &mut RESETS,
        _p5: HertzU32,
    ) -> I2C<I2C1, (A, B)> {
        I2C((p0, (p1, p2)))
    }

    pub fn write(&mut self, address: u8, payload: &[u8]) -> Result<(), I2cError> {
        // NOTE: In between writes we might want to advance the state of the external simulated devices.

        // We might actually want to write a state to some external device (rpi power on state)
        match address {
            ATTINY_ADDRESS => match payload[0] {
                ATTINY_REG_KEEP_ALIVE => match payload[1] {
                    0x01 => {
                        TEST_SIM_STATE.with(|s| {
                            let mut s = s.borrow_mut();
                            let current_time = s.current_time;
                            s.attiny_keep_alive = current_time;
                        });
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
                    TEST_SIM_STATE.with(|s| {
                        let mut s = s.borrow_mut();
                        s.attiny_power_ctrl_state = payload[1];
                        match payload[1] {
                            0x00 => {
                                // Tell rPi it can shutdown.
                                if s.camera_state == CameraState::PoweredOn {
                                    s.camera_state = CameraState::PoweringOff;
                                }
                            }
                            0x01 => {
                                // Tell rPi to wake up.
                                if s.camera_state == CameraState::PoweredOff {
                                    s.camera_state = CameraState::PoweringOn;
                                }
                            }
                            0x02 => {
                                // Tell attiny to shut US down.  Can we simulate this?
                                // Maybe it's the same as a restart, with the time advanced?
                                //*CAMERA_STATE.lock().unwrap() = CameraState::PoweringOf;

                                {
                                    // Advance to the next alarm time when we sleep.
                                    let now = s.current_time;
                                    if s.rtc_alarm_state.is_initialised() {
                                        let minutes =
                                            decode_bcd(s.rtc_alarm_state.minutes & 0b0111_1111);
                                        let hours =
                                            decode_bcd(s.rtc_alarm_state.hours & 0b0011_1111);
                                        let day = decode_bcd(s.rtc_alarm_state.day & 0b0011_1111);

                                        let naive_date = NaiveDate::from_ymd_opt(
                                            now.year(),
                                            now.month(),
                                            u32::from(day),
                                        )
                                        .unwrap();
                                        let naive_time = NaiveTime::from_hms_opt(
                                            u32::from(hours),
                                            u32::from(minutes),
                                            0,
                                        )
                                        .unwrap();
                                        let utc_datetime =
                                            NaiveDateTime::new(naive_date, naive_time).and_utc();
                                        // info!("Advancing to alarm time: {:?}", utc_datetime);
                                        s.current_time = utc_datetime;
                                        // Make sure the alarm is set to "has been triggered".
                                        s.rtc_alarm_state.enabled |= 0b0000_1000;
                                    } else {
                                        error!("Alarm not initialised");
                                    }
                                }
                            }
                            _ => panic!(
                                "Unsupported attiny power control state 0x{:02x?}",
                                payload[1]
                            ),
                        }
                    });
                }
                ATTINY_REG_TC2_AGENT_STATE => {
                    TEST_SIM_STATE.with(|s| s.borrow_mut().tc2_agent_state = payload[1].into());
                }
                _ => panic!("Unsupported attiny command 0x{:02x?}", payload[0]),
            },
            RTC_ADDRESS => match payload[0] {
                RTC_REG_ALARM_MINUTES => {
                    TEST_SIM_STATE.with(|s| {
                        let mut s = s.borrow_mut();
                        s.rtc_alarm_state.minutes = payload[1];
                        s.rtc_alarm_state.hours = payload[2];
                        s.rtc_alarm_state.day = payload[3];
                        s.rtc_alarm_state.weekday_alarm_mode = payload[4];
                    });
                }
                RTC_REG_ALARM_CONTROL => {
                    TEST_SIM_STATE.with(|s| s.borrow_mut().rtc_alarm_state.enabled = payload[1]);
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
                _ => {
                    // warn!("Unsupported EEPROM write: {:02x?}", request[0]);
                }
            },
            RTC_ADDRESS => {
                let current_time = TEST_SIM_STATE.with(|s| s.borrow().current_time);
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
                        payload[0] = TEST_SIM_STATE.with(|s| s.borrow().rtc_alarm_state.enabled);
                    }
                    RTC_REG_ALARM_MINUTES => {
                        TEST_SIM_STATE.with(|s| {
                            let s = s.borrow();
                            payload[0] = s.rtc_alarm_state.minutes;
                            payload[1] = s.rtc_alarm_state.hours;
                            payload[2] = s.rtc_alarm_state.day;
                            payload[3] = s.rtc_alarm_state.weekday_alarm_mode;
                        });
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
                    payload[0] =
                        TEST_SIM_STATE.with(|s| s.borrow().expected_attiny_firmware_version); // Set ATTINY FIRMWARE_VERSION to 1
                    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                    BigEndian::write_u16(&mut payload[1..=2], crc);
                }
                ATTINY_REG_CAMERA_STATE => {
                    // Write the camera state:
                    {
                        payload[0] = TEST_SIM_STATE.with(|s| s.borrow().camera_state) as u8;
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    // Maybe advance the simulation?
                    {
                        TEST_SIM_STATE.with(|s| {
                            let mut s = s.borrow_mut();
                            debug!("Got camera state {:?}", s.camera_state);
                            if s.camera_state == CameraState::PoweringOn {
                                s.camera_state = CameraState::PoweredOn;
                            }
                            if s.camera_state == CameraState::PoweringOff {
                                s.camera_state = CameraState::PoweredOff;
                                s.tc2_agent_state.unset_flag(tc2_agent_state::READY);
                                s.last_frame = None;
                                s.frame_num = 0;
                            }
                        });
                    }
                    {
                        debug!(
                            "Got updated camera state {:?}",
                            TEST_SIM_STATE.with(|s| s.borrow().camera_state)
                        );
                    }
                }
                ATTINY_REG_RP2040_PI_POWER_CTRL => {
                    payload[0] = TEST_SIM_STATE.with(|s| s.borrow().attiny_power_ctrl_state);
                    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                    BigEndian::write_u16(&mut payload[1..=2], crc);
                }
                ATTINY_REG_TC2_AGENT_STATE => {
                    // Once we're powered on, we care about the tc2-agent state.
                    {
                        payload[0] = u8::from(TEST_SIM_STATE.with(|s| s.borrow().tc2_agent_state));
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    {
                        //Advance state
                        TEST_SIM_STATE.with(|s| {
                            let mut s = s.borrow_mut();
                            debug!("Got tc2-agent state {}", s.tc2_agent_state);
                            if !s.tc2_agent_state.flag_is_set(tc2_agent_state::READY) {
                                s.tc2_agent_state.set_flag(tc2_agent_state::READY);
                            }
                        });
                    }
                }
                ATTINY_REG_CAMERA_CONNECTION => {}
                _ => panic!("Unsupported attiny command 0x{:02x?}", request[0]),
            },
            _ => panic!("Unsupported i2c peripheral 0x{:02x?}", address),
        }
        Ok(())
    }
    pub fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), I2cError> {
        Ok(())
    }

    pub fn tx_fifo_used(&self) -> u32 {
        0
    }
}

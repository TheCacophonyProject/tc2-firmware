// #![allow(warnings)]
#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]
mod attiny_rtc_i2c;
mod bsp;
mod byte_slice_cursor;
mod clock_utils;
mod core0_audio;
mod core0_task;
mod core1_sub_tasks;
mod core1_task;
mod cptv_encoder;
mod device_config;
mod event_logger;
mod ext_spi_transfers;
mod lepton;
mod motion_detector;
mod onboard_flash;
mod pdm_microphone;
mod pdmfilter;
mod rp2040_flash;
mod sun_times;
mod utils;
use crate::attiny_rtc_i2c::SharedI2C;
use crate::core0_audio::audio_task;
pub use crate::core0_task::frame_acquisition_loop;
use cortex_m::asm::nop;

use crate::core1_task::{core_1_task, wake_raspberry_pi, Core1Pins, Core1Task};
use crate::cptv_encoder::FRAME_WIDTH;
use crate::device_config::DeviceConfig;
use crate::lepton::{init_lepton_module, LeptonPins};
use crate::onboard_flash::extend_lifetime_generic;
use bsp::{
    entry,
    hal::{
        clocks::Clock,
        multicore::{Multicore, Stack},
        pac,
        sio::Sio,
    },
    pac::Peripherals,
};
use cortex_m::asm::wfe;
use rp2040_hal::rosc::RingOscillator;

use core::cell::RefCell;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::gpio::FunctionI2C;
use rp2040_hal::{Watchdog, I2C};
// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 10;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 12;
const ROSC_TARGET_CLOCK_FREQ_HZ_THERMAL: u32 = 150_000_000;

// got funny results at 150 for aduio seems to work better at 120
const ROSC_TARGET_CLOCK_FREQ_HZ_AUDIO: u32 = 120_000_000;

const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins between FFCs
pub type FramePacketData = [u8; FRAME_WIDTH];
pub type FrameSegments = [[FramePacketData; 61]; 4];
const TRANSFER_HEADER_LENGTH: usize = 18;
pub struct FrameBuffer([u8; TRANSFER_HEADER_LENGTH + (160 * 61 * 4) + 2]);

impl FrameBuffer {
    pub const fn new() -> FrameBuffer {
        // NOTE: Put an 18 byte padding at the start, and a 2 byte padding at the end, to make it 32bit aligned
        FrameBuffer([0u8; TRANSFER_HEADER_LENGTH + (160 * 61 * 4) + 2])
    }

    pub fn as_u8_slice(&self) -> &[u8] {
        &self.0[TRANSFER_HEADER_LENGTH..TRANSFER_HEADER_LENGTH + 39040]
    }

    pub fn as_u8_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }

    pub fn frame_data_as_u8_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0[TRANSFER_HEADER_LENGTH..TRANSFER_HEADER_LENGTH + 39040]
    }

    pub fn packet(&mut self, segment: usize, packet_id: usize) -> &mut [u8] {
        let segment_offset = FRAME_WIDTH * 61 * segment;
        let packet_offset = FRAME_WIDTH * packet_id;
        &mut self.0[TRANSFER_HEADER_LENGTH..]
            [segment_offset + packet_offset..segment_offset + packet_offset + FRAME_WIDTH]
    }
}
// use rp2040_hal::pac;
use crate::core1_task::{advise_raspberry_pi_it_may_shutdown, SyncedDateTime};
use crate::device_config::get_naive_datetime;
use crate::onboard_flash::{extend_lifetime_generic_mut, OnboardFlash};

#[entry]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);
    // let mut core1stack: Stack<45000> = Stack::new(); // 174,000 bytes

    // TODO: Check wake_en and sleep_en registers to make sure we're not enabling any clocks we don't need.
    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    let is_audio = DeviceConfig::is_audio_device();
    let freq;
    if is_audio {
        freq = ROSC_TARGET_CLOCK_FREQ_HZ_AUDIO.Hz();
    } else {
        freq = ROSC_TARGET_CLOCK_FREQ_HZ_THERMAL.Hz();
    }
    let (clocks, rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        freq,
    );
    // let clocks = clock_utils::normal_clock();

    let clocks: &'static ClocksManager = unsafe { extend_lifetime_generic(&clocks) };

    let system_clock_freq = clocks.system_clock.freq().to_Hz();

    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );

    // Watchdog ticks are required to run the timer peripheral, since they're shared between both.

    let mut watchdog = bsp::hal::Watchdog::new(peripherals.WATCHDOG);
    watchdog.enable_tick_generation((system_clock_freq / 1_000_000) as u8);

    watchdog.pause_on_debug(true);
    watchdog.start(8388607.micros());

    info!("Enabled watchdog timer");
    let mut timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = Delay::new(core.SYST, system_clock_freq);
    let mut sio = Sio::new(peripherals.SIO);
    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // Attiny + RTC comms
    let sda_pin = pins.gpio6.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_function::<FunctionI2C>();
    let i2c1 = I2C::i2c1(
        peripherals.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );
    for i in 0..30 {
        info!("Waiting {}", 30 - i);
        watchdog.feed();
        delay.delay_ms(1000);
    }

    info!("Initing shared i2c");
    let mut shared_i2c = SharedI2C::new(i2c1, &mut delay);
    info!("Got shared i2c");
    let alarm_woke_us = shared_i2c.alarm_triggered(&mut delay);
    info!("Woken by RTC alarm? {}", alarm_woke_us);
    if alarm_woke_us {
        shared_i2c.clear_alarm();
    }
    shared_i2c.disable_alarm(&mut delay);

    let mut synced_date_time = SyncedDateTime::default();
    match shared_i2c.get_datetime(&mut delay) {
        Ok(now) => {
            info!("Date time {}:{}:{}", now.hours, now.minutes, now.seconds);
            synced_date_time.set(get_naive_datetime(now), &timer);
        }
        Err(_) => error!("Unable to get DateTime from RTC"),
    }
    shared_i2c.enable_alarm(&mut delay);

    let wake_in: i32 = 60 * 2;

    let wakeup = synced_date_time.date_time_utc + chrono::Duration::seconds(wake_in as i64);
    if let Ok(_) = shared_i2c.set_wakeup_alarm(&wakeup, &mut delay) {
        let alarm_enabled = shared_i2c.alarm_interrupt_enabled();
        info!("Wake up alarm interrupt enabled {}", alarm_enabled);
        info!(
            "Recording scheduled for {}:{} enabled ??{}",
            shared_i2c.get_alarm_hours(),
            shared_i2c.get_alarm_minutes(),
            alarm_enabled,
        );
    } else {
        error!("Failed setting wake alarm, can't go to sleep");
    }
    watchdog.feed();
    delay.delay_ms(1000);
    watchdog.feed();
    loop {
        advise_raspberry_pi_it_may_shutdown(&mut shared_i2c, &mut delay);
        if let Ok(pi_is_powered_down) = shared_i2c.pi_is_powered_down(&mut delay, true) {
            if pi_is_powered_down {
                let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);

                //just incase it triggers here
                if alarm_triggered {
                    warn!("Alarm triggered after taking a recording resetting rp2040");
                    loop {
                        // wait for watchdog to reset rp2040
                        wfe();
                    }
                }
                info!("SLEEPING");

                if let Ok(_) = shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay) {
                } else {
                    error!("Coudln't sleep")
                }
            }
        }
        let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);
        if alarm_triggered {
            warn!("Alarm triggered after taking a recording resetting rp2040");
            loop {
                // wait for watchdog to reset rp2040
                wfe();
            }
        }
        delay.delay_ms(5 * 1000);
        watchdog.feed();
    }
}

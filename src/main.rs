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
use crate::core1_task::{core_1_task, Core1Pins, Core1Task};
use crate::cptv_encoder::FRAME_WIDTH;
use crate::lepton::{init_lepton_module, LeptonPins};
use crate::motion_detector::DetectionMask;
use crate::onboard_flash::extend_lifetime_generic;
use attiny_rtc_i2c::tc2_agent_state;
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
use chrono::NaiveDateTime;
use cortex_m::asm::nop;
use device_config::{get_naive_datetime, AudioMode, DeviceConfig};
use rp2040_hal::rosc::RingOscillator;

use crate::onboard_flash::{extend_lifetime_generic_mut, extend_lifetime_generic_mut_2};
use bsp::hal::watchdog::Watchdog;
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
use rp2040_hal::gpio::{FunctionI2C, FunctionSio, PullDown, SioInput};
use rp2040_hal::I2C;
// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 14;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version. // TODO Check against minor version also.
                                                    // const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 150_000_000;

// got funny results at 150 for aduio seems to work better at 125
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;

const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins between FFCs
pub type FramePacketData = [u8; FRAME_WIDTH];
pub type FrameSegments = [[FramePacketData; 61]; 4];
const TRANSFER_HEADER_LENGTH: usize = 18;

#[repr(C, align(32))]
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

    pub fn ffc_imminent(&mut self, is_imminent: bool) {
        self.0[TRANSFER_HEADER_LENGTH + 636] = if is_imminent { 1 } else { 0 };
        self.0[TRANSFER_HEADER_LENGTH + 637] = if is_imminent { 1 } else { 0 };
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
use crate::byte_slice_cursor::Cursor;
use crate::rp2040_flash::read_device_config_from_rp2040_flash;
use embedded_io::Read;

#[entry]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);
    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    //this uses 5Kib
    let config = DeviceConfig::load_existing_config_from_flash();
    let config = config.unwrap();
    loop {
        info!(
            "Extra used is config is {} mask {}",
            config.config(),
            config.motion_detection_mask
        );
        nop();
    }

    //this also use 5kib
    let slice = read_device_config_from_rp2040_flash();
    let inner = DeviceConfig::inner_from_bytes(slice);

    // if inner.is_none() {
    //     // Device config is uninitialised in flash
    //     return None;
    // }
    let (inner, mut cursor_pos) = inner.unwrap();
    // let mask_length = cursor.read_i32();
    let mut cursor = Cursor::new(slice);
    cursor.set_position(cursor_pos);
    let mut motion_detection_mask;
    motion_detection_mask = DetectionMask::new(Some([0u8; 2400]));

    //thius line is what uses extra 2.46Kib
    let len = cursor.read(&mut motion_detection_mask.inner).unwrap();

    if len != motion_detection_mask.inner.len() {
        // This payload came without the mask attached (i.e. from the rPi)
        motion_detection_mask.set_empty();
    } else {
        cursor_pos = cursor.position();
    }

    let config = Some(DeviceConfig {
        config_inner: inner,
        motion_detection_mask,
        cursor_position: cursor_pos,
    });
    let config = config.unwrap();
    loop {
        info!(
            "Extra used is config is {} mask {}",
            config.config(),
            config.motion_detection_mask
        );
        nop();
    }

    //this uses 2.48KIB
    let config: Option<(device_config::DeviceConfigInner, usize)> =
        DeviceConfig::load_existing_inner_config_from_flash();
    let mask = DetectionMask::new(Some([0u8; 2400]));

    if config.is_none() {
        // Device config is uninitialised in flash
        info!("was none");
    }
    let (unwrapped, cursor) = config.unwrap();

    let config = Some(DeviceConfig {
        config_inner: unwrapped,
        motion_detection_mask: mask,
        cursor_position: cursor,
    });
    let config = config.unwrap();
    loop {
        info!(
            "Extra used is config is {} mask {}",
            config.config(),
            config.motion_detection_mask
        );
        nop();
    }

    let mut is_audio = false;
    // let mut is_audio: bool = config.is_some() && config.as_mut().unwrap().0.is_audio_device();
    let (clocks, rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        ROSC_TARGET_CLOCK_FREQ_HZ.Hz(),
    );
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
    let timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS, clocks);

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = Delay::new(core.SYST, system_clock_freq);
    let sio = Sio::new(peripherals.SIO);

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

    info!("Initing shared i2c");
    // We need to get the GPIO pin for determining who is using the I2C bus.
    let unlocked_pin = pins
        .gpio3
        .into_function::<FunctionSio<SioInput>>()
        .into_pull_type::<PullDown>();

    let mut shared_i2c = SharedI2C::new(i2c1, unlocked_pin, &mut delay);
    info!("Got shared i2c");
    let alarm_woke_us = shared_i2c.alarm_triggered(&mut delay);
    info!("Woken by RTC alarm? {}", alarm_woke_us);
    if alarm_woke_us {
        shared_i2c.clear_alarm(&mut delay);
    }

    if let Ok(audio_only) = shared_i2c.is_audio_device(&mut delay) {
        info!("EEPROM audio device: {}", audio_only);
        is_audio = is_audio || audio_only;
    }

    if !is_audio {
        let disabled_alarm = shared_i2c.disable_alarm(&mut delay);
        if disabled_alarm.is_err() {
            error!("{}", disabled_alarm.unwrap());
        }
    }

    let date_time: NaiveDateTime;
    match shared_i2c.get_datetime(&mut delay) {
        Ok(now) => {
            info!("Date time {}:{}:{}", now.hours, now.minutes, now.seconds);
            date_time = get_naive_datetime(now)
        }
        Err(_) => crate::panic!("Unable to get DateTime from RTC"),
    }

    // if is_audio {
    //     let config = config.as_mut().unwrap().config();
    //     match config.audio_mode {
    //         AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
    //             if let Ok(state) = shared_i2c.tc2_agent_state(&mut delay) {
    //                 if (state & (tc2_agent_state::THERMAL_MODE)) > 0 {
    //                     let _ = shared_i2c.tc2_agent_clear_thermal_mode(&mut delay);
    //                     info!("Audio request thermal mode");
    //                     //audio mode wants to go in thermal mode
    //                     is_audio = false;
    //                 } else {
    //                     let (start_time, end_time) =
    //                         config.next_or_current_recording_window(&date_time);

    //                     let in_window = config.time_is_in_recording_window(&date_time, &None);
    //                     if in_window {
    //                         is_audio = (state
    //                             & (tc2_agent_state::LONG_AUDIO_RECORDING
    //                                 | tc2_agent_state::TAKE_AUDIO
    //                                 | tc2_agent_state::TEST_AUDIO_RECORDING))
    //                             > 0;
    //                         if is_audio {
    //                             info!("Is audio because thermal requested or test rec");
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         _ => (),
    //     }
    // }
    let (i2c1, unlocked_pin) = shared_i2c.free();
    if is_audio {
        let gpio0 = pins.gpio0;
        let gpio1 = pins.gpio1;
        let pins = Core1Pins {
            pi_ping: pins.gpio5.into_pull_down_input(),

            pi_miso: pins.gpio15.into_floating_disabled(),
            pi_mosi: pins.gpio12.into_floating_disabled(),
            pi_cs: pins.gpio13.into_floating_disabled(),
            pi_clk: pins.gpio14.into_floating_disabled(),

            fs_cs: pins.gpio9.into_push_pull_output(),
            fs_miso: pins.gpio8.into_pull_down_disabled().into_pull_type(),
            fs_mosi: pins.gpio11.into_pull_down_disabled().into_pull_type(),
            fs_clk: pins.gpio10.into_pull_down_disabled().into_pull_type(),
        };

        audio_branch(
            i2c1,
            system_clock_freq,
            timer,
            pins,
            gpio0,
            gpio1,
            watchdog,
            alarm_woke_us,
            unlocked_pin,
        );
    } else {
        let lepton_pins = LeptonPins {
            tx: pins.gpio23.into_function(),
            rx: pins.gpio20.into_function(),
            clk: pins.gpio22.into_function(),
            cs: pins.gpio21.into_function(),

            vsync: pins.gpio19.into_function(),

            sda: pins.gpio24.into_function(),
            scl: pins.gpio25.into_function(),

            power_down: pins.gpio28.into_push_pull_output(),
            power_enable: pins.gpio18.into_push_pull_output(),
            reset: pins.gpio29.into_push_pull_output(),
            clk_disable: pins.gpio27.into_push_pull_output(),
            master_clk: pins.gpio26.into_floating_input(),
        };
        let pins = Core1Pins {
            pi_ping: pins.gpio5.into_pull_down_input(),

            pi_miso: pins.gpio15.into_floating_disabled(),
            pi_mosi: pins.gpio12.into_floating_disabled(),
            pi_cs: pins.gpio13.into_floating_disabled(),
            pi_clk: pins.gpio14.into_floating_disabled(),

            fs_cs: pins.gpio9.into_push_pull_output(),
            fs_miso: pins.gpio8.into_pull_down_disabled().into_pull_type(),
            fs_mosi: pins.gpio11.into_pull_down_disabled().into_pull_type(),
            fs_clk: pins.gpio10.into_pull_down_disabled().into_pull_type(),
        };
        loop {
            nop();
        }

        // thermal_code(
        //     lepton_pins,
        //     pins,
        //     watchdog,
        //     system_clock_freq,
        //     delay,
        //     timer,
        //     i2c1,
        //     clocks,
        //     rosc,
        //     alarm_woke_us,
        //     unlocked_pin,
        //     config.unwrap(),
        // );
    }
}
use crate::attiny_rtc_i2c::I2CConfig;

pub fn audio_branch(
    i2c_config: I2CConfig,
    clock_freq: u32,
    mut timer: bsp::hal::Timer,
    pins: Core1Pins,
    gpio0: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio0,
        rp2040_hal::gpio::FunctionNull,
        rp2040_hal::gpio::PullDown,
    >,
    gpio1: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio1,
        rp2040_hal::gpio::FunctionNull,
        rp2040_hal::gpio::PullDown,
    >,
    mut watchdog: bsp::hal::Watchdog,
    alarm_triggered: bool,
    unlocked_pin: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio3,
        FunctionSio<SioInput>,
        PullDown,
    >,
) -> ! {
    audio_task(
        i2c_config,
        clock_freq,
        &mut timer,
        pins,
        gpio0,
        gpio1,
        &mut watchdog,
        alarm_triggered,
        unlocked_pin,
    );
}
pub fn thermal_code(
    lepton_pins: LeptonPins,
    pins: Core1Pins,
    mut watchdog: Watchdog,
    system_clock_freq: u32,
    mut delay: Delay,
    timer: bsp::hal::Timer,
    i2c1: I2CConfig,
    clocks: &ClocksManager,
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    alarm_woke_us: bool,
    unlocked_pin: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio3,
        FunctionSio<SioInput>,
        PullDown,
    >,
    device_config: DeviceConfig,
) -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut sio = Sio::new(peripherals.SIO);
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);

    let cores = mc.cores();
    let core1 = &mut cores[1];
    let mut lepton = init_lepton_module(
        peripherals.SPI0,
        peripherals.I2C0,
        system_clock_freq,
        &mut peripherals.RESETS,
        &mut delay,
        lepton_pins,
    );

    let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    let lepton_serial = lepton.get_camera_serial().map_or(None, |x| Some(x));
    let lepton_firmware_version = lepton.get_firmware_version().map_or(None, |x| Some(x));
    if let Some(((m_major, m_minor, m_build), (d_major, d_minor, d_build))) =
        lepton_firmware_version
    {
        info!(
            "Camera firmware versions: main: {}.{}.{}, dsp: {}.{}.{}",
            m_major, m_minor, m_build, d_major, d_minor, d_build
        );
    }
    info!("Camera serial #{}", lepton_serial);
    info!("Radiometry enabled? {}", radiometric_mode);

    let mut fb0 = FrameBuffer::new();
    let mut fb1 = FrameBuffer::new();
    let mut core1_stack: Stack<44250> = Stack::new();
    let frame_buffer = Mutex::new(RefCell::new(Some(unsafe {
        extend_lifetime_generic_mut(&mut fb0)
    })));
    let frame_buffer_2 = Mutex::new(RefCell::new(Some(unsafe {
        extend_lifetime_generic_mut(&mut fb1)
    })));
    // Shenanigans to convince the second thread that all these values exist for the lifetime of the
    // program.
    let frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>> =
        unsafe { extend_lifetime_generic(&frame_buffer) };
    let frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>> =
        unsafe { extend_lifetime_generic(&frame_buffer_2) };
    watchdog.feed();
    watchdog.disable();
    info!(
        "id address is {:#x}",
        &device_config.config().device_id as *const _ as usize
    );
    let device_config = unsafe { extend_lifetime_generic(&device_config) };
    let config = RefCell::new(device_config);

    let peripheral_clock_freq = clocks.peripheral_clock.freq();
    {
        let _ = core1.spawn(
            unsafe { extend_lifetime_generic_mut_2(&mut core1_stack.mem) },
            move || {
                core_1_task(
                    frame_buffer_local,
                    frame_buffer_local_2,
                    system_clock_freq,
                    pins,
                    i2c1,
                    unlocked_pin,
                    lepton_serial,
                    lepton_firmware_version,
                    alarm_woke_us,
                    timer,
                    // config,
                )
            },
        );
    }
    let result = sio.fifo.read_blocking();
    crate::assert_eq!(result, Core1Task::Ready.into());
    sio.fifo
        .write_blocking(if radiometric_mode { 2 } else { 1 });

    let result = sio.fifo.read_blocking();
    if result == Core1Task::RequestReset.into() {
        watchdog.start(100.micros());
        loop {
            nop();
        }
    }
    crate::assert_eq!(result, Core1Task::Ready.into());

    frame_acquisition_loop(
        rosc,
        &mut lepton,
        &mut sio.fifo,
        peripheral_clock_freq,
        &mut delay,
        &mut peripherals.RESETS,
        frame_buffer_local,
        frame_buffer_local_2,
        watchdog,
    );
}

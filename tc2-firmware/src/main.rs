#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]

mod lepton;
mod utils;

mod bsp;
mod clock_utils;
mod core0_task;
mod core1_task;
mod cptv_encoder;
mod double_frame_buffer;
mod ext_spi_transfers;
mod motion_detector;
mod onboard_flash;

pub use crate::core0_task::begin_frame_acquisition_loop;
use crate::core1_task::{core_1_task, Core1Pins, Core1Task};
use crate::cptv_encoder::FRAME_WIDTH;
use crate::lepton::{init_lepton_module, LeptonPins};
use crate::onboard_flash::extend_lifetime_generic;
use crate::utils::any_as_u8_slice;
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
use core::cell::RefCell;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::*;

use defmt_rtt as _;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
use fugit::RateExtU32;
use panic_probe as _;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::gpio::FunctionI2C;
use rp2040_hal::I2C;
// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub static FIRMWARE_VERSION: u32 = 3;
static mut CORE1_STACK: Stack<43500> = Stack::new(); // 174,000 bytes
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 150_000_000;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins between FFCs
pub type FramePacketData = [u8; FRAME_WIDTH];
pub struct FrameSeg([FramePacketData; 61]);

impl FrameSeg {
    pub const fn new() -> FrameSeg {
        FrameSeg([[0u8; FRAME_WIDTH]; 61])
    }

    pub fn as_u8_slice(&self) -> &[u8] {
        unsafe { any_as_u8_slice(&self.0) }
    }
    pub fn packet(&mut self, packet_id: usize) -> &mut FramePacketData {
        &mut self.0[packet_id]
    }
}

#[entry]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);

    // TODO: Check wake_en and sleep_en registers to make sure we're not enabling any clocks we don't need.
    let mut peripherals = Peripherals::take().unwrap();
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

    let core = pac::CorePeripherals::take().unwrap();
    let mut sio = Sio::new(peripherals.SIO);

    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);

    let cores = mc.cores();
    let core1 = &mut cores[1];

    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    let frame_buffer = Mutex::new(RefCell::new([
        FrameSeg::new(),
        FrameSeg::new(),
        FrameSeg::new(),
        FrameSeg::new(),
    ]));
    // Shenanigans to convince the second thread that all these values exist for the lifetime of the
    // program.
    let frame_buffer_local: &'static Mutex<RefCell<[FrameSeg; 4]>> =
        unsafe { extend_lifetime_generic(&frame_buffer) };
    {
        let pins = Core1Pins {
            pi_ping: pins.gpio5.into_push_pull_output(),

            pi_miso: pins.gpio15.into_floating_disabled(),
            pi_mosi: pins.gpio12.into_floating_disabled(),
            pi_cs: pins.gpio13.into_floating_disabled(),
            pi_clk: pins.gpio14.into_floating_disabled(),

            fs_cs: pins.gpio9.into_push_pull_output(),
            fs_miso: pins.gpio8.into_pull_down_disabled().into_pull_type(),
            fs_mosi: pins.gpio11.into_pull_down_disabled().into_pull_type(),
            fs_clk: pins.gpio10.into_pull_down_disabled().into_pull_type(),
        };
        let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            core_1_task(frame_buffer_local, system_clock_freq, pins)
        });
    }

    let mut delay = Delay::new(core.SYST, system_clock_freq);

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
    let mut lepton = init_lepton_module(
        peripherals.SPI0,
        peripherals.I2C0,
        system_clock_freq,
        &mut peripherals.RESETS,
        &mut delay,
        lepton_pins,
    );

    //let mut wake_interrupt_pin = pins.gpio4.into_floating_input();

    let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    info!("Radiometry enabled? {}", radiometric_mode);
    let result = sio.fifo.read_blocking();
    crate::assert_eq!(result, Core1Task::Ready.into());
    sio.fifo
        .write_blocking(if radiometric_mode { 1 } else { 0 });

    let result = sio.fifo.read_blocking();
    crate::assert_eq!(result, Core1Task::Ready.into());

    // Attiny + RTC comms
    let sda_pin = pins.gpio6.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_function::<FunctionI2C>();
    let mut i2c = I2C::i2c1(
        peripherals.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );
    // Write three bytes to the I²C device with 7-bit address 0x2C
    // for i in 0u8..127u8 {
    //     if i2c.write(i, &[1, 2, 3]).is_ok() {
    //         info!("Found i2c device at {:#x}", i);
    //     }
    // }
    let mut attiny_regs = [0u8; 24];
    if i2c.read(0x25, &mut attiny_regs).is_ok() {
        if attiny_regs[1] == 2 {
            info!("Should power off");
        }
        info!("Attiny camera state {:?}", attiny_regs);
    } else {
        info!("Failed to read i2c state from attiny");
    }
    // let mut cmd = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    // i2c.write_read(0x51, &[
    //     PCF8563_SEC_REG,
    //     PCF8563_MIN_REG,
    //     PCF8563_HR_REG,
    //     PCF8563_DAY_REG,
    //     PCF8563_MONTH_REG,
    //     PCF8563_YEAR_REG
    // ], &mut cmd).unwrap();
    // for (i, (unit, mask)) in [
    //     ("Second", 0x7f),
    //     ("Minute", 0x7f),
    //     ("Hour", 0x3f),
    //     ("Day", 0x07),
    //     ("Month", 0x1F),
    //     ("Year", 0xff)
    // ].iter().enumerate() {
    //     info!("#{}: {}: {}, {}", i, unit, bcd2dec(cmd[i] & mask), bcd2dec(cmd[i]));
    // }
    let i2c_poll_counter = 0;

    begin_frame_acquisition_loop(
        rosc,
        &mut lepton,
        &mut sio.fifo,
        &clocks,
        &mut delay,
        &mut peripherals.RESETS,
        frame_buffer_local,
    );
}

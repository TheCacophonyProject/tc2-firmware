#![no_std]
#![no_main]
pub mod bsp;
mod utils;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::_embedded_hal_blocking_spi_Transfer;
use embedded_hal::spi::{Mode, MODE_3};
use fugit::{HertzU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::gpio::DynPinMode::Function;
use rp2040_hal::gpio::FunctionSpi;

use crate::bsp::Pins;
use bsp::{
    entry,
    hal::{
        clocks::{Clock, ClockSource, ClocksManager, StoppableClock},
        multicore::{Multicore, Stack},
        pac,
        rosc::RingOscillator,
        sio::Sio,
        xosc::setup_xosc_blocking,
        Spi,
    },
    pac::Peripherals,
    XOSC_CRYSTAL_FREQ,
};

#[entry]
fn main() -> ! {
    // Setup clocks etc.
    let rosc_clock_freq: HertzU32 = 150_000_000.Hz(); //52.76mA, 25Mhz spi clock
    let lepton_spi_clock_freq: HertzU32 = 40_000_000.Hz();
    let mut peripherals = Peripherals::take().unwrap();
    let xosc = match setup_xosc_blocking(peripherals.XOSC, XOSC_CRYSTAL_FREQ.Hz()) {
        Ok(xosc) => xosc,
        Err(_) => crate::panic!("xosc"),
    };

    let mut clocks = ClocksManager::new(peripherals.CLOCKS);
    clocks
        .reference_clock
        .configure_clock(&xosc, XOSC_CRYSTAL_FREQ.Hz())
        .unwrap();

    let measured_rosc_frequency =
        utils::find_target_rosc_frequency(&peripherals.ROSC, rosc_clock_freq.to_Hz());
    let rosc = RingOscillator::new(peripherals.ROSC);
    let mut rosc = rosc.initialize_with_freq(measured_rosc_frequency);
    let measured_hz: HertzU32 = measured_rosc_frequency.Hz();

    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    let _xosc_disabled = xosc.disable();
    clocks.usb_clock.disable();
    clocks.gpio_output0_clock.disable();
    clocks.gpio_output1_clock.disable();
    clocks.gpio_output2_clock.disable();
    clocks.gpio_output3_clock.disable();
    clocks.adc_clock.disable();
    clocks.rtc_clock.disable();
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    let sio = Sio::new(peripherals.SIO);
    let pins = Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // Setup SPI
    let _mosi = pins.gpio12.into_mode::<FunctionSpi>();
    let _sck = pins.gpio14.into_mode::<FunctionSpi>();
    let _miso = pins.gpio15.into_mode::<FunctionSpi>();
    let _cs = pins.gpio13.into_mode::<FunctionSpi>();

    let (mut spi, rate) =
        Spi::<_, _, 8>::new(peripherals.SPI1).init_slave(&mut peripherals.RESETS, &MODE_3);

    let mut buf = [0u8; 1];
    info!("Start polling spi");
    loop {
        let response = spi.transfer(&mut buf).unwrap();
        info!("got back {}", response[0]);
    }
}

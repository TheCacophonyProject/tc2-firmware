#![cfg_attr(feature = "no-std", no_std)]
#![cfg(feature = "no-std")]
#![no_main]
#![warn(clippy::all, clippy::pedantic)]
mod attiny_rtc_i2c;
mod audio_task;
pub mod bsp;
mod byte_slice_cursor;
mod clock_utils;
mod constants;
mod cptv_encoder;
mod device_config;
mod entry;
mod event_logger;
mod ext_spi_transfers;
mod formatted_time;
mod frame_processing;
mod lepton;
mod lepton_task;
mod lepton_telemetry;
mod motion_detector;
mod onboard_flash;
mod pdm_filter;
mod pdm_microphone;
mod re_exports;
mod rpi_power;
mod startup_functions;
mod sub_tasks;
mod sun_times;
mod synced_date_time;
mod tests;
mod utils;

use crate::entry::real_main;
use crate::re_exports::bsp::entry;
use crate::re_exports::bsp::hal::Watchdog;
use crate::re_exports::bsp::pac::Peripherals;
use crate::utils::restart;
use defmt_rtt as _;
use panic_probe as _;

#[entry]
fn main() -> ! {
    real_main();
    let peripherals = unsafe { Peripherals::steal() };
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    watchdog.enable_tick_generation(130);
    restart(&mut watchdog);
}

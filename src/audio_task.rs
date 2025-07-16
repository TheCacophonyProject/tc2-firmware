use crate::attiny_rtc_i2c::{MainI2C, RecordingRequestType};
use crate::bsp;
use crate::bsp::pac::{DMA, PIO1, Peripherals, RESETS};
use crate::device_config::DeviceConfig;
use crate::event_logger::{Event, EventLogger};
use crate::onboard_flash::OnboardFlash;
use crate::pdm_microphone::PdmMicrophone;
use byteorder::{ByteOrder, LittleEndian};
use defmt::{error, info, warn};
use rp2040_hal::gpio;

use chrono::{Duration, TimeDelta};
use crc::{CRC_16_XMODEM, Crc};
use fugit::HertzU32;

use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::synced_date_time::SyncedDateTime;
use crate::utils::restart;
use bsp::hal::Watchdog;
use gpio::FunctionNull;
use gpio::bank0::{Gpio0, Gpio1};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::PullDown;
use rp2040_hal::pio::PIO;
use rp2040_hal::pio::SM1;
use rp2040_hal::pio::UninitStateMachine;

pub const DEV_MODE: bool = false;
pub const MAX_INTERVAL_BETWEEN_AUDIO_RECORDINGS: TimeDelta = Duration::minutes(60);

fn send_camera_connect_info(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    dma: &mut DMA,
    resets: &mut RESETS,
) {
    if let Some(free_spi) = fs.free_spi() {
        let mut payload = [0u8; 16];
        pi_spi.enable(free_spi, resets);
        LittleEndian::write_u32(&mut payload[12..16], 1); // Audio mode
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        info!("Sending camera connect info");
        let success = pi_spi.send_message_over_spi(
            ExtTransferMessage::CameraConnectInfo,
            &payload,
            crc,
            dma,
            resets,
            None,
        );
        if let Some(spi) = pi_spi.disable() {
            fs.take_spi(spi, resets);
        }
    }
}

#[allow(clippy::too_many_lines)]
pub fn record_audio(
    mut i2c: MainI2C,
    system_clock_freq: HertzU32,
    gpio0: gpio::Pin<Gpio0, FunctionNull, PullDown>,
    gpio1: gpio::Pin<Gpio1, FunctionNull, PullDown>,
    mut watchdog: Watchdog,
    recording_request_type: RecordingRequestType,
    config: &DeviceConfig,
    mut fs: OnboardFlash,
    mut events: EventLogger,
    mut pi_spi: ExtSpiTransfers,
    time: &SyncedDateTime,
    pio1: PIO<PIO1>,
    sm1: UninitStateMachine<(PIO1, SM1)>,
) -> ! {
    info!("=== Core 0 Audio Recording start ===");
    if DEV_MODE {
        warn!("DEV MODE");
    } else {
        warn!("FIELD MODE");
    }
    watchdog.feed();
    // Are we going to record now?
    if let Err(e) = i2c.started_recording() {
        error!("Error setting recording flag on attiny: {}", e);
    }
    let timestamp = time.get_timestamp_micros();
    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

    // Tell tc2-agent we're in audio mode
    send_camera_connect_info(
        &mut fs,
        &mut pi_spi,
        &mut peripherals.DMA,
        &mut peripherals.RESETS,
    );

    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);
    let mut microphone = PdmMicrophone::new(
        gpio0.into_function().into_pull_type(),
        gpio1.into_function().into_pull_type(),
        system_clock_freq,
        pio1,
        sm1,
    );
    events.log(Event::StartedAudioRecording, time, &mut fs);
    let recording_failed = !microphone.record_for_n_seconds(
        recording_request_type.duration_seconds(),
        dma_channels.ch3,
        dma_channels.ch4,
        &mut peripherals.RESETS,
        &peripherals.SPI1,
        &mut fs,
        timestamp,
        &mut watchdog,
        time,
        recording_request_type.is_user_requested(),
    );
    if let Err(e) = i2c.stopped_recording() {
        error!("Error unsetting recording flag on attiny: {}", e);
    }
    if recording_failed {
        events.log(Event::AudioRecordingFailed, time, &mut fs);
        info!("Recording failed, restarting and will try again");
    } else {
        // If the audio recording succeeded, we'll restart and possibly offload if either of:
        // a) this was a recording requested from thermal more in `AudioAndThermal` mode.
        // b) this was a user-requested test recording.
        events.log(Event::EndedRecording, time, &mut fs);
        watchdog.feed();
        if let Err(e) = i2c.tc2_agent_clear_mode_flags() {
            error!("Failed to clear mode flags {}", e);
        }
        if let Err(e) = i2c.clear_and_disable_alarm(time) {
            error!("Failed to clear and disable alarm: {}", e);
        }
    }
    restart(&mut watchdog)
}

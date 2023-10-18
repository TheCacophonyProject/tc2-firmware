use crate::bsp;
use crate::bsp::pac::rosc::ctrl::FREQ_RANGE_A;
use crate::bsp::pac::{Peripherals, CLOCKS, ROSC, XOSC};
use crate::bsp::XOSC_CRYSTAL_FREQ;
use defmt::info;
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::clocks::{ClockSource, ClocksManager, StoppableClock};
use rp2040_hal::rosc::RingOscillator;
use rp2040_hal::xosc::setup_xosc_blocking;
use rp2040_hal::Clock;

fn set_rosc_div(rosc: &ROSC, div: u32) {
    assert!(div <= 32);
    let div = if div == 32 { 0 } else { div };
    rosc.div.write(|w| unsafe { w.bits(0xaa0 + div) });
}

fn reset_rosc_operating_frequency(rosc: &ROSC) {
    // Set divider to 1
    set_rosc_div(rosc, 1);
    rosc.ctrl.write(|w| w.freq_range().low());
    write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
}

fn read_freq_stage(rosc: &ROSC, stage: u8) -> u8 {
    match stage {
        0 => rosc.freqa.read().ds0().bits(),
        1 => rosc.freqa.read().ds1().bits(),
        2 => rosc.freqa.read().ds2().bits(),
        3 => rosc.freqa.read().ds3().bits(),
        4 => rosc.freqb.read().ds4().bits(),
        5 => rosc.freqb.read().ds5().bits(),
        6 => rosc.freqb.read().ds6().bits(),
        7 => rosc.freqb.read().ds7().bits(),
        _ => panic!("invalid frequency drive strength stage"),
    }
}

/// Increase the ROSC drive strength bits for the current freq_range
fn increase_drive_strength(rosc: &ROSC) -> bool {
    const MAX_STAGE_DRIVE: u8 = 3;
    // Assume div is 1, and freq_range is high
    let mut stages: [u8; 8] = [0; 8];
    for (stage_index, stage) in stages.iter_mut().enumerate() {
        *stage = read_freq_stage(rosc, stage_index as u8)
    }
    let num_stages_at_drive_level = match rosc.ctrl.read().freq_range().variant() {
        Some(FREQ_RANGE_A::LOW) => 8,
        Some(FREQ_RANGE_A::MEDIUM) => 6,
        Some(FREQ_RANGE_A::HIGH) => 4,
        Some(FREQ_RANGE_A::TOOHIGH) => panic!("Don't use TOOHIGH freq_range"),
        None => {
            // Start out at initial unset drive stage
            return false;
        }
    };
    let mut next_i = 0;
    for (index, x) in stages[0..num_stages_at_drive_level].windows(2).enumerate() {
        if x[1] < x[0] {
            next_i = index + 1;
            break;
        }
    }
    if stages[next_i] < MAX_STAGE_DRIVE {
        stages[next_i] += 1;
        let min = *stages[0..num_stages_at_drive_level]
            .iter()
            .min()
            .unwrap_or(&0);
        for stage in &mut stages[num_stages_at_drive_level..] {
            *stage = min;
        }
        write_freq_stages(rosc, &stages);
        true
    } else {
        false
    }
}

/// Sets the `freqa` and `freqb` ROSC drive strength stage registers.
fn write_freq_stages(rosc: &ROSC, stages: &[u8; 8]) {
    let passwd: u32 = 0x9696 << 16;
    let mut freq_a = passwd;
    let mut freq_b = passwd;
    for (stage_index, stage) in stages.iter().enumerate().take(4) {
        freq_a |= ((*stage & 0x07) as u32) << (stage_index * 4);
    }
    for (stage_index, stage) in stages.iter().enumerate().skip(4) {
        freq_b |= ((*stage & 0x07) as u32) << ((stage_index - 4) * 4);
    }
    rosc.freqa.write(|w| unsafe { w.bits(freq_a) });
    rosc.freqb.write(|w| unsafe { w.bits(freq_b) });
}

/// Increase the rosc frequency range up to the next step.
/// Returns a boolean to indicate whether the frequency was increased.
fn increase_freq_range(rosc: &ROSC) -> bool {
    match rosc.ctrl.read().freq_range().variant() {
        None => {
            // Initial unset frequency range, move to LOW frequency range
            rosc.ctrl.write(|w| w.freq_range().low());
            // Reset all the drive strength bits.
            write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::LOW) => {
            // Transition from LOW to MEDIUM frequency range
            rosc.ctrl.write(|w| w.freq_range().medium());
            // Reset all the drive strength bits.
            write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::MEDIUM) => {
            // Transition from MEDIUM to HIGH frequency range
            rosc.ctrl.write(|w| w.freq_range().high());
            // Reset all the drive strength bits.
            write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::HIGH) | Some(FREQ_RANGE_A::TOOHIGH) => {
            // Already in the HIGH frequency range, and can't increase
            false
        }
    }
}

/// Measure the actual speed of the ROSC at the current freq_range and drive strength config
fn rosc_frequency_count_hz(clocks: &CLOCKS) -> HertzU32 {
    // Wait for the frequency counter to be ready
    while clocks.fc0_status.read().running().bit_is_set() {
        cortex_m::asm::nop();
    }

    // Set the speed of the reference clock in kHz.
    clocks
        .fc0_ref_khz
        .write(|w| unsafe { w.fc0_ref_khz().bits(XOSC_CRYSTAL_FREQ / 1000) });

    // Corresponds to a 1ms test time, which seems to give good enough accuracy
    clocks
        .fc0_interval
        .write(|w| unsafe { w.fc0_interval().bits(10) });

    // We don't really care about the min/max, so these are just set to min/max values.
    clocks
        .fc0_min_khz
        .write(|w| unsafe { w.fc0_min_khz().bits(0) });
    clocks
        .fc0_max_khz
        .write(|w| unsafe { w.fc0_max_khz().bits(0x1fff_ffff) });

    // To measure rosc directly we use the value 0x03.
    clocks.fc0_src.write(|w| unsafe { w.fc0_src().bits(0x03) });

    // Wait until the measurement is ready
    while clocks.fc0_status.read().done().bit_is_clear() {
        cortex_m::asm::nop();
    }

    let speed_hz = clocks.fc0_result.read().khz().bits() * 1000;
    speed_hz.Hz()
}

/// Resets ROSC frequency range and stages drive strength, then increases the frequency range,
/// drive strength bits, and finally divider in order to try to come close to the desired target
/// frequency, returning the final measured ROSC frequency attained.
fn find_target_rosc_frequency(
    rosc: &ROSC,
    clocks: &CLOCKS,
    target_frequency: HertzU32,
) -> HertzU32 {
    // Make sure xosc is set as the reference clock source.
    clocks.clk_ref_ctrl.write(|w| w.src().xosc_clksrc());

    reset_rosc_operating_frequency(rosc);
    let mut div = 1;
    let mut measured_rosc_frequency;
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz(clocks);
        // If it has overshot the target frequency, increase the divider and continue.
        if measured_rosc_frequency > target_frequency {
            div += 1;
            set_rosc_div(rosc, div);
        } else {
            break;
        }
    }
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz(clocks);
        if measured_rosc_frequency > target_frequency {
            // And probably want to step it down a notch?
            break;
        }
        let can_increase = increase_drive_strength(rosc);
        if !can_increase {
            let can_increase_range = increase_freq_range(rosc);
            if !can_increase_range {
                break;
            }
        }
    }
    measured_rosc_frequency
}

pub fn setup_rosc_as_system_clock(
    clocks_peripheral: CLOCKS,
    xosc_peripheral: XOSC,
    rosc_peripheral: ROSC,
    desired_rosc_freq: HertzU32,
) -> (ClocksManager, RingOscillator<bsp::hal::rosc::Enabled>) {
    // Setup the crystal oscillator to do accurate measurements against
    let peripherals = unsafe { Peripherals::steal() };
    let xosc = setup_xosc_blocking(xosc_peripheral, XOSC_CRYSTAL_FREQ.Hz()).unwrap();

    // Find appropriate settings for the desired ring oscillator frequency.
    let measured_rosc_frequency =
        find_target_rosc_frequency(&rosc_peripheral, &clocks_peripheral, desired_rosc_freq);
    let rosc = RingOscillator::new(rosc_peripheral);

    // Now initialise the ROSC with the reached frequency and set it as the system clock.
    let rosc = rosc.initialize_with_freq(measured_rosc_frequency);

    let mut clocks = ClocksManager::new(clocks_peripheral);
    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    clocks
        .reference_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    // Now we can disable the crystal oscillator and run off the ring oscillator, for power savings.
    let _xosc_disabled = xosc.disable();

    // NOTE: PLLs are disabled by default.
    // You may also wish to disable other clocks/peripherals that you don't need.
    clocks.usb_clock.disable();
    clocks.gpio_output0_clock.disable();
    clocks.gpio_output1_clock.disable();
    clocks.gpio_output2_clock.disable();
    clocks.gpio_output3_clock.disable();
    clocks.adc_clock.disable();
    clocks.rtc_clock.disable();

    (clocks, rosc)
}

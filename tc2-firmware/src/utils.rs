use crate::bsp::pac;
use crate::bsp::pac::rosc::ctrl::FREQ_RANGE_A;
use crate::bsp::pac::ROSC;
use crate::XOSC_CRYSTAL_FREQ;

pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    core::slice::from_raw_parts((p as *const T) as *const u8, core::mem::size_of::<T>())
}

pub unsafe fn u8_slice_to_u16(p: &[u8]) -> &[u16] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u16, p.len() / 2)
}

pub unsafe fn u16_slice_to_u8(p: &[u16]) -> &[u8] {
    core::slice::from_raw_parts((p as *const [u16]) as *const u8, p.len() * 2)
}

pub unsafe fn u16_slice_to_u8_mut(p: &mut [u16]) -> &mut [u8] {
    core::slice::from_raw_parts_mut((p as *mut [u16]) as *mut u8, p.len() * 2)
}

pub unsafe fn i32_slice_to_u8(p: &[i32]) -> &[u8] {
    core::slice::from_raw_parts((p as *const [i32]) as *const u8, p.len() * 4)
}

pub unsafe fn u8_slice_to_u32(p: &[u8]) -> &[u32] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u32, p.len() / 4)
}

pub unsafe fn any_as_u32_slice<T: Sized>(p: &T) -> &'static [u32] {
    core::slice::from_raw_parts((p as *const T) as *const u32, core::mem::size_of::<T>() / 4)
}

pub fn rosc_frequency_count_hz() -> u32 {
    // Use the reference xosc while enabled to measure the speed of the rosc.
    // We need to steal the clocks block which is already "owned" by the ClocksManager
    // struct, which already configured the xosc as the reference clock, and is needed
    // for this measurement to work correctly.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let clocks = peripherals.CLOCKS;

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
        .write(|w| unsafe { w.fc0_max_khz().bits(0xffffffff) });

    // To measure rosc directly we use the value 0x03.
    clocks.fc0_src.write(|w| unsafe { w.fc0_src().bits(0x03) });

    // Wait until the measurement is ready
    while clocks.fc0_status.read().done().bit_is_clear() {
        cortex_m::asm::nop();
    }

    let speed_hz = clocks.fc0_result.read().khz().bits() * 1000;
    speed_hz
}

fn set_rosc_div(rosc: &ROSC, div: u32) {
    assert!(div <= 32);
    let div = if div == 32 { 0 } else { div };
    rosc.div.write(|w| unsafe { w.bits(0xaa0 + div) });
}

fn reset_rosc_operating_frequency(rosc: &ROSC) {
    // Set divider to 1
    set_rosc_div(rosc, 1);
    rosc.ctrl.write(|w| w.freq_range().low());
    write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
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

const MAX_STAGE_DRIVE: u8 = 3;

fn increase_freq(rosc: &ROSC) -> bool {
    // Assume div is 1, and freq_range is high

    let mut stages: [u8; 8] = [0; 8];
    // TODO: Do this read in two reads rather than 8
    for stage in 0..8 {
        stages[stage] = read_freq_stage(&rosc, stage as u8)
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
        for mut stage in &mut stages[num_stages_at_drive_level..] {
            *stage = min;
        }
        write_freq_stages(&rosc, &stages);
        true
    } else {
        false
    }
    // NOTE: Medium should be 1.33 x low, high should be about 2x low.
}

fn write_freq_stages(rosc: &ROSC, stages: &[u8; 8]) {
    let passwd: u32 = 0x9696 << 16;
    let mut freq_a = passwd;
    let mut freq_b = passwd;
    for stage in 0..4 {
        freq_a |= ((stages[stage] & 0x07) as u32) << stage * 4;
    }
    for stage in 4..8 {
        freq_b |= ((stages[stage] & 0x07) as u32) << (stage - 4) * 4;
    }
    rosc.freqa.write(|w| unsafe { w.bits(freq_a) });
    rosc.freqb.write(|w| unsafe { w.bits(freq_b) });
}

///! Increase the rosc frequency range up to the next step.
fn increase_freq_range(rosc: &ROSC) -> bool {
    let did_increase_freq_range = match rosc.ctrl.read().freq_range().variant() {
        None => {
            // Initial unset frequency range, move to LOW frequency range
            rosc.ctrl.write(|w| w.freq_range().low());
            // Reset all the drive strength bits.
            write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::LOW) => {
            // Transition from LOW to MEDIUM frequency range
            rosc.ctrl.write(|w| w.freq_range().medium());
            // Reset all the drive strength bits.
            write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::MEDIUM) => {
            // Transition from MEDIUM to HIGH frequency range
            rosc.ctrl.write(|w| w.freq_range().high());
            // Reset all the drive strength bits.
            write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::HIGH) | Some(FREQ_RANGE_A::TOOHIGH) => {
            // Already in the HIGH frequency range, and can't increase
            false
        }
    };
    did_increase_freq_range
}

pub fn find_target_rosc_frequency(rosc: &ROSC, target_frequency: u32) -> u32 {
    reset_rosc_operating_frequency(rosc);
    let mut div = 1;
    let mut measured_rosc_frequency;
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz();
        // If it has overshot the target frequency, increase the divider and continue.
        if measured_rosc_frequency > target_frequency {
            div += 1;
            set_rosc_div(rosc, div);
        } else {
            break;
        }
    }
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz();
        if measured_rosc_frequency > target_frequency {
            // And probably want to step it down a notch?
            break;
        }
        let can_increase = increase_freq(rosc);
        if !can_increase {
            let can_increase_range = increase_freq_range(rosc);
            if !can_increase_range {
                break;
            }
        }
    }
    measured_rosc_frequency
}

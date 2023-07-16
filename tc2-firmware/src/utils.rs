use defmt::info;
use crate::XOSC_CRYSTAL_FREQ;
use crate::bsp::pac;
use crate::bsp::pac::ROSC;

pub fn rosc_frequency_count_hz() -> u32 {
    // Use the reference xosc while enabled to measure the speed of the rosc.
    // We need to steal the clocks block which is already "owned" by the ClocksManager
    // struct, which configured the xosc as the reference clock, which is needed
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
    let div = if div == 32 {
        0
    } else {
        div
    };
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
        _ => panic!("invalid frequency drive strength stage")
    }
}

fn increase_freq(rosc: &ROSC) -> bool {
    let range = rosc.ctrl.read().freq_range().bits();
    // Assume div is 1, and freq_range is high
    let mut stages: [u8;8] = [0;8];
    for stage in 0..8 {
        stages[stage] = read_freq_stage(&rosc, stage as u8)
    }
    let num_stages = match range {
        0xfa4 => 8, // LOW
        0xfa5 => 6, // MEDIUM
        0xfa7 => 4, // HIGH
        0xaa0 => {
            info!("INITIAL STAGE");
            return false;
        },
        _ => panic!("Invalid stage")
    };
    let mut next_i = 0;
    for (index, x) in stages[0..num_stages].windows(2).enumerate() {
        if x[1] < x[0] {
            next_i = index + 1;
            break;
        }
    }
    if stages[next_i] < 3 {
        stages[next_i] += 1;
        let mut min = u8::MAX;
        for stage in 0..num_stages {
            min = stages[stage].min(min);
        }
        for stage in num_stages..8 {
            stages[stage] = min;
        }
        write_freq_stages(&rosc, &stages);
        true
    } else {
        false
    }
    // NOTE: Medium should be 1.33 x low, high should be about 2x low.
}

fn write_freq_stages(rosc: &ROSC, stages: &[u8;8]) {
    let passwd: u32 = 0x9696 << 16;
    let mut freqa = passwd;
    let mut freqb = passwd;
    for stage in 0..4 {
        freqa |= ((stages[stage] & 0x07) as u32) << stage * 4;
    }
    for stage in 4..8 {
        freqb |= ((stages[stage] & 0x07) as u32) << (stage - 4) * 4;
    }
    rosc.freqa.write(|w| unsafe { w.bits(freqa) });
    rosc.freqb.write(|w| unsafe { w.bits(freqb) });
}

fn increase_freq_range(rosc: &ROSC) -> bool {
    let range = rosc.ctrl.read().freq_range().bits();
    if range == 0xaa0 {
        info!("Set freq range LOW");
        rosc.ctrl.write(|w| unsafe { w.freq_range().bits(0xfa4) });
        write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
        true
    } else if range == 0xfa4 {
        info!("Set freq range MEDIUM");
        rosc.ctrl.write(|w| unsafe { w.freq_range().bits(0xfa5) });
        write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
        true
    } else if range == 0xfa5 {
        info!("Set freq range HIGH");
        rosc.ctrl.write(|w| unsafe { w.freq_range().bits(0xfa7) });
        write_freq_stages(&rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
        true
    } else {
        false
    }
}

pub fn find_target_rosc_frequency(rosc: &ROSC, target_frequency: u32) -> u32 {
    // So, we want a certain target clock speed.
    // We want to slowly ramp up our rosc to get there.
    // There's also the option of setting the divider.
    reset_rosc_operating_frequency(rosc);
    let mut div = 1;
    let mut measured_rosc_frequency;
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz();
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

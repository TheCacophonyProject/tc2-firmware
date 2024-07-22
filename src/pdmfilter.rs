const PDM_DECIMATION: u8 = 64;
const PI: f32 = 3.14159;
const SINCN: u8 = 3;
const FILTER_GAIN: u8 = 64;
const MAX_VOLUME: u8 = 64;
use defmt::{error, info, warn};

//this is ported from
//https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico/blob/main/src/pdm_microphone.c

pub struct PDMFilter {
    lut: [u32; (SINCN * PDM_DECIMATION / 8) as usize * 256],
    fs: u32,
    coef: [u32; 2],
    sub_const: u32,
    old_out: i64,
    old_in: i64,
    oldz: i64,
    hp_alpha: u32,
    lp_alpha: u32,
    div_const: u32,
}
impl PDMFilter {
    pub fn new(sample_rate: u32) -> PDMFilter {
        PDMFilter {
            lut: [0u32; (SINCN * PDM_DECIMATION / 8) as usize * 256],
            fs: sample_rate as u32,
            coef: [0, 0],
            sub_const: 0,
            old_out: 0,
            old_in: 0,
            oldz: 0,
            lp_alpha: 0,
            hp_alpha: 0,
            div_const: 0,
        }
    }
    pub fn init(&mut self) {
        let lp_hz: f32 = self.fs as f32 / 2.0;
        let hp_hz: f32 = 40.0;
        if lp_hz != 0.0 {
            self.lp_alpha = (lp_hz * 256.0 / (lp_hz + self.fs as f32 / (2.0 * PI))) as u32;
        }
        if hp_hz != 0.0 {
            self.hp_alpha = (self.fs as f32 * 256.0 / (2.0 * PI * hp_hz + self.fs as f32)) as u32;
        }

        let sinc = [1u16; PDM_DECIMATION as usize];
        let mut sinc_out = [0u16; PDM_DECIMATION as usize * 2 - 1];

        let mut sinc2 = [0u16; PDM_DECIMATION as usize * 3];
        _ = convolve(
            &sinc,
            PDM_DECIMATION as usize,
            &sinc,
            PDM_DECIMATION as usize,
            sinc_out.as_mut(),
        );
        let sum = convolve(
            &sinc_out,
            PDM_DECIMATION as usize * 2 - 1,
            &sinc,
            PDM_DECIMATION as usize,
            sinc2[1..].as_mut(),
        );

        self.sub_const = sum >> 1;
        self.div_const = self.sub_const * MAX_VOLUME as u32 / 32768 / FILTER_GAIN as u32;
        if self.div_const == 0 {
            self.div_const = 1;
        }
        info!(
            "Set alphas lp {} hp {} syv const {} div const {} sum {}",
            self.lp_alpha, self.hp_alpha, self.sub_const, self.div_const, sum
        );
        for s in 0..SINCN {
            let offset: usize = (s * PDM_DECIMATION) as usize;

            for c in 0..256u32 {
                for d in 0..(PDM_DECIMATION / 8) as usize {
                    // (j * PDM_DECIMATION + i) as usize] as u32;

                    let coef_offset = offset + d * 8;
                    self.lut[((s as usize * 256 * 8) + c as usize * 8 + d) as usize] = (c >> 7)
                        * sinc2[coef_offset] as u32
                        + ((c >> 6) & 0x01) * sinc2[coef_offset + 1] as u32
                        + ((c >> 5) & 0x01) * sinc2[coef_offset + 2] as u32
                        + ((c >> 4) & 0x01) * sinc2[coef_offset + 3] as u32
                        + ((c >> 3) & 0x01) * sinc2[coef_offset + 4] as u32
                        + ((c >> 2) & 0x01) * sinc2[coef_offset + 5] as u32
                        + ((c >> 1) & 0x01) * sinc2[coef_offset + 6] as u32
                        + ((c) & 0x01) * sinc2[coef_offset + 7] as u32;
                }
            }
        }
    }

    pub fn filter(&mut self, data: &[u8], volume: u8, dataout: &mut [u16], saveout: bool) {
        let mut old_out: i64 = self.old_out;
        let mut old_in: i64 = self.old_in;
        let mut oldz: i64 = self.oldz;
        let mut out_index = 0;

        for i in (0..=data.len() - 8).step_by(PDM_DECIMATION as usize >> 3) {
            let index = i as usize;
            // 3 polyphase FIR?
            let z0 = filter_table_mono_64(&self.lut, &data[index..index + 8], 0);
            let z1 = filter_table_mono_64(&self.lut, &data[index..index + 8], 1);
            let z2 = filter_table_mono_64(&self.lut, &data[index..index + 8], 2);
            let mut z: i64 = self.coef[1] as i64 + z2 as i64 - self.sub_const as i64;

            self.coef[1] = self.coef[0] + z1 as u32;

            self.coef[0] = z0 as u32;

            old_out = (self.hp_alpha as i64 * (old_out + z - old_in)) >> 8;
            old_in = z;

            oldz = ((256 - self.lp_alpha as i64) * oldz + self.lp_alpha as i64 * old_out) >> 8;
            z = oldz * volume as i64;

            z = round_div(z, self.div_const as i64);
            z = satural_lh(z, -32700 as i64, 32700 as i64);

            if (saveout) {
                dataout[out_index] = (z as u16);
                out_index += 1;
            }
        }
        self.old_out = old_out;
        self.old_in = old_in;
        self.oldz = oldz;
    }
}

// round(a/b)
fn round_div(a: i64, b: i64) -> i64 {
    if a > 0 {
        return (a + b / 2) / b;
    }
    return (a - b / 2) / b;
}
// clip???
fn satural_lh(n: i64, l: i64, h: i64) -> i64 {
    if n < l {
        return l;
    } else if n > h {
        return h;
    }
    return n;
}

// apply weights on each bit of input data
fn filter_table_mono_64(lut: &[u32], data: &[u8], s: u8) -> u32 {
    let s_offset: usize = s as usize * 256 * 8;
    // because of endiness the first byte of a 32 bit is at index 3, 2, 1 .. 0
    return lut[s_offset + (data[3] as usize * PDM_DECIMATION as usize / 8) as usize]
        + lut[s_offset + (data[2] as usize * PDM_DECIMATION as usize / 8 + 1) as usize]
        + lut[s_offset + (data[1] as usize * PDM_DECIMATION as usize / 8 + 2) as usize]
        + lut[s_offset + (data[0] as usize * PDM_DECIMATION as usize / 8 + 3) as usize]
        + lut[s_offset + (data[7] as usize * PDM_DECIMATION as usize / 8 + 4) as usize]
        + lut[s_offset + (data[6] as usize * PDM_DECIMATION as usize / 8 + 5) as usize]
        + lut[s_offset + (data[5] as usize * PDM_DECIMATION as usize / 8 + 6) as usize]
        + lut[s_offset + (data[4] as usize * PDM_DECIMATION as usize / 8 + 7) as usize];
}
fn convolve(
    signal: &[u16],
    signal_len: usize,
    kernel: &[u16],
    kernel_len: usize,
    out: &mut [u16],
) -> u32 {
    let outlen = signal_len + kernel_len - 1;
    let mut sum = 0u32;
    for n in 0..outlen {
        let mut kmin: usize = 0;
        if n >= kernel_len - 1 {
            kmin = n - (kernel_len - 1)
        }

        let mut kmax: usize = signal_len - 1;
        if n < signal_len - 1 {
            kmax = n
        }
        let mut acc: u16 = 0;

        for k in kmin..=kmax {
            acc += signal[k] * kernel[n - k]
        }

        out[n] = acc;
        sum += acc as u32;
    }
    return sum;
}

use num_traits::FloatConst;

const PDM_DECIMATION: usize = 64;
const SINCN: usize = 3;
const FILTER_GAIN: u8 = 16;
const MAX_VOLUME: u8 = 64;

//this is ported from
//https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico/blob/main/src/pdm_microphone.c

pub struct PDMFilter {
    lut: [u32; (SINCN * PDM_DECIMATION / 8) * 256],
    fs: f32,
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
    #[allow(clippy::large_stack_arrays)]
    pub fn new(sample_rate: f32) -> PDMFilter {
        PDMFilter {
            lut: [0u32; (SINCN * PDM_DECIMATION / 8) * 256],
            fs: sample_rate,
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
    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::cast_sign_loss)]
    pub fn init(&mut self) {
        let lp_hz: f32 = self.fs / 2.0;
        let hp_hz: f32 = 40.0;

        if lp_hz != 0.0 {
            self.lp_alpha = (lp_hz * 256.0 / (lp_hz + self.fs / (2.0 * f32::PI()))) as u32;
        }

        // high pass filter does not seem to be doing anything
        if hp_hz != 0.0 {
            self.hp_alpha = (self.fs * 256.0 / (2.0 * f32::PI() * hp_hz + self.fs)) as u32;
        }

        let sinc = [1u16; PDM_DECIMATION];
        let mut sinc_out = [0u16; PDM_DECIMATION * 2 - 1];

        let mut sinc2 = [0u16; PDM_DECIMATION * 3];
        let _ = convolve(
            &sinc,
            PDM_DECIMATION,
            &sinc,
            PDM_DECIMATION,
            sinc_out.as_mut(),
        );

        // https://s3.amazonaws.com/embeddedrelated/user/114298/lti%20filters_3552.pdf
        // adding 0x00008000 to sum, to undo filter noise bias
        let sum = convolve(
            &sinc_out,
            PDM_DECIMATION * 2 - 1,
            &sinc,
            PDM_DECIMATION,
            sinc2[1..].as_mut(),
        ) + 0x0000_8000;

        self.sub_const = sum >> 1;
        self.div_const = self.sub_const * u32::from(MAX_VOLUME) / 32768 / u32::from(FILTER_GAIN);
        if self.div_const == 0 {
            self.div_const = 1;
        }
        // info!(
        //     "Set alphas lp {} hp {} sub const {} div const {} sum {}",
        //     self.lp_alpha, self.hp_alpha, self.sub_const, self.div_const, sum
        // );
        for s in 0..SINCN {
            let offset = s * PDM_DECIMATION;
            for c in 0..256u32 {
                for d in 0..8 {
                    let coef_offset = offset + d * 8;
                    self.lut[(s * 256 * 8) + c as usize * 8 + d] = (c >> 7)
                        * u32::from(sinc2[coef_offset])
                        + ((c >> 6) & 0x01) * u32::from(sinc2[coef_offset + 1])
                        + ((c >> 5) & 0x01) * u32::from(sinc2[coef_offset + 2])
                        + ((c >> 4) & 0x01) * u32::from(sinc2[coef_offset + 3])
                        + ((c >> 3) & 0x01) * u32::from(sinc2[coef_offset + 4])
                        + ((c >> 2) & 0x01) * u32::from(sinc2[coef_offset + 5])
                        + ((c >> 1) & 0x01) * u32::from(sinc2[coef_offset + 6])
                        + ((c) & 0x01) * u32::from(sinc2[coef_offset + 7]);
                }
            }
        }
    }

    pub fn filter(&mut self, data: &[u8], volume: u8, mut data_out: Option<&mut [u16]>) {
        let mut old_out = self.old_out;
        let mut old_in = self.old_in;
        let mut oldz = self.oldz;

        for (out_index, data_slice) in data.chunks_exact(PDM_DECIMATION >> 3).enumerate() {
            // 3 polyphase FIR?
            let z0 = filter_table_mono_64(&self.lut, data_slice, 0);
            let z1 = filter_table_mono_64(&self.lut, data_slice, 1);
            let z2 = filter_table_mono_64(&self.lut, data_slice, 2);
            let mut z = i64::from(self.coef[1]) + i64::from(z2) - i64::from(self.sub_const);

            self.coef[1] = self.coef[0] + z1;
            self.coef[0] = z0;

            old_out = (i64::from(self.hp_alpha) * (old_out + z - old_in)) >> 8;
            old_in = z;

            oldz =
                ((256 - i64::from(self.lp_alpha)) * oldz + i64::from(self.lp_alpha) * old_out) >> 8;
            z = oldz * i64::from(volume);
            z = round_div(z, i64::from(self.div_const));
            // NOTE: Any reason why this isn't i16::MIN/i16::MAX?  We don't really know, it's like
            //  that in the code this was ported from.
            z = z.clamp(-32700, 32700);

            #[allow(clippy::cast_possible_truncation)]
            #[allow(clippy::cast_sign_loss)]
            if let Some(ref mut data_out) = data_out {
                *unsafe { data_out.get_unchecked_mut(out_index) } = z as u16;
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
    (a - b / 2) / b
}

// apply weights on each bit of input data
fn filter_table_mono_64(lut: &[u32], data: &[u8], s: usize) -> u32 {
    let s_offset = s * 256 * 8;
    let lut = &lut[s_offset..];
    let dec = PDM_DECIMATION / 8;
    lut[data[3] as usize * dec]
        + lut[data[2] as usize * dec + 1]
        + lut[data[1] as usize * dec + 2]
        + lut[data[0] as usize * dec + 3]
        + lut[data[7] as usize * dec + 4]
        + lut[data[6] as usize * dec + 5]
        + lut[data[5] as usize * dec + 6]
        + lut[data[4] as usize * dec + 7]
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
            kmin = n - (kernel_len - 1);
        }

        let mut kmax: usize = signal_len - 1;
        if n < signal_len - 1 {
            kmax = n;
        }
        let mut acc: u16 = 0;

        for k in kmin..=kmax {
            acc += signal[k] * kernel[n - k];
        }

        out[n] = acc;
        sum += u32::from(acc);
    }
    sum
}

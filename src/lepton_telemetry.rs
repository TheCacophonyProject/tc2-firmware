use crate::lepton::FFCStatus;
use byteorder::{ByteOrder, LittleEndian};
use defmt::{Format, info, warn};

#[derive(Debug, Format)]
pub struct Telemetry {
    pub revision: [u8; 2],
    pub frame_num: u32,
    pub msec_on: u32,
    pub msec_since_last_ffc: u32,
    pub time_at_last_ffc: u32,
    pub fpa_temp_c: f32,
    pub fpa_temp_c_at_last_ffc: f32,
    pub ffc_status: FFCStatus,
}

impl Telemetry {
    pub fn from_bytes(buf: &[u8]) -> Self {
        let telemetry_revision = LittleEndian::read_u16(&buf[0..2]);
        let frame_num = LittleEndian::read_u32(&buf[40..44]);
        let msec_on = LittleEndian::read_u32(&buf[2..6]);
        let time_at_last_ffc = LittleEndian::read_u32(&buf[60..64]);
        let msec_since_last_ffc = msec_on.saturating_sub(time_at_last_ffc);
        let status_bits = LittleEndian::read_u32(&buf[6..10]);
        let ffc_state = (((status_bits >> 4) & 0b11) as u8).into();
        let fpa_temp_kelvin_x_100 = LittleEndian::read_u16(&buf[48..=49]);
        let fpa_temp_kelvin_x_100_at_last_ffc = LittleEndian::read_u16(&buf[58..=59]);

        let fpa_temp_c = (f32::from(fpa_temp_kelvin_x_100) / 100.0) - 273.15;
        let fpa_temp_c_at_last_ffc =
            (f32::from(fpa_temp_kelvin_x_100_at_last_ffc) / 100.0) - 273.15;
        #[allow(clippy::cast_possible_truncation)]
        Telemetry {
            revision: [
                (telemetry_revision << 8) as u8,
                (telemetry_revision & 0x0f) as u8,
            ],
            frame_num,
            msec_on,
            time_at_last_ffc,
            msec_since_last_ffc,
            fpa_temp_c,
            fpa_temp_c_at_last_ffc,
            ffc_status: ffc_state,
        }
    }

    pub fn is_valid(&self, telemetry_revision_stable: &mut ([u8; 2], i8)) -> bool {
        // Sometimes the header is invalid, but the frame becomes valid and gets sync.
        // Because the telemetry revision is static across frame headers we can detect this
        // case and not send the frame, as it may cause false triggers.
        if telemetry_revision_stable.1 > -1 && telemetry_revision_stable.1 <= 2 {
            if telemetry_revision_stable.0[0] == self.revision[0]
                && telemetry_revision_stable.0[1] == self.revision[1]
            {
                telemetry_revision_stable.1 += 1;
            } else {
                telemetry_revision_stable.1 = -1;
            }
            if telemetry_revision_stable.1 > 2 {
                info!("Got stable telemetry revision (core 0) {:?}", self.revision);
            }
        }
        if telemetry_revision_stable.1 == -1 {
            // Initialise seen telemetry revision.
            telemetry_revision_stable.0 = [self.revision[0], self.revision[1]];
            telemetry_revision_stable.1 += 1;
        }
        if telemetry_revision_stable.1 < 2 {
            false
        } else if telemetry_revision_stable.1 > 2
            && (telemetry_revision_stable.0[0] != self.revision[0]
                || telemetry_revision_stable.0[1] != self.revision[1])
        {
            // We have a misaligned/invalid frame.
            warn!("Misaligned header (core 0)");
            false
        } else {
            true
        }
    }
}

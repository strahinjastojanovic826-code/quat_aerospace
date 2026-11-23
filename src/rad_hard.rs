use crate::logic::{Quat, QuatError};

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub struct ScrubReport {
    pub bytes_checked: usize,
    pub corrected_bytes: usize,
    pub uncorrectable_mismatches: usize,
}

pub fn crc8_atm(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) for telemetry frames & NVM blocks.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Performs per-quat (2-bit lane) TMR majority voting across 3 packed bytes (`a`, `b`, `c`).
#[inline]
pub const fn tmr_vote_quats_in_byte(a: u8, b: u8, c: u8) -> u8 {
    let mut out: u8 = 0;
    let mut shift: u8 = 0;
    while shift < 8 {
        let qa = (a >> shift) & 0b11;
        let qb = (b >> shift) & 0b11;
        let qc = (c >> shift) & 0b11;

        let voted = if qa == qb || qa == qc {
            qa
        } else if qb == qc {
            qb
        } else {
            // Unanimous disagreement in this 2-bit lane: default to lowest/safest state `0b00`.
            0b00
        };
        out |= voted << shift;
        shift += 2;
    }
    out
}

#[inline]
const fn majority_byte(a: u8, b: u8, c: u8) -> (u8, bool, bool) {
    let voted = tmr_vote_quats_in_byte(a, b, c);
    let any_diff = (a != b) || (b != c);
    let all_diff = (a != b) && (b != c) && (a != c);
    (voted, any_diff, all_diff)
}

pub fn scrub_tmr_bufs(
    replica_a: &mut [u8],
    replica_b: &mut [u8],
    replica_c: &mut [u8],
) -> Result<ScrubReport, QuatError> {
    if replica_a.len() != replica_b.len() || replica_b.len() != replica_c.len() {
        return Err(QuatError::LengthMismatch);
    }

    let mut report = ScrubReport {
        bytes_checked: replica_a.len(),
        corrected_bytes: 0,
        uncorrectable_mismatches: 0,
    };

    for i in 0..replica_a.len() {
        let (voted, any_diff, all_diff) = majority_byte(replica_a[i], replica_b[i], replica_c[i]);
        if any_diff {
            if all_diff {
                report.uncorrectable_mismatches += 1;
            } else {
                report.corrected_bytes += 1;
            }
            replica_a[i] = voted;
            replica_b[i] = voted;
            replica_c[i] = voted;
        }
    }

    Ok(report)
}

#[inline]
pub const fn tmr_vote_quat(a: Quat, b: Quat, c: Quat, fallback: Quat) -> Quat {
    let ab = a.bits() == b.bits();
    let ac = a.bits() == c.bits();
    let bc = b.bits() == c.bits();

    if ab || ac {
        a
    } else if bc {
        b
    } else {
        fallback
    }
}
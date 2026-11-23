#![no_std]

pub mod fsm;
pub mod logic;
pub mod packed;
pub mod precision;
pub mod rad_hard;
pub mod ring;

pub use fsm::{FaultReason, FlightMode, FlightModeController, TransitionError};
pub use logic::{Quat, QuatError};
pub use packed::{PackedQuatBuf, PackedQuatIter};
pub use precision::{
    FixedPoint16, FixedPoint32, PrecisionScale, QuantizedSignal, QuatWord16, QuatWord4,
};
pub use rad_hard::{
    crc16_ccitt, crc8_atm, scrub_tmr_bufs, tmr_vote_quat, tmr_vote_quats_in_byte, ScrubReport,
};
pub use ring::PackedQuatRing;

#[cfg(test)]
mod tests {
    extern crate std;

    use core::hint::black_box;
    use std::println;
    use std::time::Instant;

    use super::*;

    fn bench_report(name: &str, ops: u64, start: Instant) {
        let elapsed = start.elapsed();
        let total_ns = elapsed.as_nanos();
        let ns_per_op = if ops > 0 {
            total_ns as f64 / ops as f64
        } else {
            0.0
        };
        println!(
            "[BENCH] {name}: ops={ops}, elapsed={elapsed:?}, ns_per_op={ns_per_op:.2}"
        );
    }

    // 1. Easiest: basic 2-bit bitmasking and construction
    #[test]
    fn test_01_quat_bit_construction_and_masking() {
        let start = Instant::now();
        let iterations: u64 = 100_000;
        let mut acc: u8 = 0;

        for i in 0..iterations {
            let raw = black_box((i & 0xFF) as u8);
            let q = Quat::from_bits_masked(raw);
            let strict = Quat::try_from_bits(raw & 0b11).unwrap();
            assert_eq!(q, strict);
            acc = acc.wrapping_add(q.bits());
        }

        black_box(acc);
        bench_report("test_01_quat_bit_construction_and_masking", iterations, start);
    }

    // 2. Easy: core quaternary logic gates & saturating ops
    #[test]
    fn test_02_quat_logic_and_saturating_ops() {
        let start = Instant::now();
        let iterations: u64 = 100_000;
        let mut checksum: u8 = 0;

        for i in 0..iterations {
            let a = Quat::from_bits_masked(black_box((i & 0b11) as u8));
            let b = Quat::from_bits_masked(black_box(((i >> 2) & 0b11) as u8));

            assert_eq!(a.invert().invert(), a);
            assert!(a.q_max(b).bits() >= a.q_min(b).bits());

            let muxed = Quat::mux(a, Quat::Q01, Quat::Q11);
            let combined = a.q_xor(b).sat_add(muxed).sat_sub(Quat::Q00).rot_inc();
            checksum = checksum.wrapping_add(combined.bits());
        }

        black_box(checksum);
        bench_report("test_02_quat_logic_and_saturating_ops", iterations, start);
    }

    // 3. Easy-Medium: 75% memory savings check (4 quats per byte)
    #[test]
    fn test_03_packed_buf_75_percent_memory_savings() {
        let start = Instant::now();
        let quat_count = 64usize;
        let mut raw_mem = [0u8; 16]; // 64 quats / 4 = 16 bytes instead of 64 bytes (75% saved)
        let iterations: u64 = 25_000;

        for iter in 0..iterations {
            let mut buf = PackedQuatBuf::new_zeroed(&mut raw_mem, quat_count).unwrap();
            assert_eq!(PackedQuatBuf::required_bytes(quat_count), 16);
            assert_eq!(buf.as_bytes().len(), 16);

            let idx = (iter as usize) % quat_count;
            let val = Quat::from_bits_masked((iter & 0b11) as u8);
            buf.set(idx, val).unwrap();
            assert_eq!(buf.get(idx).unwrap(), val);
            black_box(buf.as_bytes());
        }

        bench_report("test_03_packed_buf_75_percent_memory_savings", iterations, start);
    }

    // 4. Medium: decoupled PrecisionScale and QuantizedSignal
    #[test]
    fn test_04_decoupled_precision_scale_and_signal() {
        let start = Instant::now();
        let scale = PrecisionScale::new(-1000, 500, 1000).unwrap();
        let iterations: u64 = 100_000;
        let mut sum: i64 = 0;

        for i in 0..iterations {
            let q = Quat::from_bits_masked(black_box((i & 0b11) as u8));
            let sig = QuantizedSignal::new(q, scale);
            let scaled = sig.precision.eval_scaled(sig.state);
            let whole = sig.value_int();
            let rem = sig.value_rem();
            assert_eq!(scaled, whole * 1000 + rem);
            sum = sum.wrapping_add(scaled);
        }

        black_box(sum);
        bench_report("test_04_decoupled_precision_scale_and_signal", iterations, start);
    }

    // 5. Medium: QuatWord4 packing and lane access
    #[test]
    fn test_05_quat_word4_lanes() {
        let start = Instant::now();
        let iterations: u64 = 100_000;
        let mut acc: u8 = 0;

        for i in 0..iterations {
            let q0 = Quat::from_bits_masked((i & 0b11) as u8);
            let q1 = Quat::from_bits_masked(((i >> 2) & 0b11) as u8);
            let q2 = Quat::from_bits_masked(((i >> 4) & 0b11) as u8);
            let q3 = Quat::from_bits_masked(((i >> 6) & 0b11) as u8);

            let word = QuatWord4::from_quats(q0, q1, q2, q3);
            assert_eq!(word.get(0).unwrap(), q0);
            assert_eq!(word.get(1).unwrap(), q1);
            assert_eq!(word.get(2).unwrap(), q2);
            assert_eq!(word.get(3).unwrap(), q3);
            acc = acc.wrapping_add(word.to_u8());
        }

        black_box(acc);
        bench_report("test_05_quat_word4_lanes", iterations, start);
    }

    // 6. Medium: FixedPoint16 Q8.8 arithmetic decoupled from Quat
    #[test]
    fn test_06_fixed_point_16_q8_8_math() {
        let start = Instant::now();
        let iterations: u64 = 100_000;
        let mut acc: i16 = 0;

        for i in 0..iterations {
            let q = Quat::from_bits_masked(black_box((i & 0b11) as u8));
            let term = FixedPoint16::from_quat_scaled(q, 128, 64); // step=0.5, offset=0.25
            let gain = FixedPoint16::from_int(2);
            let prod = term.mul_q8_8(gain).sat_add(FixedPoint16::from_int(1));
            acc = acc.wrapping_add(prod.raw());
        }

        black_box(acc);
        bench_report("test_06_fixed_point_16_q8_8_math", iterations, start);
    }

    // 7. Medium-High: QuatWord16 (32-bit word of 16 quats) and FixedPoint32 (Q16.16)
    #[test]
    fn test_07_quat_word16_and_fixed_point_32() {
        let start = Instant::now();
        let iterations: u64 = 50_000;
        let mut acc: i32 = 0;

        for i in 0..iterations {
            let lane = (i & 0x0F) as u8;
            let q = Quat::from_bits_masked(((i >> 4) & 0b11) as u8);
            let word = QuatWord16(0x5555_5555).with_set(lane, q).unwrap();
            assert_eq!(word.get(lane).unwrap(), q);

            let fp = FixedPoint32::from_quat_scaled(q, 1 << 15, 1 << 14);
            let scaled = fp.mul_q16_16(FixedPoint32::from_int(3));
            acc = acc.wrapping_add(scaled.raw() ^ (word.to_u32() as i32));
        }

        black_box(acc);
        bench_report("test_07_quat_word16_and_fixed_point_32", iterations, start);
    }

    // 8. Medium-High: bulk pack/unpack and iterator over PackedQuatBuf
    #[test]
    fn test_08_packed_buf_bulk_and_iterator() {
        let start = Instant::now();
        let mut src = [Quat::Q00; 32];
        for (idx, slot) in src.iter_mut().enumerate() {
            *slot = Quat::from_bits_masked((idx & 0b11) as u8);
        }

        let mut storage = [0u8; 8];
        let mut dst = [Quat::Q00; 32];
        let iterations: u64 = 25_000;

        for _ in 0..iterations {
            let mut packed = PackedQuatBuf::new_zeroed(&mut storage, 32).unwrap();
            packed.pack_from_slice(&src).unwrap();
            packed.unpack_into_slice(&mut dst).unwrap();
            assert_eq!(src, dst);

            let iter_sum: u8 = packed.iter().map(|q| q.bits()).sum();
            black_box(iter_sum);
        }

        bench_report("test_08_packed_buf_bulk_and_iterator", iterations, start);
    }

    // 9. High: FlightModeController transitions and fault escalation
    #[test]
    fn test_09_flight_mode_fsm_and_fault_escalation() {
        let start = Instant::now();
        let iterations: u64 = 50_000;
        let mut modes_seen: u8 = 0;

        for _ in 0..iterations {
            let mut fsm = FlightModeController::new(FlightMode::Safe, 2);
            assert!(fsm.step_to(FlightMode::Critical).is_err());

            assert_eq!(fsm.step_to(FlightMode::Standby).unwrap(), FlightMode::Standby);
            assert_eq!(fsm.step_to(FlightMode::Nominal).unwrap(), FlightMode::Nominal);
            assert_eq!(fsm.step_to(FlightMode::Critical).unwrap(), FlightMode::Critical);

            assert_eq!(fsm.report_fault(FaultReason::SensorDisagreement), FlightMode::Nominal);
            assert_eq!(fsm.report_fault(FaultReason::RadiationUpset), FlightMode::Safe);

            fsm.clear_faults();
            assert_eq!(fsm.fault_streak(), 0);
            modes_seen = modes_seen.wrapping_add(fsm.mode().to_quat().bits());
        }

        black_box(modes_seen);
        bench_report("test_09_flight_mode_fsm_and_fault_escalation", iterations, start);
    }

    // 10. High: CRC-8, CRC-16, and single-quat TMR voting
    #[test]
    fn test_10_crc_and_single_quat_tmr_voting() {
        let start = Instant::now();
        let payload = [0xA4u8, 0x19, 0xFF, 0x03, 0x5C, 0x88, 0x11, 0x2A];
        let expected_c8 = crc8_atm(&payload);
        let expected_c16 = crc16_ccitt(&payload);
        let iterations: u64 = 50_000;

        for i in 0..iterations {
            let c8 = crc8_atm(black_box(&payload));
            let c16 = crc16_ccitt(black_box(&payload));
            assert_eq!(c8, expected_c8);
            assert_eq!(c16, expected_c16);

            let consensus = Quat::from_bits_masked((i & 0b11) as u8);
            let upset = consensus.invert();
            let voted = tmr_vote_quat(consensus, upset, consensus, Quat::Q00);
            assert_eq!(voted, consensus);
        }

        bench_report("test_10_crc_and_single_quat_tmr_voting", iterations, start);
    }

    // 11. Stress: lock-free PackedQuatRing wrap-around and full/empty cycles
    #[test]
    fn test_11_stress_lockfree_packed_quat_ring() {
        let start = Instant::now();
        let mut backing = [0u8; 64]; // 256 quaternary slots in 64 bytes
        let ring = PackedQuatRing::new(&mut backing).unwrap();
        assert_eq!(ring.capacity(), 256);

        let total_pushes_and_pops: u64 = 200_000;
        let mut expected_val: u8 = 0;

        for i in 0..total_pushes_and_pops {
            // Burst push up to 192 items, then drain
            if (i % 256) < 192 {
                let q = Quat::from_bits_masked((i & 0b11) as u8);
                if ring.push(q).is_ok() {
                    black_box(q);
                }
            }
            if let Ok(popped) = ring.pop() {
                expected_val = expected_val.wrapping_add(popped.bits());
            }
        }

        while ring.pop().is_ok() {}
        assert!(ring.is_empty());
        black_box(expected_val);

        bench_report(
            "test_11_stress_lockfree_packed_quat_ring",
            total_pushes_and_pops,
            start,
        );
    }

    // 12. Most stressful: heavy radiation SEU storm + per-quat TMR scrubbing + CRC verification
    #[test]
    fn test_12_stress_radiation_seu_storm_and_tmr_scrubbing() {
        let start = Instant::now();
        const BUF_BYTES: usize = 256; // 1,024 packed quaternaries per replica (3,072 quats across TMR)
        let mut rep_a = [0u8; BUF_BYTES];
        let mut rep_b = [0u8; BUF_BYTES];
        let mut rep_c = [0u8; BUF_BYTES];

        for i in 0..BUF_BYTES {
            let pattern = QuatWord4::from_quats(
                Quat::Q00,
                Quat::Q01,
                Quat::Q10,
                Quat::Q11,
            )
            .to_u8()
            .wrapping_add(i as u8);
            rep_a[i] = pattern;
            rep_b[i] = pattern;
            rep_c[i] = pattern;
        }

        let golden_crc16 = crc16_ccitt(&rep_a);
        let storm_cycles: u64 = 10_000;
        let mut total_corrected: usize = 0;

        for cycle in 0..storm_cycles {
            // Inject single-bit / single-quat SEU upsets into one replica per byte
            for i in 0..BUF_BYTES {
                let lane_shift = (((cycle as usize) + i) & 0b11) * 2;
                let flip_mask = 0b01u8 << lane_shift;
                match (cycle as usize + i) % 3 {
                    0 => rep_a[i] ^= flip_mask,
                    1 => rep_b[i] ^= flip_mask,
                    _ => rep_c[i] ^= flip_mask,
                }
            }

            let lane_voted = tmr_vote_quats_in_byte(rep_a[0], rep_b[0], rep_c[0]);
            black_box(lane_voted);

            let report = scrub_tmr_bufs(&mut rep_a, &mut rep_b, &mut rep_c).unwrap();
            assert_eq!(report.bytes_checked, BUF_BYTES);
            assert_eq!(report.corrected_bytes, BUF_BYTES);
            assert_eq!(report.uncorrectable_mismatches, 0);
            assert_eq!(rep_a, rep_b);
            assert_eq!(rep_b, rep_c);
            assert_eq!(crc16_ccitt(&rep_a), golden_crc16);

            total_corrected += report.corrected_bytes;
        }

        black_box(total_corrected);
        bench_report(
            "test_12_stress_radiation_seu_storm_and_tmr_scrubbing",
            storm_cycles,
            start,
        );
    }
}
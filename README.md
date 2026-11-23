# `quat_aerospace`

A zero-allocation, `#![no_std]` Rust library providing **2-bit quaternary logic** (`00`, `01`, `10`, `11`) with **75% memory savings** (`4` quaternary states packed per `u8`)

Precision and physical scaling are kept **strictly decoupled** from `Quat` logic:
- **`Quat`** stays a pure 2-bit value (`Q00`, `Q01`, `Q10`, `Q11`) with logic, masking, saturation, and rotation operations.
- **`precision`** (`PrecisionScale`, `QuantizedSignal`, `QuatWord4`, `QuatWord16`, `FixedPoint16`, `FixedPoint32`) is applied only where numeric scaling, fixed-point math, or multi-quat base-4 words are needed.

## Key Features

- **100% `#![no_std]` and Zero-Heap (`no alloc`)**: Deterministic stack and static buffer usage suitable for flight loops and interrupt contexts.
- **75% Memory Reduction**: `PackedQuatBuf` stores `4` quaternaries per byte (`ceil(len / 4)` bytes).
- **Decoupled Precision**: Separate `Quat` logic from `PrecisionScale`, `FixedPoint16` (Q8.8), and `FixedPoint32` (Q16.16).
- **Multi-Quat Words**: `QuatWord4` (`u8`, 4 quats) and `QuatWord16` (`u32`, 16 quats) for compact bit-lane registers.
- **Radiation-Hardened Primitives**: Single-quat and per-lane byte TMR voting (`tmr_vote_quat`, `tmr_vote_quats_in_byte`), in-place TMR buffer scrubbing (`scrub_tmr_bufs`), `crc8_atm`, and `crc16_ccitt`.
- **Deterministic Flight Mode FSM**: `FlightModeController` with step-size transition guards and fault-streak escalation (`FaultReason`).
- **Lock-Free Packed Ring Buffer**: `PackedQuatRing` using `AtomicUsize` indices for producer/consumer decoupling without heap allocation.

## Licensing

This project is dual-licensed under:

1. **GNU Affero General Public License v3.0 (AGPL-3.0)** for open-source use.
2. **Commercial License** for proprietary/commercial applications.

### Commercial License
If you wish to use `quat_aerospace` in closed-source software, proprietary products, or environments where AGPL-3.0 compliance is not feasible (e.g., embedding DLLs/shared libraries without disclosing host source code), a commercial license is required.

For commercial licensing inquiries, please contact: `strahinjastojanovic826@gmail.com`

## 📊 Benchmark

| Test Name | Operations (`ops`) | Elapsed Time | `ns_per_op` | Status |
| :--- | ---: | ---: | ---: | :---: |
| `test_03_packed_buf_75_percent_memory_savings` | 25,000 | 3.1098ms | 124.39 | `ok` |
| `test_06_fixed_point_16_q8_8_math` | 100,000 | 4.4592ms | 44.59 | `ok` |
| `test_04_decoupled_precision_scale_and_signal` | 100,000 | 5.6752ms | 56.75 | `ok` |
| `test_01_quat_bit_construction_and_masking` | 100,000 | 6.3912ms | 63.91 | `ok` |
| `test_02_quat_logic_and_saturating_ops` | 100,000 | 6.4222ms | 64.22 | `ok` |
| `test_07_quat_word16_and_fixed_point_32` | 50,000 | 6.5701ms | 131.40 | `ok` |
| `test_09_flight_mode_fsm_and_fault_escalation` | 50,000 | 9.2795ms | 185.59 | `ok` |
| `test_11_stress_lockfree_packed_quat_ring` | 200,000 | 13.5367ms | 67.68 | `ok` |
| `test_05_quat_word4_lanes` | 100,000 | 14.9742ms | 149.74 | `ok` |
| `test_10_crc_and_single_quat_tmr_voting` | 50,000 | 40.8254ms | 816.51 | `ok` |
| `test_08_packed_buf_bulk_and_iterator` | 25,000 | 91.5413ms | 3661.65 | `ok` |
| `test_12_stress_radiation_seu_storm_and_tmr_scrubbing` | 10,000 | 351.3737ms | 35137.37 | `ok` |

## Project Structure

```text
quat_aerospace/
├── Cargo.toml
├── README.md
└── src/
    ├── lib.rs        # Crate root (#![no_std]), public re-exports, and benchmarked test suite
    ├── logic.rs      # Pure 2-bit quaternary type (Quat), logic gates, saturation, errors
    ├── packed.rs     # 2-bit packed buffer (PackedQuatBuf) and iterator (PackedQuatIter)
    ├── precision.rs  # Decoupled scaling, QuatWord4/QuatWord16, FixedPoint16/FixedPoint32
    ├── rad_hard.rs   # TMR voting, buffer scrubbing, CRC-8, and CRC-16/CCITT
    ├── fsm.rs        # FlightMode, FlightModeController, transition & fault escalation logic
    └── ring.rs       # Lock-free atomic SPSC packed quaternary ring buffer
```

## Importing and Usage

### Add to `Cargo.toml`

Depending on how you organize your workspace or local crate:

```toml
[dependencies]
quat_aerospace = "0.1.0"
```

### Basic Usage: Packed Storage, TMR Voting, and Decoupled Precision

```rust
use quat_aerospace::{
    crc16_ccitt,
    scrub_tmr_bufs,
    tmr_vote_quat,
    FixedPoint16,
    FlightMode,
    FlightModeController,
    PackedQuatBuf,
    Quat,
};

pub fn flight_control_tick() {
    // 16 quaternary states fit in 4 bytes (75% less than [u8; 16])
    let mut backing = [0u8; 4];
    let mut states = PackedQuatBuf::new_zeroed(&mut backing, 16).unwrap();
    states.set(0, Quat::Q01).unwrap();
    states.set(1, Quat::Q10).unwrap();
    states.set(2, Quat::Q11).unwrap();

    // Triple Modular Redundancy (TMR) vote on a single quaternary state
    let voted = tmr_vote_quat(Quat::Q10, Quat::Q10, Quat::Q00, Quat::Q00);

    // Apply precision only where numeric math is needed (Q8.8 fixed-point)
    let scale_q8_8 = FixedPoint16::from_quat_scaled(voted, 128, 0); // 0.5 per step
    let gain_q8_8 = FixedPoint16::from_int(2);
    let _command = scale_q8_8.mul_q8_8(gain_q8_8);

    // Scrub three redundant packed buffers and compute frame CRC-16
    let mut a = *states.as_bytes();
    let mut b = a;
    let mut c = a;
    let _scrub = scrub_tmr_bufs(&mut a, &mut b, &mut c).unwrap();
    let _crc = crc16_ccitt(&a);

    // Deterministic flight mode management
    let mut fsm = FlightModeController::new(FlightMode::Standby, 3);
    let _ = fsm.step_to(FlightMode::Nominal);
}
```

### Lock-Free Packed Ring Buffer (`PackedQuatRing`)

```rust
use quat_aerospace::{PackedQuatRing, Quat};

pub fn telemetry_ring_example() {
    // 8 bytes backing => 32 packed quaternary slots (power-of-two byte length)
    let mut mem = [0u8; 8];
    let ring = PackedQuatRing::new(&mut mem).unwrap();

    ring.push(Quat::Q01).unwrap();
    ring.push(Quat::Q11).unwrap();

    let first = ring.pop().unwrap();
    let second = ring.pop().unwrap();
    assert_eq!(first, Quat::Q01);
    assert_eq!(second, Quat::Q11);
}
```

## Running Tests and Benchmarks

The test suite in `src/lib.rs` includes 12 tests ordered from basic bit operations (`test_01`) to heavy stress tests (`test_11` ring buffer stress and `test_12` radiation SEU storm + TMR scrubbing), each reporting operation count, elapsed time, and `ns_per_op` using `std::time::Instant` and `core::hint::black_box` (no `criterion` dependency required).

Run all tests with benchmark output printed to stdout:

```bash
cargo test -- --nocapture
```

```bash
cargo test 
```
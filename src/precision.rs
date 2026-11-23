use crate::logic::{Quat, QuatError};

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub struct QuatWord4(pub u8);

impl QuatWord4 {
    #[inline]
    pub const fn from_quats(q0: Quat, q1: Quat, q2: Quat, q3: Quat) -> Self {
        let byte = q0.bits()
            | (q1.bits() << 2)
            | (q2.bits() << 4)
            | (q3.bits() << 6);
        Self(byte)
    }

    #[inline]
    pub const fn get(self, lane: u8) -> Result<Quat, QuatError> {
        if lane > 3 {
            return Err(QuatError::IndexOutOfBounds);
        }
        Ok(Quat::from_bits_masked((self.0 >> (lane * 2)) & 0b11))
    }

    #[inline]
    pub const fn to_u8(self) -> u8 {
        self.0
    }
}

/// Sixteen quaternary digits packed in a single `u32` (little-endian lane `0..=15`).
#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub struct QuatWord16(pub u32);

impl QuatWord16 {
    #[inline]
    pub const fn from_word4s(w0: QuatWord4, w1: QuatWord4, w2: QuatWord4, w3: QuatWord4) -> Self {
        let word = (w0.0 as u32)
            | ((w1.0 as u32) << 8)
            | ((w2.0 as u32) << 16)
            | ((w3.0 as u32) << 24);
        Self(word)
    }

    #[inline]
    pub const fn get(self, lane: u8) -> Result<Quat, QuatError> {
        if lane > 15 {
            return Err(QuatError::IndexOutOfBounds);
        }
        Ok(Quat::from_bits_masked(((self.0 >> (lane * 2)) & 0b11) as u8))
    }

    #[inline]
    pub const fn with_set(self, lane: u8, q: Quat) -> Result<Self, QuatError> {
        if lane > 15 {
            return Err(QuatError::IndexOutOfBounds);
        }
        let shift = lane * 2;
        let mask = !(0b11u32 << shift);
        Ok(Self((self.0 & mask) | ((q.bits() as u32) << shift)))
    }

    #[inline]
    pub const fn to_u32(self) -> u32 {
        self.0
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub struct FixedPoint16(pub i16);

impl FixedPoint16 {
    pub const FRAC_BITS: u8 = 8;
    pub const SCALE: i32 = 1 << Self::FRAC_BITS;

    #[inline]
    pub const fn from_int(v: i8) -> Self {
        Self((v as i16) << Self::FRAC_BITS)
    }

    #[inline]
    pub const fn from_quat_scaled(q: Quat, step_q8_8: i16, offset_q8_8: i16) -> Self {
        let prod = (q.bits() as i32) * (step_q8_8 as i32) + (offset_q8_8 as i32);
        Self(prod as i16)
    }

    #[inline]
    pub const fn sat_add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }

    #[inline]
    pub const fn mul_q8_8(self, rhs: Self) -> Self {
        let prod = ((self.0 as i32) * (rhs.0 as i32)) >> Self::FRAC_BITS;
        Self(prod as i16)
    }

    #[inline]
    pub const fn raw(self) -> i16 {
        self.0
    }
}

/// Q16.16 fixed-point number (`raw / 65536`) stored in `i32` for attitude / sensor pipelines.
#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub struct FixedPoint32(pub i32);

impl FixedPoint32 {
    pub const FRAC_BITS: u8 = 16;

    #[inline]
    pub const fn from_int(v: i16) -> Self {
        Self((v as i32) << Self::FRAC_BITS)
    }

    #[inline]
    pub const fn from_quat_scaled(q: Quat, step_q16_16: i32, offset_q16_16: i32) -> Self {
        let prod = (q.bits() as i64) * (step_q16_16 as i64) + (offset_q16_16 as i64);
        Self(prod as i32)
    }

    #[inline]
    pub const fn mul_q16_16(self, rhs: Self) -> Self {
        let prod = ((self.0 as i64) * (rhs.0 as i64)) >> Self::FRAC_BITS;
        Self(prod as i32)
    }

    #[inline]
    pub const fn raw(self) -> i32 {
        self.0
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct PrecisionScale {
    pub offset_num: i32,
    pub step_num: i32,
    pub denom: i32,
}

impl PrecisionScale {
    pub const fn new(offset_num: i32, step_num: i32, denom: i32) -> Option<Self> {
        if denom == 0 {
            return None;
        }
        Some(Self {
            offset_num,
            step_num,
            denom,
        })
    }

    #[inline]
    pub const fn eval_scaled(self, q: Quat) -> i64 {
        let level = q.bits() as i64;
        (self.offset_num as i64) + level * (self.step_num as i64)
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct QuantizedSignal {
    pub state: Quat,
    pub precision: PrecisionScale,
}

impl QuantizedSignal {
    #[inline]
    pub const fn new(state: Quat, precision: PrecisionScale) -> Self {
        Self { state, precision }
    }

    #[inline]
    pub const fn value_int(self) -> i64 {
        self.precision.eval_scaled(self.state) / (self.precision.denom as i64)
    }

    #[inline]
    pub const fn value_rem(self) -> i64 {
        self.precision.eval_scaled(self.state) % (self.precision.denom as i64)
    }
}
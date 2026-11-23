use core::fmt;

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum Quat {
    Q00 = 0b00,
    Q01 = 0b01,
    Q10 = 0b10,
    Q11 = 0b11,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum QuatError {
    InvalidBits(u8),
    BufferTooSmall,
    IndexOutOfBounds,
    LengthMismatch,
    RingFull,
    RingEmpty,
}

impl Quat {
    #[inline(always)]
    pub const fn from_bits_masked(raw: u8) -> Self {
        match raw & 0b11 {
            0b00 => Quat::Q00,
            0b01 => Quat::Q01,
            0b10 => Quat::Q10,
            _ => Quat::Q11,
        }
    }

    #[inline(always)]
    pub const fn try_from_bits(raw: u8) -> Result<Self, QuatError> {
        match raw {
            0b00 => Ok(Quat::Q00),
            0b01 => Ok(Quat::Q01),
            0b10 => Ok(Quat::Q10),
            0b11 => Ok(Quat::Q11),
            other => Err(QuatError::InvalidBits(other)),
        }
    }

    #[inline(always)]
    pub const fn from_bool_pair(high: bool, low: bool) -> Self {
        let bits = ((high as u8) << 1) | (low as u8);
        Self::from_bits_masked(bits)
    }

    #[inline(always)]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    #[inline(always)]
    pub const fn invert(self) -> Self {
        Quat::from_bits_masked(0b11 ^ self.bits())
    }

    #[inline(always)]
    pub const fn q_max(self, rhs: Self) -> Self {
        if self.bits() >= rhs.bits() {
            self
        } else {
            rhs
        }
    }

    #[inline(always)]
    pub const fn q_min(self, rhs: Self) -> Self {
        if self.bits() <= rhs.bits() {
            self
        } else {
            rhs
        }
    }

    #[inline(always)]
    pub const fn q_xor(self, rhs: Self) -> Self {
        Quat::from_bits_masked(self.bits() ^ rhs.bits())
    }

    #[inline(always)]
    pub const fn mux(sel: Self, when_low: Self, when_high: Self) -> Self {
        if sel.bits() >= 0b10 {
            when_high
        } else {
            when_low
        }
    }

    #[inline(always)]
    pub const fn rot_inc(self) -> Self {
        Quat::from_bits_masked(self.bits().wrapping_add(1))
    }

    #[inline(always)]
    pub const fn sat_add(self, rhs: Self) -> Self {
        let sum = self.bits() + rhs.bits();
        if sum >= 0b11 {
            Quat::Q11
        } else {
            Quat::from_bits_masked(sum)
        }
    }

    #[inline(always)]
    pub const fn sat_sub(self, rhs: Self) -> Self {
        Quat::from_bits_masked(self.bits().saturating_sub(rhs.bits()))
    }
}

impl fmt::Debug for Quat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Quat::Q00 => f.write_str("00"),
            Quat::Q01 => f.write_str("01"),
            Quat::Q10 => f.write_str("10"),
            Quat::Q11 => f.write_str("11"),
        }
    }
}
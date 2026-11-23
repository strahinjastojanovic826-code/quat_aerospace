use crate::logic::{Quat, QuatError};
use crate::precision::QuatWord4;

pub struct PackedQuatBuf<'a> {
    storage: &'a mut [u8],
    len: usize,
}

impl<'a> PackedQuatBuf<'a> {
    pub fn new_zeroed(storage: &'a mut [u8], len: usize) -> Result<Self, QuatError> {
        let needed_bytes = Self::required_bytes(len);
        if storage.len() < needed_bytes {
            return Err(QuatError::BufferTooSmall);
        }
        storage[..needed_bytes].fill(0);
        Ok(Self { storage, len })
    }

    pub fn from_raw(storage: &'a mut [u8], len: usize) -> Result<Self, QuatError> {
        let needed_bytes = Self::required_bytes(len);
        if storage.len() < needed_bytes {
            return Err(QuatError::BufferTooSmall);
        }
        Ok(Self { storage, len })
    }

    #[inline(always)]
    pub const fn required_bytes(quat_count: usize) -> usize {
        quat_count.div_ceil(4)
    }

    #[inline(always)]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[inline(always)]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline(always)]
    pub fn as_bytes(&self) -> &[u8] {
        let used = Self::required_bytes(self.len);
        &self.storage[..used]
    }

    #[inline(always)]
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        let used = Self::required_bytes(self.len);
        &mut self.storage[..used]
    }

    pub fn fill(&mut self, value: Quat) {
        let b = value.bits();
        let byte_pattern = b | (b << 2) | (b << 4) | (b << 6);
        let used = Self::required_bytes(self.len);
        self.storage[..used].fill(byte_pattern);
    }

    pub fn pack_from_slice(&mut self, src: &[Quat]) -> Result<(), QuatError> {
        if src.len() > self.len {
            return Err(QuatError::BufferTooSmall);
        }
        for (i, &q) in src.iter().enumerate() {
            self.set(i, q)?;
        }
        Ok(())
    }

    pub fn unpack_into_slice(&self, dst: &mut [Quat]) -> Result<(), QuatError> {
        if dst.len() < self.len {
            return Err(QuatError::BufferTooSmall);
        }
        for (i, slot) in dst[..self.len].iter_mut().enumerate() {
            *slot = self.get(i)?;
        }
        Ok(())
    }

    /// Reads 4 consecutive quaternaries starting at byte-aligned `quat_index` (`quat_index % 4 == 0`).
    #[inline]
    pub fn word4_at_aligned(&self, quat_index: usize) -> Result<QuatWord4, QuatError> {
        if quat_index >= self.len || (quat_index & 0b11) != 0 {
            return Err(QuatError::IndexOutOfBounds);
        }
        Ok(QuatWord4(self.storage[quat_index >> 2]))
    }

    #[inline(always)]
    fn byte_and_shift(index: usize) -> (usize, u8) {
        let byte_idx = index >> 2;
        let shift = ((index & 0b11) as u8) * 2;
        (byte_idx, shift)
    }

    #[inline]
    pub fn get(&self, index: usize) -> Result<Quat, QuatError> {
        if index >= self.len {
            return Err(QuatError::IndexOutOfBounds);
        }
        let (byte_idx, shift) = Self::byte_and_shift(index);
        let raw = (self.storage[byte_idx] >> shift) & 0b11;
        Ok(Quat::from_bits_masked(raw))
    }

    #[inline]
    pub fn set(&mut self, index: usize, value: Quat) -> Result<(), QuatError> {
        if index >= self.len {
            return Err(QuatError::IndexOutOfBounds);
        }
        let (byte_idx, shift) = Self::byte_and_shift(index);
        let mask = !(0b11u8 << shift);
        self.storage[byte_idx] = (self.storage[byte_idx] & mask) | (value.bits() << shift);
        Ok(())
    }

    pub fn iter(&self) -> PackedQuatIter<'_> {
        PackedQuatIter {
            buf: self,
            index: 0,
        }
    }
}

pub struct PackedQuatIter<'a> {
    buf: &'a PackedQuatBuf<'a>,
    index: usize,
}

impl<'a> Iterator for PackedQuatIter<'a> {
    type Item = Quat;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.buf.len() {
            return None;
        }
        let val = self.buf.get(self.index).ok()?;
        self.index += 1;
        Some(val)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let rem = self.buf.len().saturating_sub(self.index);
        (rem, Some(rem))
    }
}
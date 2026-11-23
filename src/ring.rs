use core::sync::atomic::{AtomicUsize, Ordering};

use crate::logic::{Quat, QuatError};

/// Lock-free single-producer single-consumer (SPSC) packed quaternary ring buffer.
///
/// Capacity is `4 * backing.len()` quaternary elements packed at 2 bits per element.
pub struct PackedQuatRing<'a> {
    backing: &'a mut [u8],
    capacity_quats: usize,
    head: AtomicUsize, // next write quat index (monotonically increasing)
    tail: AtomicUsize, // next read quat index (monotonically increasing)
}

impl<'a> PackedQuatRing<'a> {
    /// Creates a new ring buffer over `backing`.
    /// Requires `backing.len() >= 1` and `backing.len().is_power_of_two()` for fast wrap-around.
    pub fn new(backing: &'a mut [u8]) -> Result<Self, QuatError> {
        if backing.is_empty() || !backing.len().is_power_of_two() {
            return Err(QuatError::BufferTooSmall);
        }
        backing.fill(0);
        let capacity_quats = backing.len() * 4;
        Ok(Self {
            backing,
            capacity_quats,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        })
    }

    #[inline]
    pub const fn capacity(&self) -> usize {
        self.capacity_quats
    }

    #[inline]
    pub fn len(&self) -> usize {
        let h = self.head.load(Ordering::Acquire);
        let t = self.tail.load(Ordering::Acquire);
        h.wrapping_sub(t).min(self.capacity_quats)
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == self.capacity_quats
    }

    /// Pushes a single `Quat` from the producer side.
    pub fn push(&self, value: Quat) -> Result<(), QuatError> {
        let h = self.head.load(Ordering::Relaxed);
        let t = self.tail.load(Ordering::Acquire);
        if h.wrapping_sub(t) >= self.capacity_quats {
            return Err(QuatError::RingFull);
        }

        let slot = h & (self.capacity_quats - 1);
        let byte_idx = slot >> 2;
        let shift = ((slot & 0b11) as u8) * 2;
        let mask = !(0b11u8 << shift);

        // SAFETY: SPSC producer owns `slot` (`h` is only updated by producer after writing).
        unsafe {
            let ptr = self.backing.as_ptr().add(byte_idx) as *mut u8;
            let cur = core::ptr::read_volatile(ptr);
            let next = (cur & mask) | (value.bits() << shift);
            core::ptr::write_volatile(ptr, next);
        }

        self.head.store(h.wrapping_add(1), Ordering::Release);
        Ok(())
    }

    /// Pops a single `Quat` from the consumer side.
    pub fn pop(&self) -> Result<Quat, QuatError> {
        let t = self.tail.load(Ordering::Relaxed);
        let h = self.head.load(Ordering::Acquire);
        if h.wrapping_sub(t) == 0 {
            return Err(QuatError::RingEmpty);
        }

        let slot = t & (self.capacity_quats - 1);
        let byte_idx = slot >> 2;
        let shift = ((slot & 0b11) as u8) * 2;

        // SAFETY: SPSC consumer only reads committed slots strictly before `head`.
        let raw = unsafe {
            let ptr = self.backing.as_ptr().add(byte_idx);
            (core::ptr::read_volatile(ptr) >> shift) & 0b11
        };

        self.tail.store(t.wrapping_add(1), Ordering::Release);
        Ok(Quat::from_bits_masked(raw))
    }
}
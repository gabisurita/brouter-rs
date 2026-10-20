//! High-speed non-cryptographic hasher and hash collections.
//!
//! Provides `FastHashMap` and `FastHashSet` using a lightweight multi-round
//! rotate-and-multiply hash algorithm optimized for 64-bit integer graph keys.

use std::collections::{HashMap, HashSet};
use std::hash::{BuildHasherDefault, Hasher};

/// Fast non-cryptographic hasher for 64-bit integer graph node keys.
#[derive(Default)]
pub struct FastHasher(u64);

impl Hasher for FastHasher {
    #[inline(always)]
    fn finish(&self) -> u64 {
        self.0
    }

    #[inline(always)]
    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = self.0.rotate_left(5) ^ (byte as u64);
            self.0 = self.0.wrapping_mul(0x517cc1b727220a95);
        }
    }

    #[inline(always)]
    fn write_u32(&mut self, i: u32) {
        self.0 = (i as u64).wrapping_mul(0x517cc1b727220a95);
    }

    #[inline(always)]
    fn write_u64(&mut self, i: u64) {
        self.0 = self.0.rotate_left(5) ^ i;
        self.0 = self.0.wrapping_mul(0x517cc1b727220a95);
    }
}

pub type FastBuildHasher = BuildHasherDefault<FastHasher>;
pub type FastHashMap<K, V> = HashMap<K, V, FastBuildHasher>;
pub type FastHashSet<T> = HashSet<T, FastBuildHasher>;

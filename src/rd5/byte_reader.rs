//! Rust implementation corresponding to:
//! - Java class: `btools.util.ByteDataReader`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-util/src/main/java/btools/util/ByteDataReader.java`
//!
//! # Summary
//! Fast binary data reading for big-endian integers, longs, shorts, and LEB128 variable-length numbers.

pub struct ByteDataReader<'a> {
    pub ab: &'a [u8],
    pub offset: usize,
}

impl<'a> ByteDataReader<'a> {
    pub fn new(ab: &'a [u8]) -> Self {
        Self { ab, offset: 0 }
    }

    pub fn with_offset(ab: &'a [u8], offset: usize) -> Self {
        Self { ab, offset }
    }

    pub fn read_int(&mut self) -> i32 {
        let b0 = self.ab[self.offset] as u32;
        let b1 = self.ab[self.offset + 1] as u32;
        let b2 = self.ab[self.offset + 2] as u32;
        let b3 = self.ab[self.offset + 3] as u32;
        self.offset += 4;
        ((b0 << 24) | (b1 << 16) | (b2 << 8) | b3) as i32
    }

    pub fn read_long(&mut self) -> i64 {
        let mut val = 0u64;
        for i in 0..8 {
            val = (val << 8) | (self.ab[self.offset + i] as u64);
        }
        self.offset += 8;
        val as i64
    }

    pub fn read_byte(&mut self) -> i8 {
        let b = self.ab[self.offset] as i8;
        self.offset += 1;
        b
    }

    /// Reads an unsigned variable-length integer encoded in LEB128 format (7 bits per byte).
    ///
    /// Matches `ByteDataReader.readVarLengthUnsigned` in Java BRouter.
    pub fn read_var_length_unsigned(&mut self) -> i32 {
        let mut b = self.ab[self.offset] as i8;
        self.offset += 1;
        let mut v = (b as i32) & 0x7f;
        if b >= 0 {
            return v;
        }

        b = self.ab[self.offset] as i8;
        self.offset += 1;
        v |= ((b as i32) & 0x7f) << 7;
        if b >= 0 {
            return v;
        }

        b = self.ab[self.offset] as i8;
        self.offset += 1;
        v |= ((b as i32) & 0x7f) << 14;
        if b >= 0 {
            return v;
        }

        b = self.ab[self.offset] as i8;
        self.offset += 1;
        v |= ((b as i32) & 0x7f) << 21;
        if b >= 0 {
            return v;
        }

        b = self.ab[self.offset] as i8;
        self.offset += 1;
        v |= ((b as i32) & 0xf) << 28;
        v
    }

    /// Reads a signed variable-length integer using least-significant-bit sign encoding:
    /// even values are positive (`v >> 1`), odd values are negative (`-(v >> 1)`).
    ///
    /// Matches `ByteDataReader.readVarLengthSigned` in Java BRouter.
    pub fn read_var_length_signed(&mut self) -> i32 {
        let v = self.read_var_length_unsigned();
        if (v & 1) == 0 {
            v >> 1
        } else {
            -(v >> 1)
        }
    }

    pub fn has_more_data(&self) -> bool {
        self.offset < self.ab.len()
    }
}

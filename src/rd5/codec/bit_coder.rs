//! Rust implementation corresponding to:
//! - Java class: `btools.util.BitCoderContext`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-util/src/main/java/btools/util/BitCoderContext.java`
//! - Java class: `btools.codec.StatCoderContext`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/main/java/btools/codec/StatCoderContext.java`
//! - Java class: `btools.codec.NoisyDiffCoder`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/main/java/btools/codec/NoisyDiffCoder.java`
//!
//! # Summary
//! Bit-level coder providing variable-bit integer encoding/decoding, bounded
//! integers, noisy diff decoding, predicted coordinate values, and sorted Morton arrays.

use std::sync::OnceLock;

/// Lookup tables initialized once for fast variable-bit coding.
struct Tables {
    vl_values: [i32; 4096],
    vl_length: [i32; 4096],
    vc_values: [i32; 4096],
    vc_length: [i32; 4096],
    reverse_byte: [u8; 256],
    noisy_bits: [i32; 1024],
}

static TABLES: OnceLock<Tables> = OnceLock::new();

fn get_tables() -> &'static Tables {
    TABLES.get_or_init(|| {
        let mut vl_values = [0i32; 4096];
        let mut vl_length = [0i32; 4096];
        let mut vc_values = [0i32; 4096];
        let mut vc_length = [0i32; 4096];
        let mut reverse_byte = [0u8; 256];
        let mut noisy_bits = [0i32; 1024];

        let mut dummy = [0u8; 4];
        let mut bc = BitCoderContext::new(&mut dummy);

        for i in 0..4096 {
            bc.reset();
            bc.bits = 14;
            bc.b = 0x1000 + i as i32;

            let b0 = bc.get_reading_bit_position();
            vl_values[i] = bc.decode_var_bits2();
            vl_length[i] = bc.get_reading_bit_position() - b0;
        }

        for i in 0..4096 {
            bc.reset();
            let b0 = bc.get_writing_bit_position();
            bc.encode_var_bits2(i as i32);
            vc_values[i] = bc.b;
            vc_length[i] = bc.get_writing_bit_position() - b0;
        }

        for (b, slot) in reverse_byte.iter_mut().enumerate() {
            let mut r = 0u8;
            for i in 0..8 {
                if (b & (1 << i)) != 0 {
                    r |= 1 << (7 - i);
                }
            }
            *slot = r;
        }

        for (i, slot) in noisy_bits.iter_mut().enumerate() {
            let mut p = i;
            let mut nb = 0;
            while p > 2 {
                nb += 1;
                p >>= 1;
            }
            *slot = nb;
        }

        Tables {
            vl_values,
            vl_length,
            vc_values,
            vc_length,
            reverse_byte,
            noisy_bits,
        }
    })
}

/// Bit-level stream reader and writer.
pub struct BitCoderContext<'a> {
    pub ab: &'a mut [u8],
    idx_max: isize,
    idx: isize,
    pub bits: i32,
    pub b: i32,
}

/// Bit-level stream reader over an immutable byte slice (zero allocations).
pub struct BitReader<'a> {
    pub ab: &'a [u8],
    idx_max: isize,
    idx: isize,
    pub bits: i32,
    pub b: i32,
}

impl<'a> BitReader<'a> {
    pub fn new(ab: &'a [u8]) -> Self {
        let len = ab.len() as isize;
        Self {
            ab,
            idx_max: len - 1,
            idx: -1,
            bits: 0,
            b: 0,
        }
    }

    pub fn reset(&mut self) {
        self.idx = -1;
        self.bits = 0;
        self.b = 0;
    }

    pub fn decode_bit(&mut self) -> bool {
        if self.bits == 0 {
            self.bits = 8;
            self.idx += 1;
            self.b = if self.idx < self.ab.len() as isize {
                self.ab[self.idx as usize] as i32 & 0xff
            } else {
                0
            };
        }
        let value = (self.b & 1) != 0;
        self.b = ((self.b as u32) >> 1) as i32;
        self.bits -= 1;
        value
    }

    pub fn decode_bounded(&mut self, max: i32) -> i32 {
        let mut value = 0i32;
        let mut im = 1i32;
        while (value | im) <= max {
            if self.bits == 0 {
                self.bits = 8;
                self.idx += 1;
                self.b = if self.idx < self.ab.len() as isize {
                    self.ab[self.idx as usize] as i32 & 0xff
                } else {
                    0
                };
            }
            if (self.b & 1) != 0 {
                value |= im;
            }
            self.b = ((self.b as u32) >> 1) as i32;
            self.bits -= 1;
            im <<= 1;
        }
        value
    }

    pub fn decode_var_bits2(&mut self) -> i32 {
        let mut range = 0i32;
        while !self.decode_bit() {
            range = 2 * range + 1;
        }
        range + self.decode_bounded(range)
    }

    pub fn decode_var_bits(&mut self) -> i32 {
        self.fill_buffer();
        let tables = get_tables();
        let b12 = (self.b & 0xfff) as usize;
        let len = tables.vl_length[b12];

        if len <= 12 {
            self.b = ((self.b as u32) >> len) as i32;
            self.bits -= len;
            return tables.vl_values[b12];
        }

        if len <= 23 {
            let len2 = len >> 1;
            self.b = ((self.b as u32) >> (len2 + 1)) as i32;
            let mut mask = (0xffff_ffffu32 >> (32 - len2)) as i32;
            mask += self.b & mask;
            self.b = ((self.b as u32) >> len2) as i32;
            self.bits -= len;
            return mask;
        }

        if (self.b & 0x00ff_ffff) != 0 {
            self.b = ((self.b as u32) >> 12) as i32;
            let len3 = 1 + (tables.vl_length[(self.b & 0xfff) as usize] >> 1);
            self.b = ((self.b as u32) >> len3) as i32;
            let len2 = 11 + len3;
            self.bits -= len2 + 1;
            self.fill_buffer();
            let mut mask = (0xffff_ffffu32 >> (32 - len2)) as i32;
            mask += self.b & mask;
            self.b = ((self.b as u32) >> len2) as i32;
            self.bits -= len2;
            return mask;
        }

        self.decode_var_bits2()
    }

    pub fn decode_bits(&mut self, count: i32) -> i32 {
        if count == 0 {
            return 0;
        }
        self.fill_buffer();
        let mask = (0xffff_ffffu32 >> (32 - count)) as i32;
        let value = self.b & mask;
        self.b = ((self.b as u32) >> count) as i32;
        self.bits -= count;
        value
    }

    pub fn decode_bits_reverse(&mut self, mut count: i32) -> i32 {
        self.fill_buffer();
        let tables = get_tables();
        let mut value = 0i32;

        while count > 8 {
            value = (value << 8) | (tables.reverse_byte[(self.b & 0xff) as usize] as i32);
            self.b = ((self.b as u32) >> 8) as i32;
            count -= 8;
            self.bits -= 8;
            self.fill_buffer();
        }

        let rev = (tables.reverse_byte[(self.b & 0xff) as usize] as i32) >> (8 - count);
        value = (value << count) | rev;
        self.bits -= count;
        self.b = ((self.b as u32) >> count) as i32;
        value
    }

    fn fill_buffer(&mut self) {
        while self.bits < 24 {
            self.idx += 1;
            if self.idx <= self.idx_max {
                let byte_val = self.ab[self.idx as usize] as i32 & 0xff;
                self.b |= byte_val << self.bits;
            }
            self.bits += 8;
        }
    }
}

impl<'a> BitCoderContext<'a> {
    pub fn new(ab: &'a mut [u8]) -> Self {
        let len = ab.len() as isize;
        Self {
            ab,
            idx_max: len - 1,
            idx: -1,
            bits: 0,
            b: 0,
        }
    }

    pub fn reset(&mut self) {
        self.idx = -1;
        self.bits = 0;
        self.b = 0;
    }

    pub fn encode_bit(&mut self, value: bool) {
        if self.bits > 31 {
            self.idx += 1;
            if self.idx < self.ab.len() as isize {
                self.ab[self.idx as usize] = (self.b & 0xff) as u8;
            }
            self.b = ((self.b as u32) >> 8) as i32;
            self.bits -= 8;
        }
        if value {
            self.b |= 1 << self.bits;
        }
        self.bits += 1;
    }

    pub fn decode_bit(&mut self) -> bool {
        if self.bits == 0 {
            self.bits = 8;
            self.idx += 1;
            self.b = if self.idx < self.ab.len() as isize {
                self.ab[self.idx as usize] as i32 & 0xff
            } else {
                0
            };
        }
        let value = (self.b & 1) != 0;
        self.b = ((self.b as u32) >> 1) as i32;
        self.bits -= 1;
        value
    }

    pub fn encode_bounded(&mut self, mut max: i32, value: i32) {
        let mut im = 1i32;
        while im <= max {
            if (value & im) != 0 {
                self.encode_bit(true);
                max -= im;
            } else {
                self.encode_bit(false);
            }
            im <<= 1;
        }
    }

    pub fn decode_bounded(&mut self, max: i32) -> i32 {
        let mut value = 0i32;
        let mut im = 1i32;
        while (value | im) <= max {
            if self.bits == 0 {
                self.bits = 8;
                self.idx += 1;
                self.b = if self.idx < self.ab.len() as isize {
                    self.ab[self.idx as usize] as i32 & 0xff
                } else {
                    0
                };
            }
            if (self.b & 1) != 0 {
                value |= im;
            }
            self.b = ((self.b as u32) >> 1) as i32;
            self.bits -= 1;
            im <<= 1;
        }
        value
    }

    pub fn encode_var_bits2(&mut self, mut value: i32) {
        let mut range = 0i32;
        while value > range {
            self.encode_bit(false);
            value -= range + 1;
            range = 2 * range + 1;
        }
        self.encode_bit(true);
        self.encode_bounded(range, value);
    }

    pub fn decode_var_bits2(&mut self) -> i32 {
        let mut range = 0i32;
        while !self.decode_bit() {
            range = 2 * range + 1;
        }
        range + self.decode_bounded(range)
    }

    pub fn encode_var_bits(&mut self, value: i32) {
        let tables = get_tables();
        if (value & 0xfff) == value {
            self.flush_buffer();
            self.b |= tables.vc_values[value as usize] << self.bits;
            self.bits += tables.vc_length[value as usize];
        } else {
            self.encode_var_bits2(value);
        }
    }

    pub fn decode_var_bits(&mut self) -> i32 {
        self.fill_buffer();
        let tables = get_tables();
        let b12 = (self.b & 0xfff) as usize;
        let len = tables.vl_length[b12];

        if len <= 12 {
            self.b = ((self.b as u32) >> len) as i32;
            self.bits -= len;
            return tables.vl_values[b12];
        }

        if len <= 23 {
            let len2 = len >> 1;
            self.b = ((self.b as u32) >> (len2 + 1)) as i32;
            let mut mask = (0xffff_ffffu32 >> (32 - len2)) as i32;
            mask += self.b & mask;
            self.b = ((self.b as u32) >> len2) as i32;
            self.bits -= len;
            return mask;
        }

        if (self.b & 0x00ff_ffff) != 0 {
            self.b = ((self.b as u32) >> 12) as i32;
            let len3 = 1 + (tables.vl_length[(self.b & 0xfff) as usize] >> 1);
            self.b = ((self.b as u32) >> len3) as i32;
            let len2 = 11 + len3;
            self.bits -= len2 + 1;
            self.fill_buffer();
            let mut mask = (0xffff_ffffu32 >> (32 - len2)) as i32;
            mask += self.b & mask;
            self.b = ((self.b as u32) >> len2) as i32;
            self.bits -= len2;
            return mask;
        }

        self.decode_var_bits2()
    }

    pub fn decode_bits(&mut self, count: i32) -> i32 {
        if count == 0 {
            return 0;
        }
        self.fill_buffer();
        let mask = (0xffff_ffffu32 >> (32 - count)) as i32;
        let value = self.b & mask;
        self.b = ((self.b as u32) >> count) as i32;
        self.bits -= count;
        value
    }

    pub fn decode_bits_reverse(&mut self, mut count: i32) -> i32 {
        self.fill_buffer();
        let tables = get_tables();
        let mut value = 0i32;

        while count > 8 {
            value = (value << 8) | (tables.reverse_byte[(self.b & 0xff) as usize] as i32);
            self.b = ((self.b as u32) >> 8) as i32;
            count -= 8;
            self.bits -= 8;
            self.fill_buffer();
        }

        let rev = (tables.reverse_byte[(self.b & 0xff) as usize] as i32) >> (8 - count);
        value = (value << count) | rev;
        self.bits -= count;
        self.b = ((self.b as u32) >> count) as i32;
        value
    }

    fn fill_buffer(&mut self) {
        while self.bits < 24 {
            self.idx += 1;
            if self.idx <= self.idx_max {
                let byte_val = self.ab[self.idx as usize] as i32 & 0xff;
                self.b |= byte_val << self.bits;
            }
            self.bits += 8;
        }
    }

    fn flush_buffer(&mut self) {
        while self.bits > 7 {
            self.idx += 1;
            if self.idx < self.ab.len() as isize {
                self.ab[self.idx as usize] = (self.b & 0xff) as u8;
            }
            self.b = ((self.b as u32) >> 8) as i32;
            self.bits -= 8;
        }
    }

    pub fn close_and_get_encoded_length(&mut self) -> usize {
        self.flush_buffer();
        if self.bits > 0 {
            self.idx += 1;
            if self.idx < self.ab.len() as isize {
                self.ab[self.idx as usize] = (self.b & 0xff) as u8;
            }
        }
        (self.idx + 1).max(0) as usize
    }

    pub fn get_writing_bit_position(&self) -> i32 {
        ((self.idx << 3) + 8 + self.bits as isize) as i32
    }

    pub fn get_reading_bit_position(&self) -> i32 {
        ((self.idx << 3) + 8 - self.bits as isize) as i32
    }

    pub fn set_reading_bit_position(&mut self, pos: i32) {
        self.idx = (pos >> 3) as isize;
        self.bits = ((self.idx << 3) + 8 - pos as isize) as i32;
        self.b = if self.idx >= 0 && self.idx < self.ab.len() as isize {
            self.ab[self.idx as usize] as i32 & 0xff
        } else {
            0
        };
        self.b = ((self.b as u32) >> (8 - self.bits)) as i32;
    }
}

/// StatCoderContext provides statistical number decoding for RD5 files.
pub struct StatCoderContext<'a> {
    pub bit_coder: BitCoderContext<'a>,
}

impl<'a> StatCoderContext<'a> {
    pub fn new(ab: &'a mut [u8]) -> Self {
        Self {
            bit_coder: BitCoderContext::new(ab),
        }
    }

    pub fn decode_var_bits(&mut self) -> i32 {
        self.bit_coder.decode_var_bits()
    }

    pub fn decode_bit(&mut self) -> bool {
        self.bit_coder.decode_bit()
    }

    pub fn decode_bounded(&mut self, max: i32) -> i32 {
        self.bit_coder.decode_bounded(max)
    }

    pub fn encode_noisy_number(&mut self, mut value: i32, noisybits: i32) {
        assert!(value >= 0, "encode_noisy_number expects non-negative value");
        if noisybits > 0 {
            let mask = (0xffff_ffffu32 >> (32 - noisybits)) as i32;
            self.bit_coder.encode_bounded(mask, value & mask);
            value >>= noisybits;
        }
        self.bit_coder.encode_var_bits(value);
    }

    pub fn decode_noisy_number(&mut self, noisybits: i32) -> i32 {
        let value = self.bit_coder.decode_bits(noisybits);
        value | (self.bit_coder.decode_var_bits() << noisybits)
    }

    pub fn encode_noisy_diff(&mut self, mut value: i32, noisybits: i32) {
        if noisybits > 0 {
            value += 1 << (noisybits - 1);
            let mask = (0xffff_ffffu32 >> (32 - noisybits)) as i32;
            self.bit_coder.encode_bounded(mask, value & mask);
            value >>= noisybits;
        }
        self.bit_coder
            .encode_var_bits(if value < 0 { -value } else { value });
        if value != 0 {
            self.bit_coder.encode_bit(value < 0);
        }
    }

    pub fn decode_noisy_diff(&mut self, noisybits: i32) -> i32 {
        let mut value = 0i32;
        if noisybits > 0 {
            value = self.bit_coder.decode_bits(noisybits) - (1 << (noisybits - 1));
        }
        let mut val2 = self.bit_coder.decode_var_bits() << noisybits;
        if val2 != 0 && self.bit_coder.decode_bit() {
            val2 = -val2;
        }
        value + val2
    }

    pub fn encode_predicted_value(&mut self, value: i32, predictor: i32) {
        let mut p = if predictor < 0 { -predictor } else { predictor };
        let mut noisybits = 0;
        while p > 2 {
            noisybits += 1;
            p >>= 1;
        }
        self.encode_noisy_diff(value - predictor, noisybits);
    }

    pub fn decode_predicted_value(&mut self, predictor: i32) -> i32 {
        let tables = get_tables();
        let mut p = if predictor < 0 { -predictor } else { predictor };
        let mut noisybits = 0;
        while p > 1023 {
            noisybits += 1;
            p >>= 1;
        }
        predictor + self.decode_noisy_diff(noisybits + tables.noisy_bits[p as usize])
    }

    pub fn encode_sorted_array(
        &mut self,
        values: &[i32],
        offset: usize,
        subsize: usize,
        mut nextbit: i32,
        mut mask: i32,
    ) {
        if subsize == 1 {
            while nextbit != 0 {
                self.bit_coder.encode_bit((values[offset] & nextbit) != 0);
                nextbit >>= 1;
            }
        }
        if nextbit == 0 {
            return;
        }

        let data = mask & values[offset];
        mask |= nextbit;

        let mut i = offset;
        let end = subsize + offset;
        while i < end {
            if (values[i] & mask) != data {
                break;
            }
            i += 1;
        }

        let size1 = (i - offset) as i32;
        let size2 = (subsize as i32) - size1;

        self.bit_coder.encode_bounded(subsize as i32, size1);
        if size1 > 0 {
            self.encode_sorted_array(values, offset, size1 as usize, nextbit >> 1, mask);
        }
        if size2 > 0 {
            self.encode_sorted_array(values, i, size2 as usize, nextbit >> 1, mask);
        }
    }

    pub fn decode_sorted_array(
        &mut self,
        values: &mut [i32],
        mut offset: usize,
        mut subsize: usize,
        nextbitpos: i32,
        mut value: i32,
    ) {
        if subsize == 1 {
            if nextbitpos >= 0 {
                value |= self.bit_coder.decode_bits_reverse(nextbitpos + 1);
            }
            values[offset] = value;
            return;
        }
        if nextbitpos < 0 {
            while subsize > 0 {
                values[offset] = value;
                offset += 1;
                subsize -= 1;
            }
            return;
        }

        let size1 = self.bit_coder.decode_bounded(subsize as i32) as usize;
        let size2 = subsize - size1;

        if size1 > 0 {
            self.decode_sorted_array(values, offset, size1, nextbitpos - 1, value);
        }
        if size2 > 0 {
            self.decode_sorted_array(
                values,
                offset + size1,
                size2,
                nextbitpos - 1,
                value | (1 << nextbitpos),
            );
        }
    }
}

/// NoisyDiffCoder decodes signed differences using an adaptively encoded noisy-bits count.
pub struct NoisyDiffCoder {
    pub noisybits: i32,
}

impl NoisyDiffCoder {
    pub fn new(bc: &mut StatCoderContext) -> Self {
        let noisybits = bc.decode_var_bits();
        Self { noisybits }
    }

    pub fn decode_signed_value(&self, bc: &mut StatCoderContext) -> i32 {
        bc.decode_noisy_diff(self.noisybits)
    }
}

//! Rust implementation corresponding to:
//! - Java class: `btools.codec.MicroCache`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/main/java/btools/codec/MicroCache.java`
//! - Java class: `btools.codec.MicroCache2`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/main/java/btools/codec/MicroCache2.java`
//! - Java class: `btools.mapaccess.DirectWeaver`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/DirectWeaver.java`
//!
//! # Summary
//! Decodes a single MicroCache cell from a raw bitstream into nodes, turn restrictions,
//! links, geometries, and elevations, matching DirectWeaver and MicroCache2.

use std::io::{self, Error, ErrorKind};
use std::sync::OnceLock;

use super::byte_reader::ByteDataReader;
use super::codec::bit_coder::{NoisyDiffCoder, StatCoderContext};
use super::codec::tag_coder::{TagValueCoder, TagValueValidator, TagValueWrapper};
use super::crc;

/// Precomputed tables for fast 29-bit Morton/quadtree coordinate expansion.
struct MortonTables {
    id32_00_lon: [i32; 1024],
    id32_00_lat: [i32; 1024],
    id32_10_lon: [i32; 1024],
    id32_10_lat: [i32; 1024],
    id32_20_lon: [i32; 1024],
    id32_20_lat: [i32; 1024],
}

static MORTON_TABLES: OnceLock<MortonTables> = OnceLock::new();

fn get_morton_tables() -> &'static MortonTables {
    MORTON_TABLES.get_or_init(|| {
        let mut t = MortonTables {
            id32_00_lon: [0; 1024],
            id32_00_lat: [0; 1024],
            id32_10_lon: [0; 1024],
            id32_10_lat: [0; 1024],
            id32_20_lon: [0; 1024],
            id32_20_lat: [0; 1024],
        };

        for i in 0..1024 {
            let (lon0, lat0) = deinterleave(i);
            t.id32_00_lon[i] = lon0;
            t.id32_00_lat[i] = lat0;

            let (lon1, lat1) = deinterleave(i << 10);
            t.id32_10_lon[i] = lon1;
            t.id32_10_lat[i] = lat1;

            let (lon2, lat2) = deinterleave(i << 20);
            t.id32_20_lon[i] = lon2;
            t.id32_20_lat[i] = lat2;
        }

        t
    })
}

/// Bit-deinterleaves a Morton quadtree index into delta longitude and delta latitude.
///
/// Matches Java class `btools.codec.MicroCache2`:
/// Even bits map to `dlon`, odd bits map to `dlat`.
fn deinterleave(mut id32: usize) -> (i32, i32) {
    let mut dlon = 0i32;
    let mut dlat = 0i32;
    let mut bm = 1i32;
    while bm < 0x8000 {
        if (id32 & 1) != 0 {
            dlon |= bm;
        }
        if (id32 & 2) != 0 {
            dlat |= bm;
        }
        id32 >>= 2;
        bm <<= 1;
    }
    (dlon, dlat)
}

/// Expands a 29-bit Morton quadtree node ID into absolute integer microdegrees `(ilon, ilat)`.
///
/// Matches `MicroCache2.expandCoords`:
/// Uses three 10-bit lookups from precomputed `MORTON_TABLES` to reconstruct coordinates
/// within the microcache cell without bit loops.
fn expand_coords(lon_idx: i32, lat_idx: i32, cellsize: i32, id32: i32) -> (i32, i32) {
    let tables = get_morton_tables();
    let idx = id32 as usize;
    let i0 = idx & 1023;
    let i1 = (idx >> 10) & 1023;
    let i2 = (idx >> 20) & 1023;

    let dlon = tables.id32_00_lon[i0] + tables.id32_10_lon[i1] + tables.id32_20_lon[i2];
    let dlat = tables.id32_00_lat[i0] + tables.id32_10_lat[i1] + tables.id32_20_lat[i2];

    (lon_idx * cellsize + dlon, lat_idx * cellsize + dlat)
}

use crate::types::Point2D;

#[derive(Debug, Clone, PartialEq)]
pub struct DecodedTurnRestriction {
    pub exceptions: i16,
    pub is_positive: bool,
    pub from: Point2D,
    pub to: Point2D,
}

impl DecodedTurnRestriction {
    pub fn is_turn_forbidden(
        restrictions: &[DecodedTurnRestriction],
        from: Point2D,
        to: Point2D,
        bike_mode: bool,
        car_mode: bool,
    ) -> bool {
        let mut has_any_positive = false;
        let mut has_positive = false;
        let mut has_negative = false;

        for tr in restrictions {
            if ((tr.exceptions & 1) != 0 && bike_mode) || ((tr.exceptions & 2) != 0 && car_mode) {
                continue;
            }
            if tr.from == from {
                if tr.is_positive {
                    has_any_positive = true;
                }
                if tr.to == to {
                    if tr.is_positive {
                        has_positive = true;
                    } else {
                        has_negative = true;
                    }
                }
            }
        }
        !has_positive && (has_any_positive || has_negative)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DecodedGeometryPoint {
    pub dlon: i32,
    pub dlat: i32,
    pub delev: i32,
}

/// Directed road link decoded from a MicroCache cell.
///
/// Corresponds to Java class `btools.mapaccess.OsmLink`.
#[derive(Debug, Clone)]
pub struct DecodedLink {
    pub target_lon: i32,
    pub target_lat: i32,
    pub is_reverse: bool,
    pub way_tags: Option<TagValueWrapper>,
    pub geometry: Vec<DecodedGeometryPoint>,
}

#[derive(Debug, Clone)]
pub struct DecodedNode {
    pub ilon: i32,
    pub ilat: i32,
    pub selev: i16,
    pub node_tags: Option<TagValueWrapper>,
    pub turn_restrictions: Vec<DecodedTurnRestriction>,
    pub links: Vec<DecodedLink>,
}

#[derive(Debug, Clone)]
pub struct DecodedMicroCache {
    pub lon_idx: i32,
    pub lat_idx: i32,
    pub nodes: Vec<DecodedNode>,
}

impl DecodedMicroCache {
    /// Decodes a MicroCache cell from its raw byte buffer.
    pub fn decode(
        mut cell_bytes: Vec<u8>,
        lon_idx: i32,
        lat_idx: i32,
        divisor: i32,
        way_validator: Option<&dyn TagValueValidator>,
    ) -> io::Result<Self> {
        if cell_bytes.is_empty() {
            return Ok(Self {
                lon_idx,
                lat_idx,
                nodes: Vec::new(),
            });
        }

        let asize = cell_bytes.len();
        if asize < 4 {
            return Err(Error::new(ErrorKind::InvalidData, "Cell data too short"));
        }
        if divisor <= 0 {
            return Err(Error::new(ErrorKind::InvalidData, "Invalid divisor"));
        }

        let cellsize = 1_000_000 / divisor;
        let max_bits = (asize - 4).saturating_mul(8);

        let mut bc = StatCoderContext::new(&mut cell_bytes);

        let way_tag_coder = TagValueCoder::try_new(&mut bc, way_validator)?;
        let node_tag_coder = TagValueCoder::try_new(&mut bc, None)?;
        let node_idx_diff = NoisyDiffCoder::new(&mut bc);
        let node_ele_diff = NoisyDiffCoder::new(&mut bc);
        let ext_lon_diff = NoisyDiffCoder::new(&mut bc);
        let ext_lat_diff = NoisyDiffCoder::new(&mut bc);
        let trans_ele_diff = NoisyDiffCoder::new(&mut bc);
        for coder in [
            &node_idx_diff,
            &node_ele_diff,
            &ext_lon_diff,
            &ext_lat_diff,
            &trans_ele_diff,
        ] {
            if !(0..=30).contains(&coder.noisybits) {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "MicroCache noisybits out of range",
                ));
            }
        }

        let size_i32 = bc.decode_noisy_number(5);
        if bc.bit_coder.is_past_eof() || size_i32 < 0 || (size_i32 as usize) > max_bits {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Invalid MicroCache node count",
            ));
        }
        let size = size_i32 as usize;
        let mut faid = vec![0i32; size];
        bc.decode_sorted_array(&mut faid, 0, size, 29, 0);
        if bc.bit_coder.is_past_eof() {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Truncated MicroCache node index array",
            ));
        }

        let mut nodes = Vec::with_capacity(size);
        for &id32 in &faid {
            let (ilon, ilat) = expand_coords(lon_idx, lat_idx, cellsize, id32);
            nodes.push(DecodedNode {
                ilon,
                ilat,
                selev: 0,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: Vec::new(),
            });
        }

        let _netdatasize = bc.decode_noisy_number(10);

        let mut selev = 0i32;

        for n in 0..size {
            if bc.bit_coder.is_past_eof() {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Truncated MicroCache node body",
                ));
            }
            let ilon = nodes[n].ilon;
            let ilat = nodes[n].ilat;

            // Turn restrictions
            let mut tr_exceptions = 0i16;
            loop {
                let feature_id = bc.decode_var_bits();
                if bc.bit_coder.is_past_eof() || feature_id < 0 {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "Truncated MicroCache turn restriction",
                    ));
                }
                if feature_id == 0 {
                    break;
                }
                let bitsize = bc.decode_noisy_number(5);
                if bc.bit_coder.is_past_eof() || bitsize < 0 || (bitsize as usize) > max_bits {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "Invalid MicroCache feature bitsize",
                    ));
                }

                if feature_id == 2 {
                    tr_exceptions = bc.decode_bounded(1023) as i16;
                } else if feature_id == 1 {
                    let is_positive = bc.decode_bit();
                    let from_lon = ilon.wrapping_add(bc.decode_noisy_diff(10));
                    let from_lat = ilat.wrapping_add(bc.decode_noisy_diff(10));
                    let to_lon = ilon.wrapping_add(bc.decode_noisy_diff(10));
                    let to_lat = ilat.wrapping_add(bc.decode_noisy_diff(10));

                    nodes[n].turn_restrictions.push(DecodedTurnRestriction {
                        exceptions: tr_exceptions,
                        is_positive,
                        from: Point2D::new(from_lon, from_lat),
                        to: Point2D::new(to_lon, to_lat),
                    });
                    tr_exceptions = 0;
                } else {
                    for _ in 0..bitsize {
                        bc.decode_bit();
                    }
                }
            }

            selev = selev.wrapping_add(node_ele_diff.decode_signed_value(&mut bc));
            nodes[n].selev = selev as i16;
            nodes[n].node_tags = node_tag_coder.decode_tag_value_set(&mut bc);

            let links_count_i32 = bc.decode_noisy_number(1);
            if bc.bit_coder.is_past_eof()
                || links_count_i32 < 0
                || (links_count_i32 as usize) > max_bits
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Invalid MicroCache link count",
                ));
            }
            let links_count = links_count_i32 as usize;
            for _ in 0..links_count {
                if bc.bit_coder.is_past_eof() {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "Truncated MicroCache link",
                    ));
                }
                let node_idx_offset = node_idx_diff.decode_signed_value(&mut bc);
                let Some(node_idx_i32) = (n as i32).checked_add(node_idx_offset) else {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "MicroCache internal link node index overflow",
                    ));
                };
                if node_idx_i32 < 0 || (node_idx_i32 as usize) >= nodes.len() {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        "MicroCache internal link node index out of bounds",
                    ));
                }
                let node_idx = node_idx_i32 as usize;

                let is_internal = node_idx != n;
                let mut is_reverse = false;
                let mut dlon_remaining: i32;
                let mut dlat_remaining: i32;
                let target_lon: i32;
                let target_lat: i32;

                if is_internal {
                    dlon_remaining = nodes[node_idx].ilon - ilon;
                    dlat_remaining = nodes[node_idx].ilat - ilat;
                    target_lon = nodes[node_idx].ilon;
                    target_lat = nodes[node_idx].ilat;
                } else {
                    is_reverse = bc.decode_bit();
                    dlon_remaining = ext_lon_diff.decode_signed_value(&mut bc);
                    dlat_remaining = ext_lat_diff.decode_signed_value(&mut bc);
                    target_lon = ilon.wrapping_add(dlon_remaining);
                    target_lat = ilat.wrapping_add(dlat_remaining);
                }

                let way_tags = way_tag_coder.decode_tag_value_set(&mut bc);

                let mut geometry = Vec::new();
                if !is_reverse {
                    let transcount = bc.decode_var_bits();
                    if bc.bit_coder.is_past_eof()
                        || transcount < 0
                        || (transcount as usize) > max_bits
                    {
                        return Err(Error::new(
                            ErrorKind::InvalidData,
                            "Invalid MicroCache geometry point count",
                        ));
                    }
                    let mut count = transcount + 1;
                    for _ in 0..transcount {
                        if bc.bit_coder.is_past_eof() {
                            return Err(Error::new(
                                ErrorKind::InvalidData,
                                "Truncated MicroCache geometry",
                            ));
                        }
                        let dlon = bc.decode_predicted_value(dlon_remaining / count);
                        let dlat = bc.decode_predicted_value(dlat_remaining / count);
                        dlon_remaining = dlon_remaining.wrapping_sub(dlon);
                        dlat_remaining = dlat_remaining.wrapping_sub(dlat);
                        count -= 1;
                        let delev = trans_ele_diff.decode_signed_value(&mut bc);
                        if way_tags.is_some() {
                            geometry.push(DecodedGeometryPoint { dlon, dlat, delev });
                        }
                    }
                }

                if way_tags.is_some() {
                    nodes[n].links.push(DecodedLink {
                        target_lon,
                        target_lat,
                        is_reverse,
                        way_tags,
                        geometry,
                    });
                }
            }
        }

        // Integrity check of cell footer CRC and exact decoded byte length.
        // In BRouter RD5 format, the microcache cell footer stores `crc ^ 2`.
        let read_bytes = ((bc.bit_coder.get_reading_bit_position() + 7) >> 3) as usize;
        let crc_data = crc::crc(&bc.bit_coder.ab[..asize - 4]);
        let mut dis = ByteDataReader::with_offset(bc.bit_coder.ab, asize - 4);
        let crc_footer = dis.read_int();
        if (crc_data ^ 2) != crc_footer {
            return Err(Error::new(
                ErrorKind::InvalidData,
                format!(
                    "MicroCache cell checksum error: computed {:x}, footer {:x}",
                    crc_data ^ 2,
                    crc_footer
                ),
            ));
        }
        if read_bytes != asize - 4 {
            return Err(Error::new(
                ErrorKind::InvalidData,
                format!(
                    "MicroCache cell length mismatch: read {} bytes, expected {}",
                    read_bytes,
                    asize - 4
                ),
            ));
        }

        Ok(Self {
            lon_idx,
            lat_idx,
            nodes,
        })
    }
}

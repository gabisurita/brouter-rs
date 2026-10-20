//! Rust implementation corresponding to:
//! - Java package: `btools.mapaccess`
//! - Java source: `https://github.com/abrensch/brouter/tree/master/brouter-mapaccess/src/main/java/btools/mapaccess`
//!
//! # Summary
//! BRouter binary routing segment (`.rd5`) container, index parsers, and cell decoder.

pub mod byte_reader;
pub mod codec;
pub mod crc;
pub mod micro_cache;
pub mod osm_file;
pub mod physical_file;
pub mod rd5_slice;
pub mod tile_manager;

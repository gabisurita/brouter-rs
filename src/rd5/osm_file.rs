//! Rust implementation corresponding to:
//! - Java class: `btools.mapaccess.OsmFile`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/OsmFile.java`
//!
//! # Summary
//! Reads a 1°×1° square within a 5°×5° RD5 file, parsing the sub-index of `divisor * divisor`
//! MicroCache cell offsets and providing raw byte slices for each cell.

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};

use super::byte_reader::ByteDataReader;
use super::crc;
use super::physical_file::PhysicalFile;

pub struct OsmFile {
    pub lon_degree: i32,
    pub lat_degree: i32,
    pub tile_index: usize,
    pub file_offset: u64,
    pub divisor: i32,
    pub cellsize: i32,
    pub indexsize: usize,
    pub pos_idx: Vec<i32>,
    pub elevation_type: u8,
    pub has_data: bool,
}

impl OsmFile {
    pub fn new(pf: &mut PhysicalFile, lon_degree: i32, lat_degree: i32) -> io::Result<Self> {
        let lon_mod5 = lon_degree % 5;
        let lat_mod5 = lat_degree % 5;
        let tile_index = (lon_mod5 * 5 + lat_mod5) as usize;

        let (start_offset, end_offset) = pf.get_square_offsets(tile_index);
        let divisor = pf.divisor;
        let cellsize = 1_000_000 / divisor;
        let ncaches = (divisor * divisor) as usize;
        let indexsize = ncaches * 4;
        let elevation_type = pf.elevation_type;

        if start_offset == end_offset {
            // Empty 1°×1° square (e.g. ocean)
            return Ok(Self {
                lon_degree,
                lat_degree,
                tile_index,
                file_offset: start_offset,
                divisor,
                cellsize,
                indexsize,
                pos_idx: Vec::new(),
                elevation_type,
                has_data: false,
            });
        }

        let mut index_buf = vec![0u8; indexsize];
        pf.file.seek(SeekFrom::Start(start_offset))?;
        pf.file.read_exact(&mut index_buf)?;

        let header_crc = crc::crc(&index_buf);
        if pf.file_header_crcs[tile_index] != header_crc {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Sub-index checksum mismatch for tile {}", tile_index),
            ));
        }

        let mut dis = ByteDataReader::new(&index_buf);
        let mut pos_idx = Vec::with_capacity(ncaches);
        for _ in 0..ncaches {
            pos_idx.push(dis.read_int());
        }

        Ok(Self {
            lon_degree,
            lat_degree,
            tile_index,
            file_offset: start_offset,
            divisor,
            cellsize,
            indexsize,
            pos_idx,
            elevation_type,
            has_data: true,
        })
    }

    /// Returns `(start_offset, end_offset)` in the physical file for the cell at `sub_idx`.
    ///
    /// Matches `OsmFile.getCellOffsets`:
    /// The sub-index stores cumulative byte offsets for each microcache cell.
    pub fn get_cell_offsets(&self, sub_idx: usize) -> (u64, u64) {
        assert!(sub_idx < self.pos_idx.len(), "sub_idx out of bounds");
        let start_pos = if sub_idx == 0 {
            self.indexsize as i32
        } else {
            self.pos_idx[sub_idx - 1]
        };
        let end_pos = self.pos_idx[sub_idx];
        (
            self.file_offset + start_pos as u64,
            self.file_offset + end_pos as u64,
        )
    }

    pub fn read_cell_bytes(&self, file: &mut File, sub_idx: usize) -> io::Result<Vec<u8>> {
        let (start, end) = self.get_cell_offsets(sub_idx);
        let size = (end - start) as usize;
        if size == 0 {
            return Ok(Vec::new());
        }

        let mut buf = vec![0u8; size];
        file.seek(SeekFrom::Start(start))?;
        file.read_exact(&mut buf)?;
        Ok(buf)
    }
}

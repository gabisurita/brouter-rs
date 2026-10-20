//! Rust implementation corresponding to:
//! - Java class: `btools.mapaccess.PhysicalFile`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/PhysicalFile.java`
//!
//! # Summary
//! Reads a 5°×5° BRouter binary segment file (`.rd5`), containing 25 1°×1° square index offsets,
//! file trailer metadata, divisor detection (80 or 32), and sub-index CRC checksums.

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::Path;

use super::byte_reader::ByteDataReader;
use super::crc;

pub struct PhysicalFile {
    pub file_name: String,
    pub file: File,
    pub file_index: [u64; 25],
    pub file_header_crcs: [i32; 25],
    pub creation_time: i64,
    pub divisor: i32,
    pub elevation_type: u8,
    pub lookup_version: i16,
}

impl PhysicalFile {
    pub fn open<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let file_name = path
            .as_ref()
            .file_name()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_default();
        let mut file = File::open(path)?;

        let mut header_buf = [0u8; 200];
        file.read_exact(&mut header_buf)?;
        let file_index_crc = crc::crc(&header_buf);

        let mut dis = ByteDataReader::new(&header_buf);
        let mut file_index = [0u64; 25];
        let mut lookup_version = -1i16;

        for (i, entry) in file_index.iter_mut().enumerate() {
            let lv = dis.read_long() as u64;
            if i == 0 {
                lookup_version = (lv >> 48) as i16;
            }
            *entry = lv & 0x0000_ffff_ffff_ffff;
        }

        let file_len = file.metadata()?.len();
        let pos = file_index[24];
        let mut extra_len = 8 + 26 * 4;

        if file_len == pos {
            // Old format without trailer
            return Ok(Self {
                file_name,
                file,
                file_index,
                file_header_crcs: [0; 25],
                creation_time: 0,
                divisor: 80,
                elevation_type: 3,
                lookup_version,
            });
        }

        if file_len > pos + extra_len as u64 {
            extra_len += 1;
        }

        if file_len < pos + extra_len as u64 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "File size {} too short, expected at least {}",
                    file_len,
                    pos + extra_len as u64
                ),
            ));
        }

        let mut trailer_buf = vec![0u8; extra_len];
        file.seek(SeekFrom::Start(pos))?;
        file.read_exact(&mut trailer_buf)?;

        let mut dis = ByteDataReader::new(&trailer_buf);
        let creation_time = dis.read_long();
        let crc_data = dis.read_int();

        // In BRouter RD5 format, bit 1 of the top index CRC is flipped (`^ 2`)
        // to flag a 32x32 microcache cell divisor instead of the standard 80x80 divisor.
        // Matches Java class `btools.mapaccess.PhysicalFile`:
        //   divisor = crcData == fileIndexCrc ? 80 : ((crcData ^ 2) == fileIndexCrc ? 32 : throw...)
        let divisor = if crc_data == file_index_crc {
            80
        } else if (crc_data ^ 2) == file_index_crc {
            32
        } else {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Top index checksum error in RD5 file",
            ));
        };

        let mut file_header_crcs = [0i32; 25];
        for crc_entry in &mut file_header_crcs {
            *crc_entry = dis.read_int();
        }

        let elevation_type = if dis.has_more_data() {
            dis.read_byte() as u8
        } else {
            3
        };

        Ok(Self {
            file_name,
            file,
            file_index,
            file_header_crcs,
            creation_time,
            divisor,
            elevation_type,
            lookup_version,
        })
    }

    /// Returns `(start_offset, end_offset)` for the given tile index in `0..25`.
    pub fn get_square_offsets(&self, tile_index: usize) -> (u64, u64) {
        assert!(tile_index < 25, "Tile index out of range");
        let start = if tile_index > 0 {
            self.file_index[tile_index - 1]
        } else {
            200
        };
        let end = self.file_index[tile_index];
        (start, end)
    }

    /// Returns true if the 1°×1° square at `tile_index` contains graph data.
    pub fn has_data_for_tile(&self, tile_index: usize) -> bool {
        let (start, end) = self.get_square_offsets(tile_index);
        start < end
    }
}

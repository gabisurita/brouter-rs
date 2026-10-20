//! On-demand reader and cell provider (`LazyRd5Provider`) for 5°×5° RD5 segment data.

use std::io::{self, Error, ErrorKind};

use super::byte_reader::ByteDataReader;
use super::crc;
use super::micro_cache::DecodedMicroCache;

/// Trait for random-access byte slice reading from 5°×5° RD5 segments (e.g. disk files or Worker Blobs).
pub trait Rd5SegmentReader: Send + Sync {
    /// Returns the total byte length of `<segment_name>.rd5` (e.g. `"W5_N50"`), or `None` if unavailable.
    fn segment_len(&self, segment_name: &str) -> Option<u64>;

    /// Reads the byte range `[start..end]` from `<segment_name>.rd5`.
    fn read_range(&self, segment_name: &str, start: u64, end: u64) -> io::Result<Vec<u8>>;
}

/// Filesystem-backed `Rd5SegmentReader` for native random-access cell loading.
pub struct FileRd5SegmentReader {
    segment_dir: std::path::PathBuf,
    open_files: std::sync::RwLock<
        std::collections::HashMap<String, Option<(std::sync::Arc<std::fs::File>, u64)>>,
    >,
}

impl FileRd5SegmentReader {
    pub fn new<P: AsRef<std::path::Path>>(segment_dir: P) -> Self {
        Self {
            segment_dir: segment_dir.as_ref().to_path_buf(),
            open_files: std::sync::RwLock::new(std::collections::HashMap::new()),
        }
    }

    fn get_or_open_file(&self, segment_name: &str) -> Option<(std::sync::Arc<std::fs::File>, u64)> {
        if let Ok(files) = self.open_files.read() {
            if let Some(entry) = files.get(segment_name) {
                return entry.clone();
            }
        }
        let path = self.segment_dir.join(format!("{segment_name}.rd5"));
        let opened = std::fs::File::open(path).ok().and_then(|f| {
            let len = f.metadata().ok()?.len();
            Some((std::sync::Arc::new(f), len))
        });
        if let Ok(mut files) = self.open_files.write() {
            files.insert(segment_name.to_string(), opened.clone());
        }
        opened
    }
}

impl Rd5SegmentReader for FileRd5SegmentReader {
    fn segment_len(&self, segment_name: &str) -> Option<u64> {
        self.get_or_open_file(segment_name).map(|(_, len)| len)
    }

    fn read_range(&self, segment_name: &str, start: u64, end: u64) -> io::Result<Vec<u8>> {
        if end < start {
            return Err(Error::new(ErrorKind::InvalidInput, "end < start"));
        }
        let len = (end - start) as usize;
        if len == 0 {
            return Ok(Vec::new());
        }
        let Some((file, _)) = self.get_or_open_file(segment_name) else {
            return Err(Error::new(ErrorKind::NotFound, "Segment file not found"));
        };
        let mut buf = vec![0u8; len];
        #[cfg(unix)]
        {
            use std::os::unix::fs::FileExt;
            file.read_exact_at(&mut buf, start)?;
        }
        #[cfg(not(unix))]
        {
            use std::io::{Read, Seek, SeekFrom};
            let mut f = file.as_ref();
            f.seek(SeekFrom::Start(start))?;
            f.read_exact(&mut buf)?;
        }
        Ok(buf)
    }
}

#[derive(Clone, Debug)]
struct Rd5SegmentMeta {
    file_index: [u64; 25],
    file_header_crcs: [i32; 25],
    divisor: i32,
    elevation_type: u8,
    total_len: u64,
}

impl Rd5SegmentMeta {
    #[inline]
    fn get_square_offsets(&self, tile_index: usize) -> (u64, u64) {
        let start = if tile_index > 0 {
            self.file_index[tile_index - 1]
        } else {
            200
        };
        let end = self.file_index[tile_index];
        (start, end)
    }
}

#[derive(Clone, Debug)]
struct Rd5SquareIndex {
    segment_name: String,
    square_start: u64,
    square_len: u64,
    divisor: i32,
    elevation_type: u8,
    pos_idx: Vec<u32>,
}

/// Lazy on-demand `1/80°` (`MicroCache`) cell provider backed by an `Rd5SegmentReader`.
/// Reads only the 313-byte segment header/trailer, 25.6 KB square sub-index, and individual ~10 KB cell slices.
pub struct LazyRd5Provider {
    reader: std::sync::Arc<dyn Rd5SegmentReader>,
    segments: std::sync::RwLock<std::collections::HashMap<String, Option<Rd5SegmentMeta>>>,
    squares: std::sync::RwLock<
        std::collections::HashMap<(i32, i32), Option<std::sync::Arc<Rd5SquareIndex>>>,
    >,
    missing_segments: std::sync::RwLock<std::collections::HashSet<String>>,
}

impl LazyRd5Provider {
    pub fn new(reader: std::sync::Arc<dyn Rd5SegmentReader>) -> Self {
        Self {
            reader,
            segments: std::sync::RwLock::new(std::collections::HashMap::new()),
            squares: std::sync::RwLock::new(std::collections::HashMap::new()),
            missing_segments: std::sync::RwLock::new(std::collections::HashSet::new()),
        }
    }

    /// Computes the 5°×5° segment name (e.g. `"W5_N50"`) from degree indices `(lon_degree, lat_degree)`.
    pub fn segment_name_for_degree(lon_degree: i32, lat_degree: i32) -> String {
        let lon_mod5 = lon_degree.rem_euclid(5);
        let lat_mod5 = lat_degree.rem_euclid(5);
        let lon = lon_degree - 180 - lon_mod5;
        let lat = lat_degree - 90 - lat_mod5;
        let slon = if lon < 0 {
            format!("W{}", -lon)
        } else {
            format!("E{}", lon)
        };
        let slat = if lat < 0 {
            format!("S{}", -lat)
        } else {
            format!("N{}", lat)
        };
        format!("{slon}_{slat}")
    }

    /// Drains and returns the names of any segments that were requested when unavailable in the reader.
    pub fn take_missing_segments(&self) -> Vec<String> {
        if let Ok(mut missing) = self.missing_segments.write() {
            missing.drain().collect()
        } else {
            Vec::new()
        }
    }

    /// Clears cached segment headers, square sub-indices, and missing segment records.
    pub fn clear_cache(&self) {
        if let Ok(mut segs) = self.segments.write() {
            segs.clear();
        }
        if let Ok(mut sqs) = self.squares.write() {
            sqs.clear();
        }
        if let Ok(mut missing) = self.missing_segments.write() {
            missing.clear();
        }
    }

    fn read_segment_range(&self, segment_name: &str, start: u64, end: u64) -> io::Result<Vec<u8>> {
        let expected = (end.saturating_sub(start)) as usize;
        match self.reader.read_range(segment_name, start, end) {
            Ok(buf) if buf.len() >= expected => Ok(buf),
            Ok(_) => {
                self.invalidate_segment(segment_name);
                Err(Error::new(
                    ErrorKind::UnexpectedEof,
                    format!("Short RD5 slice read from {segment_name}"),
                ))
            }
            Err(e) => {
                self.invalidate_segment(segment_name);
                Err(e)
            }
        }
    }

    fn invalidate_segment(&self, segment_name: &str) {
        if let Ok(mut segs) = self.segments.write() {
            segs.remove(segment_name);
        }
        if let Ok(mut sqs) = self.squares.write() {
            sqs.retain(|_, v| {
                v.as_ref()
                    .map(|sq| sq.segment_name != segment_name)
                    .unwrap_or(true)
            });
        }
        if let Ok(mut missing) = self.missing_segments.write() {
            missing.insert(segment_name.to_string());
        }
    }

    fn get_or_load_segment(&self, segment_name: &str) -> io::Result<Option<Rd5SegmentMeta>> {
        if let Ok(segs) = self.segments.read() {
            if let Some(cached) = segs.get(segment_name) {
                return Ok(cached.clone());
            }
        }

        let Some(total_len) = self.reader.segment_len(segment_name) else {
            if let Ok(mut missing) = self.missing_segments.write() {
                missing.insert(segment_name.to_string());
            }
            return Ok(None);
        };

        if total_len < 200 {
            if let Ok(mut segs) = self.segments.write() {
                segs.insert(segment_name.to_string(), None);
            }
            return Ok(None);
        }

        let header_buf = self.read_segment_range(segment_name, 0, 200)?;

        let file_index_crc = crc::crc(&header_buf);
        let mut dis = ByteDataReader::new(&header_buf);
        let mut file_index = [0u64; 25];
        for entry in file_index.iter_mut() {
            let lv = dis.read_long() as u64;
            *entry = lv & 0x0000_ffff_ffff_ffff;
        }

        let pos = file_index[24];
        let meta = if total_len == pos {
            Rd5SegmentMeta {
                file_index,
                file_header_crcs: [0; 25],
                divisor: 80,
                elevation_type: 3,
                total_len,
            }
        } else {
            let mut extra_len = (8 + 26 * 4) as u64;
            if total_len > pos + extra_len {
                extra_len += 1;
            }
            if total_len < pos + extra_len {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "RD5 segment too short for trailer",
                ));
            }
            let trailer_buf = self.read_segment_range(segment_name, pos, pos + extra_len)?;
            let mut tdis = ByteDataReader::new(&trailer_buf);
            let _creation_time = tdis.read_long();
            let crc_data = tdis.read_int();
            let divisor = if crc_data == file_index_crc {
                80
            } else if (crc_data ^ 2) == file_index_crc {
                32
            } else {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Top index checksum error in RD5 segment",
                ));
            };
            let mut file_header_crcs = [0i32; 25];
            for crc_entry in &mut file_header_crcs {
                *crc_entry = tdis.read_int();
            }
            let elevation_type = if tdis.has_more_data() {
                tdis.read_byte() as u8
            } else {
                3
            };
            Rd5SegmentMeta {
                file_index,
                file_header_crcs,
                divisor,
                elevation_type,
                total_len,
            }
        };

        if let Ok(mut segs) = self.segments.write() {
            segs.insert(segment_name.to_string(), Some(meta.clone()));
        }
        Ok(Some(meta))
    }

    fn get_or_load_square(
        &self,
        lon_degree: i32,
        lat_degree: i32,
    ) -> io::Result<Option<std::sync::Arc<Rd5SquareIndex>>> {
        let key = (lon_degree, lat_degree);
        if let Ok(sqs) = self.squares.read() {
            if let Some(cached) = sqs.get(&key) {
                return Ok(cached.clone());
            }
        }

        let segment_name = Self::segment_name_for_degree(lon_degree, lat_degree);
        let Some(meta) = self.get_or_load_segment(&segment_name)? else {
            return Ok(None);
        };

        let lon_mod5 = lon_degree.rem_euclid(5);
        let lat_mod5 = lat_degree.rem_euclid(5);
        let tile_index = (lon_mod5 * 5 + lat_mod5) as usize;
        let (start_offset, end_offset) = meta.get_square_offsets(tile_index);

        if start_offset >= end_offset || end_offset > meta.total_len {
            if let Ok(mut sqs) = self.squares.write() {
                sqs.insert(key, None);
            }
            return Ok(None);
        }

        let square_len = end_offset - start_offset;
        let divisor = meta.divisor;
        let ncaches = (divisor * divisor) as usize;
        let indexsize = (ncaches * 4) as u64;

        if square_len < indexsize {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Square data too short for sub-index",
            ));
        }

        let index_buf =
            self.read_segment_range(&segment_name, start_offset, start_offset + indexsize)?;
        let header_crc = crc::crc(&index_buf);
        if meta.file_header_crcs[tile_index] != header_crc {
            return Err(Error::new(
                ErrorKind::InvalidData,
                format!("Sub-index checksum mismatch for tile {tile_index}"),
            ));
        }

        let mut dis = ByteDataReader::new(&index_buf);
        let mut pos_idx = Vec::with_capacity(ncaches);
        for _ in 0..ncaches {
            pos_idx.push(dis.read_int() as u32);
        }

        let sq = std::sync::Arc::new(Rd5SquareIndex {
            segment_name,
            square_start: start_offset,
            square_len,
            divisor,
            elevation_type: meta.elevation_type,
            pos_idx,
        });

        if let Ok(mut sqs) = self.squares.write() {
            sqs.insert(key, Some(std::sync::Arc::clone(&sq)));
        }
        Ok(Some(sq))
    }
}

impl crate::graph::routing_graph::CellProvider for LazyRd5Provider {
    fn divisor_for_degree(&self, lon_degree: i32, lat_degree: i32) -> i32 {
        let segment_name = Self::segment_name_for_degree(lon_degree, lat_degree);
        if let Ok(Some(meta)) = self.get_or_load_segment(&segment_name) {
            meta.divisor
        } else {
            0
        }
    }

    fn load_cell(
        &self,
        lon_degree: i32,
        lat_degree: i32,
        lon_idx: i32,
        lat_idx: i32,
    ) -> io::Result<Option<(DecodedMicroCache, u8)>> {
        let Some(sq) = self.get_or_load_square(lon_degree, lat_degree)? else {
            return Ok(None);
        };

        let divisor = sq.divisor;
        let rel_lon = lon_idx - lon_degree * divisor;
        let rel_lat = lat_idx - lat_degree * divisor;
        if rel_lon < 0 || rel_lon >= divisor || rel_lat < 0 || rel_lat >= divisor {
            return Ok(None);
        }
        let sub_idx = (rel_lat * divisor + rel_lon) as usize;
        let ncaches = (divisor * divisor) as usize;
        if sub_idx >= ncaches {
            return Ok(None);
        }

        let indexsize = (ncaches * 4) as u64;
        let start_pos = if sub_idx == 0 {
            indexsize
        } else {
            sq.pos_idx[sub_idx - 1] as u64
        };
        let end_pos = sq.pos_idx[sub_idx] as u64;

        if start_pos >= end_pos || end_pos > sq.square_len {
            return Ok(None);
        }

        let cell_bytes = self.read_segment_range(
            &sq.segment_name,
            sq.square_start + start_pos,
            sq.square_start + end_pos,
        )?;
        if cell_bytes.is_empty() {
            return Ok(None);
        }

        let mc = DecodedMicroCache::decode(cell_bytes, lon_idx, lat_idx, divisor, None)?;
        if mc.nodes.is_empty() {
            return Ok(None);
        }
        Ok(Some((mc, sq.elevation_type)))
    }
}


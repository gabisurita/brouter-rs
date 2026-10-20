//! Rust implementation corresponding to:
//! - Java class: `btools.mapaccess.NodesCache`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/NodesCache.java`
//! - Java class: `btools.mapaccess.StorageConfigHelper`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/StorageConfigHelper.java`
//!
//! # Summary
//! Manages open `.rd5` physical files in a segment directory, computes tile coordinates,
//! and loads/decodes 1°×1° squares and MicroCache cells with LRU square and route caching.

use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::{self, Error, ErrorKind};
use std::path::{Path, PathBuf};
use std::sync::Arc;

#[cfg(feature = "rayon")]
use rayon::prelude::*;

use super::codec::tag_coder::TagValueValidator;
use super::micro_cache::DecodedMicroCache;
use super::osm_file::OsmFile;
use super::physical_file::PhysicalFile;

pub const DEFAULT_MAX_CACHED_SQUARES: usize = 64;

pub struct TileManager {
    pub segment_dir: PathBuf,
    pub files: HashMap<String, PhysicalFile>,
    pub max_cached_squares: usize,
    pub square_cache: HashMap<(i32, i32), Arc<Vec<DecodedMicroCache>>>,
    pub lru_order: VecDeque<(i32, i32)>,
}

impl TileManager {
    pub fn new<P: AsRef<Path>>(segment_dir: P) -> Self {
        Self::with_cache_capacity(segment_dir, DEFAULT_MAX_CACHED_SQUARES)
    }

    pub fn with_cache_capacity<P: AsRef<Path>>(
        segment_dir: P,
        max_cached_squares: usize,
    ) -> Self {
        Self {
            segment_dir: segment_dir.as_ref().to_path_buf(),
            files: HashMap::new(),
            max_cached_squares,
            square_cache: HashMap::new(),
            lru_order: VecDeque::new(),
        }
    }

    /// Returns the number of 1°×1° squares currently held in the LRU cache.
    pub fn cached_square_count(&self) -> usize {
        self.square_cache.len()
    }

    /// Clears the decoded 1°×1° square cache.
    pub fn clear_cache(&mut self) {
        self.square_cache.clear();
        self.lru_order.clear();
    }

    /// Computes the 5°×5° filename base (e.g., "W5_N50") from microdegree coordinates.
    pub fn get_segment_filename_base(ilon: i32, ilat: i32) -> String {
        let lon_degree = ilon / 1_000_000;
        let lat_degree = ilat / 1_000_000;

        let lon_mod5 = lon_degree % 5;
        let lat_mod5 = lat_degree % 5;

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

        format!("{}_{}", slon, slat)
    }

    /// Opens or retrieves an existing PhysicalFile for the given tile name (e.g., "W5_N50").
    pub fn get_physical_file(&mut self, filename_base: &str) -> io::Result<&mut PhysicalFile> {
        if !self.files.contains_key(filename_base) {
            let path = self.segment_dir.join(format!("{}.rd5", filename_base));
            if !path.exists() {
                return Err(Error::new(
                    ErrorKind::NotFound,
                    format!("RD5 segment file not found: {:?}", path),
                ));
            }
            let pf = PhysicalFile::open(&path)?;
            self.files.insert(filename_base.to_string(), pf);
        }

        Ok(self.files.get_mut(filename_base).unwrap())
    }

    /// Returns the elevation type (1 = 1-arcsec, 3 = 3-arcsec) for the given 1°×1° square.
    pub fn get_elevation_type(&mut self, lon_degree: i32, lat_degree: i32) -> u8 {
        let ilon = lon_degree * 1_000_000;
        let ilat = lat_degree * 1_000_000;
        let filename_base = Self::get_segment_filename_base(ilon, ilat);
        if let Ok(pf) = self.get_physical_file(&filename_base) {
            pf.elevation_type
        } else {
            3
        }
    }

    /// Loads the `OsmFile` for a specific 1°×1° square.
    pub fn load_osm_file(
        &mut self,
        lon_degree: i32,
        lat_degree: i32,
    ) -> io::Result<(OsmFile, File)> {
        let ilon = lon_degree * 1_000_000;
        let ilat = lat_degree * 1_000_000;
        let filename_base = Self::get_segment_filename_base(ilon, ilat);
        let pf = self.get_physical_file(&filename_base)?;

        let osmf = OsmFile::new(pf, lon_degree, lat_degree)?;
        let file_clone = pf.file.try_clone()?;
        Ok((osmf, file_clone))
    }

    /// Retrieves a decoded 1°×1° square from the LRU cache, or decodes it from disk on miss.
    pub fn decode_square_cached(
        &mut self,
        lon_degree: i32,
        lat_degree: i32,
    ) -> io::Result<Arc<Vec<DecodedMicroCache>>> {
        let key = (lon_degree, lat_degree);
        if let Some(cached) = self.square_cache.get(&key).cloned() {
            if let Some(pos) = self.lru_order.iter().position(|&k| k == key) {
                self.lru_order.remove(pos);
            }
            self.lru_order.push_back(key);
            return Ok(cached);
        }

        let decoded = Arc::new(self.decode_all_in_square(lon_degree, lat_degree, None)?);
        if self.max_cached_squares > 0 {
            while self.square_cache.len() >= self.max_cached_squares {
                if let Some(oldest) = self.lru_order.pop_front() {
                    self.square_cache.remove(&oldest);
                } else {
                    break;
                }
            }
            self.square_cache.insert(key, Arc::clone(&decoded));
            self.lru_order.push_back(key);
        }
        Ok(decoded)
    }

    /// Decodes all microcaches in a 1°×1° square.
    pub fn decode_all_in_square(
        &mut self,
        lon_degree: i32,
        lat_degree: i32,
        way_validator: Option<&dyn TagValueValidator>,
    ) -> io::Result<Vec<DecodedMicroCache>> {
        let (osmf, mut file) = self.load_osm_file(lon_degree, lat_degree)?;
        if !osmf.has_data {
            return Ok(Vec::new());
        }

        let divisor = osmf.divisor;
        let ncaches = (divisor * divisor) as usize;
        let mut raw_cells = Vec::with_capacity(ncaches);

        for sub_idx in 0..ncaches {
            let bytes = osmf.read_cell_bytes(&mut file, sub_idx)?;
            if !bytes.is_empty() {
                let rel_lat = (sub_idx as i32) / divisor;
                let rel_lon = (sub_idx as i32) % divisor;
                let lon_idx = divisor * lon_degree + rel_lon;
                let lat_idx = divisor * lat_degree + rel_lat;
                raw_cells.push((bytes, lon_idx, lat_idx));
            }
        }

        #[cfg(feature = "rayon")]
        let results: io::Result<Vec<DecodedMicroCache>> = raw_cells
            .into_par_iter()
            .map(|(bytes, lon_idx, lat_idx)| {
                DecodedMicroCache::decode(bytes, lon_idx, lat_idx, divisor, way_validator)
            })
            .filter_map(|res| match res {
                Ok(mc) if !mc.nodes.is_empty() => Some(Ok(mc)),
                Ok(_) => None,
                Err(e) => Some(Err(e)),
            })
            .collect();

        #[cfg(not(feature = "rayon"))]
        let results: io::Result<Vec<DecodedMicroCache>> = raw_cells
            .into_iter()
            .map(|(bytes, lon_idx, lat_idx)| {
                DecodedMicroCache::decode(bytes, lon_idx, lat_idx, divisor, way_validator)
            })
            .filter_map(|res| match res {
                Ok(mc) if !mc.nodes.is_empty() => Some(Ok(mc)),
                Ok(_) => None,
                Err(e) => Some(Err(e)),
            })
            .collect();

        results
    }
}

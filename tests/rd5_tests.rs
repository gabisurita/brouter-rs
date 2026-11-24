//! Tests for parsing and decoding real RD5 files.
//! Corresponding to official BRouter test:
//! - Java test: `btools.mapaccess.IntegrityCheckTest`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/test/java/btools/mapaccess/IntegrityCheckTest.java`

use std::path::{Path, PathBuf};
use std::time::Instant;

use brouter_rs::rd5::osm_file::OsmFile;
use brouter_rs::rd5::physical_file::PhysicalFile;
use brouter_rs::rd5::tile_manager::TileManager;

fn repo_path(rel: &str) -> PathBuf {
    let p = Path::new(rel);
    if p.exists() {
        return p.to_path_buf();
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn test_real_rd5_loading_and_decoding() {
    let rd5_path = repo_path("data/segments4/E0_N50.rd5");
    if !rd5_path.exists() {
        eprintln!("Skipping test: E0_N50.rd5 not yet downloaded");
        return;
    }

    println!("Testing real RD5 physical file header parsing...");
    let t0 = Instant::now();
    let mut pf = PhysicalFile::open(&rd5_path).expect("Failed to open E0_N50.rd5");
    println!(
        "Physical file opened in {:.2} ms: divisor={}, creation_time={}, elevation_type={}, lookup_version={}",
        t0.elapsed().as_secs_f64() * 1000.0,
        pf.divisor,
        pf.creation_time,
        pf.elevation_type,
        pf.lookup_version
    );

    assert!(pf.divisor == 80 || pf.divisor == 32);
    assert!(pf.lookup_version >= 0);

    // Count squares with data
    let mut data_squares = 0;
    for i in 0..25 {
        if pf.has_data_for_tile(i) {
            data_squares += 1;
        }
    }
    println!("Squares with data: {} / 25", data_squares);
    assert!(data_squares > 0, "No squares with data found in RD5 file");

    // Test OsmFile reading on a square with data (e.g. London East: lon 0, lat 51 -> lonDegree 180, latDegree 141)
    let lon_degree = 180;
    let lat_degree = 141;
    let lon_mod5 = lon_degree % 5;
    let lat_mod5 = lat_degree % 5;
    let tile_index = (lon_mod5 * 5 + lat_mod5) as usize;

    println!(
        "Testing 1°×1° square sub-index parsing for tile {} (lonDegree={}, latDegree={})...",
        tile_index, lon_degree, lat_degree
    );
    let osmf = OsmFile::new(&mut pf, lon_degree, lat_degree).expect("Failed to load OsmFile");
    assert!(osmf.has_data, "Expected square (180, 141) to have data");

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    println!("Decoding all MicroCache cells in square (180, 141) [London East / Kent]...");
    let t_decode = Instant::now();
    let microcaches = tile_mgr
        .decode_all_in_square(lon_degree, lat_degree, None)
        .expect("Failed to decode square");
    let elapsed = t_decode.elapsed();

    let total_nodes: usize = microcaches.iter().map(|mc| mc.nodes.len()).sum();
    let total_links: usize = microcaches
        .iter()
        .map(|mc| mc.nodes.iter().map(|n| n.links.len()).sum::<usize>())
        .sum();

    println!(
        "Decoded {} MicroCache cells with {} nodes and {} links in {:.2} ms ({:.1} nodes/sec)",
        microcaches.len(),
        total_nodes,
        total_links,
        elapsed.as_secs_f64() * 1000.0,
        total_nodes as f64 / elapsed.as_secs_f64()
    );

    assert!(
        total_nodes > 0,
        "Expected non-zero nodes in populated square"
    );
    assert!(
        total_links > 0,
        "Expected non-zero links in populated square"
    );
}

#[test]
fn test_w5_n50_tile_header_and_data() {
    let rd5_path = repo_path("data/segments4/W5_N50.rd5");
    if !rd5_path.exists() {
        println!("Skipping W5_N50 test: file not found");
        return;
    }

    let pf = PhysicalFile::open(&rd5_path).expect("Failed to open W5_N50.rd5");
    assert!(pf.divisor == 80 || pf.divisor == 32);
    let mut data_squares = 0;
    for i in 0..25 {
        if pf.has_data_for_tile(i) {
            data_squares += 1;
        }
    }
    println!(
        "W5_N50.rd5: divisor={}, data_squares={}/25",
        pf.divisor, data_squares
    );
    assert!(data_squares > 0);
}

#[test]
fn test_corrupt_microcache_and_trailerless_rd5() {
    use brouter_rs::graph::routing_graph::CellProvider;
    use brouter_rs::rd5::micro_cache::DecodedMicroCache;
    use brouter_rs::rd5::rd5_slice::{FileRd5SegmentReader, LazyRd5Provider};
    use std::fs::File;
    use std::io::Write;
    use std::sync::Arc;

    // 1. Corrupt / truncated / bad-CRC MicroCache inputs must return Err cleanly
    for bad in [
        vec![0u8; 4],
        vec![0xffu8; 16],
        vec![0u8; 32],
        vec![0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
    ] {
        assert!(
            DecodedMicroCache::decode(bad, 14400, 11280, 80, None).is_err(),
            "Corrupt microcache bytes should fail cleanly"
        );
    }

    // 2. Synthetic trailerless RD5 file: 200-byte header + 1 square sub-index + 1 empty cell
    // Tile (180, 141) -> E0_N50.rd5, tile_index = (180 % 5) * 5 + (141 % 5) = 1
    let mut temp_dir = std::env::temp_dir();
    temp_dir.push(format!("brouter_rs_trailerless_{}", std::process::id()));
    let _ = std::fs::create_dir_all(&temp_dir);
    let rd5_file = temp_dir.join("E0_N50.rd5");

    let index_bytes: i32 = 80 * 80 * 4;
    let tile1_end = 200u64 + index_bytes as u64;
    let mut data = Vec::with_capacity(tile1_end as usize);
    // Tile 0 empty (end = 200), Tiles 1..25 end at tile1_end
    data.extend_from_slice(&200u64.to_be_bytes());
    for _ in 1..25 {
        data.extend_from_slice(&tile1_end.to_be_bytes());
    }
    // Tile 1 sub-index: all cells point to end of index (0-byte cells)
    for _ in 0..(80 * 80) {
        data.extend_from_slice(&index_bytes.to_be_bytes());
    }
    assert_eq!(data.len(), tile1_end as usize);

    {
        let mut f = File::create(&rd5_file).unwrap();
        f.write_all(&data).unwrap();
    }

    let mut pf = PhysicalFile::open(&rd5_file).expect("Trailerless RD5 should open");
    assert!(!pf.has_trailer);
    let osmf = OsmFile::new(&mut pf, 180, 141).expect("Trailerless OsmFile square should load");
    assert!(osmf.has_data);

    let reader = Arc::new(FileRd5SegmentReader::new(&temp_dir));
    let provider = LazyRd5Provider::new(reader);
    assert_eq!(provider.divisor_for_degree(180, 141), 80);
    assert!(provider
        .load_cell(180, 141, 180 * 80, 141 * 80)
        .expect("Trailerless LazyRd5Provider square load should succeed")
        .is_none());

    let _ = std::fs::remove_dir_all(&temp_dir);
}

#[test]
fn test_file_rd5_segment_reader_does_not_cache_missing_file() {
    use brouter_rs::rd5::rd5_slice::{FileRd5SegmentReader, Rd5SegmentReader};
    use std::fs::File;
    use std::io::Write;

    let mut temp_dir = std::env::temp_dir();
    temp_dir.push(format!("brouter_rs_neg_cache_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&temp_dir);
    std::fs::create_dir_all(&temp_dir).unwrap();

    let reader = FileRd5SegmentReader::new(&temp_dir);
    assert_eq!(reader.segment_len("E0_N50"), None);

    let payload = b"brouter-rd5-test-payload";
    {
        let mut f = File::create(temp_dir.join("E0_N50.rd5")).unwrap();
        f.write_all(payload).unwrap();
    }

    assert_eq!(reader.segment_len("E0_N50"), Some(payload.len() as u64));
    assert_eq!(
        reader
            .read_range("E0_N50", 0, payload.len() as u64)
            .unwrap(),
        payload
    );

    let _ = std::fs::remove_dir_all(&temp_dir);
}

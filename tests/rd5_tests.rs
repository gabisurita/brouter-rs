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

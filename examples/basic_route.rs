//! Minimal example demonstrating how to compute a route using `brouter-rs`.
//!
//! Run with:
//! ```bash
//! cargo run --release --example basic_route
//! ```

use std::sync::Arc;

use brouter_rs::rd5::rd5_slice::{FileRd5SegmentReader, LazyRd5Provider};
use brouter_rs::{AStarRouter, LookupData, Profile, RouteRequest, RoutingGraph, RoutingOptions};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Load the BRouter tag lookup dictionary and routing profile (.brf)
    let lookups = LookupData::load_from_file("profiles/lookups.dat")?;
    let profile = Profile::load_from_file("profiles/fastbike-lowtraffic.brf", &lookups)?;

    // 2. Attach a lazy on-demand .rd5 segment reader pointing to data/segments4/
    let reader = Arc::new(FileRd5SegmentReader::new("data/segments4"));
    let provider = Arc::new(LazyRd5Provider::new(reader));
    let mut graph = RoutingGraph::with_cell_provider(provider);

    // 3. Define waypoints (lon, lat): Clapham -> Hackney (London)
    let waypoints = [(-0.143613, 51.462414), (-0.055296, 51.544998)];

    // 4. Compute the route using 2-pass guided A*
    let router = AStarRouter::new();
    let route = router.route(RouteRequest {
        graph: &mut graph,
        lookups: &lookups,
        profile: &profile,
        waypoints: &waypoints,
        options: RoutingOptions {
            alternative_idx: 0,
            nogos: &[],
            ..Default::default()
        },
    })?;

    println!("Route computed:");
    println!("  Distance:        {:.2} km", route.total_distance_m / 1000.0);
    println!("  Travel time:     {:.1} min", route.total_time_s / 60.0);
    println!("  Filtered ascend: {:.0} m", route.filtered_ascend_m);
    println!("  Total cost:      {}", route.total_cost);
    println!("  Points:          {}", route.points.len());
    println!("  Cells decoded:   {}", graph.loaded_cell_count());

    // Export as GeoJSON, GPX, or KML
    let _geojson = route.to_geojson("fastbike-lowtraffic");
    let _gpx = route.to_gpx("fastbike-lowtraffic");

    Ok(())
}

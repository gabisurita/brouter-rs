//! Real tests for Graph Topology, CheapRuler spatial geodesics, and Waypoint Snapping.

use std::path::{Path, PathBuf};
use std::time::Instant;

use brouter_rs::graph::matcher::{MatchedWaypoint, WaypointMatcher};
use brouter_rs::graph::node::{make_node_id, unpack_node_id, OsmNode};
use brouter_rs::graph::routing_graph::RoutingGraph;
use brouter_rs::graph::spatial::{
    calc_angle, distance, elevation_to_selev, get_difference_from_direction, get_direction,
    ilat_to_lat, ilon_to_lon, lat_to_ilat, lon_to_ilon, selev_to_elevation,
};
use brouter_rs::rd5::tile_manager::TileManager;
use brouter_rs::types::Point2D;

fn repo_path(rel: &str) -> PathBuf {
    let p = Path::new(rel);
    if p.exists() {
        return p.to_path_buf();
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn test_cheap_ruler_and_angles() {
    // 1. Coordinate conversions
    let lon = -0.1278; // London Trafalgar Square
    let lat = 51.5074;
    let ilon = lon_to_ilon(lon);
    let ilat = lat_to_ilat(lat);
    let lon_back = ilon_to_lon(ilon);
    let lat_back = ilat_to_lat(ilat);

    assert!((lon - lon_back).abs() < 1e-6, "lon conversion error");
    assert!((lat - lat_back).abs() < 1e-6, "lat conversion error");

    // 2. Elevation conversions
    let elev_m = 125.75;
    let selev = elevation_to_selev(elev_m);
    assert_eq!(selev, 503);
    let elev_back = selev_to_elevation(selev);
    assert_eq!(elev_m, elev_back);

    // 3. Known geodesic distance: London Trafalgar Square to London Bridge (~2.7 km)
    let tb_lon = -0.0877;
    let tb_lat = 51.5079;
    let tb_ilon = lon_to_ilon(tb_lon);
    let tb_ilat = lat_to_ilat(tb_lat);

    let d_m = distance(Point2D::new(ilon, ilat), Point2D::new(tb_ilon, tb_ilat));
    assert!(
        (d_m - 2770.0).abs() < 100.0,
        "Distance between Trafalgar and London Bridge should be ~2.77 km, got {:.1} m",
        d_m
    );

    // 4. Direction and angle
    // Due east: angle should be 90 degrees
    let dir_east = get_direction(Point2D::new(ilon, ilat), Point2D::new(ilon + 10000, ilat));
    assert!(
        (dir_east - 90.0).abs() < 1.0,
        "Expected east = 90 deg, got {}",
        dir_east
    );

    // Due north: angle should be 0 degrees
    let dir_north = get_direction(Point2D::new(ilon, ilat), Point2D::new(ilon, ilat + 10000));
    assert!(
        (dir_north - 0.0).abs() < 1.0,
        "Expected north = 0 deg, got {}",
        dir_north
    );

    // Turn 90 degrees right: lon0=(0, 0), lon1=(0, 100), lon2=(100, 100)
    let (angle, _cos) = calc_angle(
        Point2D::new(180_000_000, 141_000_000),
        Point2D::new(180_000_000, 141_010_000),
        Point2D::new(180_010_000, 141_010_000),
    );
    assert!(
        (angle - 90.0).abs() < 2.0,
        "Expected right turn ~90 degrees, got {}",
        angle
    );

    // Difference between 350 deg and 10 deg should be 20 deg
    let diff = get_difference_from_direction(350.0, 10.0);
    assert_eq!(diff, 20.0);

    println!("CheapRuler and angle tests passed successfully.");
}

#[test]
fn test_node_id_packing() {
    let ilon = 179_872_200;
    let ilat = 141_507_400;
    let id = make_node_id(ilon, ilat);
    let pos_back = unpack_node_id(id);
    assert_eq!(ilon, pos_back.lon);
    assert_eq!(ilat, pos_back.lat);

    let node = OsmNode::new(ilon, ilat);
    assert_eq!(node.id(), id);
}

#[test]
fn test_synthetic_waypoint_projection() {
    let mut mwp = MatchedWaypoint::new("test_pt", Point2D::new(180_000_000, 141_000_500), 200.0);

    // Check segment along horizontal latitude from lon=179_999_000 to lon=180_001_000 at lat=141_000_000
    WaypointMatcher::check_segment(
        &mut mwp,
        Point2D::new(179_999_000, 141_000_000),
        Point2D::new(180_001_000, 141_000_000),
        Point2D::new(179_999_000, 141_000_000),
        Point2D::new(180_001_000, 141_000_000),
    );

    assert!(
        mwp.radius < 100.0,
        "Radius should be ~55 meters, got {:.2}",
        mwp.radius
    );
    assert_eq!(mwp.crosspoint.lon, 180_000_000);
    assert_eq!(mwp.crosspoint.lat, 141_000_000);

    println!(
        "Synthetic projection test passed: snapped to ({}, {}) with distance {:.2}m",
        mwp.crosspoint.lon, mwp.crosspoint.lat, mwp.radius
    );
}

#[test]
fn test_real_rd5_graph_loading_and_snapping() {
    let segment_path = repo_path("data/segments4/E0_N50.rd5");
    if !segment_path.exists() {
        println!("Skipping real RD5 test: {:?} does not exist", segment_path);
        return;
    }

    println!(
        "Loading cells from real RD5 tile (180, 141) [London East / Kent] into RoutingGraph..."
    );
    let t0 = Instant::now();

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(180, 141, None)
        .expect("Failed to decode square (180, 141)");

    let mut graph = RoutingGraph::with_capacities(50_000, 100_000);
    let cells_loaded = microcaches.len();
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }

    let elapsed = t0.elapsed();
    println!(
        "Loaded {} MicroCache cells into graph in {:.2} ms (Nodes: {}, Edges: {})",
        cells_loaded,
        elapsed.as_secs_f64() * 1000.0,
        graph.node_count(),
        graph.edge_count()
    );

    assert!(graph.node_count() > 1000, "Graph should contain nodes");
    assert!(graph.edge_count() > 1000, "Graph should contain edges");

    // Pick a real node from the graph and test snapping near it
    let (_sample_node_id, sample_node) = graph.nodes.iter().next().unwrap();
    let sample_lon_deg = ilon_to_lon(sample_node.ilon);
    let sample_lat_deg = ilat_to_lat(sample_node.ilat);

    // Offset test point slightly (e.g. ~15 meters away)
    let query_lon = sample_lon_deg + 0.0001;
    let query_lat = sample_lat_deg + 0.0001;

    let lookups = brouter_rs::dsl::lookups::LookupData::load_from_file(&repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups");
    let profile = brouter_rs::dsl::parser::Profile::load_from_file(&repo_path("profiles/trekking.brf"), &lookups)
        .expect("Failed to load trekking profile");
    let mut evaluator =
        brouter_rs::dsl::evaluator::ProfileEvaluator::new(&profile, &lookups.way, &lookups.node);

    let snap_res = graph.snap_waypoint(
        "query_point",
        query_lon,
        query_lat,
        200.0,
        &mut evaluator,
        &lookups,
    );
    assert!(
        snap_res.is_some(),
        "Should find a snap candidate near real graph node"
    );

    let matched = snap_res.unwrap();
    println!(
        "Successfully snapped ({:.6}, {:.6}) -> crosspoint ({:.6}, {:.6}) at distance {:.2}m",
        query_lon,
        query_lat,
        ilon_to_lon(matched.crosspoint.lon),
        ilat_to_lat(matched.crosspoint.lat),
        matched.radius
    );
    assert!(matched.radius <= 200.0);

    // Verify profile access filtering:
    // A profile with costfactor >= 9999 should reject all edges and return None.
    let forbidden_profile = brouter_rs::dsl::parser::Profile::parse(
        "forbidden",
        "---context:global\n---context:way\nassign costfactor = 9999.0\n---context:node\n",
        &lookups,
    )
    .expect("Failed to parse forbidden profile");
    let mut forbidden_evaluator = brouter_rs::dsl::evaluator::ProfileEvaluator::new(
        &forbidden_profile,
        &lookups.way,
        &lookups.node,
    );
    let snap_forbidden = graph.snap_waypoint(
        "query_point",
        query_lon,
        query_lat,
        200.0,
        &mut forbidden_evaluator,
        &lookups,
    );
    assert!(
        snap_forbidden.is_none(),
        "Forbidden road (costfactor >= 9999) must not be snapped to"
    );

    // An allowed profile with costfactor < 9999 should successfully snap.
    let allowed_profile = brouter_rs::dsl::parser::Profile::parse(
        "allowed",
        "---context:global\n---context:way\nassign costfactor = 1.5\n---context:node\n",
        &lookups,
    )
    .expect("Failed to parse allowed profile");
    let mut allowed_evaluator = brouter_rs::dsl::evaluator::ProfileEvaluator::new(
        &allowed_profile,
        &lookups.way,
        &lookups.node,
    );
    let snap_allowed = graph.snap_waypoint(
        "query_point",
        query_lon,
        query_lat,
        200.0,
        &mut allowed_evaluator,
        &lookups,
    );
    assert!(
        snap_allowed.is_some(),
        "Allowed road (costfactor < 9999) should be snapped to"
    );
}

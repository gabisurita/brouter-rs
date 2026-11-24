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

    let lookups =
        brouter_rs::dsl::lookups::LookupData::load_from_file(repo_path("profiles/lookups.dat"))
            .expect("Failed to load lookups");
    let profile = brouter_rs::dsl::parser::Profile::load_from_file(
        repo_path("profiles/trekking.brf"),
        &lookups,
    )
    .expect("Failed to load trekking profile");
    let mut evaluator =
        brouter_rs::dsl::evaluator::ProfileEvaluator::new(&profile, &lookups.way, &lookups.node);

    let snap_req = brouter_rs::SnapRequest::new("query_point", query_lon, query_lat, 200.0);
    let snap_res = graph.snap(snap_req, &mut evaluator, &lookups);
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
    let snap_forbidden = graph.snap(snap_req, &mut forbidden_evaluator, &lookups);
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
    let snap_allowed = graph.snap(snap_req, &mut allowed_evaluator, &lookups);
    assert!(
        snap_allowed.is_some(),
        "Allowed road (costfactor < 9999) should be snapped to"
    );
}

#[test]
fn test_self_loop_ensure_node_finalized_no_panic() {
    use brouter_rs::graph::routing_graph::CellProvider;
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{
        DecodedGeometryPoint, DecodedLink, DecodedMicroCache, DecodedNode,
    };
    use std::sync::Arc;

    struct SelfLoopProvider {
        cache: DecodedMicroCache,
    }

    impl CellProvider for SelfLoopProvider {
        fn divisor_for_degree(&self, _lon_degree: i32, _lat_degree: i32) -> i32 {
            80
        }

        fn load_cell(
            &self,
            _lon_degree: i32,
            _lat_degree: i32,
            lon_idx: i32,
            lat_idx: i32,
        ) -> std::io::Result<Option<(DecodedMicroCache, u8)>> {
            if lon_idx == self.cache.lon_idx && lat_idx == self.cache.lat_idx {
                Ok(Some((self.cache.clone(), 3)))
            } else {
                Ok(None)
            }
        }
    }

    let ilon: i32 = 180_005_000;
    let ilat: i32 = 141_005_000;
    let lon_idx = ilon.div_euclid(12_500);
    let lat_idx = ilat.div_euclid(12_500);

    let cache = DecodedMicroCache {
        lon_idx,
        lat_idx,
        nodes: vec![DecodedNode {
            ilon,
            ilat,
            selev: 100,
            node_tags: None,
            turn_restrictions: Vec::new(),
            links: vec![DecodedLink {
                target_lon: ilon,
                target_lat: ilat,
                is_reverse: false,
                way_tags: Some(TagValueWrapper {
                    data: vec![1, 2],
                    access_type: 2,
                }),
                geometry: vec![
                    DecodedGeometryPoint {
                        dlon: 500,
                        dlat: 0,
                        delev: 0,
                    },
                    DecodedGeometryPoint {
                        dlon: 0,
                        dlat: 500,
                        delev: 0,
                    },
                ],
            }],
        }],
    };

    let mut graph = RoutingGraph::with_cell_provider(Arc::new(SelfLoopProvider { cache }));
    let s_id = make_node_id(ilon, ilat);
    let (start, count) = graph.ensure_node_finalized(s_id);
    assert_eq!(count, 2);

    let edges = graph.get_edges_by_slice(start, count);
    assert_eq!(edges.len(), 2);
    for edge in edges {
        assert_eq!(edge.target_id, s_id);
        assert_eq!(edge.target_edges_start, start);
        assert_eq!(edge.target_edges_count, count);
        assert_ne!(edge.rev_edge_idx, u32::MAX);
    }
    assert_eq!(edges[0].rev_edge_idx, edges[1].edge_idx);
    assert_eq!(edges[1].rev_edge_idx, edges[0].edge_idx);
}

#[test]
fn test_lazy_snap_after_cross_border_finalization() {
    use brouter_rs::dsl::evaluator::ProfileEvaluator;
    use brouter_rs::dsl::lookups::LookupData;
    use brouter_rs::dsl::parser::Profile;
    use brouter_rs::graph::routing_graph::CellProvider;
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{DecodedLink, DecodedMicroCache, DecodedNode};
    use std::collections::HashMap;
    use std::sync::Arc;

    struct TwoRegionProvider {
        cells: HashMap<(i32, i32), DecodedMicroCache>,
    }

    impl CellProvider for TwoRegionProvider {
        fn divisor_for_degree(&self, _lon_degree: i32, _lat_degree: i32) -> i32 {
            80
        }

        fn load_cell(
            &self,
            _lon_degree: i32,
            _lat_degree: i32,
            lon_idx: i32,
            lat_idx: i32,
        ) -> std::io::Result<Option<(DecodedMicroCache, u8)>> {
            Ok(self.cells.get(&(lon_idx, lat_idx)).cloned().map(|c| (c, 3)))
        }
    }

    let way_tags = || {
        Some(TagValueWrapper {
            data: vec![1],
            access_type: 2,
        })
    };
    let lat: i32 = 141_005_000;
    let lat_idx = lat.div_euclid(12_500);

    // Cell C1 around 180_005_000 with a 6-node connected component and a long edge to C5 (180_055_000)
    let c1_lons = [
        180_005_000,
        180_005_500,
        180_006_000,
        180_006_500,
        180_007_000,
        180_007_500,
    ];
    let c5_far_lon = 180_055_000;
    let mut c1_nodes = Vec::new();
    for (i, &lon) in c1_lons.iter().enumerate() {
        let mut links = Vec::new();
        if i + 1 < c1_lons.len() {
            links.push(DecodedLink {
                target_lon: c1_lons[i + 1],
                target_lat: lat,
                is_reverse: false,
                way_tags: way_tags(),
                geometry: Vec::new(),
            });
        }
        if i == 0 {
            links.push(DecodedLink {
                target_lon: c5_far_lon,
                target_lat: lat,
                is_reverse: false,
                way_tags: way_tags(),
                geometry: Vec::new(),
            });
        }
        c1_nodes.push(DecodedNode {
            ilon: lon,
            ilat: lat,
            selev: 100,
            node_tags: None,
            turn_restrictions: Vec::new(),
            links,
        });
    }

    // Cell C5 around 180_055_000 containing target road r0..r5 (at lat + 2000 so separate from c5_far)
    let c5_road_lat: i32 = 141_007_000;
    let c5_lons = [
        180_054_000,
        180_054_500,
        180_055_000,
        180_055_500,
        180_056_000,
        180_056_500,
    ];
    let mut c5_nodes = vec![DecodedNode {
        ilon: c5_far_lon,
        ilat: lat,
        selev: 100,
        node_tags: None,
        turn_restrictions: Vec::new(),
        links: Vec::new(),
    }];
    for i in 0..c5_lons.len() - 1 {
        c5_nodes.push(DecodedNode {
            ilon: c5_lons[i],
            ilat: c5_road_lat,
            selev: 100,
            node_tags: None,
            turn_restrictions: Vec::new(),
            links: vec![DecodedLink {
                target_lon: c5_lons[i + 1],
                target_lat: c5_road_lat,
                is_reverse: false,
                way_tags: way_tags(),
                geometry: Vec::new(),
            }],
        });
    }
    c5_nodes.push(DecodedNode {
        ilon: *c5_lons.last().unwrap(),
        ilat: c5_road_lat,
        selev: 100,
        node_tags: None,
        turn_restrictions: Vec::new(),
        links: Vec::new(),
    });

    let c1_idx = (c1_lons[0].div_euclid(12_500), lat_idx);
    let c5_idx = (c5_far_lon.div_euclid(12_500), lat_idx);
    let mut cells = HashMap::new();
    cells.insert(
        c1_idx,
        DecodedMicroCache {
            lon_idx: c1_idx.0,
            lat_idx: c1_idx.1,
            nodes: c1_nodes,
        },
    );
    cells.insert(
        c5_idx,
        DecodedMicroCache {
            lon_idx: c5_idx.0,
            lat_idx: c5_idx.1,
            nodes: c5_nodes,
        },
    );

    let lookups = LookupData::parse("---context:way\n---context:node\n").unwrap();
    let profile = Profile::parse(
        "test",
        "---context:global\n---context:way\nassign costfactor = 1.0\n---context:node\n",
        &lookups,
    )
    .unwrap();
    let mut evaluator = ProfileEvaluator::new(&profile, &lookups.way, &lookups.node);

    let mut graph = RoutingGraph::with_cell_provider(Arc::new(TwoRegionProvider { cells }));

    // 1. Snap in C1; finalizing node c1_lons[0] loads C5 with track_spatial_grid = false
    let mwp1 = graph.snap_lazy(
        brouter_rs::SnapRequest::new("from", ilon_to_lon(180_005_250), ilat_to_lat(lat), 50.0),
        &mut evaluator,
        &lookups,
    );
    assert!(mwp1.is_some(), "Start snap in C1 should succeed");

    // 2. Snap in C5; spatial_grid must rebuild so unfinalized road in C5 is found
    let mwp2 = graph.snap_lazy(
        brouter_rs::SnapRequest::new(
            "to",
            ilon_to_lon(180_055_250),
            ilat_to_lat(c5_road_lat),
            50.0,
        ),
        &mut evaluator,
        &lookups,
    );
    let mwp2 = mwp2.expect("Target snap in C5 must succeed after C5 was loaded during C1 finalize");
    assert!(
        mwp2.radius < 5.0,
        "Expected snap within 5m on C5 road, got {:.2}m",
        mwp2.radius
    );
}

#[test]
fn test_compact_live_nodes_preserves_shared_geometry() {
    use brouter_rs::graph::fast_map::FastHashSet;
    use brouter_rs::graph::routing_graph::CellProvider;
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{
        DecodedGeometryPoint, DecodedLink, DecodedMicroCache, DecodedNode,
    };
    use std::sync::Arc;

    struct TwoNodeProvider {
        cache: DecodedMicroCache,
    }

    impl CellProvider for TwoNodeProvider {
        fn divisor_for_degree(&self, _lon_degree: i32, _lat_degree: i32) -> i32 {
            80
        }

        fn load_cell(
            &self,
            _lon_degree: i32,
            _lat_degree: i32,
            lon_idx: i32,
            lat_idx: i32,
        ) -> std::io::Result<Option<(DecodedMicroCache, u8)>> {
            if lon_idx == self.cache.lon_idx && lat_idx == self.cache.lat_idx {
                Ok(Some((self.cache.clone(), 3)))
            } else {
                Ok(None)
            }
        }
    }

    let ilon1: i32 = 180_001_000;
    let ilat1: i32 = 141_001_000;
    let ilon2: i32 = 180_006_000;
    let ilat2: i32 = 141_001_000;
    let lon_idx = ilon1.div_euclid(12_500);
    let lat_idx = ilat1.div_euclid(12_500);

    let geom: Vec<DecodedGeometryPoint> = (1..=5)
        .map(|i| DecodedGeometryPoint {
            dlon: 800,
            dlat: if i % 2 == 0 { 100 } else { -100 },
            delev: 0,
        })
        .collect();

    let cache = DecodedMicroCache {
        lon_idx,
        lat_idx,
        nodes: vec![
            DecodedNode {
                ilon: ilon1,
                ilat: ilat1,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: ilon2,
                    target_lat: ilat2,
                    is_reverse: false,
                    way_tags: Some(TagValueWrapper {
                        data: vec![1],
                        access_type: 2,
                    }),
                    geometry: geom,
                }],
            },
            DecodedNode {
                ilon: ilon2,
                ilat: ilat2,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: Vec::new(),
            },
        ],
    };

    let mut graph = RoutingGraph::with_cell_provider(Arc::new(TwoNodeProvider { cache }));
    let id1 = make_node_id(ilon1, ilat1);
    let id2 = make_node_id(ilon2, ilat2);
    graph.ensure_node_finalized(id1);
    graph.ensure_node_finalized(id2);
    assert_eq!(graph.geometry_pool.len(), 5);

    let mut keep_nodes = FastHashSet::default();
    keep_nodes.insert(id1);
    keep_nodes.insert(id2);
    let mut frontier_cells = FastHashSet::default();
    frontier_cells.insert((lon_idx, lat_idx));

    graph.compact_live_nodes(&mut keep_nodes, &frontier_cells);
    assert_eq!(
        graph.geometry_pool.len(),
        5,
        "Bidirectional edges must continue sharing geometry_pool slice across compaction"
    );
}

#[test]
fn test_edge_count_mixed_and_is_connected_above_12() {
    use brouter_rs::dsl::evaluator::ProfileEvaluator;
    use brouter_rs::dsl::lookups::LookupData;
    use brouter_rs::dsl::parser::Profile;
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{DecodedLink, DecodedMicroCache, DecodedNode};

    let lookups = LookupData::parse("---context:way\n---context:node\n").unwrap();
    let profile = Profile::parse(
        "connected_test",
        "---context:global\n---context:way\nassign costfactor = 1.0\n---context:node\n",
        &lookups,
    )
    .unwrap();
    let mut evaluator = ProfileEvaluator::new(&profile, &lookups.way, &lookups.node);
    let mut way_buf = vec![0i32; lookups.way.names.len()];

    let way_tags = || {
        Some(TagValueWrapper {
            data: vec![1],
            access_type: 2,
        })
    };

    let ilat: i32 = 141_000_000;
    let lons: Vec<i32> = (0..16).map(|i| 180_000_000 + i * 500).collect();
    let mut nodes = Vec::with_capacity(lons.len());
    for (i, &lon) in lons.iter().enumerate() {
        let mut links = Vec::new();
        if i + 1 < lons.len() {
            links.push(DecodedLink {
                target_lon: lons[i + 1],
                target_lat: ilat,
                is_reverse: false,
                way_tags: way_tags(),
                geometry: Vec::new(),
            });
        }
        nodes.push(DecodedNode {
            ilon: lon,
            ilat,
            selev: 100,
            node_tags: None,
            turn_restrictions: Vec::new(),
            links,
        });
    }

    let mut graph = RoutingGraph::new();
    graph.add_micro_cache(DecodedMicroCache {
        lon_idx: 180_000_000i32.div_euclid(12_500),
        lat_idx: ilat.div_euclid(12_500),
        nodes,
    });

    // 15 forward + 15 reverse = 30 edges initially in `graph.edges`
    assert_eq!(graph.edge_count(), 30);

    // Finalize only the first node (moves its 1 outgoing edge to `flat_edges`, leaving 29 in `edges`)
    let n0 = make_node_id(lons[0], ilat);
    let n1 = make_node_id(lons[1], ilat);
    graph.ensure_node_finalized(n0);
    assert_eq!(graph.edge_count(), 30);

    // Test min_nodes <= 1 and min_nodes > 12 (no panic, exact connectivity)
    assert!(graph.is_connected_passable_edge(n0, n1, 1, &mut evaluator, &lookups, &mut way_buf));
    assert!(graph.is_connected_passable_edge(n0, n1, 15, &mut evaluator, &lookups, &mut way_buf));
    assert!(!graph.is_connected_passable_edge(n0, n1, 20, &mut evaluator, &lookups, &mut way_buf));
}

//! Real tests for kinematics, elevation hysteresis, and full route search.

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use brouter_rs::dsl::evaluator::WayCostResult;
use brouter_rs::dsl::lookups::LookupData;
use brouter_rs::dsl::parser::Profile;
use brouter_rs::graph::routing_graph::RoutingGraph;
use brouter_rs::graph::spatial::{distance, lat_to_ilat, lon_to_ilon, Point2D};
use brouter_rs::rd5::tile_manager::TileManager;
use brouter_rs::routing::cost::KinematicConfig;
use brouter_rs::routing::nogo::NogoArea;
use brouter_rs::routing::path::{ElevationConfig, PathState, SectionInput, StdPathCalculator};
use brouter_rs::routing::search::{AStarRouter, RouteRequest, RoutingOptions};

fn repo_path(rel: &str) -> PathBuf {
    let p = Path::new(rel);
    if p.exists() {
        return p.to_path_buf();
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn test_cubic_speed_solver_and_kinematics() {
    let cfg = KinematicConfig::default();

    // 1. Test cubic solver: 0.225 * v^3 + 8.829 * v = 100
    // At v = 7.024: 0.225 * 346.4 + 8.829 * 7.024 = 77.94 + 62.01 = ~140, wait:
    // Let's test residual directly:
    let a = cfg.s_c_x;
    let f_roll = cfg.total_mass * 9.81 * cfg.c_r; // 90 * 9.81 * 0.01 = 8.829
    let power = cfg.biker_power; // 100 W

    let v = KinematicConfig::solve_cubic(a, f_roll, power);
    let power_calc = (a * v * v + f_roll) * v;
    assert!(
        (power_calc - power).abs() < 0.2,
        "Cubic solver residual too large: computed power={:.2}, expected={:.2}",
        power_calc,
        power
    );
    // Typical cycling speed on flat with 100W is ~25 km/h = ~7 m/s
    let speed_kmh = v * 3.6;
    assert!(
        (20.0..=25.0).contains(&speed_kmh),
        "Expected cycling speed ~21.4 km/h, got {:.1} km/h",
        speed_kmh
    );

    // 2. Test downhill speed: negative incline reduces f_roll, increasing speed
    let (speed_flat, energy_flat) = cfg.compute_speed_and_energy(100.0, 0.0, 0.0);
    let (speed_down, energy_down) = cfg.compute_speed_and_energy(100.0, -0.02, 0.0);
    assert!(
        speed_down > speed_flat,
        "Downhill speed should exceed flat speed"
    );
    assert!(
        energy_down <= energy_flat,
        "Downhill energy should be lower than flat"
    );

    println!(
        "Kinematic test passed: flat speed={:.1} km/h, downhill speed={:.1} km/h",
        speed_flat * 3.6,
        speed_down * 3.6
    );
}

#[test]
fn test_elevation_hysteresis_and_cutoffs() {
    let mut state = PathState::default();
    let elev_cfg = ElevationConfig::default();
    let kinematic_cfg = KinematicConfig::default();

    let mut way_cost = WayCostResult {
        costfactor: 1.25,
        turncost: 90.0,
        uphillcostfactor: 1.25,
        downhillcostfactor: 1.25,
        initialcost: 0.0,
        nodeaccessgranted: true,
        initialclassifier: 0,
        trafficsourcedensity: 0.0,
        istrafficbackbone: false,
        priorityclassifier: 0,
        classifiermask: 0,
        maxspeed: 50.0,
        uphillcost: 60.0,
        downhillcost: 60.0,
        uphillcutoff: 1.5,   // 1.5% cutoff
        downhillcutoff: 1.5, // 1.5% cutoff
        uphillmaxslope: 15.0,
        downhillmaxslope: 15.0,
        uphillmaxslopecost: 0.0,
        downhillmaxslopecost: 0.0,
        ..Default::default()
    };
    way_cost.init_derived();

    // Sub-cutoff slope: 100m distance with 1.0m elevation rise = 1.0% slope (< 1.5% cutoff)
    // Should NOT incur any elevation penalty
    let sec1 = StdPathCalculator::process_section(
        &mut state,
        &way_cost,
        &elev_cfg,
        &kinematic_cfg,
        &SectionInput {
            dist: 100.0,
            delta_h_micros: 1_000_000,
            cos_angle: 1.0,      // straight turn
            crossing_penalty: 0, // no crossing penalty
            is_detail: true,
        },
    );
    assert_eq!(
        sec1.elevation_cost, 0.0,
        "Sub-cutoff slope should have 0 elevation cost"
    );

    // Steeper slope: 1000m distance with 80m rise = 8.0% slope
    // Should fill buffer and incur elevation cost
    let sec2 = StdPathCalculator::process_section(
        &mut state,
        &way_cost,
        &elev_cfg,
        &kinematic_cfg,
        &SectionInput {
            dist: 1000.0,
            delta_h_micros: 80_000_000,
            cos_angle: 1.0,
            crossing_penalty: 0,
            is_detail: true,
        },
    );
    assert!(
        sec2.elevation_cost > 0.0,
        "Steep slope should incur elevation cost"
    );
    assert!(
        sec2.section_cost > 1000.0 * 1.25,
        "Total cost should exceed baseline distance cost"
    );

    println!(
        "Elevation hysteresis test passed: sub-cutoff cost={:.1}, steep slope cost={:.1}",
        sec1.section_cost, sec2.section_cost
    );
}

#[test]
fn test_real_full_route_generation() {
    let segment_path = repo_path("data/segments4/E0_N50.rd5");
    if !segment_path.exists() {
        println!(
            "Skipping real route test: {:?} does not exist",
            segment_path
        );
        return;
    }

    println!("Loading Lookups and Fastbike Profile...");
    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike profile");

    println!("Loading square (180, 141) into RoutingGraph...");
    let t0 = Instant::now();
    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(180, 141, None)
        .expect("Failed to decode square (180, 141)");

    let mut graph = RoutingGraph::with_capacities(50_000, 100_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();
    println!(
        "Graph ready in {:.2} ms (Nodes: {}, Edges: {})",
        t0.elapsed().as_secs_f64() * 1000.0,
        graph.node_count(),
        graph.edge_count()
    );

    // Route coordinates in East London (Woolwich / Greenwich East to Barking)
    // Start: Woolwich / Thamesmead: lon = 0.07, lat = 51.50
    // Target: Barking / East Ham: lon = 0.08, lat = 51.53
    let start_lon = 0.07;
    let start_lat = 51.50;
    let target_lon = 0.08;
    let target_lat = 51.53;

    let router = AStarRouter::new();
    println!("Computing full route using 2-pass guided A*...");
    let t_route = Instant::now();
    let route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[(start_lon, start_lat), (target_lon, target_lat)],
            options: RoutingOptions::default(),
        })
        .expect("Route search failed");
    let route_elapsed = t_route.elapsed();

    println!("--- Route Summary ---");
    println!("Total distance: {:.1} meters", route.total_distance_m);
    println!(
        "Total time:     {:.0} seconds ({:.1} min)",
        route.total_time_s,
        route.total_time_s / 60.0
    );
    println!("Total energy:   {:.0} Joules", route.total_energy_j);
    println!("Filtered ascend:{:.1} meters", route.filtered_ascend_m);
    println!("Total cost:     {}", route.total_cost);
    println!("Route points:   {}", route.points.len());
    println!(
        "Search time:    {:.2} ms",
        route_elapsed.as_secs_f64() * 1000.0
    );

    assert!(route.total_distance_m > 3000.0 && route.total_distance_m < 15000.0);
    assert!(route.points.len() > 50);
    assert!(route.total_cost > 0);
    assert!(route.total_time_s > 0.0);

    // Verify GeoJSON format
    let geojson = route.to_geojson("fastbike");
    assert!(geojson.contains("\"FeatureCollection\""));
    assert!(geojson.contains("\"LineString\""));
    assert!(geojson.contains("\"track-length\""));

    // Verify GPX format
    let gpx = route.to_gpx("fastbike");
    assert!(gpx.contains("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
    assert!(gpx.contains("<gpx version=\"1.1\" creator=\"brouter-rs\""));
    assert!(gpx.contains("<name>fastbike</name>"));
    assert!(gpx.contains("<trk>"));
    assert!(gpx.contains("<trkseg>"));
    assert!(gpx.contains("<trkpt lat="));
    assert!(gpx.contains("<ele>"));

    // Save route artifacts to target dir for verification and inspection
    let _ = fs::create_dir_all("target/routes");
    let _ = fs::write("target/routes/east_london_fastbike.geojson", &geojson);
    let _ = fs::write("target/routes/east_london_fastbike.gpx", &gpx);
    println!("Saved route to target/routes/east_london_fastbike.geojson and .gpx");
}

#[test]
fn test_route_result_to_gpx() {
    use brouter_rs::routing::search::{RoutePoint, RouteResult};

    let points = vec![
        RoutePoint {
            lon: -0.143613,
            lat: 51.462414,
            elevation_m: 24.25,
            distance_from_start_m: 0.0,
            time_from_start_s: 0.0,
            energy_j: 0.0,
            cost: 0,
        },
        RoutePoint {
            lon: -0.142000,
            lat: 51.463000,
            elevation_m: 25.50,
            distance_from_start_m: 120.0,
            time_from_start_s: 18.0,
            energy_j: 1800.0,
            cost: 150,
        },
    ];

    let route = RouteResult {
        total_distance_m: 120.0,
        total_time_s: 18.0,
        total_energy_j: 1800.0,
        total_cost: 150,
        filtered_ascend_m: 1.25,
        plain_ascend_m: 1.25,
        points,
        ..Default::default()
    };

    let gpx = route.to_gpx("test & track");
    assert!(gpx.contains("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
    assert!(gpx.contains("<gpx version=\"1.1\" creator=\"brouter-rs\""));
    assert!(gpx.contains("<name>test &amp; track</name>"));
    assert!(gpx.contains("<trkpt lat=\"51.462414\" lon=\"-0.143613\">"));
    assert!(gpx.contains("<ele>24.25</ele>"));
    assert!(gpx.contains("<trkpt lat=\"51.463000\" lon=\"-0.142000\">"));
    assert!(gpx.contains("<ele>25.50</ele>"));
    assert!(gpx.contains("</trkseg>"));
    assert!(gpx.contains("</trk>"));
    assert!(gpx.contains("</gpx>"));
}

#[test]
fn test_clapham_hackney_fastbike() {
    let segment_path = repo_path("data/segments4/W5_N50.rd5");
    if !segment_path.exists() {
        println!(
            "Skipping clapham_hackney test: {:?} does not exist",
            segment_path
        );
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike profile");

    println!("Loading square (179, 141) from W5_N50.rd5 into RoutingGraph...");
    let t0 = Instant::now();
    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");

    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();
    println!(
        "Graph ready in {:.2} ms (Nodes: {}, Edges: {})",
        t0.elapsed().as_secs_f64() * 1000.0,
        graph.node_count(),
        graph.edge_count()
    );

    // Clapham -> Hackney coordinates
    let start_lon = -0.143613;
    let start_lat = 51.462414;
    let target_lon = -0.055296;
    let target_lat = 51.544998;

    let router = AStarRouter::new();
    println!("Routing Clapham -> Hackney with fastbike...");
    let t_route = Instant::now();
    let route = router
        .query(
            &graph,
            &lookups,
            &profile,
            &[(start_lon, start_lat), (target_lon, target_lat)],
        )
        .compute()
        .expect("Clapham -> Hackney routing failed");
    let route_elapsed = t_route.elapsed();

    println!("--- Clapham -> Hackney (fastbike) Summary ---");
    println!(
        "Total distance: {:.1} m (BRouter baseline: 13200 m)",
        route.total_distance_m
    );
    println!(
        "Total time:     {:.0} s (BRouter baseline: 2248 s)",
        route.total_time_s
    );
    println!(
        "Total energy:   {:.0} J (BRouter baseline: 224871 J)",
        route.total_energy_j
    );
    println!(
        "Filtered ascend:{:.1} m (BRouter baseline: 12 m)",
        route.filtered_ascend_m
    );
    println!(
        "Total cost:     {} (BRouter baseline: 21851)",
        route.total_cost
    );
    println!("Route points:   {}", route.points.len());
    println!(
        "Search time:    {:.2} ms",
        route_elapsed.as_secs_f64() * 1000.0
    );

    let _ = fs::create_dir_all("target/routes");
    let geojson = route.to_geojson("clapham_hackney_fastbike");
    let _ = fs::write("target/routes/clapham_hackney_fastbike.geojson", &geojson);

    // Enforce exact parity against official BRouter baseline
    let dist_diff = (route.total_distance_m - 13174.0).abs();
    assert!(
        dist_diff <= 1.0,
        "Distance divergence > 1m: got {}",
        route.total_distance_m
    );
    assert_eq!(
        route.total_cost, 21871,
        "Cost divergence from BRouter baseline"
    );
    assert_eq!(
        route.points.len(),
        918,
        "Point count divergence from BRouter baseline"
    );
}

#[test]
fn test_clapham_hackney_lowtraffic() {
    let segment_path = repo_path("data/segments4/W5_N50.rd5");
    if !segment_path.exists() {
        println!("Skipping test: {:?} does not exist", segment_path);
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike-lowtraffic.brf"), &lookups)
        .expect("Failed to load fastbike-lowtraffic profile");

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");

    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();

    let start_lon = -0.143613;
    let start_lat = 51.462414;
    let target_lon = -0.055296;
    let target_lat = 51.544998;

    let router = AStarRouter::new();
    println!("Routing Clapham -> Hackney with fastbike-lowtraffic...");
    let t_route = Instant::now();
    let route = router
        .query(
            &graph,
            &lookups,
            &profile,
            &[(start_lon, start_lat), (target_lon, target_lat)],
        )
        .compute()
        .expect("Clapham -> Hackney lowtraffic routing failed");
    let route_elapsed = t_route.elapsed();

    println!("--- Clapham -> Hackney (fastbike-lowtraffic) Summary ---");
    println!(
        "Total distance: {:.1} m (BRouter baseline: 14291 m)",
        route.total_distance_m
    );
    println!(
        "Total time:     {:.0} s (BRouter baseline: 2431 s)",
        route.total_time_s
    );
    println!(
        "Total energy:   {:.0} J (BRouter baseline: 243111 J)",
        route.total_energy_j
    );
    println!(
        "Filtered ascend:{:.1} m (BRouter baseline: 12 m)",
        route.filtered_ascend_m
    );
    println!(
        "Total cost:     {} (BRouter baseline: 24745)",
        route.total_cost
    );
    println!("Route points:   {}", route.points.len());
    println!(
        "Search time:    {:.2} ms",
        route_elapsed.as_secs_f64() * 1000.0
    );

    let _ = fs::create_dir_all("target/routes");
    let geojson = route.to_geojson("clapham_hackney_fastbike-lowtraffic");
    let _ = fs::write(
        "target/routes/clapham_hackney_fastbike-lowtraffic.geojson",
        &geojson,
    );

    // Enforce exact parity against official BRouter baseline
    let dist_diff = (route.total_distance_m - 14021.0).abs();
    assert!(
        dist_diff <= 1.0,
        "Distance divergence > 1m: got {}",
        route.total_distance_m
    );
    assert_eq!(
        route.total_cost, 24692,
        "Cost divergence from BRouter baseline"
    );
    assert_eq!(
        route.points.len(),
        911,
        "Point count divergence from BRouter baseline"
    );
}

#[test]
fn test_clapham_hackney_verylowtraffic() {
    let segment_path = repo_path("data/segments4/W5_N50.rd5");
    if !segment_path.exists() {
        println!("Skipping test: {:?} does not exist", segment_path);
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile =
        Profile::load_from_file(repo_path("profiles/fastbike-verylowtraffic.brf"), &lookups)
            .expect("Failed to load fastbike-verylowtraffic profile");

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");

    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();

    let start_lon = -0.143613;
    let start_lat = 51.462414;
    let target_lon = -0.055296;
    let target_lat = 51.544998;

    let router = AStarRouter::new();
    println!("Routing Clapham -> Hackney with fastbike-verylowtraffic...");
    let t_route = Instant::now();
    let route = router
        .query(
            &graph,
            &lookups,
            &profile,
            &[(start_lon, start_lat), (target_lon, target_lat)],
        )
        .compute()
        .expect("Clapham -> Hackney verylowtraffic routing failed");
    let route_elapsed = t_route.elapsed();

    println!("--- Clapham -> Hackney (fastbike-verylowtraffic) Summary ---");
    println!(
        "Total distance: {:.1} m (BRouter baseline: 14254 m)",
        route.total_distance_m
    );
    println!(
        "Total time:     {:.0} s (BRouter baseline: 2423 s)",
        route.total_time_s
    );
    println!(
        "Total energy:   {:.0} J (BRouter baseline: 242346 J)",
        route.total_energy_j
    );
    println!(
        "Filtered ascend:{:.1} m (BRouter baseline: 12 m)",
        route.filtered_ascend_m
    );
    println!(
        "Total cost:     {} (BRouter baseline: 25141)",
        route.total_cost
    );
    println!("Route points:   {}", route.points.len());
    println!(
        "Search time:    {:.2} ms",
        route_elapsed.as_secs_f64() * 1000.0
    );

    let _ = fs::create_dir_all("target/routes");
    let geojson = route.to_geojson("clapham_hackney_fastbike-verylowtraffic");
    let _ = fs::write(
        "target/routes/clapham_hackney_fastbike-verylowtraffic.geojson",
        &geojson,
    );

    // Enforce exact parity against official BRouter baseline
    let dist_diff = (route.total_distance_m - 14394.0).abs();
    assert!(
        dist_diff <= 1.0,
        "Distance divergence > 1m: got {}",
        route.total_distance_m
    );
    assert_eq!(
        route.total_cost, 25097,
        "Cost divergence from BRouter baseline"
    );
    assert_eq!(
        route.points.len(),
        913,
        "Point count divergence from BRouter baseline"
    );
}

#[test]
fn test_profile_configs_initialization() {
    let profile_content = r#"
---context:global
assign totalMass = 85.0
assign maxSpeed = 36.0
assign S_C_x = 0.21
assign C_r = 0.008
assign bikerPower = 150.0
assign validForFoot = 0
assign elevationpenaltybuffer = 4.0
assign elevationmaxbuffer = 8.0
assign elevationbufferreduce = 0.0001
"#;
    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile =
        Profile::parse("test_profile", profile_content, &lookups).expect("Profile parse failed");

    let k_cfg = KinematicConfig::from_profile(&profile);
    assert!((k_cfg.total_mass - 85.0).abs() < 1e-5);
    assert!((k_cfg.max_speed - 10.0).abs() < 1e-5);
    assert!((k_cfg.s_c_x - 0.21).abs() < 1e-5);
    assert!((k_cfg.c_r - 0.008).abs() < 1e-5);
    assert!((k_cfg.biker_power - 150.0).abs() < 1e-5);
    assert!(!k_cfg.is_foot_mode);

    let e_cfg = ElevationConfig::from_profile(&profile);
    assert_eq!(e_cfg.penalty_buffer, 4_000_000);
    assert_eq!(e_cfg.max_buffer, 8_000_000);
    assert_eq!(e_cfg.buffer_reduce, 1);
}

#[test]
fn test_route_with_nogo_zone() {
    let segment_path = repo_path("data/segments4/W5_N50.rd5");
    if !segment_path.exists() {
        println!(
            "Skipping test_route_with_nogo_zone: {:?} does not exist",
            segment_path
        );
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike profile");

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");

    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();

    let start = (-0.143613, 51.462414);
    let target = (-0.055296, 51.544998);
    let router = AStarRouter::new();

    // 1. Baseline route
    let base_route = router
        .query(&graph, &lookups, &profile, &[start, target])
        .compute()
        .expect("Baseline route failed");

    // 2. Place a 30m hard nogo directly on a midpoint along the baseline route
    let mid_idx = base_route.points.len() / 2;
    let mid_pt = &base_route.points[mid_idx];
    let nogo_center = Point2D::new(lon_to_ilon(mid_pt.lon), lat_to_ilat(mid_pt.lat));
    let nogo = NogoArea::new(mid_pt.lon, mid_pt.lat, 30.0);

    // 3. Compute detour route with nogo
    let detour_route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: RoutingOptions {
                nogos: &[nogo],
                ..Default::default()
            },
        })
        .expect("Detour route failed");

    // 4. Verify detour route avoids the nogo circle
    for pt in &detour_route.points {
        let pt_2d = Point2D::new(lon_to_ilon(pt.lon), lat_to_ilat(pt.lat));
        let dist_to_center = distance(pt_2d, nogo_center);
        assert!(
            dist_to_center >= 29.0, // slight tolerance for discretisation
            "Detour route came too close to nogo center: {:.1}m < 30m",
            dist_to_center
        );
    }

    // 5. Verify the detour route changed
    assert_ne!(base_route.total_cost, detour_route.total_cost);
    println!(
        "Nogo test passed: baseline cost {}, detour cost {}, baseline pts {}, detour pts {}",
        base_route.total_cost,
        detour_route.total_cost,
        base_route.points.len(),
        detour_route.points.len()
    );
}

#[test]
fn test_car_kinematic_and_foot_modes() {
    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");

    // 1. Verify foot mode Tobler's hiking speed is not clamped by way maxspeed
    let hiking = Profile::load_from_file(repo_path("profiles/hiking-mountain.brf"), &lookups)
        .expect("Failed to load hiking-mountain.brf");
    let hiking_k = KinematicConfig::from_profile(&hiking);
    assert!(hiking_k.is_foot_mode);
    let (foot_speed, foot_energy) = hiking_k.compute_speed_and_energy(100.0, 0.0, 0.5);
    assert!(
        foot_speed > 1.0,
        "Foot mode speed should follow Tobler's formula, got {}",
        foot_speed
    );
    assert!(foot_energy > 0.0);

    // 2. Verify non-bike/non-foot StdModel (car-fast) uses profile maxSpeed
    let car_fast = Profile::load_from_file(repo_path("profiles/car-fast.brf"), &lookups)
        .expect("Failed to load car-fast.brf");
    let car_fast_k = KinematicConfig::from_profile(&car_fast);
    assert!(!car_fast_k.is_foot_mode);
    assert!(!car_fast_k.is_bike_mode);
    let (car_fast_speed, _) = car_fast_k.compute_speed_and_energy(100.0, 0.0, 45.0 / 3.6);
    assert!((car_fast_speed - car_fast_k.max_speed).abs() < 1e-6);

    // 3. Verify car-eco KinematicModel routing on square (180, 141) if data exists
    let segment_path = repo_path("data/segments4/E0_N50.rd5");
    if !segment_path.exists() {
        return;
    }
    let car_eco = Profile::load_from_file(repo_path("profiles/car-eco.brf"), &lookups)
        .expect("Failed to load car-eco.brf");
    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(180, 141, None)
        .expect("Failed to decode square (180, 141)");
    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();

    let router = AStarRouter::new();
    let route = router
        .query(&graph, &lookups, &car_eco, &[(0.07, 51.50), (0.08, 51.53)])
        .compute()
        .expect("car-eco routing failed");
    assert!(route.total_distance_m > 1000.0);
    assert!(route.total_time_s > 0.0);
    assert!(route.total_energy_j > 0.0);
    assert!(route.total_cost > 0);
}

#[test]
fn test_voicehints_messages_times_and_kml() {
    let segment_path = repo_path("data/segments4/W5_N50.rd5");
    if !segment_path.exists() {
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike profile");

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");

    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();

    let start = (-0.143613, 51.462414);
    let target = (-0.055296, 51.544998);
    let router = AStarRouter::new();
    let options = brouter_rs::routing::search::RoutingOptions {
        turn_instruction_mode: Some(4),
        ..Default::default()
    };

    let route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options,
        })
        .expect("Routing failed");

    assert!(
        !route.voice_hints.is_empty(),
        "Expected non-empty voice_hints for urban route"
    );
    assert!(
        !route.messages.is_empty(),
        "Expected non-empty messages for urban route"
    );

    let geojson_str = route.to_geojson("fastbike");
    let json: serde_json::Value = serde_json::from_str(&geojson_str).expect("Invalid GeoJSON");
    let props = &json["features"][0]["properties"];

    let voicehints = props["voicehints"]
        .as_array()
        .expect("Missing voicehints array in GeoJSON");
    assert_eq!(voicehints.len(), route.voice_hints.len());
    for vh in voicehints {
        let arr = vh.as_array().expect("voicehint item should be an array");
        assert_eq!(arr.len(), 6);
    }

    let messages = props["messages"]
        .as_array()
        .expect("Missing messages matrix in GeoJSON");
    assert!(messages.len() > 1, "messages should have header + rows");
    let header = messages[0].as_array().unwrap();
    assert_eq!(header.len(), 13);
    assert_eq!(header[0].as_str().unwrap(), "Longitude");
    assert_eq!(header[11].as_str().unwrap(), "Time");
    assert_eq!(header[12].as_str().unwrap(), "Energy");

    let times = props["times"]
        .as_array()
        .expect("Missing times array in GeoJSON");
    assert_eq!(times.len(), route.points.len());

    let kml = route.to_kml("fastbike");
    assert!(kml.contains("<kml xmlns=\"http://earth.google.com/kml/2.0\">"));
    assert!(kml.contains("<LineString>"));
    assert!(kml.contains("<Folder>"));
    assert!(kml.contains("<name>Paths</name>"));

    let gpx = route.to_gpx("fastbike");
    assert!(gpx.contains("<trkpt "));
    assert!(gpx.contains("<sym>"));
}

#[test]
fn test_heading_beeline_and_nogo_polygons() {
    let segment_path = repo_path("data/segments4/W5_N50.rd5");
    if !segment_path.exists() {
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike profile");

    let mut tile_mgr = TileManager::new(repo_path("data/segments4"));
    let microcaches = tile_mgr
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");

    let mut graph = RoutingGraph::with_capacities(100_000, 250_000);
    for cache in microcaches {
        graph.add_micro_cache(cache);
    }
    graph.resolve_edge_target_elevations();

    let start = (-0.143613, 51.462414);
    let target = (-0.135000, 51.468000);
    let router = AStarRouter::new();

    // 1. Direct beeline leg
    let beeline_opts = brouter_rs::routing::search::RoutingOptions {
        direct_legs: &[true],
        ..Default::default()
    };
    let beeline_route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: beeline_opts,
        })
        .expect("Beeline route failed");
    assert_eq!(beeline_route.points.len(), 2);
    assert_eq!(
        beeline_route.total_cost,
        beeline_route.total_distance_m as u32
    );
    assert!(beeline_route.total_distance_m > 500.0);

    // 2. Departure heading penalty
    let north_opts = brouter_rs::routing::search::RoutingOptions {
        start_direction: Some(45.0),
        force_start_direction: true,
        ..Default::default()
    };
    let south_opts = brouter_rs::routing::search::RoutingOptions {
        start_direction: Some(225.0),
        force_start_direction: true,
        ..Default::default()
    };
    let north_route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: north_opts,
        })
        .expect("North heading route failed");
    let south_route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: south_opts,
        })
        .expect("South heading route failed");
    assert_ne!(north_route.total_cost, south_route.total_cost);

    // 3. Nogo polyline across the midpoint of north_route
    let mid = &north_route.points[north_route.points.len() / 2];
    let polyline_str = format!(
        "{:.6},{:.6},{:.6},{:.6}",
        mid.lon - 0.002,
        mid.lat - 0.002,
        mid.lon + 0.002,
        mid.lat + 0.002
    );
    let nogo_polys = brouter_rs::routing::parse_nogo_polygons(&polyline_str, false);
    assert_eq!(nogo_polys.len(), 1);
    let poly_opts = brouter_rs::routing::search::RoutingOptions {
        nogos: &nogo_polys,
        ..Default::default()
    };
    let detour = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: poly_opts,
        })
        .expect("Polyline detour route failed");
    assert_ne!(detour.total_cost, north_route.total_cost);
}

#[test]
fn test_lazy_rd5_on_demand_cell_loading_parity() {
    use brouter_rs::rd5::rd5_slice::{FileRd5SegmentReader, LazyRd5Provider};
    use std::sync::Arc;

    let segments_dir = repo_path("data/segments4");
    if !segments_dir.join("W5_N50.rd5").exists() {
        return;
    }

    let lookups = LookupData::load_from_file(repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike profile");

    let start = (-0.143613, 51.462414);
    let via = (-0.148000, 51.472000);
    let target = (-0.125000, 51.482000);

    // 1. Preloaded reference graph
    let mut tm = TileManager::new(&segments_dir);
    let microcaches = tm
        .decode_all_in_square(179, 141, None)
        .expect("Failed to decode square (179, 141)");
    let mut ref_graph = RoutingGraph::with_capacities(100_000, 250_000);
    for mc in microcaches {
        ref_graph.add_micro_cache(mc);
    }
    ref_graph.resolve_edge_target_elevations();

    let router = AStarRouter::new();
    let ref_route_1 = router
        .query(&ref_graph, &lookups, &profile, &[start, target])
        .compute()
        .expect("Reference route 1 failed");
    let ref_route_2 = router
        .query(&ref_graph, &lookups, &profile, &[start, via, target])
        .compute()
        .expect("Reference route 2 failed");

    // 2. Lazy on-demand graph starting with 0 loaded cells
    let reader = Arc::new(FileRd5SegmentReader::new(&segments_dir));
    let provider = Arc::new(LazyRd5Provider::new(reader));
    let mut lazy_graph = RoutingGraph::with_cell_provider(provider);
    assert_eq!(lazy_graph.loaded_cell_count(), 0);

    let t0 = Instant::now();
    let lazy_route_1 = router
        .route(RouteRequest {
            graph: &mut lazy_graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: RoutingOptions::default(),
        })
        .expect("Lazy route 1 failed");
    let t1 = t0.elapsed();
    let cells_after_1 = lazy_graph.loaded_cell_count();

    let t2 = Instant::now();
    let lazy_route_2 = router
        .route(RouteRequest {
            graph: &mut lazy_graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, via, target],
            options: RoutingOptions::default(),
        })
        .expect("Lazy route 2 failed");
    let t3 = t2.elapsed();
    let cells_after_2 = lazy_graph.loaded_cell_count();

    println!(
        "Lazy route 1: {:?}, cells loaded: {} (nodes: {}, edges: {})",
        t1,
        cells_after_1,
        lazy_graph.node_count(),
        lazy_graph.edge_count()
    );
    println!(
        "Lazy route 2: {:?}, total cells loaded: {} (+{} incremental)",
        t3,
        cells_after_2,
        cells_after_2 - cells_after_1
    );

    assert_eq!(lazy_route_1.total_cost, ref_route_1.total_cost);
    assert!((lazy_route_1.total_distance_m - ref_route_1.total_distance_m).abs() < 1e-6);
    assert_eq!(lazy_route_1.points.len(), ref_route_1.points.len());
    assert_eq!(
        lazy_route_1.voice_hints.len(),
        ref_route_1.voice_hints.len()
    );

    assert_eq!(lazy_route_2.total_cost, ref_route_2.total_cost);
    assert!((lazy_route_2.total_distance_m - ref_route_2.total_distance_m).abs() < 1e-6);
    assert_eq!(lazy_route_2.points.len(), ref_route_2.points.len());
    assert_eq!(
        lazy_route_2.voice_hints.len(),
        ref_route_2.voice_hints.len()
    );

    assert!(
        cells_after_1 > 0 && cells_after_1 < 60,
        "Expected < 60 cells, got {}",
        cells_after_1
    );
    assert!(
        cells_after_2 >= cells_after_1 && cells_after_2 < 80,
        "Expected < 80 cells, got {}",
        cells_after_2
    );

    let safe_profile =
        Profile::load_from_file(repo_path("profiles/fastbike-lowtraffic.brf"), &lookups)
            .expect("Failed to load fastbike-lowtraffic profile");
    let waypoints_11 = [
        (-0.1421, 51.4611),
        (-0.1401, 51.4651),
        (-0.1371, 51.4691),
        (-0.1321, 51.4721),
        (-0.1291, 51.4771),
        (-0.1261, 51.4811),
        (-0.1221, 51.4881),
        (-0.1181, 51.4951),
        (-0.0881, 51.5141),
        (-0.0791, 51.5401),
        (-0.0541, 51.5431),
    ];
    let ref_route_11 = router
        .query(&ref_graph, &lookups, &safe_profile, &waypoints_11)
        .compute()
        .expect("Reference 11-waypoint route failed");

    let reader_11 = Arc::new(FileRd5SegmentReader::new(&segments_dir));
    let provider_11 = Arc::new(LazyRd5Provider::new(reader_11));
    let mut lazy_graph_11 = RoutingGraph::with_cell_provider(provider_11);
    let t_11_start = Instant::now();
    let lazy_route_11 = router
        .route(RouteRequest {
            graph: &mut lazy_graph_11,
            lookups: &lookups,
            profile: &safe_profile,
            waypoints: &waypoints_11,
            options: RoutingOptions::default(),
        })
        .expect("Lazy 11-waypoint route failed");
    println!(
        "Lazy 11-waypoint route: {:?}, cells loaded: {} (cost: {}, dist: {}m)",
        t_11_start.elapsed(),
        lazy_graph_11.loaded_cell_count(),
        lazy_route_11.total_cost,
        lazy_route_11.total_distance_m
    );

    assert_eq!(lazy_route_11.total_cost, ref_route_11.total_cost);
    assert!((lazy_route_11.total_distance_m - ref_route_11.total_distance_m).abs() < 1e-6);
    assert_eq!(lazy_route_11.points.len(), ref_route_11.points.len());

    let mut waypoints_11_moved = waypoints_11;
    waypoints_11_moved[4] = (-0.1340, 51.4790);
    let ref_route_11_moved = router
        .query(&ref_graph, &lookups, &safe_profile, &waypoints_11_moved)
        .compute()
        .expect("Reference moved 11-waypoint route failed");
    let t_11_warm = Instant::now();
    let lazy_route_11_moved = router
        .route(RouteRequest {
            graph: &mut lazy_graph_11,
            lookups: &lookups,
            profile: &safe_profile,
            waypoints: &waypoints_11_moved,
            options: RoutingOptions::default(),
        })
        .expect("Lazy warm 11-waypoint route failed");
    println!(
        "Lazy 11-waypoint warm edit: {:?}, cells loaded: {} (cost: {}, dist: {}m)",
        t_11_warm.elapsed(),
        lazy_graph_11.loaded_cell_count(),
        lazy_route_11_moved.total_cost,
        lazy_route_11_moved.total_distance_m
    );
    assert_eq!(
        lazy_route_11_moved.total_cost,
        ref_route_11_moved.total_cost
    );
    assert!(
        (lazy_route_11_moved.total_distance_m - ref_route_11_moved.total_distance_m).abs() < 1e-6
    );
    assert_eq!(
        lazy_route_11_moved.points.len(),
        ref_route_11_moved.points.len()
    );
}

#[test]
fn test_oneway_same_street_loop_around() {
    use brouter_rs::graph::spatial::{ilat_to_lat, ilon_to_lon};
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{DecodedLink, DecodedMicroCache, DecodedNode};

    let lookups = LookupData::parse("---context:way\n---context:node\n").unwrap();
    let profile = Profile::parse(
        "oneway_loop",
        "---context:global\n\
         ---context:way\n\
         assign costfactor = if reversedirection=yes then 10000 else 1.0\n\
         ---context:node\n",
        &lookups,
    )
    .unwrap();

    let way_tags = || {
        Some(TagValueWrapper {
            data: vec![1],
            access_type: 2,
        })
    };

    // 6-node one-way loop: A -> B -> C -> D -> E -> F -> A
    let coords = [
        (180_000_000, 141_000_000), // A
        (180_004_000, 141_000_000), // B
        (180_004_000, 141_002_000), // C
        (180_002_000, 141_002_000), // D
        (180_000_000, 141_002_000), // E
        (180_000_000, 141_001_000), // F
    ];

    let mut nodes = Vec::new();
    for i in 0..coords.len() {
        let next = coords[(i + 1) % coords.len()];
        nodes.push(DecodedNode {
            ilon: coords[i].0,
            ilat: coords[i].1,
            selev: 100,
            node_tags: None,
            turn_restrictions: Vec::new(),
            links: vec![DecodedLink {
                target_lon: next.0,
                target_lat: next.1,
                is_reverse: false,
                way_tags: way_tags(),
                geometry: Vec::new(),
            }],
        });
    }

    let mut graph = RoutingGraph::new();
    graph.add_micro_cache(DecodedMicroCache {
        lon_idx: 180_000_000i32.div_euclid(12_500),
        lat_idx: 141_000_000i32.div_euclid(12_500),
        nodes,
    });
    graph.resolve_edge_target_elevations();

    // Start at 75% along A -> B, target at 25% along A -> B
    let start = (ilon_to_lon(180_003_000), ilat_to_lat(141_000_000));
    let target = (ilon_to_lon(180_001_000), ilat_to_lat(141_000_000));

    let router = AStarRouter::new();
    let route = router
        .query(&graph, &lookups, &profile, &[start, target])
        .compute()
        .expect("One-way same-street loop-around must succeed");

    // Expected path: start -> B -> C -> D -> E -> F -> A -> target (8 points)
    assert_eq!(route.points.len(), 8, "Expected 8-point loop around block");
    assert!(route.total_distance_m > 500.0);
}

#[test]
fn test_target_link_fallthrough_and_same_segment_direction() {
    use brouter_rs::graph::spatial::{ilat_to_lat, ilon_to_lon};
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{
        DecodedGeometryPoint, DecodedLink, DecodedMicroCache, DecodedNode,
    };

    let lookups = LookupData::parse("---context:way\n---context:node\n").unwrap();
    let profile = Profile::parse(
        "target_fallthrough",
        "---context:global\n\
         ---context:way\n\
         assign costfactor = 1.0\n\
         ---context:node\n",
        &lookups,
    )
    .unwrap();

    let way_tags = || {
        Some(TagValueWrapper {
            data: vec![1],
            access_type: 2,
        })
    };

    // Build connected graph of >= 6 nodes: S0->S1->S2->S3->A, and two parallel edges A<->B:
    // E1 (north arc via M_north) and E2 (south arc via M_south).
    // Hard nogo sits on chord M_north <-> M_south (clear of both arcs, blocking sliced chord).
    let lat_base: i32 = 141_000_000;
    let a_lon: i32 = 180_004_000;
    let b_lon: i32 = 180_008_000;
    let mid_lon: i32 = 180_006_000;
    let m_north_lat: i32 = lat_base + 2_000;
    let m_south_lat: i32 = lat_base - 2_000;

    let cache = DecodedMicroCache {
        lon_idx: 180_000_000i32.div_euclid(12_500),
        lat_idx: lat_base.div_euclid(12_500),
        nodes: vec![
            DecodedNode {
                ilon: 180_000_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_001_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_001_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_002_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_002_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_003_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_003_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: a_lon,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: a_lon,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    // E1: north arc A -> M_north -> B
                    DecodedLink {
                        target_lon: b_lon,
                        target_lat: lat_base,
                        is_reverse: false,
                        way_tags: way_tags(),
                        geometry: vec![DecodedGeometryPoint {
                            dlon: mid_lon - a_lon,
                            dlat: m_north_lat - lat_base,
                            delev: 0,
                        }],
                    },
                    // E2: south arc A -> M_south -> B
                    DecodedLink {
                        target_lon: b_lon,
                        target_lat: lat_base,
                        is_reverse: false,
                        way_tags: way_tags(),
                        geometry: vec![DecodedGeometryPoint {
                            dlon: mid_lon - a_lon,
                            dlat: m_south_lat - lat_base,
                            delev: 0,
                        }],
                    },
                ],
            },
            DecodedNode {
                ilon: b_lon,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: Vec::new(),
            },
        ],
    };

    let mut graph = RoutingGraph::new();
    graph.add_micro_cache(cache);
    graph.resolve_edge_target_elevations();

    let start = (ilon_to_lon(180_000_500), ilat_to_lat(lat_base));
    // Target is right on M_south on E2
    let target = (ilon_to_lon(mid_lon), ilat_to_lat(m_south_lat));
    // Nogo blocks A -> M_south (so E2 from A is blocked) AND blocks chord M_north -> M_south (so E1 sliced is blocked),
    // while E1 full path A -> M_north -> B and B -> M_south remain clear!
    let nogo_block_a_south = NogoArea::new(
        ilon_to_lon((a_lon + mid_lon) / 2),
        ilat_to_lat((lat_base + m_south_lat) / 2),
        50.0,
    );
    let nogo_chord = NogoArea::new(ilon_to_lon(mid_lon), ilat_to_lat(lat_base + 500), 60.0);

    let router = AStarRouter::new();
    let route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: RoutingOptions {
                nogos: &[nogo_block_a_south, nogo_chord],
                ..Default::default()
            },
        })
        .expect("Route must fall through blocked is_target_link onto E1 -> B -> M_south");
    assert!(route.total_cost > 0);

    // Also test same_segment start_direction / force_start_direction penalty
    let same_seg_start = (ilon_to_lon(180_001_200), ilat_to_lat(lat_base));
    let same_seg_target = (ilon_to_lon(180_001_800), ilat_to_lat(lat_base));
    let east_dir = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[same_seg_start, same_seg_target],
            options: RoutingOptions {
                start_direction: Some(90.0),
                force_start_direction: true,
                ..Default::default()
            },
        })
        .unwrap();
    let west_dir = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[same_seg_start, same_seg_target],
            options: RoutingOptions {
                start_direction: Some(270.0),
                force_start_direction: true,
                ..Default::default()
            },
        })
        .unwrap();
    assert!(
        west_dir.total_cost >= east_dir.total_cost + 10_000,
        "same_segment must apply force_start_direction penalty (east={}, west={})",
        east_dir.total_cost,
        west_dir.total_cost
    );
}

#[test]
fn test_soft_nogo_large_weight_saturates_without_overflow() {
    use brouter_rs::graph::spatial::{ilat_to_lat, ilon_to_lon};
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{
        DecodedGeometryPoint, DecodedLink, DecodedMicroCache, DecodedNode,
    };

    let lookups = LookupData::parse("---context:way\n---context:node\n").unwrap();
    let profile = Profile::parse(
        "nogo_overflow",
        "---context:global\n\
         ---context:way\n\
         assign costfactor = 1.0\n\
         ---context:node\n",
        &lookups,
    )
    .unwrap();

    let way_tags = || {
        Some(TagValueWrapper {
            data: vec![1],
            access_type: 2,
        })
    };

    let lat_base: i32 = 141_000_000;
    let cache = DecodedMicroCache {
        lon_idx: 180_000_000i32.div_euclid(12_500),
        lat_idx: lat_base.div_euclid(12_500),
        nodes: vec![
            DecodedNode {
                ilon: 180_000_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_002_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_002_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_004_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    // Direct penalized edge with soft nogo
                    DecodedLink {
                        target_lon: 180_008_000,
                        target_lat: lat_base,
                        is_reverse: false,
                        way_tags: way_tags(),
                        geometry: Vec::new(),
                    },
                    // Clean parallel bypass going north
                    DecodedLink {
                        target_lon: 180_008_000,
                        target_lat: lat_base,
                        is_reverse: false,
                        way_tags: way_tags(),
                        geometry: vec![DecodedGeometryPoint {
                            dlon: 2_000,
                            dlat: 2_000,
                            delev: 0,
                        }],
                    },
                ],
            },
            DecodedNode {
                ilon: 180_008_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_010_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_010_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_012_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: way_tags(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_012_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: Vec::new(),
            },
        ],
    };

    let mut graph = RoutingGraph::new();
    graph.add_micro_cache(cache);
    graph.resolve_edge_target_elevations();

    let start = (ilon_to_lon(180_000_500), ilat_to_lat(lat_base));
    let target = (ilon_to_lon(180_011_500), ilat_to_lat(lat_base));
    let soft_nogo =
        NogoArea::with_weight(ilon_to_lon(180_006_000), ilat_to_lat(lat_base), 50.0, 1e9);

    let router = AStarRouter::new();
    let route = router
        .route(RouteRequest {
            graph: &graph,
            lookups: &lookups,
            profile: &profile,
            waypoints: &[start, target],
            options: RoutingOptions {
                nogos: &[soft_nogo],
                ..Default::default()
            },
        })
        .expect("Route should bypass saturated soft nogo without overflow");

    assert!(
        route
            .points
            .iter()
            .any(|p| p.lat > ilat_to_lat(lat_base) + 0.001),
        "Route must choose clean north bypass rather than overflowing soft nogo cost"
    );
    assert!(route.total_cost < 10_000);
}

#[test]
fn test_retrack_message_costs_and_parallel_edge_lookup() {
    use brouter_rs::graph::spatial::{ilat_to_lat, ilon_to_lon};
    use brouter_rs::rd5::codec::bit_coder::StatCoderContext;
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{
        DecodedGeometryPoint, DecodedLink, DecodedMicroCache, DecodedNode,
    };

    let lookups = LookupData::parse(
        "---context:way\n\
         highway residential\n\
         highway cycleway\n\
         highway track\n\
         ---context:node\n\
         barrier gate\n",
    )
    .unwrap();

    let profile = Profile::parse(
        "retrack_costs",
        "---context:global\n\
         assign elevationpenaltybuffer = 5\n\
         assign elevationmaxbuffer = 10\n\
         assign elevationbufferreduce = 0\n\
         ---context:way\n\
         assign turncost = 100\n\
         assign uphillcost = 100\n\
         assign uphillcutoff = 0\n\
         assign initialclassifier = switch highway=cycleway 2 1\n\
         assign initialcost = switch highway=cycleway 250 0\n\
         assign costfactor = switch highway=track 100 1.0\n\
         ---context:node\n\
         assign initialcost = switch barrier=gate 120 0\n",
        &lookups,
    )
    .unwrap();

    let encode_single_tag = |dd_val: i32, access_type: i32| -> TagValueWrapper {
        let mut buf = vec![0u8; 8];
        let len = {
            let mut ctx = StatCoderContext::new(&mut buf);
            ctx.bit_coder.encode_var_bits(1); // delta = 1 (first tag after hardwired idx 0)
            ctx.bit_coder.encode_var_bits(dd_val);
            ctx.bit_coder.encode_var_bits(0); // terminator
            ctx.bit_coder.close_and_get_encoded_length()
        };
        buf.truncate(len);
        TagValueWrapper {
            data: buf,
            access_type,
        }
    };

    let tag_residential = || Some(encode_single_tag(0, 2)); // dd=0 -> idx 2 (residential)
    let tag_cycleway = || Some(encode_single_tag(1, 2)); // dd=1 -> idx 3 (cycleway)
    let tag_track = || Some(encode_single_tag(2, 2)); // dd=2 -> idx 4 (track)
    let node_gate = || Some(encode_single_tag(0, 0)); // dd=0 -> idx 2 (gate)

    // Build 6-node chain: S0 -> A -> B =(parallel track|cycleway)=> C(gate) -> D(climb) -> E
    let lat_base: i32 = 141_000_000;
    let cache = DecodedMicroCache {
        lon_idx: 180_000_000i32.div_euclid(12_500),
        lat_idx: lat_base.div_euclid(12_500),
        nodes: vec![
            DecodedNode {
                ilon: 180_000_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_002_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: tag_residential(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_002_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_004_000,
                    target_lat: lat_base,
                    is_reverse: false,
                    way_tags: tag_residential(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base,
                selev: 100,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    // Expensive direct edge (edges[0])
                    DecodedLink {
                        target_lon: 180_004_000,
                        target_lat: lat_base + 4_000,
                        is_reverse: false,
                        way_tags: tag_track(),
                        geometry: Vec::new(),
                    },
                    // Cheap parallel curved cycleway (edges[1], detours east at midpoint)
                    DecodedLink {
                        target_lon: 180_004_000,
                        target_lat: lat_base + 4_000,
                        is_reverse: false,
                        way_tags: tag_cycleway(),
                        geometry: vec![DecodedGeometryPoint {
                            dlon: 1_000,
                            dlat: 2_000,
                            delev: 0,
                        }],
                    },
                ],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base + 4_000,
                selev: 100,
                node_tags: node_gate(),
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_008_000,
                    target_lat: lat_base + 4_000,
                    is_reverse: false,
                    way_tags: tag_residential(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_008_000,
                ilat: lat_base + 4_000,
                selev: 500, // +100m climb
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![DecodedLink {
                    target_lon: 180_010_000,
                    target_lat: lat_base + 4_000,
                    is_reverse: false,
                    way_tags: tag_residential(),
                    geometry: Vec::new(),
                }],
            },
            DecodedNode {
                ilon: 180_010_000,
                ilat: lat_base + 4_000,
                selev: 500,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: Vec::new(),
            },
        ],
    };

    let mut graph = RoutingGraph::new();
    graph.add_micro_cache(cache);
    graph.resolve_edge_target_elevations();

    let start = (ilon_to_lon(180_000_500), ilat_to_lat(lat_base));
    let target = (ilon_to_lon(180_009_500), ilat_to_lat(lat_base + 4_000));
    let router = AStarRouter::new();
    let route = router
        .query(&graph, &lookups, &profile, &[start, target])
        .compute()
        .expect("Route must succeed");

    let msgs: Vec<_> = route.messages.iter().flatten().collect();
    // Parallel edge 1 (cycleway with midpoint dlat=2_000) must be selected over edge 0 (track)
    assert!(
        msgs.iter()
            .any(|m| m.way_key_values.contains("highway=cycleway")),
        "expected cycleway message, got: {msgs:?}"
    );
    assert!(
        route
            .points
            .iter()
            .any(|p| p.lon > ilon_to_lon(180_004_500) && p.lat < ilat_to_lat(lat_base + 4_000)),
        "expected cycleway east-detour intermediate geometry point"
    );

    let max_turn: i32 = msgs.iter().map(|m| m.linkturncost).max().unwrap_or(0);
    let total_init: i32 = msgs.iter().map(|m| m.linkinitcost).sum();
    let total_node: i32 = msgs.iter().map(|m| m.linknodecost).sum();
    let total_elev: i32 = msgs.iter().map(|m| m.linkelevationcost).sum();

    // Un-halved 90-degree turn with turncost=100 gives ~100 (previously ~50)
    assert!(
        max_turn >= 80,
        "expected un-halved turn cost >= 80, got {max_turn}"
    );
    assert_eq!(
        total_init, 250,
        "expected initialcost 250 on classifier switch"
    );
    assert_eq!(total_node, 120, "expected barrier=gate nodecost 120");
    assert!(
        total_elev > 0,
        "expected non-zero linkelevationcost on +100m climb"
    );
}

#[test]
fn test_recalc_track_beeline_guard_and_incline_sign() {
    use brouter_rs::routing::cost::KinematicConfig;
    use brouter_rs::routing::track::{RoutePoint, RouteResult};
    use brouter_rs::routing::voice::WayMessage;

    let graph = RoutingGraph::new();
    let cfg = KinematicConfig {
        total_mass: 90.0,
        biker_power: 100.0,
        c_r: 0.01,
        s_c_x: 0.225,
        max_speed: 12.5,
        is_bike_mode: true,
        is_foot_mode: false,
    };

    // (a) Regular non-beeline step taking exactly 1.0s must NOT be altered
    let mut non_beeline = RouteResult {
        total_distance_m: 10.0,
        total_time_s: 1.0,
        total_energy_j: 100.0,
        total_cost: 10,
        filtered_ascend_m: 0.0,
        plain_ascend_m: 0.0,
        points: vec![
            RoutePoint {
                lon: 0.0,
                lat: 51.5,
                elevation_m: 10.0,
                distance_from_start_m: 0.0,
                time_from_start_s: 0.0,
                energy_j: 0.0,
                cost: 0,
            },
            RoutePoint {
                lon: 0.000144,
                lat: 51.5,
                elevation_m: 10.0,
                distance_from_start_m: 10.0,
                time_from_start_s: 1.0,
                energy_j: 100.0,
                cost: 10,
            },
        ],
        messages: vec![
            None,
            Some(WayMessage {
                way_key_values: "highway=residential".to_string(),
                time: 1.0,
                energy: 100.0,
                ..Default::default()
            }),
        ],
        voice_hints: Vec::new(),
        turn_instruction_mode: 0,
    };
    non_beeline.recalc_track(&graph, &cfg);
    assert!((non_beeline.total_time_s - 1.0).abs() < 1e-5);
    assert!((non_beeline.total_energy_j - 100.0).abs() < 1e-4);

    // (b) Uphill vs downhill beeline step: uphill adds more energy, downhill never adds negative energy
    let make_track = |ele0: f64, ele1: f64| RouteResult {
        total_distance_m: 200.0,
        total_time_s: 21.0,
        total_energy_j: 500.0,
        total_cost: 200,
        filtered_ascend_m: 0.0,
        plain_ascend_m: 0.0,
        points: vec![
            RoutePoint {
                lon: 0.0,
                lat: 51.5,
                elevation_m: 100.0,
                distance_from_start_m: 0.0,
                time_from_start_s: 0.0,
                energy_j: 0.0,
                cost: 0,
            },
            RoutePoint {
                lon: 0.00144,
                lat: 51.5,
                elevation_m: ele0,
                distance_from_start_m: 100.0,
                time_from_start_s: 20.0, // speed = 18 km/h
                energy_j: 500.0,
                cost: 100,
            },
            RoutePoint {
                lon: 0.00288,
                lat: 51.5,
                elevation_m: ele1,
                distance_from_start_m: 200.0,
                time_from_start_s: 21.0, // tmptime == 1.0s
                energy_j: 500.0,
                cost: 200,
            },
        ],
        messages: vec![
            None,
            Some(WayMessage {
                way_key_values: "highway=residential".to_string(),
                time: 20.0,
                energy: 500.0,
                ..Default::default()
            }),
            Some(WayMessage {
                way_key_values: "beeline=yes".to_string(),
                time: 21.0,
                energy: 500.0,
                ..Default::default()
            }),
        ],
        voice_hints: Vec::new(),
        turn_instruction_mode: 0,
    };

    let mut uphill = make_track(100.0, 120.0);
    let mut downhill = make_track(120.0, 100.0);
    uphill.recalc_track(&graph, &cfg);
    downhill.recalc_track(&graph, &cfg);

    let uphill_added = uphill.total_energy_j - 500.0;
    let downhill_added = downhill.total_energy_j - 500.0;
    assert!(
        uphill_added > downhill_added,
        "uphill added energy ({uphill_added}) must exceed downhill ({downhill_added})"
    );
    assert!(
        downhill_added >= 0.0,
        "downhill beeline energy must never be negative ({downhill_added})"
    );

    // (c) Track with only fast (>=9999 km/h) 1.0s beeline step must not divide by 9999.0
    let mut only_beeline = RouteResult {
        total_distance_m: 4000.0,
        total_time_s: 1.0,
        total_energy_j: 0.0,
        total_cost: 4000,
        filtered_ascend_m: 0.0,
        plain_ascend_m: 0.0,
        points: vec![
            RoutePoint {
                lon: 0.0,
                lat: 51.5,
                elevation_m: 0.0,
                distance_from_start_m: 0.0,
                time_from_start_s: 0.0,
                energy_j: 0.0,
                cost: 0,
            },
            RoutePoint {
                lon: 0.0576,
                lat: 51.5,
                elevation_m: 0.0,
                distance_from_start_m: 4000.0,
                time_from_start_s: 1.0,
                energy_j: 0.0,
                cost: 4000,
            },
        ],
        messages: vec![
            None,
            Some(WayMessage {
                way_key_values: "beeline=yes".to_string(),
                time: 1.0,
                energy: 0.0,
                ..Default::default()
            }),
        ],
        voice_hints: Vec::new(),
        turn_instruction_mode: 0,
    };
    only_beeline.recalc_track(&graph, &cfg);
    assert!((only_beeline.total_time_s - 1.0).abs() < 1e-5);
}

#[test]
fn test_geojson_escaping_profile_and_control_chars() {
    use brouter_rs::routing::track::{RoutePoint, RouteResult};
    use brouter_rs::routing::voice::WayMessage;

    let tricky_profile = "fast\"bike\\\n\t\x01";
    let tricky_tags = "name=Main \"St\"\\\r\n\t\x08\x0c\x01".to_string();

    let route = RouteResult {
        total_distance_m: 100.0,
        total_time_s: 10.0,
        total_energy_j: 50.0,
        total_cost: 100,
        filtered_ascend_m: 0.0,
        plain_ascend_m: 0.0,
        points: vec![
            RoutePoint {
                lon: 0.0,
                lat: 51.5,
                elevation_m: 0.0,
                distance_from_start_m: 0.0,
                time_from_start_s: 0.0,
                energy_j: 0.0,
                cost: 0,
            },
            RoutePoint {
                lon: 0.001,
                lat: 51.5,
                elevation_m: 0.0,
                distance_from_start_m: 100.0,
                time_from_start_s: 10.0,
                energy_j: 50.0,
                cost: 100,
            },
        ],
        messages: vec![
            None,
            Some(WayMessage {
                linkdist: 100,
                way_key_values: tricky_tags.clone(),
                ..Default::default()
            }),
        ],
        voice_hints: Vec::new(),
        turn_instruction_mode: 0,
    };

    let geojson_str = route.to_geojson(tricky_profile);
    let parsed: serde_json::Value =
        serde_json::from_str(&geojson_str).expect("GeoJSON must be valid JSON");
    let props = &parsed["features"][0]["properties"];
    assert_eq!(props["name"].as_str(), Some("fast\"bike\\\n\t\x01_route"));
    assert_eq!(props["messages"][1][9].as_str(), Some(tricky_tags.as_str()));
}

#[test]
fn test_multileg_unified_elevation_voicehints_and_via_correction() {
    use brouter_rs::graph::spatial::{ilat_to_lat, ilon_to_lon};
    use brouter_rs::rd5::codec::tag_coder::TagValueWrapper;
    use brouter_rs::rd5::micro_cache::{DecodedLink, DecodedMicroCache, DecodedNode};
    use brouter_rs::routing::voice::VoiceHint;

    let lookups = LookupData::parse("---context:way\n---context:node\n").unwrap();
    let profile_no_corr = Profile::parse(
        "multileg_no_corr",
        "---context:global\n\
         assign turnInstructionMode = 4\n\
         assign turnInstructionCatchingRange = 1\n\
         ---context:way\n\
         assign costfactor = 1.0\n\
         assign priorityclassifier = 10\n\
         ---context:node\n",
        &lookups,
    )
    .unwrap();
    let profile_with_corr = Profile::parse(
        "multileg_with_corr",
        "---context:global\n\
         assign turnInstructionMode = 4\n\
         assign turnInstructionCatchingRange = 1\n\
         assign correctMisplacedViaPoints = 1\n\
         ---context:way\n\
         assign costfactor = 1.0\n\
         assign priorityclassifier = 10\n\
         ---context:node\n",
        &lookups,
    )
    .unwrap();
    let profile_with_dist_corr = Profile::parse(
        "multileg_with_dist_corr",
        "---context:global\n\
         assign turnInstructionMode = 4\n\
         assign turnInstructionCatchingRange = 1\n\
         assign correctMisplacedViaPoints = 1\n\
         assign correctMisplacedViaPointsDistance = 400\n\
         ---context:way\n\
         assign costfactor = 1.0\n\
         assign priorityclassifier = 10\n\
         ---context:node\n",
        &lookups,
    )
    .unwrap();

    let way_tags = || {
        Some(TagValueWrapper {
            data: vec![1],
            access_type: 2,
        })
    };

    let lat_base: i32 = 141_000_000;
    // Bidirectional L-turn at J(180_004_000, lat_base, selev=i16::MIN) plus Spur East to 180_005_000:
    // A(0,0,100m) - B(2000,0,100m) - J(4000,0,MIN) - Spur(5000,0,100m)
    //                                       |
    //                                C(4000,+2000,150m) - D(4000,+4000,150m) - E(4000,+6000,150m)
    let make_link = |lon: i32, lat: i32| DecodedLink {
        target_lon: lon,
        target_lat: lat,
        is_reverse: false,
        way_tags: way_tags(),
        geometry: Vec::new(),
    };

    let cache = DecodedMicroCache {
        lon_idx: 180_000_000i32.div_euclid(12_500),
        lat_idx: lat_base.div_euclid(12_500),
        nodes: vec![
            DecodedNode {
                ilon: 180_000_000,
                ilat: lat_base,
                selev: 400,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![make_link(180_002_000, lat_base)],
            },
            DecodedNode {
                ilon: 180_002_000,
                ilat: lat_base,
                selev: 400,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    make_link(180_000_000, lat_base),
                    make_link(180_004_000, lat_base),
                ],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base,
                selev: i16::MIN, // missing bridge/tunnel elevation right at via junction
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    make_link(180_002_000, lat_base),
                    make_link(180_005_000, lat_base),
                    make_link(180_004_000, lat_base + 2_000),
                ],
            },
            DecodedNode {
                ilon: 180_005_000,
                ilat: lat_base,
                selev: 400,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![make_link(180_004_000, lat_base)],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base + 2_000,
                selev: 600,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    make_link(180_004_000, lat_base),
                    make_link(180_004_000, lat_base + 4_000),
                ],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base + 4_000,
                selev: 600,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![
                    make_link(180_004_000, lat_base + 2_000),
                    make_link(180_004_000, lat_base + 6_000),
                ],
            },
            DecodedNode {
                ilon: 180_004_000,
                ilat: lat_base + 6_000,
                selev: 600,
                node_tags: None,
                turn_restrictions: Vec::new(),
                links: vec![make_link(180_004_000, lat_base + 4_000)],
            },
        ],
    };

    let mut graph = RoutingGraph::new();
    graph.add_micro_cache(cache);
    graph.resolve_edge_target_elevations();

    let start = (ilon_to_lon(180_000_500), ilat_to_lat(lat_base));
    let target = (ilon_to_lon(180_004_000), ilat_to_lat(lat_base + 5_500));
    let j_lon = ilon_to_lon(180_004_000);
    let j_lat = ilat_to_lat(lat_base);
    let router = AStarRouter::new();

    // 1. Via right at J: verify missing elevation at J interpolated between 100m and 150m + unified turn voice hint
    let route_via_j = router
        .query(
            &graph,
            &lookups,
            &profile_no_corr,
            &[start, (j_lon, j_lat), target],
        )
        .compute()
        .expect("Multi-leg route through J must succeed");
    let pt_j = route_via_j
        .points
        .iter()
        .find(|p| (p.lon - j_lon).abs() < 1e-5 && (p.lat - j_lat).abs() < 1e-5)
        .expect("Junction J point must exist");
    assert!(
        pt_j.elevation_m > 110.0 && pt_j.elevation_m < 140.0,
        "Expected interpolated elevation at via J strictly between 100m and 150m, got {}",
        pt_j.elevation_m
    );
    assert!(
        route_via_j
            .voice_hints
            .iter()
            .any(|vh| vh.cmd == VoiceHint::TL || (vh.angle < -60.0)),
        "Expected left-turn voice hint at via junction J, got: {:?}",
        route_via_j.voice_hints
    );
    assert!(
        route_via_j
            .voice_hints
            .iter()
            .all(|vh| vh.cmd != VoiceHint::END),
        "Voice hints must not contain END command: {:?}",
        route_via_j.voice_hints
    );

    // 2. Misplaced via point along Spur: trims out-and-back stub only when correctMisplacedViaPoints=1
    let misplaced_via = (ilon_to_lon(180_004_500), ilat_to_lat(lat_base));
    let untrimmed = router
        .query(
            &graph,
            &lookups,
            &profile_no_corr,
            &[start, misplaced_via, target],
        )
        .compute()
        .expect("Untrimmed misplaced-via route must succeed");
    assert!(
        untrimmed.points.iter().any(|p| p.lon > j_lon + 0.0002),
        "Without correctMisplacedViaPoints, route must detour onto Spur"
    );

    for prof in [&profile_with_corr, &profile_with_dist_corr] {
        let trimmed = router
            .query(&graph, &lookups, prof, &[start, misplaced_via, target])
            .compute()
            .expect("Trimmed misplaced-via route must succeed");
        assert!(
            trimmed.points.iter().all(|p| p.lon <= j_lon + 1e-5),
            "With correctMisplacedViaPoints, Spur stub must be trimmed"
        );
        assert!(trimmed.total_distance_m < untrimmed.total_distance_m);
        assert!(
            trimmed
                .voice_hints
                .iter()
                .any(|vh| vh.cmd == VoiceHint::TL || (vh.angle < -60.0)),
            "Expected left-turn voice hint at trimmed junction J, got: {:?}",
            trimmed.voice_hints
        );
    }
}

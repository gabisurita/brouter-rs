//! Real tests for kinematics, elevation hysteresis, and full route search.

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use brouter_rs::dsl::evaluator::WayCostResult;
use brouter_rs::dsl::lookups::LookupData;
use brouter_rs::dsl::parser::Profile;
use brouter_rs::graph::routing_graph::RoutingGraph;
use brouter_rs::rd5::tile_manager::TileManager;
use brouter_rs::graph::spatial::{distance, lat_to_ilat, lon_to_ilon, Point2D};
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
            cos_angle: 1.0, // straight turn
            crossing_penalty: 0,   // no crossing penalty
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
        println!("Skipping real route test: {:?} does not exist", segment_path);
        return;
    }

    println!("Loading Lookups and Fastbike Profile...");
    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike.brf"), &lookups)
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
        println!("Skipping clapham_hackney test: {:?} does not exist", segment_path);
        return;
    }

    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike.brf"), &lookups)
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
    let dist_diff = (route.total_distance_m - 13171.0).abs();
    assert!(
        dist_diff <= 1.0,
        "Distance divergence > 1m: got {}",
        route.total_distance_m
    );
    assert_eq!(
        route.total_cost, 21803,
        "Cost divergence from BRouter baseline"
    );
    assert_eq!(
        route.points.len(),
        919,
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

    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike-lowtraffic.brf"), &lookups)
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
        route.total_cost, 24661,
        "Cost divergence from BRouter baseline"
    );
    assert_eq!(
        route.points.len(),
        910,
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

    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike-verylowtraffic.brf"), &lookups)
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
    let dist_diff = (route.total_distance_m - 14267.0).abs();
    assert!(
        dist_diff <= 1.0,
        "Distance divergence > 1m: got {}",
        route.total_distance_m
    );
    assert_eq!(
        route.total_cost, 25075,
        "Cost divergence from BRouter baseline"
    );
    assert_eq!(
        route.points.len(),
        885,
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
    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
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
        println!("Skipping test_route_with_nogo_zone: {:?} does not exist", segment_path);
        return;
    }

    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike.brf"), &lookups)
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
    let lookups =
        LookupData::load_from_file(repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");

    // 1. Verify foot mode Tobler's hiking speed is not clamped by way maxspeed
    let hiking = Profile::load_from_file(repo_path("profiles/hiking-mountain.brf"), &lookups)
        .expect("Failed to load hiking-mountain.brf");
    let hiking_k = KinematicConfig::from_profile(&hiking);
    assert!(hiking_k.is_foot_mode);
    let (foot_speed, foot_energy) = hiking_k.compute_speed_and_energy(100.0, 0.0, 0.5);
    assert!(foot_speed > 1.0, "Foot mode speed should follow Tobler's formula, got {}", foot_speed);
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

    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike.brf"), &lookups)
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

    let lookups =
        LookupData::load_from_file(&repo_path("profiles/lookups.dat")).expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike.brf"), &lookups)
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
    assert_eq!(beeline_route.total_cost, beeline_route.total_distance_m as u32);
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
    use std::sync::Arc;
    use brouter_rs::rd5::rd5_slice::{FileRd5SegmentReader, LazyRd5Provider};

    let segments_dir = repo_path("data/segments4");
    if !segments_dir.join("W5_N50.rd5").exists() {
        return;
    }

    let lookups = LookupData::load_from_file(&repo_path("profiles/lookups.dat"))
        .expect("Failed to load lookups.dat");
    let profile = Profile::load_from_file(&repo_path("profiles/fastbike.brf"), &lookups)
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
    assert_eq!(lazy_route_1.voice_hints.len(), ref_route_1.voice_hints.len());

    assert_eq!(lazy_route_2.total_cost, ref_route_2.total_cost);
    assert!((lazy_route_2.total_distance_m - ref_route_2.total_distance_m).abs() < 1e-6);
    assert_eq!(lazy_route_2.points.len(), ref_route_2.points.len());
    assert_eq!(lazy_route_2.voice_hints.len(), ref_route_2.voice_hints.len());

    assert!(cells_after_1 > 0 && cells_after_1 < 60, "Expected < 60 cells, got {}", cells_after_1);
    assert!(cells_after_2 >= cells_after_1 && cells_after_2 < 80, "Expected < 80 cells, got {}", cells_after_2);

    let safe_profile = Profile::load_from_file(&repo_path("profiles/fastbike-lowtraffic.brf"), &lookups)
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
    assert_eq!(lazy_route_11_moved.total_cost, ref_route_11_moved.total_cost);
    assert!((lazy_route_11_moved.total_distance_m - ref_route_11_moved.total_distance_m).abs() < 1e-6);
    assert_eq!(lazy_route_11_moved.points.len(), ref_route_11_moved.points.len());
}







//! Tests for parsing and evaluating all BRouter profiles.
//! Corresponding to official BRouter tests:
//! - Java test: `btools.expressions.EncodeDecodeTest`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/test/java/btools/expressions/EncodeDecodeTest.java`
//! - Java test: `btools.expressions.IntegrityCheckProfileTest`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/test/java/btools/expressions/IntegrityCheckProfileTest.java`

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use brouter_rs::dsl::evaluator::ProfileEvaluator;
use brouter_rs::dsl::lookups::LookupData;
use brouter_rs::dsl::parser::Profile;

fn repo_path(rel: &str) -> PathBuf {
    let p = Path::new(rel);
    if p.exists() {
        return p.to_path_buf();
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn test_parse_lookups_dat() {
    let lookups_path = repo_path("profiles/lookups.dat");
    assert!(lookups_path.exists(), "lookups.dat must exist in profiles/");

    let lookups = LookupData::load_from_file(&lookups_path).expect("Failed to parse lookups.dat");

    assert_eq!(lookups.lookup_version, 11);
    assert_eq!(lookups.minor_version, 2);

    // Verify hardwired context tags
    assert_eq!(lookups.way.names[0], "reversedirection");
    assert_eq!(lookups.node.names[0], "nodeaccessgranted");

    // Verify common OSM tags
    assert!(lookups.way.get_name_idx("highway").is_some());
    assert!(lookups.way.get_name_idx("surface").is_some());
    assert!(lookups.way.get_name_idx("maxspeed").is_some());
    assert!(lookups.way.get_name_idx("bicycle").is_some());

    let highway_idx = lookups.way.get_name_idx("highway").unwrap();
    assert!(lookups.way.get_value_idx(highway_idx, "primary").is_some());
    assert!(lookups
        .way
        .get_value_idx(highway_idx, "residential")
        .is_some());
    assert!(lookups.way.get_value_idx(highway_idx, "cycleway").is_some());
    assert!(lookups.way.get_value_idx(highway_idx, "track").is_some());

    println!(
        "lookups.dat loaded: {} way tag names, {} node tag names",
        lookups.way.names.len(),
        lookups.node.names.len()
    );
}

#[test]
fn test_parse_all_brouter_profiles() {
    let lookups =
        LookupData::load_from_file(repo_path("profiles/lookups.dat")).expect("Failed to parse lookups.dat");
    let profiles_dir = repo_path("profiles");

    let mut profile_paths = Vec::new();
    for entry in fs::read_dir(profiles_dir).expect("Failed to read profiles dir") {
        let entry = entry.expect("Directory entry error");
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) == Some("brf") {
            profile_paths.push(path);
        }
    }

    profile_paths.sort();
    assert!(
        profile_paths.len() >= 7,
        "Expected at least 7 test profile files, found {}",
        profile_paths.len()
    );

    println!(
        "\nParsing all {} profiles from BRouter...",
        profile_paths.len()
    );

    let t0 = Instant::now();
    let mut parsed_profiles = Vec::new();

    for path in &profile_paths {
        let name = path.file_stem().unwrap().to_string_lossy().to_string();
        let t_p = Instant::now();
        match Profile::load_from_file(path, &lookups) {
            Ok(p) => {
                let el = t_p.elapsed().as_secs_f64() * 1000.0;
                println!(
                    "  [OK] {:30} in {:6.2} ms (global: {:3} stmts, way: {:3} stmts, node: {:2} stmts)",
                    name,
                    el,
                    p.global_block.statements.len(),
                    p.way_block.statements.len(),
                    p.node_block.statements.len()
                );
                parsed_profiles.push(p);
            }
            Err(e) => {
                panic!("Failed to parse profile {}: {}", name, e);
            }
        }
    }

    let total_elapsed = t0.elapsed();
    println!(
        "\nSuccessfully parsed all {} profiles in {:.2} ms (average {:.2} ms/profile)!",
        parsed_profiles.len(),
        total_elapsed.as_secs_f64() * 1000.0,
        total_elapsed.as_secs_f64() * 1000.0 / parsed_profiles.len() as f64
    );

    // Verify key focus profiles
    let fastbike = parsed_profiles
        .iter()
        .find(|p| p.name == "fastbike")
        .expect("fastbike profile not found");
    assert_eq!(fastbike.get_global_var("validForBikes", 0.0), 1.0);
    assert_eq!(fastbike.get_global_var("allow_steps", 0.0), 1.0);
    assert_eq!(fastbike.get_global_var("allow_motorways", 0.0), 0.0);

    let lowtraffic = parsed_profiles
        .iter()
        .find(|p| p.name == "fastbike-lowtraffic")
        .expect("fastbike-lowtraffic profile not found");
    assert_eq!(lowtraffic.get_global_var("validForBikes", 0.0), 1.0);

    let verylowtraffic = parsed_profiles
        .iter()
        .find(|p| p.name == "fastbike-verylowtraffic")
        .expect("fastbike-verylowtraffic profile not found");
    assert_eq!(verylowtraffic.get_global_var("validForBikes", 0.0), 1.0);
}

#[test]
fn test_evaluate_profile_way_tags() {
    let lookups =
        LookupData::load_from_file(repo_path("profiles/lookups.dat")).expect("Failed to parse lookups.dat");
    let profile = Profile::load_from_file(repo_path("profiles/fastbike.brf"), &lookups)
        .expect("Failed to load fastbike");

    let mut evaluator = ProfileEvaluator::new(&profile, &lookups.way, &lookups.node);

    let mut ld = vec![0i32; lookups.way.names.len()];

    // Case 1: Motorway -> should have very high costfactor (forbidden)
    let hw_idx = lookups.way.get_name_idx("highway").unwrap();
    let motorway_val = lookups.way.get_value_idx(hw_idx, "motorway").unwrap();
    ld[hw_idx] = motorway_val;

    let res_motorway = evaluator.evaluate_way(&ld);
    assert!(
        res_motorway.costfactor >= 9999.0 || !res_motorway.nodeaccessgranted,
        "Motorway should be forbidden on bike profile, got costfactor={}",
        res_motorway.costfactor
    );

    // Case 2: Cycleway with asphalt -> should have low costfactor (preferred)
    let cycleway_val = lookups.way.get_value_idx(hw_idx, "cycleway").unwrap();
    let surf_idx = lookups.way.get_name_idx("surface").unwrap();
    let asphalt_val = lookups.way.get_value_idx(surf_idx, "asphalt").unwrap();
    ld[hw_idx] = cycleway_val;
    ld[surf_idx] = asphalt_val;

    let res_cycleway = evaluator.evaluate_way(&ld);
    assert!(
        res_cycleway.costfactor > 0.0 && res_cycleway.costfactor <= 2.0,
        "Cycleway should have low costfactor, got {}",
        res_cycleway.costfactor
    );
    assert!(!res_cycleway.nodeaccessgranted);

    // Case 3: Cycleway with lcn=yes -> nodeaccessgranted should become true
    if let Some(lcn_idx) = lookups.way.get_name_idx("lcn") {
        if let Some(yes_val) = lookups.way.get_value_idx(lcn_idx, "yes") {
            ld[lcn_idx] = yes_val;
            let res_lcn = evaluator.evaluate_way(&ld);
            assert!(res_lcn.nodeaccessgranted);
        }
    }

    println!(
        "Evaluation test passed: Motorway costfactor={:.1}, Cycleway costfactor={:.2}",
        res_motorway.costfactor, res_cycleway.costfactor
    );
}

#[test]
fn test_profile_custom_params_and_no_start_way() {
    use std::collections::HashMap;
    use brouter_rs::routing::cost::CarKinematicModel;

    let lookups =
        LookupData::load_from_file(repo_path("profiles/lookups.dat")).expect("Failed to parse lookups.dat");

    let mut params = HashMap::new();
    params.insert("avoid_main_roads".to_string(), "8".to_string());
    let fastbike_custom = Profile::load_from_file_with_params(
        repo_path("profiles/fastbike.brf"),
        &lookups,
        Some(&params),
    )
    .expect("Failed to load fastbike with custom params");
    assert_eq!(fastbike_custom.get_global_var("avoid_main_roads", 0.0), 8.0);

    // Check car-eco profile loads CarKinematicModel
    let car_eco = Profile::load_from_file(repo_path("profiles/car-eco.brf"), &lookups)
        .expect("Failed to load car-eco.brf");
    let km = CarKinematicModel::from_profile(&car_eco).expect("Expected CarKinematicModel for car-eco");
    assert!(!km.no_cost_factor);
    assert!((km.total_weight - 1640.0).abs() < 1e-3);

    // Check noStartWay parsing and check_start_way_decoded
    let no_start_src = r#"
---context:global
assign check_start_way = true # %check_start_way% | Do not start on motorways | noStartWay=highway,motorway;highway,trunk
---context:way
assign costfactor = 1
---context:node
assign initialcost = 0
"#;
    let no_start_prof =
        Profile::parse("no_start_test", no_start_src, &lookups).expect("Failed to parse no_start_test");
    assert_eq!(no_start_prof.no_start_ways.len(), 2);

    let hw_idx = lookups.way.get_name_idx("highway").unwrap();
    let motorway_val = lookups.way.get_value_idx(hw_idx, "motorway").unwrap();
    let residential_val = lookups.way.get_value_idx(hw_idx, "residential").unwrap();
    let mut ld = vec![0i32; lookups.way.names.len()];
    ld[hw_idx] = motorway_val;
    assert!(!no_start_prof.check_start_way_decoded(&ld));
    ld[hw_idx] = residential_val;
    assert!(no_start_prof.check_start_way_decoded(&ld));
}

#[test]
fn test_foreign_variable_resolution() {
    let lookups =
        LookupData::load_from_file(repo_path("profiles/lookups.dat")).expect("Failed to parse lookups.dat");
    let profile = |node_expr: &str| {
        let src = format!(
            "---context:global\nassign g = 1\n---context:way\nassign costfactor = 1\nassign w = 2\n---context:node\nassign initialcost = {node_expr}\n"
        );
        Profile::parse("foreign_test", &src, &lookups)
    };

    let ok = profile("way:w").expect("way:w should resolve");
    let idx = ok.way_block.var_map["w"];
    assert_eq!(ok.node_block.foreign_var_indices(), vec![idx]);

    for (expr, msg) in [
        ("way:typo", "unknown variable: way:typo"),
        ("node:w", "unknown foreign context: node"),
        ("way:g", "bad access to global variable: way:g"),
    ] {
        let err = profile(expr).expect_err(expr);
        assert!(err.contains(msg), "{expr}: {err}");
    }

    let src = "---context:global\n---context:way\nassign costfactor = way:x\n---context:node\nassign initialcost = 0\n";
    let err = Profile::parse("foreign_in_way", src, &lookups).expect_err("way context has no foreign context");
    assert!(err.contains("unknown foreign context: way"), "{err}");
}

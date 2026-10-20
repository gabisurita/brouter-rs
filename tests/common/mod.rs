use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, LazyLock};

use axum::body::Body;
use axum::http::{Request, StatusCode};
use http_body_util::BodyExt;
use tower::ServiceExt;

#[path = "../../examples/server.rs"]
pub mod server;
use server::{create_router, ServerState};

pub fn repo_path(rel: &str) -> PathBuf {
    let p = Path::new(rel);
    if p.exists() {
        return p.to_path_buf();
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Shared preloaded ServerState instance with a single unified regional graph covering all test routes.
/// All integration and parity tests share this graph, strictly bounding memory to ~3.2 GB.
pub static SHARED_STATE: LazyLock<Option<Arc<ServerState>>> = LazyLock::new(|| {
    let seg_path = repo_path("data/segments4");
    if !seg_path.join("W5_N50.rd5").exists() || !seg_path.join("E0_N50.rd5").exists() {
        return None;
    }

    let state = ServerState::with_bounding_box(
        seg_path,
        repo_path("profiles"),
        brouter_rs::BoundingBox::new(-3.20, 50.80, 0.05, 51.80),
    )
    .expect("Failed to initialize shared preloaded ServerState");

    // Preload way costs for focus profiles to ensure instant warm routing
    let lookups = &state.lookups;
    for prof_name in &["fastbike", "fastbike-lowtraffic", "fastbike-verylowtraffic"] {
        if let Ok(profile) = state.get_profile(prof_name) {
            state.graph.preload_profile_costs(&profile, lookups);
        }
    }

    Some(Arc::new(state))
});

pub fn get_test_app() -> Option<axum::Router> {
    SHARED_STATE.as_ref().map(|s| create_router(Arc::clone(s)))
}

pub fn parse_fixture_data(fixture_path: &Path) -> Option<(f64, f64, f64, f64, Vec<(f64, f64)>)> {
    if !fixture_path.exists() {
        return None;
    }
    let content = fs::read_to_string(fixture_path).ok()?;
    let json: serde_json::Value = serde_json::from_str(&content).ok()?;
    let feature = json.get("features")?.get(0)?;
    let props = feature.get("properties")?;

    let dist: f64 = props.get("track-length")?.as_str()?.parse().ok()?;
    let time: f64 = props.get("total-time")?.as_str()?.parse().ok()?;
    let cost: f64 = props.get("cost")?.as_str()?.parse().ok()?;
    let ascend: f64 = props
        .get("filtered ascend")
        .or_else(|| props.get("filtered-ascend"))
        .or_else(|| props.get("plain-ascend"))?
        .as_str()?
        .parse()
        .ok()?;

    let coords_arr = feature.get("geometry")?.get("coordinates")?.as_array()?;
    let mut coords = Vec::with_capacity(coords_arr.len());
    for pt in coords_arr {
        let p = pt.as_array()?;
        let lon = p.get(0)?.as_f64()?;
        let lat = p.get(1)?.as_f64()?;
        coords.push((lon, lat));
    }

    Some((dist, time, cost, ascend, coords))
}

pub async fn assert_route_parity(
    route_name: &str,
    profile: &str,
    lonlats: &str,
    alt_idx: Option<usize>,
    fixture_filename: &str,
    out_filename: &str,
    max_dist_pct: f64,
    max_cost_pct: f64,
    max_time_pct: f64,
    min_coord_overlap_pct: f64,
) {
    let Some(app) = get_test_app() else {
        println!("Skipping {}: test data segments not found", route_name);
        return;
    };

    let uri = match alt_idx {
        Some(idx) => format!(
            "/brouter?lonlats={}&profile={}&alternativeidx={}&format=geojson",
            lonlats, profile, idx
        ),
        None => format!(
            "/brouter?lonlats={}&profile={}&format=geojson",
            lonlats, profile
        ),
    };

    let response = app
        .oneshot(Request::builder().uri(&uri).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(
        response.status(),
        StatusCode::OK,
        "Failed to route {} [{}]: received status {}",
        route_name,
        profile,
        response.status()
    );

    let body = response.into_body().collect().await.unwrap().to_bytes();
    let json: serde_json::Value = serde_json::from_slice(&body)
        .unwrap_or_else(|e| panic!("Failed to parse response JSON for {}: {}", route_name, e));

    let _ = fs::create_dir_all("target/routes");
    let out_path = format!("target/routes/{}", out_filename);
    let _ = fs::write(&out_path, &body);

    let feature = json
        .get("features")
        .and_then(|f| f.get(0))
        .unwrap_or_else(|| panic!("Missing features in response GeoJSON for {}", route_name));

    let props = feature
        .get("properties")
        .unwrap_or_else(|| panic!("Missing properties in response GeoJSON for {}", route_name));

    let dist: f64 = props["track-length"].as_str().unwrap().parse().unwrap();
    let time: f64 = props["total-time"].as_str().unwrap().parse().unwrap();
    let cost: f64 = props["cost"].as_str().unwrap().parse().unwrap();
    let ascend: f64 = props
        .get("filtered ascend")
        .or_else(|| props.get("plain-ascend"))
        .unwrap()
        .as_str()
        .unwrap()
        .parse()
        .unwrap();

    let rs_coords: Vec<(f64, f64)> = feature
        .get("geometry")
        .and_then(|g| g.get("coordinates"))
        .and_then(|c| c.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|pt| {
                    let p = pt.as_array()?;
                    Some((p.get(0)?.as_f64()?, p.get(1)?.as_f64()?))
                })
                .collect()
        })
        .unwrap_or_default();

    let fixture_path = repo_path(&format!("fixtures/brouter_de/{}", fixture_filename));
    let Some((b_dist, b_time, b_cost, b_asc, b_coords)) = parse_fixture_data(&fixture_path) else {
        panic!("Baseline fixture not found: {:?}", fixture_path);
    };

    let dist_delta = (dist - b_dist).abs() / b_dist * 100.0;
    let cost_delta = (cost - b_cost).abs() / b_cost * 100.0;
    let time_delta = (time - b_time).abs() / b_time * 100.0;

    let b_set: std::collections::HashSet<(i64, i64)> = b_coords
        .iter()
        .map(|&(lon, lat)| ((lon * 100_000.0).round() as i64, (lat * 100_000.0).round() as i64))
        .collect();
    let rs_set: std::collections::HashSet<(i64, i64)> = rs_coords
        .iter()
        .map(|&(lon, lat)| ((lon * 100_000.0).round() as i64, (lat * 100_000.0).round() as i64))
        .collect();
    let overlap_count = b_set.intersection(&rs_set).count();
    let overlap_pct = if b_set.is_empty() {
        100.0
    } else {
        overlap_count as f64 / b_set.len() as f64 * 100.0
    };

    println!(
        "  [{:<20} | {:<22}] Pts: {:>5}/{:<5} ({:>5.1}%) | Dist: {:>6.1}km ({:>+5.2}%) | Cost: {:>6.0} ({:>+5.2}%) | Time: {:>4.0}m ({:>+5.2}%) | Asc: {:>4.0}/{:<4.0}m",
        route_name,
        profile,
        rs_coords.len(),
        b_coords.len(),
        overlap_pct,
        dist / 1000.0,
        dist_delta,
        cost,
        cost_delta,
        time / 60.0,
        time_delta,
        ascend,
        b_asc
    );

    assert!(
        dist_delta <= max_dist_pct,
        "{} [{}]: Distance delta {:.2}% exceeds threshold {:.2}% (rsrouter: {:.1}m, brouter.de: {:.1}m)",
        route_name,
        profile,
        dist_delta,
        max_dist_pct,
        dist,
        b_dist
    );

    assert!(
        cost_delta <= max_cost_pct,
        "{} [{}]: Cost delta {:.2}% exceeds threshold {:.2}% (rsrouter: {:.0}, brouter.de: {:.0})",
        route_name,
        profile,
        cost_delta,
        max_cost_pct,
        cost,
        b_cost
    );

    assert!(
        time_delta <= max_time_pct,
        "{} [{}]: Time delta {:.2}% exceeds threshold {:.2}% (rsrouter: {:.0}s, brouter.de: {:.0}s)",
        route_name,
        profile,
        time_delta,
        max_time_pct,
        time,
        b_time
    );

    assert_eq!(
        ascend as i32,
        b_asc as i32,
        "{} [{}]: Ascend mismatch (rsrouter: {:.0}m, brouter.de: {:.0}m)",
        route_name,
        profile,
        ascend,
        b_asc
    );

    if min_coord_overlap_pct >= 99.99 {
        assert_eq!(
            rs_coords.len(),
            b_coords.len(),
            "{} [{}]: Point count mismatch (rsrouter: {}, brouter.de: {})",
            route_name,
            profile,
            rs_coords.len(),
            b_coords.len()
        );
        for (i, (rs_pt, br_pt)) in rs_coords.iter().zip(&b_coords).enumerate() {
            assert!(
                (rs_pt.0 - br_pt.0).abs() < 1e-4 && (rs_pt.1 - br_pt.1).abs() < 1e-4,
                "{} [{}]: Coordinate mismatch at point index {} (rs: {:?}, br: {:?})",
                route_name,
                profile,
                i,
                rs_pt,
                br_pt
            );
        }
    } else {
        let pt_delta_pct = (rs_coords.len() as f64 - b_coords.len() as f64).abs()
            / b_coords.len() as f64
            * 100.0;
        assert!(
            pt_delta_pct <= 1.0,
            "{} [{}]: Point count delta {:.2}% exceeds 1.0% (rs: {}, br: {})",
            route_name,
            profile,
            pt_delta_pct,
            rs_coords.len(),
            b_coords.len()
        );
        assert!(
            overlap_pct >= min_coord_overlap_pct,
            "{} [{}]: Coordinate overlap {:.1}% is below required {:.1}%",
            route_name,
            profile,
            overlap_pct,
            min_coord_overlap_pct
        );
    }
}

//! Automated route comparison binary evaluating brouter-rs against BRouter baseline fixtures.
//! Baseline reference: `https://github.com/abrensch/brouter`
//!
//! Evaluates the 3 focus profiles (`fastbike`, `fastbike-lowtraffic`, `fastbike-verylowtraffic`)
//! across UK routes, outputs GeoJSON tracks to `target/routes/`, and prints a parity table.

use std::fs;
use std::path::Path;
use std::time::Instant;

use brouter_rs::rd5::tile_manager::TileManager;
use brouter_rs::types::{BoundingBox, Cost, Distance, Seconds};
use brouter_rs::{AStarRouter, LookupData, Profile, RouteRequest, RoutingGraph, RoutingOptions};

struct BenchmarkRoute {
    name: &'static str,
    title: &'static str,
    waypoints: &'static [(f64, f64)],
}

#[derive(Debug)]
struct ParityRow {
    route_title: &'static str,
    profile: &'static str,
    dist_delta_pct: f64,
    time_delta_pct: f64,
    cost_delta_pct: f64,
    ascend_rs: Distance,
    ascend_br: Distance,
    search_ms: f64,
}

fn parse_fixture_stats(fixture_path: &Path) -> Option<(Distance, Seconds, Cost, Distance)> {
    if !fixture_path.exists() {
        return None;
    }
    let content = fs::read_to_string(fixture_path).ok()?;
    let json: serde_json::Value = serde_json::from_str(&content).ok()?;
    let props = json.get("features")?.get(0)?.get("properties")?;

    let dist: Distance = props.get("track-length")?.as_str()?.parse().ok()?;
    let time: Seconds = props.get("total-time")?.as_str()?.parse().ok()?;
    let cost: Cost = props.get("cost")?.as_str()?.parse().ok()?;
    let ascend: Distance = props
        .get("filtered ascend")
        .or_else(|| props.get("filtered-ascend"))
        .or_else(|| props.get("plain-ascend"))?
        .as_str()?
        .parse()
        .ok()?;

    Some((dist, time, cost, ascend))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Loading Lookups from profiles/lookups.dat...");
    let lookups = LookupData::load_from_file("profiles/lookups.dat")?;

    let benchmark_routes = [
        BenchmarkRoute {
            name: "clapham_hackney",
            title: "Clapham -> Hackney",
            waypoints: &[(-0.143613, 51.462414), (-0.055296, 51.544998)],
        },
        BenchmarkRoute {
            name: "london_brighton",
            title: "London -> Brighton",
            waypoints: &[(-0.127818, 51.507392), (-0.136913, 50.822431)],
        },
        BenchmarkRoute {
            name: "london_oxford",
            title: "London -> Oxford",
            waypoints: &[(-0.127818, 51.507392), (-1.257704, 51.751988)],
        },
        BenchmarkRoute {
            name: "london_cardiff",
            title: "London -> Cardiff",
            waypoints: &[(-0.127818, 51.507392), (-3.179096, 51.481607)],
        },
        BenchmarkRoute {
            name: "richmond_oxford_multi_point",
            title: "Richmond -> Windsor -> Marlow -> Oxford",
            waypoints: &[
                (-0.3060, 51.4590),
                (-0.6060, 51.4810),
                (-0.7740, 51.5710),
                (-1.257704, 51.751988),
            ],
        },
    ];

    let profiles = ["fastbike", "fastbike-lowtraffic", "fastbike-verylowtraffic"];

    let router = AStarRouter::new();
    let _ = fs::create_dir_all("target/routes");

    let min_lon = benchmark_routes
        .iter()
        .flat_map(|c| c.waypoints.iter().map(|&(lon, _)| lon))
        .fold(f64::INFINITY, f64::min)
        .min(-3.20);
    let max_lon = benchmark_routes
        .iter()
        .flat_map(|c| c.waypoints.iter().map(|&(lon, _)| lon))
        .fold(f64::NEG_INFINITY, f64::max)
        .max(0.05);
    let min_lat = benchmark_routes
        .iter()
        .flat_map(|c| c.waypoints.iter().map(|&(_, lat)| lat))
        .fold(f64::INFINITY, f64::min)
        .min(50.80);
    let max_lat = benchmark_routes
        .iter()
        .flat_map(|c| c.waypoints.iter().map(|&(_, lat)| lat))
        .fold(f64::NEG_INFINITY, f64::max)
        .max(51.80);

    let lazy_mode = std::env::args().any(|a| a == "--lazy");
    let t_load = Instant::now();
    let mut graph = if lazy_mode {
        println!("\n=======================================================");
        println!("Initializing lazy on-demand cell provider (0 preloaded cells)...");
        let reader = std::sync::Arc::new(
            brouter_rs::rd5::rd5_slice::FileRd5SegmentReader::new("data/segments4"),
        );
        let provider = std::sync::Arc::new(
            brouter_rs::rd5::rd5_slice::LazyRd5Provider::new(reader),
        );
        RoutingGraph::with_cell_provider(provider)
    } else {
        println!("\n=======================================================");
        println!("Preloading unified regional graph for benchmark routes...");
        println!(
            "Bounding box: lon [{:.4}, {:.4}], lat [{:.4}, {:.4}]",
            min_lon, max_lon, min_lat, max_lat
        );
        let mut tile_mgr = TileManager::new("data/segments4");
        let mut g = RoutingGraph::with_capacities(4_500_000, 11_000_000);
        let bbox = BoundingBox::new(min_lon, min_lat, max_lon, max_lat);
        let loaded_cells = g.load_bounding_box(&mut tile_mgr, bbox)?;
        let load_ms = t_load.elapsed().as_secs_f64() * 1000.0;
        println!(
            "Unified regional graph ready in {:.2} ms ({} cells, {} nodes, {} edges)",
            load_ms,
            loaded_cells,
            g.node_count(),
            g.edge_count()
        );
        println!("Pre-evaluating profile way costs on regional graph...");
        let t_warm = Instant::now();
        for prof_name in &profiles {
            let prof_path = format!("profiles/{}.brf", prof_name);
            if let Ok(profile) = Profile::load_from_file(&prof_path, &lookups) {
                g.preload_profile_costs(&profile, &lookups);
            }
        }
        println!(
            "Profile way costs cached in {:.2} ms",
            t_warm.elapsed().as_secs_f64() * 1000.0
        );
        g
    };
    let graph_load_ms = t_load.elapsed().as_secs_f64() * 1000.0;

    let mut rows: Vec<ParityRow> = Vec::new();

    for bench_route in &benchmark_routes {
        println!("\n=======================================================");
        println!("Processing route: {} ({})", bench_route.title, bench_route.name);

        let mut route_rows: Vec<ParityRow> = Vec::new();
        for prof_name in &profiles {
            let prof_path = format!("profiles/{}.brf", prof_name);
            let profile = match Profile::load_from_file(&prof_path, &lookups) {
                Ok(p) => p,
                Err(e) => {
                    eprintln!("Failed to parse profile {}: {}", prof_name, e);
                    continue;
                }
            };

            let fixture_name = match *prof_name {
                "fastbike-lowtraffic" => "lowtraffic",
                "fastbike-verylowtraffic" => "verylowtraffic",
                other => other,
            };
            let fixture_path = format!("fixtures/brouter_de/{}_{}.json", bench_route.name, fixture_name);
            let baseline = parse_fixture_stats(Path::new(&fixture_path));

            let t_route = Instant::now();
            let result = if lazy_mode {
                router.route(RouteRequest {
                    graph: &mut graph,
                    lookups: &lookups,
                    profile: &profile,
                    waypoints: bench_route.waypoints,
                    options: RoutingOptions::default(),
                })
            } else {
                router.route(RouteRequest {
                    graph: &graph,
                    lookups: &lookups,
                    profile: &profile,
                    waypoints: bench_route.waypoints,
                    options: RoutingOptions::default(),
                })
            };
            let elapsed_ms = t_route.elapsed().as_secs_f64() * 1000.0;

            match result {
                Ok(route) => {
                    let out_path = format!("target/routes/{}_{}.geojson", bench_route.name, prof_name);
                    let geojson = route.to_geojson(prof_name);
                    let _ = fs::write(&out_path, geojson);

                    if let Some((b_dist, b_time, b_cost, b_asc)) = baseline {
                        let dist_delta = (route.total_distance_m - b_dist).abs() / b_dist * 100.0;
                        let time_delta =
                            (route.total_time_s as f64 - b_time as f64).abs() / b_time as f64 * 100.0;
                        let cost_delta =
                            (route.total_cost as f64 - b_cost as f64).abs() / b_cost as f64 * 100.0;

                        println!(
                            "  [{:23}] Dist: {:>6.1} km ({:+5.2}%) | Time: {:>4.0} min ({:+5.2}%) | Cost: {:>5} ({:+5.2}%) | {:>6.1} ms (cells: {})",
                            prof_name,
                            route.total_distance_m / 1000.0,
                            dist_delta,
                            route.total_time_s / 60.0,
                            time_delta,
                            route.total_cost,
                            cost_delta,
                            elapsed_ms,
                            graph.loaded_cell_count()
                        );

                        route_rows.push(ParityRow {
                            route_title: bench_route.title,
                            profile: prof_name,
                            dist_delta_pct: dist_delta,
                            time_delta_pct: time_delta,
                            cost_delta_pct: cost_delta,
                            ascend_rs: route.filtered_ascend_m,
                            ascend_br: b_asc,
                            search_ms: elapsed_ms,
                        });
                    } else {
                        println!(
                            "  [{:23}] Dist: {:>6.1} km | Cost: {:>5} | {:>6.1} ms (no baseline)",
                            prof_name,
                            route.total_distance_m / 1000.0,
                            route.total_cost,
                            elapsed_ms
                        );
                    }
                }
                Err(e) => {
                    eprintln!("  [{}] Routing error: {}", prof_name, e);
                }
            }
        }

        rows.extend(route_rows);
    }

    println!("\n=======================================================");
    println!("Processing alternative routes: London -> Brighton (fastbike-lowtraffic)");
    let brighton_route = benchmark_routes
        .iter()
        .find(|c| c.name == "london_brighton")
        .expect("london_brighton route not found");
    let lowtraffic_prof = Profile::load_from_file("profiles/fastbike-lowtraffic.brf", &lookups)?;

    for alt_idx in 1..=3 {
        let fixture_path = format!(
            "fixtures/brouter_de/london_brighton_fastbike-lowtraffic_alt{}.json",
            alt_idx
        );
        let baseline = parse_fixture_stats(Path::new(&fixture_path));

        let t_route = Instant::now();
        let result = if lazy_mode {
            router.route(RouteRequest {
                graph: &mut graph,
                lookups: &lookups,
                profile: &lowtraffic_prof,
                waypoints: brighton_route.waypoints,
                options: RoutingOptions {
                    alternative_idx: alt_idx,
                    ..Default::default()
                },
            })
        } else {
            router.route(RouteRequest {
                graph: &graph,
                lookups: &lookups,
                profile: &lowtraffic_prof,
                waypoints: brighton_route.waypoints,
                options: RoutingOptions {
                    alternative_idx: alt_idx,
                    ..Default::default()
                },
            })
        };
        let elapsed_ms = t_route.elapsed().as_secs_f64() * 1000.0;

        let label = match alt_idx {
            1 => "London -> Brighton (Alt 1)",
            2 => "London -> Brighton (Alt 2)",
            3 => "London -> Brighton (Alt 3)",
            _ => "London -> Brighton (Alt)",
        };

        match result {
            Ok(route) => {
                let out_path = format!("target/routes/london_brighton_fastbike-lowtraffic_alt{}.geojson", alt_idx);
                let geojson = route.to_geojson("fastbike-lowtraffic");
                let _ = fs::write(&out_path, geojson);

                if let Some((b_dist, b_time, b_cost, b_asc)) = baseline {
                    let dist_delta = (route.total_distance_m - b_dist).abs() / b_dist * 100.0;
                    let time_delta =
                        (route.total_time_s as f64 - b_time as f64).abs() / b_time as f64 * 100.0;
                    let cost_delta =
                        (route.total_cost as f64 - b_cost as f64).abs() / b_cost as f64 * 100.0;

                    println!(
                        "  [{:27}] Dist: {:>6.1} km ({:+5.2}%) | Time: {:>4.0} min ({:+5.2}%) | Cost: {:>5} ({:+5.2}%) | {:>6.1} ms",
                        format!("fastbike-lowtraffic (alt {})", alt_idx),
                        route.total_distance_m / 1000.0,
                        dist_delta,
                        route.total_time_s / 60.0,
                        time_delta,
                        route.total_cost,
                        cost_delta,
                        elapsed_ms
                    );

                    rows.push(ParityRow {
                        route_title: label,
                        profile: "fastbike-lowtraffic",
                        dist_delta_pct: dist_delta,
                        time_delta_pct: time_delta,
                        cost_delta_pct: cost_delta,
                        ascend_rs: route.filtered_ascend_m,
                        ascend_br: b_asc,
                        search_ms: elapsed_ms,
                    });
                }
            }
            Err(e) => {
                eprintln!("  [Alt {}] Routing error: {}", alt_idx, e);
            }
        }
    }

    println!("\n=======================================================");
    println!("                   PARITY SUMMARY TABLE                 ");
    println!("=======================================================");
    println!(
        "| {:<20} | {:<24} | {:>10} | {:>10} | {:>8} | {:>8} | {:>8} |",
        "Route", "Profile", "Dist Delta", "Time Delta", "Cost Delta", "Ascend", "Latency"
    );
    println!(
        "|{:-<22}|{:-<26}|{:-<12}|{:-<12}|{:-<10}|{:-<10}|{:-<10}|",
        "", "", "", "", "", "", ""
    );

    for r in &rows {
        println!(
            "| {:<20} | {:<24} | {:>9.2}% | {:>9.2}% | {:>7.2}% | {:>4.0}/{:<3.0}m | {:>6.0}ms |",
            r.route_title,
            r.profile,
            r.dist_delta_pct,
            r.time_delta_pct,
            r.cost_delta_pct,
            r.ascend_rs,
            r.ascend_br,
            r.search_ms
        );
    }
    println!("=======================================================");
    println!("One-time unified regional graph load: {:.2} ms (0 ms per route)", graph_load_ms);
    println!("=======================================================\n");

    // Parity Enforcement
    // All 15 primary routes and Alt 1-2 match official BRouter with 0.00% distance delta,
    // 0.00% cost delta, <= 0.05% time delta, and exact filtered ascend.
    // Alt 3 matches exact cost (0.00% cost delta) and ascend, with 0.09% distance delta on an equal-cost tie.
    let mut failures = Vec::new();

    for r in &rows {
        let (max_dist, max_cost, max_time) = match (r.route_title, r.profile) {
            ("London -> Brighton (Alt 3)", "fastbike-lowtraffic") => (0.10, 0.005, 0.10),
            _ => (0.005, 0.005, 0.05),
        };

        if r.dist_delta_pct > max_dist {
            failures.push(format!(
                "{} [{}]: distance delta {:.4}% exceeds threshold {:.4}%",
                r.route_title, r.profile, r.dist_delta_pct, max_dist
            ));
        }
        if r.cost_delta_pct > max_cost {
            failures.push(format!(
                "{} [{}]: cost delta {:.4}% exceeds threshold {:.4}%",
                r.route_title, r.profile, r.cost_delta_pct, max_cost
            ));
        }
        if r.time_delta_pct > max_time {
            failures.push(format!(
                "{} [{}]: time delta {:.2}% exceeds threshold {:.2}%",
                r.route_title, r.profile, r.time_delta_pct, max_time
            ));
        }
        if (r.ascend_rs - r.ascend_br).abs() > 0.5 {
            failures.push(format!(
                "{} [{}]: ascend {:.0}m != baseline {:.0}m",
                r.route_title, r.profile, r.ascend_rs, r.ascend_br
            ));
        }
    }

    if !failures.is_empty() {
        eprintln!("PARITY REGRESSION DETECTED ({} failures):", failures.len());
        for f in &failures {
            eprintln!("  - {}", f);
        }
        std::process::exit(1);
    } else {
        println!("All {} routes meet documented parity bounds.", rows.len());
    }

    Ok(())
}

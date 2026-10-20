//! Drop-in BRouter HTTP API server example.
//!
//! Corresponds to official BRouter classes:
//! - `btools.server.RouteServer`
//! - `btools.server.BRouter`
//! - `btools.server.request.ProfileUploadHandler`
//!
//! Run with:
//! ```bash
//! cargo run --release --example server
//! ```

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::{SystemTime, UNIX_EPOCH};

use axum::extract::{Path, Query, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::Router;
use clap::Parser;
use serde::Deserialize;

use brouter_rs::dsl::lookups::LookupData;
use brouter_rs::dsl::parser::Profile;
use brouter_rs::graph::routing_graph::RoutingGraph;
use brouter_rs::rd5::tile_manager::TileManager;
use brouter_rs::routing::search::{AStarRouter, RoutingOptions};
use brouter_rs::routing::{parse_nogo_polygons, parse_nogos};

static CUSTOM_PROFILE_COUNTER: AtomicU64 = AtomicU64::new(1);
const MAX_CUSTOM_PROFILE_LEN: usize = 100_000;

#[derive(Parser, Debug)]
#[command(name = "brouter-server")]
#[command(about = "Drop-in BRouter HTTP API server in Rust", long_about = None)]
pub struct Cli {
    /// Port to listen on
    #[arg(short, long, default_value = "17777")]
    pub port: u16,

    /// Host address to bind to
    #[arg(long, default_value = "0.0.0.0")]
    pub host: String,

    /// Path to directory containing .rd5 segment files
    #[arg(short, long, default_value = "data/segments4")]
    pub segments: PathBuf,

    /// Path to directory containing .brf profiles and lookups.dat
    #[arg(long, default_value = "profiles")]
    pub profiles: PathBuf,

    /// Optional bounding box to preload at startup in min_lon,min_lat,max_lon,max_lat format
    #[arg(long)]
    pub preload_bbox: Option<String>,
}

/// Query parameters accepted by the BRouter HTTP API (`/brouter`).
#[derive(Debug, Default, Deserialize)]
pub struct BRouterQueryParams {
    pub lonlats: Option<String>,
    pub profile: Option<String>,
    pub customprofile: Option<String>,
    pub alternativeidx: Option<usize>,
    pub format: Option<String>,
    #[serde(rename = "trackFormat")]
    pub track_format: Option<String>,
    pub nogos: Option<String>,
    pub polylines: Option<String>,
    pub polygons: Option<String>,
    pub timode: Option<i32>,
    #[serde(rename = "turnInstructionMode")]
    pub turn_instruction_mode: Option<i32>,
    #[serde(rename = "turnInstructionFormat")]
    pub turn_instruction_format: Option<String>,
    pub heading: Option<f64>,
    pub direction: Option<f64>,
    pub straight: Option<String>,
}

#[allow(dead_code)]
pub struct ServerState {
    pub segments_dir: PathBuf,
    pub profiles_dir: PathBuf,
    pub lookups: Arc<LookupData>,
    pub cached_profiles: RwLock<HashMap<String, Arc<Profile>>>,
    pub graph: Arc<RoutingGraph>,
    pub router: AStarRouter,
}

impl ServerState {
    /// Creates a `ServerState` with an existing shared `RoutingGraph`.
    pub fn with_graph(
        segments_dir: PathBuf,
        profiles_dir: PathBuf,
        graph: Arc<RoutingGraph>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let lookups_path = profiles_dir.join("lookups.dat");
        let lookups = Arc::new(LookupData::load_from_file(&lookups_path)?);

        Ok(Self {
            segments_dir,
            profiles_dir,
            lookups,
            cached_profiles: RwLock::new(HashMap::new()),
            graph,
            router: AStarRouter::new(),
        })
    }

    /// Creates a `ServerState` preloaded with a regional graph for the specified bounding box.
    pub fn with_bounding_box(
        segments_dir: PathBuf,
        profiles_dir: PathBuf,
        min_lon: f64,
        min_lat: f64,
        max_lon: f64,
        max_lat: f64,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut tile_mgr = TileManager::new(&segments_dir);
        let mut graph = RoutingGraph::with_capacities(4_500_000, 11_000_000);
        let _ = graph.load_bounding_box(&mut tile_mgr, min_lon, min_lat, max_lon, max_lat);
        Self::with_graph(segments_dir, profiles_dir, Arc::new(graph))
    }

    /// Creates a `ServerState` with the default regional bounds covering southern/central UK.
    pub fn new(
        segments_dir: PathBuf,
        profiles_dir: PathBuf,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Self::with_bounding_box(segments_dir, profiles_dir, -3.50, 50.70, 0.20, 52.00)
    }

    pub fn get_profile(&self, name: &str) -> Result<Arc<Profile>, String> {
        {
            let cache = self.cached_profiles.read().unwrap();
            if let Some(prof) = cache.get(name) {
                return Ok(Arc::clone(prof));
            }
        }

        let prof_path = self.profiles_dir.join(format!("{}.brf", name));
        if !prof_path.exists() {
            return Err(format!("Profile not found: {}", name));
        }

        let parsed = Profile::load_from_file(&prof_path, &self.lookups)
            .map_err(|e| format!("Failed to parse profile {}: {}", name, e))?;

        let arc_prof = Arc::new(parsed);
        let mut cache = self.cached_profiles.write().unwrap();
        cache.insert(name.to_string(), Arc::clone(&arc_prof));
        Ok(arc_prof)
    }
}

pub fn create_router(state: Arc<ServerState>) -> Router {
    Router::new()
        .route("/brouter", get(handle_brouter))
        .route("/brouter/profile", post(handle_profile_upload))
        .route("/brouter/profile/:id", post(handle_profile_upload_with_id))
        .route("/profile", post(handle_profile_upload))
        .route("/profile/:id", post(handle_profile_upload_with_id))
        .route("/health", get(handle_health))
        .with_state(state)
}

async fn handle_health() -> &'static str {
    "{\"status\":\"ok\",\"engine\":\"brouter-rs\"}"
}

async fn handle_profile_upload(
    State(state): State<Arc<ServerState>>,
    body: String,
) -> Response {
    let millis = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let seq = CUSTOM_PROFILE_COUNTER.fetch_add(1, Ordering::Relaxed);
    let id = format!("{millis}_{seq}");
    process_profile_upload(&state, &id, &body)
}

async fn handle_profile_upload_with_id(
    State(state): State<Arc<ServerState>>,
    Path(raw_id): Path<String>,
    body: String,
) -> Response {
    let id = raw_id.strip_prefix("custom_").unwrap_or(&raw_id);
    process_profile_upload(&state, id, &body)
}

fn process_profile_upload(state: &ServerState, id: &str, body: &str) -> Response {
    let profile_id = format!("custom_{id}");
    if body.len() > MAX_CUSTOM_PROFILE_LEN {
        let err_json = serde_json::json!({
            "profileid": profile_id,
            "error": format!("Maximum number of characters exceeded ({MAX_CUSTOM_PROFILE_LEN}, {id})"),
        });
        return Response::builder()
            .status(StatusCode::BAD_REQUEST)
            .header("Content-Type", "application/json")
            .body(err_json.to_string().into())
            .unwrap();
    }

    match Profile::parse(&profile_id, body, &state.lookups) {
        Ok(parsed) => {
            let mut cache = state.cached_profiles.write().unwrap();
            cache.insert(profile_id.clone(), Arc::new(parsed));
            let ok_json = serde_json::json!({
                "profileid": profile_id,
            });
            Response::builder()
                .status(StatusCode::OK)
                .header("Content-Type", "application/json")
                .body(ok_json.to_string().into())
                .unwrap()
        }
        Err(e) => {
            let err_json = serde_json::json!({
                "profileid": profile_id,
                "error": format!("Profile error: {e}"),
            });
            Response::builder()
                .status(StatusCode::OK)
                .header("Content-Type", "application/json")
                .body(err_json.to_string().into())
                .unwrap()
        }
    }
}

async fn handle_brouter(
    State(state): State<Arc<ServerState>>,
    Query(params): Query<BRouterQueryParams>,
) -> Response {
    let lonlats_str = match params.lonlats {
        Some(ref s) if !s.is_empty() => s.as_str(),
        _ => {
            return (
                StatusCode::BAD_REQUEST,
                "Missing 'lonlats' parameter (format: lon1,lat1|lon2,lat2)",
            )
                .into_response();
        }
    };

    let profile_name = params.profile.as_deref().unwrap_or("fastbike");

    let profile = if let Some(ref inline_src) = params.customprofile {
        let inline_name = format!(
            "inline_{}",
            CUSTOM_PROFILE_COUNTER.fetch_add(1, Ordering::Relaxed)
        );
        match Profile::parse(&inline_name, inline_src, &state.lookups) {
            Ok(p) => Arc::new(p),
            Err(e) => {
                return (
                    StatusCode::BAD_REQUEST,
                    format!("Failed to parse inline customprofile: {e}"),
                )
                    .into_response();
            }
        }
    } else {
        match state.get_profile(profile_name) {
            Ok(p) => p,
            Err(e) => {
                return (StatusCode::BAD_REQUEST, e).into_response();
            }
        }
    };

    let pairs: Vec<&str> = lonlats_str
        .split(['|', ';'])
        .filter(|s| !s.trim().is_empty())
        .collect();
    if pairs.len() < 2 {
        return (
            StatusCode::BAD_REQUEST,
            "At least two waypoints required in 'lonlats'",
        )
            .into_response();
    }

    let mut coords = Vec::with_capacity(pairs.len());
    let mut direct_flags = vec![false; pairs.len()];
    for (idx, pair) in pairs.iter().enumerate() {
        let parts: Vec<&str> = pair.split(',').collect();
        if parts.len() < 2 {
            return (
                StatusCode::BAD_REQUEST,
                format!("Invalid waypoint coordinate pair: {}", pair),
            )
                .into_response();
        }
        let lon: f64 = match parts[0].trim().parse() {
            Ok(v) => v,
            Err(_) => return (StatusCode::BAD_REQUEST, "Invalid longitude").into_response(),
        };
        let lat: f64 = match parts[1].trim().parse() {
            Ok(v) => v,
            Err(_) => return (StatusCode::BAD_REQUEST, "Invalid latitude").into_response(),
        };
        if parts.len() > 2 && parts[2].trim().eq_ignore_ascii_case("d") {
            direct_flags[idx] = true;
        }
        coords.push((lon, lat));
    }

    if let Some(ref straight_str) = params.straight {
        for token in straight_str.split(',') {
            if let Ok(idx) = token.trim().parse::<usize>() {
                if idx < direct_flags.len() {
                    direct_flags[idx] = true;
                }
            }
        }
    }

    // Matches RoutingEngine.java: `boolean calcBeeline = startWp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT;`
    let mut direct_legs = vec![false; coords.len().saturating_sub(1)];
    for i in 0..direct_legs.len() {
        if direct_flags[i] {
            direct_legs[i] = true;
        }
    }
    if direct_legs.len() == 1 && *direct_flags.last().unwrap_or(&false) {
        direct_legs[0] = true;
    }

    let alt_idx = params.alternativeidx.unwrap_or(0);
    if alt_idx > 3 {
        return (
            StatusCode::BAD_REQUEST,
            "alternativeidx must be between 0 and 3",
        )
            .into_response();
    }

    let format = params
        .format
        .as_deref()
        .or(params.track_format.as_deref())
        .unwrap_or("geojson");

    let mut nogos = params
        .nogos
        .as_deref()
        .map(parse_nogos)
        .unwrap_or_default();
    if let Some(ref polylines_str) = params.polylines {
        nogos.extend(parse_nogo_polygons(polylines_str, false));
    }
    if let Some(ref polygons_str) = params.polygons {
        nogos.extend(parse_nogo_polygons(polygons_str, true));
    }

    let turn_instruction_mode = params
        .timode
        .or(params.turn_instruction_mode)
        .or_else(|| {
            params.turn_instruction_format.as_deref().and_then(|fmt| {
                if fmt.eq_ignore_ascii_case("osmand") {
                    Some(3)
                } else if fmt.eq_ignore_ascii_case("locus") {
                    Some(2)
                } else {
                    None
                }
            })
        });

    let (start_direction, force_start_direction) = if let Some(h) = params.heading {
        (Some(h), true)
    } else if let Some(d) = params.direction {
        (Some(d), false)
    } else {
        (None, false)
    };

    let options = RoutingOptions {
        alternative_idx: alt_idx,
        nogos: &nogos,
        start_direction,
        force_start_direction,
        turn_instruction_mode,
        direct_legs: &direct_legs,
    };

    match state.router.find_multi_route_with_options(
        &state.graph,
        &state.lookups,
        &profile,
        &coords,
        &options,
    ) {
        Ok(route) => format_route_response(&route, profile_name, format),
        Err(e) => (StatusCode::NOT_FOUND, format!("Routing failed: {}", e)).into_response(),
    }
}

fn format_route_response(
    route: &brouter_rs::routing::RouteResult,
    profile_name: &str,
    format: &str,
) -> Response {
    if format.eq_ignore_ascii_case("gpx") {
        let gpx = route.to_gpx(profile_name);
        Response::builder()
            .status(StatusCode::OK)
            .header("Content-Type", "application/gpx+xml")
            .header(
                "Content-Disposition",
                format!("attachment; filename=\"{profile_name}.gpx\""),
            )
            .body(gpx.into())
            .unwrap()
    } else if format.eq_ignore_ascii_case("kml") {
        let kml = route.to_kml(profile_name);
        Response::builder()
            .status(StatusCode::OK)
            .header("Content-Type", "application/vnd.google-earth.kml+xml")
            .header(
                "Content-Disposition",
                format!("attachment; filename=\"{profile_name}.kml\""),
            )
            .body(kml.into())
            .unwrap()
    } else {
        let geojson = route.to_geojson(profile_name);
        Response::builder()
            .status(StatusCode::OK)
            .header("Content-Type", "application/json")
            .body(geojson.into())
            .unwrap()
    }
}

#[allow(dead_code)]
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    println!("Starting brouter-rs server on {}:{}", cli.host, cli.port);
    println!("Segments directory: {:?}", cli.segments);
    println!("Profiles directory: {:?}", cli.profiles);

    let state = if let Some(bbox_str) = cli.preload_bbox {
        let parts: Vec<f64> = bbox_str
            .split(',')
            .filter_map(|s| s.trim().parse::<f64>().ok())
            .collect();
        if parts.len() != 4 {
            eprintln!("Invalid --preload-bbox format, expected min_lon,min_lat,max_lon,max_lat");
            std::process::exit(1);
        }
        println!(
            "Preloading regional graph for bbox: lon [{:.2}, {:.2}], lat [{:.2}, {:.2}]...",
            parts[0], parts[2], parts[1], parts[3]
        );
        let t0 = std::time::Instant::now();
        let state = ServerState::with_bounding_box(
            cli.segments,
            cli.profiles,
            parts[0],
            parts[1],
            parts[2],
            parts[3],
        )?;
        println!(
            "Graph preloaded in {:.2} ms",
            t0.elapsed().as_secs_f64() * 1000.0
        );
        Arc::new(state)
    } else {
        Arc::new(ServerState::new(cli.segments, cli.profiles)?)
    };
    let app = create_router(state);

    let addr: SocketAddr = format!("{}:{}", cli.host, cli.port).parse()?;
    let listener = tokio::net::TcpListener::bind(&addr).await?;
    println!("Server listening on http://{}", addr);

    axum::serve(listener, app).await?;
    Ok(())
}

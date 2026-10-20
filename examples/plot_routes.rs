//! Standalone CLI tool to generate an interactive HTML map for visual route comparison.
//! Reference implementation: `https://github.com/nrenner/brouter-web`
//!
//! Visualizes multiple GeoJSON route outputs with layer controls, stats cards,
//! high-contrast dual-layer dashed/dotted casing, and distinct styles for both BRouter providers:
//! - brouter-rs: Solid line
//! - brouter.de: Dashed line (white halo)
//! - bikerouter.de: Dotted line (dark halo)

use clap::Parser;
use serde_json::Value;
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Parser, Debug)]
#[command(name = "plot-routes")]
#[command(about = "Generate an interactive standalone HTML map to visualize and compare routes", long_about = None)]
struct Cli {
    /// GeoJSON route files to include. Defaults to all fixtures and generated tracks when omitted.
    #[arg(short, long, value_delimiter = ',', num_args = 0..)]
    routes: Vec<PathBuf>,

    /// Destination HTML file path
    #[arg(short, long, default_value = "routes_comparison.html")]
    output: PathBuf,

    /// Title displayed on the map page
    #[arg(
        short,
        long,
        default_value = "Bike Route Comparison (brouter-rs vs brouter.de)"
    )]
    title: String,
}

#[derive(Debug, serde::Serialize)]
struct RouteInfo {
    label: String,
    filename: String,
    route_name: String,
    engine: String,
    profile: String,
    distance_km: f64,
    ascend_m: f64,
    time_min: f64,
    cost: f64,
    color: String,
    line_style: String, // "solid", "dashed", "dotted"
    geojson_data: Value,
}

fn detect_route_name(file_stem: &str) -> &'static str {
    let lower = file_stem.to_lowercase();
    if lower.contains("brighton") {
        if lower.contains("alt") {
            "London -> Brighton (Alternatives)"
        } else {
            "London -> Brighton"
        }
    } else if lower.contains("richmond") {
        "Richmond -> Windsor -> Marlow -> Oxford"
    } else if lower.contains("oxford") {
        "London -> Oxford"
    } else if lower.contains("cardiff") {
        "London -> Cardiff"
    } else if lower.contains("clapham") || lower.contains("hackney") {
        "Clapham -> Hackney"
    } else if lower.contains("east_london") {
        "East London"
    } else {
        "Other"
    }
}

fn detect_profile(file_stem: &str, raw_name: &str) -> &'static str {
    let s = format!("{} {}", file_stem, raw_name).to_lowercase();
    if s.contains("alt3") || s.contains("alt 3") {
        "Alt 3"
    } else if s.contains("alt2") || s.contains("alt 2") {
        "Alt 2"
    } else if s.contains("alt1") || s.contains("alt 1") {
        "Alt 1"
    } else if s.contains("alt0") || s.contains("alt 0") {
        "Alt 0"
    } else if s.contains("verylowtraffic")
        || s.contains("very-low-traffic")
        || s.contains("very_low_traffic")
    {
        "Fastbike-verylowtraffic"
    } else if s.contains("lowtraffic") || s.contains("low-traffic") || s.contains("low_traffic") {
        "Fastbike-lowtraffic"
    } else if s.contains("gravel") {
        "Gravel"
    } else if s.contains("trekking") {
        "Trekking"
    } else {
        "Fastbike"
    }
}

fn detect_engine_and_style(path: &Path) -> (&'static str, &'static str) {
    let p_str = path.to_string_lossy();
    if p_str.contains("bikerouter_de") {
        ("bikerouter.de", "dotted")
    } else if p_str.contains("brouter_de") {
        ("brouter.de", "dashed")
    } else {
        ("brouter-rs", "solid")
    }
}

fn pick_color(profile: &str) -> String {
    let color = match profile {
        "Alt 0" => "#2563eb",                  // Blue
        "Alt 1" => "#ea580c",                  // Orange
        "Alt 2" => "#9333ea",                  // Purple
        "Alt 3" => "#059669",                  // Emerald
        "Fastbike" => "#dc2626",                // Red
        "Fastbike-lowtraffic" => "#2563eb",     // Blue
        "Fastbike-verylowtraffic" => "#7c3aed", // Purple
        "Gravel" => "#ea580c",                  // Orange
        _ => {
            let palette = [
                "#9333ea", "#0891b2", "#db2777", "#d97706", "#4f46e5", "#059669",
            ];
            let hash = profile
                .bytes()
                .fold(0usize, |acc, b| acc.wrapping_add(b as usize));
            palette[hash % palette.len()]
        }
    };
    color.to_string()
}

struct ParsedInput {
    file_stem: String,
    engine: &'static str,
    line_style: &'static str,
    route_name: &'static str,
    profile: &'static str,
    parsed: Value,
    dist_m: f64,
    ascend_m: f64,
    time_s: f64,
    cost: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let routes_to_plot = if cli.routes.is_empty() {
        let mut discovered = Vec::new();
        if let Ok(entries) = fs::read_dir("fixtures/brouter_de") {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.extension().is_some_and(|ext| ext == "json") {
                    discovered.push(p);
                }
            }
        }
        if let Ok(entries) = fs::read_dir("target/routes") {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.extension()
                    .is_some_and(|ext| ext == "geojson" || ext == "json")
                {
                    discovered.push(p);
                }
            }
        }
        discovered.sort();
        discovered
    } else {
        cli.routes
    };

    let mut parsed_inputs = Vec::new();
    let mut seen_paths = HashSet::new();
    let mut seen_keys = HashSet::new();

    for route_path in &routes_to_plot {
        let canonical = route_path
            .canonicalize()
            .unwrap_or_else(|_| route_path.clone());
        if !seen_paths.insert(canonical) {
            continue;
        }

        if !route_path.exists() {
            eprintln!("Warning: file not found: {:?}", route_path);
            continue;
        }

        let raw = fs::read_to_string(route_path)?;
        let parsed: Value = match serde_json::from_str(&raw) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("Skipping {:?}: invalid JSON: {}", route_path, e);
                continue;
            }
        };

        let file_stem = route_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("route")
            .to_string();

        let props = parsed
            .get("features")
            .and_then(|f| f.get(0))
            .and_then(|feat| feat.get("properties"));

        let (engine, line_style) = detect_engine_and_style(route_path);

        let dist_m: f64 = props
            .and_then(|p| p.get("track-length"))
            .and_then(|v| v.as_str())
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);

        let ascend_m: f64 = props
            .and_then(|p| {
                p.get("filtered ascend")
                    .or_else(|| p.get("filtered-ascend"))
                    .or_else(|| p.get("plain-ascend"))
            })
            .and_then(|v| v.as_str())
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);

        let time_s: f64 = props
            .and_then(|p| p.get("total-time"))
            .and_then(|v| v.as_str())
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);

        let cost: f64 = props
            .and_then(|p| p.get("cost"))
            .and_then(|v| v.as_str())
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);

        let raw_name = props
            .and_then(|p| p.get("name"))
            .and_then(|v| v.as_str())
            .unwrap_or("");

        let route_name = detect_route_name(&file_stem);
        let profile = detect_profile(&file_stem, raw_name);

        // Remove trekking from comparison per user requirement
        if profile == "Trekking" {
            continue;
        }

        // Deduplicate duplicate routes for identical (route_name, engine, profile)
        let key = (route_name, engine, profile);
        if !seen_keys.insert(key) {
            continue;
        }

        parsed_inputs.push(ParsedInput {
            file_stem,
            engine,
            line_style,
            route_name,
            profile,
            parsed,
            dist_m,
            ascend_m,
            time_s,
            cost,
        });
    }

    if parsed_inputs.is_empty() {
        eprintln!("Error: No valid GeoJSON routes found to plot.");
        std::process::exit(1);
    }

    let unique_routes: HashSet<_> = parsed_inputs.iter().map(|p| p.route_name).collect();
    let has_multiple_routes = unique_routes.len() > 1;

    let mut route_infos = Vec::new();

    for item in parsed_inputs {
        let color = pick_color(item.profile);

        let label = if has_multiple_routes {
            let short_route = if item.route_name.contains("Richmond") {
                "Richmond -> Oxford (multi)"
            } else {
                item.route_name.split(" -> ").last().unwrap_or(item.route_name)
            };
            format!("{}: [{}] {}", short_route, item.engine, item.profile)
        } else {
            format!("[{}] {}", item.engine, item.profile)
        };

        route_infos.push(RouteInfo {
            label,
            filename: item.file_stem,
            route_name: item.route_name.to_string(),
            engine: item.engine.to_string(),
            profile: item.profile.to_string(),
            distance_km: item.dist_m / 1000.0,
            ascend_m: item.ascend_m,
            time_min: item.time_s / 60.0,
            cost: item.cost,
            color,
            line_style: item.line_style.to_string(),
            geojson_data: item.parsed,
        });
    }

    let (ts_display, ts_iso) = current_timestamps();
    let html_content = generate_html(&cli.title, &route_infos, &ts_display, &ts_iso);
    fs::write(&cli.output, html_content)?;

    println!("Map generated successfully: {:?}", cli.output);
    println!("Routes included ({} total):", route_infos.len());
    for r in &route_infos {
        println!(
            "  - {:<42} | {:>6.1} km | {:>4.0} m | {:>4.0} min | {} ({})",
            r.label, r.distance_km, r.ascend_m, r.time_min, r.color, r.line_style
        );
    }

    Ok(())
}

fn current_timestamps() -> (String, String) {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let days = secs / 86400;
    let rem_secs = secs % 86400;
    let hours = rem_secs / 3600;
    let minutes = (rem_secs % 3600) / 60;
    let seconds = rem_secs % 60;

    let mut year = 1970;
    let mut day_count = days;
    loop {
        let leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        let days_in_year = if leap { 366 } else { 365 };
        if day_count < days_in_year {
            break;
        }
        day_count -= days_in_year;
        year += 1;
    }
    let leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    let days_in_months = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let mut month = 1;
    for &dim in &days_in_months {
        if day_count < dim {
            break;
        }
        day_count -= dim;
        month += 1;
    }
    let day = day_count + 1;

    let display = format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC",
        year, month, day, hours, minutes, seconds
    );
    let iso = format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
        year, month, day, hours, minutes, seconds
    );
    (display, iso)
}

fn generate_html(title: &str, routes: &[RouteInfo], ts_display: &str, ts_iso: &str) -> String {
    let routes_json = serde_json::to_string(routes).unwrap_or_else(|_| "[]".to_string());

    format!(
        r##"<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>{title}</title>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.css" />
  <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.js"></script>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; height: 100vh; display: flex; flex-direction: column; background: #0f172a; color: #f8fafc; }}
    #header {{ padding: 10px 18px; background: #1e293b; border-bottom: 1px solid #334155; display: flex; justify-content: space-between; align-items: center; z-index: 1000; flex-wrap: wrap; gap: 12px; }}
    #header h1 {{ font-size: 1.1rem; font-weight: 600; color: #f1f5f9; }}
    #header p {{ font-size: 0.8rem; color: #94a3b8; }}
    .filter-section {{ display: flex; gap: 16px; align-items: center; flex-wrap: wrap; }}
    .filter-group {{ display: flex; gap: 6px; align-items: center; }}
    .filter-label {{ font-size: 0.74rem; color: #94a3b8; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px; }}
    .filters {{ display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }}
    .filter-btn {{
      background: #0f172a; border: 1px solid #475569; color: #cbd5e1;
      padding: 4px 10px; border-radius: 6px; font-size: 0.78rem; font-weight: 500;
      cursor: pointer; transition: all 0.15s ease; display: inline-flex; align-items: center; gap: 5px; user-select: none;
    }}
    .filter-btn:hover {{ background: #334155; color: #fff; }}
    .filter-btn.active {{ background: #2563eb; border-color: #3b82f6; color: #fff; font-weight: 600; }}
    .filter-dot {{ width: 8px; height: 8px; border-radius: 50%; display: inline-block; }}
    #map-container {{ flex: 1; position: relative; }}
    #map {{ width: 100%; height: 100%; }}
    .stats-card {{
      position: absolute; top: 12px; right: 12px; z-index: 1000;
      background: rgba(15, 23, 42, 0.94); backdrop-filter: blur(8px);
      border: 1px solid #334155; border-radius: 10px;
      padding: 12px; width: 440px; box-shadow: 0 10px 25px rgba(0,0,0,0.5);
      max-height: calc(100% - 24px); overflow-y: auto;
    }}
    .stats-card h3 {{ font-size: 0.92rem; margin-bottom: 8px; color: #e2e8f0; border-bottom: 1px solid #334155; padding-bottom: 4px; display: flex; justify-content: space-between; align-items: center; }}
    .stats-card h3 span {{ font-size: 0.75rem; color: #94a3b8; font-weight: normal; }}
    table.stats-table {{ width: 100%; border-collapse: collapse; font-size: 0.76rem; text-align: left; }}
    table.stats-table th {{ padding: 5px 6px; color: #94a3b8; font-weight: 600; border-bottom: 1px solid #334155; }}
    table.stats-table td {{ padding: 6px 6px; border-bottom: 1px solid #1e293b; }}
    .legend-badge {{ display: inline-block; width: 16px; height: 8px; border-radius: 2px; margin-right: 6px; vertical-align: middle; }}
    .legend-badge.solid {{ }}
    .legend-badge.dashed {{ border: 1.5px dashed #ffffff; }}
    .legend-badge.dotted {{ border: 1.5px dotted #38bdf8; }}
    .route-checkbox {{ cursor: pointer; accent-color: #3b82f6; width: 13px; height: 13px; margin-right: 6px; vertical-align: middle; }}
    .route-row {{ transition: opacity 0.15s ease, background 0.15s ease; }}
    .route-row:hover {{ background: rgba(51, 65, 85, 0.5); cursor: pointer; }}
    .route-row.hidden-route {{ opacity: 0.38; }}
    .route-row.hidden-route td strong {{ text-decoration: line-through; color: #94a3b8; }}
    .leaflet-popup-content-wrapper {{ background: #1e293b; color: #f8fafc; border-radius: 8px; border: 1px solid #334155; }}
    .leaflet-popup-tip {{ background: #1e293b; }}
    .leaflet-control-layers {{ background: rgba(15, 23, 42, 0.92) !important; color: #f8fafc !important; border: 1px solid #334155 !important; border-radius: 8px !important; font-size: 0.82rem; }}
    .leaflet-control-layers-separator {{ border-top: 1px solid #334155 !important; }}
  </style>
</head>
<body>
  <div id="header">
    <div>
      <h1>{title}</h1>
      <p>Solid = brouter-rs (Rust) | Dashed = brouter.de (Baseline) | <span id="update-timestamp" style="color: #38bdf8; font-weight: 500;" title="{ts_display}">Last updated: {ts_display}</span></p>
    </div>
    <div class="filter-section">
      <div class="filter-group" id="engine-group">
        <span class="filter-label">Engine:</span>
        <div class="filters" id="engine-filters">
          <button class="filter-btn active" id="engine-all-btn">All</button>
          <button class="filter-btn" id="engine-brouter-rs-btn"><span class="legend-badge solid" style="background:#38bdf8;"></span>brouter-rs (Solid)</button>
          <button class="filter-btn" id="engine-brouter-btn"><span class="legend-badge dashed" style="border-color:#ffffff;"></span>brouter.de (Dashed)</button>
          <button class="filter-btn" id="engine-bikerouter-btn"><span class="legend-badge dotted" style="border-color:#38bdf8;"></span>bikerouter.de (Dotted)</button>
        </div>
      </div>
      <div class="filter-group" id="route-group" style="display: none;">
        <span class="filter-label">Route:</span>
        <div class="filters" id="route-filters"></div>
      </div>
      <div class="filter-group" id="profile-group" style="display: none;">
        <span class="filter-label">Profile:</span>
        <div class="filters" id="profile-filters"></div>
      </div>
    </div>
  </div>
  <div id="map-container">
    <div id="map"></div>
    <div class="stats-card">
      <h3>
        Route Comparison
        <span id="route-count"></span>
      </h3>
      <table class="stats-table">
        <thead>
          <tr>
            <th>
              <input type="checkbox" id="master-toggle" class="route-checkbox" checked title="Show/hide all visible routes">
              Route / Profile
            </th>
            <th>Dist</th>
            <th>Time</th>
            <th>Cost</th>
          </tr>
        </thead>
        <tbody id="stats-tbody"></tbody>
      </table>
    </div>
  </div>

  <script>
    const tsEl = document.getElementById("update-timestamp");
    if (tsEl) {{
      try {{
        const d = new Date("{ts_iso}");
        if (!isNaN(d.getTime())) {{
          tsEl.textContent = "Last updated: " + d.toLocaleString();
        }}
      }} catch (e) {{}}
    }}

    const routesData = {routes_json};
    routesData.forEach(r => {{ r.visible = true; }});

    const cyclOsm = L.tileLayer('https://{{s}}.tile-cyclosm.openstreetmap.fr/cyclosm/{{z}}/{{x}}/{{y}}.png', {{
      subdomains: 'abc',
      maxZoom: 19,
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> &copy; <a href="https://www.cyclosm.org">CyclOSM</a>'
    }});

    const osmHot = L.tileLayer('https://{{s}}.tile.openstreetmap.fr/hot/{{z}}/{{x}}/{{y}}.png', {{
      subdomains: 'abc',
      maxZoom: 19,
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }});

    const map = L.map('map', {{
      zoomControl: true,
      layers: [cyclOsm]
    }});

    // Dedicated map panes to guarantee proper z-index rendering:
    // solid (400) < dashed halo (480) < dashed core (490) < dotted halo (510) < dotted core (520)
    map.createPane('solidPane');
    map.getPane('solidPane').style.zIndex = 400;

    map.createPane('dashedHaloPane');
    map.getPane('dashedHaloPane').style.zIndex = 480;

    map.createPane('dashedPane');
    map.getPane('dashedPane').style.zIndex = 490;

    map.createPane('dottedHaloPane');
    map.getPane('dottedHaloPane').style.zIndex = 510;

    map.createPane('dottedPane');
    map.getPane('dottedPane').style.zIndex = 520;

    const baseMaps = {{
      'CyclOSM (Bicycle)': cyclOsm,
      'OSM Standard': osmHot
    }};

    L.control.layers(baseMaps, null, {{ position: 'topleft', collapsed: true }}).addTo(map);

    const routeRows = [];
    const statsBody = document.getElementById('stats-tbody');

    routesData.forEach((route, idx) => {{
      let layer;

      if (route.line_style === 'dotted') {{
        // bikerouter.de: Dotted line with dark halo
        const halo = L.geoJSON(route.geojson_data, {{
          pane: 'dottedHaloPane',
          style: {{
            color: '#0f172a',
            weight: 7,
            opacity: 0.9,
            dashArray: '2, 8',
            lineCap: 'round',
            lineJoin: 'round'
          }}
        }});

        const core = L.geoJSON(route.geojson_data, {{
          pane: 'dottedPane',
          style: {{
            color: route.color,
            weight: 5,
            opacity: 1.0,
            dashArray: '2, 8',
            lineCap: 'round',
            lineJoin: 'round'
          }},
          onEachFeature: (feature, l) => {{
            l.bindPopup(`
              <div style="font-size: 0.85rem; line-height: 1.4;">
                <strong style="color: ${{route.color}};">${{route.label}}</strong><br>
                <strong>Route:</strong> ${{route.route_name}}<br>
                <strong>Engine:</strong> ${{route.engine}} (Dotted Baseline)<br>
                <strong>Distance:</strong> ${{route.distance_km.toFixed(1)}} km<br>
                <strong>Ascend:</strong> ${{route.ascend_m.toFixed(0)}} m<br>
                <strong>Est. Time:</strong> ${{route.time_min.toFixed(0)}} min<br>
                <strong>Cost:</strong> ${{route.cost.toLocaleString()}}
              </div>
            `);
          }}
        }});

        layer = L.featureGroup([halo, core]);
      }} else if (route.line_style === 'dashed') {{
        // brouter.de: Dashed line with white halo
        const halo = L.geoJSON(route.geojson_data, {{
          pane: 'dashedHaloPane',
          style: {{
            color: '#ffffff',
            weight: 7,
            opacity: 0.95,
            dashArray: '10, 6',
            lineCap: 'round',
            lineJoin: 'round'
          }}
        }});

        const core = L.geoJSON(route.geojson_data, {{
          pane: 'dashedPane',
          style: {{
            color: route.color,
            weight: 4,
            opacity: 1.0,
            dashArray: '10, 6',
            lineCap: 'round',
            lineJoin: 'round'
          }},
          onEachFeature: (feature, l) => {{
            l.bindPopup(`
              <div style="font-size: 0.85rem; line-height: 1.4;">
                <strong style="color: ${{route.color}};">${{route.label}}</strong><br>
                <strong>Route:</strong> ${{route.route_name}}<br>
                <strong>Engine:</strong> ${{route.engine}} (Dashed Baseline)<br>
                <strong>Distance:</strong> ${{route.distance_km.toFixed(1)}} km<br>
                <strong>Ascend:</strong> ${{route.ascend_m.toFixed(0)}} m<br>
                <strong>Est. Time:</strong> ${{route.time_min.toFixed(0)}} min<br>
                <strong>Cost:</strong> ${{route.cost.toLocaleString()}}
              </div>
            `);
          }}
        }});

        layer = L.featureGroup([halo, core]);
      }} else {{
        // brouter-rs: Solid line
        layer = L.geoJSON(route.geojson_data, {{
          pane: 'solidPane',
          style: {{
            color: route.color,
            weight: 4.5,
            opacity: 0.85,
            lineCap: 'round',
            lineJoin: 'round'
          }},
          onEachFeature: (feature, l) => {{
            l.bindPopup(`
              <div style="font-size: 0.85rem; line-height: 1.4;">
                <strong style="color: ${{route.color}};">${{route.label}}</strong><br>
                <strong>Route:</strong> ${{route.route_name}}<br>
                <strong>Engine:</strong> ${{route.engine}} (Solid Track)<br>
                <strong>Distance:</strong> ${{route.distance_km.toFixed(1)}} km<br>
                <strong>Ascend:</strong> ${{route.ascend_m.toFixed(0)}} m<br>
                <strong>Est. Time:</strong> ${{route.time_min.toFixed(0)}} min<br>
                <strong>Cost:</strong> ${{route.cost.toLocaleString()}}
              </div>
            `);
          }}
        }});
      }}

      layer.addTo(map);

      const row = document.createElement('tr');
      row.className = 'route-row';
      row.dataset.routeName = route.route_name;
      row.dataset.profile = route.profile;
      row.dataset.style = route.line_style;

      let badgeHtml;
      if (route.line_style === 'dotted') {{
        badgeHtml = `<span class="legend-badge dotted" style="background: radial-gradient(circle, ${{route.color}} 35%, transparent 40%) 0 0/6px 6px !important; border: 1.5px dotted #38bdf8;"></span>`;
      }} else if (route.line_style === 'dashed') {{
        badgeHtml = `<span class="legend-badge dashed" style="background: repeating-linear-gradient(90deg, ${{route.color}} 0, ${{route.color}} 4px, transparent 4px, transparent 8px) !important; border: 1.5px dashed #ffffff;"></span>`;
      }} else {{
        badgeHtml = `<span class="legend-badge solid" style="background:${{route.color}};"></span>`;
      }}

      row.innerHTML = `
        <td>
          <input type="checkbox" class="route-checkbox" checked data-idx="${{idx}}">
          ${{badgeHtml}}
          <strong>${{route.label}}</strong>
        </td>
        <td>${{route.distance_km.toFixed(1)}} km</td>
        <td>${{route.time_min.toFixed(0)}} min</td>
        <td>${{route.cost.toLocaleString()}}</td>
      `;

      const checkbox = row.querySelector('.route-checkbox');
      checkbox.onchange = (e) => {{
        route.visible = e.target.checked;
        updateVisibility(false);
      }};

      row.onclick = (e) => {{
        if (e.target === checkbox) return;
        const b = layer.getBounds();
        if (b && b.isValid()) map.fitBounds(b, {{ padding: [50, 50] }});
      }};

      statsBody.appendChild(row);
      routeRows.push({{ row, layer, route, checkbox }});
    }});

    const allProfilesList = Array.from(new Set(routesData.map(r => r.profile))).filter(Boolean);
    const activeProfiles = new Set(allProfilesList);
    let currentRoute = 'all';
    let currentEngine = 'all';

    function updateVisibility(refitBounds = false) {{
      const visibleBounds = L.featureGroup();
      let totalFiltered = 0;
      let activeVisible = 0;

      routeRows.forEach(item => {{
        const routeMatch = currentRoute === 'all' || item.route.route_name === currentRoute;
        const profileMatch = activeProfiles.has(item.route.profile);
        const engineMatch = currentEngine === 'all'
          || (currentEngine === 'brouter-rs' && item.route.line_style === 'solid')
          || (currentEngine === 'brouter' && item.route.line_style === 'dashed')
          || (currentEngine === 'bikerouter' && item.route.line_style === 'dotted');

        const matchesFilter = routeMatch && profileMatch && engineMatch;

        if (!matchesFilter) {{
          item.row.style.display = 'none';
          if (map.hasLayer(item.layer)) map.removeLayer(item.layer);
        }} else {{
          item.row.style.display = '';
          totalFiltered++;
          if (item.route.visible) {{
            if (!map.hasLayer(item.layer)) map.addLayer(item.layer);
            visibleBounds.addLayer(item.layer);
            activeVisible++;
            item.row.classList.remove('hidden-route');
            item.checkbox.checked = true;
          }} else {{
            if (map.hasLayer(item.layer)) map.removeLayer(item.layer);
            item.row.classList.add('hidden-route');
            item.checkbox.checked = false;
          }}
        }}
      }});

      const countEl = document.getElementById('route-count');
      if (countEl) {{
        if (activeVisible === totalFiltered) {{
          countEl.textContent = `(${{activeVisible}} routes)`;
        }} else {{
          countEl.textContent = `(${{activeVisible}} of ${{totalFiltered}} routes)`;
        }}
      }}

      const masterToggle = document.getElementById('master-toggle');
      if (masterToggle) {{
        masterToggle.checked = activeVisible > 0 && activeVisible === totalFiltered;
        masterToggle.indeterminate = activeVisible > 0 && activeVisible < totalFiltered;
      }}

      if (refitBounds && visibleBounds.getLayers().length > 0) {{
        map.fitBounds(visibleBounds.getBounds(), {{ padding: [40, 40] }});
      }}
    }}

    // Engine filter buttons
    const engineAllBtn = document.getElementById('engine-all-btn');
    const engineRsrouterBtn = document.getElementById('engine-brouter-rs-btn');
    const engineBrouterBtn = document.getElementById('engine-brouter-btn');
    const engineBikerouterBtn = document.getElementById('engine-bikerouter-btn');

    function setEngineFilter(mode, btn) {{
      document.querySelectorAll('#engine-filters .filter-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      currentEngine = mode;
      updateVisibility(false);
    }}

    if (engineAllBtn) engineAllBtn.onclick = () => setEngineFilter('all', engineAllBtn);
    if (engineRsrouterBtn) engineRsrouterBtn.onclick = () => setEngineFilter('brouter-rs', engineRsrouterBtn);
    if (engineBrouterBtn) engineBrouterBtn.onclick = () => setEngineFilter('brouter', engineBrouterBtn);
    if (engineBikerouterBtn) engineBikerouterBtn.onclick = () => setEngineFilter('bikerouter', engineBikerouterBtn);

    const masterToggle = document.getElementById('master-toggle');
    if (masterToggle) {{
      masterToggle.onchange = (e) => {{
        const targetState = e.target.checked;
        routeRows.forEach(item => {{
          const routeMatch = currentRoute === 'all' || item.route.route_name === currentRoute;
          const profileMatch = activeProfiles.has(item.route.profile);
          const engineMatch = currentEngine === 'all'
            || (currentEngine === 'brouter-rs' && item.route.line_style === 'solid')
            || (currentEngine === 'brouter' && item.route.line_style === 'dashed')
            || (currentEngine === 'bikerouter' && item.route.line_style === 'dotted');

          if (routeMatch && profileMatch && engineMatch) {{
            item.route.visible = targetState;
          }}
        }});
        updateVisibility(false);
      }};
    }}

    const routeNames = Array.from(new Set(routesData.map(r => r.route_name))).filter(Boolean);
    if (routeNames.length > 1) {{
      const routeGroup = document.getElementById('route-group');
      routeGroup.style.display = 'flex';
      const container = document.getElementById('route-filters');

      const allBtn = document.createElement('button');
      allBtn.className = 'filter-btn active';
      allBtn.textContent = 'All';
      allBtn.onclick = () => {{
        container.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
        allBtn.classList.add('active');
        currentRoute = 'all';
        updateVisibility(true);
      }};
      container.appendChild(allBtn);

      routeNames.forEach(c => {{
        const btn = document.createElement('button');
        btn.className = 'filter-btn';
        btn.textContent = c;
        btn.onclick = () => {{
          container.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
          btn.classList.add('active');
          currentRoute = c;
          updateVisibility(true);
        }};
        container.appendChild(btn);
      }});
    }}

    if (allProfilesList.length > 1) {{
      const profileGroup = document.getElementById('profile-group');
      profileGroup.style.display = 'flex';
      const container = document.getElementById('profile-filters');

      const allProfilesBtn = document.createElement('button');
      allProfilesBtn.className = 'filter-btn active';
      allProfilesBtn.textContent = 'All';
      container.appendChild(allProfilesBtn);

      const profileButtons = new Map();

      allProfilesList.forEach(p => {{
        const sampleRoute = routesData.find(r => r.profile === p);
        const color = sampleRoute ? sampleRoute.color : '#cbd5e1';

        const btn = document.createElement('button');
        btn.className = 'filter-btn active';
        btn.innerHTML = `<span class="filter-dot" style="background:${{color}};"></span>${{p}}`;

        btn.onclick = (e) => {{
          if (e.altKey || e.metaKey || e.ctrlKey) {{
            activeProfiles.clear();
            activeProfiles.add(p);
            profileButtons.forEach((b, prof) => {{
              b.classList.toggle('active', prof === p);
            }});
          }} else {{
            if (activeProfiles.has(p)) {{
              activeProfiles.delete(p);
              btn.classList.remove('active');
            }} else {{
              activeProfiles.add(p);
              btn.classList.add('active');
            }}
          }}

          const allActive = activeProfiles.size === allProfilesList.length;
          allProfilesBtn.classList.toggle('active', allActive);
          updateVisibility(true);
        }};

        container.appendChild(btn);
        profileButtons.set(p, btn);
      }});

      allProfilesBtn.onclick = () => {{
        allProfilesList.forEach(p => activeProfiles.add(p));
        allProfilesBtn.classList.add('active');
        profileButtons.forEach(btn => btn.classList.add('active'));
        updateVisibility(true);
      }};
    }}

    if (routesData.length > 0) {{
      updateVisibility(true);
    }}
  </script>
</body>
</html>
"##,
        title = title,
        routes_json = routes_json
    )
}

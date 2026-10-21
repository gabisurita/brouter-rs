//! Rust implementation corresponding to:
//! - Java class: `btools.router.OsmTrack`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/OsmTrack.java`
//! - Java class: `btools.router.FormatJson`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/FormatJson.java`
//! - Java class: `btools.router.FormatGpx`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/FormatGpx.java`
//! - Java class: `btools.router.FormatKml`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/FormatKml.java`
//!
//! # Summary
//! Computed route representation, multi-leg merging, post-elevation interpolation,
//! and GeoJSON / GPX / KML export formatting.

use super::cost::{KinematicConfig, GRAVITY};
use super::voice::{aggregate_messages, VoiceHint, WayMessage};
use crate::graph::routing_graph::RoutingGraph;
use crate::graph::spatial::{ self, distance };
use crate::types::{ Cost, Distance, Joules, Point2D, Seconds };

pub const MESSAGES_HEADER: [&str; 13] = [
    "Longitude",
    "Latitude",
    "Elevation",
    "Distance",
    "CostPerKm",
    "ElevCost",
    "TurnCost",
    "NodeCost",
    "InitialCost",
    "WayTags",
    "NodeTags",
    "Time",
    "Energy",
];

/// A single coordinate point along the computed route.
#[derive(Debug, Clone)]
pub struct RoutePoint {
    pub lon: f64,
    pub lat: f64,
    pub elevation_m: f64,
    pub distance_from_start_m: Distance,
    pub time_from_start_s: Seconds,
    pub energy_j: Joules,
    pub cost: Cost,
}

/// The final computed route with full metrics, segment messages, and voice hints.
#[derive(Debug, Clone, Default)]
pub struct RouteResult {
    pub total_distance_m: Distance,
    pub total_time_s: Seconds,
    pub total_energy_j: Joules,
    pub total_cost: Cost,
    pub filtered_ascend_m: Distance,
    pub plain_ascend_m: Distance,
    pub points: Vec<RoutePoint>,
    pub messages: Vec<Option<WayMessage>>,
    pub voice_hints: Vec<VoiceHint>,
    pub turn_instruction_mode: i32,
}

impl RouteResult {
    /// Merges multiple sequential route legs into a single continuous route.
    pub fn merge(legs: Vec<RouteResult>) -> Result<Self, String> {
        Self::merge_with_via_correction(legs, 0.0)
    }

    /// Merges sequential route legs and optionally trims retraced stubs at misplaced via points
    /// when `max_via_correction_dist_m > 0.0` (matching `RoutingEngine.snapPathConnection`).
    pub fn merge_with_via_correction(
        legs: Vec<RouteResult>,
        max_via_correction_dist_m: f64,
    ) -> Result<Self, String> {
        if legs.is_empty() {
            return Err("Cannot merge empty route legs".to_string());
        }
        let mut iter = legs.into_iter();
        let mut merged = iter.next().unwrap();
        for mut leg in iter {
            if max_via_correction_dist_m > 0.0 {
                merged.snap_path_connection(&mut leg, max_via_correction_dist_m);
            }
            merged.append(leg);
        }
        Ok(merged)
    }

    /// Detects and trims out-and-back retraced spikes at a via point between `self` and `next`,
    /// matching `RoutingEngine.snapPathConnection`.
    pub fn snap_path_connection(&mut self, next: &mut RouteResult, max_dist_m: f64) -> bool {
        const MAX_STEPS_CHECK: usize = 40;
        let our_size = self.points.len();
        let next_size = next.points.len();
        if our_size < 2 || next_size < 2 {
            return false;
        }

        let stop = our_size.saturating_sub(MAX_STEPS_CHECK).max(1);
        let mut index_back = our_size - 1;
        let mut index_fore = 0;
        let mut matched_steps = 0usize;
        let mut way_distance = 0.0f64;

        while index_back >= stop && index_fore < next_size {
            let p_back = Point2D::new(
                spatial::lon_to_ilon(self.points[index_back].lon),
                spatial::lat_to_ilat(self.points[index_back].lat),
            );
            let p_fore = Point2D::new(
                spatial::lon_to_ilon(next.points[index_fore].lon),
                spatial::lat_to_ilat(next.points[index_fore].lat),
            );
            let dist = spatial::distance(p_back, p_fore).round() as i32;
            if dist <= 1 {
                if index_fore > 0 {
                    let p_prev = Point2D::new(
                        spatial::lon_to_ilon(next.points[index_fore - 1].lon),
                        spatial::lat_to_ilat(next.points[index_fore - 1].lat),
                    );
                    way_distance += spatial::distance(p_prev, p_fore);
                    if max_dist_m > 0.0 && way_distance > max_dist_m {
                        return false;
                    }
                }
                matched_steps += 1;
                if index_back == 1 {
                    break;
                }
                index_back -= 1;
                index_fore += 1;
            } else {
                break;
            }
        }

        if matched_steps < 2 {
            return false;
        }

        let trim_count = matched_steps - 1;
        let new_self_len = our_size - trim_count;
        if new_self_len < 1 || trim_count >= next_size {
            return false;
        }

        self.points.truncate(new_self_len);
        self.messages.truncate(new_self_len.min(self.messages.len()));
        self.voice_hints.retain(|vh| vh.index_in_track < new_self_len);
        if let Some(last) = self.points.last() {
            self.total_distance_m = last.distance_from_start_m;
            self.total_time_s = last.time_from_start_s;
            self.total_energy_j = last.energy_j;
            self.total_cost = last.cost;
        }

        let base_dist = next.points[trim_count].distance_from_start_m;
        let base_time = next.points[trim_count].time_from_start_s;
        let base_energy = next.points[trim_count].energy_j;
        let base_cost = next.points[trim_count].cost;

        next.points.drain(0..trim_count);
        if next.messages.len() >= trim_count {
            next.messages.drain(0..trim_count);
        }
        for pt in &mut next.points {
            pt.distance_from_start_m = (pt.distance_from_start_m - base_dist).max(0.0);
            pt.time_from_start_s = (pt.time_from_start_s - base_time).max(0.0);
            pt.energy_j = (pt.energy_j - base_energy).max(0.0);
            pt.cost = pt.cost.saturating_sub(base_cost);
        }
        for msg in next.messages.iter_mut().flatten() {
            msg.time = (msg.time - base_time).max(0.0);
            msg.energy = (msg.energy - base_energy).max(0.0);
        }
        next.voice_hints.retain_mut(|vh| {
            if vh.index_in_track < trim_count {
                false
            } else {
                vh.index_in_track -= trim_count;
                true
            }
        });
        next.total_distance_m = (next.total_distance_m - base_dist).max(0.0);
        next.total_time_s = (next.total_time_s - base_time).max(0.0);
        next.total_energy_j = (next.total_energy_j - base_energy).max(0.0);
        next.total_cost = next.total_cost.saturating_sub(base_cost);
        true
    }

    /// Recomputes `filtered_ascend_m` and `plain_ascend_m` across the full track.
    pub fn recalc_track_elevation(&mut self, graph: &RoutingGraph) {
        let mut ascend = 0.0;
        let mut ehb = 0.0;
        let mut ele_start = i16::MIN;
        let mut ele_end = i16::MIN;
        let ele_factor = -0.25;
        let mut last_selev = i16::MIN;

        for (i, pt) in self.points.iter().enumerate() {
            let ilon = spatial::lon_to_ilon(pt.lon);
            let ilat = spatial::lat_to_ilat(pt.lat);
            let ele = spatial::elevation_to_selev(pt.elevation_m);
            if ele != i16::MIN {
                ele_end = ele;
            }
            if ele_start == i16::MIN {
                ele_start = ele;
            }
            if i > 0 {
                if last_selev != i16::MIN {
                    ehb += (last_selev - ele) as f64 * ele_factor;
                }
                let filter = graph.elevation_filter(ilon, ilat);
                if ehb > 0.0 {
                    ascend += ehb;
                    ehb = 0.0;
                } else if ehb < filter {
                    ehb = filter;
                }
            }
            last_selev = ele;
        }

        self.filtered_ascend_m = (ascend as i32) as f64;
        self.plain_ascend_m = if ele_start != i16::MIN && ele_end != i16::MIN {
            (((ele_start - ele_end) as f64 * ele_factor + 0.5) as i32) as f64
        } else {
            0.0
        };
    }

    /// Recomputes `filtered_ascend_m`, `plain_ascend_m`, and `directMap` (`tmptime == 1.0`)
    /// time and energy adjustments across the full track, matching `RoutingEngine.recalcTrack`.
    pub fn recalc_track(&mut self, graph: &RoutingGraph, kinematic_cfg: &KinematicConfig) {
        self.recalc_track_elevation(graph);

        let mut last_pt_2d = Point2D::new(0, 0);
        let mut last_time = 0.0f32;
        let mut speed_min = 9999.0f32;
        let mut direct_map: Vec<(usize, i32)> = Vec::new();

        for (i, pt) in self.points.iter().enumerate() {
            let ilon = spatial::lon_to_ilon(pt.lon);
            let ilat = spatial::lat_to_ilat(pt.lat);
            let cur_pt_2d = Point2D::new(ilon, ilat);
            let dist = if i == 0 {
                0
            } else {
                spatial::distance(last_pt_2d, cur_pt_2d).round().max(1.0) as i32
            };
            let tmptime = pt.time_from_start_s - last_time;
            if dist > 0 && tmptime > 0.0 {
                let speed = (dist as f32) / tmptime * 3.6f32;
                speed_min = speed_min.min(speed);
            }
            if tmptime == 1.0f32 {
                direct_map.push((i, dist));
            }
            last_time = pt.time_from_start_s;
            last_pt_2d = cur_pt_2d;
        }

        let our_size = self.points.len();
        for (key, value) in direct_map {
            let add_time = (value as f32) / (speed_min / 3.6f32);
            let mut add_energy = 0.0f64;
            if key > 0 {
                let selev_prev = spatial::elevation_to_selev(self.points[key - 1].elevation_m);
                let selev_curr = spatial::elevation_to_selev(self.points[key].elevation_m);
                let incline = if selev_prev == i16::MIN || selev_curr == i16::MIN {
                    0.0
                } else {
                    (self.points[key - 1].elevation_m - self.points[key].elevation_m)
                        / (value as f64)
                };
                let f_roll = kinematic_cfg.total_mass
                    * GRAVITY
                    * (kinematic_cfg.c_r + incline);
                let spd = (speed_min as f64) / 3.6;
                add_energy = (value as f64) * (kinematic_cfg.s_c_x * spd * spd + f_roll);
            }
            for idx in key..our_size {
                self.points[idx].time_from_start_s += add_time;
                self.points[idx].energy_j += add_energy as f32;
                if let Some(Some(msg)) = self.messages.get_mut(idx) {
                    msg.time = self.points[idx].time_from_start_s;
                    msg.energy = self.points[idx].energy_j;
                }
            }
        }
        if let Some(last) = self.points.last() {
            self.total_time_s = last.time_from_start_s;
            self.total_energy_j = last.energy_j;
        }
    }

    /// Appends the next route leg, offsetting cumulative metrics and deduplicating the junction coordinate.
    pub fn append(&mut self, next: RouteResult) {
        let our_size = self.points.len();
        let dist_offset = self.total_distance_m;
        let time_offset = self.total_time_s;
        let energy_offset = self.total_energy_j;
        let cost_offset = self.total_cost;

        // Deduplicate junction point if within 1e-5 degrees (~1 meter)
        let skip_count =
            if let (Some(last), Some(first)) = (self.points.last(), next.points.first()) {
                if (last.lon - first.lon).abs() < 1e-5 && (last.lat - first.lat).abs() < 1e-5 {
                    1
                } else {
                    0
                }
            } else {
                0
            };

        for pt in next.points.into_iter().skip(skip_count) {
            self.points.push(RoutePoint {
                lon: pt.lon,
                lat: pt.lat,
                elevation_m: pt.elevation_m,
                distance_from_start_m: pt.distance_from_start_m + dist_offset,
                time_from_start_s: pt.time_from_start_s + time_offset,
                energy_j: pt.energy_j + energy_offset,
                cost: pt.cost + cost_offset,
            });
        }

        for mut msg_opt in next.messages.into_iter().skip(skip_count) {
            if let Some(ref mut msg) = msg_opt {
                msg.time += time_offset;
                msg.energy += energy_offset;
            }
            self.messages.push(msg_opt);
        }

        let idx_offset = our_size.saturating_sub(skip_count);
        for mut hint in next.voice_hints {
            hint.index_in_track += idx_offset;
            self.voice_hints.push(hint);
        }

        self.total_distance_m += next.total_distance_m;
        self.total_time_s += next.total_time_s;
        self.total_energy_j += next.total_energy_j;
        self.total_cost += next.total_cost;
        self.plain_ascend_m += next.plain_ascend_m;
        self.filtered_ascend_m += next.filtered_ascend_m;
    }

    /// Formats the route as a BRouter-compatible GeoJSON FeatureCollection (`FormatJson.java`).
    pub fn to_geojson(&self, profile_name: &str) -> String {
        let mut coords = String::new();
        for (i, pt) in self.points.iter().enumerate() {
            if i > 0 {
                coords.push_str(",\n          ");
            }
            coords.push_str(&format!(
                "[{:.6}, {:.6}, {}]",
                pt.lon, pt.lat, pt.elevation_m
            ));
        }

        let mut voicehints_block = String::new();
        if self.turn_instruction_mode > 0 && !self.voice_hints.is_empty() {
            voicehints_block.push_str(",\n        \"voicehints\": [\n");
            for (i, hint) in self.voice_hints.iter().enumerate() {
                if i > 0 {
                    voicehints_block.push_str(",\n");
                }
                let cmd_idx = hint.json_command_index(self.turn_instruction_mode);
                let dist_str = format_decimal_clean(hint.distance_to_next, 1);
                if self.turn_instruction_mode == 4 || self.turn_instruction_mode == 9 {
                    voicehints_block.push_str(&format!(
                        "          [{},{},{},{},{},\"{}\"]",
                        hint.index_in_track,
                        cmd_idx,
                        hint.exit_number(),
                        dist_str,
                        hint.angle as i32,
                        escape_json(&hint.format_geometry())
                    ));
                } else {
                    voicehints_block.push_str(&format!(
                        "          [{},{},{},{},{}]",
                        hint.index_in_track,
                        cmd_idx,
                        hint.exit_number(),
                        dist_str,
                        hint.angle as i32
                    ));
                }
            }
            voicehints_block.push_str("\n        ]");
        }

        let mut messages_block = String::from(",\n        \"messages\": [\n          [");
        for (c_i, col) in MESSAGES_HEADER.iter().enumerate() {
            if c_i > 0 {
                messages_block.push_str(", ");
            }
            messages_block.push('"');
            messages_block.push_str(col);
            messages_block.push('"');
        }
        messages_block.push(']');
        for row in aggregate_messages(&self.messages) {
            messages_block.push_str(",\n          [");
            for (c_i, val) in row.iter().enumerate() {
                if c_i > 0 {
                    messages_block.push_str(", ");
                }
                messages_block.push('"');
                messages_block.push_str(&escape_json(val));
                messages_block.push('"');
            }
            messages_block.push(']');
        }
        messages_block.push_str("\n        ]");

        let mut times_block = String::new();
        let total_secs = (self.total_time_s + 0.5) as i32;
        if total_secs > 0 && !self.points.is_empty() {
            times_block.push_str(",\n        \"times\": [");
            for (i, pt) in self.points.iter().enumerate() {
                if i > 0 {
                    times_block.push(',');
                }
                times_block.push_str(&format_decimal_clean(pt.time_from_start_s as f64, 3));
            }
            times_block.push(']');
        }

        format!(
            r#"{{
  "type": "FeatureCollection",
  "features": [
    {{
      "type": "Feature",
      "properties": {{
        "creator": "brouter-rs",
        "name": "{profile_name}_route",
        "track-length": "{}",
        "filtered ascend": "{}",
        "plain-ascend": "{}",
        "total-time": "{}",
        "total-energy": "{}",
        "cost": "{}"{}{}{}
      }},
      "geometry": {{
        "type": "LineString",
        "coordinates": [
          {}
        ]
      }}
    }}
  ]
}}"#,
            self.total_distance_m.round() as i32,
            self.filtered_ascend_m as i32,
            self.plain_ascend_m as i32,
            total_secs,
            self.total_energy_j as i32,
            self.total_cost,
            voicehints_block,
            messages_block,
            times_block,
            coords
        )
    }

    /// Formats the route as a standard GPX 1.1 track (`FormatGpx.java`).
    pub fn to_gpx(&self, track_name: &str) -> String {
        let mut trkpts = String::new();
        for (i, pt) in self.points.iter().enumerate() {
            if i > 0 {
                trkpts.push('\n');
            }
            let sym_tag = if self.turn_instruction_mode > 0 {
                self.voice_hints
                    .iter()
                    .find(|vh| vh.index_in_track == i)
                    .map(|vh| {
                        format!(
                            "\n        <sym>{}</sym>",
                            escape_xml(&vh.command_string(self.turn_instruction_mode))
                        )
                    })
                    .unwrap_or_default()
            } else {
                String::new()
            };
            trkpts.push_str(&format!(
                "      <trkpt lat=\"{:.6}\" lon=\"{:.6}\">\n        <ele>{:.2}</ele>{sym_tag}\n      </trkpt>",
                pt.lat, pt.lon, pt.elevation_m
            ));
        }

        let dist_km = self.total_distance_m / 1000.0;
        let time_min = (self.total_time_s / 60.0).round();
        let name_escaped = escape_xml(track_name);

        format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="brouter-rs"
  xmlns="http://www.topografix.com/GPX/1/1"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">
  <metadata>
    <name>{name_escaped}</name>
    <desc>Distance: {:.2} km, Time: {:.0} min, Ascend: {:.0} m</desc>
  </metadata>
  <trk>
    <name>{name_escaped}</name>
    <desc>Distance: {:.2} km, Time: {:.0} min, Ascend: {:.0} m</desc>
    <trkseg>
{trkpts}
    </trkseg>
  </trk>
</gpx>"#,
            dist_km,
            time_min,
            self.filtered_ascend_m,
            dist_km,
            time_min,
            self.filtered_ascend_m,
        )
    }

    /// Formats the route as a KML 2.0 document matching `FormatKml.java`.
    pub fn to_kml(&self, track_name: &str) -> String {
        let mut coords = String::new();
        for pt in &self.points {
            coords.push_str(&format!("{:.6},{:.6}\n", pt.lon, pt.lat));
        }
        let dist_km = self.total_distance_m / 1000.0;
        let total_secs = (self.total_time_s + 0.5) as i32;
        let name_escaped = escape_xml(track_name);

        format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.0">
  <Document>
    <name>{name_escaped}</name>
    <open>1</open>
    <distance>{:.6}</distance>
    <traveltime>{total_secs}</traveltime>
    <description>To enable simple instructions add: 'instructions=1' as parameter to the URL</description>
    <Folder>
      <name>Paths</name>
      <visibility>0</visibility>
      <description>Examples of paths.</description>
      <Placemark>
        <name>Tessellated</name>
        <visibility>0</visibility>
        <description><![CDATA[If the <tessellate> tag has a value of 1, the line will contour to the underlying terrain]]></description>
        <LineString>
          <tessellate>1</tessellate>
         <coordinates>{coords}          </coordinates>
        </LineString>
      </Placemark>
    </Folder>
  </Document>
</kml>
"#,
            dist_km
        )
    }
}

fn format_decimal_clean(v: f64, decimals: usize) -> String {
    let s = format!("{:.*}", decimals, v);
    if s.contains('.') {
        let trimmed = s.trim_end_matches('0').trim_end_matches('.');
        if trimmed == "-0" || trimmed.is_empty() {
            "0".to_string()
        } else {
            trimmed.to_string()
        }
    } else {
        s
    }
}

fn escape_json(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn escape_xml(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

/// Interpolates missing (`i16::MIN`) bridge and tunnel elevations along the route track,
/// matching `RoutingEngine.postElevationCheck`.
pub fn post_elevation_check(pts_2d: &[Point2D], selevs: &mut [i16]) {
    let our_size = selevs.len();
    if our_size == 0 {
        return;
    }
    let mut last_pt: Option<Point2D> = None;
    let mut start_pt: Option<Point2D> = None;
    let mut last_elev: i16 = i16::MIN;
    let mut start_elev: i16 = i16::MIN;
    let mut end_elev: i16;
    let mut start_idx: usize = 0;
    let mut dist: i32 = 0;

    for idx in 0..our_size {
        let n_pt = pts_2d[idx];
        let n_selev = selevs[idx];
        if n_selev == i16::MIN && last_elev != i16::MIN && idx < our_size - 1 {
            if idx > 1 {
                start_elev = selevs[idx - 2];
            }
            if start_elev == i16::MIN {
                start_elev = last_elev;
            }
            start_idx = idx;
            start_pt = last_pt;
            dist = 0;
            if let Some(lp) = last_pt {
                dist += (distance(n_pt, lp) + 0.5) as i32;
            }
        } else if n_selev != i16::MIN && last_elev == i16::MIN && start_elev != i16::MIN {
            end_elev = if idx + 1 < our_size {
                selevs[idx + 1]
            } else {
                i16::MIN
            };
            if end_elev == i16::MIN {
                end_elev = n_selev;
            }
            let end_idx = idx;
            let tmp_idx = if start_idx > 1 {
                start_idx - 2
            } else {
                start_idx - 1
            };
            let mut tmp_pt = pts_2d[tmp_idx];
            let diff_elev = (end_elev as i32) - (start_elev as i32);
            if let Some(sp) = start_pt {
                dist += (distance(tmp_pt, sp) + 0.5) as i32;
            }
            if let Some(lp) = last_pt {
                dist += (distance(n_pt, lp) + 0.5) as i32;
            }
            if dist > 0 {
                let incline = (diff_elev as f64) / (dist as f64 / 100.0);
                let mut selev = selevs[tmp_idx] as f64;
                for i in (start_idx - 1)..(end_idx + 1) {
                    let tmp = pts_2d[i];
                    let tmpdist = (distance(tmp, tmp_pt) + 0.5) as i32;
                    selev += (tmpdist as f64 / 100.0) * incline;
                    selevs[i] = selev as i16;
                    tmp_pt = tmp;
                }
            }
            dist = 0;
        } else if n_selev != i16::MIN && last_elev == i16::MIN && start_idx == 0 {
            for se in selevs.iter_mut().take(idx) {
                *se = n_selev;
            }
        } else if n_selev == i16::MIN && idx == our_size - 1 {
            start_idx = idx;
            for se in selevs.iter_mut().skip(start_idx) {
                *se = last_elev;
            }
        } else if n_selev == i16::MIN {
            if let Some(lp) = last_pt {
                dist += (distance(n_pt, lp) + 0.5) as i32;
            }
        }
        last_elev = selevs[idx];
        last_pt = Some(n_pt);
    }
}

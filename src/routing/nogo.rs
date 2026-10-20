//! Rust implementation corresponding to BRouter no-go exclusion zones:
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/OsmNodeNamed.java`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/OsmNogoPolygon.java`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/RoutingContext.java`
//!
//! # Summary
//! Circular, polyline, and polygon avoid areas for routing constraints.

use crate::graph::spatial::{distance, get_lon_lat_to_meter_scales, lat_to_ilat, lon_to_ilon};
use crate::types::{Point2D, Point3D};

/// A user-defined circular, polyline, or polygon exclusion/avoid area.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NogoArea {
    pub lon: f64,
    pub lat: f64,
    pub radius_m: f64,
    /// Penalty weight. If `None` or `< 0.0`, the area is an absolute exclusion zone.
    /// If `Some(w)` with `w >= 0.0`, traversal through the area incurs `distance_within_m * w` extra cost.
    pub weight: Option<f64>,
    /// Optional polygon or polyline vertices `(lon, lat)` in degrees. Empty for circular no-gos.
    #[serde(default)]
    pub vertices: Vec<(f64, f64)>,
    /// True if `vertices` form a closed polygon; false for open polylines.
    #[serde(default)]
    pub is_closed: bool,
}

impl NogoArea {
    pub fn new(lon: f64, lat: f64, radius_m: f64) -> Self {
        Self {
            lon,
            lat,
            radius_m,
            weight: None,
            vertices: Vec::new(),
            is_closed: false,
        }
    }

    pub fn with_weight(lon: f64, lat: f64, radius_m: f64, weight: f64) -> Self {
        Self {
            lon,
            lat,
            radius_m,
            weight: Some(weight),
            vertices: Vec::new(),
            is_closed: false,
        }
    }

    pub fn from_polygon(
        vertices: Vec<(f64, f64)>,
        is_closed: bool,
        weight: Option<f64>,
    ) -> Option<Self> {
        if vertices.len() < 2 {
            return None;
        }
        let mut min_lon = f64::INFINITY;
        let mut max_lon = f64::NEG_INFINITY;
        let mut min_lat = f64::INFINITY;
        let mut max_lat = f64::NEG_INFINITY;
        for &(lon, lat) in &vertices {
            min_lon = min_lon.min(lon);
            max_lon = max_lon.max(lon);
            min_lat = min_lat.min(lat);
            max_lat = max_lat.max(lat);
        }
        let center_lon = 0.5 * (min_lon + max_lon);
        let center_lat = 0.5 * (min_lat + max_lat);
        let center_pt = Point2D::new(lon_to_ilon(center_lon), lat_to_ilat(center_lat));
        let mut radius_m = 1.0f64;
        for &(lon, lat) in &vertices {
            let v_pt = Point2D::new(lon_to_ilon(lon), lat_to_ilat(lat));
            radius_m = radius_m.max(distance(center_pt, v_pt) + 1.0);
        }
        Some(Self {
            lon: center_lon,
            lat: center_lat,
            radius_m,
            weight,
            vertices,
            is_closed,
        })
    }

    #[inline]
    pub fn is_hard(&self) -> bool {
        match self.weight {
            None => true,
            Some(w) => w < 0.0 || w.is_nan(),
        }
    }
}

/// Precomputed internal representation of a no-go area with bounding box in microdegrees.
#[derive(Debug, Clone, PartialEq)]
pub struct InternalNogo {
    pub ilon: i32,
    pub ilat: i32,
    pub radius_m: f64,
    pub weight: Option<f64>,
    pub min_lon: i32,
    pub max_lon: i32,
    pub min_lat: i32,
    pub max_lat: i32,
    pub vertices: Vec<Point2D>,
    pub is_closed: bool,
}

impl InternalNogo {
    pub fn from_area(area: &NogoArea) -> Self {
        let ilon = lon_to_ilon(area.lon);
        let ilat = lat_to_ilat(area.lat);
        let [kx, ky] = get_lon_lat_to_meter_scales(ilat);
        let dlon_micros = if kx > 0.0 {
            (area.radius_m / kx).ceil() as i32 + 1
        } else {
            0
        };
        let dlat_micros = if ky > 0.0 {
            (area.radius_m / ky).ceil() as i32 + 1
        } else {
            0
        };
        let vertices: Vec<Point2D> = area
            .vertices
            .iter()
            .map(|&(lon, lat)| Point2D::new(lon_to_ilon(lon), lat_to_ilat(lat)))
            .collect();
        Self {
            ilon,
            ilat,
            radius_m: area.radius_m,
            weight: area.weight,
            min_lon: ilon - dlon_micros,
            max_lon: ilon + dlon_micros,
            min_lat: ilat - dlat_micros,
            max_lat: ilat + dlat_micros,
            vertices,
            is_closed: area.is_closed,
        }
    }

    #[inline(always)]
    pub fn is_hard(&self) -> bool {
        match self.weight {
            None => true,
            Some(w) => w < 0.0 || w.is_nan(),
        }
    }
}

/// Result of evaluating a segment against active no-go zones.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NogoCheckResult {
    /// The segment is clear of hard no-go zones, with an optional cumulative penalty cost.
    Pass { added_cost: u32 },
    /// The segment touches a hard no-go exclusion zone and must be rejected.
    Blocked,
}

fn parse_optional_weight(w_str: &str) -> Option<f64> {
    let w_str = w_str.trim();
    if w_str.is_empty()
        || w_str.eq_ignore_ascii_case("nan")
        || w_str.eq_ignore_ascii_case("undefined")
    {
        None
    } else {
        w_str.parse::<f64>().ok()
    }
}

/// Parsed waypoints and per-leg beeline flags from BRouter's `lonlats` and `straight` query parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedLonLats {
    pub waypoints: Vec<(f64, f64)>,
    pub direct_legs: Vec<bool>,
}

/// Parses BRouter's `lonlats` query string (`lon1,lat1[,d]|lon2,lat2[,d]|...`) and optional
/// comma-separated `straight` indices into `(lon, lat)` waypoints and `direct_legs` flags.
pub fn parse_lonlats(lonlats_str: &str, straight_str: Option<&str>) -> Result<ParsedLonLats, String> {
    let pairs: Vec<&str> = lonlats_str
        .split(['|', ';'])
        .filter(|s| !s.trim().is_empty())
        .collect();
    if pairs.len() < 2 {
        return Err("At least two waypoints required in 'lonlats'".to_string());
    }

    let mut waypoints = Vec::with_capacity(pairs.len());
    let mut direct_flags = vec![false; pairs.len()];
    for (idx, pair) in pairs.iter().enumerate() {
        let parts: Vec<&str> = pair.split(',').collect();
        if parts.len() < 2 {
            return Err(format!("Invalid waypoint coordinate pair: {}", pair));
        }
        let lon: f64 = parts[0]
            .trim()
            .parse()
            .map_err(|_| "Invalid longitude".to_string())?;
        let lat: f64 = parts[1]
            .trim()
            .parse()
            .map_err(|_| "Invalid latitude".to_string())?;
        if parts.len() > 2 && parts[2].trim().eq_ignore_ascii_case("d") {
            direct_flags[idx] = true;
        }
        waypoints.push((lon, lat));
    }

    if let Some(straight) = straight_str {
        for token in straight.split(',') {
            if let Ok(idx) = token.trim().parse::<usize>() {
                if idx < direct_flags.len() {
                    direct_flags[idx] = true;
                }
            }
        }
    }

    // Matches RoutingEngine.java: `boolean calcBeeline = startWp.wpttype == MatchedWaypoint.WAYPOINT_TYPE_DIRECT;`
    let mut direct_legs = vec![false; waypoints.len().saturating_sub(1)];
    for i in 0..direct_legs.len() {
        if direct_flags[i] {
            direct_legs[i] = true;
        }
    }
    if direct_legs.len() == 1 && *direct_flags.last().unwrap_or(&false) {
        direct_legs[0] = true;
    }

    Ok(ParsedLonLats {
        waypoints,
        direct_legs,
    })
}

/// Parses no-go areas from BRouter's `nogos` query parameter format:
/// `lon,lat,radius[,weight]|lon,lat,radius[,weight]|...` (also accepts `;` separator).
pub fn parse_nogos(s: &str) -> Vec<NogoArea> {
    let trimmed = s.trim();
    if trimmed.is_empty() {
        return Vec::new();
    }
    let mut res = Vec::new();
    for item in trimmed.split(['|', ';']) {
        let item = item.trim();
        if item.is_empty() {
            continue;
        }
        let parts: Vec<&str> = item.split(',').collect();
        if parts.len() < 3 {
            continue;
        }
        let Ok(lon) = parts[0].trim().parse::<f64>() else {
            continue;
        };
        let Ok(lat) = parts[1].trim().parse::<f64>() else {
            continue;
        };
        let Ok(radius_m) = parts[2].trim().parse::<f64>() else {
            continue;
        };
        if radius_m <= 0.0 {
            continue;
        }
        let weight = if parts.len() > 3 {
            parse_optional_weight(parts[3])
        } else {
            None
        };
        res.push(NogoArea {
            lon,
            lat,
            radius_m,
            weight,
            vertices: Vec::new(),
            is_closed: false,
        });
    }
    res
}

/// Parses polyline (`is_closed = false`) or polygon (`is_closed = true`) no-go areas
/// from BRouter's `polylines` / `polygons` query parameter format:
/// `lon1,lat1,lon2,lat2[,weight]|...`
pub fn parse_nogo_polygons(s: &str, is_closed: bool) -> Vec<NogoArea> {
    let trimmed = s.trim();
    if trimmed.is_empty() {
        return Vec::new();
    }
    let mut res = Vec::new();
    for item in trimmed.split(['|', ';']) {
        let item = item.trim();
        if item.is_empty() {
            continue;
        }
        let parts: Vec<&str> = item.split(',').collect();
        if parts.len() < 4 {
            continue;
        }
        let pair_count = parts.len() / 2;
        let mut vertices = Vec::with_capacity(pair_count);
        let mut valid = true;
        for i in 0..pair_count {
            let Ok(lon) = parts[2 * i].trim().parse::<f64>() else {
                valid = false;
                break;
            };
            let Ok(lat) = parts[2 * i + 1].trim().parse::<f64>() else {
                valid = false;
                break;
            };
            vertices.push((lon, lat));
        }
        if !valid {
            continue;
        }
        let weight = if parts.len() % 2 == 1 {
            parse_optional_weight(parts[parts.len() - 1])
        } else {
            None
        };
        if let Some(poly) = NogoArea::from_polygon(vertices, is_closed, weight) {
            res.push(poly);
        }
    }
    res
}

/// Removes no-go zones that contain any of the given waypoints, matching BRouter's `cleanNogoList`.
///
/// This prevents routes from becoming impossible when a waypoint is placed inside an avoided circle/polygon.
pub fn clean_nogos(nogos: &[NogoArea], waypoints: &[Point2D]) -> Vec<InternalNogo> {
    let mut result = Vec::with_capacity(nogos.len());
    for area in nogos {
        let internal = InternalNogo::from_area(area);
        let mut discard = false;
        if internal.vertices.is_empty() {
            let nogo_pt = Point2D::new(internal.ilon, internal.ilat);
            for wp in waypoints {
                if distance(*wp, nogo_pt) < internal.radius_m {
                    discard = true;
                    break;
                }
            }
        } else if internal.is_closed {
            for wp in waypoints {
                if point_in_polygon(*wp, &internal.vertices) {
                    discard = true;
                    break;
                }
            }
        }
        if !discard {
            result.push(internal);
        }
    }
    result
}

/// Calculates the distance of a segment inside a circle, matching BRouter's
/// `OsmNodeNamed.distanceWithinRadius`.
#[allow(clippy::too_many_arguments)]
pub fn distance_within_radius(
    mut lon1: i32,
    mut lat1: i32,
    mut lon2: i32,
    mut lat2: i32,
    total_segment_length: f64,
    center_lon: i32,
    center_lat: i32,
    radius: f64,
) -> f64 {
    let [kx, ky] = get_lon_lat_to_meter_scales((lat1 + lat2) >> 1);
    let first_inside =
        distance(Point2D::new(lon1, lat1), Point2D::new(center_lon, center_lat)) < radius;
    let mut last_inside =
        distance(Point2D::new(lon2, lat2), Point2D::new(center_lon, center_lat)) < radius;

    if first_inside {
        if last_inside {
            return total_segment_length;
        }
        std::mem::swap(&mut lon1, &mut lon2);
        std::mem::swap(&mut lat1, &mut lat2);
        last_inside = first_inside;
    }

    let initial_to_project = ((lon2 - lon1) as f64 * (center_lon - lon1) as f64 * kx * kx
        + (lat2 - lat1) as f64 * (center_lat - lat1) as f64 * ky * ky)
        / total_segment_length;

    let initial_to_center =
        distance(Point2D::new(center_lon, center_lat), Point2D::new(lon1, lat1));
    let under_sqrt = (radius * radius
        - (initial_to_center * initial_to_center - initial_to_project * initial_to_project))
        .max(0.0);
    let half_dist = under_sqrt.sqrt();

    if last_inside {
        half_dist + (total_segment_length - initial_to_project)
    } else {
        2.0 * half_dist
    }
}

#[inline]
fn ccw(a: Point2D, b: Point2D, c: Point2D) -> f64 {
    (b.lon - a.lon) as f64 * (c.lat - a.lat) as f64
        - (b.lat - a.lat) as f64 * (c.lon - a.lon) as f64
}

#[inline]
fn segments_intersect(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool {
    let d1 = ccw(a, b, c);
    let d2 = ccw(a, b, d);
    let d3 = ccw(c, d, a);
    let d4 = ccw(c, d, b);
    ((d1 <= 0.0 && d2 >= 0.0) || (d1 >= 0.0 && d2 <= 0.0))
        && ((d3 <= 0.0 && d4 >= 0.0) || (d3 >= 0.0 && d4 <= 0.0))
}

fn point_in_polygon(pt: Point2D, vertices: &[Point2D]) -> bool {
    if vertices.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = vertices.len() - 1;
    for i in 0..vertices.len() {
        let vi = vertices[i];
        let vj = vertices[j];
        if ((vi.lat > pt.lat) != (vj.lat > pt.lat))
            && ((pt.lon as f64)
                < (vj.lon - vi.lon) as f64 * (pt.lat - vi.lat) as f64 / (vj.lat - vi.lat) as f64
                    + vi.lon as f64)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn segment_intersects_polygon(p1: Point2D, p2: Point2D, nogo: &InternalNogo) -> bool {
    let verts = &nogo.vertices;
    if verts.len() < 2 {
        return false;
    }
    for i in 0..verts.len() - 1 {
        if segments_intersect(p1, p2, verts[i], verts[i + 1]) {
            return true;
        }
    }
    if nogo.is_closed && verts.len() >= 3 {
        if segments_intersect(p1, p2, verts[verts.len() - 1], verts[0]) {
            return true;
        }
        if point_in_polygon(p1, verts) || point_in_polygon(p2, verts) {
            return true;
        }
    }
    false
}

/// Tests whether a segment between `p1` and `p2` intersects any no-go area,
/// using the exact BRouter distance formula from `RoutingContext.calcDistance`.
pub fn check_segment_against_nogos(
    p1: Point2D,
    p2: Point2D,
    nogos: &[InternalNogo],
) -> NogoCheckResult {
    if nogos.is_empty() {
        return NogoCheckResult::Pass { added_cost: 0 };
    }

    let seg_min_lon = p1.lon.min(p2.lon);
    let seg_max_lon = p1.lon.max(p2.lon);
    let seg_min_lat = p1.lat.min(p2.lat);
    let seg_max_lat = p1.lat.max(p2.lat);

    let [dlon2m, dlat2m] = get_lon_lat_to_meter_scales((p1.lat + p2.lat) >> 1);
    let dx = (p2.lon - p1.lon) as f64 * dlon2m;
    let dy = (p2.lat - p1.lat) as f64 * dlat2m;
    let d = (dx * dx + dy * dy).sqrt();

    let mut total_added_cost = 0u32;

    for nogo in nogos {
        // Fast AABB rejection
        if seg_max_lon < nogo.min_lon
            || seg_min_lon > nogo.max_lon
            || seg_max_lat < nogo.min_lat
            || seg_min_lat > nogo.max_lat
        {
            continue;
        }

        if !nogo.vertices.is_empty() {
            if segment_intersects_polygon(p1, p2, nogo) {
                if nogo.is_hard() {
                    return NogoCheckResult::Blocked;
                } else if let Some(w) = nogo.weight {
                    total_added_cost += (d * w).round() as u32;
                }
            }
            continue;
        }

        if d <= 0.0 {
            let dist = distance(p1, Point2D::new(nogo.ilon, nogo.ilat));
            if dist < nogo.radius_m {
                if nogo.is_hard() {
                    return NogoCheckResult::Blocked;
                } else if let Some(w) = nogo.weight {
                    total_added_cost += (dist * w).round() as u32;
                }
            }
            continue;
        }

        let x1 = (p1.lon - nogo.ilon) as f64 * dlon2m;
        let y1 = (p1.lat - nogo.ilat) as f64 * dlat2m;
        let x2 = (p2.lon - nogo.ilon) as f64 * dlon2m;
        let y2 = (p2.lat - nogo.ilat) as f64 * dlat2m;
        let r12 = x1 * x1 + y1 * y1;
        let r22 = x2 * x2 + y2 * y2;
        let mut radius = (if r12 < r22 {
            y1 * dx - x1 * dy
        } else {
            y2 * dx - x2 * dy
        })
        .abs()
            / d;

        if radius < nogo.radius_m {
            let mut s1 = x1 * dx + y1 * dy;
            let mut s2 = x2 * dx + y2 * dy;

            if s1 < 0.0 {
                s1 = -s1;
                s2 = -s2;
            }
            if s2 > 0.0 {
                radius = (if s1 < s2 { r12 } else { r22 }).sqrt();
                if radius > nogo.radius_m {
                    continue;
                }
            }

            // Segment intersects the nogo circle
            if nogo.is_hard() {
                return NogoCheckResult::Blocked;
            } else if let Some(w) = nogo.weight {
                let dist_within = distance_within_radius(
                    p1.lon,
                    p1.lat,
                    p2.lon,
                    p2.lat,
                    d,
                    nogo.ilon,
                    nogo.ilat,
                    nogo.radius_m,
                );
                total_added_cost += (dist_within * w).round() as u32;
            }
        }
    }

    NogoCheckResult::Pass {
        added_cost: total_added_cost,
    }
}

/// Tests whether a polyline intersects any no-go area.
pub fn check_polyline_against_nogos(
    poly: &[Point3D],
    nogos: &[InternalNogo],
) -> NogoCheckResult {
    if nogos.is_empty() || poly.len() < 2 {
        return NogoCheckResult::Pass { added_cost: 0 };
    }
    let mut total_added_cost = 0u32;
    for i in 0..poly.len() - 1 {
        match check_segment_against_nogos(poly[i].to_2d(), poly[i + 1].to_2d(), nogos) {
            NogoCheckResult::Blocked => return NogoCheckResult::Blocked,
            NogoCheckResult::Pass { added_cost } => total_added_cost += added_cost,
        }
    }
    NogoCheckResult::Pass {
        added_cost: total_added_cost,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_nogos() {
        let empty = parse_nogos("");
        assert!(empty.is_empty());

        let single = parse_nogos("-0.12,51.50,50");
        assert_eq!(single.len(), 1);
        assert_eq!(single[0].lon, -0.12);
        assert_eq!(single[0].lat, 51.50);
        assert_eq!(single[0].radius_m, 50.0);
        assert!(single[0].is_hard());

        let weighted = parse_nogos("-0.12,51.50,50,15.5|-0.15,51.52,100,-1");
        assert_eq!(weighted.len(), 2);
        assert_eq!(weighted[0].weight, Some(15.5));
        assert!(!weighted[0].is_hard());
        assert_eq!(weighted[1].weight, Some(-1.0));
        assert!(weighted[1].is_hard());
    }

    #[test]
    fn test_segment_nogo_intersection() {
        // Point in central London
        let nogo = NogoArea::new(-0.1278, 51.5074, 50.0);
        let internal = [InternalNogo::from_area(&nogo)];

        // Segment straight through the center
        let p1 = Point2D::new(lon_to_ilon(-0.1288), lat_to_ilat(51.5074));
        let p2 = Point2D::new(lon_to_ilon(-0.1268), lat_to_ilat(51.5074));

        let res = check_segment_against_nogos(p1, p2, &internal);
        assert_eq!(res, NogoCheckResult::Blocked);

        // Segment completely outside (500m north)
        let p3 = Point2D::new(lon_to_ilon(-0.1288), lat_to_ilat(51.5150));
        let p4 = Point2D::new(lon_to_ilon(-0.1268), lat_to_ilat(51.5150));

        let res2 = check_segment_against_nogos(p3, p4, &internal);
        assert_eq!(res2, NogoCheckResult::Pass { added_cost: 0 });
    }

    #[test]
    fn test_clean_nogos_removes_containing_waypoints() {
        let nogo1 = NogoArea::new(-0.12, 51.50, 100.0);
        let nogo2 = NogoArea::new(-0.20, 51.60, 100.0);
        let wps = [Point2D::new(lon_to_ilon(-0.1201), lat_to_ilat(51.5001))];

        let cleaned = clean_nogos(&[nogo1, nogo2], &wps);
        assert_eq!(cleaned.len(), 1);
        assert_eq!(cleaned[0].ilon, lon_to_ilon(-0.20));
    }
}

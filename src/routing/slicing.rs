//! Geometric polyline slicing and projection helpers for waypoint snapping.
//!
//! Provides projection of GPS coordinates onto road polylines and slicing of edges
//! at start/target waypoint crosspoints.

use crate::graph::spatial;
use crate::types::{Distance, ElevationQuarterMeters, Point2D, Point3D};

/// Projects `cross` onto `poly`, returning
/// `(segment_index, projected_point, cumulative_distance_to_projection)`.
pub fn find_crosspoint_projection(
    poly: &[Point3D],
    cross: Point2D,
) -> (usize, Point3D, Distance) {
    if poly.len() <= 1 {
        let pt = poly
            .first()
            .copied()
            .unwrap_or(Point3D::new(cross.lon, cross.lat, i16::MIN));
        return (0, pt, 0.0);
    }

    let mut best_idx = 0;
    let mut min_dist_sq = f64::MAX;
    let mut best_elev = poly[0].elev;
    let mut best_frac = 0.0;

    for i in 0..poly.len() - 1 {
        let p1 = &poly[i];
        let p2 = &poly[i + 1];

        let scales = spatial::get_lon_lat_to_meter_scales((p1.lat + p2.lat) >> 1);
        let dlon2m = scales[0];
        let dlat2m = scales[1];

        let dx = (p2.lon - p1.lon) as f64 * dlon2m;
        let dy = (p2.lat - p1.lat) as f64 * dlat2m;
        let d_sq = dx * dx + dy * dy;

        let (dist_sq, frac) = if d_sq <= 1e-6 {
            let px = (cross.lon - p1.lon) as f64 * dlon2m;
            let py = (cross.lat - p1.lat) as f64 * dlat2m;
            (px * px + py * py, 0.0)
        } else {
            let px1 = (cross.lon - p1.lon) as f64 * dlon2m;
            let py1 = (cross.lat - p1.lat) as f64 * dlat2m;
            let t = ((px1 * dx + py1 * dy) / d_sq).clamp(0.0, 1.0);
            let proj_x = t * dx;
            let proj_y = t * dy;
            let diff_x = px1 - proj_x;
            let diff_y = py1 - proj_y;
            (diff_x * diff_x + diff_y * diff_y, t)
        };

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            best_idx = i;
            best_frac = frac;
            best_elev = if p1.elev != i16::MIN && p2.elev != i16::MIN {
                (p1.elev as f64 * (1.0 - frac) + p2.elev as f64 * frac) as ElevationQuarterMeters
            } else if p1.elev != i16::MIN {
                p1.elev
            } else {
                p2.elev
            };
        }
    }

    let mut cum_dist = 0.0;
    for i in 0..best_idx {
        cum_dist += spatial::distance(poly[i].to_2d(), poly[i + 1].to_2d());
    }
    let seg_len = spatial::distance(poly[best_idx].to_2d(), poly[best_idx + 1].to_2d());
    cum_dist += seg_len * best_frac;

    (best_idx, Point3D::new(cross.lon, cross.lat, best_elev), cum_dist)
}

/// Slices an edge polyline starting from `crosspoint` to the end.
pub fn slice_from_crosspoint(
    poly: &[Point3D],
    cross: Point2D,
) -> Vec<Point3D> {
    let (idx, cross_pt, _) = find_crosspoint_projection(poly, cross);
    let mut sliced = Vec::with_capacity(poly.len() - idx + 1);
    sliced.push(cross_pt);
    for pt in &poly[idx + 1..] {
        if pt.lon != cross_pt.lon || pt.lat != cross_pt.lat {
            sliced.push(*pt);
        }
    }
    if sliced.len() == 1 && poly.len() >= 2 {
        sliced.push(poly[poly.len() - 1]);
    }
    sliced
}

/// Slices an edge polyline starting from beginning up to `crosspoint`.
pub fn slice_to_crosspoint(
    poly: &[Point3D],
    cross: Point2D,
) -> Vec<Point3D> {
    let (idx, cross_pt, _) = find_crosspoint_projection(poly, cross);
    let mut sliced = Vec::with_capacity(idx + 2);
    for pt in &poly[..=idx] {
        if pt.lon != cross_pt.lon || pt.lat != cross_pt.lat {
            sliced.push(*pt);
        }
    }
    if sliced.is_empty() || sliced.last() != Some(&cross_pt) {
        sliced.push(cross_pt);
    }
    if sliced.len() == 1 && poly.len() >= 2 {
        sliced.insert(0, poly[0]);
    }
    sliced
}

/// Slices an edge polyline between two crosspoints on the same segment.
pub fn slice_between_crosspoints(
    poly: &[Point3D],
    start: Point2D,
    target: Point2D,
) -> Option<Vec<Point3D>> {
    let (s_idx, s_pt, s_dist) = find_crosspoint_projection(poly, start);
    let (t_idx, t_pt, t_dist) = find_crosspoint_projection(poly, target);

    if s_dist > t_dist {
        return None;
    }

    let mut sliced = vec![s_pt];
    if s_idx < t_idx {
        for pt in &poly[s_idx + 1..=t_idx] {
            if pt.lon != s_pt.lon || pt.lat != s_pt.lat {
                sliced.push(*pt);
            }
        }
    }
    if t_pt.lon != sliced.last().unwrap().lon || t_pt.lat != sliced.last().unwrap().lat {
        sliced.push(t_pt);
    }
    Some(sliced)
}

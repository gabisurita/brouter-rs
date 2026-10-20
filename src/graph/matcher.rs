//! Rust implementation corresponding to:
//! - Java class: `btools.mapaccess.MatchedWaypoint`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/MatchedWaypoint.java`
//! - Java class: `btools.mapaccess.WaypointMatcherImpl`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/WaypointMatcherImpl.java`
//!
//! # Summary
//! Projects input geographic waypoints onto nearest graph edges and computes
//! exact crosspoints, nearest node endpoints, and orthogonal projection distances.

use crate::types::{Distance, Point2D};

use super::spatial;

/// A matched waypoint snapped to the nearest road segment in the graph.
#[derive(Debug, Clone, PartialEq)]
pub struct MatchedWaypoint {
    pub name: String,
    pub waypoint: Point2D,
    pub crosspoint: Point2D,
    pub node1: Point2D,
    pub node2: Point2D,
    pub radius: Distance,
    pub direction_to_next: f64,
    pub direction_diff: f64,
}

impl MatchedWaypoint {
    pub fn new(name: &str, waypoint: Point2D, max_radius: Distance) -> Self {
        Self {
            name: name.to_string(),
            waypoint,
            crosspoint: waypoint,
            node1: waypoint,
            node2: waypoint,
            radius: max_radius,
            direction_to_next: -1.0,
            direction_diff: 360.0,
        }
    }

    pub fn with_direction(
        name: &str,
        waypoint: Point2D,
        max_radius: Distance,
        direction_to_next: f64,
    ) -> Self {
        Self {
            name: name.to_string(),
            waypoint,
            crosspoint: waypoint,
            node1: waypoint,
            node2: waypoint,
            radius: max_radius,
            direction_to_next,
            direction_diff: 360.0,
        }
    }
}

/// Snaps waypoints to road segment polylines.
///
/// Corresponds to Java class `btools.mapaccess.WaypointMatcherImpl`.
pub struct WaypointMatcher;

impl WaypointMatcher {
    /// Evaluates a single polyline segment `seg_start` -> `seg_end`
    /// belonging to edge `edge_start` -> `edge_end`.
    ///
    /// Matches `WaypointMatcherImpl.checkSegment`:
    /// Computes the orthogonal projection of the waypoint onto the segment vector in meter units,
    /// checks segment endpoints if the projection lies outside `[0, 1]`, and updates `mwp`
    /// if the perpendicular distance is smaller than current `mwp.radius` (or equal with smaller
    /// `direction_diff`).
    pub fn check_segment(
        mwp: &mut MatchedWaypoint,
        seg_start: spatial::Point2D,
        seg_end: spatial::Point2D,
        edge_start: spatial::Point2D,
        edge_end: spatial::Point2D,
    ) -> bool {
        let scales = spatial::get_lon_lat_to_meter_scales((seg_start.lat + seg_end.lat) >> 1);
        let dlon2m = scales[0];
        let dlat2m = scales[1];

        let dx = (seg_end.lon - seg_start.lon) as f64 * dlon2m;
        let dy = (seg_end.lat - seg_start.lat) as f64 * dlat2m;
        let d = (dy * dy + dx * dx).sqrt();

        if d == 0.0 {
            return false;
        }

        let wp = mwp.waypoint;
        let x1 = (seg_start.lon - wp.lon) as f64 * dlon2m;
        let y1 = (seg_start.lat - wp.lat) as f64 * dlat2m;
        let x2 = (seg_end.lon - wp.lon) as f64 * dlon2m;
        let y2 = (seg_end.lat - wp.lat) as f64 * dlat2m;
        let r12 = x1 * x1 + y1 * y1;
        let r22 = x2 * x2 + y2 * y2;

        let mut radius = if r12 < r22 {
            (y1 * dx - x1 * dy).abs() / d
        } else {
            (y2 * dx - x2 * dy).abs() / d
        };

        if radius <= mwp.radius {
            let mut s1 = x1 * dx + y1 * dy;
            let mut s2 = x2 * dx + y2 * dy;

            if s1 < 0.0 {
                s1 = -s1;
                s2 = -s2;
            }

            if s2 > 0.0 {
                radius = if s1 < s2 { r12.sqrt() } else { r22.sqrt() };
                if radius > mwp.radius {
                    return false;
                }
            }

            let crosspoint = if s2 < 0.0 {
                let wayfraction = -s2 / (d * d);
                let xm = x2 - wayfraction * dx;
                let ym = y2 - wayfraction * dy;
                Point2D::new(
                    (xm / dlon2m + wp.lon as f64) as i32,
                    (ym / dlat2m + wp.lat as f64) as i32,
                )
            } else if s1 > s2 {
                seg_end
            } else {
                seg_start
            };

            let best_diff = if mwp.direction_to_next >= 0.0 {
                let angle_fwd = spatial::get_direction(edge_start, edge_end);
                let diff_fwd =
                    spatial::get_difference_from_direction(mwp.direction_to_next, angle_fwd);
                let angle_rev = spatial::get_direction(edge_end, edge_start);
                let diff_rev =
                    spatial::get_difference_from_direction(mwp.direction_to_next, angle_rev);
                diff_fwd.min(diff_rev)
            } else {
                0.0
            };

            if radius < mwp.radius || (radius == mwp.radius && best_diff < mwp.direction_diff) {
                mwp.radius = radius;
                mwp.crosspoint = crosspoint;
                mwp.node1 = edge_start;
                mwp.node2 = edge_end;
                mwp.direction_diff = best_diff;
                return true;
            }
        }
        false
    }
}

//! Rust implementation corresponding to:
//! - Java class: `btools.util.CheapRuler`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-util/src/main/java/btools/util/CheapRuler.java`
//! - Java class: `btools.util.CheapAngleMeter`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-util/src/main/java/btools/util/CheapAngleMeter.java`
//!
//! # Summary
//! High-speed geodesic distance, scale factor, and angle calculations using
//! precomputed trigonometric lookup tables matching BRouter's 1800-entry cache.

use std::sync::OnceLock;

pub use crate::types::{Distance, ElevationQuarterMeters, MicroDegrees, Point2D, Point3D};

pub const ILATLNG_TO_LATLNG: f64 = 1e-6;
pub const KILOMETERS_TO_METERS: f64 = 1000.0;
pub const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;

/// Number of latitude bins in the scale cache (180 degrees / 0.1 degree = 1800 bins).
pub const SCALE_CACHE_LENGTH: usize = 1800;

/// Latitude step size per bin in microdegrees (0.1 degree = 100,000 microdegrees).
pub const SCALE_CACHE_INCREMENT: i32 = 100000;

/// Precomputed geodesic scale factor cache `[kx, ky]` (meters per microdegree)
/// binned by latitude.
///
/// ### Purpose
/// On an oblate spheroid (WGS-84), coordinate differences in degrees do not map
/// uniformly to meters across the globe:
/// - Longitude distance shrinks towards the poles: 1 microdegree of longitude is
///   ~0.111 m at the equator but approaches 0 m near the poles.
/// - Latitude distance varies slightly (~1%) due to Earth's ellipsoidal flattening.
///
/// Converting differences in `(ilon, ilat)` to meters requires latitude-dependent
/// scaling factors `kx` (lon -> meters) and `ky` (lat -> meters), which involve
/// multiple trigonometric calls and Chebyshev polynomial approximations.
///
/// In hot routing loops (A* node expansion, turn angle evaluation, heuristic distance),
/// computing cosines for every link would be a major performance bottleneck.
///
/// Instead, the world latitude [-90°, +90°] (0 to 180,000,000 microdegrees) is divided
/// into 1,800 buckets of 0.1° (100,000 microdegrees) each. This table precomputes
/// `[kx, ky]` for the midpoint of each bucket on first use using [`OnceLock`].
/// Lookups are then simple O(1) array accesses by `ilat / 100_000`.
static SCALE_CACHE: OnceLock<[[f64; 2]; SCALE_CACHE_LENGTH]> = OnceLock::new();

fn get_scale_cache() -> &'static [[f64; 2]; SCALE_CACHE_LENGTH] {
    SCALE_CACHE.get_or_init(|| {
        let mut cache = [[0.0; 2]; SCALE_CACHE_LENGTH];
        for (i, slot) in cache.iter_mut().enumerate() {
            let ilat = (i as i32) * SCALE_CACHE_INCREMENT + SCALE_CACHE_INCREMENT / 2;
            *slot = calc_kx_ky_from_ilat(ilat);
        }
        cache
    })
}

/// Compute `[kx, ky]` scaling factors (meters per microdegree) for an integer latitude `ilat`.
///
/// Matches the polynomial expansion from FCC / Mapbox CheapRuler:
/// - `kx = (111.41513 * cos - 0.09455 * cos3 + 0.00012 * cos5) * 1e-3`
/// - `ky = (111.13209 - 0.56605 * cos2 + 0.0012 * cos4) * 1e-3`
fn calc_kx_ky_from_ilat(ilat: i32) -> [f64; 2] {
    let lat = DEG_TO_RAD * (ilat as f64 * ILATLNG_TO_LATLNG - 90.0);
    let cos = lat.cos();
    let cos2 = 2.0 * cos * cos - 1.0;
    let cos3 = 2.0 * cos * cos2 - cos;
    let cos4 = 2.0 * cos * cos3 - cos2;
    let cos5 = 2.0 * cos * cos4 - cos3;

    let kx = (111.41513 * cos - 0.09455 * cos3 + 0.00012 * cos5)
        * ILATLNG_TO_LATLNG
        * KILOMETERS_TO_METERS;
    let ky =
        (111.13209 - 0.56605 * cos2 + 0.0012 * cos4) * ILATLNG_TO_LATLNG * KILOMETERS_TO_METERS;
    [kx, ky]
}

/// Convert longitude in decimal degrees to BRouter integer longitude (`ilon`).
#[inline]
pub fn lon_to_ilon(lon: f64) -> i32 {
    ((lon + 180.0) * 1_000_000.0).round() as i32
}

/// Convert latitude in decimal degrees to BRouter integer latitude (`ilat`).
#[inline]
pub fn lat_to_ilat(lat: f64) -> i32 {
    ((lat + 90.0) * 1_000_000.0).round() as i32
}

/// Convert BRouter integer longitude (`ilon`) to decimal degrees.
#[inline]
pub fn ilon_to_lon(ilon: i32) -> f64 {
    (ilon - 180_000_000) as f64 * ILATLNG_TO_LATLNG
}

/// Convert BRouter integer latitude (`ilat`) to decimal degrees.
#[inline]
pub fn ilat_to_lat(ilat: i32) -> f64 {
    (ilat - 90_000_000) as f64 * ILATLNG_TO_LATLNG
}

/// Convert 1/4 meter units `selev` to meters.
#[inline]
pub fn selev_to_elevation(selev: i16) -> f64 {
    if selev == i16::MIN {
        0.0
    } else {
        selev as f64 / 4.0
    }
}

/// Convert meters to 1/4 meter units `selev`.
#[inline]
pub fn elevation_to_selev(elev_m: f64) -> i16 {
    (elev_m * 4.0).round() as i16
}

/// Returns `[kx, ky]` distance multipliers (meters per microdegree) for given `ilat`.
#[inline]
pub fn get_lon_lat_to_meter_scales(ilat: i32) -> [f64; 2] {
    let cache = get_scale_cache();
    let idx = (ilat / SCALE_CACHE_INCREMENT) as usize;
    if idx < SCALE_CACHE_LENGTH {
        cache[idx]
    } else {
        cache[SCALE_CACHE_LENGTH - 1]
    }
}

/// Geodesic distance in meters between two points `p1` and `p2`.
#[inline]
pub fn distance(p1: Point2D, p2: Point2D) -> Distance {
    let kxky = get_lon_lat_to_meter_scales((p1.lat + p2.lat) >> 1);
    let dlon = (p1.lon - p2.lon) as f64 * kxky[0];
    let dlat = (p1.lat - p2.lat) as f64 * kxky[1];
    (dlat * dlat + dlon * dlon).sqrt()
}

/// Calculate cosine of turn angle for three consecutive points without computing angle in degrees.
#[inline(always)]
pub fn calc_cos_angle(p0: Point2D, p1: Point2D, p2: Point2D) -> f64 {
    let kxky = get_lon_lat_to_meter_scales(p1.lat);
    let lon2m = kxky[0];
    let lat2m = kxky[1];

    let dx10 = (p1.lon - p0.lon) as f64 * lon2m;
    let dy10 = (p1.lat - p0.lat) as f64 * lat2m;
    let dx21 = (p2.lon - p1.lon) as f64 * lon2m;
    let dy21 = (p2.lat - p1.lat) as f64 * lat2m;

    let dd = ((dx10 * dx10 + dy10 * dy10) * (dx21 * dx21 + dy21 * dy21)).sqrt();
    if dd == 0.0 {
        1.0
    } else {
        (dy10 * dy21 + dx10 * dx21) / dd
    }
}

/// Calculate turn angle and its cosine for three consecutive points.
///
/// Returns `(angle_degrees, cos_angle)`.
#[inline]
pub fn calc_angle(p0: Point2D, p1: Point2D, p2: Point2D) -> (f64, f64) {
    let kxky = get_lon_lat_to_meter_scales(p1.lat);
    let lon2m = kxky[0];
    let lat2m = kxky[1];

    let dx10 = (p1.lon - p0.lon) as f64 * lon2m;
    let dy10 = (p1.lat - p0.lat) as f64 * lat2m;
    let dx21 = (p2.lon - p1.lon) as f64 * lon2m;
    let dy21 = (p2.lat - p1.lat) as f64 * lat2m;

    let dd = ((dx10 * dx10 + dy10 * dy10) * (dx21 * dx21 + dy21 * dy21)).sqrt();
    if dd == 0.0 {
        return (0.0, 1.0);
    }

    let mut sinp = (dy10 * dx21 - dx10 * dy21) / dd;
    let cosp = (dy10 * dy21 + dx10 * dx21) / dd;
    let cosangle = cosp;

    let mut offset = 0.0;
    let mut s2 = sinp * sinp;
    if s2 > 0.5 {
        if sinp > 0.0 {
            offset = 90.0;
            sinp = -cosp;
        } else {
            offset = -90.0;
            sinp = cosp;
        }
        s2 = cosp * cosp;
    } else if cosp < 0.0 {
        sinp = -sinp;
        offset = if sinp > 0.0 { -180.0 } else { 180.0 };
    }

    // 4th-order Chebyshev polynomial approximation of arcsin in degrees,
    // matching BRouter's Java class `btools.util.CheapAngleMeter.calcAngle`:
    //   asin(x) * (180 / pi) approx x * (57.4539 + x^2 * (9.57565 + x^2 * (4.30904 + x^2 * 2.56491)))
    let angle = offset + sinp * (57.4539 + s2 * (9.57565 + s2 * (4.30904 + s2 * 2.56491)));
    (angle, cosangle)
}

/// Bearing in degrees (0..360) from point 1 to point 2.
pub fn get_direction(p1: Point2D, p2: Point2D) -> f64 {
    let xdiff = (p2.lat - p1.lat) as f64;
    let ydiff = (p2.lon - p1.lon) as f64;
    let res = ydiff.atan2(xdiff) * RAD_TO_DEG;
    normalize(res)
}

/// Normalizes an angle into `[0.0, 360.0)`.
#[inline]
pub fn normalize(a: f64) -> f64 {
    a.rem_euclid(360.0)
}

/// Absolute difference in degrees between two bearings `[0, 180]`.
#[inline]
pub fn get_difference_from_direction(b1: f64, b2: f64) -> f64 {
    let mut r = (b2 - b1) % 360.0;
    if r < -180.0 {
        r += 360.0;
    }
    if r >= 180.0 {
        r -= 360.0;
    }
    r.abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_cache_accuracy() {
        let cache = get_scale_cache();
        for (i, cached) in cache.iter().enumerate().take(SCALE_CACHE_LENGTH) {
            let ilat = (i as i32) * SCALE_CACHE_INCREMENT + SCALE_CACHE_INCREMENT / 2;
            let dynamic = calc_kx_ky_from_ilat(ilat);
            assert!((dynamic[0] - cached[0]).abs() < 1e-12);
            assert!((dynamic[1] - cached[1]).abs() < 1e-12);
        }
    }
}


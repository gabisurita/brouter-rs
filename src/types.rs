//! Domain type aliases for physical quantities, coordinates, and identifiers.

/// Internal search cost metric.
pub type Cost = u32;

/// Physical distance or length in meters.
pub type Distance = f64;

/// Elapsed travel time in seconds.
pub type Seconds = f32;

/// Mechanical energy in Joules.
pub type Joules = f32;

/// Coordinate component in microdegrees (1e-6 degrees).
pub type MicroDegrees = i32;

/// Unique 64-bit integer identifier for an OSM node.
pub type NodeId = u64;

/// Elevation in quarter-meter units (`selev`).
pub type ElevationQuarterMeters = i16;

/// 2D coordinate pair in microdegrees `(lon, lat)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct Point2D {
    pub lon: MicroDegrees,
    pub lat: MicroDegrees,
}

impl Point2D {
    #[inline(always)]
    pub const fn new(lon: MicroDegrees, lat: MicroDegrees) -> Self {
        Self { lon, lat }
    }

    #[inline(always)]
    pub fn node_id(self) -> NodeId {
        ((self.lon as u64) << 32) | (self.lat as u32 as u64)
    }
}

impl From<(MicroDegrees, MicroDegrees)> for Point2D {
    #[inline(always)]
    fn from((lon, lat): (MicroDegrees, MicroDegrees)) -> Self {
        Self { lon, lat }
    }
}

impl From<Point2D> for (MicroDegrees, MicroDegrees) {
    #[inline(always)]
    fn from(p: Point2D) -> Self {
        (p.lon, p.lat)
    }
}

/// 3D coordinate point: `(lon, lat, elev)` where `elev` is in quarter-meter units (`selev`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct Point3D {
    pub lon: MicroDegrees,
    pub lat: MicroDegrees,
    pub elev: ElevationQuarterMeters,
}

impl Point3D {
    #[inline(always)]
    pub const fn new(lon: MicroDegrees, lat: MicroDegrees, elev: ElevationQuarterMeters) -> Self {
        Self { lon, lat, elev }
    }

    #[inline(always)]
    pub const fn to_2d(self) -> Point2D {
        Point2D {
            lon: self.lon,
            lat: self.lat,
        }
    }
}

impl From<(MicroDegrees, MicroDegrees, ElevationQuarterMeters)> for Point3D {
    #[inline(always)]
    fn from((lon, lat, elev): (MicroDegrees, MicroDegrees, ElevationQuarterMeters)) -> Self {
        Self { lon, lat, elev }
    }
}

impl From<Point3D> for (MicroDegrees, MicroDegrees, ElevationQuarterMeters) {
    #[inline(always)]
    fn from(p: Point3D) -> Self {
        (p.lon, p.lat, p.elev)
    }
}

/// Geographic bounding box in degrees `(min_lon, min_lat, max_lon, max_lat)`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoundingBox {
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
}

impl BoundingBox {
    #[inline(always)]
    pub const fn new(min_lon: f64, min_lat: f64, max_lon: f64, max_lat: f64) -> Self {
        Self {
            min_lon,
            min_lat,
            max_lon,
            max_lat,
        }
    }
}

impl std::str::FromStr for BoundingBox {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let parts: Vec<f64> = s
            .split(',')
            .filter_map(|t| t.trim().parse::<f64>().ok())
            .collect();
        if parts.len() != 4 {
            return Err("Expected min_lon,min_lat,max_lon,max_lat".to_string());
        }
        Ok(Self::new(parts[0], parts[1], parts[2], parts[3]))
    }
}

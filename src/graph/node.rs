//! Rust implementation corresponding to:
//! - Java class: `btools.mapaccess.OsmNode`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/OsmNode.java`
//!
//! # Summary
//! Node container in the routing graph holding coordinates, elevation,
//! OSM node tags, and turn restrictions.

use crate::rd5::micro_cache::DecodedTurnRestriction;
pub use crate::types::{ElevationQuarterMeters, MicroDegrees, NodeId, Point2D};

/// Unique 64-bit integer identifier derived from integer `(ilon, ilat)`.
///
/// Packs `ilon` in high 32 bits and `ilat` in low 32 bits, matching Java class `btools.mapaccess.OsmNode.getIdFromPos`.
#[inline]
pub fn make_node_id(ilon: MicroDegrees, ilat: MicroDegrees) -> NodeId {
    ((ilon as u64) << 32) | (ilat as u32 as u64)
}

/// Unpack integer `(ilon, ilat)` from a 64-bit node ID.
#[inline]
pub fn unpack_node_id(id: NodeId) -> Point2D {
    let ilon = (id >> 32) as MicroDegrees;
    let ilat = id as u32 as MicroDegrees;
    Point2D::new(ilon, ilat)
}

/// An OSM node in the routing graph.
///
/// Corresponds to Java class `btools.mapaccess.OsmNode`.
#[derive(Debug, Clone, PartialEq)]
pub struct OsmNode {
    pub ilon: MicroDegrees,
    pub ilat: MicroDegrees,
    pub selev: ElevationQuarterMeters,
    pub node_desc_id: u32,
    pub turn_restrictions: Vec<DecodedTurnRestriction>,
}

impl OsmNode {
    pub fn new(ilon: MicroDegrees, ilat: MicroDegrees) -> Self {
        Self {
            ilon,
            ilat,
            selev: i16::MIN,
            node_desc_id: u32::MAX,
            turn_restrictions: Vec::new(),
        }
    }

    #[inline]
    pub fn id(&self) -> NodeId {
        make_node_id(self.ilon, self.ilat)
    }
}

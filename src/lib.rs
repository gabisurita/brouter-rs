//! # brouter-rs
//!
//! Rust port of the BRouter bicycle routing engine.
//! Supports `.rd5` binary segment files and `.brf` routing profiles.

pub mod dsl;
pub mod graph;
pub mod rd5;
pub mod routing;
pub mod types;

pub use dsl::lookups::LookupData;
pub use dsl::parser::Profile;
pub use graph::routing_graph::{RoutingGraph, SnapRequest};
pub use routing::nogo::{parse_lonlats, parse_nogo_polygons, parse_nogos, NogoArea};
pub use routing::search::{AStarRouter, GraphRef, RouteQuery, RouteRequest, RoutingOptions};
pub use routing::track::{RoutePoint, RouteResult};
pub use types::*;

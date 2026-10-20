//! # brouter-rs
//!
//! Pure bicycle routing engine in Rust with 1:1 parity against BRouter.
//! Direct RD5 binary file ingestion and full DSL profile support.

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

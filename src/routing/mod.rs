//! Rust implementation corresponding to:
//! - Java package: `btools.router`
//! - Java source: `https://github.com/abrensch/brouter/tree/master/brouter-core/src/main/java/btools/router`
//!
//! # Summary
//! Core routing engine, kinematic physics modeling, path state tracking,
//! elevation hysteresis buffers, and 2-pass guided A* search.

pub use crate::types::{Cost, Distance, Joules, Seconds};

pub mod cost;
pub mod nogo;
pub mod path;
pub mod search;
pub mod slicing;
pub mod state;
pub mod track;
pub mod voice;

pub use cost::*;
pub use nogo::*;
pub use path::*;
pub use search::*;
pub use slicing::*;
pub use state::*;
pub use track::*;
pub use voice::*;

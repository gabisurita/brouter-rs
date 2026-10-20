//! Unified end-to-end integration and parity tests for rsrouter-server.
//! Reference implementation: `https://github.com/abrensch/brouter`
//!
//! Shares a single preloaded regional graph across all tests to prevent
//! duplicate memory allocations and guarantee bounded peak RSS (~3.2 GB).

mod common;
mod api;
mod parity;

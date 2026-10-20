# brouter-rs

An opinionated Rust translation of the official [BRouter](https://github.com/abrensch/brouter) bicycle routing engine.

All credit for the routing algorithms, `.rd5` binary format, `.brf` profile DSL, kinematic and elevation hysteresis models, and 2-pass guided A* search goes to Arndt Brenschede and the official [BRouter](https://brouter.de/) contributors. See [PARITY.md](PARITY.md) for benchmark results against official BRouter.

## Usage

Add `brouter-rs` to your `Cargo.toml`:

```toml
[dependencies]
brouter-rs = "0.1"
```

Compute a route in Rust (see [`examples/basic_route.rs`](examples/basic_route.rs)):

```rust
use std::sync::Arc;
use brouter_rs::dsl::lookups::LookupData;
use brouter_rs::dsl::parser::Profile;
use brouter_rs::graph::routing_graph::RoutingGraph;
use brouter_rs::rd5::rd5_slice::{FileRd5SegmentReader, LazyRd5Provider};
use brouter_rs::routing::search::AStarRouter;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let lookups = LookupData::load_from_file("profiles/lookups.dat")?;
    let profile = Profile::load_from_file("profiles/fastbike-lowtraffic.brf", &lookups)?;

    let reader = Arc::new(FileRd5SegmentReader::new("data/segments4"));
    let provider = Arc::new(LazyRd5Provider::new(reader));
    let mut graph = RoutingGraph::with_cell_provider(provider);

    let waypoints = [(-0.143613, 51.462414), (-0.055296, 51.544998)];
    let router = AStarRouter::new();
    let route = router.find_multi_route_with_alternative_and_nogos_lazy(
        &mut graph,
        &lookups,
        &profile,
        &waypoints,
        0,
        &[],
    )?;

    println!("Distance: {:.2} km, Cost: {}", route.total_distance_m / 1000.0, route.total_cost);
    let _geojson = route.to_geojson("fastbike-lowtraffic");
    Ok(())
}
```

## Examples and Demos

| Example | Command | Description |
| :--- | :--- | :--- |
| Basic Route | `cargo run --release --example basic_route` | Minimal Rust library usage example |
| HTTP Server | `cargo run --release --example server` | Drop-in BRouter HTTP API server (`/brouter`, `/brouter/profile`) on port 17777 |
| Parity Suite | `cargo run --release --example compare_routes` | Benchmarks 18 routes across 5 UK corridors against `brouter.de` fixtures |
| Route Plotter | `cargo run --release --example plot_routes` | Generates an interactive HTML comparison map (`routes_comparison.html`) |

### Download Routing Data and Fixtures

Download `.rd5` segments and refresh baseline fixtures from `brouter.de`:

```bash
python3 tools/update_data.py --update-tests --check
```

### Run Tests

```bash
cargo test
```

## License

MIT

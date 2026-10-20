# Parity and Benchmarks

Our `brouter-rs` results are extremely close to the official BRouter (https://brouter.de/) results.  The routing is deterministic and the performance is slightly better.
Some small differences exist, but they are very small and likely insignificant for most users.

## Summary

| Metric | brouter-rs | Brouter |
| :--- | :--- | :--- |
| Exact Coordinate Match | 17 / 18 routes | 18 / 18 |
| Exact Cost Match (0.00%) | 18 / 18 routes | 18 / 18 |
| Exact Ascend Match (0 m) | 18 / 18 routes | 18 / 18 |
| Distance Match (0.00% on 17/18) | 18 / 18 within 0.09% | 18 / 18 |
| Max Time Delta | 0.02% (primary), 0.07% (Alt 3) | - |

## Route Distribution

| Route | Profile | Points (RS / BR) | Dist Delta | Cost Delta | Time Delta | Ascend (RS / BR) | Latency (RS / BR) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Clapham -> Hackney | `fastbike` | 919 / 919 | 0.00% | 0.00% | +0.01% | 12 / 12 m | 942 / 708 ms |
| Clapham -> Hackney | `fastbike-lowtraffic` | 910 / 910 | 0.00% | 0.00% | 0.00% | 12 / 12 m | 619 / 721 ms |
| Clapham -> Hackney | `fastbike-verylowtraffic` | 885 / 885 | 0.00% | 0.00% | +0.02% | 12 / 12 m | 579 / 707 ms |
| London -> Brighton | `fastbike` | 3207 / 3207 | 0.00% | 0.00% | 0.00% | 674 / 674 m | 2,886 / 2,844 ms |
| London -> Brighton | `fastbike-lowtraffic` | 3282 / 3282 | 0.00% | 0.00% | 0.00% | 798 / 798 m | 3,171 / 3,372 ms |
| London -> Brighton | `fastbike-verylowtraffic` | 3724 / 3724 | 0.00% | 0.00% | 0.00% | 594 / 594 m | 5,248 / 4,882 ms |
| London -> Oxford | `fastbike` | 3323 / 3323 | 0.00% | 0.00% | 0.00% | 551 / 551 m | 2,442 / 2,733 ms |
| London -> Oxford | `fastbike-lowtraffic` | 2766 / 2766 | 0.00% | 0.00% | 0.00% | 482 / 482 m | 2,966 / 3,109 ms |
| London -> Oxford | `fastbike-verylowtraffic` | 3908 / 3908 | 0.00% | 0.00% | 0.00% | 420 / 420 m | 5,622 / 5,045 ms |
| London -> Cardiff | `fastbike` | 7727 / 7727 | 0.00% | 0.00% | 0.00% | 1111 / 1111 m | 7,235 / 7,576 ms |
| London -> Cardiff | `fastbike-lowtraffic` | 7496 / 7496 | 0.00% | 0.00% | 0.00% | 1242 / 1242 m | 8,431 / 9,210 ms |
| London -> Cardiff | `fastbike-verylowtraffic` | 8845 / 8845 | 0.00% | 0.00% | 0.00% | 1150 / 1150 m | 11,817 / 12,442 ms |
| Richmond -> Windsor -> Marlow -> Oxford | `fastbike` | 2717 / 2717 | 0.00% | 0.00% | 0.00% | 433 / 433 m | 451 / 934 ms |
| Richmond -> Windsor -> Marlow -> Oxford | `fastbike-lowtraffic` | 2673 / 2673 | 0.00% | 0.00% | 0.00% | 445 / 445 m | 457 / 945 ms |
| Richmond -> Windsor -> Marlow -> Oxford | `fastbike-verylowtraffic` | 2834 / 2834 | 0.00% | 0.00% | 0.00% | 461 / 461 m | 787 / 1,397 ms |
| London -> Brighton (Alt 1) | `fastbike-lowtraffic` | 2966 / 2966 | 0.00% | 0.00% | 0.00% | 910 / 910 m | 7,428 ms |
| London -> Brighton (Alt 2) | `fastbike-lowtraffic` | 3140 / 3140 | 0.00% | 0.00% | 0.00% | 802 / 802 m | 14,356 ms |
| London -> Brighton (Alt 3) | `fastbike-lowtraffic` | 3299 / 3301 | +0.09% | 0.00% | +0.07% | 859 / 859 m | 26,479 ms |

## Known Differences

| Difference | Scope | Cause |
| :--- | :--- | :--- |
| Travel time (`+0.01%` to `+0.02%`) | Clapham -> Hackney (`fastbike`, `fastbike-verylowtraffic`) | `brouter-rs` accumulates travel time and cubic speed in 64-bit `f64`, while official BRouter uses 32-bit `float`. Over ~900 segments, float rounding shifts the final integer second cast by 1 s. |
| Equal-cost tie-breaking (`+0.09%` dist, `98.6%` overlap) | London -> Brighton (`Alt 3`) | Both routes have the exact same integer routing cost (`165,368`) and ascend (`859 m`). Priority queue tie-breaking on equal-cost states selects a parallel local street for ~80 m on an 89 km route. |
| Round-trip loop generation (`roundTripDistance`) | Feature scope | Official BRouter `RoundTrip` loop generator is not implemented. |

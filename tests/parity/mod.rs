use crate::common::assert_route_parity;

// ============================================================================
// Route 1: Clapham -> Hackney
// ============================================================================
const CLAPHAM_HACKNEY: &str = "-0.143613,51.462414|-0.055296,51.544998";

#[tokio::test]
async fn test_parity_clapham_hackney_fastbike() {
    assert_route_parity(
        "Clapham -> Hackney",
        "fastbike",
        CLAPHAM_HACKNEY,
        None,
        "clapham_hackney_fastbike.json",
        "clapham_hackney_fastbike.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_clapham_hackney_lowtraffic() {
    assert_route_parity(
        "Clapham -> Hackney",
        "fastbike-lowtraffic",
        CLAPHAM_HACKNEY,
        None,
        "clapham_hackney_lowtraffic.json",
        "clapham_hackney_fastbike-lowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_clapham_hackney_verylowtraffic() {
    assert_route_parity(
        "Clapham -> Hackney",
        "fastbike-verylowtraffic",
        CLAPHAM_HACKNEY,
        None,
        "clapham_hackney_verylowtraffic.json",
        "clapham_hackney_fastbike-verylowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

// ============================================================================
// Route 2: London -> Brighton
// ============================================================================
const LONDON_BRIGHTON: &str = "-0.127818,51.507392|-0.136913,50.822431";

#[tokio::test]
async fn test_parity_london_brighton_fastbike() {
    assert_route_parity(
        "London -> Brighton",
        "fastbike",
        LONDON_BRIGHTON,
        None,
        "london_brighton_fastbike.json",
        "london_brighton_fastbike.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_brighton_lowtraffic() {
    assert_route_parity(
        "London -> Brighton",
        "fastbike-lowtraffic",
        LONDON_BRIGHTON,
        None,
        "london_brighton_fastbike-lowtraffic_alt0.json",
        "london_brighton_fastbike-lowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_brighton_verylowtraffic() {
    assert_route_parity(
        "London -> Brighton",
        "fastbike-verylowtraffic",
        LONDON_BRIGHTON,
        None,
        "london_brighton_verylowtraffic.json",
        "london_brighton_fastbike-verylowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_brighton_alt1() {
    assert_route_parity(
        "London -> Brighton (Alt 1)",
        "fastbike-lowtraffic",
        LONDON_BRIGHTON,
        Some(1),
        "london_brighton_fastbike-lowtraffic_alt1.json",
        "london_brighton_fastbike-lowtraffic_alt1.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_brighton_alt2() {
    assert_route_parity(
        "London -> Brighton (Alt 2)",
        "fastbike-lowtraffic",
        LONDON_BRIGHTON,
        Some(2),
        "london_brighton_fastbike-lowtraffic_alt2.json",
        "london_brighton_fastbike-lowtraffic_alt2.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_brighton_alt3() {
    assert_route_parity(
        "London -> Brighton (Alt 3)",
        "fastbike-lowtraffic",
        LONDON_BRIGHTON,
        Some(3),
        "london_brighton_fastbike-lowtraffic_alt3.json",
        "london_brighton_fastbike-lowtraffic_alt3.geojson",
        0.10,
        0.005,
        0.10,
        98.5,
    )
    .await;
}

// ============================================================================
// Route 3: London -> Oxford
// ============================================================================
const LONDON_OXFORD: &str = "-0.127818,51.507392|-1.257704,51.751988";

#[tokio::test]
async fn test_parity_london_oxford_fastbike() {
    assert_route_parity(
        "London -> Oxford",
        "fastbike",
        LONDON_OXFORD,
        None,
        "london_oxford_fastbike.json",
        "london_oxford_fastbike.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_oxford_lowtraffic() {
    assert_route_parity(
        "London -> Oxford",
        "fastbike-lowtraffic",
        LONDON_OXFORD,
        None,
        "london_oxford_lowtraffic.json",
        "london_oxford_fastbike-lowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_oxford_verylowtraffic() {
    assert_route_parity(
        "London -> Oxford",
        "fastbike-verylowtraffic",
        LONDON_OXFORD,
        None,
        "london_oxford_verylowtraffic.json",
        "london_oxford_fastbike-verylowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

// ============================================================================
// Route 4: London -> Cardiff
// ============================================================================
const LONDON_CARDIFF: &str = "-0.127818,51.507392|-3.179096,51.481607";

#[tokio::test]
async fn test_parity_london_cardiff_fastbike() {
    assert_route_parity(
        "London -> Cardiff",
        "fastbike",
        LONDON_CARDIFF,
        None,
        "london_cardiff_fastbike.json",
        "london_cardiff_fastbike.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_cardiff_lowtraffic() {
    assert_route_parity(
        "London -> Cardiff",
        "fastbike-lowtraffic",
        LONDON_CARDIFF,
        None,
        "london_cardiff_lowtraffic.json",
        "london_cardiff_fastbike-lowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_london_cardiff_verylowtraffic() {
    assert_route_parity(
        "London -> Cardiff",
        "fastbike-verylowtraffic",
        LONDON_CARDIFF,
        None,
        "london_cardiff_verylowtraffic.json",
        "london_cardiff_fastbike-verylowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

// ============================================================================
// Route 5: Richmond -> Windsor -> Marlow -> Oxford (Multi-point)
// ============================================================================
const RICHMOND_OXFORD_MULTI: &str =
    "-0.3060,51.4590|-0.6060,51.4810|-0.7740,51.5710|-1.257704,51.751988";

#[tokio::test]
async fn test_parity_richmond_oxford_fastbike() {
    assert_route_parity(
        "Richmond -> Oxford",
        "fastbike",
        RICHMOND_OXFORD_MULTI,
        None,
        "richmond_oxford_multi_point_fastbike.json",
        "richmond_oxford_multi_point_fastbike.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_richmond_oxford_lowtraffic() {
    assert_route_parity(
        "Richmond -> Oxford",
        "fastbike-lowtraffic",
        RICHMOND_OXFORD_MULTI,
        None,
        "richmond_oxford_multi_point_lowtraffic.json",
        "richmond_oxford_multi_point_fastbike-lowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

#[tokio::test]
async fn test_parity_richmond_oxford_verylowtraffic() {
    assert_route_parity(
        "Richmond -> Oxford",
        "fastbike-verylowtraffic",
        RICHMOND_OXFORD_MULTI,
        None,
        "richmond_oxford_multi_point_verylowtraffic.json",
        "richmond_oxford_multi_point_fastbike-verylowtraffic.geojson",
        0.005,
        0.005,
        0.05,
        100.0,
    )
    .await;
}

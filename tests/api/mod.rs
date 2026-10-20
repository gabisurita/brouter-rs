use axum::body::Body;
use axum::http::{Request, StatusCode};
use http_body_util::BodyExt;
use std::sync::Arc;
use tower::ServiceExt;

use crate::common::server::create_router;
use crate::common::{get_test_app, repo_path, SHARED_STATE};

#[tokio::test]
async fn test_health_endpoint() {
    let Some(app) = get_test_app() else {
        return;
    };

    let response = app
        .oneshot(
            Request::builder()
                .uri("/health")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = response.into_body().collect().await.unwrap().to_bytes();
    let body_str = String::from_utf8_lossy(&body);
    assert!(body_str.contains("\"status\":\"ok\""));
    assert!(body_str.contains("\"engine\":\"brouter-rs\""));
}

#[tokio::test]
async fn test_brouter_endpoint_missing_params() {
    let Some(app) = get_test_app() else {
        return;
    };

    // 1. Missing lonlats
    let res1 = app
        .clone()
        .oneshot(
            Request::builder()
                .uri("/brouter")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res1.status(), StatusCode::BAD_REQUEST);

    // 2. Invalid profile
    let res2 = app
        .oneshot(
            Request::builder()
                .uri("/brouter?lonlats=-0.14,51.46|-0.05,51.54&profile=non_existent_profile")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(res2.status(), StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn test_brouter_endpoint_successful_route() {
    let Some(app) = get_test_app() else {
        println!("Skipping route test: W5_N50.rd5 not found");
        return;
    };

    let uri =
        "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=geojson";
    let response = app
        .oneshot(Request::builder().uri(uri).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);

    let body = response.into_body().collect().await.unwrap().to_bytes();
    let geojson = String::from_utf8_lossy(&body);

    assert!(geojson.contains("\"FeatureCollection\""));
    assert!(geojson.contains("\"LineString\""));
    assert!(geojson.contains("\"track-length\""));
    assert!(geojson.contains("\"cost\""));
}

#[tokio::test]
async fn test_brouter_concurrent_queries() {
    let Some(app) = get_test_app() else {
        return;
    };

    let uri1 = "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=geojson";
    let uri2 = "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike-lowtraffic&format=geojson";

    let (res1, res2) = tokio::join!(
        app.clone().oneshot(Request::builder().uri(uri1).body(Body::empty()).unwrap()),
        app.oneshot(Request::builder().uri(uri2).body(Body::empty()).unwrap()),
    );

    assert_eq!(res1.unwrap().status(), StatusCode::OK);
    assert_eq!(res2.unwrap().status(), StatusCode::OK);
}

#[tokio::test]
async fn test_brouter_shared_graph_routing() {
    let Some(state) = SHARED_STATE.as_ref() else {
        return;
    };

    assert!(state.graph.node_count() > 0);
    assert!(state.graph.edge_count() > 0);

    let app = create_router(Arc::clone(state));

    let uri =
        "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=geojson";
    let response = app
        .oneshot(Request::builder().uri(uri).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);
    let body = response.into_body().collect().await.unwrap().to_bytes();
    let geojson = String::from_utf8_lossy(&body);
    assert!(geojson.contains("\"FeatureCollection\""));
    assert!(geojson.contains("\"track-length\""));
}

#[tokio::test]
async fn test_brouter_multi_waypoint_endpoint() {
    let Some(app) = get_test_app() else {
        return;
    };

    // 4-stop route: Richmond -> Windsor -> Marlow -> Oxford
    let uri = "/brouter?lonlats=-0.3060,51.4590|-0.6060,51.4810|-0.7740,51.5710|-1.257704,51.751988&profile=fastbike&format=geojson";
    let response = app
        .oneshot(Request::builder().uri(uri).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);
    let body = response.into_body().collect().await.unwrap().to_bytes();
    let geojson = String::from_utf8_lossy(&body);

    assert!(geojson.contains("\"FeatureCollection\""));
    assert!(geojson.contains("\"LineString\""));
    assert!(geojson.contains("\"track-length\""));

    let json: serde_json::Value = serde_json::from_str(&geojson).unwrap();
    let props = &json["features"][0]["properties"];
    let dist: f64 = props["track-length"].as_str().unwrap().parse().unwrap();
    let cost: u32 = props["cost"].as_str().unwrap().parse().unwrap();

    let dist_delta = (dist - 87900.0).abs() / 87900.0 * 100.0;
    let cost_delta = (cost as f64 - 116249.0).abs() / 116249.0 * 100.0;

    assert!(dist_delta <= 0.05, "Distance delta {:.3}% > 0.05%", dist_delta);
    assert!(cost_delta <= 0.05, "Cost delta {:.3}% > 0.05%", cost_delta);
}

#[tokio::test]
async fn test_brouter_alternative_routes_endpoint() {
    let Some(app) = get_test_app() else {
        return;
    };

    // 1. Invalid alternativeidx > 3
    let uri_invalid = "/brouter?lonlats=-0.1400,51.4600|-0.0550,51.5450&profile=fastbike-lowtraffic&alternativeidx=4";
    let resp_invalid = app
        .clone()
        .oneshot(Request::builder().uri(uri_invalid).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(resp_invalid.status(), StatusCode::BAD_REQUEST);

    // 2. Query alternativeidx=1
    let uri_alt1 = "/brouter?lonlats=-0.1400,51.4600|-0.0550,51.5450&profile=fastbike-lowtraffic&alternativeidx=1&format=geojson";
    let resp_alt1 = app
        .clone()
        .oneshot(Request::builder().uri(uri_alt1).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(resp_alt1.status(), StatusCode::OK);
    let body_alt1 = resp_alt1.into_body().collect().await.unwrap().to_bytes();
    let geojson_alt1 = String::from_utf8_lossy(&body_alt1);
    let json_alt1: serde_json::Value = serde_json::from_str(&geojson_alt1).unwrap();
    let dist_alt1: f64 = json_alt1["features"][0]["properties"]["track-length"]
        .as_str()
        .unwrap()
        .parse()
        .unwrap();
    assert!(dist_alt1 > 5000.0);
}

#[tokio::test]
async fn test_brouter_endpoint_gpx_format() {
    let Some(app) = get_test_app() else {
        return;
    };

    let uri =
        "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=gpx";
    let response = app
        .oneshot(Request::builder().uri(uri).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(response.status(), StatusCode::OK);
    assert_eq!(
        response.headers().get("content-type").unwrap(),
        "application/gpx+xml"
    );

    let body = response.into_body().collect().await.unwrap().to_bytes();
    let gpx = String::from_utf8_lossy(&body);

    assert!(gpx.contains("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
    assert!(gpx.contains("<gpx version=\"1.1\""));
    assert!(gpx.contains("<trk>"));
    assert!(gpx.contains("<trkseg>"));
    assert!(gpx.contains("<trkpt lat="));
    assert!(gpx.contains("<ele>"));
}

#[tokio::test]
async fn test_brouter_endpoint_with_nogos() {
    let Some(app) = get_test_app() else {
        return;
    };

    // 1. Baseline route
    let uri_base = "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=geojson";
    let resp_base = app
        .clone()
        .oneshot(Request::builder().uri(uri_base).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(resp_base.status(), StatusCode::OK);
    let body_base = resp_base.into_body().collect().await.unwrap().to_bytes();
    let json_base: serde_json::Value = serde_json::from_slice(&body_base).unwrap();
    let base_cost = json_base["features"][0]["properties"]["cost"].as_str().unwrap().to_string();
    let coords = json_base["features"][0]["geometry"]["coordinates"].as_array().unwrap();

    let mid = &coords[coords.len() / 2];
    let mid_lon = mid[0].as_f64().unwrap();
    let mid_lat = mid[1].as_f64().unwrap();

    // 2. Route with 30m hard nogo directly on the midpoint
    let uri_nogo = format!(
        "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=geojson&nogos={:.6},{:.6},30",
        mid_lon, mid_lat
    );
    let resp_nogo = app
        .oneshot(Request::builder().uri(&uri_nogo).body(Body::empty()).unwrap())
        .await
        .unwrap();

    assert_eq!(resp_nogo.status(), StatusCode::OK);
    let body_nogo = resp_nogo.into_body().collect().await.unwrap().to_bytes();
    let json_nogo: serde_json::Value = serde_json::from_slice(&body_nogo).unwrap();
    let nogo_cost = json_nogo["features"][0]["properties"]["cost"].as_str().unwrap().to_string();

    assert_ne!(base_cost, nogo_cost);
}

#[tokio::test]
async fn test_brouter_kml_voicehints_beeline_and_custom_profile() {
    let Some(app) = get_test_app() else {
        return;
    };

    // 1. KML format
    let uri_kml =
        "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=fastbike&format=kml&timode=4";
    let resp_kml = app
        .clone()
        .oneshot(Request::builder().uri(uri_kml).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(resp_kml.status(), StatusCode::OK);
    assert_eq!(
        resp_kml.headers().get("content-type").unwrap(),
        "application/vnd.google-earth.kml+xml"
    );
    let body_kml = resp_kml.into_body().collect().await.unwrap().to_bytes();
    let kml_str = String::from_utf8_lossy(&body_kml);
    assert!(kml_str.contains("<kml xmlns=\"http://earth.google.com/kml/2.0\">"));
    assert!(kml_str.contains("<LineString>"));
    assert!(kml_str.contains("<name>Paths</name>"));

    // 2. GeoJSON voicehints, messages, and times
    let uri_json =
        "/brouter?lonlats=-0.143613,51.462414;-0.055296,51.544998&profile=fastbike&format=geojson&timode=2";
    let resp_json = app
        .clone()
        .oneshot(Request::builder().uri(uri_json).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(resp_json.status(), StatusCode::OK);
    let body_json = resp_json.into_body().collect().await.unwrap().to_bytes();
    let json: serde_json::Value = serde_json::from_slice(&body_json).unwrap();
    let props = &json["features"][0]["properties"];
    assert!(props["voicehints"].as_array().unwrap().len() > 0);
    assert!(props["messages"].as_array().unwrap().len() > 1);
    assert!(props["times"].as_array().unwrap().len() > 2);

    // 3. Beeline (,d suffix and straight=0)
    let uri_beeline =
        "/brouter?lonlats=-0.143613,51.462414,d|-0.055296,51.544998&profile=fastbike&format=geojson";
    let resp_beeline = app
        .clone()
        .oneshot(Request::builder().uri(uri_beeline).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(resp_beeline.status(), StatusCode::OK);
    let body_beeline = resp_beeline.into_body().collect().await.unwrap().to_bytes();
    let json_beeline: serde_json::Value = serde_json::from_slice(&body_beeline).unwrap();
    assert_eq!(
        json_beeline["features"][0]["properties"]["cost"]
            .as_str()
            .unwrap(),
        json_beeline["features"][0]["properties"]["track-length"]
            .as_str()
            .unwrap()
    );
    assert_eq!(
        json_beeline["features"][0]["geometry"]["coordinates"]
            .as_array()
            .unwrap()
            .len(),
        2
    );

    // 4. POST /brouter/profile/:id upload + routing with custom profile
    let custom_brf = std::fs::read_to_string(repo_path("profiles/fastbike.brf")).unwrap();
    let resp_upload = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/brouter/profile/my_fastbike")
                .body(Body::from(custom_brf))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp_upload.status(), StatusCode::OK);
    let body_upload = resp_upload.into_body().collect().await.unwrap().to_bytes();
    let json_upload: serde_json::Value = serde_json::from_slice(&body_upload).unwrap();
    assert_eq!(
        json_upload["profileid"].as_str().unwrap(),
        "custom_my_fastbike"
    );
    assert!(json_upload.get("error").is_none());

    let uri_custom =
        "/brouter?lonlats=-0.143613,51.462414|-0.055296,51.544998&profile=custom_my_fastbike&format=geojson";
    let resp_custom = app
        .oneshot(Request::builder().uri(uri_custom).body(Body::empty()).unwrap())
        .await
        .unwrap();
    assert_eq!(resp_custom.status(), StatusCode::OK);
}


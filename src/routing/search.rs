//! Rust implementation corresponding to:
//! - Java class: `btools.router.RoutingEngine`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/RoutingEngine.java`
//! - Java class: `btools.router.OsmPath`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/OsmPath.java`
//! - Java class: `btools.router.OsmPathElement`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/OsmPathElement.java`
//!
//! # Summary
//! 2-pass guided A* routing search with dynamic cost-cutting, elevation hysteresis,
//! kinematic speed/power modeling, and GeoJSON route generation.



use std::collections::HashSet;

use super::cost::{CarKinematicModel, KinematicConfig};
use super::nogo::{
    check_polyline_against_nogos, check_segment_against_nogos, clean_nogos, InternalNogo,
    NogoArea, NogoCheckResult,
};
use super::path::{
    CrossingConfig, ElevationConfig, KinematicPrePath, PathState, SectionInput, StdPathCalculator,
};
use super::slicing::{slice_between_crosspoints, slice_from_crosspoint, slice_to_crosspoint};
use super::state::{
    CandidateState, EdgeState, FastHashMap, HeapItem, HistoryStep, NodePool,
    Pass0CostGuide, QuadHeap, SearchNode, SearchTrack, TrackStep,
};
use crate::dsl::evaluator::ProfileEvaluator;
use crate::dsl::lookups::LookupData;
use crate::dsl::parser::Profile;
use crate::graph::matcher::MatchedWaypoint;
use crate::graph::routing_graph::{GraphEdge, RoutingGraph, NO_DESC};
use crate::graph::spatial::{
    self, calc_angle, distance, ilat_to_lat, ilon_to_lon, selev_to_elevation,
};
use crate::rd5::micro_cache::DecodedTurnRestriction;
use crate::types::{Cost, Distance, Joules, NodeId, Point2D, Point3D, Seconds};

pub use super::track::{
    post_elevation_check, RoutePoint, RouteResult, MESSAGES_HEADER,
};
pub use super::voice::{TransportMode, VoiceHint, VoiceHintProcessor, WayMessage};

/// Extended routing options for multi-waypoint queries (`RoutingParamCollector`).
#[derive(Debug, Clone, Default)]
pub struct RoutingOptions<'a> {
    pub alternative_idx: usize,
    pub nogos: &'a [NogoArea],
    pub start_direction: Option<f64>,
    pub force_start_direction: bool,
    pub turn_instruction_mode: Option<i32>,
    pub direct_legs: &'a [bool],
}

/// A* router engine implementing 2-pass guided search.
#[derive(Default)]
pub struct AStarRouter {
    pub kinematic_cfg: KinematicConfig,
    pub elev_cfg: ElevationConfig,
}

pub type CostCuttingGuide<'a> = &'a Pass0CostGuide;

/// Configuration bundle for edge cost and physics evaluation.
#[derive(Clone, Copy)]
pub struct RoutingConfigs<'a> {
    pub elev: &'a ElevationConfig,
    pub kinematic: &'a KinematicConfig,
    pub car_kinematic: Option<&'a CarKinematicModel>,
    pub crossing: &'a CrossingConfig,
}

/// Context of the preceding path step when entering an edge.
#[derive(Clone, Copy, Debug)]
pub struct EdgeContext<'a> {
    pub crossing_class: Option<u8>,
    pub old_prio: i32,
    pub prev_point: Point2D,
    pub is_start: bool,
    pub is_detail: bool,
    pub pre_paths: &'a [KinematicPrePath],
}

/// Reference wrapper supporting both shared preloaded graphs and mutable lazy-loaded graphs.
pub enum GraphRef<'a> {
    Shared(&'a RoutingGraph),
    Mutable(&'a mut RoutingGraph),
}

impl<'a> std::ops::Deref for GraphRef<'a> {
    type Target = RoutingGraph;
    #[inline(always)]
    fn deref(&self) -> &RoutingGraph {
        match self {
            Self::Shared(g) => g,
            Self::Mutable(g) => g,
        }
    }
}

impl<'a> GraphRef<'a> {
    #[inline(always)]
    pub fn as_mut(&mut self) -> Option<&mut RoutingGraph> {
        match self {
            Self::Shared(_) => None,
            Self::Mutable(g) => Some(*g),
        }
    }
}

/// Global graph, lookup tables, and profile context for search passes.
pub struct EngineContext<'a> {
    pub graph: &'a RoutingGraph,
    pub lookups: &'a LookupData,
    pub profile: &'a Profile,
    pub cached_way_costs: &'a [crate::dsl::evaluator::WayCostResult],
}

/// Kinematic and elevation configuration bundle for a search pass.
#[derive(Clone, Copy)]
pub struct PassConfigs<'a> {
    pub kinematic: &'a KinematicConfig,
    pub car_kinematic: Option<&'a CarKinematicModel>,
    pub elev: &'a ElevationConfig,
}

/// Waypoint and heuristics parameters for a search pass.
pub struct SearchPassParams<'a> {
    pub start_mwp: &'a MatchedWaypoint,
    pub target_mwp: &'a MatchedWaypoint,
    pub air_distance_factor: f64,
    pub cost_cutting: Option<CostCuttingGuide<'a>>,
    pub ref_nodes: Option<&'a HashSet<NodeId>>,
    pub nogos: &'a [InternalNogo],
    pub start_direction: Option<f64>,
    pub force_start_direction: bool,
}

/// Resource and termination bounds for a search pass.
#[derive(Clone, Copy, Debug)]
pub struct SearchLimits {
    pub max_cost: Option<Cost>,
    pub edge_cap: usize,
    pub heap_cap: usize,
}

/// Retrieves pre-evaluated way costs for a given description ID and direction.
#[inline(always)]
fn get_way_cost(
    cached_way_costs: &[crate::dsl::evaluator::WayCostResult],
    desc_id: u32,
    is_reverse: bool,
) -> Option<&crate::dsl::evaluator::WayCostResult> {
    if desc_id == NO_DESC {
        None
    } else {
        let idx = ((desc_id as usize) << 1) | (is_reverse as usize);
        cached_way_costs.get(idx)
    }
}

/// Retrieves pre-evaluated node costs for a given node description ID, access permission, and foreign var state.
#[inline(always)]
fn get_node_cost(
    cached_node_costs: &[crate::dsl::evaluator::NodeCostResult],
    node_desc_id: u32,
    access_granted: bool,
    foreign_var_0_one: bool,
) -> Option<&crate::dsl::evaluator::NodeCostResult> {
    if node_desc_id == NO_DESC {
        None
    } else {
        let idx = ((node_desc_id as usize) << 2)
            | ((foreign_var_0_one as usize) << 1)
            | (access_granted as usize);
        cached_node_costs.get(idx)
    }
}

impl AStarRouter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Computes a continuous route passing sequentially through multiple waypoints.
    pub fn find_multi_route(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        waypoints: &[(f64, f64)],
    ) -> Result<RouteResult, String> {
        self.find_multi_route_with_alternative_and_nogos(graph, lookups, profile, waypoints, 0, &[])
    }

    /// Computes a continuous route passing sequentially through multiple waypoints with an alternative route index (0..=3).
    pub fn find_multi_route_with_alternative(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        waypoints: &[(f64, f64)],
        alternative_idx: usize,
    ) -> Result<RouteResult, String> {
        self.find_multi_route_with_alternative_and_nogos(
            graph,
            lookups,
            profile,
            waypoints,
            alternative_idx,
            &[],
        )
    }

    /// Computes a continuous route passing sequentially through multiple waypoints with an alternative route index (0..=3) and no-go areas.
    pub fn find_multi_route_with_alternative_and_nogos(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        waypoints: &[(f64, f64)],
        alternative_idx: usize,
        nogos: &[NogoArea],
    ) -> Result<RouteResult, String> {
        let options = RoutingOptions {
            alternative_idx,
            nogos,
            ..Default::default()
        };
        self.find_multi_route_with_options(graph, lookups, profile, waypoints, &options)
    }

    /// Computes a continuous route passing sequentially through multiple waypoints with lazy on-demand cell loading.
    pub fn find_multi_route_with_alternative_and_nogos_lazy(
        &self,
        graph: &mut RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        waypoints: &[(f64, f64)],
        alternative_idx: usize,
        nogos: &[NogoArea],
    ) -> Result<RouteResult, String> {
        let options = RoutingOptions {
            alternative_idx,
            nogos,
            ..Default::default()
        };
        let mut gref = GraphRef::Mutable(graph);
        self.find_multi_route_with_options_ref(&mut gref, lookups, profile, waypoints, &options)
    }

    /// Computes a continuous route passing sequentially through multiple waypoints with full routing options.
    pub fn find_multi_route_with_options(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        waypoints: &[(f64, f64)],
        options: &RoutingOptions<'_>,
    ) -> Result<RouteResult, String> {
        let mut gref = GraphRef::Shared(graph);
        self.find_multi_route_with_options_ref(&mut gref, lookups, profile, waypoints, &options)
    }

    pub fn find_multi_route_with_options_ref(
        &self,
        graph: &mut GraphRef<'_>,
        lookups: &LookupData,
        profile: &Profile,
        waypoints: &[(f64, f64)],
        options: &RoutingOptions<'_>,
    ) -> Result<RouteResult, String> {
        if waypoints.len() < 2 {
            return Err("At least two waypoints required for routing".to_string());
        }
        let timode = options
            .turn_instruction_mode
            .unwrap_or_else(|| profile.get_global_var("turnInstructionMode", 0.0) as i32);
        let kinematic_cfg = KinematicConfig::from_profile(profile);

        let mut legs = Vec::with_capacity(waypoints.len() - 1);
        for i in 0..waypoints.len() - 1 {
            let is_direct = options.direct_legs.get(i).copied().unwrap_or(false);
            let leg = if is_direct {
                self.find_beeline_leg(
                    waypoints[i],
                    waypoints[i + 1],
                    &kinematic_cfg,
                    timode,
                )
            } else {
                let leg_start_dir = if i == 0 {
                    options.start_direction
                } else {
                    None
                };
                self.find_route_leg_ref(
                    graph,
                    lookups,
                    profile,
                    waypoints[i],
                    waypoints[i + 1],
                    options.alternative_idx,
                    options.nogos,
                    leg_start_dir,
                    options.force_start_direction && i == 0,
                    timode,
                )?
            };
            legs.push(leg);
        }
        let mut merged = RouteResult::merge(legs)?;
        merged.turn_instruction_mode = timode;
        merged.recalc_track(graph, &kinematic_cfg);
        Ok(merged)
    }

    /// Computes a complete route between `start` and `target` waypoints.
    pub fn find_route(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        start: (f64, f64),
        target: (f64, f64),
    ) -> Result<RouteResult, String> {
        self.find_route_with_alternative_and_nogos(graph, lookups, profile, start, target, 0, &[])
    }

    /// Computes a complete route between `start` and `target` waypoints with an alternative route index (0..=3).
    pub fn find_route_with_alternative(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        start: (f64, f64),
        target: (f64, f64),
        alternative_idx: usize,
    ) -> Result<RouteResult, String> {
        self.find_route_with_alternative_and_nogos(
            graph,
            lookups,
            profile,
            start,
            target,
            alternative_idx,
            &[],
        )
    }

    /// Computes a complete route between `start` and `target` waypoints with an alternative route index (0..=3) and no-go areas.
    pub fn find_route_with_alternative_and_nogos(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        start: (f64, f64),
        target: (f64, f64),
        alternative_idx: usize,
        nogos: &[NogoArea],
    ) -> Result<RouteResult, String> {
        let timode = profile.get_global_var("turnInstructionMode", 0.0) as i32;
        let mut route = self.find_route_leg(
            graph,
            lookups,
            profile,
            start,
            target,
            alternative_idx,
            nogos,
            None,
            false,
            timode,
        )?;
        route.recalc_track(graph, &KinematicConfig::from_profile(profile));
        Ok(route)
    }

    fn find_beeline_leg(
        &self,
        start: (f64, f64),
        target: (f64, f64),
        kinematic_cfg: &KinematicConfig,
        timode: i32,
    ) -> RouteResult {
        let start_pt = Point2D::new(spatial::lon_to_ilon(start.0), spatial::lat_to_ilat(start.1));
        let target_pt = Point2D::new(
            spatial::lon_to_ilon(target.0),
            spatial::lat_to_ilat(target.1),
        );
        let dist_m = distance(start_pt, target_pt).round().max(1.0);
        let (speed, energy) =
            kinematic_cfg.compute_speed_and_energy(dist_m, 0.0, kinematic_cfg.max_speed);
        let time_s = if speed > 0.0 {
            (dist_m / speed) as f32
        } else {
            1.0
        };
        let energy_j = energy.max(0.0) as f32;
        let cost = dist_m as u32;

        let msg = WayMessage {
            linkdist: dist_m as i32,
            costfactor: 1.0,
            way_key_values: "beeline=yes".to_string(),
            lon: target_pt.lon,
            lat: target_pt.lat,
            ele: 0,
            time: time_s,
            energy: energy_j,
            ..Default::default()
        };

        let vh = VoiceHint {
            ilon: start_pt.lon,
            ilat: start_pt.lat,
            selev: 0,
            cmd: VoiceHint::BL,
            old_way: msg.clone(),
            good_way: msg.clone(),
            distance_to_next: dist_m,
            index_in_track: 0,
            angle: 0.0,
            ..Default::default()
        };

        RouteResult {
            total_distance_m: dist_m,
            total_time_s: time_s,
            total_energy_j: energy_j,
            total_cost: cost,
            filtered_ascend_m: 0.0,
            plain_ascend_m: 0.0,
            points: vec![
                RoutePoint {
                    lon: start.0,
                    lat: start.1,
                    elevation_m: 0.0,
                    distance_from_start_m: 0.0,
                    time_from_start_s: 0.0,
                    energy_j: 0.0,
                    cost: 0,
                },
                RoutePoint {
                    lon: target.0,
                    lat: target.1,
                    elevation_m: 0.0,
                    distance_from_start_m: dist_m,
                    time_from_start_s: time_s,
                    energy_j,
                    cost,
                },
            ],
            messages: vec![None, Some(msg)],
            voice_hints: if timode > 0 { vec![vh] } else { Vec::new() },
            turn_instruction_mode: timode,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn find_route_leg(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        start: (f64, f64),
        target: (f64, f64),
        alternative_idx: usize,
        nogos: &[NogoArea],
        start_direction: Option<f64>,
        force_start_direction: bool,
        turn_instruction_mode: i32,
    ) -> Result<RouteResult, String> {
        let mut gref = GraphRef::Shared(graph);
        self.find_route_leg_ref(
            &mut gref,
            lookups,
            profile,
            start,
            target,
            alternative_idx,
            nogos,
            start_direction,
            force_start_direction,
            turn_instruction_mode,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn find_route_leg_ref(
        &self,
        graph: &mut GraphRef<'_>,
        lookups: &LookupData,
        profile: &Profile,
        start: (f64, f64),
        target: (f64, f64),
        alternative_idx: usize,
        nogos: &[NogoArea],
        start_direction: Option<f64>,
        force_start_direction: bool,
        turn_instruction_mode: i32,
    ) -> Result<RouteResult, String> {
        let (start_lon, start_lat) = start;
        let (target_lon, target_lat) = target;
        let mut evaluator = ProfileEvaluator::new(profile, &lookups.way, &lookups.node);

        // 1. Snap waypoints
        let start_pt = Point2D::new(spatial::lon_to_ilon(start_lon), spatial::lat_to_ilat(start_lat));
        let target_pt = Point2D::new(spatial::lon_to_ilon(target_lon), spatial::lat_to_ilat(target_lat));
        let dir_start_to_target = start_direction
            .unwrap_or_else(|| spatial::get_direction(start_pt, target_pt));
        let dir_target_to_start = spatial::get_direction(target_pt, start_pt);

        let start_mwp = if let Some(g_mut) = graph.as_mut() {
            g_mut
                .snap_waypoint_with_direction_lazy(
                    "from",
                    start_lon,
                    start_lat,
                    250.0,
                    dir_start_to_target,
                    &mut evaluator,
                    lookups,
                )
                .or_else(|| {
                    g_mut.snap_waypoint_with_direction_lazy(
                        "from",
                        start_lon,
                        start_lat,
                        1_500.0,
                        dir_start_to_target,
                        &mut evaluator,
                        lookups,
                    )
                })
                .or_else(|| {
                    g_mut.snap_waypoint_with_direction_lazy(
                        "from",
                        start_lon,
                        start_lat,
                        10_000.0,
                        dir_start_to_target,
                        &mut evaluator,
                        lookups,
                    )
                })
        } else {
            graph
                .snap_waypoint_with_direction(
                    "from",
                    start_lon,
                    start_lat,
                    250.0,
                    dir_start_to_target,
                    &mut evaluator,
                    lookups,
                )
                .or_else(|| {
                    graph.snap_waypoint_with_direction(
                        "from",
                        start_lon,
                        start_lat,
                        10_000.0,
                        dir_start_to_target,
                        &mut evaluator,
                        lookups,
                    )
                })
        }
        .ok_or_else(|| "Start position could not be mapped to graph".to_string())?;

        let target_mwp = if let Some(g_mut) = graph.as_mut() {
            g_mut
                .snap_waypoint_with_direction_lazy(
                    "to",
                    target_lon,
                    target_lat,
                    250.0,
                    dir_target_to_start,
                    &mut evaluator,
                    lookups,
                )
                .or_else(|| {
                    g_mut.snap_waypoint_with_direction_lazy(
                        "to",
                        target_lon,
                        target_lat,
                        1_500.0,
                        dir_target_to_start,
                        &mut evaluator,
                        lookups,
                    )
                })
                .or_else(|| {
                    g_mut.snap_waypoint_with_direction_lazy(
                        "to",
                        target_lon,
                        target_lat,
                        10_000.0,
                        dir_target_to_start,
                        &mut evaluator,
                        lookups,
                    )
                })
        } else {
            graph
                .snap_waypoint_with_direction(
                    "to",
                    target_lon,
                    target_lat,
                    250.0,
                    dir_target_to_start,
                    &mut evaluator,
                    lookups,
                )
                .or_else(|| {
                    graph.snap_waypoint_with_direction(
                        "to",
                        target_lon,
                        target_lat,
                        10_000.0,
                        dir_target_to_start,
                        &mut evaluator,
                        lookups,
                    )
                })
        }
        .ok_or_else(|| "Target position could not be mapped to graph".to_string())?;

        if let Some(g_mut) = graph.as_mut() {
            if g_mut.has_cell_provider() {
                for id in [
                    start_mwp.node1.node_id(),
                    start_mwp.node2.node_id(),
                    target_mwp.node1.node_id(),
                    target_mwp.node2.node_id(),
                ] {
                    if id != 0 {
                        g_mut.ensure_node_finalized(id);
                    }
                }
            }
        }

        let cleaned_nogos = clean_nogos(
            nogos,
            &[
                start_mwp.crosspoint,
                target_mwp.crosspoint,
                start_pt,
                target_pt,
            ],
        );

        let kinematic_cfg = KinematicConfig::from_profile(profile);
        let car_kinematic_cfg = CarKinematicModel::from_profile(profile);
        let elev_cfg = ElevationConfig::from_profile(profile);

        let pass1_coeff = profile.get_global_var("pass1coefficient", 1.5) as f64;
        let pass2_coeff = profile.get_global_var("pass2coefficient", 0.0) as f64;

        let air_dist_km = distance(start_mwp.crosspoint, target_mwp.crosspoint) / 1000.0;
        let edge_cap = if air_dist_km < 1.0 {
            2_048
        } else if air_dist_km < 5.0 {
            16_384
        } else if air_dist_km < 25.0 {
            131_072
        } else if air_dist_km < 100.0 {
            1_048_576
        } else {
            (graph.edge_count() / 2).clamp(1_048_576, 4_194_304)
        };
        let heap_cap = (edge_cap / 4).clamp(1_024, 262_144);

        let pass_configs = PassConfigs {
            kinematic: &kinematic_cfg,
            car_kinematic: car_kinematic_cfg.as_ref(),
            elev: &elev_cfg,
        };

        let mut ref_nodes: HashSet<NodeId> = HashSet::new();
        let mut target_result = None;

        for alt in 0..=alternative_idx {
            let ref_param = if alt > 0 { Some(&ref_nodes) } else { None };

            // 2. Run Pass 0 (fast initial search with pass1coefficient)
            let pass0_params = SearchPassParams {
                start_mwp: &start_mwp,
                target_mwp: &target_mwp,
                air_distance_factor: pass1_coeff,
                cost_cutting: None,
                ref_nodes: ref_param,
                nogos: &cleaned_nogos,
                start_direction,
                force_start_direction,
            };
            let pass0_limits = SearchLimits {
                max_cost: None,
                edge_cap,
                heap_cap,
            };
            let pass0_track = self.search_pass(
                graph,
                lookups,
                profile,
                &pass_configs,
                &pass0_params,
                &pass0_limits,
            )?;

            let winning_track = if pass2_coeff >= 0.0 {
                let mut cost_guide = Pass0CostGuide::new(pass0_track.steps.len(), pass0_track.total_cost);
                for step in &pass0_track.steps {
                    cost_guide.record_edge(step.global_edge_idx, step.cost);
                }

                let cost_cutting_threshold = Some(pass0_track.total_cost + 5000);

                let pass1_params = SearchPassParams {
                    start_mwp: &start_mwp,
                    target_mwp: &target_mwp,
                    air_distance_factor: pass2_coeff,
                    cost_cutting: Some(&cost_guide),
                    ref_nodes: ref_param,
                    nogos: &cleaned_nogos,
                    start_direction,
                    force_start_direction,
                };
                let pass1_limits = SearchLimits {
                    max_cost: cost_cutting_threshold,
                    edge_cap,
                    heap_cap,
                };
                let pass1_res = self.search_pass(
                    graph,
                    lookups,
                    profile,
                    &pass_configs,
                    &pass1_params,
                    &pass1_limits,
                );

                match pass1_res {
                    Ok(pass1_track) => {
                        if pass1_track.total_cost <= pass0_track.total_cost {
                            pass1_track
                        } else {
                            pass0_track
                        }
                    }
                    Err(_) => pass0_track,
                }
            } else {
                pass0_track
            };

            let costs = graph.sync_profile_costs(profile, lookups);

            // 3. Run Pass 2 (re-tracking): walk the winning path, decode geometry, evaluate kinematics
            let route = self.retrack_path(
                graph,
                lookups,
                profile,
                &costs.way_costs,
                &winning_track.steps,
                winning_track.total_cost,
                &kinematic_cfg,
                car_kinematic_cfg.as_ref(),
                turn_instruction_mode,
            )?;

            if alt < alternative_idx {
                for step in &winning_track.steps {
                    if step.from_node_id != 0 {
                        ref_nodes.insert(step.from_node_id);
                    }
                    if step.node_id != 0 {
                        ref_nodes.insert(step.node_id);
                    }
                }
                for pt in &route.points {
                    let id = Point2D::new(spatial::lon_to_ilon(pt.lon), spatial::lat_to_ilat(pt.lat)).node_id();
                    ref_nodes.insert(id);
                }
            }

            if alt == alternative_idx {
                target_result = Some(route);
                break;
            }
        }

        target_result.ok_or_else(|| "Failed to calculate route".to_string())
    }

    fn search_pass<'a>(
        &self,
        graph: &mut GraphRef<'_>,
        lookups: &'a LookupData,
        profile: &'a Profile,
        configs: &PassConfigs<'a>,
        params: &SearchPassParams<'a>,
        limits: &SearchLimits,
    ) -> Result<SearchTrack, String> {
        let mut ctx = SearchPassContext::new(
            graph,
            lookups,
            profile,
            configs,
            params,
        );

        let total_edges = ctx.graph.total_edges;
        let mut state = SearchPassState::new(
            &ctx,
            profile,
            limits.max_cost,
            total_edges,
            limits.edge_cap,
            limits.heap_cap,
        );
        state.seed_pass(&ctx)?;
        state.run_search(&mut ctx)?;
        state.reconstruct_track(&ctx)
    }

    #[allow(clippy::too_many_arguments)]
    fn retrack_path(
        &self,
        graph: &RoutingGraph,
        lookups: &LookupData,
        profile: &Profile,
        cached_way_costs: &[crate::dsl::evaluator::WayCostResult],
        steps: &[TrackStep],
        final_cost: u32,
        kinematic_cfg: &KinematicConfig,
        car_kinematic: Option<&CarKinematicModel>,
        turn_instruction_mode: i32,
    ) -> Result<RouteResult, String> {
        let mut route_points: Vec<RoutePoint> = Vec::new();
        let mut route_messages: Vec<Option<WayMessage>> = Vec::new();
        let mut detours_by_track_idx: FastHashMap<usize, Vec<WayMessage>> = FastHashMap::default();
        let mut pts_2d: Vec<Point2D> = Vec::new();
        let mut raw_selevs: Vec<i16> = Vec::new();
        let mut cum_dist = 0.0;
        let mut cum_time = 0.0f32;
        let mut cum_energy = 0.0f32;

        let mut carried_selev = i16::MIN;
        let mut prev_pt_pos: Option<Point2D> = None;
        let mut last_pt_pos: Option<Point2D> = None;

        let mut detail_state = PathState::default();
        let mut last_prio = 0i32;
        let mut pre_paths_buf: Vec<KinematicPrePath> = Vec::new();
        let mut tmp_poly: Vec<Point3D> = Vec::new();
        let mut way_buf = vec![0i32; lookups.way.names.len()];
        let mut node_buf = vec![0i32; lookups.node.names.len()];
        let mut retrack_eval = car_kinematic
            .map(|_| ProfileEvaluator::new(profile, &lookups.way, &lookups.node));

        for (s_i, step) in steps.iter().enumerate() {
            let mut step_way_cost = None;
            let mut step_edge_idx = u32::MAX;
            let mut endpoint_origin_ele2: Option<i16> = None;
            let mut way_kv_str = String::new();
            {
                let edges = graph.get_edges(step.from_node_id);
                let edge = if (step.edge_idx as usize) < edges.len()
                    && (step.node_id == 0 || edges[step.edge_idx as usize].target_id == step.node_id)
                {
                    Some(&edges[step.edge_idx as usize])
                } else {
                    edges.iter().find(|e| {
                        e.edge_idx == step.global_edge_idx
                            || (step.node_id != 0 && e.target_id == step.node_id)
                    })
                };

                if let Some(e) = edge {
                    step_edge_idx = e.edge_idx;
                    step_way_cost = get_way_cost(cached_way_costs, e.desc_id, e.is_reverse);
                    if s_i == steps.len() - 1 {
                        if let (Some(ref sliced), Some(from_node)) = (
                            step.sliced_points.as_ref(),
                            graph.nodes.get(&step.from_node_id),
                        ) {
                            if let Some(last_pt) = sliced.last() {
                                let t_elev = if e.target_selev != i16::MIN {
                                    e.target_selev
                                } else {
                                    graph
                                        .nodes
                                        .get(&e.target_id)
                                        .map(|n| n.selev)
                                        .unwrap_or(i16::MIN)
                                };
                                e.fill_polyline(
                                    &mut tmp_poly,
                                    from_node.ilon,
                                    from_node.ilat,
                                    from_node.selev,
                                    t_elev,
                                );
                                if tmp_poly.len() >= 2 {
                                    let (best_idx, _, _) = super::slicing::find_crosspoint_projection(
                                        &tmp_poly,
                                        last_pt.to_2d(),
                                    );
                                    if best_idx + 1 < tmp_poly.len() {
                                        endpoint_origin_ele2 = Some(tmp_poly[best_idx + 1].elev);
                                    }
                                }
                            }
                        }
                    }
                    if let Some(desc_bytes) = e.description(graph) {
                        lookups.way.decode_into(&mut way_buf, e.is_reverse, desc_bytes);
                        way_kv_str = lookups.way.format_key_values(&way_buf);
                    }
                }
            }

            let poly = if let Some(ref sliced) = step.sliced_points {
                sliced.clone()
            } else {
                let from_node = graph.nodes.get(&step.from_node_id).ok_or_else(|| {
                    format!(
                        "From node {} not found in graph during retrack",
                        step.from_node_id
                    )
                })?;
                let to_node = graph.nodes.get(&step.node_id).ok_or_else(|| {
                    format!(
                        "Target node {} not found in graph during retrack",
                        step.node_id
                    )
                })?;
                let edges = graph.get_edges(step.from_node_id);
                if edges.is_empty() {
                    return Err(format!(
                        "Outgoing edges for node {} not found during retrack",
                        step.from_node_id
                    ));
                }
                let edge = if (step.edge_idx as usize) < edges.len()
                    && edges[step.edge_idx as usize].target_id == step.node_id
                {
                    &edges[step.edge_idx as usize]
                } else {
                    edges
                        .iter()
                        .find(|e| e.target_id == step.node_id)
                        .ok_or_else(|| {
                            format!(
                                "Edge from {} to {} not found during retrack",
                                step.from_node_id, step.node_id
                            )
                        })?
                };
                edge.polyline(
                    from_node.ilon,
                    from_node.ilat,
                    from_node.selev,
                    to_node.selev,
                )
            };

            let junction_track_idx = route_points.len().saturating_sub(1);
            pre_paths_buf.clear();
            if (car_kinematic.is_some() || turn_instruction_mode > 0)
                && s_i > 0
                && step.from_node_id != 0
            {
                if let (Some(prev_pos), Some(from_node)) =
                    (prev_pt_pos, graph.nodes.get(&step.from_node_id))
                {
                    let prev_from_id = steps[s_i - 1].from_node_id;
                    let p1 = Point2D::new(from_node.ilon, from_node.ilat);
                    let mut junction_detours = Vec::new();
                    for e in graph.get_edges(step.from_node_id) {
                        if prev_from_id != 0 && e.target_id == prev_from_id {
                            continue;
                        }
                        if let Some(wc) = get_way_cost(cached_way_costs, e.desc_id, e.is_reverse) {
                            let p2 = if e.geometry.is_empty() {
                                Point2D::new(e.target_lon, e.target_lat)
                            } else {
                                e.fill_polyline(
                                    &mut tmp_poly,
                                    from_node.ilon,
                                    from_node.ilat,
                                    from_node.selev,
                                    e.target_selev,
                                );
                                if tmp_poly.len() >= 2 {
                                    tmp_poly[1].to_2d()
                                } else {
                                    Point2D::new(e.target_lon, e.target_lat)
                                }
                            };
                            let (ang, _) = calc_angle(prev_pos, p1, p2);
                            if car_kinematic.is_some() {
                                pre_paths_buf.push(KinematicPrePath {
                                    angle: ang,
                                    priorityclassifier: wc.priorityclassifier,
                                    classifiermask: wc.classifiermask,
                                });
                            }
                            if turn_instruction_mode > 0
                                && e.edge_idx != step_edge_idx
                                && (wc.priorityclassifier > 0
                                    || wc.costfactor < 9999.0
                                    || (wc.classifiermask & 1) != 0)
                            {
                                let alt_dist = distance(p1, p2).round().max(1.0) as i32;
                                junction_detours.push(WayMessage {
                                    linkdist: alt_dist,
                                    costfactor: wc.costfactor,
                                    priorityclassifier: wc.priorityclassifier,
                                    classifiermask: wc.classifiermask,
                                    turnangle: ang as f32,
                                    ..Default::default()
                                });
                            }
                        }
                    }
                    if !junction_detours.is_empty() {
                        detours_by_track_idx.insert(junction_track_idx, junction_detours);
                    }
                }
            }

            let target_node_kv = if step.node_id != 0 {
                if let Some(target_node) = graph.nodes.get(&step.node_id) {
                    if let Some(node_desc) = graph.node_descriptions.get(target_node.node_desc_id as usize) {
                        let node_access = if car_kinematic.is_some() {
                            false
                        } else {
                            step_way_cost.map_or(false, |wc| wc.nodeaccessgranted)
                        };
                        lookups.node.decode_into(&mut node_buf, node_access, node_desc);
                        lookups.node.format_key_values(&node_buf)
                    } else {
                        String::new()
                    }
                } else {
                    String::new()
                }
            } else {
                String::new()
            };

            let mut nsection = 0usize;
            for (p_i, pt) in poly.iter().enumerate() {
                if p_i == 0 && s_i > 0 {
                    continue; // skip duplicate edge junction
                }
                let lon = ilon_to_lon(pt.lon);
                let lat = ilat_to_lat(pt.lat);
                let mut ele2 = pt.elev;
                if ele2 == i16::MIN {
                    ele2 = carried_selev;
                }

                let mut seg_dist_int = 0i32;
                let mut turn_angle_deg = 0.0f32;
                let mut turn_cost_int = 0i32;

                if let Some(last_pos) = last_pt_pos {
                    let seg_d = distance(last_pos, pt.to_2d())
                        .round()
                        .max(1.0);
                    seg_dist_int = seg_d as i32;
                    cum_dist += seg_d;

                    let dh = if carried_selev == i16::MIN {
                        0.0
                    } else {
                        (ele2 - carried_selev) as f64 / 4.0
                    };

                    let is_startpoint = s_i == 0 && nsection == 0;
                    let (angle, cos_angle) = if !is_startpoint {
                        if let Some(prev_pos) = prev_pt_pos {
                            calc_angle(prev_pos, last_pos, pt.to_2d())
                        } else {
                            (0.0, 1.0)
                        }
                    } else {
                        (0.0, 1.0)
                    };
                    turn_angle_deg = angle as f32;
                    if let Some(wc) = step_way_cost {
                        if !is_startpoint && wc.turncost > 0.0 {
                            turn_cost_int =
                                ((1.0 - cos_angle) * 0.5 * (wc.turncost as f64) + 0.2) as i32;
                        }
                    }

                    if let (Some(car_model), Some(way_cost)) = (car_kinematic, step_way_cost) {
                        let elev_m = if ele2 == i16::MIN {
                            100.0
                        } else {
                            selev_to_elevation(ele2)
                        };
                        if let Some(sec) = StdPathCalculator::process_car_section(
                            &mut detail_state,
                            way_cost,
                            car_model,
                            seg_d,
                            dh,
                            elev_m,
                            angle,
                            cos_angle,
                            is_startpoint,
                            nsection,
                            last_prio,
                            &pre_paths_buf,
                        ) {
                            cum_time += sec.time_seconds;
                            cum_energy += sec.energy_joules;
                        }
                    } else {
                        let incline = StdPathCalculator::calc_incline(
                            &mut detail_state.elevation_buffer,
                            dh,
                            seg_d,
                        );
                        let (speed, energy) = kinematic_cfg.compute_speed_and_energy(
                            seg_d,
                            incline,
                            step.maxspeed as f64 / 3.6,
                        );
                        let dt = (seg_d / speed) as f32;
                        cum_time += dt;
                        if energy > 0.0 {
                            cum_energy = (cum_energy as f64 + energy) as f32;
                        }
                    }
                    prev_pt_pos = Some(last_pos);
                    nsection += 1;
                }
                let is_first_point = last_pt_pos.is_none();
                last_pt_pos = Some(pt.to_2d());
                carried_selev = ele2;
                pts_2d.push(pt.to_2d());
                let recorded_selev = if s_i == steps.len() - 1 && p_i == poly.len() - 1 {
                    endpoint_origin_ele2.unwrap_or(pt.elev)
                } else {
                    pt.elev
                };
                raw_selevs.push(recorded_selev);

                if is_first_point {
                    route_messages.push(None);
                } else {
                    let is_step_end = p_i == poly.len() - 1;
                    route_messages.push(Some(WayMessage {
                        linkdist: seg_dist_int,
                        linkelevationcost: 0,
                        linkturncost: turn_cost_int,
                        linknodecost: 0,
                        linkinitcost: 0,
                        costfactor: step_way_cost.map_or(1.0, |wc| wc.costfactor),
                        priorityclassifier: step_way_cost.map_or(0, |wc| wc.priorityclassifier),
                        classifiermask: step_way_cost.map_or(0, |wc| wc.classifiermask),
                        turnangle: turn_angle_deg,
                        way_key_values: way_kv_str.clone(),
                        node_key_values: if is_step_end {
                            target_node_kv.clone()
                        } else {
                            String::new()
                        },
                        lon: pt.lon,
                        lat: pt.lat,
                        ele: recorded_selev,
                        time: cum_time,
                        energy: cum_energy,
                    }));
                }

                route_points.push(RoutePoint {
                    lon,
                    lat,
                    elevation_m: selev_to_elevation(recorded_selev),
                    distance_from_start_m: cum_dist,
                    time_from_start_s: cum_time,
                    energy_j: cum_energy,
                    cost: step.cost,
                });
            }
            if let Some(wc) = step_way_cost {
                last_prio = wc.priorityclassifier;
            }
            if let (Some(car_model), Some(ref mut eval)) = (car_kinematic, retrack_eval.as_mut()) {
                if step.node_id != 0 {
                    if let Some(target_node) = graph.nodes.get(&step.node_id) {
                        if let Some(node_desc) = graph.node_descriptions.get(target_node.node_desc_id as usize) {
                            eval.evaluate_way(&way_buf);
                            lookups.node.decode_into(&mut node_buf, false, node_desc);
                            let node_res = eval.evaluate_node(&node_buf);
                            car_model.cut_ekin(
                                &mut detail_state.ekin,
                                (node_res.maxspeed as f64) / 3.6,
                            );
                        }
                    }
                }
            }
        }

        post_elevation_check(&pts_2d, &mut raw_selevs);
        for (idx, (pt, &selev)) in route_points.iter_mut().zip(raw_selevs.iter()).enumerate() {
            pt.elevation_m = selev_to_elevation(selev);
            if let Some(Some(msg)) = route_messages.get_mut(idx) {
                msg.ele = selev;
            }
        }

        let voice_hints = if turn_instruction_mode > 0 && route_points.len() >= 2 {
            let last_idx = route_points.len() - 1;
            let last_msg = route_messages[last_idx].clone().unwrap_or_default();
            let mut inputs = Vec::with_capacity(route_points.len());
            inputs.push(VoiceHint {
                ilon: pts_2d[last_idx].lon,
                ilat: pts_2d[last_idx].lat,
                selev: raw_selevs[last_idx],
                good_way: last_msg.clone(),
                old_way: last_msg,
                index_in_track: last_idx,
                cmd: VoiceHint::END,
                ..Default::default()
            });
            for idx in (0..last_idx).rev() {
                let good_way = route_messages[idx + 1].clone().unwrap_or_default();
                let old_way = route_messages[idx]
                    .clone()
                    .unwrap_or_else(|| good_way.clone());
                let bad_ways = detours_by_track_idx.remove(&idx).unwrap_or_default();
                inputs.push(VoiceHint {
                    ilon: pts_2d[idx].lon,
                    ilat: pts_2d[idx].lat,
                    selev: raw_selevs[idx],
                    index_in_track: idx,
                    good_way,
                    old_way,
                    bad_ways,
                    ..Default::default()
                });
            }

            let valid_for_foot = profile.get_global_var("validForFoot", 0.0) != 0.0;
            let valid_for_cars = profile.get_global_var(
                "validForCars",
                if profile.name.starts_with("car") { 1.0 } else { 0.0 },
            ) != 0.0;
            let valid_for_bikes = profile.get_global_var(
                "validForBikes",
                if !valid_for_foot && !valid_for_cars { 1.0 } else { 0.0 },
            ) != 0.0;
            let transport_mode = TransportMode::from_flags(valid_for_cars, valid_for_bikes);
            let catching_range =
                profile.get_global_var("turnInstructionCatchingRange", 40.0) as f64;
            let explicit_roundabouts =
                profile.get_global_var("turnInstructionRoundabouts", 1.0) != 0.0;

            let vproc = VoiceHintProcessor::new(explicit_roundabouts, transport_mode);
            let raw_hints = vproc.process(inputs);
            vproc.post_process(raw_hints, catching_range, transport_mode.min_distance())
        } else {
            Vec::new()
        };

        let mut result = RouteResult {
            total_distance_m: cum_dist,
            total_time_s: cum_time,
            total_energy_j: cum_energy,
            total_cost: final_cost,
            filtered_ascend_m: 0.0,
            plain_ascend_m: 0.0,
            points: route_points,
            messages: route_messages,
            voice_hints,
            turn_instruction_mode,
        };
        result.recalc_track_elevation(graph);

        Ok(result)
    }
}

/// Configuration and references for a single A* search pass.
struct SearchPassContext<'a, 'g, 'r> {
    graph: &'r mut GraphRef<'g>,
    lookups: &'a LookupData,
    profile: &'a Profile,
    costs: std::sync::Arc<crate::graph::routing_graph::ProfileCostTable>,
    node_foreign_var_count: usize,
    node_foreign_var_0: Option<usize>,
    start_mwp: &'a MatchedWaypoint,
    target_mwp: &'a MatchedWaypoint,
    start_id1: NodeId,
    start_id2: NodeId,
    target_id1: NodeId,
    target_id2: NodeId,
    air_distance_factor: f64,
    kinematic_cfg: &'a KinematicConfig,
    car_kinematic: Option<&'a CarKinematicModel>,
    elev_cfg: &'a ElevationConfig,
    crossing_cfg: CrossingConfig,
    consider_tr: bool,
    is_bike_mode: bool,
    is_car_mode: bool,
    cost_cutting: Option<CostCuttingGuide<'a>>,
    ref_nodes: Option<&'a HashSet<NodeId>>,
    protected_nodes: Vec<NodeId>,
    nogos: &'a [InternalNogo],
    start_direction: Option<f64>,
    force_start_direction: bool,
}

impl<'a, 'g, 'r> SearchPassContext<'a, 'g, 'r> {
    #[inline(always)]
    fn routing_configs(&self) -> RoutingConfigs<'_> {
        RoutingConfigs {
            elev: self.elev_cfg,
            kinematic: self.kinematic_cfg,
            car_kinematic: self.car_kinematic,
            crossing: &self.crossing_cfg,
        }
    }

    fn new(
        graph: &'r mut GraphRef<'g>,
        lookups: &'a LookupData,
        profile: &'a Profile,
        configs: &PassConfigs<'a>,
        params: &SearchPassParams<'a>,
    ) -> Self {
        let costs = graph.sync_profile_costs(profile, lookups);
        let start_id1 = params.start_mwp.node1.node_id();
        let start_id2 = params.start_mwp.node2.node_id();
        let target_id1 = params.target_mwp.node1.node_id();
        let target_id2 = params.target_mwp.node2.node_id();

        let crossing_cfg = CrossingConfig::from_profile(profile);
        let valid_for_foot = profile.get_global_var("validForFoot", 0.0) != 0.0;
        let valid_for_cars = profile.get_global_var(
            "validForCars",
            if profile.name.starts_with("car") {
                1.0
            } else {
                0.0
            },
        ) != 0.0;
        let valid_for_bikes = profile.get_global_var(
            "validForBikes",
            if !valid_for_foot && !valid_for_cars {
                1.0
            } else {
                0.0
            },
        ) != 0.0;
        let consider_tr = profile.get_global_var(
            "considerTurnRestrictions",
            if valid_for_foot { 0.0 } else { 1.0 },
        ) != 0.0;
        let is_car_mode = valid_for_cars;
        let is_bike_mode = valid_for_bikes || valid_for_foot;

        let mut protected_nodes: Vec<u64> = Vec::with_capacity(16);
        for mut id in [target_id1, target_id2, start_id1, start_id2] {
            while let Some(&p) = graph.dead_end_parents.get(&id) {
                if p == 0 || protected_nodes.contains(&id) {
                    break;
                }
                protected_nodes.push(id);
                id = p;
            }
        }

        let f_vars = profile.node_block.foreign_var_indices();
        let node_foreign_var_count = f_vars.len();
        let node_foreign_var_0 = f_vars.into_iter().next();

        Self {
            graph,
            lookups,
            profile,
            costs,
            node_foreign_var_count,
            node_foreign_var_0,
            start_mwp: params.start_mwp,
            target_mwp: params.target_mwp,
            start_id1,
            start_id2,
            target_id1,
            target_id2,
            air_distance_factor: params.air_distance_factor,
            kinematic_cfg: configs.kinematic,
            car_kinematic: configs.car_kinematic,
            elev_cfg: configs.elev,
            crossing_cfg,
            consider_tr,
            is_bike_mode,
            is_car_mode,
            cost_cutting: params.cost_cutting,
            ref_nodes: params.ref_nodes,
            protected_nodes,
            nogos: params.nogos,
            start_direction: params.start_direction,
            force_start_direction: params.force_start_direction,
        }
    }
}

/// Mutable search state for a single A* search pass.
struct SearchPassState<'a> {
    evaluator: ProfileEvaluator<'a>,
    open_set: QuadHeap,
    node_pool: NodePool,
    edge_heads: Vec<u32>,
    closed_edges: Vec<u64>,
    edge_states_pool: Vec<EdgeState>,
    history: Vec<HistoryStep>,
    sliced_map: FastHashMap<u32, Vec<Point3D>>,
    poly_buf: Vec<Point3D>,
    node_lookup_buf: Vec<i32>,
    way_lookup_buf: Vec<i32>,
    max_cost_limit: Option<u32>,
    final_cost: u32,
    best_target_history: Option<u32>,
    nodes_popped: usize,
    edges_evaluated: usize,
}

impl<'a> SearchPassState<'a> {
    fn new(
        ctx: &SearchPassContext<'a, '_, '_>,
        profile: &'a Profile,
        max_cost_limit: Option<u32>,
        total_edges: usize,
        edge_cap: usize,
        heap_cap: usize,
    ) -> Self {
        let num_edge_words = total_edges.div_ceil(64);
        Self {
            evaluator: ProfileEvaluator::new(profile, &ctx.lookups.way, &ctx.lookups.node),
            open_set: QuadHeap::with_capacity(heap_cap),
            node_pool: NodePool::with_capacity(heap_cap),
            edge_heads: vec![u32::MAX; total_edges],
            closed_edges: vec![0u64; num_edge_words],
            edge_states_pool: Vec::with_capacity(edge_cap),
            history: Vec::with_capacity(edge_cap),
            sliced_map: FastHashMap::default(),
            poly_buf: Vec::with_capacity(64),
            node_lookup_buf: vec![0i32; ctx.lookups.node.names.len()],
            way_lookup_buf: vec![0i32; ctx.lookups.way.names.len()],
            max_cost_limit,
            final_cost: u32::MAX,
            best_target_history: None,
            nodes_popped: 0,
            edges_evaluated: 0,
        }
    }

    #[inline(always)]
    fn is_edge_closed(&self, edge_idx: u32) -> bool {
        let idx = edge_idx as usize;
        let word = idx >> 6;
        word < self.closed_edges.len() && (self.closed_edges[word] & (1u64 << (idx & 63))) != 0
    }

    #[inline(always)]
    fn mark_edge_closed(&mut self, edge_idx: u32) {
        let idx = edge_idx as usize;
        let word = idx >> 6;
        if word >= self.closed_edges.len() {
            self.closed_edges.resize(word + 1, 0u64);
        }
        self.closed_edges[word] |= 1u64 << (idx & 63);
    }

    #[inline(always)]
    fn get_edge_state(&self, edge_idx: u32) -> Option<&EdgeState> {
        let entry_idx = self
            .edge_heads
            .get(edge_idx as usize)
            .copied()
            .unwrap_or(u32::MAX);
        if entry_idx != u32::MAX {
            Some(&self.edge_states_pool[entry_idx as usize])
        } else {
            None
        }
    }

    #[inline(always)]
    fn has_edge_state(&self, edge_idx: u32) -> bool {
        self.edge_heads
            .get(edge_idx as usize)
            .copied()
            .unwrap_or(u32::MAX)
            != u32::MAX
    }

    #[inline(always)]
    fn get_or_create_edge_state(&mut self, edge_idx: u32) -> &mut EdgeState {
        let idx = edge_idx as usize;
        if idx >= self.edge_heads.len() {
            self.edge_heads.resize(idx + 1, u32::MAX);
        }
        let entry_idx = self.edge_heads[idx];
        if entry_idx != u32::MAX {
            &mut self.edge_states_pool[entry_idx as usize]
        } else {
            let new_idx = self.edge_states_pool.len() as u32;
            self.edge_states_pool.push(EdgeState::default());
            self.edge_heads[idx] = new_idx;
            &mut self.edge_states_pool[new_idx as usize]
        }
    }

    /// Seeds the search open set from start waypoint projections and checks same-segment shortcuts.
    fn seed_pass(&mut self, ctx: &SearchPassContext) -> Result<(), String> {
        // 1. Check if start and target are on the same segment
        let same_segment = (ctx.start_id1 == ctx.target_id1 && ctx.start_id2 == ctx.target_id2)
            || (ctx.start_id1 == ctx.target_id2 && ctx.start_id2 == ctx.target_id1);

        if same_segment {
            for &(s_id, t_id, s_node_pt) in &[
                (ctx.start_id1, ctx.start_id2, ctx.start_mwp.node1),
                (ctx.start_id2, ctx.start_id1, ctx.start_mwp.node2),
            ] {
                let edges = ctx.graph.get_edges(s_id);
                for (edge_idx, edge) in edges.iter().enumerate() {
                    if edge.target_id == t_id {
                        let Some(way_cost) =
                            get_way_cost(&ctx.costs.way_costs, edge.desc_id, edge.is_reverse)
                        else {
                            continue;
                        };
                        if way_cost.costfactor > 9998.0 {
                            continue;
                        }

                        let s_elev = ctx
                            .graph
                            .nodes
                            .get(&s_id)
                            .map(|n| n.selev)
                            .unwrap_or(i16::MIN);
                        let t_elev = ctx
                            .graph
                            .nodes
                            .get(&t_id)
                            .map(|n| n.selev)
                            .unwrap_or(i16::MIN);
                        let full_poly = edge.polyline(
                            s_node_pt.lon,
                            s_node_pt.lat,
                            s_elev,
                            t_elev,
                        );
                        if let Some(sliced) = slice_between_crosspoints(
                            &full_poly,
                            ctx.start_mwp.crosspoint,
                            ctx.target_mwp.crosspoint,
                        ) {
                            if sliced.len() >= 2 {
                                let nogo_res = check_polyline_against_nogos(&sliced, ctx.nogos);
                                let nogo_cost = match nogo_res {
                                    NogoCheckResult::Blocked => continue,
                                    NogoCheckResult::Pass { added_cost } => added_cost,
                                };
                                let mut path_state = PathState::default();
                                let mut path_selev = sliced[0].elev;
                                let edge_ctx = EdgeContext {
                                    crossing_class: None,
                                    old_prio: 0,
                                    prev_point: sliced[0].to_2d(),
                                    is_start: true,
                                    is_detail: false,
                                    pre_paths: &[],
                                };
                                let (mut cost, _, _, _, _) = process_polyline(
                                    &sliced,
                                    &mut path_state,
                                    &mut path_selev,
                                    way_cost,
                                    &ctx.routing_configs(),
                                    &edge_ctx,
                                );
                                if cost == u32::MAX {
                                    continue;
                                }
                                if way_cost.istrafficbackbone {
                                    cost = 0;
                                }
                                let cost = cost + nogo_cost;
                                if cost < self.final_cost {
                                    self.final_cost = cost;
                                    let h_idx = self.history.len() as u32;
                                    self.sliced_map.insert(h_idx, sliced.clone());
                                    self.history.push(HistoryStep {
                                        node_id: 0,
                                        from_node_id: s_id,
                                        cost,
                                        global_edge_idx: edge.edge_idx,
                                        edge_idx: edge_idx as u16,
                                        maxspeed: way_cost.maxspeed,
                                        prev_idx: u32::MAX,
                                    });
                                    let node_idx = self.node_pool.alloc(SearchNode {
                                        node_id: 0,
                                        from_node_id: s_id,
                                        ilon: ctx.target_mwp.crosspoint.lon,
                                        ilat: ctx.target_mwp.crosspoint.lat,
                                        cost,
                                        air_dist: 0,
                                        priority: cost,
                                        path_state,
                                        prev_lon: sliced[0].lon,
                                        prev_lat: sliced[0].lat,
                                        priority_classifier: 0,
                                        down_div: 0,
                                        up_div: 0,
                                        history_idx: h_idx,
                                        selev: 0,
                                        path_selev,
                                        last_classifier: 0,
                                        in_destination: false,
                                        can_leave_destination: false,
                                        has_tr: false,
                                        edge_idx: edge.edge_idx,
                                        rev_edge_idx: edge.rev_edge_idx,
                                        out_edges_start: 0,
                                        out_edges_count: 0,
                                        node_desc_id: NO_DESC,
                                    });
                                    self.open_set.push(HeapItem {
                                        priority: cost,
                                        node_idx,
                                    });
                                    self.best_target_history = Some(h_idx);
                                }
                            }
                        }
                    }
                }
            }
        }

        // 2. Seed start search paths
        if ctx.start_id1 == ctx.start_id2 {
            if let Some(node) = ctx.graph.nodes.get(&ctx.start_id1) {
                let air_dist = distance(
                    Point2D::new(node.ilon, node.ilat),
                    ctx.target_mwp.crosspoint,
                );
                let priority = (air_dist * ctx.air_distance_factor) as u32;

                let h_idx = self.history.len() as u32;
                self.sliced_map
                    .insert(h_idx, vec![Point3D::new(node.ilon, node.ilat, node.selev)]);
                self.history.push(HistoryStep {
                    node_id: ctx.start_id1,
                    from_node_id: 0,
                    cost: 0,
                    global_edge_idx: u32::MAX,
                    edge_idx: 0,
                    maxspeed: 0.0,
                    prev_idx: u32::MAX,
                });

                let (out_start, out_count) = ctx.graph.get_node_edge_slice(ctx.start_id1);
                let node_idx = self.node_pool.alloc(SearchNode {
                    node_id: ctx.start_id1,
                    from_node_id: 0,
                    ilon: node.ilon,
                    ilat: node.ilat,
                    cost: 0,
                    air_dist: air_dist as u32,
                    priority,
                    path_state: PathState::default(),
                    prev_lon: node.ilon,
                    prev_lat: node.ilat,
                    priority_classifier: 0,
                    down_div: 0,
                    up_div: 0,
                    history_idx: h_idx,
                    selev: node.selev,
                    path_selev: node.selev,
                    last_classifier: 0,
                    in_destination: false,
                    can_leave_destination: false,
                    has_tr: !node.turn_restrictions.is_empty(),
                    edge_idx: u32::MAX,
                    rev_edge_idx: u32::MAX,
                    out_edges_start: out_start,
                    out_edges_count: out_count,
                    node_desc_id: node.node_desc_id,
                });
                self.open_set.push(HeapItem { priority, node_idx });
            }
        } else {
            for &(s_id, t_id, s_node_pt, t_node_pt) in &[
                (
                    ctx.start_id1,
                    ctx.start_id2,
                    ctx.start_mwp.node1,
                    ctx.start_mwp.node2,
                ),
                (
                    ctx.start_id2,
                    ctx.start_id1,
                    ctx.start_mwp.node2,
                    ctx.start_mwp.node1,
                ),
            ] {
                let edges = ctx.graph.get_edges(s_id);
                for (edge_idx, edge) in edges.iter().enumerate() {
                    if edge.target_id == t_id {
                        let Some(way_cost) =
                            get_way_cost(&ctx.costs.way_costs, edge.desc_id, edge.is_reverse)
                        else {
                            continue;
                        };
                        if way_cost.costfactor > 9998.0 {
                            continue;
                        }

                        let s_node = ctx.graph.nodes.get(&s_id);
                        let t_node = ctx.graph.nodes.get(&t_id);
                        let s_elev = s_node.map(|n| n.selev).unwrap_or(i16::MIN);
                        let t_elev = t_node.map(|n| n.selev).unwrap_or(i16::MIN);
                        let full_poly = edge.polyline(
                            s_node_pt.lon,
                            s_node_pt.lat,
                            s_elev,
                            t_elev,
                        );
                        let sliced = slice_from_crosspoint(
                            &full_poly,
                            ctx.start_mwp.crosspoint,
                        );
                        if sliced.len() >= 2 {
                            let nogo_res = check_polyline_against_nogos(&sliced, ctx.nogos);
                            let nogo_cost = match nogo_res {
                                NogoCheckResult::Blocked => continue,
                                NogoCheckResult::Pass { added_cost } => added_cost,
                            };
                            let mut path_state = PathState::default();
                            let mut path_selev = sliced[0].elev;
                            let edge_ctx = EdgeContext {
                                crossing_class: None,
                                old_prio: 0,
                                prev_point: sliced[0].to_2d(),
                                is_start: true,
                                is_detail: false,
                                pre_paths: &[],
                            };
                            let (mut cost, _, _, _, penultimate_pt) =
                                process_polyline(
                                    &sliced,
                                    &mut path_state,
                                    &mut path_selev,
                                    way_cost,
                                    &ctx.routing_configs(),
                                    &edge_ctx,
                                );
                            if cost == u32::MAX {
                                continue;
                            }
                            if way_cost.istrafficbackbone {
                                cost = 0;
                            }

                            let mut dir_penalty = 0u32;
                            if let Some(start_dir) = ctx.start_direction {
                                let edge_dir =
                                    spatial::get_direction(sliced[0].to_2d(), sliced[1].to_2d());
                                let diff =
                                    spatial::get_difference_from_direction(start_dir, edge_dir);
                                if ctx.force_start_direction && diff > 90.0 {
                                    dir_penalty = 10_000;
                                } else if diff > 45.0 {
                                    dir_penalty = ((diff - 45.0) * (way_cost.turncost as f64).max(50.0) / 90.0) as u32;
                                }
                            }

                            let mut total_cost = cost + nogo_cost + dir_penalty;
                            if let Some(target_node) = t_node {
                                if target_node.node_desc_id != NO_DESC {
                                    let access_granted = if ctx.car_kinematic.is_some() {
                                        false
                                    } else {
                                        way_cost.nodeaccessgranted
                                    };
                                    let node_res = if ctx.node_foreign_var_count == 0
                                        || (ctx.node_foreign_var_count == 1
                                            && (way_cost.foreign_var_0 == 0.0
                                                || way_cost.foreign_var_0 == 1.0))
                                    {
                                        if let Some(&res) = get_node_cost(
                                            &ctx.costs.node_costs,
                                            target_node.node_desc_id,
                                            access_granted,
                                            way_cost.foreign_var_0 != 0.0,
                                        ) {
                                            res
                                        } else {
                                            continue;
                                        }
                                    } else if let Some(node_desc) = ctx
                                        .graph
                                        .node_descriptions
                                        .get(target_node.node_desc_id as usize)
                                    {
                                        if ctx.node_foreign_var_count > 0 {
                                            if let Some(desc_bytes) = edge.description(ctx.graph) {
                                                ctx.lookups.way.decode_into(
                                                    &mut self.way_lookup_buf,
                                                    edge.is_reverse,
                                                    desc_bytes,
                                                );
                                                self.evaluator.evaluate_way(&self.way_lookup_buf);
                                            }
                                        }
                                        ctx.lookups.node.decode_into(
                                            &mut self.node_lookup_buf,
                                            access_granted,
                                            node_desc,
                                        );
                                        self.evaluator.evaluate_node(&self.node_lookup_buf)
                                    } else {
                                        continue;
                                    };
                                    if node_res.initialcost >= 1_000_000.0 {
                                        continue;
                                    }
                                    if let Some(car_model) = ctx.car_kinematic {
                                        car_model.cut_ekin(
                                            &mut path_state.ekin,
                                            (node_res.maxspeed as f64) / 3.6,
                                        );
                                    }
                                    total_cost += node_res.initialcost as u32;
                                }
                            }

                            let air_dist = distance(
                                t_node_pt,
                                ctx.target_mwp.crosspoint,
                            );
                            let priority = total_cost + (air_dist * ctx.air_distance_factor) as u32;

                            let (down_div, up_div) = if ctx.car_kinematic.is_some() {
                                (-1, 0)
                            } else {
                                StdPathCalculator::cost_divisors(way_cost)
                            };
                            let h_idx = self.history.len() as u32;
                            self.sliced_map.insert(h_idx, sliced);
                            self.history.push(HistoryStep {
                                node_id: t_id,
                                from_node_id: s_id,
                                cost: total_cost,
                                global_edge_idx: edge.edge_idx,
                                edge_idx: edge_idx as u16,
                                maxspeed: way_cost.maxspeed,
                                prev_idx: u32::MAX,
                            });

                            let in_destination = (way_cost.classifiermask & 64) != 0;
                            self.get_or_create_edge_state(edge.edge_idx).push(
                                CandidateState::with_context(
                                    total_cost,
                                    path_state.ehbd,
                                    path_state.ehbu,
                                    h_idx,
                                    path_selev,
                                    in_destination,
                                    in_destination,
                                ),
                            );

                            let node_idx = self.node_pool.alloc(SearchNode {
                                node_id: t_id,
                                from_node_id: s_id,
                                ilon: t_node_pt.lon,
                                ilat: t_node_pt.lat,
                                cost: total_cost,
                                air_dist: air_dist as u32,
                                priority,
                                path_state,
                                prev_lon: penultimate_pt.lon,
                                prev_lat: penultimate_pt.lat,
                                priority_classifier: way_cost.priorityclassifier,
                                down_div,
                                up_div,
                                history_idx: h_idx,
                                selev: t_elev,
                                path_selev,
                                last_classifier: way_cost.initialclassifier,
                                in_destination,
                                can_leave_destination: in_destination,
                                has_tr: edge.has_target_tr,
                                edge_idx: edge.edge_idx,
                                rev_edge_idx: edge.rev_edge_idx,
                                out_edges_start: edge.target_edges_start,
                                out_edges_count: edge.target_edges_count,
                                node_desc_id: edge.target_node_desc_id,
                            });
                            self.open_set.push(HeapItem { priority, node_idx });
                        }
                    }
                }
            }
        }

        // Fallback if neither partial edge was formed
        if self.open_set.is_empty() && self.final_cost == u32::MAX {
            for &s_id in &[ctx.start_id1, ctx.start_id2] {
                if let Some(node) = ctx.graph.nodes.get(&s_id) {
                    let air_dist = distance(
                        Point2D::new(node.ilon, node.ilat),
                        ctx.target_mwp.crosspoint,
                    );
                    let priority = (air_dist * ctx.air_distance_factor) as u32;

                    let h_idx = self.history.len() as u32;
                    self.sliced_map
                        .insert(h_idx, vec![Point3D::new(node.ilon, node.ilat, node.selev)]);
                    self.history.push(HistoryStep {
                        node_id: s_id,
                        from_node_id: 0,
                        cost: 0,
                        global_edge_idx: u32::MAX,
                        edge_idx: 0,
                        maxspeed: 0.0,
                        prev_idx: u32::MAX,
                    });

                    let (out_start, out_count) = ctx.graph.get_node_edge_slice(s_id);
                    let node_idx = self.node_pool.alloc(SearchNode {
                        node_id: s_id,
                        from_node_id: 0,
                        ilon: node.ilon,
                        ilat: node.ilat,
                        cost: 0,
                        air_dist: air_dist as u32,
                        priority,
                        path_state: PathState::default(),
                        prev_lon: node.ilon,
                        prev_lat: node.ilat,
                        priority_classifier: 0,
                        down_div: 0,
                        up_div: 0,
                        history_idx: h_idx,
                        selev: node.selev,
                        path_selev: node.selev,
                        last_classifier: 0,
                        in_destination: false,
                        can_leave_destination: false,
                        has_tr: !node.turn_restrictions.is_empty(),
                        edge_idx: u32::MAX,
                        rev_edge_idx: u32::MAX,
                        out_edges_start: out_start,
                        out_edges_count: out_count,
                        node_desc_id: node.node_desc_id,
                    });
                    self.open_set.push(HeapItem { priority, node_idx });
                }
            }
        }

        if self.open_set.is_empty() && self.best_target_history.is_none() {
            return Err("Start nodes not found in graph".to_string());
        }

        Ok(())
    }

    /// Ensures the expanded node's cell and 1-ring neighbor cells are loaded and its outgoing edges are finalized.
    #[inline(always)]
    fn ensure_node_ready(
        &mut self,
        ctx: &mut SearchPassContext<'a, '_, '_>,
        curr: &mut SearchNode,
    ) {
        if curr.out_edges_start != u32::MAX {
            return;
        }
        let Some(graph_mut) = ctx.graph.as_mut() else {
            return;
        };
        if !graph_mut.has_cell_provider() {
            return;
        }
        let prev_edges = graph_mut.total_edges;
        let prev_descs = graph_mut.descriptions.len();
        let prev_node_descs = graph_mut.node_descriptions.len();
        let (out_start, out_count) = graph_mut.ensure_node_finalized(curr.node_id);
        curr.out_edges_start = out_start;
        curr.out_edges_count = out_count;
        if curr.edge_idx != u32::MAX {
            if let Some(edge) = graph_mut.flat_edges.get_mut(curr.edge_idx as usize) {
                edge.target_edges_start = out_start;
                edge.target_edges_count = out_count;
                curr.rev_edge_idx = edge.rev_edge_idx;
            }
        }
        if graph_mut.total_edges > prev_edges {
            let new_total = graph_mut.total_edges;
            if self.edge_heads.len() < new_total {
                self.edge_heads.resize(new_total, u32::MAX);
            }
            let num_words = new_total.div_ceil(64);
            if self.closed_edges.len() < num_words {
                self.closed_edges.resize(num_words, 0u64);
            }
        }
        if graph_mut.descriptions.len() > prev_descs
            || graph_mut.node_descriptions.len() > prev_node_descs
        {
            ctx.costs = std::sync::Arc::default();
            ctx.costs = graph_mut.sync_profile_costs(ctx.profile, ctx.lookups);
        }
    }

    /// Executes the main priority-queue A* search loop.
    fn run_search(&mut self, ctx: &mut SearchPassContext<'a, '_, '_>) -> Result<(), String> {
        let mut cands_buf: Vec<CandidateState> = Vec::with_capacity(8);
        let mut pre_paths_buf: Vec<KinematicPrePath> = Vec::with_capacity(8);
        while let Some(item) = self.open_set.pop() {
            let mut curr = self.node_pool.free(item.node_idx);
            self.nodes_popped += 1;
            if curr.node_id == 0 {
                self.final_cost = curr.cost;
                self.best_target_history = Some(curr.history_idx);
                break;
            }
            if curr.priority >= self.final_cost {
                break;
            }
            if let Some(max_cost) = self.max_cost_limit {
                if ctx.air_distance_factor == 0.0 && curr.cost >= max_cost {
                    break;
                }
                if curr.cost + curr.air_dist > max_cost {
                    continue;
                }
            }
            if ctx.graph.has_cell_provider() && self.nodes_popped > 12_000_000 {
                return Err("Search exceeded maximum node limit".to_string());
            }

            cands_buf.clear();
            if curr.edge_idx != u32::MAX {
                if self.is_edge_closed(curr.edge_idx) {
                    continue;
                }
                if let Some(edge_entry) = self.get_edge_state(curr.edge_idx) {
                    let mut dominated = false;
                    let curr_cand = CandidateState::new(curr.cost, curr.path_state.ehbd, curr.path_state.ehbu);
                    for cand in edge_entry.candidates() {
                        if cand.cost < curr.cost
                            && curr_cand.is_definitely_worse_than(cand, curr.down_div, curr.up_div)
                        {
                            dominated = true;
                            break;
                        }
                    }
                    if dominated {
                        continue;
                    }
                    if ctx.car_kinematic.is_none() && edge_entry.candidates().len() > 1 {
                        cands_buf.extend_from_slice(edge_entry.candidates());
                    }
                }
                self.mark_edge_closed(curr.edge_idx);
            }

            self.ensure_node_ready(ctx, &mut curr);

            if curr.edge_idx != u32::MAX {
                // Bidirectional counterlink closure: if counterlink has no path yet and no turn restrictions, close it
                if !ctx.consider_tr
                    && curr.rev_edge_idx != u32::MAX
                    && !self.is_edge_closed(curr.rev_edge_idx)
                    && !self.has_edge_state(curr.rev_edge_idx)
                {
                    self.mark_edge_closed(curr.rev_edge_idx);
                }
            }

            // Pop cost-cutting with elevation correction
            if let Some(guide) = ctx.cost_cutting {
                if let Some(p0_link_cost) = guide.get_cost(curr.edge_idx) {
                    let elev_corr = StdPathCalculator::elevation_correction(
                        curr.path_state.ehbd,
                        curr.path_state.ehbu,
                        curr.down_div,
                        curr.up_div,
                    );
                    let remaining_p0 = guide.pass0_total_cost.saturating_sub(p0_link_cost);
                    let cost_estimate = curr.cost + elev_corr + remaining_p0;
                    if let Some(ref mut max_cost) = self.max_cost_limit {
                        if cost_estimate < *max_cost {
                            *max_cost = cost_estimate;
                        }
                    }
                }
            }

            // Standalone target node check
            if ctx.target_id1 == ctx.target_id2
                && curr.node_id == ctx.target_id1
                && curr.cost < self.final_cost
            {
                self.final_cost = curr.cost;
                self.best_target_history = Some(curr.history_idx);
                break;
            }

            let edges = ctx.graph.get_edges_by_slice(curr.out_edges_start, curr.out_edges_count);
            if edges.is_empty() {
                continue;
            }

            // Check if curr node has turn restrictions (only if profile considers TRs and not start node)
            let curr_trs: &[DecodedTurnRestriction] = if ctx.consider_tr
                && curr.has_tr
                && curr.from_node_id != 0
            {
                ctx.graph
                    .turn_restrictions
                    .get(&curr.node_id)
                    .map(|v| v.as_slice())
                    .unwrap_or(&[])
            } else {
                &[]
            };

            // Check if curr node has estimated_crossing_class (only if crossing evaluation is active)
            let crossing_class = if ctx.crossing_cfg.consider_crossing
                && curr.node_desc_id != NO_DESC
                && curr.priority_classifier >= ctx.crossing_cfg.crossing_prio_h
            {
                if let Some(nc) = get_node_cost(&ctx.costs.node_costs, curr.node_desc_id, false, false) {
                    if nc.crossing_class > 0 {
                        Some(nc.crossing_class)
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                None
            };

            pre_paths_buf.clear();
            if ctx.car_kinematic.is_some() && curr.from_node_id != 0 {
                let p0 = Point2D::new(curr.prev_lon, curr.prev_lat);
                let p1 = Point2D::new(curr.ilon, curr.ilat);
                for e in edges.iter() {
                    if e.target_id == curr.from_node_id {
                        continue;
                    }
                    if let Some(wc) = get_way_cost(&ctx.costs.way_costs, e.desc_id, e.is_reverse) {
                        let p2 = if e.geometry.is_empty() {
                            Point2D::new(e.target_lon, e.target_lat)
                        } else {
                            e.fill_polyline(
                                &mut self.poly_buf,
                                curr.ilon,
                                curr.ilat,
                                curr.selev,
                                e.target_selev,
                            );
                            if self.poly_buf.len() >= 2 {
                                self.poly_buf[1].to_2d()
                            } else {
                                Point2D::new(e.target_lon, e.target_lat)
                            }
                        };
                        let (ang, _) = calc_angle(p0, p1, p2);
                        pre_paths_buf.push(KinematicPrePath {
                            angle: ang,
                            priorityclassifier: wc.priorityclassifier,
                            classifiermask: wc.classifiermask,
                        });
                    }
                }
            }

            for (edge_idx, edge) in edges.iter().enumerate() {
                self.edges_evaluated += 1;
                self.relax_edge(
                    ctx,
                    &curr,
                    &cands_buf,
                    &pre_paths_buf,
                    edge_idx,
                    edge,
                    curr_trs,
                    crossing_class,
                );
            }
        }
        Ok(())
    }

    /// Evaluates and relaxes an outgoing directed edge from the currently expanded node.
    #[inline(always)]
    #[allow(clippy::too_many_arguments)]
    fn relax_edge(
        &mut self,
        ctx: &SearchPassContext,
        curr: &SearchNode,
        extra_cands: &[CandidateState],
        pre_paths: &[KinematicPrePath],
        edge_idx: usize,
        edge: &GraphEdge,
        curr_trs: &[DecodedTurnRestriction],
        crossing_class: Option<u8>,
    ) {
        // Prevent immediate backtrack / U-turn
        if curr.from_node_id != 0 && edge.target_id == curr.from_node_id {
            return;
        }

        // Fast-skip dead-end neighbors that cannot reach the destination
        if edge.is_target_dead_end
            && edge.target_id != ctx.target_id1
            && edge.target_id != ctx.target_id2
            && !ctx.protected_nodes.contains(&edge.target_id)
        {
            return;
        }

        // Fast-skip edges that have already been closed
        if self.is_edge_closed(edge.edge_idx) {
            return;
        }

        let Some(way_cost) = get_way_cost(&ctx.costs.way_costs, edge.desc_id, edge.is_reverse)
        else {
            return;
        };

        if way_cost.costfactor > 9998.0 {
            return; // forbidden way
        }

        // Initial cost on classifier change
        let new_classifier = way_cost.initialclassifier;
        let new_initial_cost = way_cost.initialcost;
        let mut classifier_cost = 0u32;
        if new_classifier != 0
            && curr.last_classifier != 0
            && new_classifier != curr.last_classifier
        {
            if new_initial_cost >= 1_000_000.0 {
                return;
            }
            classifier_cost = new_initial_cost as u32;
        }

        let is_destination = (way_cost.classifiermask & 64) != 0;
        let mut new_destination = curr.in_destination;
        let mut can_leave = curr.can_leave_destination;
        if curr.from_node_id == 0 {
            new_destination = is_destination;
            can_leave = is_destination;
        } else if is_destination {
            new_destination = true;
        } else if new_destination {
            if !can_leave {
                return;
            }
            new_destination = false;
            can_leave = false;
        }

        let mut air_dist_opt = None;
        if let Some(max_cost) = self.max_cost_limit {
            let d = spatial::distance(
                Point2D::new(edge.target_lon, edge.target_lat),
                ctx.target_mwp.crosspoint,
            ) as u32;
            if curr.cost + d > max_cost {
                return;
            }
            air_dist_opt = Some(d);
        }

        let t_elev = if ctx.graph.is_resolved || edge.target_selev != i16::MIN {
            edge.target_selev
        } else {
            ctx.graph
                .nodes
                .get(&edge.target_id)
                .map(|n| n.selev)
                .unwrap_or(i16::MIN)
        };

        let is_straight = edge.geometry.is_empty();
        let prev_pt = Point2D::new(curr.prev_lon, curr.prev_lat);
        if is_straight {
            if !curr_trs.is_empty()
                && DecodedTurnRestriction::is_turn_forbidden(
                    curr_trs,
                    prev_pt,
                    Point2D::new(edge.target_lon, edge.target_lat),
                    ctx.is_bike_mode,
                    ctx.is_car_mode,
                )
            {
                return;
            }
        } else {
            edge.fill_polyline(&mut self.poly_buf, curr.ilon, curr.ilat, curr.selev, t_elev);
            if self.poly_buf.len() < 2 {
                return;
            }
            if !curr_trs.is_empty()
                && DecodedTurnRestriction::is_turn_forbidden(
                    curr_trs,
                    prev_pt,
                    self.poly_buf[1].to_2d(),
                    ctx.is_bike_mode,
                    ctx.is_car_mode,
                )
            {
                return;
            }
        }

        let is_target_link = (curr.node_id == ctx.target_id1 && edge.target_id == ctx.target_id2)
            || (curr.node_id == ctx.target_id2 && edge.target_id == ctx.target_id1);

        if is_target_link {
            if is_straight {
                edge.fill_polyline(&mut self.poly_buf, curr.ilon, curr.ilat, curr.selev, t_elev);
            }
            let sliced = slice_to_crosspoint(
                &self.poly_buf,
                ctx.target_mwp.crosspoint,
            );
            if sliced.len() >= 2 {
                let nogo_res = check_polyline_against_nogos(&sliced, ctx.nogos);
                let nogo_cost = match nogo_res {
                    NogoCheckResult::Blocked => return,
                    NogoCheckResult::Pass { added_cost } => added_cost,
                };
                let mut path_state = curr.path_state;
                let mut path_selev = curr.path_selev;
                let edge_ctx = EdgeContext {
                    crossing_class,
                    old_prio: curr.priority_classifier,
                    prev_point: prev_pt,
                    is_start: false,
                    is_detail: false,
                    pre_paths,
                };
                let (mut edge_cost, _, _, _, penultimate_pt) = process_polyline(
                    &sliced,
                    &mut path_state,
                    &mut path_selev,
                    way_cost,
                    &ctx.routing_configs(),
                    &edge_ctx,
                );
                if edge_cost == u32::MAX {
                    return;
                }
                if curr.cost == 0 && way_cost.istrafficbackbone {
                    edge_cost = 0;
                }
                edge_cost += classifier_cost + nogo_cost;

                let target_cost = curr.cost + edge_cost;
                if target_cost < self.final_cost {
                    self.final_cost = target_cost;
                    let h_idx = self.history.len() as u32;
                    self.sliced_map.insert(h_idx, sliced);
                    self.history.push(HistoryStep {
                        node_id: 0,
                        from_node_id: curr.node_id,
                        cost: target_cost,
                        global_edge_idx: edge.edge_idx,
                        edge_idx: edge_idx as u16,
                        maxspeed: way_cost.maxspeed,
                        prev_idx: curr.history_idx,
                    });
                    let node_idx = self.node_pool.alloc(SearchNode {
                        node_id: 0,
                        from_node_id: curr.node_id,
                        ilon: ctx.target_mwp.crosspoint.lon,
                        ilat: ctx.target_mwp.crosspoint.lat,
                        cost: target_cost,
                        air_dist: 0,
                        priority: target_cost,
                        path_state,
                        prev_lon: penultimate_pt.lon,
                        prev_lat: penultimate_pt.lat,
                        priority_classifier: curr.priority_classifier,
                        down_div: 0,
                        up_div: 0,
                        history_idx: h_idx,
                        selev: 0,
                        path_selev,
                        last_classifier: 0,
                        in_destination: false,
                        can_leave_destination: false,
                        has_tr: false,
                        edge_idx: edge.edge_idx,
                        rev_edge_idx: edge.rev_edge_idx,
                        out_edges_start: 0,
                        out_edges_count: 0,
                        node_desc_id: NO_DESC,
                    });
                    self.open_set.push(HeapItem {
                        priority: target_cost,
                        node_idx,
                    });
                    self.best_target_history = Some(h_idx);
                }
            }
            return;
        }

        let mut path_state = curr.path_state;
        let mut path_selev = curr.path_selev;
        let mut chosen_history_idx = curr.history_idx;
        let edge_ctx = EdgeContext {
            crossing_class,
            old_prio: curr.priority_classifier,
            prev_point: prev_pt,
            is_start: false,
            is_detail: false,
            pre_paths,
        };
        let routing_cfgs = ctx.routing_configs();
        let seg_dist_straight = if is_straight {
            if edge.straight_dist > 0.0 {
                edge.straight_dist
            } else {
                spatial::distance(
                    Point2D::new(curr.ilon, curr.ilat),
                    Point2D::new(edge.target_lon, edge.target_lat),
                )
                .round()
                .max(1.0)
            }
        } else {
            0.0
        };
        let (nogo_cost, (mut edge_cost, edge_dist, _, _, penultimate_pt)) = if is_straight {
            let nogo_res = check_segment_against_nogos(
                Point2D::new(curr.ilon, curr.ilat),
                Point2D::new(edge.target_lon, edge.target_lat),
                ctx.nogos,
            );
            let nogo_cost = match nogo_res {
                NogoCheckResult::Blocked => return,
                NogoCheckResult::Pass { added_cost } => added_cost,
            };
            (
                nogo_cost,
                process_straight_edge(
                    Point3D::new(curr.ilon, curr.ilat, curr.selev),
                    Point3D::new(edge.target_lon, edge.target_lat, t_elev),
                    seg_dist_straight,
                    &mut path_state,
                    &mut path_selev,
                    way_cost,
                    &routing_cfgs,
                    &edge_ctx,
                ),
            )
        } else {
            let nogo_res = check_polyline_against_nogos(&self.poly_buf, ctx.nogos);
            let nogo_cost = match nogo_res {
                NogoCheckResult::Blocked => return,
                NogoCheckResult::Pass { added_cost } => added_cost,
            };
            (
                nogo_cost,
                process_polyline(
                    &self.poly_buf,
                    &mut path_state,
                    &mut path_selev,
                    way_cost,
                    &routing_cfgs,
                    &edge_ctx,
                ),
            )
        };
        if edge_cost == u32::MAX {
            return;
        }
        if curr.cost == 0 && way_cost.istrafficbackbone {
            edge_cost = 0;
        }

        // If multiple undominated elevation-hysteresis candidates exist on curr.edge_idx,
        // evaluate them and select the candidate producing the lowest total cost
        // (matching RoutingEngine.java:2257-2274).
        let mut base_parent_cost = curr.cost;
        if extra_cands.len() > 1 {
            let mut best_combined = base_parent_cost + edge_cost;
            for cand in extra_cands {
                if (cand.history_idx == curr.history_idx && cand.cost == curr.cost)
                    || cand.cost >= best_combined
                {
                    continue;
                }
                let mut cand_dest = cand.in_destination;
                let mut cand_can_leave = cand.can_leave_destination;
                if is_destination {
                    cand_dest = true;
                } else if cand_dest {
                    if !cand_can_leave {
                        continue;
                    }
                    cand_dest = false;
                    cand_can_leave = false;
                }
                let mut cand_state = PathState {
                    ehbd: cand.ehbd,
                    ehbu: cand.ehbu,
                    ..Default::default()
                };
                let mut cand_selev = cand.path_selev;
                let (mut cand_edge_cost, _, _, _, _) = if is_straight {
                    process_straight_edge(
                        Point3D::new(curr.ilon, curr.ilat, curr.selev),
                        Point3D::new(edge.target_lon, edge.target_lat, t_elev),
                        seg_dist_straight,
                        &mut cand_state,
                        &mut cand_selev,
                        way_cost,
                        &routing_cfgs,
                        &edge_ctx,
                    )
                } else {
                    process_polyline(
                        &self.poly_buf,
                        &mut cand_state,
                        &mut cand_selev,
                        way_cost,
                        &routing_cfgs,
                        &edge_ctx,
                    )
                };
                if cand_edge_cost == u32::MAX {
                    continue;
                }
                if cand.cost == 0 && way_cost.istrafficbackbone {
                    cand_edge_cost = 0;
                }
                let combined = cand.cost + cand_edge_cost;
                if combined < best_combined {
                    best_combined = combined;
                    base_parent_cost = cand.cost;
                    edge_cost = cand_edge_cost;
                    path_state = cand_state;
                    path_selev = cand_selev;
                    chosen_history_idx = cand.history_idx;
                    new_destination = cand_dest;
                    can_leave = cand_can_leave;
                }
            }
        }

        edge_cost += classifier_cost + nogo_cost;

        if let Some(ref_nodes) = ctx.ref_nodes {
            let source_id = Point2D::new(curr.ilon, curr.ilat).node_id();
            let target_id = edge.target_id;
            if ref_nodes.contains(&source_id) && ref_nodes.contains(&target_id) {
                let reftrackcost = edge_dist.round().max(1.0) as u32;
                edge_cost += reftrackcost;
            }
        }

        // Check node costs for target node
        let target_desc_id = if edge.target_node_desc_id != NO_DESC {
            edge.target_node_desc_id
        } else if !ctx.graph.is_resolved {
            ctx.graph
                .nodes
                .get(&edge.target_id)
                .map(|n| n.node_desc_id)
                .unwrap_or(NO_DESC)
        } else {
            NO_DESC
        };
        if target_desc_id != NO_DESC {
            let access_granted = if ctx.car_kinematic.is_some() {
                false
            } else {
                way_cost.nodeaccessgranted
            };
            let node_res_opt = if ctx.node_foreign_var_count == 0
                || (ctx.node_foreign_var_count == 1
                    && (way_cost.foreign_var_0 == 0.0 || way_cost.foreign_var_0 == 1.0))
            {
                get_node_cost(
                    &ctx.costs.node_costs,
                    target_desc_id,
                    access_granted,
                    way_cost.foreign_var_0 != 0.0,
                )
                .copied()
            } else if ctx.node_foreign_var_count == 1 {
                if let Some(node_desc) = ctx.graph.node_descriptions.get(target_desc_id as usize) {
                    if let Some(fv_idx) = ctx.node_foreign_var_0 {
                        self.evaluator.way_vars[fv_idx] = way_cost.foreign_var_0;
                    }
                    ctx.lookups.node.decode_into(
                        &mut self.node_lookup_buf,
                        access_granted,
                        node_desc,
                    );
                    Some(self.evaluator.evaluate_node(&self.node_lookup_buf))
                } else {
                    None
                }
            } else if let Some(node_desc) = ctx.graph.node_descriptions.get(target_desc_id as usize) {
                if let Some(desc_bytes) = edge.description(ctx.graph) {
                    ctx.lookups.way.decode_into(
                        &mut self.way_lookup_buf,
                        edge.is_reverse,
                        desc_bytes,
                    );
                    self.evaluator.evaluate_way(&self.way_lookup_buf);
                }
                ctx.lookups.node.decode_into(
                    &mut self.node_lookup_buf,
                    access_granted,
                    node_desc,
                );
                Some(self.evaluator.evaluate_node(&self.node_lookup_buf))
            } else {
                None
            };
            if let Some(node_res) = node_res_opt {
                if node_res.initialcost >= 1_000_000.0 {
                    return; // blocked node
                }
                if let Some(car_model) = ctx.car_kinematic {
                    car_model.cut_ekin(
                        &mut path_state.ekin,
                        (node_res.maxspeed as f64) / 3.6,
                    );
                }
                edge_cost += node_res.initialcost as u32;
            }
        }

        let new_cost = base_parent_cost + edge_cost;
        let (down_div, up_div) = if ctx.car_kinematic.is_some() {
            (-1, 0)
        } else {
            StdPathCalculator::cost_divisors(way_cost)
        };

        if let Some(guide) = ctx.cost_cutting {
            if let Some(p0_link_cost) = guide.get_cost(edge.edge_idx) {
                let elev_corr = StdPathCalculator::elevation_correction(
                    path_state.ehbd,
                    path_state.ehbu,
                    down_div,
                    up_div,
                );
                let remaining_p0 = guide.pass0_total_cost.saturating_sub(p0_link_cost);
                let cost_estimate = new_cost + elev_corr + remaining_p0;
                if let Some(ref mut max_cost) = self.max_cost_limit {
                    if cost_estimate < *max_cost {
                        *max_cost = cost_estimate;
                    }
                }
            }
        }

        if new_cost >= self.final_cost {
            return;
        }
        if let Some(max_cost) = self.max_cost_limit {
            let d = air_dist_opt.unwrap_or_else(|| {
                spatial::distance(
                    Point2D::new(edge.target_lon, edge.target_lat),
                    ctx.target_mwp.crosspoint,
                ) as u32
            });
            if new_cost + d > max_cost {
                return;
            }
            air_dist_opt = Some(d);
        }

        if self.is_edge_closed(edge.edge_idx) {
            return;
        }

        let h_idx = self.history.len() as u32;

        // Fast dominance check against existing candidate paths on this edge (if any exist)
        let new_cand = CandidateState::with_context(
            new_cost,
            path_state.ehbd,
            path_state.ehbu,
            h_idx,
            path_selev,
            new_destination,
            can_leave,
        );
        let entry_idx = self
            .edge_heads
            .get(edge.edge_idx as usize)
            .copied()
            .unwrap_or(u32::MAX);
        if entry_idx != u32::MAX {
            let edge_entry = &self.edge_states_pool[entry_idx as usize];
            let mut dominated = false;
            for cand in edge_entry.candidates() {
                if new_cand.is_definitely_worse_than(cand, down_div, up_div)
                    || (down_div >= 0
                        && new_cand.cost >= cand.cost
                        && (down_div == 0 || new_cand.ehbd >= cand.ehbd)
                        && (up_div == 0 || new_cand.ehbu >= cand.ehbu)
                        && new_cand.in_destination == cand.in_destination
                        && new_cand.can_leave_destination == cand.can_leave_destination
                        && new_cand.path_selev == cand.path_selev)
                {
                    dominated = true;
                    break;
                }
            }
            if dominated {
                return;
            }
        }

        let edge_entry = self.get_or_create_edge_state(edge.edge_idx);
        edge_entry.retain(|cand| {
            !cand.is_definitely_worse_than(&new_cand, down_div, up_div)
        });

        edge_entry.push(new_cand);

        let air_dist = air_dist_opt.unwrap_or_else(|| {
            spatial::distance(
                Point2D::new(edge.target_lon, edge.target_lat),
                ctx.target_mwp.crosspoint,
            ) as u32
        });
        let priority = new_cost + (air_dist as f64 * ctx.air_distance_factor) as u32;

        self.history.push(HistoryStep {
            node_id: edge.target_id,
            from_node_id: curr.node_id,
            cost: new_cost,
            global_edge_idx: edge.edge_idx,
            edge_idx: edge_idx as u16,
            maxspeed: way_cost.maxspeed,
            prev_idx: chosen_history_idx,
        });

        let node_idx = self.node_pool.alloc(SearchNode {
            node_id: edge.target_id,
            from_node_id: curr.node_id,
            ilon: edge.target_lon,
            ilat: edge.target_lat,
            cost: new_cost,
            air_dist,
            priority,
            path_state,
            prev_lon: penultimate_pt.lon,
            prev_lat: penultimate_pt.lat,
            priority_classifier: way_cost.priorityclassifier,
            down_div,
            up_div,
            history_idx: h_idx,
            selev: t_elev,
            path_selev,
            last_classifier: new_classifier,
            in_destination: new_destination,
            can_leave_destination: can_leave,
            has_tr: edge.has_target_tr,
            edge_idx: edge.edge_idx,
            rev_edge_idx: edge.rev_edge_idx,
            out_edges_start: edge.target_edges_start,
            out_edges_count: edge.target_edges_count,
            node_desc_id: edge.target_node_desc_id,
        });
        self.open_set.push(HeapItem { priority, node_idx });
    }

    /// Reconstructs the winning route track by backtracking through search history.
    fn reconstruct_track(&self, _ctx: &SearchPassContext) -> Result<SearchTrack, String> {
        let Some(target_h_idx) = self.best_target_history else {
            return Err("No path found between waypoints".to_string());
        };

        let mut steps = Vec::new();
        let mut curr_idx = target_h_idx;
        while curr_idx != u32::MAX {
            let h = self.history[curr_idx as usize];
            let sliced = self.sliced_map.get(&curr_idx).cloned();
            steps.push(TrackStep {
                node_id: h.node_id,
                from_node_id: h.from_node_id,
                cost: h.cost,
                global_edge_idx: h.global_edge_idx,
                edge_idx: h.edge_idx,
                maxspeed: h.maxspeed,
                sliced_points: sliced,
            });
            curr_idx = h.prev_idx;
        }
        steps.reverse();

        Ok(SearchTrack {
            steps,
            total_cost: self.final_cost,
        })
    }
}

/// Evaluates cost, distance, time, and kinematics along a straight 2-point edge without polyline allocation.
#[inline]
#[allow(clippy::too_many_arguments)]
fn process_straight_edge(
    s_pt: Point3D,
    t_pt: Point3D,
    seg_dist: Distance,
    path_state: &mut PathState,
    path_selev: &mut i16,
    way_cost: &crate::dsl::evaluator::WayCostResult,
    configs: &RoutingConfigs,
    edge_ctx: &EdgeContext,
) -> (Cost, Distance, Seconds, Joules, Point2D) {
    let mut ele2 = t_pt.elev;
    if ele2 == i16::MIN {
        ele2 = *path_selev;
    }
    let delta_h_micros = if *path_selev == i16::MIN {
        0
    } else {
        (ele2 - *path_selev) as i32 * 250_000
    };
    *path_selev = ele2;

    let need_crossing =
        !edge_ctx.is_start && configs.crossing.consider_crossing && edge_ctx.old_prio > 0;
    let (angle, cos_angle) = if !edge_ctx.is_start {
        if configs.car_kinematic.is_some() || need_crossing {
            calc_angle(edge_ctx.prev_point, s_pt.to_2d(), t_pt.to_2d())
        } else if way_cost.turncost > 0.0 {
            (0.0, spatial::calc_cos_angle(edge_ctx.prev_point, s_pt.to_2d(), t_pt.to_2d()))
        } else {
            (0.0, 1.0)
        }
    } else {
        (0.0, 1.0)
    };

    let crossing_penalty = if need_crossing {
        if edge_ctx.old_prio >= configs.crossing.crossing_prio_h
            && way_cost.priorityclassifier <= configs.crossing.crossing_prio_l
        {
            if let Some(cls) = edge_ctx.crossing_class {
                if (1..=6).contains(&cls) {
                    let idx = (cls - 1) as usize;
                    if angle < 0.0 {
                        configs.crossing.cost_to_left[idx]
                    } else if angle > 0.0 {
                        configs.crossing.cost_to_right[idx]
                    } else {
                        0
                    }
                } else {
                    0
                }
            } else {
                0
            }
        } else {
            0
        }
    } else {
        0
    };

    let sec = if let Some(car_model) = configs.car_kinematic {
        let delta_h = delta_h_micros as f64 * 1e-6;
        let elev_m = if ele2 == i16::MIN {
            100.0
        } else {
            selev_to_elevation(ele2)
        };
        let Some(sec) = StdPathCalculator::process_car_section(
            path_state,
            way_cost,
            car_model,
            seg_dist,
            delta_h,
            elev_m,
            angle,
            cos_angle,
            edge_ctx.is_start,
            0,
            edge_ctx.old_prio,
            edge_ctx.pre_paths,
        ) else {
            return (u32::MAX, seg_dist, 0.0, 0.0, s_pt.to_2d());
        };
        sec
    } else {
        let input = SectionInput {
            dist: seg_dist,
            delta_h_micros,
            cos_angle,
            crossing_penalty,
            is_detail: edge_ctx.is_detail,
        };
        StdPathCalculator::process_section(
            path_state,
            way_cost,
            configs.elev,
            configs.kinematic,
            &input,
        )
    };

    (
        sec.section_cost_int,
        seg_dist,
        sec.time_seconds,
        sec.energy_joules,
        s_pt.to_2d(),
    )
}

/// Evaluates cost, distance, time, and kinematics along a polyline.
fn process_polyline(
    poly: &[Point3D],
    path_state: &mut PathState,
    path_selev: &mut i16,
    way_cost: &crate::dsl::evaluator::WayCostResult,
    configs: &RoutingConfigs,
    edge_ctx: &EdgeContext,
) -> (Cost, Distance, Seconds, Joules, Point2D) {
    let mut edge_cost = 0;
    let mut edge_dist = 0.0;
    let mut edge_time = 0.0;
    let mut edge_energy = 0.0;

    if edge_ctx.is_start && !poly.is_empty() {
        *path_selev = poly[0].elev;
    }

    let mut p_prev = edge_ctx.prev_point;

    for i in 0..poly.len() - 1 {
        let p1 = &poly[i];
        let p2 = &poly[i + 1];

        let seg_dist = distance(p1.to_2d(), p2.to_2d()).round().max(1.0);
        if seg_dist <= 0.0 {
            continue;
        }

        let mut ele2 = p2.elev;
        if ele2 == i16::MIN {
            ele2 = *path_selev;
        }
        let delta_h_micros = if *path_selev == i16::MIN {
            0
        } else {
            (ele2 - *path_selev) as i32 * 250_000
        };
        *path_selev = ele2;

        let is_junction = i == 0;
        let is_startpoint = is_junction && edge_ctx.is_start;
        let need_crossing = is_junction
            && !edge_ctx.is_start
            && configs.crossing.consider_crossing
            && edge_ctx.old_prio > 0;
        let (angle, cos_angle) = if !is_startpoint {
            if configs.car_kinematic.is_some() || need_crossing {
                calc_angle(p_prev, p1.to_2d(), p2.to_2d())
            } else if way_cost.turncost > 0.0 {
                (0.0, spatial::calc_cos_angle(p_prev, p1.to_2d(), p2.to_2d()))
            } else {
                (0.0, 1.0)
            }
        } else {
            (0.0, 1.0)
        };

        let crossing_penalty = if need_crossing {
            if edge_ctx.old_prio >= configs.crossing.crossing_prio_h
                && way_cost.priorityclassifier <= configs.crossing.crossing_prio_l
            {
                if let Some(cls) = edge_ctx.crossing_class {
                    if (1..=6).contains(&cls) {
                        let idx = (cls - 1) as usize;
                        if angle < 0.0 {
                            configs.crossing.cost_to_left[idx]
                        } else if angle > 0.0 {
                            configs.crossing.cost_to_right[idx]
                        } else {
                            0
                        }
                    } else {
                        0
                    }
                } else {
                    0
                }
            } else {
                0
            }
        } else {
            0
        };

        let sec = if let Some(car_model) = configs.car_kinematic {
            let delta_h = delta_h_micros as f64 * 1e-6;
            let elev_m = if ele2 == i16::MIN {
                100.0
            } else {
                selev_to_elevation(ele2)
            };
            let Some(sec) = StdPathCalculator::process_car_section(
                path_state,
                way_cost,
                car_model,
                seg_dist,
                delta_h,
                elev_m,
                angle,
                cos_angle,
                is_startpoint,
                i,
                edge_ctx.old_prio,
                edge_ctx.pre_paths,
            ) else {
                return (u32::MAX, edge_dist, 0.0, 0.0, p1.to_2d());
            };
            sec
        } else {
            let input = SectionInput {
                dist: seg_dist,
                delta_h_micros,
                cos_angle,
                crossing_penalty,
                is_detail: edge_ctx.is_detail,
            };
            StdPathCalculator::process_section(
                path_state,
                way_cost,
                configs.elev,
                configs.kinematic,
                &input,
            )
        };

        edge_cost += sec.section_cost_int;
        edge_dist += seg_dist;
        edge_time += sec.time_seconds;
        edge_energy += sec.energy_joules;

        p_prev = p1.to_2d();
    }

    let penultimate_pt = if poly.len() >= 2 {
        poly[poly.len() - 2].to_2d()
    } else {
        edge_ctx.prev_point
    };

    (
        edge_cost,
        edge_dist,
        edge_time,
        edge_energy,
        penultimate_pt,
    )
}

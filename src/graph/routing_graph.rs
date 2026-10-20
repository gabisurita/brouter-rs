//! Rust implementation corresponding to:
//! - Java class: `btools.mapaccess.OsmNodesMap`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/OsmNodesMap.java`
//! - Java class: `btools.mapaccess.NodesCache`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-mapaccess/src/main/java/btools/mapaccess/NodesCache.java`
//!
//! # Summary
//! High-performance in-memory routing graph container holding nodes and directed edges,
//! built from decoded RD5 MicroCache tiles with spatial waypoint snapping.

use std::sync::{Arc, RwLock};

#[cfg(feature = "rayon")]
use rayon::prelude::*;

use super::fast_map::FastHashMap;
use super::matcher::{MatchedWaypoint, WaypointMatcher};
use super::node::{make_node_id, OsmNode};
use super::spatial;
use crate::dsl::evaluator::ProfileEvaluator;
use crate::dsl::lookups::LookupData;
use crate::dsl::parser::Profile;
use crate::rd5::micro_cache::{DecodedGeometryPoint, DecodedMicroCache, DecodedTurnRestriction};
use crate::types::{Distance, ElevationQuarterMeters, MicroDegrees, NodeId, Point2D, Point3D};

pub const NO_DESC: u32 = u32::MAX;

/// Directed edge in the routing graph with polyline geometry and direct target slice links.
#[derive(Debug, Clone)]
pub struct GraphEdge {
    pub source_id: NodeId,
    pub target_id: NodeId,
    pub target_lon: MicroDegrees,
    pub target_lat: MicroDegrees,
    pub desc_id: u32,
    pub target_node_desc_id: u32,
    pub edge_idx: u32,
    pub rev_edge_idx: u32,
    pub target_selev: ElevationQuarterMeters,
    pub is_reverse: bool,
    pub has_node_desc: bool,
    pub has_target_tr: bool,
    pub is_target_dead_end: bool,
    pub target_edges_start: u32,
    pub target_edges_count: u16,
    pub straight_dist: Distance,
    pub geometry: Vec<DecodedGeometryPoint>,
}

impl GraphEdge {
    /// Returns the raw description bytes from the graph if present.
    #[inline]
    pub fn description<'a>(&self, graph: &'a RoutingGraph) -> Option<&'a [u8]> {
        if self.desc_id == NO_DESC {
            None
        } else {
            graph
                .descriptions
                .get(self.desc_id as usize)
                .map(|v| v.as_slice())
        }
    }

    /// Fills an existing polyline buffer (source, intermediates, target) without allocating.
    pub fn fill_polyline(
        &self,
        pts: &mut Vec<Point3D>,
        s_lon: i32,
        s_lat: i32,
        s_elev: i16,
        t_elev: i16,
    ) {
        pts.clear();
        pts.reserve(self.geometry.len() + 2);
        pts.push(Point3D::new(s_lon, s_lat, s_elev));

        if !self.is_reverse {
            let mut olon = s_lon;
            let mut olat = s_lat;
            let mut oelev = s_elev;
            for gp in &self.geometry {
                olon += gp.dlon;
                olat += gp.dlat;
                oelev = oelev.wrapping_add(gp.delev as i16);
                pts.push(Point3D::new(olon, olat, oelev));
            }
        } else {
            let mut olon = self.target_lon;
            let mut olat = self.target_lat;
            let mut oelev = t_elev;
            let start_rev = pts.len();
            for gp in &self.geometry {
                olon += gp.dlon;
                olat += gp.dlat;
                oelev = oelev.wrapping_add(gp.delev as i16);
                pts.push(Point3D::new(olon, olat, oelev));
            }
            pts[start_rev..].reverse();
        }

        pts.push(Point3D::new(self.target_lon, self.target_lat, t_elev));
    }

    /// Expands the full polyline of coordinates (source, intermediates, target)
    /// in forward traversal order (source to target).
    pub fn polyline(
        &self,
        s_lon: i32,
        s_lat: i32,
        s_elev: i16,
        t_elev: i16,
    ) -> Vec<Point3D> {
        let mut pts = Vec::with_capacity(self.geometry.len() + 2);
        self.fill_polyline(&mut pts, s_lon, s_lat, s_elev, t_elev);
        pts
    }
}

/// Provides on-demand `MicroCache` cell data (`divisor == 32` or `80`) for lazy graph expansion.
pub trait CellProvider: Send + Sync {
    /// Returns the grid divisor (`32` or `80`) for the `1° × 1°` square `(lon_degree, lat_degree)`.
    fn divisor_for_degree(&self, lon_degree: i32, lat_degree: i32) -> i32;

    /// Loads and decodes a single `MicroCache` cell at `(lon_idx, lat_idx)` within `(lon_degree, lat_degree)`.
    /// Also returns the `elevation_type` (`1` or `3`) for the parent `1° × 1°` square if known.
    fn load_cell(
        &self,
        lon_degree: i32,
        lat_degree: i32,
        lon_idx: i32,
        lat_idx: i32,
    ) -> std::io::Result<Option<(DecodedMicroCache, u8)>>;
}

#[derive(Clone, Default)]
pub struct CellProviderHandle(pub Option<Arc<dyn CellProvider>>);

impl std::fmt::Debug for CellProviderHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("CellProviderHandle")
            .field(&self.0.as_ref().map(|_| "<CellProvider>"))
            .finish()
    }
}

/// Pre-evaluated way and node costs for a compiled `Profile` across all interned descriptions in a `RoutingGraph`.
#[derive(Debug, Default, Clone)]
pub struct ProfileCostTable {
    pub way_costs: Vec<crate::dsl::evaluator::WayCostResult>,
    pub node_costs: Vec<crate::dsl::evaluator::NodeCostResult>,
}

/// In-memory graph representing the road network for routing.
#[derive(Debug, Default, Clone)]
pub struct RoutingGraph {
    pub nodes: FastHashMap<u64, OsmNode>,
    pub edges: FastHashMap<u64, Vec<GraphEdge>>,
    pub flat_edges: Vec<GraphEdge>,
    pub node_edge_offsets: FastHashMap<u64, (u32, u16)>,
    pub turn_restrictions: FastHashMap<u64, Vec<DecodedTurnRestriction>>,
    pub profile_cost_cache: Arc<RwLock<FastHashMap<u64, Arc<ProfileCostTable>>>>,
    pub elevation_types: FastHashMap<(i32, i32), u8>,
    pub descriptions: Vec<Vec<u8>>,
    pub desc_to_id: FastHashMap<Vec<u8>, u32>,
    pub node_descriptions: Vec<Vec<u8>>,
    pub node_desc_to_id: FastHashMap<Vec<u8>, u32>,
    pub spatial_grid: FastHashMap<(i32, i32), Vec<u64>>,
    pub is_resolved: bool,
    pub total_edges: usize,
    pub dead_end_parents: FastHashMap<u64, u64>,
    pub cell_provider: CellProviderHandle,
    pub loaded_cells: FastHashMap<(i32, i32), bool>,
    pub cell_divisors: FastHashMap<(i32, i32), i32>,
}

pub const GRID_CELL_SIZE: i32 = 10_000;
pub const MICRO_CACHE_CELL_SIZE: i32 = 12_500;

impl RoutingGraph {
    pub fn new() -> Self {
        Self {
            nodes: FastHashMap::default(),
            edges: FastHashMap::default(),
            flat_edges: Vec::new(),
            node_edge_offsets: FastHashMap::default(),
            turn_restrictions: FastHashMap::default(),
            profile_cost_cache: Arc::new(RwLock::new(FastHashMap::default())),
            elevation_types: FastHashMap::default(),
            descriptions: Vec::new(),
            desc_to_id: FastHashMap::default(),
            node_descriptions: Vec::new(),
            node_desc_to_id: FastHashMap::default(),
            spatial_grid: FastHashMap::default(),
            is_resolved: false,
            total_edges: 0,
            dead_end_parents: FastHashMap::default(),
            cell_provider: CellProviderHandle(None),
            loaded_cells: FastHashMap::default(),
            cell_divisors: FastHashMap::default(),
        }
    }

    pub fn with_cell_provider(provider: Arc<dyn CellProvider>) -> Self {
        let mut g = Self::new();
        g.cell_provider = CellProviderHandle(Some(provider));
        g
    }

    pub fn set_cell_provider(&mut self, provider: Option<Arc<dyn CellProvider>>) {
        self.cell_provider = CellProviderHandle(provider);
    }

    #[inline(always)]
    pub fn has_cell_provider(&self) -> bool {
        self.cell_provider.0.is_some()
    }

    /// Clears all graph nodes, edges, and cached cells while preserving the attached `CellProvider`.
    pub fn clear_graph_data(&mut self) {
        self.nodes.clear();
        self.nodes.shrink_to_fit();
        self.edges.clear();
        self.edges.shrink_to_fit();
        self.flat_edges.clear();
        self.flat_edges.shrink_to_fit();
        self.node_edge_offsets.clear();
        self.node_edge_offsets.shrink_to_fit();
        self.turn_restrictions.clear();
        self.turn_restrictions.shrink_to_fit();
        if let Ok(mut cache) = self.profile_cost_cache.write() {
            cache.clear();
        }
        self.elevation_types.clear();
        self.descriptions.clear();
        self.descriptions.shrink_to_fit();
        self.desc_to_id.clear();
        self.desc_to_id.shrink_to_fit();
        self.node_descriptions.clear();
        self.node_descriptions.shrink_to_fit();
        self.node_desc_to_id.clear();
        self.node_desc_to_id.shrink_to_fit();
        self.spatial_grid.clear();
        self.spatial_grid.shrink_to_fit();
        self.is_resolved = false;
        self.total_edges = 0;
        self.dead_end_parents.clear();
        self.dead_end_parents.shrink_to_fit();
        self.loaded_cells.clear();
        self.loaded_cells.shrink_to_fit();
        self.cell_divisors.clear();
        self.cell_divisors.shrink_to_fit();
    }

    /// Returns the number of non-empty `1/80°` cells currently loaded in the graph.
    pub fn loaded_cell_count(&self) -> usize {
        self.loaded_cells.values().filter(|&&v| v).count()
    }

    pub fn with_capacities(node_cap: usize, edge_cap: usize) -> Self {
        Self {
            nodes: FastHashMap::with_capacity_and_hasher(node_cap, Default::default()),
            edges: FastHashMap::with_capacity_and_hasher(edge_cap, Default::default()),
            flat_edges: Vec::with_capacity(edge_cap),
            node_edge_offsets: FastHashMap::with_capacity_and_hasher(node_cap, Default::default()),
            turn_restrictions: FastHashMap::default(),
            profile_cost_cache: Arc::new(RwLock::new(FastHashMap::default())),
            elevation_types: FastHashMap::default(),
            descriptions: Vec::new(),
            desc_to_id: FastHashMap::default(),
            node_descriptions: Vec::new(),
            node_desc_to_id: FastHashMap::default(),
            spatial_grid: FastHashMap::default(),
            is_resolved: false,
            total_edges: 0,
            dead_end_parents: FastHashMap::default(),
            cell_provider: CellProviderHandle(None),
            loaded_cells: FastHashMap::default(),
            cell_divisors: FastHashMap::default(),
        }
    }

    /// Evaluates or incrementally updates both way and node costs for `profile` across all
    /// currently interned descriptions in the graph. Keyed by `profile.content_hash`.
    pub fn sync_profile_costs(
        &self,
        profile: &Profile,
        lookups: &LookupData,
    ) -> Arc<ProfileCostTable> {
        let expected_way_len = self.descriptions.len() * 2;
        let expected_node_len = self.node_descriptions.len() * 4;

        if let Ok(cache) = self.profile_cost_cache.read() {
            if let Some(table) = cache.get(&profile.content_hash) {
                if table.way_costs.len() == expected_way_len
                    && table.node_costs.len() == expected_node_len
                {
                    return Arc::clone(table);
                }
            }
        }

        let mut cache = self.profile_cost_cache.write().unwrap();
        let entry = cache
            .entry(profile.content_hash)
            .or_insert_with(|| Arc::new(ProfileCostTable::default()));
        let table = Arc::make_mut(entry);

        if table.way_costs.len() < expected_way_len {
            let start_way = table.way_costs.len() / 2;
            let remaining = &self.descriptions[start_way..];

            #[cfg(feature = "rayon")]
            if start_way == 0 && remaining.len() >= 1024 {
                table.way_costs = remaining
                    .par_chunks(512)
                    .flat_map_iter(|chunk| {
                        let mut way_buf = vec![0i32; lookups.way.names.len()];
                        let mut ev = ProfileEvaluator::new(profile, &lookups.way, &lookups.node);
                        let mut results = Vec::with_capacity(chunk.len() * 2);
                        for desc_bytes in chunk {
                            lookups.way.decode_into(&mut way_buf, false, desc_bytes);
                            results.push(ev.evaluate_way(&way_buf));
                            way_buf[0] = 2;
                            results.push(ev.evaluate_way(&way_buf));
                        }
                        results
                    })
                    .collect();
            } else {
                table.way_costs.reserve(expected_way_len - table.way_costs.len());
                let mut way_buf = vec![0i32; lookups.way.names.len()];
                let mut ev = ProfileEvaluator::new(profile, &lookups.way, &lookups.node);
                for desc_bytes in remaining {
                    lookups.way.decode_into(&mut way_buf, false, desc_bytes);
                    table.way_costs.push(ev.evaluate_way(&way_buf));
                    way_buf[0] = 2;
                    table.way_costs.push(ev.evaluate_way(&way_buf));
                }
            }

            #[cfg(not(feature = "rayon"))]
            {
                table.way_costs.reserve(expected_way_len - table.way_costs.len());
                let mut way_buf = vec![0i32; lookups.way.names.len()];
                let mut ev = ProfileEvaluator::new(profile, &lookups.way, &lookups.node);
                for desc_bytes in remaining {
                    lookups.way.decode_into(&mut way_buf, false, desc_bytes);
                    table.way_costs.push(ev.evaluate_way(&way_buf));
                    way_buf[0] = 2;
                    table.way_costs.push(ev.evaluate_way(&way_buf));
                }
            }
        }

        if table.node_costs.len() < expected_node_len {
            let start_node = table.node_costs.len() / 4;
            table.node_costs.reserve(expected_node_len - table.node_costs.len());
            let crossing_tag_idx = lookups.node.get_name_idx("estimated_crossing_class");
            let mut node_buf = vec![0i32; lookups.node.names.len()];
            let mut ev = ProfileEvaluator::new(profile, &lookups.way, &lookups.node);

            for desc_bytes in &self.node_descriptions[start_node..] {
                lookups.node.decode_into(&mut node_buf, false, desc_bytes);
                let crossing_class = crossing_tag_idx
                    .and_then(|c_idx| {
                        let v_idx = *node_buf.get(c_idx)?;
                        if v_idx > 0 {
                            lookups
                                .node
                                .values
                                .get(c_idx)?
                                .get(v_idx as usize)?
                                .value
                                .parse::<u8>()
                                .ok()
                        } else {
                            None
                        }
                    })
                    .unwrap_or(0);

                if let Some(fv_idx) = ev.node_foreign_var_0 {
                    ev.way_vars[fv_idx] = 0.0;
                }
                let mut r0 = ev.evaluate_node(&node_buf);
                r0.crossing_class = crossing_class;
                table.node_costs.push(r0);

                node_buf[0] = 2;
                let mut r1 = ev.evaluate_node(&node_buf);
                r1.crossing_class = crossing_class;
                table.node_costs.push(r1);

                if let Some(fv_idx) = ev.node_foreign_var_0 {
                    ev.way_vars[fv_idx] = 1.0;
                    node_buf[0] = 1;
                    let mut r2 = ev.evaluate_node(&node_buf);
                    r2.crossing_class = crossing_class;
                    table.node_costs.push(r2);

                    node_buf[0] = 2;
                    let mut r3 = ev.evaluate_node(&node_buf);
                    r3.crossing_class = crossing_class;
                    table.node_costs.push(r3);
                    ev.way_vars[fv_idx] = 0.0;
                } else {
                    table.node_costs.push(r0);
                    table.node_costs.push(r1);
                }
            }
        }

        Arc::clone(entry)
    }

    /// Preloads profile way and node costs into the graph's cache.
    pub fn preload_profile_costs(
        &self,
        profile: &Profile,
        lookups: &LookupData,
    ) -> Arc<ProfileCostTable> {
        self.sync_profile_costs(profile, lookups)
    }

    /// Returns outgoing edges for the given node ID.
    #[inline(always)]
    pub fn get_edges(&self, s_id: u64) -> &[GraphEdge] {
        if let Some(&(start, count)) = self.node_edge_offsets.get(&s_id) {
            &self.flat_edges[start as usize..(start as usize + count as usize)]
        } else if let Some(v) = self.edges.get(&s_id) {
            v.as_slice()
        } else {
            &[]
        }
    }

    /// Returns outgoing edges directly by precomputed slice offset and count.
    #[inline(always)]
    pub fn get_edges_by_slice(&self, start: u32, count: u16) -> &[GraphEdge] {
        if count == 0 {
            &[]
        } else {
            &self.flat_edges[start as usize..(start as usize + count as usize)]
        }
    }

    /// Returns the precomputed outgoing edge slice offset and count for a node ID.
    #[inline(always)]
    pub fn get_node_edge_slice(&self, s_id: u64) -> (u32, u16) {
        let default_start = if self.has_cell_provider() { u32::MAX } else { 0 };
        self.node_edge_offsets
            .get(&s_id)
            .copied()
            .unwrap_or((default_start, 0))
    }

    pub fn get_or_intern_description(&mut self, desc: Option<&[u8]>) -> u32 {
        match desc {
            None => NO_DESC,
            Some(d) => {
                if let Some(&id) = self.desc_to_id.get(d) {
                    id
                } else {
                    let id = self.descriptions.len() as u32;
                    let v = d.to_vec();
                    self.desc_to_id.insert(v.clone(), id);
                    self.descriptions.push(v);
                    id
                }
            }
        }
    }

    pub fn get_or_intern_node_description(&mut self, desc: Option<&[u8]>) -> u32 {
        match desc {
            None => NO_DESC,
            Some(d) => {
                if let Some(&id) = self.node_desc_to_id.get(d) {
                    id
                } else {
                    let id = self.node_descriptions.len() as u32;
                    let v = d.to_vec();
                    self.node_desc_to_id.insert(v.clone(), id);
                    self.node_descriptions.push(v);
                    id
                }
            }
        }
    }

    fn insert_or_update_edge(&mut self, s_id: u64, edge: GraphEdge) {
        if let Some(&(start, count)) = self.node_edge_offsets.get(&s_id) {
            let slice =
                &mut self.flat_edges[start as usize..(start as usize + count as usize)];
            if let Some(existing) = slice.iter_mut().find(|e| {
                e.target_id == edge.target_id
                    && e.is_reverse == edge.is_reverse
                    && (e.desc_id == edge.desc_id || e.desc_id == NO_DESC || edge.desc_id == NO_DESC)
            }) {
                if existing.geometry.is_empty() && !edge.geometry.is_empty() {
                    existing.geometry = edge.geometry;
                    existing.straight_dist = 0.0;
                }
                if existing.desc_id == NO_DESC && edge.desc_id != NO_DESC {
                    existing.desc_id = edge.desc_id;
                }
                if existing.target_selev == i16::MIN && edge.target_selev != i16::MIN {
                    existing.target_selev = edge.target_selev;
                }
                if !existing.has_node_desc && edge.has_node_desc {
                    existing.has_node_desc = true;
                }
                if existing.target_node_desc_id == NO_DESC && edge.target_node_desc_id != NO_DESC {
                    existing.target_node_desc_id = edge.target_node_desc_id;
                }
                if !existing.has_target_tr && edge.has_target_tr {
                    existing.has_target_tr = true;
                }
            }
            return;
        }
        let edge_list = self.edges.entry(s_id).or_default();
        if let Some(existing) = edge_list.iter_mut().find(|e| {
            e.target_id == edge.target_id
                && e.is_reverse == edge.is_reverse
                && (e.desc_id == edge.desc_id || e.desc_id == NO_DESC || edge.desc_id == NO_DESC)
        }) {
            if existing.geometry.is_empty() && !edge.geometry.is_empty() {
                existing.geometry = edge.geometry;
            }
            if existing.desc_id == NO_DESC && edge.desc_id != NO_DESC {
                existing.desc_id = edge.desc_id;
            }
            if existing.target_selev == i16::MIN && edge.target_selev != i16::MIN {
                existing.target_selev = edge.target_selev;
            }
            if !existing.has_node_desc && edge.has_node_desc {
                existing.has_node_desc = true;
            }
            if existing.target_node_desc_id == NO_DESC && edge.target_node_desc_id != NO_DESC {
                existing.target_node_desc_id = edge.target_node_desc_id;
            }
            if !existing.has_target_tr && edge.has_target_tr {
                existing.has_target_tr = true;
            }
        } else {
            edge_list.push(edge);
        }
    }

    /// Incorporates a borrowed decoded MicroCache cell into the graph.
    pub fn add_micro_cache_ref(&mut self, cache: &DecodedMicroCache) {
        let track_grid = self.has_cell_provider();
        for node in &cache.nodes {
            let s_id = make_node_id(node.ilon, node.ilat);
            let s_lon = node.ilon;
            let s_lat = node.ilat;
            let s_elev = node.selev;

            let trs: Vec<DecodedTurnRestriction> = node.turn_restrictions.clone();
            let s_has_tr = !trs.is_empty();
            if s_has_tr {
                self.turn_restrictions.insert(s_id, trs.clone());
            }
            let n_desc_slice = node.node_tags.as_ref().map(|t| t.data.as_slice());
            let has_n_desc = n_desc_slice.is_some();
            let s_node_desc_id = self.get_or_intern_node_description(n_desc_slice);

            let osm_node = OsmNode {
                ilon: s_lon,
                ilat: s_lat,
                selev: s_elev,
                node_desc_id: s_node_desc_id,
                turn_restrictions: trs,
            };
            self.nodes.insert(s_id, osm_node);

            if track_grid {
                let cell = (
                    s_lon.div_euclid(GRID_CELL_SIZE),
                    s_lat.div_euclid(GRID_CELL_SIZE),
                );
                self.spatial_grid.entry(cell).or_default().push(s_id);
            }

            for link in &node.links {
                let t_id = make_node_id(link.target_lon, link.target_lat);
                let (t_selev, t_node_desc_id, t_has_tr) = self
                    .nodes
                    .get(&t_id)
                    .map(|n| (n.selev, n.node_desc_id, !n.turn_restrictions.is_empty()))
                    .unwrap_or((i16::MIN, NO_DESC, false));

                self.nodes
                    .entry(t_id)
                    .or_insert_with(|| OsmNode::new(link.target_lon, link.target_lat));

                let desc_id = self
                    .get_or_intern_description(link.way_tags.as_ref().map(|t| t.data.as_slice()));

                let edge_fwd = GraphEdge {
                    source_id: s_id,
                    target_id: t_id,
                    target_lon: link.target_lon,
                    target_lat: link.target_lat,
                    is_reverse: link.is_reverse,
                    target_selev: t_selev,
                    has_node_desc: t_node_desc_id != NO_DESC,
                    has_target_tr: t_has_tr,
                    is_target_dead_end: false,
                    target_edges_start: u32::MAX,
                    target_edges_count: 0,
                    desc_id,
                    target_node_desc_id: t_node_desc_id,
                    edge_idx: 0,
                    rev_edge_idx: u32::MAX,
                    straight_dist: 0.0,
                    geometry: link.geometry.clone(),
                };
                self.insert_or_update_edge(s_id, edge_fwd);

                let edge_rev = GraphEdge {
                    source_id: t_id,
                    target_id: s_id,
                    target_lon: s_lon,
                    target_lat: s_lat,
                    is_reverse: !link.is_reverse,
                    target_selev: s_elev,
                    has_node_desc: has_n_desc,
                    has_target_tr: s_has_tr,
                    is_target_dead_end: false,
                    target_edges_start: u32::MAX,
                    target_edges_count: 0,
                    desc_id,
                    target_node_desc_id: s_node_desc_id,
                    edge_idx: 0,
                    rev_edge_idx: u32::MAX,
                    straight_dist: 0.0,
                    geometry: link.geometry.clone(),
                };
                self.insert_or_update_edge(t_id, edge_rev);
            }
        }
    }

    /// Incorporates a decoded MicroCache cell into the graph.
    pub fn add_micro_cache(&mut self, cache: DecodedMicroCache) {
        self.add_micro_cache_ref(&cache);
    }

    /// Returns true if the `MicroCache` cell containing `(ilon, ilat)` has been checked/loaded
    /// (i.e. its parent `.rd5` segment was available).
    #[inline]
    fn is_cell_checked_at_microdeg(&self, ilon: i32, ilat: i32) -> bool {
        let lon_deg = ilon.div_euclid(1_000_000);
        let lat_deg = ilat.div_euclid(1_000_000);
        let Some(&divisor) = self.cell_divisors.get(&(lon_deg, lat_deg)) else {
            return false;
        };
        if divisor <= 0 {
            return false;
        }
        let cell_size = 1_000_000 / divisor;
        let lon_idx = ilon.div_euclid(cell_size);
        let lat_idx = ilat.div_euclid(cell_size);
        self.loaded_cells.contains_key(&(lon_idx, lat_idx))
    }

    /// Lazily loads the `MicroCache` cell containing microdegree coordinate `(ilon, ilat)` if not already attempted.
    #[inline]
    pub fn ensure_cell_at_microdeg(&mut self, ilon: i32, ilat: i32) -> bool {
        if self.cell_provider.0.is_none() {
            return false;
        }
        let lon_deg = ilon.div_euclid(1_000_000);
        let lat_deg = ilat.div_euclid(1_000_000);
        let divisor = if let Some(&d) = self.cell_divisors.get(&(lon_deg, lat_deg)) {
            d
        } else {
            let d = self
                .cell_provider
                .0
                .as_ref()
                .unwrap()
                .divisor_for_degree(lon_deg, lat_deg);
            if d <= 0 {
                return false;
            }
            self.cell_divisors.insert((lon_deg, lat_deg), d);
            d
        };
        let cell_size = 1_000_000 / divisor;
        let lon_idx = ilon.div_euclid(cell_size);
        let lat_idx = ilat.div_euclid(cell_size);
        if self.loaded_cells.contains_key(&(lon_idx, lat_idx)) {
            return false;
        }
        let provider = self.cell_provider.0.clone().unwrap();
        match provider.load_cell(lon_deg, lat_deg, lon_idx, lat_idx) {
            Ok(Some((cache, elev_type))) => {
                self.elevation_types.insert((lon_deg, lat_deg), elev_type);
                self.loaded_cells.insert((lon_idx, lat_idx), true);
                self.add_micro_cache_ref(&cache);
                true
            }
            Ok(None) => {
                self.loaded_cells.insert((lon_idx, lat_idx), false);
                false
            }
            Err(_) => {
                self.cell_divisors.remove(&(lon_deg, lat_deg));
                false
            }
        }
    }

    /// Ensures all `MicroCache` cells overlapping the given microdegree bounding box are loaded.
    pub fn ensure_bbox_cells_loaded(
        &mut self,
        min_ilon: i32,
        min_ilat: i32,
        max_ilon: i32,
        max_ilat: i32,
    ) {
        if !self.has_cell_provider() {
            return;
        }
        let mut y = min_ilat;
        while y <= max_ilat + MICRO_CACHE_CELL_SIZE {
            let cy = y.min(max_ilat);
            let mut x = min_ilon;
            while x <= max_ilon + MICRO_CACHE_CELL_SIZE {
                let cx = x.min(max_ilon);
                self.ensure_cell_at_microdeg(cx, cy);
                if x >= max_ilon {
                    break;
                }
                x += MICRO_CACHE_CELL_SIZE;
            }
            if y >= max_ilat {
                break;
            }
            y += MICRO_CACHE_CELL_SIZE;
        }
    }

    /// Ensures that for all candidate nodes in the spatial grid range `[min_cx..=max_cx, min_cy..=max_cy]`,
    /// any external link whose geometry is stored in the neighbor cell (`is_reverse && geometry.is_empty()`)
    /// has its neighbor cell loaded before waypoint or track snapping.
    pub fn ensure_external_link_cells_for_grid(
        &mut self,
        min_cx: i32,
        max_cx: i32,
        min_cy: i32,
        max_cy: i32,
    ) {
        if !self.has_cell_provider() || self.spatial_grid.is_empty() {
            return;
        }
        const CELL_GCD: i32 = 6_250;
        let mut target_coords: Vec<(i32, i32)> = Vec::new();
        for cx in min_cx..=max_cx {
            for cy in min_cy..=max_cy {
                if let Some(nodes) = self.spatial_grid.get(&(cx, cy)) {
                    for &s_id in nodes {
                        if let Some(edges) = self.edges.get(&s_id) {
                            for e in edges {
                                if e.is_reverse && e.geometry.is_empty() {
                                    target_coords.push((e.target_lon, e.target_lat));
                                }
                            }
                        }
                    }
                }
            }
        }
        let mut last_cell = (i32::MIN, i32::MIN);
        for (t_ilon, t_ilat) in target_coords {
            let cell = (t_ilon.div_euclid(CELL_GCD), t_ilat.div_euclid(CELL_GCD));
            if cell != last_cell {
                last_cell = cell;
                self.ensure_cell_at_microdeg(t_ilon, t_ilat);
            }
        }
    }

    /// Ensures that node `s_id` has its own cell and all neighbor cells loaded, and that its
    /// outgoing edges are resolved and moved into `self.flat_edges`.
    /// Returns `(out_edges_start, out_edges_count)`.
    pub fn ensure_node_finalized(&mut self, s_id: u64) -> (u32, u16) {
        if let Some(&slice) = self.node_edge_offsets.get(&s_id) {
            return slice;
        }
        if !self.has_cell_provider() {
            return (0, 0);
        }
        let s_pos = super::node::unpack_node_id(s_id);
        self.ensure_cell_at_microdeg(s_pos.lon, s_pos.lat);
        if !self.is_cell_checked_at_microdeg(s_pos.lon, s_pos.lat) {
            return (0, 0);
        }
        const CELL_GCD: i32 = 6_250; // gcd(12_500, 31_250)
        let mut last_cell = (
            s_pos.lon.div_euclid(CELL_GCD),
            s_pos.lat.div_euclid(CELL_GCD),
        );

        let mut stack_coords = [(0i32, 0i32); 8];
        let mut stack_len = 0usize;
        let mut overflow_coords: Option<Vec<(i32, i32)>> = None;
        if let Some(edges) = self.edges.get(&s_id) {
            if edges.len() <= stack_coords.len() {
                for (i, e) in edges.iter().enumerate() {
                    stack_coords[i] = (e.target_lon, e.target_lat);
                }
                stack_len = edges.len();
            } else {
                overflow_coords = Some(edges.iter().map(|e| (e.target_lon, e.target_lat)).collect());
            }
        }

        if let Some(coords) = overflow_coords {
            for (t_ilon, t_ilat) in coords {
                let cell = (
                    t_ilon.div_euclid(CELL_GCD),
                    t_ilat.div_euclid(CELL_GCD),
                );
                if cell != last_cell {
                    last_cell = cell;
                    self.ensure_cell_at_microdeg(t_ilon, t_ilat);
                }
            }
        } else {
            for &(t_ilon, t_ilat) in &stack_coords[..stack_len] {
                let cell = (
                    t_ilon.div_euclid(CELL_GCD),
                    t_ilat.div_euclid(CELL_GCD),
                );
                if cell != last_cell {
                    last_cell = cell;
                    self.ensure_cell_at_microdeg(t_ilon, t_ilat);
                }
            }
        }

        let Some(edges) = self.edges.remove(&s_id) else {
            if self.nodes.contains_key(&s_id) {
                self.node_edge_offsets.insert(s_id, (0, 0));
            }
            return (0, 0);
        };

        let start = self.flat_edges.len() as u32;
        let count = edges.len() as u16;
        self.node_edge_offsets.insert(s_id, (start, count));

        for (e_i, mut edge) in edges.into_iter().enumerate() {
            let edge_idx = start + e_i as u32;
            edge.edge_idx = edge_idx;
            if let Some(target_node) = self.nodes.get(&edge.target_id) {
                if edge.target_selev == i16::MIN && target_node.selev != i16::MIN {
                    edge.target_selev = target_node.selev;
                }
                edge.has_node_desc = target_node.node_desc_id != NO_DESC;
                edge.target_node_desc_id = target_node.node_desc_id;
                edge.has_target_tr = !target_node.turn_restrictions.is_empty();
            }
            if edge.geometry.is_empty() {
                edge.straight_dist =
                    spatial::distance(s_pos, Point2D::new(edge.target_lon, edge.target_lat))
                        .round()
                        .max(1.0);
            }
            let target_cell_checked =
                self.is_cell_checked_at_microdeg(edge.target_lon, edge.target_lat);
            let t_offset = self.node_edge_offsets.get(&edge.target_id).copied();
            let is_dead = if !target_cell_checked {
                false
            } else if let Some(&parent) = self.dead_end_parents.get(&edge.target_id) {
                parent == s_id
            } else if let Some((t_start, t_count)) = t_offset {
                if t_count == 0 {
                    true
                } else {
                    let t_slice =
                        &self.flat_edges[t_start as usize..(t_start as usize + t_count as usize)];
                    let only_s = t_slice.iter().all(|te| te.target_id == s_id);
                    if only_s {
                        self.dead_end_parents.insert(edge.target_id, s_id);
                    }
                    only_s
                }
            } else if let Some(t_edges) = self.edges.get(&edge.target_id) {
                let only_s = !t_edges.is_empty() && t_edges.iter().all(|te| te.target_id == s_id);
                if only_s {
                    self.dead_end_parents.insert(edge.target_id, s_id);
                }
                only_s
            } else {
                !self.nodes.contains_key(&edge.target_id)
            };
            edge.is_target_dead_end = is_dead;

            if let Some((t_start, t_count)) = t_offset {
                edge.target_edges_start = t_start;
                edge.target_edges_count = t_count;
                let t_end = t_start as usize + t_count as usize;
                let mut exact_rev: Option<usize> = None;
                let mut fallback_rev: Option<usize> = None;
                for rev_idx in (t_start as usize)..t_end {
                    let rev_edge = &mut self.flat_edges[rev_idx];
                    if rev_edge.target_id == s_id {
                        rev_edge.target_edges_start = start;
                        rev_edge.target_edges_count = count;
                        if rev_edge.desc_id == edge.desc_id
                            && rev_edge.is_reverse != edge.is_reverse
                            && rev_edge.rev_edge_idx == u32::MAX
                        {
                            exact_rev = Some(rev_idx);
                            break;
                        } else if fallback_rev.is_none() {
                            fallback_rev = Some(rev_idx);
                        }
                    }
                }
                // Ensure remaining parallel reverse edges also get target_edges_start/count updated
                for rev_idx in (t_start as usize)..t_end {
                    let rev_edge = &mut self.flat_edges[rev_idx];
                    if rev_edge.target_id == s_id {
                        rev_edge.target_edges_start = start;
                        rev_edge.target_edges_count = count;
                    }
                }
                if let Some(rev_idx) = exact_rev.or(fallback_rev) {
                    edge.rev_edge_idx = rev_idx as u32;
                    if self.flat_edges[rev_idx].rev_edge_idx == u32::MAX {
                        self.flat_edges[rev_idx].rev_edge_idx = edge_idx;
                    }
                }
            }
            self.flat_edges.push(edge);
        }
        self.total_edges = self.flat_edges.len();
        self.is_resolved = true;
        (start, count)
    }

    /// Returns the number of nodes in the graph.
    #[inline]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns total number of directed edges in the graph.
    pub fn edge_count(&self) -> usize {
        if self.total_edges > 0 {
            self.total_edges
        } else {
            self.edges.values().map(|v| v.len()).sum()
        }
    }

    /// Snaps geographic coordinates `(lon_deg, lat_deg)` to the nearest road edge,
    /// lazily loading `1/80°` cells around the coordinate if a `CellProvider` is attached.
    pub fn snap_waypoint_lazy(
        &mut self,
        name: &str,
        lon_deg: f64,
        lat_deg: f64,
        max_distance_meters: f64,
        evaluator: &mut ProfileEvaluator,
        lookups: &LookupData,
    ) -> Option<MatchedWaypoint> {
        self.snap_waypoint_with_direction_lazy(
            name,
            lon_deg,
            lat_deg,
            max_distance_meters,
            -1.0,
            evaluator,
            lookups,
        )
    }

    /// Snaps geographic coordinates `(lon_deg, lat_deg)` with direction tie-breaking,
    /// lazily loading `1/80°` cells around the coordinate if a `CellProvider` is attached.
    #[allow(clippy::too_many_arguments)]
    pub fn snap_waypoint_with_direction_lazy(
        &mut self,
        name: &str,
        lon_deg: f64,
        lat_deg: f64,
        max_distance_meters: f64,
        direction_to_next: f64,
        evaluator: &mut ProfileEvaluator,
        lookups: &LookupData,
    ) -> Option<MatchedWaypoint> {
        if self.has_cell_provider() {
            let wp_lon = spatial::lon_to_ilon(lon_deg);
            let wp_lat = spatial::lat_to_ilat(lat_deg);
            let scales = spatial::get_lon_lat_to_meter_scales(wp_lat);
            let max_dlon = (max_distance_meters / scales[0]).ceil() as i32 + 500;
            let max_dlat = (max_distance_meters / scales[1]).ceil() as i32 + 500;
            let min_lon = wp_lon - max_dlon;
            let max_lon = wp_lon + max_dlon;
            let min_lat = wp_lat - max_dlat;
            let max_lat = wp_lat + max_dlat;

            let min_cx = (min_lon - 20000).div_euclid(GRID_CELL_SIZE);
            let max_cx = (max_lon + 20000).div_euclid(GRID_CELL_SIZE);
            let min_cy = (min_lat - 20000).div_euclid(GRID_CELL_SIZE);
            let max_cy = (max_lat + 20000).div_euclid(GRID_CELL_SIZE);

            self.ensure_bbox_cells_loaded(
                min_cx * GRID_CELL_SIZE,
                min_cy * GRID_CELL_SIZE,
                (max_cx + 1) * GRID_CELL_SIZE,
                (max_cy + 1) * GRID_CELL_SIZE,
            );
            self.ensure_external_link_cells_for_grid(min_cx, max_cx, min_cy, max_cy);
        }

        let res = self.snap_waypoint_with_direction(
            name,
            lon_deg,
            lat_deg,
            max_distance_meters,
            direction_to_next,
            evaluator,
            lookups,
        );
        if let Some(ref mwp) = res {
            if self.has_cell_provider() {
                self.ensure_node_finalized(mwp.node1.node_id());
                self.ensure_node_finalized(mwp.node2.node_id());
            }
        }
        res
    }

    /// Snaps geographic coordinates `(lon_deg, lat_deg)` to the nearest road edge
    /// within `max_distance_meters` that is accessible according to `evaluator`.
    pub fn snap_waypoint(
        &self,
        name: &str,
        lon_deg: f64,
        lat_deg: f64,
        max_distance_meters: f64,
        evaluator: &mut ProfileEvaluator,
        lookups: &LookupData,
    ) -> Option<MatchedWaypoint> {
        self.snap_waypoint_with_direction(
            name,
            lon_deg,
            lat_deg,
            max_distance_meters,
            -1.0,
            evaluator,
            lookups,
        )
    }

    /// Returns true if the undirected passable component containing `(s_id, t_id)` has at least `min_nodes` nodes.
    /// Rejects isolated OSM micro-islands (such as courtyard service roads surrounded by footways or private ways).
    #[inline]
    pub fn is_connected_passable_edge(
        &self,
        s_id: u64,
        t_id: u64,
        min_nodes: usize,
        evaluator: &mut ProfileEvaluator,
        lookups: &LookupData,
        way_lookup_buf: &mut [i32],
    ) -> bool {
        let mut seen = [0u64; 12];
        let mut count = 0usize;
        seen[count] = s_id;
        count += 1;
        if t_id != s_id {
            seen[count] = t_id;
            count += 1;
        }
        let mut head = 0usize;
        while head < count && count < min_nodes {
            let u = seen[head];
            head += 1;
            for e in self.get_edges(u) {
                if seen[..count].contains(&e.target_id) {
                    continue;
                }
                let Some(desc_bytes) = e.description(self) else {
                    continue;
                };
                lookups.way.decode_into(way_lookup_buf, e.is_reverse, desc_bytes);
                let mut passable = evaluator.evaluate_way(way_lookup_buf).costfactor < 9999.0;
                if !passable {
                    lookups.way.decode_into(way_lookup_buf, !e.is_reverse, desc_bytes);
                    passable = evaluator.evaluate_way(way_lookup_buf).costfactor < 9999.0;
                }
                if passable {
                    seen[count] = e.target_id;
                    count += 1;
                    if count >= min_nodes {
                        return true;
                    }
                }
            }
        }
        count >= min_nodes
    }

    /// Snaps geographic coordinates `(lon_deg, lat_deg)` to the nearest road edge
    /// within `max_distance_meters` that is accessible according to `evaluator`,
    /// using `direction_to_next` for tie-breaking and `check_start_way` when `name == "from"`.
    #[allow(clippy::too_many_arguments)]
    pub fn snap_waypoint_with_direction(
        &self,
        name: &str,
        lon_deg: f64,
        lat_deg: f64,
        max_distance_meters: f64,
        direction_to_next: f64,
        evaluator: &mut ProfileEvaluator,
        lookups: &LookupData,
    ) -> Option<MatchedWaypoint> {
        let wp_lon = spatial::lon_to_ilon(lon_deg);
        let wp_lat = spatial::lat_to_ilat(lat_deg);

        let mut mwp = MatchedWaypoint::with_direction(
            name,
            Point2D::new(wp_lon, wp_lat),
            max_distance_meters,
            direction_to_next,
        );
        let mut found = false;
        let is_from = name == "from";

        // Bounding box filter for quick rejection
        let scales = spatial::get_lon_lat_to_meter_scales(wp_lat);
        let max_dlon = (max_distance_meters / scales[0]).ceil() as i32 + 500;
        let max_dlat = (max_distance_meters / scales[1]).ceil() as i32 + 500;

        let mut min_lon = wp_lon - max_dlon;
        let mut max_lon = wp_lon + max_dlon;
        let mut min_lat = wp_lat - max_dlat;
        let mut max_lat = wp_lat + max_dlat;

        let mut way_lookup_buf = vec![0i32; lookups.way.names.len()];
        let mut poly: Vec<Point3D> = Vec::with_capacity(32);

        let mut candidate_nodes: Vec<u64> = Vec::new();
        if !self.spatial_grid.is_empty() {
            let min_cx = (min_lon - 20000).div_euclid(GRID_CELL_SIZE);
            let max_cx = (max_lon + 20000).div_euclid(GRID_CELL_SIZE);
            let min_cy = (min_lat - 20000).div_euclid(GRID_CELL_SIZE);
            let max_cy = (max_lat + 20000).div_euclid(GRID_CELL_SIZE);
            for cx in min_cx..=max_cx {
                for cy in min_cy..=max_cy {
                    if let Some(nodes) = self.spatial_grid.get(&(cx, cy)) {
                        candidate_nodes.extend_from_slice(nodes);
                    }
                }
            }
        }

        let node_iter: Box<dyn Iterator<Item = u64> + '_> = if !candidate_nodes.is_empty() {
            Box::new(candidate_nodes.into_iter())
        } else if self.is_resolved {
            Box::new(self.node_edge_offsets.keys().copied())
        } else {
            Box::new(self.edges.keys().copied())
        };

        for s_id in node_iter {
            let s_pos = super::node::unpack_node_id(s_id);
            let s_ilon = s_pos.lon;
            let s_ilat = s_pos.lat;

            // Check if source node is reasonably near the bounding box
            if s_ilon < min_lon - 50000 || s_ilon > max_lon + 50000 {
                continue;
            }
            if s_ilat < min_lat - 50000 || s_ilat > max_lat + 50000 {
                continue;
            }

            let Some(source_node) = self.nodes.get(&s_id) else {
                continue;
            };

            let edge_vec = self.get_edges(s_id);
            if edge_vec.is_empty() {
                continue;
            }

            for edge in edge_vec {
                let edge_min_lon = s_ilon.min(edge.target_lon);
                let edge_max_lon = s_ilon.max(edge.target_lon);
                let edge_min_lat = s_ilat.min(edge.target_lat);
                let edge_max_lat = s_ilat.max(edge.target_lat);

                let margin = if edge.geometry.is_empty() { 0 } else { 10000 };
                if edge_max_lon + margin < min_lon
                    || edge_min_lon - margin > max_lon
                    || edge_max_lat + margin < min_lat
                    || edge_min_lat - margin > max_lat
                {
                    continue;
                }

                let Some(desc_bytes) = edge.description(self) else {
                    continue;
                };

                let t_elev = if edge.target_selev != i16::MIN {
                    edge.target_selev
                } else {
                    self.nodes
                        .get(&edge.target_id)
                        .map(|n| n.selev)
                        .unwrap_or(source_node.selev)
                };

                edge.fill_polyline(
                    &mut poly,
                    source_node.ilon,
                    source_node.ilat,
                    source_node.selev,
                    t_elev,
                );

                let mut edge_valid: Option<bool> = None;
                for i in 0..poly.len() - 1 {
                    let p1 = &poly[i];
                    let p2 = &poly[i + 1];

                    // Segment bounding box check
                    let seg_min_lon = p1.lon.min(p2.lon);
                    let seg_max_lon = p1.lon.max(p2.lon);
                    let seg_min_lat = p1.lat.min(p2.lat);
                    let seg_max_lat = p1.lat.max(p2.lat);

                    if seg_max_lon < min_lon
                        || seg_min_lon > max_lon
                        || seg_max_lat < min_lat
                        || seg_min_lat > max_lat
                    {
                        continue;
                    }

                    let mut cand_mwp = MatchedWaypoint {
                        name: String::new(),
                        waypoint: mwp.waypoint,
                        crosspoint: mwp.crosspoint,
                        node1: mwp.node1,
                        node2: mwp.node2,
                        radius: mwp.radius,
                        direction_to_next: mwp.direction_to_next,
                        direction_diff: mwp.direction_diff,
                    };

                    if !WaypointMatcher::check_segment(
                        &mut cand_mwp,
                        p1.to_2d(),
                        p2.to_2d(),
                        Point2D::new(source_node.ilon, source_node.ilat),
                        Point2D::new(edge.target_lon, edge.target_lat),
                    ) {
                        continue;
                    }

                    let valid = *edge_valid.get_or_insert_with(|| {
                        lookups
                            .way
                            .decode_into(&mut way_lookup_buf, edge.is_reverse, desc_bytes);
                        if is_from && !evaluator.profile.check_start_way_decoded(&way_lookup_buf) {
                            return false;
                        }
                        let way_cost = evaluator.evaluate_way(&way_lookup_buf);
                        if way_cost.costfactor >= 9999.0 {
                            return false;
                        }
                        self.is_connected_passable_edge(
                            s_id,
                            edge.target_id,
                            6,
                            evaluator,
                            lookups,
                            &mut way_lookup_buf,
                        )
                    });
                    if !valid {
                        break;
                    }

                    mwp.radius = cand_mwp.radius;
                    mwp.crosspoint = cand_mwp.crosspoint;
                    mwp.node1 = cand_mwp.node1;
                    mwp.node2 = cand_mwp.node2;
                    mwp.direction_diff = cand_mwp.direction_diff;
                    found = true;

                    let cur_dlon = (mwp.radius / scales[0]).ceil() as i32 + 50;
                    let cur_dlat = (mwp.radius / scales[1]).ceil() as i32 + 50;
                    min_lon = wp_lon - cur_dlon;
                    max_lon = wp_lon + cur_dlon;
                    min_lat = wp_lat - cur_dlat;
                    max_lat = wp_lat + cur_dlat;
                }
            }
        }

        if found {
            Some(mwp)
        } else {
            None
        }
    }

    /// Resolves target node elevation and node description presence for all edges.
    ///
    /// Precomputes dead-end subtrees via iterative topological leaf peeling matching
    /// Java class `btools.mapaccess.OsmNodesMap`:
    /// Nodes with 0 or 1 outgoing target are peeled as leaves. Each leaf records its
    /// parent node in `dead_end_parents`. When a parent node loses all but one remaining
    /// target, it is in turn peeled into the queue. During routing search, edges leading
    /// into dead-end subtrees are fast-skipped unless the target waypoint lies within that subtree.
    pub fn resolve_edge_target_elevations(&mut self) {
        let mut dead_end_parents: FastHashMap<u64, u64> =
            FastHashMap::with_capacity_and_hasher(self.edges.len() / 4, Default::default());
        let mut queue: Vec<u64> = Vec::new();
        let mut remaining_targets: FastHashMap<u64, Vec<u64>> =
            FastHashMap::with_capacity_and_hasher(self.edges.len(), Default::default());

        for (&s_id, edges) in &self.edges {
            let mut unique_targets: Vec<u64> = Vec::with_capacity(edges.len());
            for e in edges {
                if !unique_targets.contains(&e.target_id) {
                    unique_targets.push(e.target_id);
                }
            }
            if unique_targets.is_empty() {
                dead_end_parents.insert(s_id, 0);
                queue.push(s_id);
            } else if unique_targets.len() == 1 {
                let parent = unique_targets[0];
                dead_end_parents.insert(s_id, parent);
                queue.push(s_id);
            } else {
                remaining_targets.insert(s_id, unique_targets);
            }
        }

        while let Some(leaf) = queue.pop() {
            if let Some(&parent) = dead_end_parents.get(&leaf) {
                if parent != 0 {
                    if let Some(targets) = remaining_targets.get_mut(&parent) {
                        targets.retain(|&t| t != leaf);
                        if targets.len() == 1 {
                            let grand_parent = targets[0];
                            dead_end_parents.insert(parent, grand_parent);
                            queue.push(parent);
                        } else if targets.is_empty() {
                            dead_end_parents.insert(parent, 0);
                            queue.push(parent);
                        }
                    }
                }
            }
        }

        let total_edge_count: usize = self.edges.values().map(|v| v.len()).sum();
        let mut flat_edges = Vec::with_capacity(total_edge_count);
        let mut node_edge_offsets =
            FastHashMap::with_capacity_and_hasher(self.edges.len(), Default::default());

        for (&s_id, edges) in self.edges.iter_mut() {
            let s_pos = super::node::unpack_node_id(s_id);
            let start = flat_edges.len() as u32;
            let count = edges.len() as u16;
            node_edge_offsets.insert(s_id, (start, count));

            for (e_i, edge) in edges.iter_mut().enumerate() {
                edge.edge_idx = start + e_i as u32;
                if let Some(target_node) = self.nodes.get(&edge.target_id) {
                    if edge.target_selev == i16::MIN && target_node.selev != i16::MIN {
                        edge.target_selev = target_node.selev;
                    }
                    edge.has_node_desc = target_node.node_desc_id != NO_DESC;
                    edge.target_node_desc_id = target_node.node_desc_id;
                    edge.has_target_tr = !target_node.turn_restrictions.is_empty();
                }
                if edge.geometry.is_empty() {
                    edge.straight_dist =
                        spatial::distance(s_pos, Point2D::new(edge.target_lon, edge.target_lat))
                            .round()
                            .max(1.0);
                }
                let is_dead = match dead_end_parents.get(&edge.target_id) {
                    Some(&parent) => parent == s_id,
                    None => !self.nodes.contains_key(&edge.target_id),
                };
                edge.is_target_dead_end = is_dead;
            }
            flat_edges.append(edges);
        }

        self.spatial_grid.clear();
        for &s_id in node_edge_offsets.keys() {
            let pos = super::node::unpack_node_id(s_id);
            let cell = (
                pos.lon.div_euclid(GRID_CELL_SIZE),
                pos.lat.div_euclid(GRID_CELL_SIZE),
            );
            self.spatial_grid.entry(cell).or_default().push(s_id);
        }
        self.total_edges = flat_edges.len();

        // Resolve bidirectional counterlink indices and target edge slice offsets
        for i in 0..flat_edges.len() {
            let target_id = flat_edges[i].target_id;
            let source_id = flat_edges[i].source_id;
            let desc_id = flat_edges[i].desc_id;
            let is_rev = flat_edges[i].is_reverse;
            if let Some(&(t_start, t_count)) = node_edge_offsets.get(&target_id) {
                flat_edges[i].target_edges_start = t_start;
                flat_edges[i].target_edges_count = t_count;
                let t_end = t_start as usize + t_count as usize;
                let mut matched_rev = u32::MAX;
                for rev_idx in (t_start as usize)..t_end {
                    let rev_edge = &flat_edges[rev_idx];
                    if rev_edge.target_id == source_id {
                        if rev_edge.desc_id == desc_id && rev_edge.is_reverse != is_rev {
                            matched_rev = rev_idx as u32;
                            break;
                        } else if matched_rev == u32::MAX {
                            matched_rev = rev_idx as u32;
                        }
                    }
                }
                flat_edges[i].rev_edge_idx = matched_rev;
            } else {
                flat_edges[i].target_edges_start = 0;
                flat_edges[i].target_edges_count = 0;
            }
        }

        self.flat_edges = flat_edges;
        self.node_edge_offsets = node_edge_offsets;
        self.dead_end_parents = dead_end_parents;
        self.edges.clear();
        self.edges.shrink_to_fit();
        self.is_resolved = true;
    }

    /// Loads all microcaches in a bounding box of 1°x1° squares into the graph.
    pub fn load_bounding_box(
        &mut self,
        tile_mgr: &mut crate::rd5::tile_manager::TileManager,
        min_lon: f64,
        min_lat: f64,
        max_lon: f64,
        max_lat: f64,
    ) -> std::io::Result<usize> {
        let min_lon_deg = (min_lon + 180.0).floor() as i32;
        let max_lon_deg = (max_lon + 180.0).floor() as i32;
        let min_lat_deg = (min_lat + 90.0).floor() as i32;
        let max_lat_deg = (max_lat + 90.0).floor() as i32;

        let mut loaded = 0;
        for lon_deg in min_lon_deg..=max_lon_deg {
            for lat_deg in min_lat_deg..=max_lat_deg {
                let elev_type = tile_mgr.get_elevation_type(lon_deg, lat_deg);
                self.elevation_types.insert((lon_deg, lat_deg), elev_type);
                if let Ok(caches) = tile_mgr.decode_square_cached(lon_deg, lat_deg) {
                    for cache in caches.iter() {
                        self.add_micro_cache_ref(cache);
                        loaded += 1;
                    }
                }
            }
        }
        self.resolve_edge_target_elevations();
        Ok(loaded)
    }

    /// Returns the elevation type for given microdegree coordinates (1 = 1-arcsec, 3 = 3-arcsec).
    pub fn get_elevation_type(&self, ilon: i32, ilat: i32) -> u8 {
        let lon_deg = ilon / 1_000_000;
        let lat_deg = ilat / 1_000_000;
        self.elevation_types
            .get(&(lon_deg, lat_deg))
            .copied()
            .unwrap_or(3)
    }

    /// Returns the filtered ascend threshold matching BRouter:
    /// -5.0 m for 1-arcsec tiles (elevationType == 1), -10.0 m for 3-arcsec tiles.
    pub fn elevation_filter(&self, ilon: i32, ilat: i32) -> f64 {
        if self.get_elevation_type(ilon, ilat) == 1 {
            -5.0
        } else {
            -10.0
        }
    }
}

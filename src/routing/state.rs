//! Internal search data structures and state management for A* routing.
//!
//! Includes compact open-set node representation, min-heap priority items,
//! slot-recycled node pools, and edge candidate dominance tracking.

use std::cmp::Ordering;

use super::path::PathState;
use crate::types::{Cost, ElevationQuarterMeters, MicroDegrees, NodeId, Point3D};

/// Candidate path reaching an edge, used for Pareto dominance pruning and multi-candidate expansion.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct CandidateState {
    pub cost: Cost,
    pub ehbd: i32,
    pub ehbu: i32,
    pub history_idx: u32,
    pub path_selev: ElevationQuarterMeters,
    pub in_destination: bool,
    pub can_leave_destination: bool,
}

impl CandidateState {
    #[inline(always)]
    pub fn new(cost: Cost, ehbd: i32, ehbu: i32) -> Self {
        Self {
            cost,
            ehbd,
            ehbu,
            history_idx: u32::MAX,
            path_selev: i16::MIN,
            in_destination: false,
            can_leave_destination: false,
        }
    }

    #[inline(always)]
    pub fn with_context(
        cost: Cost,
        ehbd: i32,
        ehbu: i32,
        history_idx: u32,
        path_selev: ElevationQuarterMeters,
        in_destination: bool,
        can_leave_destination: bool,
    ) -> Self {
        Self {
            cost,
            ehbd,
            ehbu,
            history_idx,
            path_selev,
            in_destination,
            can_leave_destination,
        }
    }

    /// Evaluates if `self` is definitely worse than `existing` under Pareto dominance
    /// across cumulative cost and elevation hysteresis buffers.
    ///
    /// Matches `StdPath.definitlyWorseThan` (and `KinematicPath.definitlyWorseThan` when `downhillcostdiv < 0`)
    /// in Java BRouter.
    #[inline(always)]
    pub fn is_definitely_worse_than(
        &self,
        existing: &Self,
        downhillcostdiv: i32,
        uphillcostdiv: i32,
    ) -> bool {
        if downhillcostdiv < 0 {
            return self.cost > existing.cost.saturating_add(100);
        }
        let mut c = existing.cost;
        if downhillcostdiv > 0 {
            let delta = existing.ehbd / downhillcostdiv - self.ehbd / downhillcostdiv;
            if delta > 0 {
                c = c.saturating_add(delta as u32);
            }
        }
        if uphillcostdiv > 0 {
            let delta = existing.ehbu / uphillcostdiv - self.ehbu / uphillcostdiv;
            if delta > 0 {
                c = c.saturating_add(delta as u32);
            }
        }
        self.cost > c
    }
}

pub const INLINE_CANDS: usize = 6;

/// Multi-candidate dominance state per directed edge.
///
/// Uses an inline array of up to 6 candidates to avoid dynamic heap allocations.
#[derive(Clone, Copy)]
pub struct EdgeState {
    pub cands: [CandidateState; INLINE_CANDS],
    pub len: u8,
}

impl Default for EdgeState {
    #[inline(always)]
    fn default() -> Self {
        Self {
            cands: [CandidateState::default(); INLINE_CANDS],
            len: 0,
        }
    }
}

impl EdgeState {
    #[inline(always)]
    pub fn candidates(&self) -> &[CandidateState] {
        &self.cands[..self.len as usize]
    }

    #[inline(always)]
    pub fn retain<F>(&mut self, mut f: F)
    where
        F: FnMut(&CandidateState) -> bool,
    {
        let mut new_len = 0;
        for i in 0..self.len as usize {
            if f(&self.cands[i]) {
                self.cands[new_len] = self.cands[i];
                new_len += 1;
            }
        }
        self.len = new_len as u8;
    }

    #[inline(always)]
    pub fn push(&mut self, cand: CandidateState) {
        let len = self.len as usize;
        if len < INLINE_CANDS {
            self.cands[len] = cand;
            self.len += 1;
        } else {
            let mut max_idx = 0;
            for i in 1..len {
                if self.cands[i].cost > self.cands[max_idx].cost {
                    max_idx = i;
                }
            }
            if cand.cost < self.cands[max_idx].cost {
                self.cands[max_idx] = cand;
            }
        }
    }
}

pub use crate::graph::fast_map::{FastBuildHasher, FastHashMap, FastHashSet, FastHasher};

/// Search node stored in the preallocated node pool.
#[derive(Clone, Copy, Default)]
pub struct SearchNode {
    pub node_id: NodeId,
    pub from_node_id: NodeId,
    pub ilon: MicroDegrees,
    pub ilat: MicroDegrees,
    pub cost: Cost,
    pub air_dist: Cost,
    pub priority: Cost,
    pub path_state: PathState,
    pub prev_lon: MicroDegrees,
    pub prev_lat: MicroDegrees,
    pub priority_classifier: i32,
    pub down_div: i32,
    pub up_div: i32,
    pub history_idx: u32,
    pub selev: ElevationQuarterMeters,
    pub path_selev: ElevationQuarterMeters,
    pub last_classifier: u8,
    pub in_destination: bool,
    pub can_leave_destination: bool,
    pub has_tr: bool,
    pub edge_idx: u32,
    pub rev_edge_idx: u32,
    pub out_edges_start: u32,
    pub out_edges_count: u16,
    pub node_desc_id: u32,
}

/// Compact 8-byte item stored in the BinaryHeap priority queue.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct HeapItem {
    pub priority: Cost,
    pub node_idx: u32,
}

impl Ord for HeapItem {
    #[inline(always)]
    fn cmp(&self, other: &Self) -> Ordering {
        other.priority.cmp(&self.priority) // min-heap
    }
}

impl PartialOrd for HeapItem {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Cache-efficient 4-ary min-heap for search items.
///
/// Compared to standard binary heaps:
/// 1. Halves heap depth: log4(N) vs log2(N), halving cache misses on tree traversal.
/// 2. Four 8-byte children fit in 32 bytes (half a cache line), making child selection in sift-down cache-local.
/// 3. Sift-up does half the comparisons using bit-shift parent calculation ((hole - 1) >> 2).
pub struct QuadHeap {
    data: Vec<HeapItem>,
}

impl QuadHeap {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
        }
    }

    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    #[inline(always)]
    pub fn push(&mut self, item: HeapItem) {
        let mut hole = self.data.len();
        self.data.push(item);
        while hole > 0 {
            let parent = (hole - 1) >> 2;
            if item.priority < self.data[parent].priority {
                self.data[hole] = self.data[parent];
                hole = parent;
            } else {
                break;
            }
        }
        self.data[hole] = item;
    }

    #[inline(always)]
    pub fn pop(&mut self) -> Option<HeapItem> {
        if self.data.is_empty() {
            return None;
        }
        let root = self.data[0];
        let last = self.data.pop().unwrap();
        let len = self.data.len();
        if len > 0 {
            let mut hole = 0;
            loop {
                let child_base = (hole << 2) + 1;
                if child_base >= len {
                    break;
                }
                let mut min_child = child_base;
                let mut min_prio = self.data[child_base].priority;

                let c1 = child_base + 1;
                if c1 < len && self.data[c1].priority < min_prio {
                    min_prio = self.data[c1].priority;
                    min_child = c1;
                }
                let c2 = child_base + 2;
                if c2 < len && self.data[c2].priority < min_prio {
                    min_prio = self.data[c2].priority;
                    min_child = c2;
                }
                let c3 = child_base + 3;
                if c3 < len && self.data[c3].priority < min_prio {
                    min_prio = self.data[c3].priority;
                    min_child = c3;
                }

                if last.priority <= min_prio {
                    break;
                }
                self.data[hole] = self.data[min_child];
                hole = min_child;
            }
            self.data[hole] = last;
        }
        Some(root)
    }

    #[inline(always)]
    pub fn clear(&mut self) {
        self.data.clear();
    }

    #[inline(always)]
    pub fn reserve(&mut self, capacity: usize) {
        if self.data.capacity() < capacity {
            self.data.reserve(capacity - self.data.capacity());
        }
    }
}


/// Preallocated memory pool with slot recycling for search nodes.
pub struct NodePool {
    nodes: Vec<SearchNode>,
    free_list: Vec<u32>,
}

impl NodePool {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            nodes: Vec::with_capacity(capacity),
            free_list: Vec::with_capacity(capacity / 4),
        }
    }

    #[inline(always)]
    pub fn alloc(&mut self, node: SearchNode) -> u32 {
        if let Some(idx) = self.free_list.pop() {
            self.nodes[idx as usize] = node;
            idx
        } else {
            let idx = self.nodes.len() as u32;
            self.nodes.push(node);
            idx
        }
    }

    #[inline(always)]
    pub fn free(&mut self, idx: u32) -> SearchNode {
        let node = self.nodes[idx as usize];
        self.free_list.push(idx);
        node
    }

    #[inline(always)]
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.free_list.clear();
    }

    #[inline(always)]
    pub fn reserve(&mut self, capacity: usize) {
        if self.nodes.capacity() < capacity {
            self.nodes.reserve(capacity - self.nodes.capacity());
        }
    }
}


/// Step recorded in the search history DAG for path backtracking.
#[derive(Clone, Copy)]
pub struct HistoryStep {
    pub node_id: NodeId,
    pub from_node_id: NodeId,
    pub cost: Cost,
    pub global_edge_idx: u32,
    pub edge_idx: u16,
    pub maxspeed: f32,
    /// Predecessor index in history array (u32::MAX represents None/root).
    pub prev_idx: u32,
}

/// Reconstructed track step with optional sliced polyline points.
#[derive(Clone)]
pub struct TrackStep {
    pub node_id: NodeId,
    pub from_node_id: NodeId,
    pub cost: Cost,
    pub global_edge_idx: u32,
    pub edge_idx: u16,
    pub maxspeed: f32,
    pub sliced_points: Option<Vec<Point3D>>,
}

/// Full sequence of steps produced by an A* search pass.
#[derive(Clone)]
pub struct SearchTrack {
    pub steps: Vec<TrackStep>,
    pub total_cost: Cost,
}

/// Cache-resident cost cutting guide built from Pass 0 search results.
///
/// Maps directed global edge indices along the winning Pass 0 route to their
/// cumulative cost. Stored in a compact, power-of-two open-addressing table (typically 16-128 KB)
/// that fits entirely within L1/L2 CPU cache, eliminating 30+ MB heap allocations
/// and DRAM cache misses during Pass 1 edge relaxation.
#[derive(Debug, Clone)]
pub struct Pass0CostGuide {
    table: Vec<(u32, Cost)>,
    mask: usize,
    pub pass0_total_cost: Cost,
}

impl Pass0CostGuide {
    /// Creates a new guide with capacity proportional to the number of steps in the Pass 0 route.
    pub fn new(step_count: usize, pass0_total_cost: Cost) -> Self {
        let cap = (step_count * 3).next_power_of_two().max(512);
        Self {
            table: vec![(u32::MAX, 0); cap],
            mask: cap - 1,
            pass0_total_cost,
        }
    }

    /// Records an edge on the winning Pass 0 path with its cumulative cost.
    #[inline(always)]
    pub fn record_edge(&mut self, edge_idx: u32, cost: Cost) {
        if edge_idx == u32::MAX {
            return;
        }
        let mut idx = (edge_idx as usize) & self.mask;
        loop {
            let entry = &mut self.table[idx];
            if entry.0 == u32::MAX || entry.0 == edge_idx {
                *entry = (edge_idx, cost);
                return;
            }
            idx = (idx + 1) & self.mask;
        }
    }

    /// Retrieves cumulative cost if this edge was part of the Pass 0 route.
    #[inline(always)]
    pub fn get_cost(&self, edge_idx: u32) -> Option<Cost> {
        if edge_idx == u32::MAX {
            return None;
        }
        let mut idx = (edge_idx as usize) & self.mask;
        loop {
            let entry = self.table[idx];
            if entry.0 == edge_idx {
                return Some(entry.1);
            }
            if entry.0 == u32::MAX {
                return None;
            }
            idx = (idx + 1) & self.mask;
        }
    }
}

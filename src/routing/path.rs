//! Rust implementation corresponding to:
//! - Java class: `btools.router.StdPath`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/StdPath.java`
//! - Java class: `btools.router.OsmPath`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/OsmPath.java`
//!
//! # Summary
//! Path state tracker with 3-state elevation hysteresis buffers, slope cutoffs,
//! turn angle penalties, and kinematic travel time integration.

use super::cost::{CarKinematicModel, KinematicConfig, ModeFlags};
use super::state::CandidateState;
use crate::dsl::evaluator::WayCostResult;
use crate::types::{Cost, Distance, Joules, Seconds};

/// Geometric and behavioral inputs for evaluating a path section.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionInput {
    pub dist: Distance,
    pub delta_h_micros: i32,
    pub cos_angle: f64,
    pub crossing_penalty: i32,
    pub is_detail: bool,
}

/// Pre-evaluated outgoing way summary at a junction for `KinematicModel` (`KinematicPrePath`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicPrePath {
    pub angle: f64,
    pub priorityclassifier: i32,
    pub classifiermask: i32,
}

/// Elevation hysteresis parameters configured globally or by profile.
#[derive(Debug, Clone)]
pub struct ElevationConfig {
    pub penalty_buffer: i32, // default 5.0m in micrometers = 5,000,000
    pub max_buffer: i32,     // default 10.0m in micrometers = 10,000,000
    pub buffer_reduce: i32,  // default 0.0 * 10000 = 0
}

impl Default for ElevationConfig {
    fn default() -> Self {
        Self {
            penalty_buffer: 5_000_000,
            max_buffer: 10_000_000,
            buffer_reduce: 0,
        }
    }
}

impl ElevationConfig {
    pub fn from_profile(profile: &crate::dsl::parser::Profile) -> Self {
        Self {
            penalty_buffer: (profile.get_global_var("elevationpenaltybuffer", 5.0) * 1_000_000.0)
                as i32,
            max_buffer: (profile.get_global_var("elevationmaxbuffer", 10.0) * 1_000_000.0) as i32,
            buffer_reduce: (profile.get_global_var("elevationbufferreduce", 0.0) * 10_000.0) as i32,
        }
    }
}

/// Dynamic path state carried across road segments along a search path.
#[derive(Debug, Clone, Copy, Default)]
pub struct PathState {
    pub ehbd: i32,                  // downhill elevation hysteresis buffer in micrometers
    pub ehbu: i32,                  // uphill elevation hysteresis buffer in micrometers
    pub elevation_buffer: f32,      // buffer for continuous incline estimation
    pub total_time_s: Seconds,      // cumulative travel time in seconds
    pub total_energy_j: Joules,     // cumulative energy in Joules
    pub ekin: f64,                  // kinetic energy in Joules (for KinematicModel)
    pub floating_angle_left: f32,   // sliding average left bend in degrees (for KinematicModel)
    pub floating_angle_right: f32,  // sliding average right bend in degrees (for KinematicModel)
}

/// Turn crossing cost configuration for junctions with `estimated_crossing_class`.
#[derive(Debug, Clone, Default)]
pub struct CrossingConfig {
    pub consider_crossing: bool,
    pub crossing_prio_h: i32,
    pub crossing_prio_l: i32,
    pub cost_to_left: [i32; 6],
    pub cost_to_right: [i32; 6],
}

impl CrossingConfig {
    pub fn from_profile(profile: &crate::dsl::parser::Profile) -> Self {
        let valid_for_bikes = ModeFlags::from_profile(profile).bike;
        let consider_crossing =
            valid_for_bikes && (profile.get_global_var("consider_crossing", 0.0) != 0.0);
        let crossing_prio_h = profile.get_global_var("crossing_Prio_H", 0.0) as i32;
        let crossing_prio_l = profile.get_global_var("crossing_Prio_L", 0.0) as i32;
        let cost_to_left = [
            profile.get_global_var("cost_ToLeft_from_H_class1", 0.0) as i32,
            profile.get_global_var("cost_ToLeft_from_H_class2", 0.0) as i32,
            profile.get_global_var("cost_ToLeft_from_H_class3", 0.0) as i32,
            profile.get_global_var("cost_ToLeft_from_H_class4", 0.0) as i32,
            profile.get_global_var("cost_ToLeft_from_H_class5", 0.0) as i32,
            profile.get_global_var("cost_ToLeft_from_H_class6", 0.0) as i32,
        ];
        let cost_to_right = [
            profile.get_global_var("cost_ToRight_from_H_class1", 0.0) as i32,
            profile.get_global_var("cost_ToRight_from_H_class2", 0.0) as i32,
            profile.get_global_var("cost_ToRight_from_H_class3", 0.0) as i32,
            profile.get_global_var("cost_ToRight_from_H_class4", 0.0) as i32,
            profile.get_global_var("cost_ToRight_from_H_class5", 0.0) as i32,
            profile.get_global_var("cost_ToRight_from_H_class6", 0.0) as i32,
        ];
        Self {
            consider_crossing,
            crossing_prio_h,
            crossing_prio_l,
            cost_to_left,
            cost_to_right,
        }
    }
}

/// Evaluated metrics for a single road sub-segment.
#[derive(Debug, Clone)]
pub struct SectionCost {
    pub section_cost: f64,
    pub section_cost_int: Cost,
    pub turn_cost: f64,
    pub elevation_cost: f64,
    pub effective_costfactor: f32,
    pub time_seconds: Seconds,
    pub energy_joules: Joules,
}

pub struct StdPathCalculator;

impl StdPathCalculator {
    /// Computes incline using an exponential decay buffer matching BRouter's `calcIncline`.
    pub fn calc_incline(elevation_buffer: &mut f32, delta_h: f64, dist: f64) -> f64 {
        if dist <= 0.0 {
            return 0.0;
        }
        *elevation_buffer = (*elevation_buffer as f64 + delta_h) as f32;

        let min_delta = 3.0f64;
        let mut shift = 0.0f64;
        if (*elevation_buffer as f64) > min_delta {
            shift = -min_delta;
        } else if (*elevation_buffer as f64) < -min_delta {
            shift = min_delta;
        }

        let decay_factor = (-dist / 100.0).exp();
        let new_buffer = ((*elevation_buffer as f64 + shift) * decay_factor - shift) as f32;
        let incline = (*elevation_buffer - new_buffer) as f64 / dist;
        *elevation_buffer = new_buffer;
        incline
    }

    /// Evaluates a road segment with turn cost, elevation hysteresis, and kinematics (`StdPath`).
    pub fn process_section(
        state: &mut PathState,
        way_cost: &WayCostResult,
        elev_cfg: &ElevationConfig,
        kinematic_cfg: &KinematicConfig,
        input: &SectionInput,
    ) -> SectionCost {
        let dist = input.dist;
        let delta_h_micros = input.delta_h_micros;
        let cos_angle = input.cos_angle;
        let crossing_penalty = input.crossing_penalty;
        let is_detail = input.is_detail;

        let turncost_base = way_cost.turncost;
        let uphillcutoff = way_cost.uphillcutoff_f;
        let downhillcutoff = way_cost.downhillcutoff_f;
        let uphillmaxslope = way_cost.uphillmaxslope_f;
        let downhillmaxslope = way_cost.downhillmaxslope_f;

        let cfup = way_cost.uphillcostfactor;
        let cfdown = way_cost.downhillcostfactor;
        let cf = way_cost.costfactor;

        let downhillcostdiv = way_cost.downhillcostdiv;
        let downhillmaxslopecostdiv = way_cost.downhillmaxslopecostdiv;
        let uphillcostdiv = way_cost.uphillcostdiv;
        let uphillmaxslopecostdiv = way_cost.uphillmaxslopecostdiv;

        let dist_int = dist as i32;
        let dist_f = dist_int as f32;

        // 1. Turn penalty with crossing cost
        let turn_cost = ((1.0 - cos_angle) * turncost_base as f64 + 0.2) as i32 + crossing_penalty;
        let mut section_cost = turn_cost as f64;

        // 2. Elevation penalty with hysteresis buffer (matching Java `int += float` semantics)
        state.ehbd = (state.ehbd as f32
            + (delta_h_micros.saturating_neg() as f32 - dist_f * downhillcutoff))
            as i32;
        state.ehbu =
            (state.ehbu as f32 + (delta_h_micros as f32 - dist_f * uphillcutoff)) as i32;

        let mut downweight = 0.0f32;
        let mut total_elev_cost = 0.0f64;

        if state.ehbd > elev_cfg.penalty_buffer {
            downweight = 1.0;
            let excess = state.ehbd - elev_cfg.penalty_buffer;
            let mut reduce = dist_int * elev_cfg.buffer_reduce;
            if reduce > excess {
                downweight = excess as f32 / reduce as f32;
                reduce = excess;
            }
            let excess_max = state.ehbd - elev_cfg.max_buffer;
            if reduce < excess_max {
                reduce = excess_max;
            }
            state.ehbd -= reduce;

            let mut elev_cost_down = 0.0f32;
            if downhillcostdiv > 0 {
                elev_cost_down +=
                    (reduce as f32).min(dist_f * downhillmaxslope) / downhillcostdiv as f32;
            }
            if downhillmaxslopecostdiv > 0 {
                elev_cost_down += (0.0f32).max(reduce as f32 - dist_f * downhillmaxslope)
                    / downhillmaxslopecostdiv as f32;
            }
            if elev_cost_down > 0.0 {
                section_cost += elev_cost_down as f64;
                total_elev_cost += elev_cost_down as f64;
            }
        } else if state.ehbd < 0 {
            state.ehbd = 0;
        }

        let mut upweight = 0.0f32;
        if state.ehbu > elev_cfg.penalty_buffer {
            upweight = 1.0;
            let excess = state.ehbu - elev_cfg.penalty_buffer;
            let mut reduce = dist_int * elev_cfg.buffer_reduce;
            if reduce > excess {
                upweight = excess as f32 / reduce as f32;
                reduce = excess;
            }
            let excess_max = state.ehbu - elev_cfg.max_buffer;
            if reduce < excess_max {
                reduce = excess_max;
            }
            state.ehbu -= reduce;

            let mut elev_cost_up = 0.0f32;
            if uphillcostdiv > 0 {
                elev_cost_up +=
                    (reduce as f32).min(dist_f * uphillmaxslope) / uphillcostdiv as f32;
            }
            if uphillmaxslopecostdiv > 0 {
                elev_cost_up += (0.0f32).max(reduce as f32 - dist_f * uphillmaxslope)
                    / uphillmaxslopecostdiv as f32;
            }
            if elev_cost_up > 0.0 {
                section_cost += elev_cost_up as f64;
                total_elev_cost += elev_cost_up as f64;
            }
        } else if state.ehbu < 0 {
            state.ehbu = 0;
        }

        // 3. Slope-dependent effective costfactor
        let costfactor = cfup * upweight + cf * (1.0 - upweight - downweight) + cfdown * downweight;
        section_cost += (dist_f * costfactor + 0.5f32) as f64;

        let section_cost_int = (section_cost.max(0.0)) as u32;

        // 4. Kinematics (travel time & energy) - only computed in detail mode matching BRouter
        let (dt, energy) = if is_detail {
            let seg_d = dist_int as f64;
            let delta_h = delta_h_micros as f64 * 1e-6;
            let incline = Self::calc_incline(&mut state.elevation_buffer, delta_h, seg_d);
            let (speed, energy) = kinematic_cfg.compute_speed_and_energy(
                seg_d,
                incline,
                way_cost.maxspeed as f64 / 3.6,
            );
            let dt = (seg_d / speed) as f32;
            state.total_time_s += dt;
            let energy_f = if energy > 0.0 {
                let prev_energy = state.total_energy_j;
                state.total_energy_j = (state.total_energy_j as f64 + energy) as f32;
                state.total_energy_j - prev_energy
            } else {
                0.0f32
            };
            (dt, energy_f)
        } else {
            (0.0f32, 0.0f32)
        };

        SectionCost {
            section_cost,
            section_cost_int,
            turn_cost: turn_cost as f64,
            elevation_cost: total_elev_cost,
            effective_costfactor: costfactor,
            time_seconds: dt,
            energy_joules: energy,
        }
    }

    /// Evaluates a road segment using `KinematicPath` or `KinematicNoCostPath` (`CarKinematicModel`).
    /// Returns `None` if the segment is impassable (`effective_speed_limit <= 0.0` or `section_cost < 0.0`).
    #[allow(clippy::too_many_arguments)]
    pub fn process_car_section(
        state: &mut PathState,
        way_cost: &WayCostResult,
        km: &CarKinematicModel,
        dist: Distance,
        delta_h: f64,
        elevation: f64,
        angle: f64,
        cos_angle: f64,
        is_startpoint: bool,
        nsection: usize,
        last_prio: i32,
        pre_paths: &[KinematicPrePath],
    ) -> Option<SectionCost> {
        let mut cost = 0.0f64;
        let mut extra_time = 0.0f64;
        let priorityclassifier = way_cost.priorityclassifier;

        if is_startpoint {
            extra_time = 0.5 * (1.0 - cos_angle) * 40.0;
        } else {
            let mut turnspeed = 999.0f64;
            if km.turn_angle_decay_time != 0.0 {
                if angle < 0.0 {
                    state.floating_angle_left -= angle as f32;
                } else {
                    state.floating_angle_right += angle as f32;
                }
                let aa = state.floating_angle_left.max(state.floating_angle_right);
                let curve_speed = if aa > 10.0 { 200.0 / aa as f64 } else { 20.0 };
                let distance_time = dist / curve_speed;
                let decay_factor = (-distance_time / km.turn_angle_decay_time).exp();
                state.floating_angle_left = (state.floating_angle_left as f64 * decay_factor) as f32;
                state.floating_angle_right =
                    (state.floating_angle_right as f64 * decay_factor) as f32;
                if curve_speed < 20.0 {
                    turnspeed = curve_speed;
                }
            }

            if nsection == 0 {
                let mut junctionspeed = 999.0f64;
                let classifiermask = way_cost.classifiermask;
                let mut has_left_way = false;
                let mut has_right_way = false;
                let mut has_residential = false;

                for pp in pre_paths {
                    if ((pp.classifiermask ^ classifiermask) & 8) != 0 {
                        continue;
                    }
                    if (pp.classifiermask & 32) != 0 {
                        has_residential = true;
                    }
                    if pp.priorityclassifier > priorityclassifier
                        || (pp.priorityclassifier == priorityclassifier && priorityclassifier < 20)
                    {
                        let diff = pp.angle - angle;
                        if diff < -40.0 && diff > -140.0 {
                            has_left_way = true;
                        }
                        if diff > 40.0 && diff < 140.0 {
                            has_right_way = true;
                        }
                    }
                }

                let residential_speed = 13.0;
                if has_left_way && junctionspeed > km.left_way_speed {
                    junctionspeed = km.left_way_speed;
                }
                if has_right_way && junctionspeed > km.right_way_speed {
                    junctionspeed = km.right_way_speed;
                }
                if has_residential && junctionspeed > residential_speed {
                    junctionspeed = residential_speed;
                }

                if (last_prio < 20) ^ (priorityclassifier < 20) {
                    extra_time += 10.0;
                    junctionspeed = 0.0;
                }
                if last_prio != priorityclassifier && (classifiermask & 8) != 0 {
                    extra_time += 2.0;
                }
                if turnspeed > junctionspeed {
                    turnspeed = junctionspeed;
                }
            }
            km.cut_ekin(&mut state.ekin, turnspeed);
        }

        let tcorr = (20.0 - km.outside_temp) * 0.0035;
        let ecorr = 0.0001375 * (elevation - 100.0);
        let f_air = km.f_air * (1.0 + tcorr - ecorr);

        let (distance_cost, dt_evolve, de_evolve) = km.evolve_distance(
            &mut state.ekin,
            &mut state.total_time_s,
            &mut state.total_energy_j,
            way_cost,
            dist,
            delta_h,
            f_air,
        );
        if distance_cost < 0.0 {
            return None;
        }

        if !km.no_cost_factor {
            cost += dist * (way_cost.costfactor as f64) + 0.5;
        }

        let turn_cost = extra_time * km.pw / km.cost0;
        cost += turn_cost;
        state.total_time_s += extra_time as f32;
        cost += distance_cost;

        if cost < 0.0 {
            return None;
        }

        Some(SectionCost {
            section_cost: cost,
            section_cost_int: cost as u32,
            turn_cost,
            elevation_cost: 0.0,
            effective_costfactor: if dist > 0.0 {
                (distance_cost / dist) as f32
            } else {
                way_cost.costfactor
            },
            time_seconds: dt_evolve + extra_time as f32,
            energy_joules: de_evolve,
        })
    }

    /// Computes uphill and downhill cost divisors from way cost parameters.
    #[inline]
    pub fn cost_divisors(way_cost: &WayCostResult) -> (i32, i32) {
        (way_cost.downhillcostdiv, way_cost.uphillcostdiv)
    }

    /// Calculates the elevation buffer correction term matching BRouter's `StdPath.elevationCorrection`.
    #[inline]
    pub fn elevation_correction(
        ehbd: i32,
        ehbu: i32,
        downhillcostdiv: i32,
        uphillcostdiv: i32,
    ) -> u32 {
        let down = if downhillcostdiv > 0 {
            (ehbd / downhillcostdiv).max(0) as u32
        } else {
            0
        };
        let up = if uphillcostdiv > 0 {
            (ehbu / uphillcostdiv).max(0) as u32
        } else {
            0
        };
        down + up
    }

    /// Evaluates if a candidate path is definitely worse than an existing path.
    ///
    /// Matches `StdPath.definitlyWorseThan` in Java BRouter.
    /// Uses Pareto dominance over cumulative cost and elevation hysteresis buffers `ehbd` and `ehbu`.
    /// When `cand_cost > exist_cost + delta_elevation_cost`, the candidate path cannot overtake
    /// the existing path even under optimal downhill or uphill buffer exploitation.
    #[inline]
    pub fn definitely_worse_than(
        cand: CandidateState,
        exist: CandidateState,
        downhillcostdiv: i32,
        uphillcostdiv: i32,
    ) -> bool {
        cand.is_definitely_worse_than(&exist, downhillcostdiv, uphillcostdiv)
    }

    /// Compatibility forwarder matching Java BRouter spelling `StdPath.definitlyWorseThan`.
    #[inline]
    pub fn definitly_worse_than(
        cand: CandidateState,
        exist: CandidateState,
        downhillcostdiv: i32,
        uphillcostdiv: i32,
    ) -> bool {
        cand.is_definitely_worse_than(&exist, downhillcostdiv, uphillcostdiv)
    }
}

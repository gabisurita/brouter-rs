//! Rust implementation corresponding to:
//! - Java class: `btools.expressions.BExpression`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpression.java`
//! - Java class: `btools.expressions.BExpressionContextWay`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpressionContextWay.java`
//! - Java class: `btools.expressions.BExpressionContextNode`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpressionContextNode.java`
//!
//! # Summary
//! High-performance profile evaluator computing costfactors, turncosts, and penalties
//! over dense integer lookup data arrays without dynamic heap allocations.

use super::ast::BExpr;
use super::lookups::ContextLookups;
use super::parser::Profile;

/// Computed cost metrics for an OSM way segment.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct WayCostResult {
    pub costfactor: f32,
    pub turncost: f32,
    pub uphillcostfactor: f32,
    pub downhillcostfactor: f32,
    pub initialcost: f32,
    pub nodeaccessgranted: bool,
    pub initialclassifier: u8,
    pub trafficsourcedensity: f32,
    pub istrafficbackbone: bool,
    pub priorityclassifier: i32,
    pub classifiermask: i32,
    pub maxspeed: f32,
    pub uphillcost: f32,
    pub downhillcost: f32,
    pub uphillcutoff: f32,
    pub downhillcutoff: f32,
    pub uphillmaxslope: f32,
    pub downhillmaxslope: f32,
    pub uphillmaxslopecost: f32,
    pub downhillmaxslopecost: f32,
    pub maxspeed_explicit: f32,
    pub minspeed: f32,
    // Precomputed derived fields
    pub uphillcutoff_f: f32,
    pub downhillcutoff_f: f32,
    pub uphillmaxslope_f: f32,
    pub downhillmaxslope_f: f32,
    pub uphillcutoff_i: i32,
    pub downhillcutoff_i: i32,
    pub uphillmaxslope_i: i32,
    pub downhillmaxslope_i: i32,
    pub downhillcostdiv: i32,
    pub downhillmaxslopecostdiv: i32,
    pub uphillcostdiv: i32,
    pub uphillmaxslopecostdiv: i32,
    pub foreign_var_0: f32,
}

impl WayCostResult {
    pub fn init_derived(&mut self) {
        self.uphillcutoff_f = self.uphillcutoff * 10000.0;
        self.downhillcutoff_f = self.downhillcutoff * 10000.0;
        self.uphillmaxslope_f = self.uphillmaxslope * 10000.0;
        self.downhillmaxslope_f = self.downhillmaxslope * 10000.0;

        self.uphillcutoff_i = self.uphillcutoff_f as i32;
        self.downhillcutoff_i = self.downhillcutoff_f as i32;
        self.uphillmaxslope_i = self.uphillmaxslope_f as i32;
        self.downhillmaxslope_i = self.downhillmaxslope_f as i32;

        let down_cost = self.downhillcost as i32;
        self.downhillcostdiv = if down_cost > 0 {
            1_000_000 / down_cost
        } else {
            0
        };

        let down_max_cost = self.downhillmaxslopecost as i32;
        self.downhillmaxslopecostdiv = if down_max_cost > 0 {
            1_000_000 / down_max_cost
        } else {
            self.downhillcostdiv
        };

        let up_cost = self.uphillcost as i32;
        self.uphillcostdiv = if up_cost > 0 { 1_000_000 / up_cost } else { 0 };

        let up_max_cost = self.uphillmaxslopecost as i32;
        self.uphillmaxslopecostdiv = if up_max_cost > 0 {
            1_000_000 / up_max_cost
        } else {
            self.uphillcostdiv
        };
    }
}

/// Computed cost penalty for a node.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NodeCostResult {
    pub initialcost: f32,
    pub maxspeed: f32,
    pub crossing_class: u8,
}

pub struct ProfileEvaluator<'a> {
    pub profile: &'a Profile,
    pub way_lookups: &'a ContextLookups,
    pub node_lookups: &'a ContextLookups,
    pub way_vars: Vec<f32>,
    pub node_vars: Vec<f32>,
    pub node_foreign_var_0: Option<usize>,
}

impl<'a> ProfileEvaluator<'a> {
    pub fn new(
        profile: &'a Profile,
        way_lookups: &'a ContextLookups,
        node_lookups: &'a ContextLookups,
    ) -> Self {
        let mut way_vars = vec![0.0f32; profile.way_block.num_vars];
        let copy_way_len = profile.way_block.min_write_idx.min(profile.global_values.len());
        way_vars[..copy_way_len].copy_from_slice(&profile.global_values[..copy_way_len]);

        let mut node_vars = vec![0.0f32; profile.node_block.num_vars];
        let copy_node_len = profile.node_block.min_write_idx.min(profile.global_values.len());
        node_vars[..copy_node_len].copy_from_slice(&profile.global_values[..copy_node_len]);

        let node_foreign_var_0 = profile.node_block.foreign_var_indices().into_iter().next();

        Self {
            profile,
            way_lookups,
            node_lookups,
            way_vars,
            node_vars,
            node_foreign_var_0,
        }
    }

    /// Evaluates a way segment using its dense lookup array.
    pub fn evaluate_way(&mut self, lookup_data: &[i32]) -> WayCostResult {
        // Reset local variables above min_write_idx
        for i in self.profile.way_block.min_write_idx..self.way_vars.len() {
            self.way_vars[i] = 0.0;
        }

        for stmt in &self.profile.way_block.statements {
            Self::eval_expr(
                stmt,
                lookup_data,
                &mut self.way_vars,
                self.way_lookups,
                None,
            );
        }

        // Extract evaluated built-in variables.
        // Indices 0..19 map 1:1 to `WAY_BUILTIN_VARIABLES` in `parser.rs`,
        // matching the variable layout in Java class `btools.expressions.BExpressionContextWay`:
        //  0: costfactor            1: turncost               2: uphillcostfactor
        //  3: downhillcostfactor    4: initialcost            5: nodeaccessgranted
        //  6: initialclassifier     7: trafficsourcedensity   8: istrafficbackbone
        //  9: priorityclassifier   10: classifiermask        11: maxspeed
        // 12: uphillcost           13: downhillcost          14: uphillcutoff
        // 15: downhillcutoff       16: uphillmaxslope        17: downhillmaxslope
        // 18: uphillmaxslopecost   19: downhillmaxslopecost
        let get_var = |idx: usize| -> f32 {
            self.profile
                .way_block
                .builtin_indices
                .get(idx)
                .and_then(|opt| *opt)
                .map(|v_idx| self.way_vars[v_idx])
                .unwrap_or(0.0)
        };

        let cf = get_var(0);
        let turncost = get_var(1);
        let cfup = get_var(2);
        let cfdown = get_var(3);
        let initialcost = get_var(4);
        let nodeaccessgranted = get_var(5) != 0.0;
        let initialclassifier = get_var(6) as u8;
        let trafficsourcedensity = get_var(7);
        let istrafficbackbone = get_var(8) != 0.0;
        let priorityclassifier = get_var(9) as i32;
        let classifiermask = get_var(10) as i32;
        let maxspeed = get_var(11);
        let uphillcost = get_var(12);
        let downhillcost = get_var(13);
        let uphillcutoff = get_var(14);
        let downhillcutoff = get_var(15);
        let uphillmaxslope = get_var(16);
        let downhillmaxslope = get_var(17);
        let uphillmaxslopecost = get_var(18);
        let downhillmaxslopecost = get_var(19);
        let maxspeed_explicit = get_var(20);
        let minspeed = get_var(21);
        let foreign_var_0 = self
            .node_foreign_var_0
            .and_then(|i| self.way_vars.get(i).copied())
            .unwrap_or(0.0);

        let mut res = WayCostResult {
            costfactor: cf,
            turncost,
            uphillcostfactor: if cfup == 0.0 { cf } else { cfup },
            downhillcostfactor: if cfdown == 0.0 { cf } else { cfdown },
            initialcost,
            nodeaccessgranted,
            initialclassifier,
            trafficsourcedensity,
            istrafficbackbone,
            priorityclassifier,
            classifiermask,
            maxspeed,
            uphillcost,
            downhillcost,
            uphillcutoff,
            downhillcutoff,
            uphillmaxslope,
            downhillmaxslope,
            uphillmaxslopecost,
            downhillmaxslopecost,
            maxspeed_explicit,
            minspeed,
            foreign_var_0,
            ..Default::default()
        };
        res.init_derived();
        res
    }

    /// Evaluates a node using its dense lookup array.
    pub fn evaluate_node(&mut self, lookup_data: &[i32]) -> NodeCostResult {
        for i in self.profile.node_block.min_write_idx..self.node_vars.len() {
            self.node_vars[i] = 0.0;
        }

        for stmt in &self.profile.node_block.statements {
            Self::eval_expr(
                stmt,
                lookup_data,
                &mut self.node_vars,
                self.node_lookups,
                Some(&self.way_vars),
            );
        }

        let get_node_var = |idx: usize| -> f32 {
            self.profile
                .node_block
                .builtin_indices
                .get(idx)
                .and_then(|opt| *opt)
                .map(|v_idx| self.node_vars[v_idx])
                .unwrap_or(0.0)
        };

        NodeCostResult {
            initialcost: get_node_var(0),
            maxspeed: get_node_var(1),
            crossing_class: 0,
        }
    }

    fn eval_expr(
        expr: &BExpr,
        lookup_data: &[i32],
        vars: &mut [f32],
        lookups: &ContextLookups,
        foreign_vars: Option<&[f32]>,
    ) -> f32 {
        match expr {
            BExpr::Number(n) => *n,
            BExpr::Variable(idx) => vars[*idx],
            BExpr::ForeignVariable { var_idx, .. } => {
                if let Some(fv) = foreign_vars {
                    if *var_idx < fv.len() {
                        fv[*var_idx]
                    } else {
                        0.0
                    }
                } else {
                    0.0
                }
            }
            BExpr::VariableGet(name_idx) => {
                if *name_idx < lookup_data.len() {
                    lookups.get_lookup_value(*name_idx, lookup_data[*name_idx])
                } else {
                    0.0
                }
            }
            BExpr::Lookup {
                name_idx,
                allowed_values,
            } => {
                if *name_idx < lookup_data.len() {
                    let val = lookup_data[*name_idx];
                    if allowed_values.contains(&val) {
                        1.0
                    } else {
                        0.0
                    }
                } else {
                    0.0
                }
            }
            BExpr::Not(inner) => {
                let v = Self::eval_expr(inner, lookup_data, vars, lookups, foreign_vars);
                if v == 0.0 {
                    1.0
                } else {
                    0.0
                }
            }
            BExpr::Binary { op, left, right } => {
                let v1 = Self::eval_expr(left, lookup_data, vars, lookups, foreign_vars);
                let v2 = Self::eval_expr(right, lookup_data, vars, lookups, foreign_vars);
                op.eval(v1, v2)
            }
            BExpr::Switch {
                cond,
                then_expr,
                else_expr,
            } => {
                let c = Self::eval_expr(cond, lookup_data, vars, lookups, foreign_vars);
                if c != 0.0 {
                    Self::eval_expr(then_expr, lookup_data, vars, lookups, foreign_vars)
                } else {
                    Self::eval_expr(else_expr, lookup_data, vars, lookups, foreign_vars)
                }
            }
            BExpr::Assign { var_idx, expr } => {
                let v = Self::eval_expr(expr, lookup_data, vars, lookups, foreign_vars);
                vars[*var_idx] = v;
                v
            }
        }
    }
}

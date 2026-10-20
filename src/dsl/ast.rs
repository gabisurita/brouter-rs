//! Rust implementation corresponding to:
//! - Java class: `btools.expressions.BExpression`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpression.java`
//!
//! # Summary
//! Abstract syntax tree (AST) nodes and constant folding optimizations for BRouter profiles.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinaryOp {
    Or,
    And,
    Xor,
    Add,
    Sub,
    Multiply,
    Divide,
    Max,
    Min,
    Equal,
    Greater,
    Lesser,
}

impl BinaryOp {
    pub fn eval(&self, v1: f32, v2: f32) -> f32 {
        match self {
            BinaryOp::Or => {
                if v1 != 0.0 || v2 != 0.0 {
                    1.0
                } else {
                    0.0
                }
            }
            BinaryOp::And => {
                if v1 != 0.0 && v2 != 0.0 {
                    1.0
                } else {
                    0.0
                }
            }
            BinaryOp::Xor => {
                if (v1 != 0.0) ^ (v2 != 0.0) {
                    1.0
                } else {
                    0.0
                }
            }
            BinaryOp::Add => v1 + v2,
            BinaryOp::Sub => v1 - v2,
            BinaryOp::Multiply => v1 * v2,
            BinaryOp::Divide => {
                if v2 == 0.0 {
                    0.0
                } else {
                    v1 / v2
                }
            }
            BinaryOp::Max => v1.max(v2),
            BinaryOp::Min => v1.min(v2),
            BinaryOp::Equal => {
                if v1 == v2 {
                    1.0
                } else {
                    0.0
                }
            }
            BinaryOp::Greater => {
                if v1 > v2 {
                    1.0
                } else {
                    0.0
                }
            }
            BinaryOp::Lesser => {
                if v1 < v2 {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }
}

/// Profile expression AST node.
///
/// Corresponds to Java class `btools.expressions.BExpression`.
#[derive(Debug, Clone, PartialEq)]
pub enum BExpr {
    /// Literal float constant.
    Number(f32),
    /// Local or inherited variable reference by index in the context array.
    Variable(usize),
    /// Foreign context variable reference (for example, `way:highway` accessed inside `node` context).
    ForeignVariable {
        context: String,
        var_name: String,
        var_idx: usize,
    },
    /// Numerical tag extraction via `v:<tag>` (e.g. `v:maxspeed`).
    VariableGet(usize),
    /// Tag match test against a set of allowed enumerated value indices.
    Lookup {
        name_idx: usize,
        allowed_values: Vec<i32>,
    },
    /// Logical negation (`not <expr>`).
    Not(Box<BExpr>),
    /// Binary arithmetic, logical, or comparison operator.
    Binary {
        op: BinaryOp,
        left: Box<BExpr>,
        right: Box<BExpr>,
    },
    /// Ternary conditional (`switch <cond> <then> <else>` or `if <cond> then <then> else <else>`).
    Switch {
        cond: Box<BExpr>,
        then_expr: Box<BExpr>,
        else_expr: Box<BExpr>,
    },
    /// Assignment of evaluated expression result to local variable index.
    Assign {
        var_idx: usize,
        expr: Box<BExpr>,
    },
}

impl BExpr {
    /// Attempts constant folding and AST simplification matching BExpression.tryCollapse / tryEvaluateConstant.
    pub fn optimize(self) -> Self {
        match self {
            BExpr::Not(e) => {
                let opt = e.optimize();
                if let BExpr::Number(n) = opt {
                    BExpr::Number(if n == 0.0 { 1.0 } else { 0.0 })
                } else {
                    BExpr::Not(Box::new(opt))
                }
            }
            BExpr::Binary { op, left, right } => {
                let opt_l = left.optimize();
                let opt_r = right.optimize();

                // If both are numbers, fold
                if let (BExpr::Number(n1), BExpr::Number(n2)) = (&opt_l, &opt_r) {
                    return BExpr::Number(op.eval(*n1, *n2));
                }

                // Short-circuit / identity simplifications
                match op {
                    BinaryOp::Or => {
                        if let BExpr::Number(n) = opt_l {
                            if n != 0.0 {
                                return BExpr::Number(1.0);
                            }
                            return opt_r;
                        }
                        if let BExpr::Number(n) = opt_r {
                            if n != 0.0 {
                                return BExpr::Number(1.0);
                            }
                            return opt_l;
                        }
                    }
                    BinaryOp::And => {
                        if let BExpr::Number(n) = opt_l {
                            if n == 0.0 {
                                return BExpr::Number(0.0);
                            }
                            return opt_r;
                        }
                        if let BExpr::Number(n) = opt_r {
                            if n == 0.0 {
                                return BExpr::Number(0.0);
                            }
                            return opt_l;
                        }
                    }
                    BinaryOp::Add => {
                        if let BExpr::Number(n) = opt_l {
                            if n == 0.0 {
                                return opt_r;
                            }
                        }
                        if let BExpr::Number(n) = opt_r {
                            if n == 0.0 {
                                return opt_l;
                            }
                        }
                    }
                    _ => {}
                }

                BExpr::Binary {
                    op,
                    left: Box::new(opt_l),
                    right: Box::new(opt_r),
                }
            }
            BExpr::Switch {
                cond,
                then_expr,
                else_expr,
            } => {
                let opt_c = cond.optimize();
                let opt_t = then_expr.optimize();
                let opt_e = else_expr.optimize();

                if let BExpr::Number(c) = opt_c {
                    if c != 0.0 {
                        opt_t
                    } else {
                        opt_e
                    }
                } else {
                    BExpr::Switch {
                        cond: Box::new(opt_c),
                        then_expr: Box::new(opt_t),
                        else_expr: Box::new(opt_e),
                    }
                }
            }
            BExpr::Assign { var_idx, expr } => BExpr::Assign {
                var_idx,
                expr: Box::new(expr.optimize()),
            },
            other => other,
        }
    }

    /// Returns true if this expression subtree references any `ForeignVariable`.
    pub fn has_foreign_variable(&self) -> bool {
        match self {
            BExpr::ForeignVariable { .. } => true,
            BExpr::Not(e) => e.has_foreign_variable(),
            BExpr::Binary { left, right, .. } => {
                left.has_foreign_variable() || right.has_foreign_variable()
            }
            BExpr::Switch {
                cond,
                then_expr,
                else_expr,
            } => {
                cond.has_foreign_variable()
                    || then_expr.has_foreign_variable()
                    || else_expr.has_foreign_variable()
            }
            BExpr::Assign { expr, .. } => expr.has_foreign_variable(),
            _ => false,
        }
    }

    /// Appends unique `var_idx` values of any `ForeignVariable` referenced in this expression subtree.
    pub fn collect_foreign_vars(&self, out: &mut Vec<usize>) {
        match self {
            BExpr::ForeignVariable { var_idx, .. } => {
                if !out.contains(var_idx) {
                    out.push(*var_idx);
                }
            }
            BExpr::Not(e) => e.collect_foreign_vars(out),
            BExpr::Binary { left, right, .. } => {
                left.collect_foreign_vars(out);
                right.collect_foreign_vars(out);
            }
            BExpr::Switch {
                cond,
                then_expr,
                else_expr,
            } => {
                cond.collect_foreign_vars(out);
                then_expr.collect_foreign_vars(out);
                else_expr.collect_foreign_vars(out);
            }
            BExpr::Assign { expr, .. } => expr.collect_foreign_vars(out),
            _ => {}
        }
    }
}

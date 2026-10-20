//! Rust implementation corresponding to:
//! - Java class: `btools.expressions.BExpression`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpression.java`
//! - Java class: `btools.expressions.BExpressionContext`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpressionContext.java`
//!
//! # Summary
//! Tokenizer and recursive descent parser for BRouter profile scripts (`.brf`).
//! Compiles global, way, and node expression blocks into optimized ASTs.

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use super::ast::{BExpr, BinaryOp};
use super::lookups::{ContextLookups, LookupData};

pub const WAY_BUILTIN_VARIABLES: [&str; 22] = [
    "costfactor",
    "turncost",
    "uphillcostfactor",
    "downhillcostfactor",
    "initialcost",
    "nodeaccessgranted",
    "initialclassifier",
    "trafficsourcedensity",
    "istrafficbackbone",
    "priorityclassifier",
    "classifiermask",
    "maxspeed",
    "uphillcost",
    "downhillcost",
    "uphillcutoff",
    "downhillcutoff",
    "uphillmaxslope",
    "downhillmaxslope",
    "uphillmaxslopecost",
    "downhillmaxslopecost",
    "maxspeed_explicit",
    "minspeed",
];

pub const NODE_BUILTIN_VARIABLES: [&str; 2] = ["initialcost", "maxspeed"];

/// Tokenizer splitting profile text while ignoring comments.
pub struct Tokenizer<'a> {
    chars: std::str::Chars<'a>,
    peeked: Option<char>,
}

impl<'a> Tokenizer<'a> {
    pub fn new(source: &'a str) -> Self {
        Self {
            chars: source.chars(),
            peeked: None,
        }
    }

    fn next_char(&mut self) -> Option<char> {
        self.peeked.take().or_else(|| self.chars.next())
    }

    fn peek_char(&mut self) -> Option<char> {
        if self.peeked.is_none() {
            self.peeked = self.chars.next();
        }
        self.peeked
    }

    pub fn next_token(&mut self) -> Option<String> {
        loop {
            // Skip whitespace
            while let Some(c) = self.peek_char() {
                if c.is_whitespace() {
                    self.next_char();
                } else {
                    break;
                }
            }

            let c = self.peek_char()?;

            // Comments start with # and run to end of line
            if c == '#' {
                while let Some(c) = self.next_char() {
                    if c == '\n' || c == '\r' {
                        break;
                    }
                }
                continue;
            }

            // Grouping parentheses
            if c == '(' || c == ')' {
                self.next_char();
                return Some(c.to_string());
            }

            // Word token
            let mut word = String::with_capacity(16);
            while let Some(ch) = self.peek_char() {
                if ch.is_whitespace() || ch == '#' || ch == '(' || ch == ')' {
                    break;
                }
                word.push(self.next_char().unwrap());
            }

            if !word.is_empty() {
                return Some(word);
            }
        }
    }
}

/// Compiled execution scope for a single context (`global`, `way`, or `node`).
#[derive(Debug, Clone)]
pub struct ContextBlock {
    pub statements: Vec<BExpr>,
    pub var_names: Vec<String>,
    pub var_map: HashMap<String, usize>,
    pub min_write_idx: usize,
    pub num_vars: usize,
    pub builtin_indices: Vec<Option<usize>>,
}

impl ContextBlock {
    /// Returns true if any statement in this context block references a `ForeignVariable`.
    pub fn has_foreign_variables(&self) -> bool {
        self.statements.iter().any(|s| s.has_foreign_variable())
    }

    /// Returns unique foreign variable indices referenced in this context block.
    pub fn foreign_var_indices(&self) -> Vec<usize> {
        let mut out = Vec::new();
        for s in &self.statements {
            s.collect_foreign_vars(&mut out);
        }
        out
    }
}

/// A fully compiled BRouter routing profile.
#[derive(Debug, Clone)]
pub struct Profile {
    pub name: String,
    pub content_hash: u64,
    pub model_class: Option<String>,
    pub global_block: ContextBlock,
    pub global_values: Vec<f32>,
    pub way_block: ContextBlock,
    pub node_block: ContextBlock,
    /// Disallowed starting way `(lookup_name_idx, lookup_value_idx)` pairs parsed from
    /// `# ... noStartWay ...` comments when `check_start_way` is defined in the profile.
    pub no_start_ways: Vec<(usize, i32)>,
}

impl Profile {
    pub fn get_global_var(&self, name: &str, default: f32) -> f32 {
        if let Some(&idx) = self.global_block.var_map.get(name) {
            if idx < self.global_values.len() {
                return self.global_values[idx];
            }
        }
        default
    }

    /// Checks whether a way description bitmap is permitted as a starting way
    /// according to `# ... noStartWay ...` rules (`BExpressionContext.checkStartWay`).
    pub fn check_start_way(&self, description: &[u8], way_lookups: &ContextLookups) -> bool {
        if description.is_empty() || self.no_start_ways.is_empty() {
            return true;
        }
        let mut ld = vec![0i32; way_lookups.names.len()];
        way_lookups.decode_into(&mut ld, false, description);
        self.check_start_way_decoded(&ld)
    }

    /// Checks whether a decoded way lookup buffer is permitted as a starting way.
    #[inline]
    pub fn check_start_way_decoded(&self, way_lookup_buf: &[i32]) -> bool {
        for &(key, value) in &self.no_start_ways {
            if key < way_lookup_buf.len() && way_lookup_buf[key] == value {
                return false;
            }
        }
        true
    }

    pub fn load_from_file<P: AsRef<Path>>(path: P, lookups: &LookupData) -> Result<Self, String> {
        Self::load_from_file_with_params(path, lookups, None)
    }

    pub fn load_from_file_with_params<P: AsRef<Path>>(
        path: P,
        lookups: &LookupData,
        params: Option<&HashMap<String, String>>,
    ) -> Result<Self, String> {
        let name = path
            .as_ref()
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_default();
        let content = fs::read_to_string(&path)
            .map_err(|e| format!("Failed to read profile file {:?}: {}", path.as_ref(), e))?;

        Self::parse_with_params(&name, &content, lookups, params)
    }

    pub fn parse(name: &str, content: &str, lookups: &LookupData) -> Result<Self, String> {
        Self::parse_with_params(name, content, lookups, None)
    }

    pub fn parse_with_params(
        name: &str,
        content: &str,
        lookups: &LookupData,
        params: Option<&HashMap<String, String>>,
    ) -> Result<Self, String> {
        // Collect tokens by section
        let mut tokenizer = Tokenizer::new(content);
        let mut model_class = None;

        let mut current_section = "global";
        let mut global_tokens = Vec::new();
        let mut way_tokens = Vec::new();
        let mut node_tokens = Vec::new();

        while let Some(token) = tokenizer.next_token() {
            if let Some(rest) = token.strip_prefix("---context:") {
                let ctx = rest.trim();
                current_section = match ctx {
                    "global" => "global",
                    "way" => "way",
                    "node" => "node",
                    _ => current_section,
                };
                continue;
            }
            if let Some(rest) = token.strip_prefix("---model:") {
                model_class = Some(rest.trim().to_string());
                continue;
            }

            match current_section {
                "global" => global_tokens.push(token),
                "way" => way_tokens.push(token),
                "node" => node_tokens.push(token),
                _ => {}
            }
        }

        // 1. Compile and evaluate global section
        let (global_block, global_values) =
            compile_global_block(&global_tokens, &lookups.way, params)?;

        // Parse noStartWays from comments if check_start_way is defined in global context
        let no_start_ways = if global_block.var_map.contains_key("check_start_way") {
            parse_no_start_ways(content, &lookups.way)
        } else {
            Vec::new()
        };

        // 2. Compile way section with global variables inherited as read-only
        let way_block = compile_context_block(
            "way",
            &way_tokens,
            &lookups.way,
            &WAY_BUILTIN_VARIABLES,
            &global_block,
            &global_values,
        )?;

        // 3. Compile node section with global variables inherited as read-only
        let mut node_block = compile_context_block(
            "node",
            &node_tokens,
            &lookups.node,
            &NODE_BUILTIN_VARIABLES,
            &global_block,
            &global_values,
        )?;

        for stmt in &mut node_block.statements {
            resolve_foreign_vars(stmt, &way_block.var_map);
        }

        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        std::hash::Hash::hash(content, &mut hasher);
        if let Some(p) = params {
            let mut kv: Vec<(&String, &String)> = p.iter().collect();
            kv.sort_by_key(|&(k, _)| k);
            std::hash::Hash::hash(&kv, &mut hasher);
        }
        let content_hash = std::hash::Hasher::finish(&hasher);

        Ok(Self {
            name: name.to_string(),
            content_hash,
            model_class,
            global_block,
            global_values,
            way_block,
            node_block,
            no_start_ways,
        })
    }
}

fn parse_no_start_ways(content: &str, way_lookups: &ContextLookups) -> Vec<(usize, i32)> {
    let mut no_start_ways = Vec::new();
    for line in content.lines() {
        if let Some(hash_idx) = line.find('#') {
            let comment = line[hash_idx + 1..].trim();
            if no_start_ways.is_empty() && comment.contains("noStartWay") {
                for part in comment.split('|') {
                    let part = part.trim();
                    if part.starts_with("noStartWay") {
                        if let Some(eq_idx) = part.find('=') {
                            let list_str = part[eq_idx + 1..].trim();
                            for pair in list_str.split(';') {
                                let kv: Vec<&str> = pair.split(',').collect();
                                if kv.len() >= 2 {
                                    let name = kv[0].trim();
                                    let value = kv[1].trim();
                                    if let Some(nidx) = way_lookups.get_name_idx(name) {
                                        let vidx =
                                            way_lookups.get_value_idx(nidx, value).unwrap_or(-1);
                                        no_start_ways.push((nidx, vidx));
                                    } else {
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    no_start_ways
}

fn resolve_foreign_vars(expr: &mut BExpr, way_var_map: &HashMap<String, usize>) {
    match expr {
        BExpr::ForeignVariable {
            context,
            var_name,
            var_idx,
        } => {
            if context == "way" {
                if let Some(&idx) = way_var_map.get(var_name) {
                    *var_idx = idx;
                }
            }
        }
        BExpr::Not(inner) => resolve_foreign_vars(inner, way_var_map),
        BExpr::Binary { left, right, .. } => {
            resolve_foreign_vars(left, way_var_map);
            resolve_foreign_vars(right, way_var_map);
        }
        BExpr::Switch {
            cond,
            then_expr,
            else_expr,
        } => {
            resolve_foreign_vars(cond, way_var_map);
            resolve_foreign_vars(then_expr, way_var_map);
            resolve_foreign_vars(else_expr, way_var_map);
        }
        BExpr::Assign { expr, .. } => {
            resolve_foreign_vars(expr, way_var_map);
        }
        _ => {}
    }
}

fn compile_global_block(
    tokens: &[String],
    lookups: &ContextLookups,
    params: Option<&HashMap<String, String>>,
) -> Result<(ContextBlock, Vec<f32>), String> {
    let mut var_names = Vec::new();
    let mut var_map = HashMap::new();
    let mut statements = Vec::new();
    let mut last_assigned_const: Vec<Option<f32>> = Vec::new();
    let mut protected_consts: Vec<Option<f32>> = Vec::new();

    if let Some(kv) = params {
        for (key, value) in kv {
            let val = match value.trim() {
                "true" => 1.0,
                "false" => 0.0,
                s => s
                    .parse::<f32>()
                    .map_err(|e| format!("Invalid float parameter '{}={}': {}", key, value, e))?,
            };
            let idx = if let Some(&i) = var_map.get(key) {
                i
            } else {
                let i = var_names.len();
                var_names.push(key.clone());
                var_map.insert(key.clone(), i);
                last_assigned_const.push(None);
                protected_consts.push(None);
                i
            };
            last_assigned_const[idx] = Some(val);
            protected_consts[idx] = Some(val);
            statements.push(BExpr::Assign {
                var_idx: idx,
                expr: Box::new(BExpr::Number(val)),
            });
        }
    }

    let mut cursor = 0;
    while cursor < tokens.len() {
        let stmt = parse_statement(
            tokens,
            &mut cursor,
            lookups,
            &mut var_names,
            &mut var_map,
            &mut last_assigned_const,
            &mut protected_consts,
            0,
        )?;
        statements.push(stmt);
    }

    let num_vars = var_names.len();
    let mut values = vec![0.0f32; num_vars];

    // Evaluate global statements once
    for stmt in &statements {
        eval_simple(stmt, &mut values);
    }

    Ok((
        ContextBlock {
            statements,
            var_names,
            var_map,
            min_write_idx: 0,
            num_vars,
            builtin_indices: Vec::new(),
        },
        values,
    ))
}

fn compile_context_block(
    _context_name: &str,
    tokens: &[String],
    lookups: &ContextLookups,
    builtins: &[&str],
    global_block: &ContextBlock,
    global_values: &[f32],
) -> Result<ContextBlock, String> {
    let mut var_names = Vec::new();
    let mut var_map = HashMap::new();
    let mut last_assigned_const = Vec::new();
    let mut protected_consts = Vec::new();

    // 1. Inherit all global variables as read-only at the beginning of the variable vector
    for (i, name) in global_block.var_names.iter().enumerate() {
        var_names.push(name.clone());
        var_map.insert(name.clone(), i);
        last_assigned_const.push(global_values.get(i).copied());
        protected_consts.push(None);
    }

    let min_write_idx = var_names.len();
    let mut statements = Vec::new();

    let mut cursor = 0;
    while cursor < tokens.len() {
        let stmt = parse_statement(
            tokens,
            &mut cursor,
            lookups,
            &mut var_names,
            &mut var_map,
            &mut last_assigned_const,
            &mut protected_consts,
            min_write_idx,
        )?;
        statements.push(stmt);
    }

    // 2. Resolve indices for built-in variables
    let mut builtin_indices = Vec::with_capacity(builtins.len());
    for &b in builtins {
        builtin_indices.push(var_map.get(b).copied());
    }

    let num_vars = var_names.len();
    Ok(ContextBlock {
        statements,
        var_names,
        var_map,
        min_write_idx,
        num_vars,
        builtin_indices,
    })
}

#[allow(clippy::too_many_arguments)]
fn parse_statement(
    tokens: &[String],
    cursor: &mut usize,
    lookups: &ContextLookups,
    var_names: &mut Vec<String>,
    var_map: &mut HashMap<String, usize>,
    last_assigned_const: &mut Vec<Option<f32>>,
    protected_consts: &mut Vec<Option<f32>>,
    min_write_idx: usize,
) -> Result<BExpr, String> {
    if *cursor >= tokens.len() {
        return Err("Unexpected end of tokens".to_string());
    }

    let op = &tokens[*cursor];
    *cursor += 1;

    if op != "assign" {
        return Err(format!("Expected 'assign' statement, got '{}'", op));
    }

    if *cursor >= tokens.len() {
        return Err("Expected variable name after 'assign'".to_string());
    }

    let var_name = tokens[*cursor].clone();
    *cursor += 1;

    // Optional '='
    if *cursor < tokens.len() && tokens[*cursor] == "=" {
        *cursor += 1;
    }

    let var_idx = if let Some(&idx) = var_map.get(&var_name) {
        if idx < min_write_idx && min_write_idx > 0 {
            return Err(format!(
                "Cannot assign to read-only inherited global variable '{}'",
                var_name
            ));
        }
        idx
    } else {
        let idx = var_names.len();
        var_names.push(var_name.clone());
        var_map.insert(var_name, idx);
        last_assigned_const.push(None);
        protected_consts.push(None);
        idx
    };

    let raw_expr = parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
    let opt_expr = if let Some(protected_val) = protected_consts[var_idx].take() {
        BExpr::Number(protected_val)
    } else {
        raw_expr.optimize()
    };

    last_assigned_const[var_idx] = match &opt_expr {
        BExpr::Number(n) => Some(*n),
        _ => None,
    };

    Ok(BExpr::Assign {
        var_idx,
        expr: Box::new(opt_expr),
    })
}

fn parse_expression(
    tokens: &[String],
    cursor: &mut usize,
    lookups: &ContextLookups,
    var_map: &HashMap<String, usize>,
    last_assigned_const: &[Option<f32>],
) -> Result<BExpr, String> {
    if *cursor >= tokens.len() {
        return Err("Unexpected end of expression tokens".to_string());
    }

    let mut brackets = false;
    let mut token = tokens[*cursor].clone();
    *cursor += 1;

    if token == "(" {
        brackets = true;
        if *cursor >= tokens.len() {
            return Err("Expected expression after '('".to_string());
        }
        token = tokens[*cursor].clone();
        *cursor += 1;
    }

    let expr = match token.as_str() {
        "switch" => {
            let cond = parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            let then_expr =
                parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            let else_expr =
                parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            BExpr::Switch {
                cond: Box::new(cond),
                then_expr: Box::new(then_expr),
                else_expr: Box::new(else_expr),
            }
        }
        "if" => {
            let cond = parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            expect_token(tokens, cursor, "then")?;
            let then_expr =
                parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            expect_token(tokens, cursor, "else")?;
            let else_expr =
                parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            BExpr::Switch {
                cond: Box::new(cond),
                then_expr: Box::new(then_expr),
                else_expr: Box::new(else_expr),
            }
        }
        "not" => {
            let inner = parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            BExpr::Not(Box::new(inner))
        }
        "or" | "and" | "xor" | "add" | "sub" | "multiply" | "divide" | "max" | "min" | "equal"
        | "greater" | "lesser" => {
            let bin_op = match token.as_str() {
                "or" => BinaryOp::Or,
                "and" => BinaryOp::And,
                "xor" => BinaryOp::Xor,
                "add" => BinaryOp::Add,
                "sub" => BinaryOp::Sub,
                "multiply" => BinaryOp::Multiply,
                "divide" => BinaryOp::Divide,
                "max" => BinaryOp::Max,
                "min" => BinaryOp::Min,
                "equal" => BinaryOp::Equal,
                "greater" => BinaryOp::Greater,
                "lesser" => BinaryOp::Lesser,
                _ => unreachable!(),
            };
            let left = parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            let right = parse_expression(tokens, cursor, lookups, var_map, last_assigned_const)?;
            BExpr::Binary {
                op: bin_op,
                left: Box::new(left),
                right: Box::new(right),
            }
        }
        "true" => BExpr::Number(1.0),
        "false" => BExpr::Number(0.0),
        _ => {
            // Elementary expressions:
            // 1. Tag lookup: contains '='
            if let Some(eq_pos) = token.find('=') {
                let name = &token[..eq_pos];
                let values_str = &token[eq_pos + 1..];

                let name_idx = lookups
                    .get_name_idx(name)
                    .ok_or_else(|| format!("Unknown lookup name in profile: '{}'", name))?;

                let allowed_values: Vec<i32> = if values_str.is_empty() {
                    vec![0] // Empty tag match
                } else {
                    let mut vals = Vec::new();
                    for v in values_str.split('|') {
                        let v_idx = lookups.get_value_idx(name_idx, v).ok_or_else(|| {
                            format!("Unknown lookup value in profile: '{}={}'", name, v)
                        })?;
                        vals.push(v_idx);
                    }
                    vals
                };

                BExpr::Lookup {
                    name_idx,
                    allowed_values,
                }
            } else if let Some(rest) = token.strip_prefix("v:") {
                // 2. Numerical tag extraction: v:<tag>
                let name_idx = lookups
                    .get_name_idx(rest)
                    .ok_or_else(|| format!("Unknown tag for v: '{}'", rest))?;
                BExpr::VariableGet(name_idx)
            } else if let Some(colon_pos) = token.find(':') {
                // 3. Foreign context variable: <context>:<var>
                let context = token[..colon_pos].to_string();
                let var_name = token[colon_pos + 1..].to_string();
                BExpr::ForeignVariable {
                    context,
                    var_name,
                    var_idx: 0,
                }
            } else if let Some(&idx) = var_map.get(&token) {
                // 4. Known local or inherited variable (propagate constant if known)
                if let Some(Some(const_val)) = last_assigned_const.get(idx) {
                    BExpr::Number(*const_val)
                } else {
                    BExpr::Variable(idx)
                }
            } else if let Ok(val) = token.parse::<f32>() {
                // 5. Number literal
                BExpr::Number(val)
            } else {
                return Err(format!("Unknown expression token: '{}'", token));
            }
        }
    };

    if brackets {
        expect_token(tokens, cursor, ")")?;
    }

    Ok(expr.optimize())
}

fn expect_token(tokens: &[String], cursor: &mut usize, expected: &str) -> Result<(), String> {
    if *cursor >= tokens.len() {
        return Err(format!("Expected '{}', got end of tokens", expected));
    }
    let actual = &tokens[*cursor];
    *cursor += 1;
    if actual != expected {
        return Err(format!("Expected '{}', got '{}'", expected, actual));
    }
    Ok(())
}

fn eval_simple(expr: &BExpr, vars: &mut [f32]) -> f32 {
    match expr {
        BExpr::Number(n) => *n,
        BExpr::Variable(idx) => vars[*idx],
        BExpr::Not(inner) => {
            let v = eval_simple(inner, vars);
            if v == 0.0 {
                1.0
            } else {
                0.0
            }
        }
        BExpr::Binary { op, left, right } => {
            let v1 = eval_simple(left, vars);
            let v2 = eval_simple(right, vars);
            op.eval(v1, v2)
        }
        BExpr::Switch {
            cond,
            then_expr,
            else_expr,
        } => {
            let c = eval_simple(cond, vars);
            if c != 0.0 {
                eval_simple(then_expr, vars)
            } else {
                eval_simple(else_expr, vars)
            }
        }
        BExpr::Assign { var_idx, expr } => {
            let v = eval_simple(expr, vars);
            vars[*var_idx] = v;
            v
        }
        _ => 0.0,
    }
}

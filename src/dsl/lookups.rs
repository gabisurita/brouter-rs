//! Rust implementation corresponding to:
//! - Java class: `btools.expressions.BExpressionMetaData`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpressionMetaData.java`
//! - Java class: `btools.expressions.BExpressionLookupValue`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-expressions/src/main/java/btools/expressions/BExpressionLookupValue.java`
//!
//! # Summary
//! Parses `lookups.dat`, assigning dense integer indices for tag names and enumerated values.
//! Decodes RD5 binary way/node description bitmasks into fast integer lookup arrays.

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader};
use std::path::Path;

use crate::rd5::codec::bit_coder::BitReader;

/// An enumerated tag value and its aliases from `lookups.dat`.
///
/// Corresponds to Java class `btools.expressions.BExpressionLookupValue`.
/// Tags in `lookups.dat` can define aliases (for example, `primary_link` as an alias of `primary`).
#[derive(Debug, Clone)]
pub struct LookupValue {
    pub value: String,
    pub aliases: Vec<String>,
}

#[derive(Debug, Clone, Default)]
pub struct ContextLookups {
    pub names: Vec<String>,
    pub name_to_idx: HashMap<String, usize>,
    pub values: Vec<Vec<LookupValue>>,
    pub value_to_idx: Vec<HashMap<String, i32>>,
}

impl ContextLookups {
    pub fn new(context_name: &str) -> Self {
        let mut cl = Self::default();
        if context_name == "way" {
            cl.add_lookup_value("reversedirection", "yes", &[]);
        } else if context_name == "node" {
            cl.add_lookup_value("nodeaccessgranted", "yes", &[]);
        }
        cl
    }

    pub fn add_lookup_value(&mut self, name: &str, value: &str, aliases: &[String]) {
        let name_idx = if let Some(&idx) = self.name_to_idx.get(name) {
            idx
        } else {
            let idx = self.names.len();
            self.names.push(name.to_string());
            self.name_to_idx.insert(name.to_string(), idx);

            // Index 0: "", Index 1: "unknown"
            let initial_values = vec![
                LookupValue {
                    value: String::new(),
                    aliases: Vec::new(),
                },
                LookupValue {
                    value: "unknown".to_string(),
                    aliases: Vec::new(),
                },
            ];
            let mut initial_map = HashMap::new();
            initial_map.insert(String::new(), 0);
            initial_map.insert("unknown".to_string(), 1);

            self.values.push(initial_values);
            self.value_to_idx.push(initial_map);
            idx
        };

        // Check if value already exists
        if let Some(&_vidx) = self.value_to_idx[name_idx].get(value) {
            // Already present, add any new aliases
            let vidx = self.value_to_idx[name_idx][value] as usize;
            for a in aliases {
                if !self.values[name_idx][vidx].aliases.contains(a) {
                    self.values[name_idx][vidx].aliases.push(a.clone());
                    self.value_to_idx[name_idx].insert(a.clone(), vidx as i32);
                }
            }
            return;
        }

        let val_idx = self.values[name_idx].len() as i32;
        self.values[name_idx].push(LookupValue {
            value: value.to_string(),
            aliases: aliases.to_vec(),
        });
        self.value_to_idx[name_idx].insert(value.to_string(), val_idx);
        for a in aliases {
            self.value_to_idx[name_idx].insert(a.clone(), val_idx);
        }
    }

    pub fn get_name_idx(&self, name: &str) -> Option<usize> {
        self.name_to_idx.get(name).copied()
    }

    pub fn get_value_idx(&self, name_idx: usize, value: &str) -> Option<i32> {
        if name_idx >= self.value_to_idx.len() {
            return None;
        }
        self.value_to_idx[name_idx].get(value).copied()
    }

    /// Decodes a binary tag bitmask into a dense integer lookup array.
    pub fn decode_into(&self, ld: &mut [i32], is_reverse: bool, bitmap: &[u8]) {
        ld.fill(0);
        if bitmap.is_empty() {
            return;
        }

        let mut ctx = BitReader::new(bitmap);

        // ld[0] is hardwired for reversedirection / nodeaccessgranted
        ld[0] = if is_reverse { 2 } else { 0 };

        let mut inum = 1usize;
        loop {
            let mut delta = ctx.decode_var_bits();
            if delta == 0 {
                break;
            }
            if inum + delta as usize > ld.len() {
                break;
            }

            while delta > 1 {
                if inum < ld.len() {
                    ld[inum] = 0;
                }
                inum += 1;
                delta -= 1;
            }

            let dd = ctx.decode_var_bits();
            let mut d = if dd == 7 {
                1
            } else if dd < 7 {
                dd + 2
            } else {
                dd + 1
            };

            if inum < self.values.len() && d as usize >= self.values[inum].len() && d < 1000 {
                d = 1;
            }
            if inum < ld.len() {
                ld[inum] = d;
            }
            inum += 1;
        }
    }

    /// Extracts the numerical float value for `v:<tag>` (e.g. `v:maxspeed`).
    ///
    /// Matches `BExpressionMetaData.getLookupValue`:
    /// - Value 0 represents an unset tag, returning `NaN`.
    /// - Values below 900 are looked up as literal strings in the dictionary and parsed.
    /// - Values 900 and above are encoded fixed-point numbers: `(val - 1000) / 100.0`.
    pub fn get_lookup_value(&self, name_idx: usize, val: i32) -> f32 {
        if val == 0 {
            return f32::NAN;
        }
        if val < 900 {
            if name_idx < self.values.len() && (val as usize) < self.values[name_idx].len() {
                let s = &self.values[name_idx][val as usize].value;
                s.parse::<f32>().unwrap_or(0.0)
            } else {
                0.0
            }
        } else {
            (val - 1000) as f32 / 100.0
        }
    }

    /// Formats a decoded lookup array into BRouter's space-separated `key=value` description
    /// matching `BExpressionContext.getKeyValueDescription`.
    pub fn format_key_values(&self, ld: &[i32]) -> String {
        let mut out = String::new();
        for (i, &val) in ld.iter().enumerate() {
            if val == 0 || i >= self.names.len() {
                continue;
            }
            if !out.is_empty() {
                out.push(' ');
            }
            out.push_str(&self.names[i]);
            out.push('=');
            if val < 900 {
                if let Some(lv) = self.values.get(i).and_then(|v| v.get(val as usize)) {
                    out.push_str(&lv.value);
                } else {
                    out.push_str("unknown");
                }
            } else {
                let num = (val - 1000) as f32 / 100.0;
                if (num - num.round()).abs() < 1e-4 {
                    out.push_str(&format!("{}", num.round() as i32));
                } else {
                    out.push_str(&format!("{}", num));
                }
            }
        }
        out
    }
}

/// Metadata and lookup dictionary loaded from `lookups.dat`.
#[derive(Debug, Clone)]
pub struct LookupData {
    pub lookup_version: i16,
    pub minor_version: i16,
    pub min_app_version: i16,
    pub way: ContextLookups,
    pub node: ContextLookups,
}

impl LookupData {
    pub fn parse(text: &str) -> io::Result<Self> {
        Self::load_from_reader(text.as_bytes())
    }

    pub fn load_from_file<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let file = File::open(path)?;
        Self::load_from_reader(BufReader::new(file))
    }

    pub fn load_from_reader<R: BufRead>(reader: R) -> io::Result<Self> {
        let mut lookup_version = -1i16;
        let mut minor_version = -1i16;
        let mut min_app_version = -1i16;

        let mut way = ContextLookups::new("way");
        let mut node = ContextLookups::new("node");
        let mut current_context = "way";

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("---context:") {
                current_context = match rest.trim() {
                    "way" => "way",
                    "node" => "node",
                    _ => current_context,
                };
                continue;
            }

            if let Some(rest) = trimmed.strip_prefix("---lookupversion:") {
                lookup_version = rest.trim().parse().unwrap_or(-1);
                continue;
            }
            if let Some(rest) = trimmed.strip_prefix("---minorversion:") {
                minor_version = rest.trim().parse().unwrap_or(-1);
                continue;
            }
            if let Some(rest) = trimmed.strip_prefix("---minappversion:") {
                min_app_version = rest.trim().parse().unwrap_or(-1);
                continue;
            }

            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }

            let mut key = parts[0];
            if let Some(idx) = key.find(';') {
                key = &key[..idx];
            }

            if key == "reversedirection" || key == "nodeaccessgranted" {
                continue;
            }

            let value = if parts.len() > 1 { parts[1] } else { "" };
            let aliases: Vec<String> = if parts.len() > 2 {
                parts[2..].iter().map(|s| s.to_string()).collect()
            } else {
                Vec::new()
            };

            if current_context == "way" {
                way.add_lookup_value(key, value, &aliases);
            } else if current_context == "node" {
                node.add_lookup_value(key, value, &aliases);
            }
        }

        Ok(Self {
            lookup_version,
            minor_version,
            min_app_version,
            way,
            node,
        })
    }
}

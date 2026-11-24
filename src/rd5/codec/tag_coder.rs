//! Rust implementation corresponding to:
//! - Java class: `btools.codec.TagValueCoder`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/main/java/btools/codec/TagValueCoder.java`
//! - Java class: `btools.codec.TagValueWrapper`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/main/java/btools/codec/TagValueWrapper.java`
//!
//! # Summary
//! Huffman tree decoder for tag value sets on nodes and links in RD5 files.

use super::bit_coder::{BitCoderContext, StatCoderContext};

/// TagValueWrapper contains the compact byte-encoded tag sequence and access permission.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TagValueWrapper {
    pub data: Vec<u8>,
    pub access_type: i32,
}

/// Optional validator for filtering tags and checking access rights during decode.
///
/// Corresponds to Java class `btools.codec.TagValueValidator`.
pub trait TagValueValidator: Sync {
    fn is_lookup_idx_used(&self, idx: i32) -> bool;
    fn access_type(&self, data: &[u8]) -> i32;
}

/// Node in the Huffman prefix tree.
pub enum HuffmanNode {
    Branch {
        child1: Box<HuffmanNode>,
        child2: Box<HuffmanNode>,
    },
    Leaf(Option<TagValueWrapper>),
}

/// TagValueCoder decodes tag descriptions using a Huffman prefix tree.
///
/// Corresponds to Java class `btools.codec.TagValueCoder`.
/// MicroCache cells encode a prefix tree header defining a dictionary of tag bitmasks.
pub struct TagValueCoder {
    pub tree: HuffmanNode,
}

const MAX_HUFFMAN_DEPTH: usize = 64;
const MAX_TAG_BUF_LEN: usize = 4096;

impl TagValueCoder {
    /// Decodes the Huffman dictionary from the bitstream.
    pub fn new(bc: &mut StatCoderContext, validator: Option<&dyn TagValueValidator>) -> Self {
        Self::try_new(bc, validator).unwrap_or(Self {
            tree: HuffmanNode::Leaf(None),
        })
    }

    /// Decodes the Huffman dictionary from the bitstream, returning an error on truncated or malformed inputs.
    pub fn try_new(
        bc: &mut StatCoderContext,
        validator: Option<&dyn TagValueValidator>,
    ) -> std::io::Result<Self> {
        let tree = Self::decode_tree_checked(&mut bc.bit_coder, validator, 0)?;
        Ok(Self { tree })
    }

    fn decode_tree_checked(
        bc: &mut BitCoderContext,
        validator: Option<&dyn TagValueValidator>,
        depth: usize,
    ) -> std::io::Result<HuffmanNode> {
        if depth > MAX_HUFFMAN_DEPTH || bc.is_past_eof() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Malformed or truncated Huffman tag tree",
            ));
        }
        let is_node = bc.decode_bit();
        if bc.is_past_eof() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Truncated Huffman tag tree",
            ));
        }
        if is_node {
            let child1 = Box::new(Self::decode_tree_checked(bc, validator, depth + 1)?);
            let child2 = Box::new(Self::decode_tree_checked(bc, validator, depth + 1)?);
            return Ok(HuffmanNode::Branch { child1, child2 });
        }

        let mut inum = 0i32;
        let mut last_encoded_inum = 0i32;
        let mut entries: Vec<(i32, i32)> = Vec::new();

        loop {
            let delta = bc.decode_var_bits();
            if bc.is_past_eof() || delta < 0 {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Malformed tag index delta in Huffman leaf",
                ));
            }
            if entries.is_empty() && delta == 0 {
                return Ok(HuffmanNode::Leaf(None));
            }
            if delta == 0 {
                break;
            }
            inum = inum.saturating_add(delta);

            let data = bc.decode_var_bits();
            if bc.is_past_eof() || data < 0 {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Malformed tag value in Huffman leaf",
                ));
            }

            let used = match validator {
                Some(v) => v.is_lookup_idx_used(inum),
                None => true,
            };

            if used {
                entries.push((inum - last_encoded_inum, data));
                last_encoded_inum = inum;
                if entries.len() > MAX_TAG_BUF_LEN {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Huffman leaf tag set exceeds maximum length",
                    ));
                }
            }
        }

        let mut cap = (entries.len() * 6 + 16).clamp(64, MAX_TAG_BUF_LEN);
        let res = loop {
            let mut buffer = vec![0u8; cap];
            let len = {
                let mut ctx = BitCoderContext::new(&mut buffer);
                for &(delta_inum, data) in &entries {
                    ctx.encode_var_bits(delta_inum);
                    ctx.encode_var_bits(data);
                }
                ctx.encode_var_bits(0);
                ctx.close_and_get_encoded_length()
            };
            if len <= buffer.len() {
                buffer.truncate(len);
                break buffer;
            }
            if cap >= MAX_TAG_BUF_LEN {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Encoded Huffman tag set exceeds maximum buffer size",
                ));
            }
            cap = MAX_TAG_BUF_LEN;
        };

        let access_type = match validator {
            Some(v) => v.access_type(&res),
            None => 2,
        };

        if access_type > 0 {
            Ok(HuffmanNode::Leaf(Some(TagValueWrapper {
                data: res,
                access_type,
            })))
        } else {
            Ok(HuffmanNode::Leaf(None))
        }
    }

    /// Decodes the next tag value set from the bitstream using the Huffman tree.
    pub fn decode_tag_value_set(&self, bc: &mut StatCoderContext) -> Option<TagValueWrapper> {
        let mut current = &self.tree;
        while let HuffmanNode::Branch { child1, child2 } = current {
            let bit = bc.decode_bit();
            current = if bit { child2 } else { child1 };
        }

        if let HuffmanNode::Leaf(ref wrapper) = current {
            wrapper.clone()
        } else {
            None
        }
    }
}

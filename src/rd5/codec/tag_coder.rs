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

impl TagValueCoder {
    /// Decodes the Huffman dictionary from the bitstream.
    pub fn new(bc: &mut StatCoderContext, validator: Option<&dyn TagValueValidator>) -> Self {
        let tree = Self::decode_tree(&mut bc.bit_coder, validator);
        Self { tree }
    }

    fn decode_tree(
        bc: &mut BitCoderContext,
        validator: Option<&dyn TagValueValidator>,
    ) -> HuffmanNode {
        let is_node = bc.decode_bit();
        if is_node {
            let child1 = Box::new(Self::decode_tree(bc, validator));
            let child2 = Box::new(Self::decode_tree(bc, validator));
            return HuffmanNode::Branch { child1, child2 };
        }

        let mut buffer = [0u8; 512];
        let mut ctx = BitCoderContext::new(&mut buffer);

        let mut inum = 0i32;
        let mut last_encoded_inum = 0i32;
        let mut has_data = false;

        loop {
            let delta = bc.decode_var_bits();
            if !has_data && delta == 0 {
                return HuffmanNode::Leaf(None);
            }
            if delta == 0 {
                ctx.encode_var_bits(0);
                break;
            }
            inum += delta;

            let data = bc.decode_var_bits();

            let used = match validator {
                Some(v) => v.is_lookup_idx_used(inum),
                None => true,
            };

            if used {
                has_data = true;
                ctx.encode_var_bits(inum - last_encoded_inum);
                ctx.encode_var_bits(data);
                last_encoded_inum = inum;
            }
        }

        let len = ctx.close_and_get_encoded_length().min(buffer.len());
        let res = buffer[..len].to_vec();

        let access_type = match validator {
            Some(v) => v.access_type(&res),
            None => 2,
        };

        if access_type > 0 {
            HuffmanNode::Leaf(Some(TagValueWrapper {
                data: res,
                access_type,
            }))
        } else {
            HuffmanNode::Leaf(None)
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

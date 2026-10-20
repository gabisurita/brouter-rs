//! Rust implementation corresponding to:
//! - Java class: `btools.router.VoiceHint`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/VoiceHint.java`
//! - Java class: `btools.router.VoiceHintProcessor`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/VoiceHintProcessor.java`
//! - Java class: `btools.router.MessageData`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/MessageData.java`
//!
//! # Summary
//! Per-segment route message data and turn-by-turn voice hint generation.

/// Per-segment metadata matching `btools.router.MessageData`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WayMessage {
    pub linkdist: i32,
    pub linkelevationcost: i32,
    pub linkturncost: i32,
    pub linknodecost: i32,
    pub linkinitcost: i32,
    pub costfactor: f32,
    pub priorityclassifier: i32,
    pub classifiermask: i32,
    pub turnangle: f32,
    pub way_key_values: String,
    pub node_key_values: String,
    pub lon: i32,
    pub lat: i32,
    pub ele: i16,
    pub time: f32,
    pub energy: f32,
}

impl WayMessage {
    #[inline]
    pub fn prio(&self) -> i32 {
        self.priorityclassifier
    }

    #[inline]
    pub fn is_bad_oneway(&self) -> bool {
        (self.classifiermask & 1) != 0
    }

    #[inline]
    pub fn is_good_oneway(&self) -> bool {
        (self.classifiermask & 2) != 0
    }

    #[inline]
    pub fn is_roundabout(&self) -> bool {
        (self.classifiermask & 4) != 0
    }

    #[inline]
    pub fn is_link_type(&self) -> bool {
        (self.classifiermask & 8) != 0
    }

    #[inline]
    pub fn is_good_for_cars(&self) -> bool {
        (self.classifiermask & 16) != 0
    }

    pub fn add(&mut self, other: &WayMessage) {
        self.linkdist += other.linkdist;
        self.linkelevationcost += other.linkelevationcost;
        self.linkturncost += other.linkturncost;
        self.linknodecost += other.linknodecost;
        self.linkinitcost += other.linkinitcost;
    }

    /// Formats the 13 columns of `MessageData.toMessage()`.
    pub fn to_message_fields(&self) -> [String; 13] {
        let i_cost = (self.costfactor * 1000.0 + 0.5) as i32;
        [
            (self.lon - 180_000_000).to_string(),
            (self.lat - 90_000_000).to_string(),
            (self.ele / 4).to_string(),
            self.linkdist.to_string(),
            i_cost.to_string(),
            self.linkelevationcost.to_string(),
            self.linkturncost.to_string(),
            self.linknodecost.to_string(),
            self.linkinitcost.to_string(),
            self.way_key_values.clone(),
            self.node_key_values.clone(),
            (self.time as i32).to_string(),
            (self.energy as i32).to_string(),
        ]
    }
}

/// Aggregates per-point `WayMessage` entries across consecutive segments with identical way tags
/// and empty intermediate node tags, matching `OsmTrack.aggregateMessages()`.
pub fn aggregate_messages(messages: &[Option<WayMessage>]) -> Vec<[String; 13]> {
    let mut res = Vec::new();
    let mut current: Option<WayMessage> = None;

    for msg_opt in messages {
        if let Some(md_ref) = msg_opt {
            if md_ref.way_key_values.is_empty() {
                continue;
            }
            let mut md = md_ref.clone();
            if let Some(cur) = current.take() {
                if !cur.node_key_values.is_empty() || cur.way_key_values != md.way_key_values {
                    res.push(cur.to_message_fields());
                } else {
                    md.add(&cur);
                }
            }
            current = Some(md);
        }
    }
    if let Some(cur) = current {
        res.push(cur.to_message_fields());
    }
    res
}

/// Transport mode used for turn instruction thresholds (`VoiceHintList`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportMode {
    Bike = 1,
    Foot = 2,
    Car = 3,
}

impl TransportMode {
    pub fn from_flags(is_car: bool, is_bike: bool) -> Self {
        if is_car {
            Self::Car
        } else if is_bike {
            Self::Bike
        } else {
            Self::Foot
        }
    }

    pub fn min_distance(self) -> f64 {
        match self {
            Self::Car => 20.0,
            Self::Foot => 3.0,
            Self::Bike => 5.0,
        }
    }
}

/// Turn instruction command constants and per-hint data matching `btools.router.VoiceHint`.
#[derive(Debug, Clone, PartialEq)]
pub struct VoiceHint {
    pub ilon: i32,
    pub ilat: i32,
    pub selev: i16,
    pub cmd: i32,
    pub old_way: WayMessage,
    pub good_way: WayMessage,
    pub bad_ways: Vec<WayMessage>,
    pub distance_to_next: f64,
    pub index_in_track: usize,
    pub angle: f32,
    pub lower_bad_way_angle: f32,
    pub higher_bad_way_angle: f32,
    pub turn_angle_consumed: bool,
    pub needs_real_turn: bool,
    pub max_bad_prio: i32,
    pub roundabout_exit: i32,
}

impl Default for VoiceHint {
    fn default() -> Self {
        Self {
            ilon: 0,
            ilat: 0,
            selev: i16::MIN,
            cmd: 0,
            old_way: WayMessage::default(),
            good_way: WayMessage::default(),
            bad_ways: Vec::new(),
            distance_to_next: 0.0,
            index_in_track: 0,
            angle: f32::MAX,
            lower_bad_way_angle: -181.0,
            higher_bad_way_angle: 181.0,
            turn_angle_consumed: false,
            needs_real_turn: false,
            max_bad_prio: -1,
            roundabout_exit: 0,
        }
    }
}

impl VoiceHint {
    pub const C: i32 = 1; // continue (go straight)
    pub const TL: i32 = 2; // turn left
    pub const TSLL: i32 = 3; // turn slightly left
    pub const TSHL: i32 = 4; // turn sharply left
    pub const TR: i32 = 5; // turn right
    pub const TSLR: i32 = 6; // turn slightly right
    pub const TSHR: i32 = 7; // turn sharply right
    pub const KL: i32 = 8; // keep left
    pub const KR: i32 = 9; // keep right
    pub const TLU: i32 = 10; // left U-turn
    pub const TRU: i32 = 11; // right U-turn
    pub const OFFR: i32 = 12; // off route
    pub const RNDB: i32 = 13; // roundabout
    pub const RNLB: i32 = 14; // roundabout left
    pub const TU: i32 = 15; // 180-degree u-turn
    pub const BL: i32 = 16; // beeline routing
    pub const EL: i32 = 17; // exit left
    pub const ER: i32 = 18; // exit right
    pub const END: i32 = 100; // end point

    #[inline]
    pub fn is_roundabout(&self) -> bool {
        self.roundabout_exit != 0
    }

    #[inline]
    pub fn exit_number(&self) -> i32 {
        self.roundabout_exit
    }

    #[inline]
    pub fn is_180_deg_angle(angle: f32) -> bool {
        angle.abs() <= 180.0 && angle.abs() >= 179.0
    }

    pub fn has_give_way(&self) -> bool {
        if !self.old_way.node_key_values.is_empty() {
            let has_stop_or_give_way = self.old_way.node_key_values.contains("highway=give_way")
                || self.old_way.node_key_values.contains("highway=stop");
            let is_bwd = self
                .old_way
                .node_key_values
                .contains("direction=backward");
            if self.old_way.way_key_values.contains("reversedirection=yes") {
                return has_stop_or_give_way && is_bwd;
            } else {
                return has_stop_or_give_way && !is_bwd;
            }
        }
        false
    }

    pub fn calc_command(&mut self) {
        for bad_way in &self.bad_ways {
            if bad_way.is_bad_oneway() {
                continue;
            }
            if self.lower_bad_way_angle < bad_way.turnangle
                && bad_way.turnangle < self.good_way.turnangle
            {
                self.lower_bad_way_angle = bad_way.turnangle;
            }
            if self.higher_bad_way_angle > bad_way.turnangle
                && bad_way.turnangle > self.good_way.turnangle
            {
                self.higher_bad_way_angle = bad_way.turnangle;
            }
        }

        let mut cmd_angle = self.angle;
        if self.angle == f32::MAX {
            cmd_angle = self.good_way.turnangle;
        }
        if self.cmd == Self::BL {
            return;
        }

        if self.roundabout_exit > 0 {
            self.cmd = Self::RNDB;
        } else if self.roundabout_exit < 0 {
            self.cmd = Self::RNLB;
        } else if Self::is_180_deg_angle(cmd_angle)
            && cmd_angle <= -179.0
            && self.higher_bad_way_angle == 181.0
            && self.lower_bad_way_angle == -181.0
        {
            self.cmd = Self::TU;
        } else if cmd_angle < -159.0 {
            self.cmd = Self::TLU;
        } else if cmd_angle < -135.0 {
            self.cmd = Self::TSHL;
        } else if cmd_angle < -45.0 {
            if cmd_angle < -95.0
                && self.higher_bad_way_angle < -30.0
                && self.lower_bad_way_angle < -180.0
            {
                self.cmd = Self::TSHL;
            } else if cmd_angle > -85.0
                && self.lower_bad_way_angle > -180.0
                && self.higher_bad_way_angle > -10.0
            {
                self.cmd = Self::TSLL;
            } else if cmd_angle < -110.0 {
                self.cmd = Self::TSHL;
            } else if cmd_angle > -60.0 {
                self.cmd = Self::TSLL;
            } else {
                self.cmd = Self::TL;
            }
        } else if cmd_angle < -21.0 {
            if self.cmd != Self::KR {
                self.cmd = Self::TSLL;
            }
        } else if cmd_angle < -5.0 {
            if self.lower_bad_way_angle < -100.0 && self.higher_bad_way_angle < 45.0 {
                self.cmd = Self::TSLL;
            } else if self.lower_bad_way_angle >= -100.0 && self.higher_bad_way_angle < 45.0 {
                self.cmd = Self::KL;
            } else if self.lower_bad_way_angle > -35.0 && self.higher_bad_way_angle > 55.0 {
                self.cmd = Self::KR;
            } else {
                self.cmd = Self::C;
            }
        } else if cmd_angle < 5.0 {
            if self.lower_bad_way_angle > -30.0 {
                self.cmd = Self::KR;
            } else if self.higher_bad_way_angle < 30.0 {
                self.cmd = Self::KL;
            } else {
                self.cmd = Self::C;
            }
        } else if cmd_angle < 21.0 {
            if self.lower_bad_way_angle > -45.0 && self.higher_bad_way_angle > 100.0 {
                self.cmd = Self::TSLR;
            } else if self.lower_bad_way_angle > -45.0 && self.higher_bad_way_angle <= 100.0 {
                self.cmd = Self::KR;
            } else if self.lower_bad_way_angle < -55.0 && self.higher_bad_way_angle < 35.0 {
                self.cmd = Self::KL;
            } else {
                self.cmd = Self::C;
            }
        } else if cmd_angle < 45.0 {
            self.cmd = Self::TSLR;
        } else if cmd_angle < 135.0 {
            if cmd_angle < 85.0
                && self.higher_bad_way_angle < 180.0
                && self.lower_bad_way_angle < 10.0
            {
                self.cmd = Self::TSLR;
            } else if cmd_angle > 95.0
                && self.lower_bad_way_angle > 30.0
                && self.higher_bad_way_angle > 180.0
            {
                self.cmd = Self::TSHR;
            } else if cmd_angle > 110.0 {
                self.cmd = Self::TSHR;
            } else if cmd_angle < 60.0 {
                self.cmd = Self::TSLR;
            } else {
                self.cmd = Self::TR;
            }
        } else if cmd_angle < 159.0 {
            self.cmd = Self::TSHR;
        } else if Self::is_180_deg_angle(cmd_angle)
            && cmd_angle >= 179.0
            && self.higher_bad_way_angle == 181.0
            && self.lower_bad_way_angle == -181.0
        {
            self.cmd = Self::TU;
        } else {
            self.cmd = Self::TRU;
        }
    }

    pub fn format_geometry(&self) -> String {
        let mut sb = format!(
            " {}({}){}",
            self.old_way.priorityclassifier,
            (self.good_way.turnangle + 0.5) as i32,
            self.good_way.priorityclassifier
        );
        for bad_way in &self.bad_ways {
            sb.push_str(&format!(
                " ({}){}",
                (bad_way.turnangle + 0.5) as i32,
                bad_way.priorityclassifier
            ));
        }
        sb
    }

    pub fn json_command_index(&self, timode: i32) -> i32 {
        match self.cmd {
            Self::TLU => 10,
            Self::TU => 15,
            Self::TSHL => 4,
            Self::TL => 2,
            Self::TSLL => 3,
            Self::KL => 8,
            Self::C => 1,
            Self::KR => 9,
            Self::TSLR => 6,
            Self::TR => 5,
            Self::TSHR => 7,
            Self::TRU => 11,
            Self::RNDB => 13,
            Self::RNLB => 14,
            Self::BL => 16,
            Self::EL => {
                if timode == 2 || timode == 9 {
                    17
                } else {
                    8
                }
            }
            Self::ER => {
                if timode == 2 || timode == 9 {
                    18
                } else {
                    9
                }
            }
            Self::OFFR => 12,
            _ => 1,
        }
    }

    pub fn command_string(&self, timode: i32) -> String {
        match self.cmd {
            Self::TLU | Self::TU => "TU".to_string(),
            Self::TSHL => "TSHL".to_string(),
            Self::TL => "TL".to_string(),
            Self::TSLL => "TSLL".to_string(),
            Self::KL => "KL".to_string(),
            Self::C => "C".to_string(),
            Self::KR => "KR".to_string(),
            Self::TSLR => "TSLR".to_string(),
            Self::TR => "TR".to_string(),
            Self::TSHR => "TSHR".to_string(),
            Self::TRU => "TRU".to_string(),
            Self::RNDB => format!("RNDB{}", self.roundabout_exit),
            Self::RNLB => format!("RNLB{}", -self.roundabout_exit),
            Self::BL => "BL".to_string(),
            Self::EL => {
                if timode == 2 || timode == 9 {
                    "EL".to_string()
                } else {
                    "KL".to_string()
                }
            }
            Self::ER => {
                if timode == 2 || timode == 9 {
                    "ER".to_string()
                } else {
                    "KR".to_string()
                }
            }
            Self::OFFR => "OFFR".to_string(),
            Self::END => "END".to_string(),
            _ => "C".to_string(),
        }
    }
}

/// Turn-by-turn voice hint processor matching `btools.router.VoiceHintProcessor`.
pub struct VoiceHintProcessor {
    pub explicit_roundabouts: bool,
    pub transport_mode: TransportMode,
}

impl VoiceHintProcessor {
    const SIGNIFICANT_ANGLE: f64 = 22.5;
    const INTERNAL_CATCHING_RANGE_NEAR: f64 = 2.0;
    const INTERNAL_CATCHING_RANGE_WIDE: f64 = 10.0;

    pub fn new(explicit_roundabouts: bool, transport_mode: TransportMode) -> Self {
        Self {
            explicit_roundabouts,
            transport_mode,
        }
    }

    fn sum_non_consumed_within_catching_range(
        inputs: &mut [VoiceHint],
        mut offset: isize,
        range: f64,
    ) -> f32 {
        let mut distance = 0.0;
        let mut angle = 0.0f32;
        while offset >= 0 && distance < range {
            let input = &mut inputs[offset as usize];
            offset -= 1;
            if input.turn_angle_consumed
                || input.cmd == VoiceHint::BL
                || input.cmd == VoiceHint::END
            {
                break;
            }
            angle += input.good_way.turnangle;
            distance += input.good_way.linkdist as f64;
            input.turn_angle_consumed = true;
        }
        angle
    }

    /// Processes reverse-ordered track node hints into forward-ordered voice hints.
    pub fn process(&self, mut inputs: Vec<VoiceHint>) -> Vec<VoiceHint> {
        let mut results: Vec<VoiceHint> = Vec::new();
        let mut distance = 0.0;
        let mut roundabout_turn_angle = 0.0f32;
        let mut roundabout_exit = 0;
        let mut roundabout_start_idx: isize = -1;

        for hint_idx in 0..inputs.len() {
            if inputs[hint_idx].cmd == VoiceHint::BL {
                results.push(inputs[hint_idx].clone());
                continue;
            }

            let turn_angle = inputs[hint_idx].good_way.turnangle;
            if hint_idx != 0 {
                distance += inputs[hint_idx].good_way.linkdist as f64;
            }

            let current_prio = inputs[hint_idx].good_way.prio();
            let old_prio = inputs[hint_idx].old_way.prio();
            let min_prio = old_prio.min(current_prio);

            let is_link_2_highway =
                inputs[hint_idx].old_way.is_link_type() && !inputs[hint_idx].good_way.is_link_type();
            let is_highway_2_link =
                !inputs[hint_idx].old_way.is_link_type() && inputs[hint_idx].good_way.is_link_type();

            if self.explicit_roundabouts && inputs[hint_idx].old_way.is_roundabout() {
                if roundabout_start_idx == -1 {
                    roundabout_start_idx = hint_idx as isize;
                }
                roundabout_turn_angle += Self::sum_non_consumed_within_catching_range(
                    &mut inputs,
                    hint_idx as isize,
                    Self::INTERNAL_CATCHING_RANGE_NEAR,
                );
                if roundabout_start_idx == hint_idx as isize {
                    if !inputs[hint_idx].bad_ways.is_empty() {
                        roundabout_turn_angle -= inputs[hint_idx].good_way.turnangle;
                        for bad_way in &inputs[hint_idx].bad_ways {
                            if !bad_way.is_bad_oneway() {
                                roundabout_turn_angle += bad_way.turnangle;
                            }
                        }
                    }
                }
                let mut is_exit = roundabout_exit == 0;
                for bad_way in &inputs[hint_idx].bad_ways {
                    if !bad_way.is_bad_oneway() && bad_way.is_good_for_cars() {
                        is_exit = true;
                        break;
                    }
                }
                if is_exit {
                    roundabout_exit += 1;
                }
                continue;
            }

            if roundabout_exit > 0 {
                inputs[hint_idx].angle = roundabout_turn_angle;
                inputs[hint_idx].good_way.turnangle = roundabout_turn_angle;
                inputs[hint_idx].distance_to_next = distance;
                inputs[hint_idx].turn_angle_consumed = true;
                inputs[hint_idx].roundabout_exit = if roundabout_turn_angle < 0.0 {
                    roundabout_exit
                } else {
                    -roundabout_exit
                };

                let mut tmp_angle = 0.0f32;
                let mut tmp_bad_ways = Vec::new();
                let mut i = (hint_idx as isize) - 1;
                while i > roundabout_start_idx {
                    let vh = &inputs[i as usize];
                    tmp_angle += vh.good_way.turnangle;
                    for bad_way in &vh.bad_ways {
                        if !bad_way.is_bad_oneway() {
                            tmp_bad_ways.push(WayMessage {
                                linkdist: vh.good_way.linkdist,
                                priorityclassifier: vh.good_way.priorityclassifier,
                                turnangle: tmp_angle,
                                ..Default::default()
                            });
                        }
                    }
                    i -= 1;
                }
                distance = 0.0;
                inputs[hint_idx].bad_ways = tmp_bad_ways;
                results.push(inputs[hint_idx].clone());
                roundabout_turn_angle = 0.0;
                roundabout_exit = 0;
                roundabout_start_idx = -1;
                continue;
            }

            let next_has_give_way = inputs
                .get(hint_idx + 1)
                .map(|n| n.has_give_way())
                .unwrap_or(false);

            let mut max_prio_all = -1;
            let mut max_prio_candidates = -1;
            let mut max_angle = -180.0f32;
            let mut min_angle = 180.0f32;
            let mut min_abs_angle_raw = 180.0f32;
            let mut is_badway_link = false;
            let has_bad_ways = !inputs[hint_idx].bad_ways.is_empty();

            let mut new_max_bad_prio = inputs[hint_idx].max_bad_prio;
            for bad_way in &inputs[hint_idx].bad_ways {
                let bad_prio = bad_way.prio();
                let bad_turn = bad_way.turnangle;
                if bad_way.is_link_type() {
                    is_badway_link = true;
                }
                if bad_prio > max_prio_all {
                    max_prio_all = bad_prio;
                    new_max_bad_prio = new_max_bad_prio.max(bad_prio);
                }
                if bad_way.is_bad_oneway() {
                    if min_abs_angle_raw == 180.0 {
                        min_abs_angle_raw = turn_angle.abs();
                    }
                    continue;
                }
                if bad_turn.abs() - turn_angle.abs() > 80.0 {
                    if min_abs_angle_raw == 180.0 {
                        min_abs_angle_raw = turn_angle.abs();
                    }
                    continue;
                }
                if bad_way.costfactor < 20.0 && bad_turn.abs() < min_abs_angle_raw {
                    min_abs_angle_raw = bad_turn.abs();
                }
                if bad_prio > max_prio_candidates {
                    max_prio_candidates = bad_prio;
                    new_max_bad_prio = new_max_bad_prio.max(bad_prio);
                }
                if bad_turn > max_angle {
                    max_angle = bad_turn;
                }
                if bad_turn < min_angle {
                    min_angle = bad_turn;
                }
            }
            inputs[hint_idx].max_bad_prio = new_max_bad_prio;

            let has_something_more_straight = turn_angle.abs() > 35.0 && has_bad_ways;
            let no_link_but_bad_way_prio = max_prio_all > min_prio && !is_link_2_highway;
            let bad_way_has_prio = max_prio_candidates > current_prio;
            let is_u_turn = VoiceHint::is_180_deg_angle(turn_angle);
            let is_bad_way_link_but_no_link =
                !is_highway_2_link && is_badway_link && turn_angle.abs() > 5.0;
            let is_link_but_no_bad_way_link =
                is_highway_2_link && !is_badway_link && turn_angle.abs() < 5.0;
            let same_prio_small_bad_angle = current_prio == old_prio
                && (min_prio - max_prio_all <= 2)
                && !is_badway_link
                && min_abs_angle_raw != 180.0
                && min_abs_angle_raw < 35.0;
            let must_give_way = self.transport_mode != TransportMode::Foot
                && has_bad_ways
                && !bad_way_has_prio
                && (inputs[hint_idx].has_give_way() || next_has_give_way);

            let unconditional_trigger = has_something_more_straight
                || no_link_but_bad_way_prio
                || bad_way_has_prio
                || is_u_turn
                || is_bad_way_link_but_no_link
                || is_link_but_no_bad_way_link
                || same_prio_small_bad_angle
                || must_give_way;

            let conditional_trigger = max_prio_candidates >= min_prio;

            if unconditional_trigger || conditional_trigger {
                inputs[hint_idx].angle = turn_angle;
                inputs[hint_idx].calc_command();
                let is_straight = inputs[hint_idx].cmd == VoiceHint::C;
                inputs[hint_idx].needs_real_turn = !unconditional_trigger && is_straight;

                if turn_angle.abs() > 5.0 {
                    if max_angle < turn_angle
                        && max_angle > turn_angle - 45.0 - turn_angle.max(0.0)
                    {
                        inputs[hint_idx].cmd = VoiceHint::KR;
                    }
                    if min_angle > turn_angle
                        && min_angle < turn_angle + 45.0 - turn_angle.min(0.0)
                    {
                        inputs[hint_idx].cmd = VoiceHint::KL;
                    }
                }

                if self.explicit_roundabouts {
                    inputs[hint_idx].angle = Self::sum_non_consumed_within_catching_range(
                        &mut inputs,
                        hint_idx as isize,
                        Self::INTERNAL_CATCHING_RANGE_WIDE,
                    );
                } else {
                    inputs[hint_idx].turn_angle_consumed = true;
                }
                inputs[hint_idx].distance_to_next = distance;
                distance = 0.0;
                results.push(inputs[hint_idx].clone());
            }
            if !results.is_empty() && distance < Self::INTERNAL_CATCHING_RANGE_NEAR {
                let extra = Self::sum_non_consumed_within_catching_range(
                    &mut inputs,
                    hint_idx as isize,
                    Self::INTERNAL_CATCHING_RANGE_NEAR,
                );
                if let Some(last) = results.last_mut() {
                    last.angle += extra;
                }
            }
        }

        let mut results2: Vec<VoiceHint> = Vec::new();
        let mut i = results.len();
        while i > 0 {
            i -= 1;
            let mut hint = results[i].clone();
            if hint.cmd == 0 {
                hint.calc_command();
            }
            if hint.cmd == VoiceHint::END {
                results2.push(hint);
                continue;
            }
            if !(hint.needs_real_turn
                && (hint.cmd == VoiceHint::C || hint.cmd == VoiceHint::BL))
            {
                let mut dist = hint.distance_to_next;
                while dist < Self::INTERNAL_CATCHING_RANGE_NEAR && i > 0 {
                    let h2 = &results[i - 1];
                    dist = h2.distance_to_next;
                    hint.distance_to_next += dist;
                    hint.angle += h2.angle;
                    i -= 1;
                    if h2.is_roundabout() {
                        let mut h2_clone = h2.clone();
                        h2_clone.angle = hint.angle;
                        h2_clone.distance_to_next = hint.distance_to_next;
                        hint = h2_clone;
                        break;
                    }
                }

                if !self.explicit_roundabouts {
                    hint.roundabout_exit = 0;
                }
                hint.calc_command();
                results2.push(hint);
            } else if hint.cmd == VoiceHint::BL {
                results2.push(hint);
            } else if let Some(last) = results2.last_mut() {
                last.distance_to_next += hint.distance_to_next;
            }
        }

        results2
    }

    pub fn post_process(
        &self,
        mut inputs: Vec<VoiceHint>,
        catching_range: f64,
        min_range: f64,
    ) -> Vec<VoiceHint> {
        let mut results: Vec<VoiceHint> = Vec::new();
        let mut last_saved_idx: Option<usize> = None;
        let mut hint_idx = 0;

        while hint_idx < inputs.len() {
            let input = inputs[hint_idx].clone();
            let has_next = hint_idx + 1 < inputs.len();

            if input.cmd == VoiceHint::BL {
                results.push(input);
                hint_idx += 1;
                continue;
            }

            if !has_next {
                if input.cmd == VoiceHint::END {
                    hint_idx += 1;
                    continue;
                } else if (input.cmd == VoiceHint::C
                    || input.cmd == VoiceHint::KR
                    || input.cmd == VoiceHint::KL)
                    && !input.good_way.is_link_type()
                {
                    let last_saved = last_saved_idx.and_then(|idx| results.get(idx));
                    if Self::check_straight_hold(&input, last_saved, min_range) {
                        results.push(input);
                    } else if let Some(last) = results.last_mut() {
                        last.distance_to_next += input.distance_to_next;
                    }
                } else {
                    results.push(input);
                }
            } else {
                let last_saved_dist_ok = last_saved_idx
                    .and_then(|idx| results.get(idx))
                    .map(|ls| ls.distance_to_next > catching_range)
                    .unwrap_or(false);

                if last_saved_dist_ok || input.distance_to_next > catching_range {
                    if input.cmd == VoiceHint::C
                        || input.cmd == VoiceHint::KR
                        || input.cmd == VoiceHint::KL
                    {
                        let last_saved = last_saved_idx.and_then(|idx| results.get(idx));
                        if Self::check_straight_hold(&input, last_saved, min_range) {
                            last_saved_idx = Some(results.len());
                            results.push(input);
                        } else if let Some(idx) = last_saved_idx {
                            results[idx].distance_to_next += input.distance_to_next;
                        }
                    } else if input.good_way.prio() == 29
                        && input.max_bad_prio == 30
                        && Self::check_for_next_none_motorway(&inputs, hint_idx, 3)
                    {
                        let mut inp = input;
                        if inp.cmd == VoiceHint::KR || inp.cmd == VoiceHint::TSLR {
                            inp.cmd = VoiceHint::ER;
                        } else if inp.cmd == VoiceHint::KL || inp.cmd == VoiceHint::TSLL {
                            inp.cmd = VoiceHint::EL;
                        }
                        last_saved_idx = Some(results.len());
                        results.push(inp);
                    } else if (input.good_way.prio() != 28
                        && input.good_way.prio() != 30
                        && input.good_way.prio() != 26)
                        || input.is_roundabout()
                        || input.angle.abs() > 21.0
                        || (input.angle.abs() - input.lower_bad_way_angle) < 21.0
                    {
                        last_saved_idx = Some(results.len());
                        results.push(input);
                    } else if let Some(idx) = last_saved_idx {
                        results[idx].distance_to_next += input.distance_to_next;
                    }
                } else if input.distance_to_next < catching_range {
                    let mut inp = input;
                    let next_input = inputs[hint_idx + 1].clone();
                    let angles = inp.angle + next_input.angle;
                    let mut save = false;

                    if (inp.cmd == VoiceHint::C
                        || inp.cmd == VoiceHint::KR
                        || inp.cmd == VoiceHint::KL)
                        && !inp.good_way.is_link_type()
                    {
                        if inp.good_way.prio() < inp.max_bad_prio {
                            let last_saved = last_saved_idx.and_then(|idx| results.get(idx));
                            if let Some(ls) = last_saved {
                                if ls.cmd != VoiceHint::C
                                    && ls.distance_to_next > min_range
                                    && self.transport_mode != TransportMode::Car
                                {
                                    save = true;
                                    if next_input.cmd == VoiceHint::C
                                        && !next_input.good_way.is_link_type()
                                    {
                                        inp.distance_to_next += next_input.distance_to_next;
                                        hint_idx += 1;
                                    }
                                }
                            }
                        } else if let Some(idx) = last_saved_idx {
                            results[idx].distance_to_next += inp.distance_to_next;
                        }
                    } else if inp.good_way.prio() == 29 && inp.max_bad_prio == 30 {
                        if inp.cmd == VoiceHint::KR || inp.cmd == VoiceHint::TSLR {
                            inp.cmd = VoiceHint::ER;
                        } else if inp.cmd == VoiceHint::KL || inp.cmd == VoiceHint::TSLL {
                            inp.cmd = VoiceHint::EL;
                        }
                        save = true;
                    } else if VoiceHint::is_180_deg_angle(inp.angle) {
                        save = true;
                    } else if self.transport_mode == TransportMode::Car
                        && (angles.abs() as f64) > 180.0 - Self::SIGNIFICANT_ANGLE
                    {
                        inp.angle = angles;
                        inp.calc_command();
                        inp.distance_to_next += next_input.distance_to_next;
                        save = true;
                        hint_idx += 1;
                    } else if (angles.abs() as f64) < Self::SIGNIFICANT_ANGLE
                        && inp.distance_to_next < min_range
                    {
                        inp.angle = angles;
                        inp.calc_command();
                        inp.distance_to_next += next_input.distance_to_next;
                        save = true;
                        hint_idx += 1;
                    } else if (inp.angle.abs() as f64) > Self::SIGNIFICANT_ANGLE {
                        save = true;
                    } else if (inp.angle.abs() as f64) < Self::SIGNIFICANT_ANGLE {
                        // ignored without adding distance, matching VoiceHintProcessor.java:468
                    } else {
                        inputs[hint_idx + 1].distance_to_next += inp.distance_to_next;
                    }

                    if save {
                        last_saved_idx = Some(results.len());
                        results.push(inp);
                    }
                } else {
                    last_saved_idx = Some(results.len());
                    results.push(input);
                }
            }
            hint_idx += 1;
        }

        if let Some(last) = results.last() {
            if last.cmd == VoiceHint::END {
                results.pop();
            }
        }

        results
    }

    fn check_for_next_none_motorway(
        inputs: &[VoiceHint],
        offset: usize,
        test_size: usize,
    ) -> bool {
        for i in 1..=test_size {
            if offset + i >= inputs.len() {
                break;
            }
            let prio = inputs[offset + i].good_way.prio();
            if prio < 29 {
                return true;
            }
            if prio == 30 {
                return false;
            }
        }
        false
    }

    fn check_straight_hold(
        input: &VoiceHint,
        input_last_saved: Option<&VoiceHint>,
        min_range: f64,
    ) -> bool {
        if input.index_in_track == 0 {
            return false;
        }
        let bad_one_way = input.bad_ways.iter().any(|md| md.is_bad_oneway());
        if bad_one_way
            && input.lower_bad_way_angle == -181.0
            && input.higher_bad_way_angle == 181.0
        {
            return false;
        }
        if (input.lower_bad_way_angle != -181.0
            && input.lower_bad_way_angle.abs() > 135.0
            && input.higher_bad_way_angle.abs() > 35.0)
            || (input.higher_bad_way_angle != 181.0
                && input.higher_bad_way_angle > 135.0
                && input.lower_bad_way_angle.abs() > 35.0)
        {
            return false;
        }

        ((input.lower_bad_way_angle.abs() < 35.0 || input.higher_bad_way_angle < 35.0)
            || input.good_way.prio() < input.max_bad_prio
            || input.good_way.prio() > input.old_way.prio())
            && input_last_saved.map_or(true, |ls| ls.distance_to_next > min_range)
            && input.distance_to_next > min_range
    }
}

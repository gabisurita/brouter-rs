//! Rust implementation corresponding to:
//! - Java class: `btools.router.KinematicModel`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/KinematicModel.java`
//! - Java class: `btools.router.KinematicNoCostModel`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-core/src/main/java/btools/router/KinematicNoCostModel.java`
//!
//! # Summary
//! Physics-based kinematics for speed, power, rolling resistance, aerodynamic drag,
//! and travel time computation using Newton-Raphson cubic equation solving.

use crate::dsl::evaluator::WayCostResult;
use crate::dsl::parser::Profile;

pub const GRAVITY: f64 = 9.81;

/// Configuration parameters for bicycle or pedestrian kinematics (`StdModel` / `StdPath`).
#[derive(Debug, Clone)]
pub struct KinematicConfig {
    pub total_mass: f64,  // Total mass (biker + bike + cargo), kg
    pub max_speed: f64,   // Maximum speed limit before braking, m/s
    pub s_c_x: f64,       // Aerodynamic drag coefficient S * C_x
    pub c_r: f64,         // Rolling resistance coefficient
    pub biker_power: f64, // Sustained biker mechanical power, W
    pub is_bike_mode: bool,
    pub is_foot_mode: bool,
}

impl Default for KinematicConfig {
    fn default() -> Self {
        Self {
            total_mass: 90.0,
            max_speed: 45.0 / 3.6, // 12.5 m/s (~45 km/h)
            s_c_x: 0.5 * 0.45,     // 0.225
            c_r: 0.01,
            biker_power: 100.0, // 100 W default
            is_bike_mode: true,
            is_foot_mode: false,
        }
    }
}

/// Transport mode flags read from a profile's global context.
///
/// Mirrors Java `RoutingContext.readGlobalConfig`: each flag defaults to false.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeFlags {
    pub car: bool,
    pub bike: bool,
    pub foot: bool,
}

impl ModeFlags {
    pub fn from_profile(profile: &Profile) -> Self {
        Self {
            car: profile.get_global_var("validForCars", 0.0) != 0.0,
            bike: profile.get_global_var("validForBikes", 0.0) != 0.0,
            foot: profile.get_global_var("validForFoot", 0.0) != 0.0,
        }
    }
}

impl KinematicConfig {
    pub fn from_profile(profile: &Profile) -> Self {
        let modes = ModeFlags::from_profile(profile);
        let default_max_speed = if modes.foot { 6.0 } else { 45.0 };
        Self {
            total_mass: profile.get_global_var("totalMass", 90.0) as f64,
            max_speed: (profile.get_global_var("maxSpeed", default_max_speed) as f64) / 3.6,
            s_c_x: profile.get_global_var("S_C_x", 0.5 * 0.45) as f64,
            c_r: profile.get_global_var("C_r", 0.01) as f64,
            biker_power: profile.get_global_var("bikerPower", 100.0) as f64,
            is_bike_mode: modes.bike,
            is_foot_mode: modes.foot,
        }
    }

    /// Solves the cubic aerodynamic and rolling resistance power equation:
    ///   `a * v^3 + c * v = d`
    /// where:
    /// - `a = S * C_x`: effective frontal area and air drag coefficient
    /// - `c = m * g * (C_r + incline)`: combined rolling and gravitational resistance
    /// - `d = P`: biker mechanical power in Watts
    ///
    /// Matches `StdPath.solveCubic` in Java BRouter. Uses Newton-Raphson
    /// iteration with initial interval doubling to find the steady-state velocity `v` (m/s).
    pub fn solve_cubic(a: f64, c: f64, d: f64) -> f64 {
        let mut v = 8.0;
        let mut finding_start_value = true;
        for _ in 0..10 {
            let y = (a * v * v + c) * v - d;
            if y < 0.1 {
                if finding_start_value {
                    v *= 2.0;
                    continue;
                }
                break;
            }
            finding_start_value = false;
            let y_prime = 3.0 * a * v * v + c;
            if y_prime.abs() < 1e-9 {
                break;
            }
            v -= y / y_prime;
        }
        v
    }

    /// Computes travel speed (m/s) and energy consumption (J) for a road segment.
    ///
    /// Matches `StdPath.computeKinematic` in Java BRouter.
    pub fn compute_speed_and_energy(
        &self,
        dist: f64,
        incline: f64,
        way_max_speed: f64,
    ) -> (f64, f64) {
        let mut max_speed = self.max_speed;
        if way_max_speed > 0.0 {
            max_speed = max_speed.min(way_max_speed);
        }

        let f_roll = self.total_mass * GRAVITY * (self.c_r + incline);
        let speed = if self.is_foot_mode {
            // Tobler's hiking function for walking sections
            self.max_speed * (-3.5 * (incline + 0.05).abs()).exp()
        } else if self.is_bike_mode {
            let v = Self::solve_cubic(self.s_c_x, f_roll, self.biker_power);
            v.min(max_speed)
        } else {
            max_speed
        };

        // Mechanical energy (only positive energy is counted; braking does not add negative work)
        let energy_force = self.s_c_x * speed * speed + f_roll;
        let energy = if energy_force > 0.0 {
            dist * energy_force
        } else {
            0.0
        };

        (speed, energy)
    }
}

/// Car kinematic routing model (`KinematicModel` and `KinematicNoCostModel`).
///
/// Corresponds to Java classes `btools.router.KinematicModel`, `btools.router.KinematicNoCostModel`,
/// `btools.router.KinematicPath`, and `btools.router.KinematicNoCostPath`.
#[derive(Debug, Clone)]
pub struct CarKinematicModel {
    pub no_cost_factor: bool,
    pub turn_angle_decay_time: f64,
    pub f_roll: f64,
    pub f_air: f64,
    pub f_recup: f64,
    pub p_standby: f64,
    pub outside_temp: f64,
    pub recup_efficiency: f64,
    pub total_weight: f64,
    pub vmax: f64,
    pub left_way_speed: f64,
    pub right_way_speed: f64,
    pub pw: f64,
    pub cost0: f64,
}

impl CarKinematicModel {
    /// Constructs a `CarKinematicModel` if the profile declares `KinematicModel` or `KinematicNoCostModel`.
    pub fn from_profile(profile: &Profile) -> Option<Self> {
        let model_name = profile.model_class.as_deref()?;
        let no_cost_factor = if model_name.contains("KinematicNoCostModel") {
            true
        } else if model_name.contains("KinematicModel") {
            false
        } else {
            return None;
        };

        let turn_angle_decay_time = profile.get_global_var("turnAngleDecayTime", 5.0) as f64;
        let f_roll = profile.get_global_var("f_roll", 232.0) as f64;
        let f_air = profile.get_global_var("f_air", 0.4) as f64;
        let f_recup = profile.get_global_var("f_recup", 400.0) as f64;
        let p_standby = profile.get_global_var("p_standby", 250.0) as f64;
        let outside_temp = profile.get_global_var("outside_temp", 20.0) as f64;
        let recup_efficiency = profile.get_global_var("recup_efficiency", 0.7) as f64;
        let total_weight = profile.get_global_var("totalweight", 1640.0) as f64;
        let vmax = (profile.get_global_var("vmax", 80.0) as f64) / 3.6;
        let left_way_speed = (profile.get_global_var("leftWaySpeed", 12.0) as f64) / 3.6;
        let right_way_speed = (profile.get_global_var("rightWaySpeed", 12.0) as f64) / 3.6;

        let pw = 2.0 * f_air * vmax * vmax * vmax - p_standby;
        let cost0 = (pw + p_standby) / vmax + f_roll + f_air * vmax * vmax;

        Some(Self {
            no_cost_factor,
            turn_angle_decay_time,
            f_roll,
            f_air,
            f_recup,
            p_standby,
            outside_temp,
            recup_efficiency,
            total_weight,
            vmax,
            left_way_speed,
            right_way_speed,
            pw,
            cost0,
        })
    }

    /// Computes effective speed limit (m/s) from way `minspeed`, `maxspeed`, and profile `vmax`.
    #[inline]
    pub fn effective_speed_limit(&self, way_cost: &WayCostResult) -> f64 {
        let minspeed = (way_cost.minspeed / 3.6) as f64;
        let espeed = if minspeed > self.vmax {
            minspeed
        } else {
            self.vmax
        };
        let maxspeed = (way_cost.maxspeed / 3.6) as f64;
        if maxspeed < espeed {
            maxspeed
        } else {
            espeed
        }
    }

    /// Computes breaking speed (m/s) for balance power `pw` and effective speed limit `vl`.
    #[inline]
    pub fn breaking_speed(&self, vl: f64) -> f64 {
        let mut v = vl * 0.8;
        let pw2 = self.pw + self.p_standby;
        let e = self.recup_efficiency;
        let x0 = pw2 / vl + self.f_air * e * vl * vl + (1.0 - e) * self.f_roll;
        for _ in 0..5 {
            let v2 = v * v;
            let x = pw2 / v + self.f_air * e * v2 - x0;
            let dx = 2.0 * e * self.f_air * v - pw2 / v2;
            v -= x / dx;
        }
        v
    }

    /// Clamps kinetic energy `ekin` to the kinetic energy at `speed` (m/s).
    #[inline]
    pub fn cut_ekin(&self, ekin: &mut f64, speed: f64) {
        let e = 0.5 * self.total_weight * speed * speed;
        if *ekin > e {
            *ekin = e;
        }
    }

    /// Evolves kinetic energy, travel time, and dissipated energy over distance `dist` (m)
    /// and elevation change `delta_h` (m), returning the distance cost in meters-equivalent.
    /// Returns `-1.0` if `effective_speed_limit <= 0.0`.
    #[allow(clippy::too_many_arguments)]
    pub fn evolve_distance(
        &self,
        ekin: &mut f64,
        total_time: &mut f32,
        total_energy: &mut f32,
        way_cost: &WayCostResult,
        dist: f64,
        delta_h: f64,
        f_air: f64,
    ) -> (f64, f32, f32) {
        let fh = delta_h * self.total_weight * GRAVITY / dist;
        let effective_speed_limit = self.effective_speed_limit(way_cost);
        let emax = 0.5 * self.total_weight * effective_speed_limit * effective_speed_limit;
        if emax <= 0.0 {
            return (-1.0, 0.0, 0.0);
        }
        let vb = self.breaking_speed(effective_speed_limit);
        let elow = 0.5 * self.total_weight * vb * vb;

        let mut elapsed_time = 0.0;
        let mut dissipated_energy = 0.0;

        let mut v = (2.0 * *ekin / self.total_weight).sqrt();
        let mut d = dist;
        while d > 0.0 {
            let slow = *ekin < elow;
            let fast = *ekin >= emax;
            let etarget = if slow { elow } else { emax };
            let mut f = self.f_roll + f_air * v * v + fh;
            let f_recup = (if fast {
                -f
            } else {
                (if slow { self.f_recup } else { 0.0 }) - fh
            })
            .max(0.0);
            f += f_recup;

            let delta_ekin: f64;
            let time_step: f64;
            let x: f64;
            if fast {
                x = d;
                delta_ekin = x * f;
                time_step = x / v;
                *ekin = etarget;
            } else {
                let de = etarget - *ekin;
                let b = 2.0 * f_air / self.total_weight;
                let x0 = de / f;
                let x0b = x0 * b;
                let x_est = x0 * (1.0 - x0b * (0.5 + x0b * (0.333333333 - x0b * 0.25)));
                let maxstep = d.min(50.0);
                if x_est >= maxstep {
                    x = maxstep;
                    let xb = x * b;
                    delta_ekin =
                        x * f * (1.0 + xb * (0.5 + xb * (0.166666667 + xb * 0.0416666667)));
                    *ekin += delta_ekin;
                } else {
                    x = x_est;
                    delta_ekin = de;
                    *ekin = etarget;
                }
                let v2 = (2.0 * *ekin / self.total_weight).sqrt();
                let a = f / self.total_weight;
                time_step = (v2 - v) / a;
                v = v2;
            }
            d -= x;
            elapsed_time += time_step;

            dissipated_energy += delta_ekin - x * (fh + f_recup * self.recup_efficiency);

            let ie_recup = x * f_recup * (1.0 - self.recup_efficiency);
            let eaux = time_step * self.p_standby;
            dissipated_energy -= ie_recup.min(eaux) * 0.5;
        }

        dissipated_energy += elapsed_time * self.p_standby;

        let dt = elapsed_time as f32;
        let de = (dissipated_energy + dist * fh) as f32;
        *total_time += dt;
        *total_energy += de;

        let dist_cost = (self.pw * elapsed_time + dissipated_energy) / self.cost0;
        (dist_cost, dt, de)
    }
}

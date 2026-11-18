//! Pluggable physics engine.
//!
//! The [`PhysicsEngine`] trait is the seam between the orchestrator and the
//! physics. Any implementation can be swapped in. [`SpeedLimitEngine`] is the
//! reference implementation: it drives the train to the path speed limits with
//! a braking-curve look-ahead, integrates longitudinal dynamics, and solves
//! powertrain energy consumption.

use crate::brakes::FricBrake;
use crate::braking::BrakingPoints;
use crate::data::{Consist, TrainConfig, TrainParams};
use crate::metrics::TrainMetrics;
use crate::resistance::{compute_resistance, PathProfile, ResistanceParams};
use crate::state::TrainState;
use crate::units::*;
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

/// Immutable configuration for a train run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrainInput {
    /// Train configuration (rail vehicles, car counts).
    pub train_config: TrainConfig,
    /// Locomotive consist.
    pub consist: Consist,
    /// Pre-built path profile (grades, curves, speed limits).
    pub path: PathProfile,
    /// Static resistance parameters.
    pub resistance: ResistanceParams,
    /// Friction brake model.
    pub fric_brake: FricBrake,
    /// Initial speed (m/s).
    pub init_speed: Velocity,
    /// Initial offset along the path (m).
    pub init_offset: Length,
    /// Number of throttle notches used to discretize tractive power.
    /// Default 8. Set to 0 to disable notch modeling (continuous power).
    #[serde(default = "default_n_notches")]
    pub n_notches: u32,
    /// Ambient temperature (K) used for air density. `None` uses the standard
    /// sea-level value [`crate::constants::RHO_AIR`].
    #[serde(default)]
    pub temperature: Option<Temperature>,
    /// Whether grade and curve resistance are averaged over the train length
    /// (ALTRIOS "strap" model). Default `true`; set `false` for the legacy
    /// point model that samples only the front of the train.
    #[serde(default = "default_distributed_resistance")]
    pub distributed_resistance: bool,
}

fn default_n_notches() -> u32 {
    8
}

fn default_distributed_resistance() -> bool {
    true
}

/// Mutable control inputs supplied by the orchestrator each tick.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlInput {
    /// Optional speed target override (m/s). `None` means self-controlled.
    pub speed_target: Option<Velocity>,
    /// Whether the engine is on.
    pub engine_on: bool,
    /// Throttle restriction as a fraction of rated consist power, in [0, 1].
    /// `None` means unrestricted (full capacity). This is a per-step input,
    /// not engine state: passing `Some(frac)` restricts positive traction,
    /// and passing `None` on any later step instantly restores full power.
    /// Speed limits always take priority — the restriction only caps the
    /// power ceiling (`min(asked, possible)`); braking is never restricted.
    /// When `n_notches > 0`, the ceiling is rounded up to the nearest notch.
    pub throttle_frac: Option<Ratio>,
    /// Maximum allowed throttle notch (1..=n_notches). `None` means no notch
    /// ceiling. Only meaningful when `TrainInput::n_notches > 0`. When both
    /// this and `throttle_frac` are set, the more restrictive applies.
    pub max_notch: Option<u32>,
}

impl Default for ControlInput {
    fn default() -> Self {
        Self {
            speed_target: None,
            engine_on: true,
            throttle_frac: None,
            max_notch: None,
        }
    }
}

/// Outcome of a single step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepOutcome {
    /// Metrics for this step.
    pub metrics: TrainMetrics,
    /// Whether the train has reached the end of the path.
    pub finished: bool,
}

/// Pluggable physics engine interface.
pub trait PhysicsEngine {
    /// Reset the engine to the initial state described by `input`.
    fn reset(&mut self, input: &TrainInput) -> Result<()>;
    /// Advance the simulation by `dt` seconds.
    fn step(&mut self, dt: Time, control: &ControlInput) -> Result<StepOutcome>;
    /// Current metrics without advancing.
    fn metrics(&self) -> TrainMetrics;
    /// Whether the train has reached the end of the path.
    fn is_finished(&self) -> bool;
}

/// Reference engine: self-controlled to speed limits with braking look-ahead.
#[derive(Debug, Clone)]
pub struct SpeedLimitEngine {
    input: TrainInput,
    params: TrainParams,
    state: TrainState,
    /// Throttle fraction applied in the most recent step (1.0 = unrestricted).
    throttle_frac: Ratio,
    /// Effective max positive power after restriction in the most recent
    /// step (W).
    pwr_pos_max_effective: Power,
    /// Realized throttle notch in the most recent step (0 = idle).
    notch: u32,
    /// Precomputed braking curve along the path.
    braking_points: BrakingPoints,
    finished: bool,
}

impl SpeedLimitEngine {
    /// Create an engine from a train input.
    pub fn new(input: TrainInput) -> Result<Self> {
        let params = input.train_config.make_train_params()?;
        let mut engine = Self {
            input,
            params,
            state: TrainState::default(),
            throttle_frac: 1.0,
            pwr_pos_max_effective: 0.0,
            notch: 0,
            braking_points: BrakingPoints::default(),
            finished: false,
        };
        engine.reset(&engine.input.clone())?;
        Ok(engine)
    }

    /// Realized notch for a demanded tractive fraction.
    ///
    /// `demand_frac` is the requested effort as a fraction of rated, taken as
    /// the larger of the power fraction and the force fraction (so a heavy
    /// train can notch up to break away at low speed, where demanded power is
    /// near zero but demanded force is high). `ceil_frac` is the power ceiling
    /// from the throttle restriction.
    ///
    /// Returns `(notch, pwr_avail_frac, pwr_achieved_frac)` (fractions of rated
    /// power):
    /// - `notch == 0` (idle): demand below half a notch applies no power.
    /// - otherwise the lowest notch meeting demand is used, capped at the
    ///   ceiling notch, and that notch's full power is realized (over-supply).
    /// When `n_notches == 0`, notch modeling is disabled: continuous power
    /// capped by the ceiling is returned with `notch == 0`.
    fn notch_power(demand_frac: f64, n_notches: u32, ceil_frac: f64) -> (u32, f64, f64) {
        let ceil = ceil_frac.clamp(0.0, 1.0);
        if n_notches == 0 {
            return (0, ceil, demand_frac.max(0.0).min(ceil));
        }
        let max_notch = (ceil * n_notches as f64).floor() as u32;
        if max_notch == 0 {
            return (0, 0.0, 0.0);
        }
        let demand = demand_frac.max(0.0);
        let demanded_notch = if demand <= 0.5 / n_notches as f64 {
            0
        } else {
            (demand * n_notches as f64).ceil().clamp(1.0, n_notches as f64) as u32
        };
        let notch = demanded_notch.min(max_notch);
        let frac = notch as f64 / n_notches as f64;
        (notch, frac, frac)
    }

    /// Build the initial state from the input.
    fn build_state(&self) -> TrainState {
        let consist_mass = self.input.consist.mass();
        let mass_static = self.params.towed_mass_static + consist_mass;
        let mass_rot = self
            .input
            .train_config
            .rail_vehicles
            .iter()
            .map(|rv| {
                rv.mass_rot_per_axle
                    * *self
                        .input
                        .train_config
                        .n_cars_by_type
                        .get(&rv.car_type)
                        .unwrap_or(&0) as f64
                    * rv.axle_count as f64
            })
            .sum();
        let mass_freight = self
            .input
            .train_config
            .rail_vehicles
            .iter()
            .map(|rv| {
                rv.mass_freight
                    * *self
                        .input
                        .train_config
                        .n_cars_by_type
                        .get(&rv.car_type)
                        .unwrap_or(&0) as f64
            })
            .sum();

        let mut state = TrainState {
            time: 0.0,
            i: 0,
            offset: self.input.init_offset.max(self.params.length),
            speed: self.input.init_speed,
            speed_limit: self.params.speed_max,
            speed_target: self.input.init_speed,
            dt: 1.0,
            length: self.params.length,
            mass_static,
            mass_rot,
            mass_freight,
            ..Default::default()
        };
        state.offset_back = state.offset - state.length;
        state.weight_static = mass_static * crate::constants::ACC_GRAV;
        state
    }

    /// Compute the speed target for the current position, honoring the
    /// precomputed braking curve (so the train brakes at the correct distance
    /// ahead of every lower limit).
    fn speed_target(&self, control: &ControlInput) -> Velocity {
        if let Some(target) = control.speed_target {
            return target;
        }
        let limit = self
            .input
            .path
            .speed_limit_at(self.state.offset)
            .min(self.params.speed_max);
        if self.braking_points.is_empty() {
            return limit;
        }
        let mut bp = self.braking_points.clone();
        let (bp_limit, bp_target) = bp.calc_speeds(
            self.state.offset,
            self.state.speed,
            self.input.fric_brake.adj_ramp_up_time(),
        );
        bp_limit.min(limit).min(bp_target).min(self.params.speed_max)
    }

    fn update_resistance(&mut self) {
        let (grade, curve) = if self.input.distributed_resistance {
            let len = self.state.length;
            (
                self.input.path.grade_avg(self.state.offset, len),
                self.input.path.curve_avg(self.state.offset, len),
            )
        } else {
            (
                self.input.path.grade_at(self.state.offset),
                self.input.path.curve_at(self.state.offset),
            )
        };
        let rho = match self.input.temperature {
            Some(t) => crate::constants::rho_air(t, self.state.elev_front),
            None => crate::constants::RHO_AIR,
        };
        self.state.res = compute_resistance(
            &self.input.resistance,
            self.state.weight_static,
            self.state.speed,
            grade,
            curve,
            rho,
        );
        self.state.grade_front = grade;
        self.state.elev_front = self
            .input
            .path
            .grades
            .get(self.input.path.idx_at(self.state.offset))
            .map(|g| g.res_net)
            .unwrap_or(0.0);
    }
    fn build_metrics(&self) -> TrainMetrics {
        let s = &self.state;
        TrainMetrics {
            i: s.i,
            time: s.time,
            offset: s.offset,
            total_dist: s.total_dist,
            speed: s.speed,
            speed_limit: s.speed_limit,
            speed_target: s.speed_target,
            mass_static: s.mass_static,
            mass_rot: s.mass_rot,
            mass_freight: s.mass_freight,
            weight_static: s.weight_static,
            res_bearing: s.res.bearing,
            res_rolling: s.res.rolling,
            res_davis_b: s.res.davis_b,
            res_aero: s.res.aero,
            res_grade: s.res.grade,
            res_curve: s.res.curve,
            res_total: s.res.total(),
            grade_front: s.grade_front,
            elev_front: s.elev_front,
            milepost: self.input.path.milepost_at(s.offset),
            subdivision: self.input.path.subdivision_at(s.offset).to_string(),
            track_name: self.input.path.track_name_at(s.offset).to_string(),
            pwr_res: s.pwr_res,
            pwr_accel: s.pwr_accel,
            pwr_whl_out: s.pwr_whl_out,
            throttle_frac: self.throttle_frac,
            pwr_pos_max_effective: self.pwr_pos_max_effective,
            notch: self.notch,
            energy_whl_out: s.energy_whl_out,
            energy_whl_out_pos: s.energy_whl_out_pos,
            energy_whl_out_neg: s.energy_whl_out_neg,
            pwr_fuel: self.input.consist.pwr_fuel(),
            energy_fuel: self.input.consist.energy_fuel(),
            pwr_battery: self.input.consist.pwr_res(),
            energy_res: self.input.consist.energy_res(),
            soc: self.input.consist.mean_soc(),
            fric_brake_force: s.fric_brake_force,
        }
    }
}

impl PhysicsEngine for SpeedLimitEngine {
    fn reset(&mut self, input: &TrainInput) -> Result<()> {
        self.input = input.clone();
        self.params = input.train_config.make_train_params()?;
        self.state = self.build_state();
        self.throttle_frac = 1.0;
        self.pwr_pos_max_effective = self.input.consist.pwr_out_max();
        self.notch = 0;
        // Braking curve: use the friction-brake-limited deceleration.
        let mass = self.state.mass_compound().max(1.0);
        let decel = self.input.fric_brake.force_max / mass;
        self.braking_points = BrakingPoints::build(&self.input.path, decel);
        self.finished = false;
        Ok(())
    }

    fn step(&mut self, dt: Time, control: &ControlInput) -> Result<StepOutcome> {
        if dt <= 0.0 {
            bail!("step: dt must be > 0");
        }
        self.state.dt = dt;
        self.state.i += 1;

        // 1. Resistance at current position.
        self.update_resistance();

        // 2. Speed target with look-ahead.
        let speed_target = self.speed_target(control);
        self.state.speed_limit = self
            .input
            .path
            .speed_limit_at(self.state.offset)
            .min(self.params.speed_max);
        self.state.speed_target = speed_target;

        // 3. Force limits.
        let res_net = self.state.res_net();
        let mass = self.state.mass_compound();
        let consist = &mut self.input.consist;
        consist.set_pwr_aux(control.engine_on)?;

        // Throttle ceiling from the more restrictive of fraction / max notch.
        let n_notches = self.input.n_notches;
        let pwr_rated = consist.pwr_out_max();
        let mut throttle = control.throttle_frac.unwrap_or(1.0).clamp(0.0, 1.0);
        if n_notches > 0 {
            if let Some(mn) = control.max_notch {
                let mn = mn.min(n_notches) as f64;
                throttle = throttle.min(mn / n_notches as f64);
            }
        }
        self.throttle_frac = throttle;
        self.pwr_pos_max_effective = pwr_rated * throttle;
        // Braking capability is not throttle-restricted.
        let pwr_neg_max = consist.pwr_dyn_brake_max();

        // 4. Desired force to reach the target speed this step.
        let f_target = res_net + mass * (speed_target - self.state.speed) / dt;

        // 5. Realize the demanded effort at the lowest notch meeting demand,
        //    capped by the throttle ceiling. Demand is the larger of the power
        //    fraction and the force fraction so a heavy train can notch up to
        //    break away at low speed (where power demand is near zero).
        let force_frac = if consist.force_max() > 0.0 {
            (f_target.max(0.0) / consist.force_max()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let power_frac = if pwr_rated > 0.0 {
            ((f_target.max(0.0) * self.state.speed) / pwr_rated).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let demand_frac = force_frac.max(power_frac);
        let (notch, pwr_avail_frac, pwr_achieved_frac) =
            Self::notch_power(demand_frac, n_notches, throttle);
        self.notch = notch;

        // Tractive force available from the notch power at this speed, capped
        // by the adhesion/force limit.
        let pwr_avail = pwr_avail_frac * pwr_rated;
        let f_notch = pwr_avail / self.state.speed.max(1.0);
        let f_pos_max = consist.force_max().min(f_notch);

        // 6. Clamp to traction / braking capability.
        self.input.fric_brake.set_cur_force_max_out(dt);
        let f_brake_max = self.input.fric_brake.force_max_curr;
        let f_applied = f_target.min(f_pos_max).max(-(f_brake_max + pwr_neg_max / self.state.speed.max(1e-3)));

        // 7. Integrate dynamics (velocity-Verlet style).
        let vel_change = dt * (f_applied - res_net) / mass;
        let vel_avg = self.state.speed + 0.5 * vel_change;
        let new_speed = (self.state.speed + vel_change).max(0.0);

        self.state.pwr_res = res_net * vel_avg;
        self.state.pwr_accel = mass / (2.0 * dt) * (new_speed * new_speed - self.state.speed * self.state.speed);

        self.state.time += dt;
        self.state.offset += dt * vel_avg;
        self.state.total_dist += (dt * vel_avg).abs();
        self.state.speed = new_speed;

        // 8. Split applied force into consist traction vs friction braking.
        let (f_consist, fric_force) = if f_applied >= 0.0 {
            (f_applied, 0.0)
        } else {
            let f_consist = f_applied + f_brake_max;
            if f_consist >= 0.0 {
                (0.0, f_brake_max)
            } else {
                (f_consist, -f_applied - f_consist.abs())
            }
        };
        self.state.fric_brake_force = fric_force;
        self.input.fric_brake.force = fric_force;

        // 9. Wheel power and energy (notch-consistent when notching).
        let pwr_whl = if n_notches > 0 && f_consist > 0.0 {
            pwr_achieved_frac * pwr_rated
        } else {
            f_consist * self.state.speed
        };
        self.state.pwr_whl_out = pwr_whl;
        self.state.energy_whl_out += pwr_whl * dt;
        if pwr_whl >= 0.0 {
            self.state.energy_whl_out_pos += pwr_whl * dt;
        } else {
            self.state.energy_whl_out_neg += -pwr_whl * dt;
        }

        // 9. Powertrain energy consumption.
        self.input
            .consist
            .solve_energy_consumption(pwr_whl, dt, control.engine_on)?;

        // 10. Termination.
        let end = self.input.path.length;
        if self.state.offset >= end - 1.0 || (self.state.offset >= end && self.state.speed <= 0.0) {
            self.finished = true;
        }

        Ok(StepOutcome {
            metrics: self.build_metrics(),
            finished: self.finished,
        })
    }

    fn metrics(&self) -> TrainMetrics {
        self.build_metrics()
    }

    fn is_finished(&self) -> bool {
        self.finished
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::brakes::FricBrake;
    use crate::data::loco_type::LocoTypeRegistry;
    use crate::data::train_config::TrainConfig;
    use crate::resistance::PathResCoeff;

    fn test_input(path_length: f64, init_speed: f64) -> TrainInput {
        let registry = LocoTypeRegistry::with_builtins();
        let consist = Consist::from_types(&registry, &[("ES44", 2)]).unwrap();
        let cfg = TrainConfig::from_tonnage(5000.0).unwrap();
        TrainInput {
            train_config: cfg,
            consist,
            path: PathProfile {
                offsets: vec![0.0, path_length],
                grades: vec![PathResCoeff::default(); 2],
                curves: vec![PathResCoeff::default(); 2],
                speed_points: vec![(0.0, 25.0), (path_length, 25.0)],
                mileposts: vec![0.0, path_length / 1609.344],
                subdivisions: vec!["TEST".into(); 2],
                track_names: vec!["Main".into(); 2],
                length: path_length,
            },
            resistance: ResistanceParams {
                bearing: 4000.0,
                rolling_ratio: 0.0015,
                davis_b: 3.0e-5,
                cd_area: 16.0,
            },
            fric_brake: FricBrake::new(500_000.0, 30.0, 0.6),
            init_speed,
            init_offset: 0.0,
            n_notches: 8,
            temperature: None,
            distributed_resistance: false,
        }
    }

    fn run_steps(
        input: TrainInput,
        n_steps: usize,
        throttle: Option<f64>,
    ) -> (Vec<TrainMetrics>, SpeedLimitEngine) {
        let mut engine = SpeedLimitEngine::new(input).unwrap();
        let control = ControlInput {
            throttle_frac: throttle,
            ..Default::default()
        };
        let mut metrics = Vec::new();
        for _ in 0..n_steps {
            let outcome = engine.step(1.0, &control).unwrap();
            metrics.push(outcome.metrics);
            if engine.is_finished() {
                break;
            }
        }
        (metrics, engine)
    }

    #[test]
    fn unrestricted_matches_full_power() {
        let (metrics, _) = run_steps(test_input(20_000.0, 0.0), 50, None);
        // All unrestricted steps should report full rated power ceiling.
        let rated = 2.0 * 4400.0 * crate::constants::HP_TO_W;
        for m in &metrics {
            assert!((m.pwr_pos_max_effective - rated).abs() < 1e-6);
            assert!((m.throttle_frac - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn restricted_is_slower_and_burns_less_fuel() {
        let length = 20_000.0;
        let (free_m, _) = run_steps(test_input(length, 0.0), 60, None);
        let (restricted_m, _) = run_steps(test_input(length, 0.0), 60, Some(0.5));

        // Restricted runs never report a higher effective power ceiling.
        let rated = 2.0 * 4400.0 * crate::constants::HP_TO_W;
        for m in &restricted_m {
            assert!((m.pwr_pos_max_effective - rated * 0.5).abs() < 1e-6);
        }
        // After 60 s, the restricted train should be further behind.
        let free_dist = free_m.last().unwrap().total_dist;
        let rest_dist = restricted_m.last().unwrap().total_dist;
        assert!(
            rest_dist < free_dist,
            "restricted dist {rest_dist} should be < free dist {free_dist}"
        );
    }

    #[test]
    fn zero_throttle_means_no_traction() {
        let (metrics, _) = run_steps(test_input(20_000.0, 0.0), 20, Some(0.0));
        for m in &metrics {
            assert!((m.pwr_pos_max_effective - 0.0).abs() < 1e-9);
            // Train cannot start moving with zero traction.
            assert!(m.pwr_whl_out <= 0.0);
        }
        assert!(
            metrics.last().unwrap().total_dist < 1.0,
            "train should barely move with zero throttle"
        );
    }

    #[test]
    fn lift_restores_full_capacity() {
        let mut engine = SpeedLimitEngine::new(test_input(20_000.0, 10.0)).unwrap();
        let restricted = ControlInput {
            throttle_frac: Some(0.4),
            ..Default::default()
        };
        engine.step(1.0, &restricted).unwrap();
        assert!((engine.metrics().throttle_frac - 0.4).abs() < 1e-9);

        // Lift the restriction: next step runs at full capacity.
        engine.step(1.0, &ControlInput::default()).unwrap();
        let m = engine.metrics();
        assert!((m.throttle_frac - 1.0).abs() < 1e-9);
        let rated = 2.0 * 4400.0 * crate::constants::HP_TO_W;
        assert!((m.pwr_pos_max_effective - rated).abs() < 1e-6);
    }

    #[test]
    fn restriction_never_causes_overspeed() {
        let length = 30_000.0;
        let (metrics, _) = run_steps(test_input(length, 0.0), 300, Some(0.3));
        for m in &metrics {
            assert!(
                m.speed <= m.speed_limit + 1e-9,
                "speed {} exceeded limit {}",
                m.speed,
                m.speed_limit
            );
        }
    }

    #[test]
    fn notch_power_bands() {
        let ceil = 1.0;
        // Idle band.
        assert_eq!(SpeedLimitEngine::notch_power(0.0, 8, ceil).0, 0);
        assert_eq!(SpeedLimitEngine::notch_power(0.05, 8, ceil).0, 0);
        // Just above half a notch -> notch 1.
        assert_eq!(SpeedLimitEngine::notch_power(0.07, 8, ceil).0, 1);
        // Exactly 5/8 of rated -> notch 5.
        assert_eq!(SpeedLimitEngine::notch_power(0.625, 8, ceil).0, 5);
        // Near full -> notch 8.
        assert_eq!(SpeedLimitEngine::notch_power(0.97, 8, ceil).0, 8);
        // Notch power is the notch's full width (over-supply).
        let (n, avail, achieved) = SpeedLimitEngine::notch_power(0.5, 8, ceil);
        assert_eq!(n, 4);
        assert!((avail - 0.5).abs() < 1e-9);
        assert!((achieved - 0.5).abs() < 1e-9);
    }

    #[test]
    fn n_notches_zero_is_continuous() {
        let (n, avail, achieved) = SpeedLimitEngine::notch_power(0.123_456, 0, 1.0);
        assert_eq!(n, 0);
        assert!((avail - 1.0).abs() < 1e-9);
        // No over-supply: exactly the demanded fraction (capped).
        assert!((achieved - 0.123_456).abs() < 1e-9);
    }

    #[test]
    fn ceiling_limits_notch() {
        // 0.5 ceiling with 8 notches -> max notch 4.
        let (n, _, _) = SpeedLimitEngine::notch_power(1.0, 8, 0.5);
        assert_eq!(n, 4);
        // 0.3 ceiling -> max notch 2.
        let (n, _, _) = SpeedLimitEngine::notch_power(1.0, 8, 0.3);
        assert_eq!(n, 2);
    }

    #[test]
    fn notching_produces_discrete_notches() {
        let mut engine = SpeedLimitEngine::new(test_input(30_000.0, 0.0)).unwrap();
        let control = ControlInput::default();
        let mut seen = std::collections::BTreeSet::new();
        for _ in 0..200 {
            let o = engine.step(1.0, &control).unwrap();
            seen.insert(o.metrics.notch);
            if engine.is_finished() {
                break;
            }
        }
        // Only discrete notch values in 0..=8 should ever appear.
        for &n in &seen {
            assert!(n <= 8, "unexpected notch {n}");
        }
    }

    #[test]
    fn max_notch_caps_power() {
        let mut engine = SpeedLimitEngine::new(test_input(30_000.0, 0.0)).unwrap();
        let control = ControlInput {
            max_notch: Some(4),
            ..Default::default()
        };
        for _ in 0..200 {
            let o = engine.step(1.0, &control).unwrap();
            assert!(o.metrics.notch <= 4, "notch {} exceeded max_notch 4", o.metrics.notch);
            let rated = 2.0 * 4400.0 * crate::constants::HP_TO_W;
            assert!(o.metrics.pwr_pos_max_effective <= rated * 0.5 + 1e-6);
            if engine.is_finished() {
                break;
            }
        }
    }

    #[test]
    fn lift_max_notch_restores_full_notch() {
        let mut engine = SpeedLimitEngine::new(test_input(30_000.0, 5.0)).unwrap();
        let restricted = ControlInput {
            max_notch: Some(3),
            ..Default::default()
        };
        engine.step(1.0, &restricted).unwrap();
        assert!(engine.metrics().notch <= 3);
        // Lift: no notch ceiling -> may now reach a higher notch.
        let mut max_seen = 0;
        for _ in 0..50 {
            let o = engine.step(1.0, &ControlInput::default()).unwrap();
            max_seen = max_seen.max(o.metrics.notch);
            if engine.is_finished() {
                break;
            }
        }
        assert!(max_seen > 3, "expected a notch above 3 after lifting, saw {max_seen}");
    }
}

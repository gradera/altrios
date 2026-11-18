//! Per-step metrics emitted by the physics engine.

use crate::units::*;
use serde::{Deserialize, Serialize};

/// All metrics produced at the end of a single time step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TrainMetrics {
    /// Step index.
    pub i: usize,
    /// Simulation time (s).
    pub time: Time,
    /// Front offset along path (m).
    pub offset: Length,
    /// Cumulative distance (m).
    pub total_dist: Length,
    /// Current speed (m/s).
    pub speed: Velocity,
    /// Speed limit (m/s).
    pub speed_limit: Velocity,
    /// Speed target (m/s).
    pub speed_target: Velocity,
    /// Static mass (kg).
    pub mass_static: Mass,
    /// Rotational mass (kg).
    pub mass_rot: Mass,
    /// Freight mass (kg).
    pub mass_freight: Mass,
    /// Static weight (N).
    pub weight_static: Force,

    /// Bearing resistance (N).
    pub res_bearing: Force,
    /// Rolling resistance (N).
    pub res_rolling: Force,
    /// Davis B resistance (N).
    pub res_davis_b: Force,
    /// Aerodynamic resistance (N).
    pub res_aero: Force,
    /// Grade resistance (N).
    pub res_grade: Force,
    /// Curve resistance (N).
    pub res_curve: Force,
    /// Total resistance (N).
    pub res_total: Force,

    /// Grade at front.
    pub grade_front: Ratio,
    /// Elevation at front (m).
    pub elev_front: Length,

    /// Milepost at the front of the train (miles).
    pub milepost: f64,
    /// Subdivision under the train.
    pub subdivision: String,
    /// Track name under the train.
    pub track_name: String,

    /// Resistance power (W).
    pub pwr_res: Power,
    /// Inertial power (W).
    pub pwr_accel: Power,
    /// Wheel power (W).
    pub pwr_whl_out: Power,
    /// Applied throttle fraction (1.0 = unrestricted).
    pub throttle_frac: Ratio,
    /// Effective maximum positive power after throttle restriction (W).
    pub pwr_pos_max_effective: Power,
    /// Realized throttle notch (0 = idle; 0 when notch modeling is disabled).
    pub notch: u32,
    /// Cumulative wheel energy (J).
    pub energy_whl_out: Energy,
    /// Cumulative positive wheel energy (J).
    pub energy_whl_out_pos: Energy,
    /// Cumulative negative wheel energy (J).
    pub energy_whl_out_neg: Energy,

    /// Consist fuel power (W).
    pub pwr_fuel: Power,
    /// Consist cumulative fuel energy (J).
    pub energy_fuel: Energy,
    /// Consist battery power (W).
    pub pwr_battery: Power,
    /// Consist cumulative battery chemical energy out (J).
    pub energy_res: Energy,
    /// Mean battery state of charge, if equipped.
    pub soc: Option<Ratio>,

    /// Applied friction brake force (N).
    pub fric_brake_force: Force,
}

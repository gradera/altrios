//! Mutable train state carried forward across time steps.

use crate::resistance::ResistanceForces;
use crate::units::*;
use serde::{Deserialize, Serialize};

/// Full mutable state of a train simulation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TrainState {
    /// Time since datum (s).
    pub time: Time,
    /// Step index.
    pub i: usize,
    /// Front-of-train offset along the path (m).
    pub offset: Length,
    /// Back-of-train offset along the path (m).
    pub offset_back: Length,
    /// Cumulative absolute distance traveled (m).
    pub total_dist: Length,
    /// Current speed (m/s).
    pub speed: Velocity,
    /// Current speed limit (m/s).
    pub speed_limit: Velocity,
    /// Current speed target (m/s).
    pub speed_target: Velocity,
    /// Time step size (s).
    pub dt: Time,
    /// Train length (m).
    pub length: Length,
    /// Static mass including freight and locomotives (kg).
    pub mass_static: Mass,
    /// Effective rotational mass (kg).
    pub mass_rot: Mass,
    /// Freight mass (kg).
    pub mass_freight: Mass,
    /// Static weight (N).
    pub weight_static: Force,
    /// Decomposed resistance forces (N).
    pub res: ResistanceForces,
    /// Grade at front of train.
    pub grade_front: Ratio,
    /// Elevation at front of train (m).
    pub elev_front: Length,
    /// Power to overcome resistance (W).
    pub pwr_res: Power,
    /// Power to overcome inertia (W).
    pub pwr_accel: Power,
    /// Total tractive power at the wheel (W).
    pub pwr_whl_out: Power,
    /// Cumulative wheel energy (J).
    pub energy_whl_out: Energy,
    /// Cumulative positive-traction wheel energy (J).
    pub energy_whl_out_pos: Energy,
    /// Cumulative negative-traction wheel energy (J).
    pub energy_whl_out_neg: Energy,
    /// Applied friction brake force (N).
    pub fric_brake_force: Force,
}

impl TrainState {
    /// Total mass including rotational mass (kg).
    pub fn mass_compound(&self) -> Mass {
        self.mass_static + self.mass_rot
    }

    /// Net resistance force (N).
    pub fn res_net(&self) -> Force {
        self.res.total()
    }
}

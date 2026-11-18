//! Friction brakes with ramped application.

use crate::units::*;
use serde::{Deserialize, Serialize};

/// Friction brake model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FricBrake {
    /// Maximum braking force (N).
    pub force_max: Force,
    /// Time to ramp from zero to max force (s).
    pub ramp_up_time: Time,
    /// Ramp-up coefficient used for look-ahead correction.
    pub ramp_up_coeff: Ratio,
    /// Current applied force (N).
    #[serde(default)]
    pub force: Force,
    /// Current maximum force given ramp-up (N).
    #[serde(default)]
    pub force_max_curr: Force,
}

impl FricBrake {
    /// Create a friction brake model.
    pub fn new(force_max: Force, ramp_up_time: Time, ramp_up_coeff: Ratio) -> Self {
        Self {
            force_max,
            ramp_up_time,
            ramp_up_coeff,
            force: 0.0,
            force_max_curr: 0.0,
        }
    }

    /// Update the ramped maximum force for the current step.
    pub fn set_cur_force_max_out(&mut self, dt: Time) {
        let ramp = if self.ramp_up_time > 0.0 {
            self.force_max / self.ramp_up_time * dt
        } else {
            self.force_max
        };
        self.force_max_curr = (self.force + ramp).min(self.force_max);
    }

    /// Effective look-ahead time accounting for approximately linear build-up.
    pub fn adj_ramp_up_time(&self) -> Time {
        self.ramp_up_time * self.ramp_up_coeff
    }
}

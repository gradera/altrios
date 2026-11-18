//! Rail vehicle (freight car) physical properties.

use crate::units::*;
use serde::{Deserialize, Serialize};

/// Physical properties of a rail vehicle type.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RailVehicle {
    /// Unique identifier for the car type (e.g. `loaded`, `empty`).
    pub car_type: String,
    /// Freight type carried (e.g. `Intermodal`).
    pub freight_type: String,
    /// Railcar length between pulling faces (m).
    pub length: Length,
    /// Axle count (typically 4).
    pub axle_count: u8,
    /// Brake valve count (typically 1).
    pub brake_count: u8,
    /// Railcar mass excluding freight (kg).
    pub mass_static_base: Mass,
    /// Freight component of total static mass (kg).
    pub mass_freight: Mass,
    /// Railcar speed limit (m/s).
    pub speed_max: Velocity,
    /// Braking force per rail vehicle weight (dimensionless).
    pub braking_ratio: Ratio,
    /// Additional rotating mass per axle (kg).
    pub mass_rot_per_axle: Mass,
    /// Bearing resistance per axle (N).
    pub bearing_res_per_axle: Force,
    /// Rolling resistance force per weight (dimensionless).
    pub rolling_ratio: Ratio,
    /// Davis B coefficient (s/m).
    pub davis_b: InverseVelocity,
    /// Drag area Cd*A (m^2).
    pub cd_area: Area,
    /// Curve coefficient 0.
    pub curve_coeff_0: Ratio,
    /// Curve coefficient 1.
    pub curve_coeff_1: Ratio,
    /// Curve coefficient 2.
    pub curve_coeff_2: Ratio,
}

impl RailVehicle {
    /// Static mass of the rail vehicle, excluding freight.
    pub fn mass_static(&self) -> Mass {
        self.mass_static_base
    }

    /// Total static mass including freight.
    pub fn mass(&self) -> Mass {
        self.mass_static_base + self.mass_freight
    }
}

//! Locomotive model with pluggable powertrain types.

use super::powertrain::{
    ElectricDrivetrain, FuelConverter, Generator, ReversibleEnergyStorage,
};
use crate::constants::ACC_GRAV;
use crate::units::*;
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

/// Powertrain variant of a locomotive.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PowertrainType {
    /// Diesel-electric: engine -> generator -> drivetrain.
    Conventional {
        /// Fuel converter.
        fc: FuelConverter,
        /// Generator.
        gen: Generator,
        /// Electric drivetrain.
        edrv: ElectricDrivetrain,
    },
    /// Battery-electric: battery -> drivetrain.
    BatteryElectric {
        /// Reversible energy storage.
        res: ReversibleEnergyStorage,
        /// Electric drivetrain.
        edrv: ElectricDrivetrain,
    },
}

/// A single locomotive.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Locomotive {
    /// Locomotive mass (kg).
    pub mass: Mass,
    /// Maximum tractive effort (N).
    pub force_max: Force,
    /// Auxiliary power offset (W).
    pub pwr_aux_offset: Power,
    /// Auxiliary power traction coefficient (W/W).
    pub pwr_aux_traction_coeff: Ratio,
    /// Powertrain.
    pub powertrain: PowertrainType,
    /// Current state.
    #[serde(default)]
    pub state: LocomotiveState,
}

/// Mutable state of a [`Locomotive`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct LocomotiveState {
    /// Achieved wheel power (W).
    pub pwr_out: Power,
    /// Auxiliary load (W).
    pub pwr_aux: Power,
    /// Cumulative wheel energy (J).
    pub energy_out: Energy,
    /// Cumulative aux energy (J).
    pub energy_aux: Energy,
}

impl Locomotive {
    /// Maximum tractive effort (N).
    pub fn force_max(&self) -> Force {
        self.force_max
    }

    /// Maximum propulsive power (W).
    pub fn pwr_out_max(&self) -> Power {
        match &self.powertrain {
            PowertrainType::Conventional { fc, .. } => fc.pwr_out_max,
            PowertrainType::BatteryElectric { edrv, .. } => edrv.pwr_out_max,
        }
    }

    /// Maximum dynamic braking power (W).
    pub fn pwr_dyn_brake_max(&self) -> Power {
        match &self.powertrain {
            PowertrainType::Conventional { edrv, .. } => edrv.pwr_out_max,
            PowertrainType::BatteryElectric { edrv, .. } => edrv.pwr_out_max,
        }
    }

    /// Adhesion-limited force check helper: `mu * mass * g`.
    pub fn adhesion_force(&self, mu: Ratio) -> Force {
        mu * self.mass * ACC_GRAV
    }

    /// Set auxiliary power based on current wheel power.
    pub fn set_pwr_aux(&mut self, loco_on: bool) -> Result<()> {
        self.state.pwr_aux = if loco_on {
            self.pwr_aux_offset + self.pwr_aux_traction_coeff * self.state.pwr_out.abs()
        } else {
            0.0
        };
        Ok(())
    }

    /// Solve energy consumption for a required wheel power over `dt`.
    pub fn solve_energy_consumption(
        &mut self,
        pwr_out_req: Power,
        dt: Time,
        engine_on: bool,
    ) -> Result<()> {
        self.state.pwr_out = pwr_out_req;
        match &mut self.powertrain {
            PowertrainType::Conventional { fc, gen, edrv } => {
                edrv.set_pwr_in_req(pwr_out_req, dt)?;
                gen.set_pwr_in_req(edrv.state.pwr_elec_prop_in, self.state.pwr_aux, engine_on, dt)?;
                fc.solve_energy_consumption(gen.state.pwr_mech_in, dt, engine_on)?;
            }
            PowertrainType::BatteryElectric { res, edrv } => {
                edrv.set_pwr_in_req(pwr_out_req, dt)?;
                res.solve(edrv.state.pwr_elec_prop_in + self.state.pwr_aux, dt)?;
            }
        }
        self.state.energy_out += pwr_out_req * dt;
        self.state.energy_aux += self.state.pwr_aux * dt;
        Ok(())
    }

    /// Cumulative fuel energy consumed (J). Zero for battery-electric.
    pub fn energy_fuel(&self) -> Energy {
        match &self.powertrain {
            PowertrainType::Conventional { fc, .. } => fc.state.energy_fuel,
            PowertrainType::BatteryElectric { .. } => 0.0,
        }
    }

    /// Current fuel power (W). Zero for battery-electric.
    pub fn pwr_fuel(&self) -> Power {
        match &self.powertrain {
            PowertrainType::Conventional { fc, .. } => fc.state.pwr_fuel,
            PowertrainType::BatteryElectric { .. } => 0.0,
        }
    }

    /// Battery state of charge, if equipped.
    pub fn soc(&self) -> Option<Ratio> {
        match &self.powertrain {
            PowertrainType::BatteryElectric { res, .. } => Some(res.state.soc),
            PowertrainType::Conventional { .. } => None,
        }
    }

    /// Cumulative battery chemical energy out (J), if equipped.
    pub fn energy_res(&self) -> Energy {
        match &self.powertrain {
            PowertrainType::BatteryElectric { res, .. } => res.state.energy_out_chemical,
            PowertrainType::Conventional { .. } => 0.0,
        }
    }

    /// Validate that the powertrain is internally consistent.
    pub fn validate(&self) -> Result<()> {
        if self.mass <= 0.0 {
            bail!("Locomotive mass must be > 0");
        }
        if self.force_max <= 0.0 {
            bail!("Locomotive force_max must be > 0");
        }
        Ok(())
    }
}

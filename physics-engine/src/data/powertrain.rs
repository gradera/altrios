//! Powertrain components: fuel converter (engine), generator, electric
//! drivetrain, and reversible energy storage (battery).
//!
//! Power flows backwards from the wheel: `ElectricDrivetrain` -> `Generator`
//! -> `FuelConverter` for a conventional locomotive, or `ElectricDrivetrain`
//! -> `ReversibleEnergyStorage` for a battery-electric locomotive. Each stage
//! applies an efficiency looked up from a power-fraction table and records
//! losses and cumulative energy.

use crate::interp::interp1d;
use crate::units::*;
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

/// Fuel converter (diesel engine or fuel cell).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FuelConverter {
    /// Maximum rated brake output power (W).
    pub pwr_out_max: Power,
    /// Starting/baseline transient power limit (W).
    pub pwr_out_max_init: Power,
    /// Lag time for ramp up (s).
    pub pwr_ramp_lag: Time,
    /// Power fractions at which efficiencies are evaluated.
    pub pwr_out_frac_interp: Vec<f64>,
    /// Efficiency at each power fraction.
    pub eta_interp: Vec<f64>,
    /// Idle fuel power to overcome internal friction (W).
    pub pwr_idle_fuel: Power,
    /// Current state.
    #[serde(default)]
    pub state: FuelConverterState,
}

/// Mutable state of a [`FuelConverter`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct FuelConverterState {
    /// Current max power (W).
    pub pwr_out_max: Power,
    /// Efficiency at current demand.
    pub eta: Ratio,
    /// Shaft power to generator (W).
    pub pwr_shaft: Power,
    /// Fuel power consumed (W).
    pub pwr_fuel: Power,
    /// Loss power including idle (W).
    pub pwr_loss: Power,
    /// Idle fuel power (W).
    pub pwr_idle_fuel: Power,
    /// Cumulative shaft energy (J).
    pub energy_shaft: Energy,
    /// Cumulative fuel energy (J).
    pub energy_fuel: Energy,
    /// Cumulative loss energy (J).
    pub energy_loss: Energy,
    /// Cumulative idle fuel energy (J).
    pub energy_idle_fuel: Energy,
    /// Whether the engine is on.
    pub engine_on: bool,
}

impl FuelConverter {
    /// Solve fuel consumption for a required shaft power over `dt`.
    pub fn solve_energy_consumption(
        &mut self,
        pwr_out_req: Power,
        dt: Time,
        engine_on: bool,
    ) -> Result<()> {
        if dt <= 0.0 {
            bail!("FuelConverter: dt must be > 0");
        }
        let pwr_out_req = pwr_out_req.max(0.0);
        self.state.pwr_shaft = pwr_out_req;
        self.state.engine_on = engine_on;

        let frac = if self.pwr_out_max > 0.0 {
            (pwr_out_req / self.pwr_out_max).clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.state.eta = interp1d(frac, &self.pwr_out_frac_interp, &self.eta_interp)?;
        if self.state.eta <= 0.0 {
            bail!("FuelConverter: efficiency must be > 0");
        }

        self.state.pwr_idle_fuel = if engine_on { self.pwr_idle_fuel } else { 0.0 };
        self.state.pwr_fuel = if engine_on {
            pwr_out_req / self.state.eta + self.state.pwr_idle_fuel
        } else {
            0.0
        };
        self.state.pwr_loss = self.state.pwr_fuel - self.state.pwr_shaft;

        self.state.energy_shaft += self.state.pwr_shaft * dt;
        self.state.energy_fuel += self.state.pwr_fuel * dt;
        self.state.energy_loss += self.state.pwr_loss * dt;
        self.state.energy_idle_fuel += self.state.pwr_idle_fuel * dt;
        Ok(())
    }
}

/// Generator / alternator.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Generator {
    /// Maximum electrical output power (W).
    pub pwr_out_max: Power,
    /// Power fractions at which efficiencies are evaluated.
    pub pwr_out_frac_interp: Vec<f64>,
    /// Efficiency at each power fraction.
    pub eta_interp: Vec<f64>,
    /// Current state.
    #[serde(default)]
    pub state: GeneratorState,
}

/// Mutable state of a [`Generator`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct GeneratorState {
    /// Efficiency at current demand.
    pub eta: Ratio,
    /// Mechanical power input (W).
    pub pwr_mech_in: Power,
    /// Electrical power output to propulsion (W).
    pub pwr_elec_prop_out: Power,
    /// Electrical power output to aux loads (W).
    pub pwr_elec_aux: Power,
    /// Loss power (W).
    pub pwr_loss: Power,
    /// Cumulative mechanical energy in (J).
    pub energy_mech_in: Energy,
    /// Cumulative electrical energy out (J).
    pub energy_elec_prop_out: Energy,
    /// Cumulative aux energy (J).
    pub energy_elec_aux: Energy,
    /// Cumulative loss energy (J).
    pub energy_loss: Energy,
}

impl Generator {
    /// Solve generator input given required electrical propulsion power.
    pub fn set_pwr_in_req(
        &mut self,
        pwr_prop_req: Power,
        pwr_aux: Power,
        engine_on: bool,
        dt: Time,
    ) -> Result<()> {
        if pwr_prop_req < 0.0 {
            bail!("Generator: propulsion power must be >= 0");
        }
        let pwr_elec_out = pwr_prop_req + pwr_aux;
        let frac = if self.pwr_out_max > 0.0 {
            (pwr_elec_out / self.pwr_out_max).clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.state.eta = interp1d(frac, &self.pwr_out_frac_interp, &self.eta_interp)?;
        if self.state.eta <= 0.0 {
            bail!("Generator: efficiency must be > 0");
        }

        self.state.pwr_elec_prop_out = pwr_prop_req;
        self.state.pwr_elec_aux = pwr_aux;
        self.state.pwr_mech_in = if engine_on {
            pwr_elec_out / self.state.eta
        } else {
            0.0
        };
        self.state.pwr_loss = self.state.pwr_mech_in - pwr_elec_out;

        self.state.energy_mech_in += self.state.pwr_mech_in * dt;
        self.state.energy_elec_prop_out += self.state.pwr_elec_prop_out * dt;
        self.state.energy_elec_aux += self.state.pwr_elec_aux * dt;
        self.state.energy_loss += self.state.pwr_loss * dt;
        Ok(())
    }
}

/// Electric drivetrain (motor/inverter/gearing).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ElectricDrivetrain {
    /// Maximum mechanical output power (W).
    pub pwr_out_max: Power,
    /// Power fractions at which efficiencies are evaluated.
    pub pwr_out_frac_interp: Vec<f64>,
    /// Efficiency at each power fraction.
    pub eta_interp: Vec<f64>,
    /// Current state.
    #[serde(default)]
    pub state: ElectricDrivetrainState,
}

/// Mutable state of an [`ElectricDrivetrain`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ElectricDrivetrainState {
    /// Efficiency at current demand.
    pub eta: Ratio,
    /// Electrical power to propulsion (W); negative indicates regen.
    pub pwr_elec_prop_in: Power,
    /// Mechanical power to propulsion (W); negative indicates regen.
    pub pwr_mech_prop_out: Power,
    /// Mechanical power from dynamic braking (W).
    pub pwr_mech_dyn_brake: Power,
    /// Loss power (W).
    pub pwr_loss: Power,
    /// Cumulative electrical energy in (J).
    pub energy_elec_prop_in: Energy,
    /// Cumulative mechanical energy out (J).
    pub energy_mech_prop_out: Energy,
    /// Cumulative dynamic-brake mechanical energy (J).
    pub energy_mech_dyn_brake: Energy,
    /// Cumulative loss energy (J).
    pub energy_loss: Energy,
}

impl ElectricDrivetrain {
    /// Solve drivetrain electrical input given required mechanical output.
    pub fn set_pwr_in_req(&mut self, pwr_out_req: Power, dt: Time) -> Result<()> {
        let frac = if self.pwr_out_max > 0.0 {
            (pwr_out_req.abs() / self.pwr_out_max).clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.state.eta = interp1d(frac, &self.pwr_out_frac_interp, &self.eta_interp)?;
        if self.state.eta <= 0.0 {
            bail!("ElectricDrivetrain: efficiency must be > 0");
        }

        self.state.pwr_mech_prop_out = pwr_out_req;
        self.state.pwr_mech_dyn_brake = 0.0;
        self.state.pwr_elec_prop_in = if pwr_out_req >= 0.0 {
            pwr_out_req / self.state.eta
        } else {
            pwr_out_req * self.state.eta
        };
        self.state.pwr_loss = (self.state.pwr_elec_prop_in - pwr_out_req).abs();

        self.state.energy_elec_prop_in += self.state.pwr_elec_prop_in * dt;
        self.state.energy_mech_prop_out += self.state.pwr_mech_prop_out * dt;
        self.state.energy_loss += self.state.pwr_loss * dt;
        Ok(())
    }
}

/// Reversible energy storage (battery).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReversibleEnergyStorage {
    /// Usable energy capacity (J).
    pub energy_capacity: Energy,
    /// Minimum state of charge.
    pub min_soc: Ratio,
    /// Maximum state of charge.
    pub max_soc: Ratio,
    /// Maximum discharge power (W).
    pub pwr_disch_max: Power,
    /// Maximum charge power (W).
    pub pwr_charge_max: Power,
    /// Power fractions at which efficiencies are evaluated.
    pub pwr_out_frac_interp: Vec<f64>,
    /// Efficiency at each power fraction.
    pub eta_interp: Vec<f64>,
    /// Current state.
    #[serde(default)]
    pub state: ReversibleEnergyStorageState,
}

/// Mutable state of a [`ReversibleEnergyStorage`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReversibleEnergyStorageState {
    /// State of charge.
    pub soc: Ratio,
    /// Efficiency at current demand.
    pub eta: Ratio,
    /// Electrical power out (W); positive is discharge.
    pub pwr_out_electrical: Power,
    /// Chemical power out (W); positive is discharge.
    pub pwr_out_chemical: Power,
    /// Loss power (W).
    pub pwr_loss: Power,
    /// Cumulative electrical energy out (J).
    pub energy_out_electrical: Energy,
    /// Cumulative chemical energy out (J).
    pub energy_out_chemical: Energy,
    /// Cumulative loss energy (J).
    pub energy_loss: Energy,
}

impl Default for ReversibleEnergyStorageState {
    fn default() -> Self {
        Self {
            soc: 0.95,
            eta: 1.0,
            pwr_out_electrical: 0.0,
            pwr_out_chemical: 0.0,
            pwr_loss: 0.0,
            energy_out_electrical: 0.0,
            energy_out_chemical: 0.0,
            energy_loss: 0.0,
        }
    }
}

impl ReversibleEnergyStorage {
    /// Solve battery power flow for a required electrical power over `dt`.
    /// Positive `pwr_out_req` discharges; negative charges.
    pub fn solve(&mut self, pwr_out_req: Power, dt: Time) -> Result<()> {
        if dt <= 0.0 {
            bail!("ReversibleEnergyStorage: dt must be > 0");
        }
        let frac = if self.pwr_disch_max > 0.0 {
            (pwr_out_req.abs() / self.pwr_disch_max).clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.state.eta = interp1d(frac, &self.pwr_out_frac_interp, &self.eta_interp)?;
        if self.state.eta <= 0.0 {
            bail!("ReversibleEnergyStorage: efficiency must be > 0");
        }

        self.state.pwr_out_electrical = pwr_out_req;
        self.state.pwr_out_chemical = if pwr_out_req >= 0.0 {
            pwr_out_req / self.state.eta
        } else {
            pwr_out_req * self.state.eta
        };
        self.state.pwr_loss = (self.state.pwr_out_chemical - pwr_out_req).abs();

        if self.energy_capacity > 0.0 {
            self.state.soc -= self.state.pwr_out_chemical * dt / self.energy_capacity;
            self.state.soc = self.state.soc.clamp(self.min_soc, self.max_soc);
        }

        self.state.energy_out_electrical += self.state.pwr_out_electrical * dt;
        self.state.energy_out_chemical += self.state.pwr_out_chemical * dt;
        self.state.energy_loss += self.state.pwr_loss * dt;
        Ok(())
    }
}

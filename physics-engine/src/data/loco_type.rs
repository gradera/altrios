//! Locomotive type catalog and horsepower-based construction.
//!
//! Freight railroads describe locomotives by type name and prime-mover
//! horsepower. [`LocoType`] captures one type's parameters; [`LocoTypeRegistry`]
//! holds a named set of types (built-ins plus any user-registered ones). A
//! consist can then be built by name and count, with HP supplied per type.

use super::locomotive::{Locomotive, PowertrainType};
use super::powertrain::{
    ElectricDrivetrain, FuelConverter, Generator, ReversibleEnergyStorage,
};
use crate::constants::{ACC_GRAV, HP_TO_W};
use crate::units::*;
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Powertrain category of a locomotive type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum LocoKind {
    /// Diesel-electric (engine -> generator -> drivetrain).
    #[default]
    Conventional,
    /// Battery-electric (battery -> drivetrain).
    BatteryElectric,
}

/// A named locomotive type: physical + powertrain parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LocoType {
    /// Type name, e.g. `ES44`.
    pub name: String,
    /// Powertrain category.
    pub kind: LocoKind,
    /// Prime-mover / rated horsepower.
    pub hp: f64,
    /// Locomotive mass (kg). If `None`, derived as ~90 kg per hp.
    #[serde(default)]
    pub mass_kg: Option<Mass>,
    /// Adhesion coefficient for tractive effort. Default 0.30.
    #[serde(default = "default_mu")]
    pub mu: Ratio,
    /// Auxiliary power offset (W). Default 8_000.
    #[serde(default = "default_pwr_aux_offset")]
    pub pwr_aux_offset_w: Power,
    /// Auxiliary power traction coefficient (W/W). Default 5e-4.
    #[serde(default = "default_pwr_aux_coeff")]
    pub pwr_aux_traction_coeff: Ratio,
    /// Power fractions for efficiency lookup. Default [0, 0.5, 1].
    #[serde(default = "default_eta_fracs")]
    pub eta_fracs: Vec<f64>,
    /// Efficiency values at the fractions. Default [0.20, 0.35, 0.38].
    #[serde(default = "default_eta_vals")]
    pub eta_vals: Vec<f64>,
    /// Battery-electric only: usable energy capacity (J).
    /// Default: 1 hour at rated power.
    #[serde(default)]
    pub energy_capacity_j: Option<Energy>,
}

fn default_mu() -> Ratio {
    0.30
}
fn default_pwr_aux_offset() -> Power {
    8_000.0
}
fn default_pwr_aux_coeff() -> Ratio {
    5.0e-4
}
fn default_eta_fracs() -> Vec<f64> {
    vec![0.0, 0.5, 1.0]
}
fn default_eta_vals() -> Vec<f64> {
    vec![0.20, 0.35, 0.38]
}

impl LocoType {
    /// Create a conventional locomotive type with defaults for aux/efficiency.
    pub fn conventional(name: &str, hp: f64, mass_kg: Mass) -> Self {
        Self {
            name: name.to_string(),
            kind: LocoKind::Conventional,
            hp,
            mass_kg: Some(mass_kg),
            mu: default_mu(),
            pwr_aux_offset_w: default_pwr_aux_offset(),
            pwr_aux_traction_coeff: default_pwr_aux_coeff(),
            eta_fracs: default_eta_fracs(),
            eta_vals: default_eta_vals(),
            energy_capacity_j: None,
        }
    }

    /// Create a battery-electric locomotive type with defaults.
    pub fn battery_electric(name: &str, hp: f64, mass_kg: Mass, energy_capacity_j: Energy) -> Self {
        Self {
            name: name.to_string(),
            kind: LocoKind::BatteryElectric,
            hp,
            mass_kg: Some(mass_kg),
            mu: default_mu(),
            pwr_aux_offset_w: default_pwr_aux_offset(),
            pwr_aux_traction_coeff: default_pwr_aux_coeff(),
            eta_fracs: default_eta_fracs(),
            eta_vals: default_eta_vals(),
            energy_capacity_j: Some(energy_capacity_j),
        }
    }

    /// Resolve the effective horsepower (override wins).
    fn resolved_hp(&self, hp_override: Option<f64>) -> f64 {
        hp_override.unwrap_or(self.hp)
    }

    /// Resolve the mass: explicit, or ~90 kg per hp as a fallback.
    fn resolved_mass(&self, hp: f64) -> Result<Mass> {
        match self.mass_kg {
            Some(m) if m > 0.0 => Ok(m),
            Some(m) => bail!("LocoType `{}` has non-positive mass {m}", self.name),
            None => Ok(90.0 * hp),
        }
    }

    /// Instantiate one [`Locomotive`] of this type, optionally overriding HP.
    pub fn to_locomotive(&self, hp_override: Option<f64>) -> Result<Locomotive> {
        if hp_override.is_some_and(|h| h <= 0.0) {
            bail!("LocoType `{}`: horsepower override must be > 0", self.name);
        }
        let hp = self.resolved_hp(hp_override);
        if hp <= 0.0 {
            bail!("LocoType `{}`: horsepower must be > 0", self.name);
        }
        let mass = self.resolved_mass(hp)?;
        let pwr_out_max = hp * HP_TO_W;

        let powertrain = match self.kind {
            LocoKind::Conventional => PowertrainType::Conventional {
                fc: FuelConverter {
                    pwr_out_max,
                    pwr_out_max_init: pwr_out_max,
                    pwr_ramp_lag: 10.0,
                    pwr_out_frac_interp: self.eta_fracs.clone(),
                    eta_interp: self.eta_vals.clone(),
                    pwr_idle_fuel: 0.02 * pwr_out_max,
                    state: Default::default(),
                },
                gen: Generator {
                    pwr_out_max: 0.95 * pwr_out_max,
                    pwr_out_frac_interp: self.eta_fracs.clone(),
                    // Generator + drivetrain share the table; electric stages
                    // are far more efficient than the prime mover.
                    eta_interp: vec![0.9, 0.92, 0.9],
                    state: Default::default(),
                },
                edrv: ElectricDrivetrain {
                    pwr_out_max: 0.9 * pwr_out_max,
                    pwr_out_frac_interp: self.eta_fracs.clone(),
                    eta_interp: vec![0.9, 0.92, 0.9],
                    state: Default::default(),
                },
            },
            LocoKind::BatteryElectric => PowertrainType::BatteryElectric {
                res: ReversibleEnergyStorage {
                    energy_capacity: self.energy_capacity_j.unwrap_or(pwr_out_max * 3600.0),
                    min_soc: 0.05,
                    max_soc: 1.0,
                    pwr_disch_max: pwr_out_max,
                    pwr_charge_max: pwr_out_max,
                    pwr_out_frac_interp: self.eta_fracs.clone(),
                    eta_interp: vec![0.95, 0.97, 0.95],
                    state: Default::default(),
                },
                edrv: ElectricDrivetrain {
                    pwr_out_max,
                    pwr_out_frac_interp: self.eta_fracs.clone(),
                    eta_interp: vec![0.9, 0.92, 0.9],
                    state: Default::default(),
                },
            },
        };

        Ok(Locomotive {
            mass,
            force_max: self.mu * mass * ACC_GRAV,
            pwr_aux_offset: self.pwr_aux_offset_w,
            pwr_aux_traction_coeff: self.pwr_aux_traction_coeff,
            powertrain,
            state: Default::default(),
        })
    }
}

/// Registry of named locomotive types.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct LocoTypeRegistry {
    types: HashMap<String, LocoType>,
}

impl LocoTypeRegistry {
    /// Registry with common US freight types built in.
    pub fn with_builtins() -> Self {
        let mut reg = Self::default();
        reg.add(LocoType::conventional("ES44", 4400.0, 194_600.0));
        reg.add(LocoType::conventional("AC4400", 4400.0, 196_000.0));
        reg.add(LocoType::conventional("SD70", 4300.0, 188_000.0));
        reg.add(LocoType::conventional("GP38", 2000.0, 123_000.0));
        reg.add(LocoType::battery_electric(
            "BEL44",
            4400.0,
            220_000.0,
            4400.0 * HP_TO_W * 3600.0,
        ));
        reg
    }

    /// Register (or replace) a type. Name is the key.
    pub fn add(&mut self, loco_type: LocoType) {
        self.types.insert(loco_type.name.clone(), loco_type);
    }

    /// Look up a type by name.
    pub fn get(&self, name: &str) -> Option<&LocoType> {
        self.types.get(name)
    }

    /// Number of registered types.
    pub fn len(&self) -> usize {
        self.types.len()
    }

    /// Whether the registry is empty.
    pub fn is_empty(&self) -> bool {
        self.types.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtins_present() {
        let reg = LocoTypeRegistry::with_builtins();
        assert!(reg.get("ES44").is_some());
        assert!(reg.get("AC4400").is_some());
        assert!(reg.get("SD70").is_some());
        assert!(reg.get("GP38").is_some());
        assert!(reg.get("BEL44").is_some());
        assert_eq!(reg.len(), 5);
    }

    #[test]
    fn es44_locomotive_values() {
        let reg = LocoTypeRegistry::with_builtins();
        let loco = reg.get("ES44").unwrap().to_locomotive(None).unwrap();
        let expected_pwr = 4400.0 * HP_TO_W;
        let expected_force = 0.30 * 194_600.0 * ACC_GRAV;
        assert!((loco.force_max() - expected_force).abs() < 1e-6);
        match &loco.powertrain {
            PowertrainType::Conventional { fc, .. } => {
                assert!((fc.pwr_out_max - expected_pwr).abs() < 1e-6);
            }
            other => panic!("expected conventional powertrain, got {other:?}"),
        }
    }

    #[test]
    fn hp_override_respected() {
        let reg = LocoTypeRegistry::with_builtins();
        let loco = reg.get("ES44").unwrap().to_locomotive(Some(4500.0)).unwrap();
        match &loco.powertrain {
            PowertrainType::Conventional { fc, .. } => {
                assert!((fc.pwr_out_max - 4500.0 * HP_TO_W).abs() < 1e-6);
            }
            other => panic!("expected conventional powertrain, got {other:?}"),
        }
    }

    #[test]
    fn mass_fallback_when_none() {
        let mut t = LocoType::conventional("TEST", 1000.0, 100_000.0);
        t.mass_kg = None;
        let loco = t.to_locomotive(None).unwrap();
        assert!((loco.mass - 90.0 * 1000.0).abs() < 1e-9);
    }

    #[test]
    fn reject_bad_hp() {
        let reg = LocoTypeRegistry::with_builtins();
        assert!(reg.get("ES44").unwrap().to_locomotive(Some(0.0)).is_err());
        assert!(reg.get("ES44").unwrap().to_locomotive(Some(-5.0)).is_err());
    }
}

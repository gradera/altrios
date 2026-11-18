//! Locomotive consist: a collection of locomotives.

use super::loco_type::LocoTypeRegistry;
use super::locomotive::Locomotive;
use crate::units::*;
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

/// A consist of one or more locomotives.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Consist {
    /// Locomotives in the consist.
    pub loco_vec: Vec<Locomotive>,
}

impl Consist {
    /// Create a consist from a vector of locomotives.
    pub fn new(loco_vec: Vec<Locomotive>) -> Self {
        Self { loco_vec }
    }

    /// Build a consist from named types and counts.
    ///
    /// # Arguments
    /// - `registry`: catalog of [`super::loco_type::LocoType`]s.
    /// - `spec`: slice of `(type_name, count)` pairs.
    ///
    /// # Examples
    /// ```ignore
    /// let reg = LocoTypeRegistry::with_builtins();
    /// let consist = Consist::from_types(&reg, &[("ES44", 2), ("SD70", 1)])?;
    /// ```
    pub fn from_types(registry: &LocoTypeRegistry, spec: &[(&str, u32)]) -> Result<Self> {
        let hp_spec: Vec<(&str, u32, Option<f64>)> =
            spec.iter().map(|&(n, c)| (n, c, None)).collect();
        Self::from_types_hp_opt(registry, &hp_spec)
    }

    /// Build a consist from named types, counts, and per-entry HP overrides.
    ///
    /// # Arguments
    /// - `registry`: catalog of [`super::loco_type::LocoType`]s.
    /// - `spec`: slice of `(type_name, count, hp_override)` pairs.
    pub fn from_types_hp(
        registry: &LocoTypeRegistry,
        spec: &[(&str, u32, f64)],
    ) -> Result<Self> {
        let hp_spec: Vec<(&str, u32, Option<f64>)> =
            spec.iter().map(|&(n, c, hp)| (n, c, Some(hp))).collect();
        Self::from_types_hp_opt(registry, &hp_spec)
    }

    /// Internal expansion shared by both builders.
    fn from_types_hp_opt(
        registry: &LocoTypeRegistry,
        spec: &[(&str, u32, Option<f64>)],
    ) -> Result<Self> {
        let mut loco_vec = Vec::new();
        for &(name, count, hp_override) in spec {
            if count == 0 {
                continue;
            }
            let loco_type = registry
                .get(name)
                .ok_or_else(|| anyhow::anyhow!("locomotive type `{name}` not found in registry"))?;
            for _ in 0..count {
                loco_vec.push(loco_type.to_locomotive(hp_override)?);
            }
        }
        if loco_vec.is_empty() {
            bail!("consist spec produced no locomotives");
        }
        Ok(Self { loco_vec })
    }

    /// Total consist mass (kg).
    pub fn mass(&self) -> Mass {
        self.loco_vec.iter().map(|l| l.mass).sum()
    }

    /// Total maximum tractive effort (N).
    pub fn force_max(&self) -> Force {
        self.loco_vec.iter().map(|l| l.force_max()).sum()
    }

    /// Total maximum propulsive power (W).
    pub fn pwr_out_max(&self) -> Power {
        self.loco_vec.iter().map(|l| l.pwr_out_max()).sum()
    }

    /// Total maximum dynamic braking power (W).
    pub fn pwr_dyn_brake_max(&self) -> Power {
        self.loco_vec.iter().map(|l| l.pwr_dyn_brake_max()).sum()
    }

    /// Set auxiliary power for all locomotives.
    pub fn set_pwr_aux(&mut self, loco_on: bool) -> Result<()> {
        for loco in &mut self.loco_vec {
            loco.set_pwr_aux(loco_on)?;
        }
        Ok(())
    }

    /// Solve energy consumption for a required total wheel power over `dt`.
    ///
    /// The required power is distributed equally across locomotives.
    pub fn solve_energy_consumption(
        &mut self,
        pwr_out_req: Power,
        dt: Time,
        engine_on: bool,
    ) -> Result<()> {
        let n = self.loco_vec.len();
        if n == 0 {
            return Ok(());
        }
        let per_loco = pwr_out_req / n as f64;
        for loco in &mut self.loco_vec {
            loco.solve_energy_consumption(per_loco, dt, engine_on)?;
        }
        Ok(())
    }

    /// Total cumulative fuel energy consumed (J).
    pub fn energy_fuel(&self) -> Energy {
        self.loco_vec.iter().map(|l| l.energy_fuel()).sum()
    }

    /// Total current fuel power (W).
    pub fn pwr_fuel(&self) -> Power {
        self.loco_vec.iter().map(|l| l.pwr_fuel()).sum()
    }

    /// Total cumulative battery chemical energy out (J).
    pub fn energy_res(&self) -> Energy {
        self.loco_vec.iter().map(|l| l.energy_res()).sum()
    }

    /// Total current battery power (W).
    pub fn pwr_res(&self) -> Power {
        self.loco_vec
            .iter()
            .map(|l| match &l.powertrain {
                super::locomotive::PowertrainType::BatteryElectric { res, .. } => {
                    res.state.pwr_out_chemical
                }
                super::locomotive::PowertrainType::Conventional { .. } => 0.0,
            })
            .sum()
    }

    /// Mean battery state of charge across equipped locomotives, if any.
    pub fn mean_soc(&self) -> Option<Ratio> {
        let socs: Vec<Ratio> = self.loco_vec.iter().filter_map(|l| l.soc()).collect();
        if socs.is_empty() {
            None
        } else {
            Some(socs.iter().sum::<Ratio>() / socs.len() as f64)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Consist;
    use super::super::loco_type::LocoTypeRegistry;
    use crate::constants::{ACC_GRAV, HP_TO_W};

    #[test]
    fn from_types_builds_correctly() {
        let reg = LocoTypeRegistry::with_builtins();
        let consist = Consist::from_types(&reg, &[("ES44", 2), ("SD70", 1)]).unwrap();
        assert_eq!(consist.loco_vec.len(), 3);
        let expected_pwr = (2.0 * 4400.0 + 4300.0) * HP_TO_W;
        assert!((consist.pwr_out_max() - expected_pwr).abs() < 1e-6);
        let expected_mass = 2.0 * 194_600.0 + 188_000.0;
        assert!((consist.mass() - expected_mass).abs() < 1e-6);
        let expected_force = 0.30 * expected_mass * ACC_GRAV;
        assert!((consist.force_max() - expected_force).abs() < 1e-6);
    }

    #[test]
    fn from_types_hp_override() {
        let reg = LocoTypeRegistry::with_builtins();
        let consist = Consist::from_types_hp(&reg, &[("ES44", 2, 4500.0)]).unwrap();
        assert_eq!(consist.loco_vec.len(), 2);
        let expected_pwr = 2.0 * 4500.0 * HP_TO_W;
        assert!((consist.pwr_out_max() - expected_pwr).abs() < 1e-6);
    }

    #[test]
    fn unknown_type_errors() {
        let reg = LocoTypeRegistry::with_builtins();
        assert!(Consist::from_types(&reg, &[("NOPE", 1)]).is_err());
    }

    #[test]
    fn zero_count_skipped_and_empty_errors() {
        let reg = LocoTypeRegistry::with_builtins();
        // All-zero spec -> error.
        assert!(Consist::from_types(&reg, &[("ES44", 0)]).is_err());
        // Zero count mixed with valid -> skipped.
        let consist = Consist::from_types(&reg, &[("ES44", 0), ("GP38", 1)]).unwrap();
        assert_eq!(consist.loco_vec.len(), 1);
    }

    #[test]
    fn battery_electric_from_registry() {
        let reg = LocoTypeRegistry::with_builtins();
        let consist = Consist::from_types(&reg, &[("BEL44", 2)]).unwrap();
        assert_eq!(consist.loco_vec.len(), 2);
        assert!(consist.mean_soc().is_some());
        assert!((consist.energy_fuel() - 0.0).abs() < 1e-9);
    }
}

//! Train configuration and derived train parameters.

use super::rail_vehicle::RailVehicle;
use crate::constants::KG_PER_SHORT_TON;
use crate::units::*;
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Train type used to select speed sets on links.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum TrainType {
    /// Freight train.
    #[default]
    Freight,
    /// Passenger train.
    Passenger,
}

/// A representative car used to expand a total-tonnage figure into a train.
///
/// When only tonnage is known, the train is modeled as N of these nominal cars
/// whose aggregate gross mass equals the tonnage. Every field can be overridden
/// via [`NominalCar::new`] or direct construction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NominalCar {
    /// Gross (loaded) mass per car in short tons. Default 100.
    pub gross_tons: f64,
    /// Tare (empty) mass per car in kg. Default 30_000.
    pub tare_mass_kg: Mass,
    /// Car length in m. Default 15.0.
    pub length_m: Length,
    /// Axles per car. Default 4.
    pub axles: u8,
    /// Brake valves per car. Default 1.
    pub brake_count: u8,
    /// Rotating mass per axle in kg. Default 680.
    pub mass_rot_per_axle_kg: Mass,
    /// Bearing resistance per axle in N. Default 200.
    pub bearing_res_per_axle_n: Force,
    /// Rolling resistance ratio. Default 0.0015.
    pub rolling_ratio: Ratio,
    /// Davis B coefficient (s/m). Default 3e-5.
    pub davis_b: InverseVelocity,
    /// Drag area Cd*A in m^2. Default 8.
    pub cd_area_m2: Area,
    /// Max speed in m/s. Default 30.
    pub speed_max_mps: Velocity,
    /// Braking ratio. Default 0.1.
    pub braking_ratio: Ratio,
}

impl Default for NominalCar {
    fn default() -> Self {
        Self {
            gross_tons: 100.0,
            tare_mass_kg: 30_000.0,
            length_m: 15.0,
            axles: 4,
            brake_count: 1,
            mass_rot_per_axle_kg: 680.0,
            bearing_res_per_axle_n: 200.0,
            rolling_ratio: 0.0015,
            davis_b: 3.0e-5,
            cd_area_m2: 8.0,
            speed_max_mps: 30.0,
            braking_ratio: 0.1,
        }
    }
}

impl NominalCar {
    /// Create a nominal car with default resistance values.
    pub fn new(gross_tons: f64, tare_mass_kg: Mass, length_m: Length, axles: u8) -> Self {
        Self {
            gross_tons,
            tare_mass_kg,
            length_m,
            axles,
            ..Self::default()
        }
    }

    fn validate(&self) -> Result<()> {
        if self.gross_tons <= 0.0 {
            bail!("NominalCar gross_tons must be > 0");
        }
        if self.tare_mass_kg < 0.0 {
            bail!("NominalCar tare_mass_kg must be >= 0");
        }
        if self.length_m <= 0.0 {
            bail!("NominalCar length_m must be > 0");
        }
        if self.axles == 0 {
            bail!("NominalCar axles must be > 0");
        }
        Ok(())
    }

    /// Gross mass per car in kg.
    pub fn gross_mass_kg(&self) -> Mass {
        self.gross_tons * KG_PER_SHORT_TON
    }

    /// Convert to a [`RailVehicle`] with the given `car_type` name.
    pub fn to_rail_vehicle(&self, car_type: String) -> RailVehicle {
        RailVehicle {
            car_type,
            freight_type: "Freight".to_string(),
            length: self.length_m,
            axle_count: self.axles,
            brake_count: self.brake_count,
            mass_static_base: self.tare_mass_kg,
            mass_freight: (self.gross_mass_kg() - self.tare_mass_kg).max(0.0),
            speed_max: self.speed_max_mps,
            braking_ratio: self.braking_ratio,
            mass_rot_per_axle: self.mass_rot_per_axle_kg,
            bearing_res_per_axle: self.bearing_res_per_axle_n,
            rolling_ratio: self.rolling_ratio,
            davis_b: self.davis_b,
            cd_area: self.cd_area_m2,
            curve_coeff_0: 0.0,
            curve_coeff_1: 0.0,
            curve_coeff_2: 0.0,
        }
    }
}

/// Pre-processed train parameters used by the resistance and speed models.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrainParams {
    /// Total train length (m).
    pub length: Length,
    /// Maximum train speed (m/s).
    pub speed_max: Velocity,
    /// Total towed mass (rail vehicles + freight), excluding locomotives (kg).
    pub towed_mass_static: Mass,
    /// Mass per brake (kg).
    pub mass_per_brake: Mass,
    /// Total axle count.
    pub axle_count: u32,
    /// Train type.
    pub train_type: TrainType,
    /// Curve coefficient 0.
    pub curve_coeff_0: Ratio,
    /// Curve coefficient 1.
    pub curve_coeff_1: Ratio,
    /// Curve coefficient 2.
    pub curve_coeff_2: Ratio,
}

/// User-defined train configuration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrainConfig {
    /// Types of rail vehicle composing the train.
    pub rail_vehicles: Vec<RailVehicle>,
    /// Number of railcars by type.
    pub n_cars_by_type: HashMap<String, u32>,
    /// Train type.
    #[serde(default)]
    pub train_type: TrainType,
    /// Optional train length override (m).
    #[serde(default)]
    pub train_length: Option<Length>,
    /// Optional total train mass override (kg).
    #[serde(default)]
    pub train_mass: Option<Mass>,
    /// Optional per-car drag areas (m^2).
    #[serde(default)]
    pub cd_area_vec: Option<Vec<Area>>,
}

impl TrainConfig {
    /// Total number of cars.
    pub fn cars_total(&self) -> u32 {
        self.n_cars_by_type.values().sum()
    }

    fn check_rv_keys(&self) -> Result<()> {
        for rv in &self.rail_vehicles {
            if !self.n_cars_by_type.contains_key(&rv.car_type) {
                bail!(
                    "rail_vehicle car_type `{}` missing from n_cars_by_type",
                    rv.car_type
                );
            }
        }
        for key in self.n_cars_by_type.keys() {
            if !self.rail_vehicles.iter().any(|rv| &rv.car_type == key) {
                bail!("n_cars_by_type key `{}` has no matching rail_vehicle", key);
            }
        }
        Ok(())
    }

    /// Derive [`TrainParams`] from this configuration.
    pub fn make_train_params(&self) -> Result<TrainParams> {
        self.check_rv_keys()?;

        let towed_mass_static = match self.train_mass {
            Some(m) => m,
            None => self
                .rail_vehicles
                .iter()
                .map(|rv| rv.mass() * *self.n_cars_by_type.get(&rv.car_type).unwrap() as f64)
                .sum(),
        };

        let length = match self.train_length {
            Some(l) => l,
            None => self
                .rail_vehicles
                .iter()
                .map(|rv| rv.length * *self.n_cars_by_type.get(&rv.car_type).unwrap() as f64)
                .sum(),
        };

        let speed_max = self
            .rail_vehicles
            .iter()
            .filter(|rv| *self.n_cars_by_type.get(&rv.car_type).unwrap() > 0)
            .map(|rv| rv.speed_max)
            .fold(f64::INFINITY, f64::min);

        let axle_count: u32 = self
            .rail_vehicles
            .iter()
            .map(|rv| rv.axle_count as u32 * *self.n_cars_by_type.get(&rv.car_type).unwrap())
            .sum();

        let brake_count: u32 = self
            .rail_vehicles
            .iter()
            .map(|rv| rv.brake_count as u32 * *self.n_cars_by_type.get(&rv.car_type).unwrap())
            .sum();

        let mass_per_brake = if brake_count > 0 {
            towed_mass_static / brake_count as f64
        } else {
            0.0
        };

        // Mass-weighted curve coefficients across the consist.
        let (mut c0, mut c1, mut c2) = (0.0, 0.0, 0.0);
        if towed_mass_static > 0.0 {
            for rv in &self.rail_vehicles {
                let n = *self.n_cars_by_type.get(&rv.car_type).unwrap() as f64;
                let w = rv.mass() * n / towed_mass_static;
                c0 += rv.curve_coeff_0 * w;
                c1 += rv.curve_coeff_1 * w;
                c2 += rv.curve_coeff_2 * w;
            }
        }

        Ok(TrainParams {
            length,
            speed_max,
            towed_mass_static,
            mass_per_brake,
            axle_count,
            train_type: self.train_type,
            curve_coeff_0: c0,
            curve_coeff_1: c1,
            curve_coeff_2: c2,
        })
    }

    /// Build a train configuration from total tonnage alone, using a default
    /// nominal car.
    ///
    /// The train is modeled as N nominal cars whose aggregate gross mass equals
    /// the requested tonnage. All other per-car properties (axles, length,
    /// resistance) come from [`NominalCar::default`].
    ///
    /// # Arguments
    /// - `short_tons`: total gross trailing tonnage in short tons (2000 lb).
    pub fn from_tonnage(short_tons: f64) -> Result<Self> {
        Self::from_tonnage_with(short_tons, NominalCar::default())
    }

    /// Build a train configuration from total tonnage with a specified nominal
    /// car. See [`TrainConfig::from_tonnage`].
    pub fn from_tonnage_with(short_tons: f64, nominal: NominalCar) -> Result<Self> {
        if short_tons <= 0.0 {
            bail!("tonnage must be > 0, got {short_tons}");
        }
        nominal.validate()?;
        let total_kg = short_tons * KG_PER_SHORT_TON;
        // Number of nominal cars needed to reach the total mass, minimum 1.
        let n_cars = ((total_kg / nominal.gross_mass_kg()).ceil() as u32).max(1);

        let car_type = "nominal".to_string();
        Ok(Self {
            rail_vehicles: vec![nominal.to_rail_vehicle(car_type.clone())],
            n_cars_by_type: HashMap::from([(car_type, n_cars)]),
            train_type: TrainType::default(),
            train_length: None,
            // Guarantee the total matches the requested tonnage exactly.
            train_mass: Some(total_kg),
            cd_area_vec: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_tonnage_mass_matches() {
        let tons = 6000.0;
        let cfg = TrainConfig::from_tonnage(tons).unwrap();
        assert!(cfg.cars_total() >= 1);
        let params = cfg.make_train_params().unwrap();
        let expected_kg = tons * KG_PER_SHORT_TON;
        assert!(
            (params.towed_mass_static - expected_kg).abs() < 1e-6,
            "towed mass {} != expected {}",
            params.towed_mass_static,
            expected_kg
        );
        assert!(params.length > 0.0);
        assert_eq!(params.train_type, TrainType::Freight);
    }

    #[test]
    fn from_tonnage_with_custom_nominal() {
        let nominal = NominalCar::new(110.0, 32_000.0, 16.0, 4);
        let cfg = TrainConfig::from_tonnage_with(5500.0, nominal).unwrap();
        let params = cfg.make_train_params().unwrap();
        let expected_kg = 5500.0 * KG_PER_SHORT_TON;
        assert!((params.towed_mass_static - expected_kg).abs() < 1e-6);
    }

    #[test]
    fn from_tonnage_rejects_bad_input() {
        assert!(TrainConfig::from_tonnage(0.0).is_err());
        assert!(TrainConfig::from_tonnage(-100.0).is_err());
    }
}

//! Physical data model: rail vehicles, train configuration, powertrain
//! components, locomotives, and consists.

pub mod consist;
pub mod loco_type;
pub mod locomotive;
pub mod powertrain;
pub mod rail_vehicle;
pub mod train_config;

pub use consist::Consist;
pub use loco_type::{LocoKind, LocoType, LocoTypeRegistry};
pub use locomotive::{Locomotive, PowertrainType};
pub use powertrain::{
    ElectricDrivetrain, FuelConverter, Generator, ReversibleEnergyStorage,
};
pub use rail_vehicle::RailVehicle;
pub use train_config::{TrainConfig, TrainParams, TrainType};

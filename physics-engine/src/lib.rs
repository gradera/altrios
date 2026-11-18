//! Standalone, pluggable train physics engine.
//!
//! This crate has no dependency on `altrios-core`. It reimplements the physics
//! needed to run a train on a track path: six-term resistance, a conventional
//! (engine -> generator -> drivetrain) and battery-electric powertrain, ramped
//! friction brakes, and longitudinal dynamics. The [`PhysicsEngine`] trait is
//! the pluggable seam used by the orchestrator.

pub mod brakes;
pub mod braking;
pub mod constants;
pub mod data;
pub mod engine;
pub mod interp;
pub mod metrics;
pub mod resistance;
pub mod state;
pub mod units;

pub use brakes::FricBrake;
pub use braking::{BrakingPoint, BrakingPoints};
pub use constants::{HP_TO_W, KG_PER_SHORT_TON};
pub use data::{
    Consist, ElectricDrivetrain, FuelConverter, Generator, LocoKind, LocoType,
    LocoTypeRegistry, Locomotive, PowertrainType, RailVehicle, ReversibleEnergyStorage,
    TrainConfig, TrainParams, TrainType,
};
pub use engine::{
    ControlInput, PhysicsEngine, SpeedLimitEngine, StepOutcome, TrainInput,
};
pub use metrics::TrainMetrics;
pub use resistance::{
    compute_resistance, PathProfile, PathResCoeff, ResistanceForces, ResistanceParams,
};
pub use state::TrainState;

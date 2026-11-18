//! Plain SI base-unit type aliases.
//!
//! The engine works in SI base units throughout. These aliases are purely
//! documentation: they make signatures self-describing without introducing a
//! unit-checking dependency. All values are `f64` in the stated unit.

/// Length in meters.
pub type Length = f64;
/// Velocity in meters per second.
pub type Velocity = f64;
/// Acceleration in meters per second squared.
pub type Acceleration = f64;
/// Mass in kilograms.
pub type Mass = f64;
/// Force in newtons.
pub type Force = f64;
/// Power in watts.
pub type Power = f64;
/// Energy in joules.
pub type Energy = f64;
/// Time in seconds.
pub type Time = f64;
/// Dimensionless ratio.
pub type Ratio = f64;
/// Area in square meters.
pub type Area = f64;
/// Inverse velocity in seconds per meter.
pub type InverseVelocity = f64;
/// Power rate in watts per second.
pub type PowerRate = f64;
/// Thermodynamic temperature in kelvin.
pub type Temperature = f64;

//! Physical constants and standard-atmosphere helpers.

use crate::units::*;

/// Acceleration due to gravity at the geographic center of the continental US
/// at sea level (WGS-84 Somigliana), matching ALTRIOS.
pub const ACC_GRAV: Acceleration = 9.801_548_494_963_14;

/// Standard sea-level air density in kg/m^3.
pub const RHO_AIR: f64 = 1.225;

/// Celsius-to-kelvin offset.
pub const CELSIUS_TO_KELVIN: f64 = 273.15;

/// Watts per mechanical horsepower (1 hp = 745.7 W).
pub const HP_TO_W: f64 = 745.7;

/// Kilograms per short ton (2000 lb).
pub const KG_PER_SHORT_TON: f64 = 907.184_74;

/// Air density (kg/m^3) for a given temperature (K) and elevation (m), using a
/// standard-atmosphere model. Mirrors ALTRIOS `uc::get_rho_air`.
pub fn rho_air(temperature: Temperature, elevation: Length) -> f64 {
    let std_temp = 15.04 - 0.00649 * elevation + CELSIUS_TO_KELVIN;
    let pressure = 101.29e3 * (std_temp / 288.08).powf(5.256);
    pressure / (287.0 * temperature)
}

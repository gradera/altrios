//! Train resistance: bearing, rolling, Davis-B, aerodynamic, grade, and curve.
//!
//! Resistance is the sum of physically distinct terms. Grade and curve are
//! position-dependent and are looked up from a pre-flattened path profile.

use crate::units::*;
use serde::{Deserialize, Serialize};

/// A resistance coefficient along the path (force per unit train weight).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct PathResCoeff {
    /// Resistance force per train weight (dimensionless).
    pub res_coeff: Ratio,
    /// Net elevation at this point (m), used for grade.
    pub res_net: Length,
}

/// Pre-flattened path profile: grade and curve coefficients plus speed limits.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct PathProfile {
    /// Cumulative offset at each profile point (m).
    pub offsets: Vec<Length>,
    /// Grade resistance coefficients.
    pub grades: Vec<PathResCoeff>,
    /// Curve resistance coefficients.
    pub curves: Vec<PathResCoeff>,
    /// Speed limit points: (offset, speed).
    pub speed_points: Vec<(Length, Velocity)>,
    /// Milepost (miles) at each profile point. Empty if not provided.
    #[serde(default)]
    pub mileposts: Vec<f64>,
    /// Subdivision name at each profile point. Empty if not provided.
    #[serde(default)]
    pub subdivisions: Vec<String>,
    /// Track name at each profile point. Empty if not provided.
    #[serde(default)]
    pub track_names: Vec<String>,
    /// Total path length (m).
    pub length: Length,
}

impl PathProfile {
    /// Index of the profile point containing `offset`.
    pub fn idx_at(&self, offset: Length) -> usize {
        if self.offsets.is_empty() {
            return 0;
        }
        match self.offsets.iter().position(|&o| o > offset) {
            Some(i) => i.saturating_sub(1),
            None => self.offsets.len() - 1,
        }
    }

    /// Coefficient index of the segment containing `offset`.
    ///
    /// The profile stores each segment's coefficient at the segment's *end*
    /// offset, so the coefficient index is one past the lower-bound index
    /// returned by [`idx_at`](Self::idx_at).
    fn coeff_idx_at(&self, offset: Length) -> usize {
        let n = self.offsets.len();
        if n <= 1 {
            return 0;
        }
        if offset <= self.offsets[0] {
            return 1.min(n - 1);
        }
        (self.idx_at(offset) + 1).min(n - 1)
    }

    /// Grade coefficient at `offset`.
    pub fn grade_at(&self, offset: Length) -> Ratio {
        if self.grades.is_empty() {
            return 0.0;
        }
        self.grades[self.coeff_idx_at(offset).min(self.grades.len() - 1)].res_coeff
    }

    /// Curve coefficient at `offset`.
    pub fn curve_at(&self, offset: Length) -> Ratio {
        if self.curves.is_empty() {
            return 0.0;
        }
        self.curves[self.coeff_idx_at(offset).min(self.curves.len() - 1)].res_coeff
    }

    /// Elevation (net) at `offset`, linearly interpolated between profile
    /// points. Returns 0 if not provided.
    pub fn elev_at(&self, offset: Length) -> Length {
        if self.grades.is_empty() {
            return 0.0;
        }
        if self.offsets.len() < 2 {
            return self.grades[0].res_net;
        }
        let i = self.idx_at(offset);
        let i_next = (i + 1).min(self.offsets.len() - 1);
        let x0 = self.offsets[i];
        let x1 = self.offsets[i_next];
        let e0 = self.grades[i].res_net;
        let e1 = self.grades[i_next].res_net;
        if (x1 - x0).abs() < 1e-9 {
            e0
        } else {
            e0 + (e1 - e0) * (offset - x0) / (x1 - x0)
        }
    }

    /// Grade coefficient averaged over the train footprint
    /// `[offset - length, offset]` (ALTRIOS "strap" model).
    ///
    /// Grade resistance is the net elevation change over the footprint divided
    /// by the footprint length, so a long train straddling a grade break sees a
    /// length-weighted blend instead of the value at the front only. Falls back
    /// to the point value when the footprint is degenerate or elevation is
    /// unavailable.
    pub fn grade_avg(&self, offset: Length, length: Length) -> Ratio {
        if self.grades.is_empty() {
            return 0.0;
        }
        if length <= 0.0 || self.offsets.len() < 2 {
            return self.grade_at(offset);
        }
        let front = offset.clamp(0.0, self.length);
        let back = (offset - length).clamp(0.0, self.length);
        let span = front - back;
        if span <= 1e-9 {
            return self.grade_at(offset);
        }
        (self.elev_at(front) - self.elev_at(back)) / span
    }

    /// Curve coefficient averaged over the train footprint
    /// `[offset - length, offset]` (ALTRIOS "strap" model).
    ///
    /// Curve resistance is the integral of the piecewise-constant curve
    /// coefficient over the footprint divided by the footprint length. Falls
    /// back to the point value when the footprint is degenerate.
    pub fn curve_avg(&self, offset: Length, length: Length) -> Ratio {
        if self.curves.is_empty() {
            return 0.0;
        }
        if length <= 0.0 || self.offsets.len() < 2 {
            return self.curve_at(offset);
        }
        let front = offset.clamp(0.0, self.length);
        let back = (offset - length).clamp(0.0, self.length);
        let span = front - back;
        if span <= 1e-9 {
            return self.curve_at(offset);
        }
        // Integrate the piecewise-constant coefficient over [back, front].
        let mut integral = 0.0;
        let mut x = back;
        while x < front {
            let i = self.idx_at(x);
            let seg_end = self
                .offsets
                .get(i + 1)
                .copied()
                .unwrap_or(front)
                .min(front);
            let seg_start = x.max(self.offsets.get(i).copied().unwrap_or(back));
            let w = seg_end - seg_start;
            if w > 1e-12 {
                let mid = 0.5 * (seg_start + seg_end);
                integral += self.curve_at(mid) * w;
                x = seg_end;
            } else {
                // Guard against non-advancing loops at duplicate offsets.
                x = seg_end.max(x + 1e-6);
            }
        }
        integral / span
    }

    /// Speed limit at `offset`.
    pub fn speed_limit_at(&self, offset: Length) -> Velocity {
        if self.speed_points.is_empty() {
            return f64::INFINITY;
        }
        let mut limit = self.speed_points[0].1;
        for &(o, v) in &self.speed_points {
            if o <= offset {
                limit = v;
            } else {
                break;
            }
        }
        limit
    }

    /// Milepost (miles) at `offset`, linearly interpolated between profile
    /// points (so it tracks distance exactly). Returns 0 if not provided.
    pub fn milepost_at(&self, offset: Length) -> f64 {
        if self.mileposts.is_empty() {
            return 0.0;
        }
        if self.mileposts.len() != self.offsets.len() || self.offsets.len() < 2 {
            return self.mileposts[0];
        }
        let i = self.idx_at(offset);
        let i_next = (i + 1).min(self.offsets.len() - 1);
        let x0 = self.offsets[i];
        let x1 = self.offsets[i_next];
        let m0 = self.mileposts[i];
        let m1 = self.mileposts[i_next];
        if (x1 - x0).abs() < 1e-9 {
            m0
        } else {
            m0 + (m1 - m0) * (offset - x0) / (x1 - x0)
        }
    }

    /// Subdivision name at `offset`, or `""` if not provided.
    pub fn subdivision_at(&self, offset: Length) -> &str {
        Self::name_at(&self.subdivisions, self.idx_at(offset))
    }

    /// Track name at `offset`, or `""` if not provided.
    pub fn track_name_at(&self, offset: Length) -> &str {
        Self::name_at(&self.track_names, self.idx_at(offset))
    }

    fn name_at(names: &[String], idx: usize) -> &str {
        if names.is_empty() {
            ""
        } else {
            names.get(idx).map(|s| s.as_str()).unwrap_or("")
        }
    }
}

/// Static resistance parameters for a train.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ResistanceParams {
    /// Bearing resistance force (N).
    pub bearing: Force,
    /// Rolling resistance ratio (force per weight).
    pub rolling_ratio: Ratio,
    /// Davis B coefficient (s/m).
    pub davis_b: InverseVelocity,
    /// Drag area Cd*A (m^2).
    pub cd_area: Area,
}

/// The six decomposed resistance forces at a point in time.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ResistanceForces {
    /// Bearing resistance (N).
    pub bearing: Force,
    /// Rolling resistance (N).
    pub rolling: Force,
    /// Davis B resistance (N).
    pub davis_b: Force,
    /// Aerodynamic resistance (N).
    pub aero: Force,
    /// Grade resistance (N).
    pub grade: Force,
    /// Curve resistance (N).
    pub curve: Force,
}

impl ResistanceForces {
    /// Sum of all resistance forces (N).
    pub fn total(&self) -> Force {
        self.bearing + self.rolling + self.davis_b + self.aero + self.grade + self.curve
    }
}

/// Compute the six resistance forces.
///
/// * `weight_static` is the train weight (N).
/// * `speed` is the train speed (m/s).
/// * `grade_coeff` / `curve_coeff` are force-per-weight coefficients at the
///   train's position.
/// * `rho` is air density (kg/m^3); use [`crate::constants::RHO_AIR`] for the
///   standard sea-level value or [`crate::constants::rho_air`] for an
///   elevation/temperature-corrected value.
pub fn compute_resistance(
    params: &ResistanceParams,
    weight_static: Force,
    speed: Velocity,
    grade_coeff: Ratio,
    curve_coeff: Ratio,
    rho: f64,
) -> ResistanceForces {
    ResistanceForces {
        bearing: params.bearing,
        rolling: params.rolling_ratio * weight_static,
        davis_b: params.davis_b * speed * weight_static,
        aero: params.cd_area * rho * speed * speed,
        grade: grade_coeff * weight_static,
        curve: curve_coeff * weight_static,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::RHO_AIR;

    #[test]
    fn resistance_terms() {
        let params = ResistanceParams {
            bearing: 1000.0,
            rolling_ratio: 0.001,
            davis_b: 0.0001,
            cd_area: 10.0,
        };
        let weight = 1_000_000.0; // N
        let speed = 20.0; // m/s
        let f = compute_resistance(&params, weight, speed, 0.01, 0.0, RHO_AIR);
        assert!((f.bearing - 1000.0).abs() < 1e-9);
        assert!((f.rolling - 1000.0).abs() < 1e-9);
        assert!((f.davis_b - 0.0001 * 20.0 * 1_000_000.0).abs() < 1e-6);
        assert!((f.aero - 10.0 * RHO_AIR * 400.0).abs() < 1e-6);
        assert!((f.grade - 0.01 * 1_000_000.0).abs() < 1e-9);
        assert!((f.curve - 0.0).abs() < 1e-9);
    }

    #[test]
    fn aero_scales_with_rho() {
        let params = ResistanceParams {
            cd_area: 10.0,
            ..Default::default()
        };
        let base = compute_resistance(&params, 1.0, 20.0, 0.0, 0.0, RHO_AIR);
        let half = compute_resistance(&params, 1.0, 20.0, 0.0, 0.0, RHO_AIR * 0.5);
        assert!((half.aero - 0.5 * base.aero).abs() < 1e-9);
    }

    /// Path with a step grade: flat to 1000 m, then 1% to 2000 m.
    fn step_path() -> PathProfile {
        PathProfile {
            offsets: vec![0.0, 1000.0, 2000.0],
            grades: vec![
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
                PathResCoeff { res_coeff: 0.01, res_net: 10.0 },
            ],
            curves: vec![
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
            ],
            speed_points: vec![(0.0, 25.0)],
            length: 2000.0,
            ..Default::default()
        }
    }

    #[test]
    fn grade_avg_uniform_matches_point() {
        // Footprint entirely within the 1% segment (offset 1500, length 400).
        let p = step_path();
        assert!((p.grade_avg(1500.0, 400.0) - 0.01).abs() < 1e-9);
    }

    #[test]
    fn grade_avg_blends_across_break() {
        // Front at 1200, length 400 -> footprint [800, 1200]. Only [1000,1200]
        // is on the 1% segment: 200 m of grade over 400 m footprint -> 0.005.
        let p = step_path();
        assert!((p.grade_avg(1200.0, 400.0) - 0.005).abs() < 1e-9);
    }

    #[test]
    fn grade_avg_zero_length_is_point() {
        let p = step_path();
        assert!((p.grade_avg(1500.0, 0.0) - p.grade_at(1500.0)).abs() < 1e-12);
    }

    /// Path with a step curve: 0 rad/m to 1000 m, then 0.001 to 2000 m.
    fn curve_step_path() -> PathProfile {
        PathProfile {
            offsets: vec![0.0, 1000.0, 2000.0],
            grades: vec![PathResCoeff::default(); 3],
            curves: vec![
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
                PathResCoeff { res_coeff: 0.0, res_net: 0.0 },
                PathResCoeff { res_coeff: 0.001, res_net: 0.0 },
            ],
            speed_points: vec![(0.0, 25.0)],
            length: 2000.0,
            ..Default::default()
        }
    }

    #[test]
    fn curve_avg_integrates_across_break() {
        // Front 1200, length 400 -> footprint [800,1200]; 200 m at 0.001 over
        // 400 m -> 0.0005.
        let p = curve_step_path();
        assert!((p.curve_avg(1200.0, 400.0) - 0.0005).abs() < 1e-9);
    }

    #[test]
    fn curve_avg_uniform_matches_point() {
        let p = curve_step_path();
        assert!((p.curve_avg(1500.0, 400.0) - 0.001).abs() < 1e-9);
    }
}

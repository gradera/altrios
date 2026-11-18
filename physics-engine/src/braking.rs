//! Precomputed braking points (ALTRIOS-style braking curve).
//!
//! A [`BrakingPoints`] instance is built once per path by sweeping the speed
//! limits backward from the end of the route, inserting a *braking point* ahead
//! of every lower limit. Each point carries the local speed limit and a
//! `speed_target` that already accounts for the distance needed to brake down
//! to all upcoming limits. This replaces a fixed-time look-ahead with a
//! physically consistent braking curve, which improves safety (the train begins
//! braking at the correct distance and does not overshoot a lower limit).

use crate::resistance::PathProfile;
use crate::units::*;
use serde::{Deserialize, Serialize};

/// A single point on the braking curve.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct BrakingPoint {
    /// Offset along the path (m).
    pub offset: Length,
    /// Speed limit in effect at this offset (m/s).
    pub speed_limit: Velocity,
    /// Speed target at this offset, accounting for upcoming limits (m/s).
    pub speed_target: Velocity,
}

/// A precomputed braking curve along a path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct BrakingPoints {
    /// Speed points `(offset, limit)`, ascending by offset.
    points: Vec<(Length, Velocity)>,
    /// Path length (m).
    length: Length,
    /// Braking deceleration (m/s^2). Non-positive disables anticipation.
    decel: Acceleration,
}

impl BrakingPoints {
    /// Build a braking curve from a path profile and a braking deceleration
    /// `decel` (m/s^2).
    ///
    /// `decel` is typically `fric_brake.force_max / train_mass`. A non-positive
    /// value disables braking-distance anticipation (targets become the running
    /// limit).
    pub fn build(path: &PathProfile, decel: Acceleration) -> Self {
        let length = path.length;
        let mut pts: Vec<(Length, Velocity)> = path
            .speed_points
            .iter()
            .copied()
            .filter(|(o, _)| *o >= -1e-6 && *o <= length + 1e-6)
            .collect();
        pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        if pts.is_empty() {
            return Self {
                points: pts,
                length,
                decel: decel.max(0.0),
            };
        }
        // Ensure a point at the start and end of the path.
        if pts[0].0 > 1e-9 {
            let v0 = pts[0].1;
            pts.insert(0, (0.0, v0));
        }
        if (pts.last().unwrap().0 - length).abs() > 1e-6 {
            let vl = pts.last().unwrap().1;
            pts.push((length, vl));
        }
        Self {
            points: pts,
            length,
            decel: decel.max(0.0),
        }
    }

    /// Whether this curve has no points.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Speed points `(offset, limit)` (ascending by offset).
    pub fn points(&self) -> &[(Length, Velocity)] {
        &self.points
    }

    /// Local speed limit at `offset` (m/s), or `+inf` if no points.
    pub fn speed_limit_at(&self, offset: Length) -> Velocity {
        if self.points.is_empty() {
            return f64::INFINITY;
        }
        let mut limit = self.points[0].1;
        for &(o, v) in &self.points {
            if o <= offset {
                limit = v;
            } else {
                break;
            }
        }
        limit
    }

    /// Speed limit and speed target at `offset`.
    ///
    /// The target is the tightest of the local limit and the braking-limited
    /// speed for every upcoming limit point (so the train starts braking at the
    /// physically correct distance). `adj_ramp_up_time` adds a look-ahead time
    /// (s) so braking begins slightly earlier to cover brake build-up.
    pub fn calc_speeds(
        &mut self,
        offset: Length,
        speed: Velocity,
        adj_ramp_up_time: Time,
    ) -> (Velocity, Velocity) {
        if self.points.is_empty() {
            return (f64::INFINITY, f64::INFINITY);
        }
        let limit = self.speed_limit_at(offset);
        let mut target = limit;
        if self.decel > 0.0 {
            let ramp_dist = (speed * adj_ramp_up_time).max(0.0);
            for &(o, v) in &self.points {
                let d = o - offset - ramp_dist;
                if d <= 0.0 {
                    // Point already at/behind us: fold its limit in.
                    target = target.min(v);
                    continue;
                }
                // Highest speed now that still allows braking to `v` by `o`.
                let v_now = (v * v + 2.0 * self.decel * d).sqrt();
                target = target.min(v_now);
            }
        }
        (limit, target.min(limit))
    }

    /// Materialize the envelope at each speed-point offset (for inspection).
    pub fn to_points(&self) -> Vec<BrakingPoint> {
        let mut me = self.clone();
        self.points
            .iter()
            .map(|&(o, _)| {
                let (speed_limit, speed_target) = me.calc_speeds(o, 0.0, 0.0);
                BrakingPoint {
                    offset: o,
                    speed_limit,
                    speed_target,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resistance::PathResCoeff;

    fn path_with_limits(limits: &[(Length, Velocity)], length: Length) -> PathProfile {
        PathProfile {
            offsets: vec![0.0, length],
            grades: vec![PathResCoeff::default(); 2],
            curves: vec![PathResCoeff::default(); 2],
            speed_points: limits.to_vec(),
            length,
            ..Default::default()
        }
    }

    #[test]
    fn uniform_limit_target_equals_limit() {
        let p = path_with_limits(&[(0.0, 20.0), (5000.0, 20.0)], 5000.0);
        let mut bp = BrakingPoints::build(&p, 0.5);
        for off in [0.0, 1000.0, 2500.0, 5000.0] {
            let (limit, target) = bp.calc_speeds(off, 20.0, 0.0);
            assert!((limit - 20.0).abs() < 1e-9);
            assert!((target - 20.0).abs() < 1e-9, "target {target} at {off}");
        }
    }

    #[test]
    fn descending_limit_brakes_early() {
        // 30 m/s until 1000 m, then 10 m/s. Decel 0.5 m/s^2.
        let p = path_with_limits(&[(0.0, 30.0), (1000.0, 10.0), (5000.0, 10.0)], 5000.0);
        let mut bp = BrakingPoints::build(&p, 0.5);
        // At offset 0 the target must allow braking to 10 m/s over 1000 m:
        // sqrt(10^2 + 2*0.5*1000) = sqrt(1100) ~= 33.17, capped by 30.
        let (_, target0) = bp.calc_speeds(0.0, 30.0, 0.0);
        assert!((target0 - 30.0).abs() < 1e-6, "target0 {target0}");
        // Halfway (500 m) the braking distance to the 10 limit is 500 m, so the
        // max speed is sqrt(10^2 + 2*0.5*500) = sqrt(600) ~= 24.49.
        let (_, target500) = bp.calc_speeds(500.0, 30.0, 0.0);
        assert!(
            (target500 - 600f64.sqrt()).abs() < 1e-6,
            "target500 {target500}"
        );
        // Past the limit, target is the low limit.
        let (_, target1200) = bp.calc_speeds(1200.0, 10.0, 0.0);
        assert!((target1200 - 10.0).abs() < 1e-9);
    }

    #[test]
    fn target_never_exceeds_limit() {
        let p = path_with_limits(&[(0.0, 30.0), (1000.0, 10.0), (3000.0, 25.0)], 3000.0);
        let mut bp = BrakingPoints::build(&p, 0.4);
        for off in (0..=3000).step_by(100) {
            let (limit, target) = bp.calc_speeds(off as f64, 20.0, 0.0);
            assert!(target <= limit + 1e-9, "target {target} > limit {limit} at {off}");
        }
    }

    #[test]
    fn empty_path_is_infinite() {
        let mut bp = BrakingPoints::default();
        let (limit, target) = bp.calc_speeds(0.0, 10.0, 0.0);
        assert!(limit.is_infinite() && target.is_infinite());
    }
}

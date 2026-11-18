//! Minimal 1-D linear interpolation used by powertrain efficiency tables.

use anyhow::{bail, Result};

/// Linearly interpolate `y` at `x` given monotonically increasing `xs` and
/// matching `ys`. Values outside the range are clamped to the endpoints.
pub fn interp1d(x: f64, xs: &[f64], ys: &[f64]) -> Result<f64> {
    if xs.len() != ys.len() {
        bail!("interp1d: xs and ys must have equal length");
    }
    if xs.is_empty() {
        bail!("interp1d: empty interpolation table");
    }
    if xs.len() == 1 {
        return Ok(ys[0]);
    }
    if x <= xs[0] {
        return Ok(ys[0]);
    }
    if x >= xs[xs.len() - 1] {
        return Ok(ys[ys.len() - 1]);
    }
    for i in 1..xs.len() {
        if x <= xs[i] {
            let x0 = xs[i - 1];
            let x1 = xs[i];
            let y0 = ys[i - 1];
            let y1 = ys[i];
            let frac = if (x1 - x0).abs() < f64::EPSILON {
                0.0
            } else {
                (x - x0) / (x1 - x0)
            };
            return Ok(y0 + frac * (y1 - y0));
        }
    }
    Ok(ys[ys.len() - 1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_and_interpolates() {
        let xs = [0.0, 0.5, 1.0];
        let ys = [0.2, 0.4, 0.3];
        assert!((interp1d(-1.0, &xs, &ys).unwrap() - 0.2).abs() < 1e-12);
        assert!((interp1d(0.25, &xs, &ys).unwrap() - 0.3).abs() < 1e-12);
        assert!((interp1d(2.0, &xs, &ys).unwrap() - 0.3).abs() < 1e-12);
    }
}

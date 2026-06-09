//! Minimal IMU CSV reader for the replay tool.
//!
//! Format: one sample per line, fields separated by commas and/or whitespace,
//! `#` comments and blank lines ignored:
//!
//! ```text
//! # t gx gy gz ax ay az   (seconds, rad/s, m/s^2)
//! 0.000  0 0 0   0 0 9.80665
//! ```

use std::io::BufRead;

use anyhow::{bail, Context, Result};
use slam_types::{ImuSample, Stamp, Vec3};

/// Read IMU samples from any reader.
pub fn read_imu<R: BufRead>(reader: R) -> Result<Vec<ImuSample>> {
    let mut out = Vec::new();
    for (idx, line) in reader.lines().enumerate() {
        let line = line.with_context(|| format!("reading line {}", idx + 1))?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let raw: Vec<&str> = trimmed
            .split(|c: char| c == ',' || c.is_whitespace())
            .filter(|s| !s.is_empty())
            .collect();
        if raw.len() != 7 {
            bail!(
                "line {}: expected 7 fields (t gx gy gz ax ay az), found {}",
                idx + 1,
                raw.len()
            );
        }
        // Timestamp parsed exactly from text (epoch-scale safe); the six axes as f64.
        let stamp = Stamp::from_seconds_str(raw[0])
            .with_context(|| format!("line {}: bad timestamp {:?}", idx + 1, raw[0]))?;
        let mut v = [0f64; 6];
        for (i, s) in raw[1..].iter().enumerate() {
            v[i] = s
                .parse::<f64>()
                .with_context(|| format!("line {}: cannot parse {s:?} as a number", idx + 1))?;
        }
        out.push(ImuSample::new(
            stamp,
            Vec3::new(v[0], v[1], v[2]),
            Vec3::new(v[3], v[4], v[5]),
        ));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_mixed_separators_and_skips_comments() {
        let text = "# t gx gy gz ax ay az\n0.0, 0,0,0, 0 0 9.81\n0.01 0 0 1  0 0 9.81\n";
        let samples = read_imu(text.as_bytes()).unwrap();
        assert_eq!(samples.len(), 2);
        assert_eq!(samples[1].gyro.z, 1.0);
        assert_eq!(samples[0].accel.z, 9.81);
    }

    #[test]
    fn rejects_wrong_field_count() {
        let err = read_imu("0.0 0 0 0 0 0\n".as_bytes()).unwrap_err();
        assert!(err.to_string().contains("expected 7 fields"));
    }
}

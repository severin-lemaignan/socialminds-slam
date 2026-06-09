//! IMU CSV interchange — the simple recorded-IMU format the tools share.
//!
//! One sample per line, fields separated by commas and/or whitespace, `#` comments and
//! blank lines ignored:
//!
//! ```text
//! # t gx gy gz ax ay az   (seconds, rad/s, m/s^2)
//! 0.000  0 0 0   0 0 9.80665
//! ```
//!
//! The timestamp is parsed exactly from text (see [`crate::time::Stamp::from_seconds_str`])
//! so high-rate streams stay associable.

use std::io::{self, BufRead, Write};

use crate::geometry::Vec3;
use crate::sensor::ImuSample;
use crate::time::Stamp;

/// Errors from parsing an IMU CSV stream.
#[derive(Debug, thiserror::Error)]
pub enum ImuCsvError {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("line {line}: expected 7 fields (t gx gy gz ax ay az), found {found}")]
    WrongFieldCount { line: usize, found: usize },
    #[error("line {line}: bad timestamp {value:?}")]
    BadTimestamp { line: usize, value: String },
    #[error("line {line}: cannot parse field {field} ({value:?}) as a number")]
    BadNumber {
        line: usize,
        field: usize,
        value: String,
    },
}

/// Read IMU samples from any reader.
pub fn read_imu<R: BufRead>(reader: R) -> Result<Vec<ImuSample>, ImuCsvError> {
    let mut out = Vec::new();
    for (idx, line) in reader.lines().enumerate() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let raw: Vec<&str> = trimmed
            .split(|c: char| c == ',' || c.is_whitespace())
            .filter(|s| !s.is_empty())
            .collect();
        if raw.len() != 7 {
            return Err(ImuCsvError::WrongFieldCount {
                line: idx + 1,
                found: raw.len(),
            });
        }
        let stamp = Stamp::from_seconds_str(raw[0]).map_err(|_| ImuCsvError::BadTimestamp {
            line: idx + 1,
            value: raw[0].to_string(),
        })?;
        let mut v = [0f64; 6];
        for (i, s) in raw[1..].iter().enumerate() {
            v[i] = s.parse::<f64>().map_err(|_| ImuCsvError::BadNumber {
                line: idx + 1,
                field: i + 2,
                value: (*s).to_string(),
            })?;
        }
        out.push(ImuSample::new(
            stamp,
            Vec3::new(v[0], v[1], v[2]),
            Vec3::new(v[3], v[4], v[5]),
        ));
    }
    Ok(out)
}

/// Write IMU samples in the canonical CSV format.
pub fn write_imu<W: Write>(samples: &[ImuSample], mut writer: W) -> io::Result<()> {
    writeln!(writer, "# t gx gy gz ax ay az  (seconds, rad/s, m/s^2)")?;
    for s in samples {
        writeln!(
            writer,
            "{} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
            s.stamp, s.gyro.x, s.gyro.y, s.gyro.z, s.accel.x, s.accel.y, s.accel.z
        )?;
    }
    Ok(())
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
        assert!(matches!(
            err,
            ImuCsvError::WrongFieldCount { line: 1, found: 6 }
        ));
    }

    #[test]
    fn rejects_bad_timestamp() {
        let err = read_imu("nope 0 0 0 0 0 9.81\n".as_bytes()).unwrap_err();
        assert!(matches!(err, ImuCsvError::BadTimestamp { line: 1, .. }));
    }

    #[test]
    fn write_read_roundtrip() {
        let samples = vec![
            ImuSample::new(
                Stamp::from_seconds(0.0),
                Vec3::new(1.0, 2.0, 3.0),
                Vec3::new(4.0, 5.0, 6.0),
            ),
            ImuSample::new(
                Stamp::from_seconds(0.005),
                Vec3::zeros(),
                Vec3::new(0.0, 0.0, 9.81),
            ),
        ];
        let mut buf = Vec::new();
        write_imu(&samples, &mut buf).unwrap();
        let back = read_imu(buf.as_slice()).unwrap();
        assert_eq!(back.len(), 2);
        assert_eq!(back[0].gyro, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(back[1].accel.z, 9.81);
    }
}

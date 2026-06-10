//! Laser-scan CSV interchange — one self-describing scan per line.
//!
//! Line format (`#` comments and blank lines ignored):
//!
//! ```text
//! # t angle_min angle_increment range_min range_max n r0 .. r(n-1)
//! 0.000 -1.5708 0.0061 0.1 25.0 4 1.2 1.3 inf 1.5
//! ```
//!
//! Non-finite ranges are written/parsed as `nan`/`inf`/`-inf` so invalid sensor returns
//! survive the round trip (validity filtering happens in
//! [`LaserScan2D::points`](crate::sensor::LaserScan2D::points), not at I/O time). The
//! timestamp is parsed exactly from text, like the IMU CSV.

use std::io::{self, BufRead, Write};

use crate::sensor::{FrameId, LaserScan2D};
use crate::time::Stamp;

/// Errors from parsing a scan CSV stream.
#[derive(Debug, thiserror::Error)]
pub enum ScanCsvError {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("line {line}: expected at least 6 fields (t angle_min angle_increment range_min range_max n …), found {found}")]
    WrongFieldCount { line: usize, found: usize },
    #[error("line {line}: bad timestamp {value:?}")]
    BadTimestamp { line: usize, value: String },
    #[error("line {line}: cannot parse field {field} ({value:?}) as a number")]
    BadNumber {
        line: usize,
        field: usize,
        value: String,
    },
    #[error("line {line}: declares {declared} ranges but carries {found}")]
    RangeCountMismatch {
        line: usize,
        declared: usize,
        found: usize,
    },
}

/// Read scans from any reader.
pub fn read_scans<R: BufRead>(reader: R) -> Result<Vec<LaserScan2D>, ScanCsvError> {
    let mut out = Vec::new();
    for (idx, line) in reader.lines().enumerate() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let raw: Vec<&str> = trimmed.split_whitespace().collect();
        if raw.len() < 6 {
            return Err(ScanCsvError::WrongFieldCount {
                line: idx + 1,
                found: raw.len(),
            });
        }
        let stamp = Stamp::from_seconds_str(raw[0]).map_err(|_| ScanCsvError::BadTimestamp {
            line: idx + 1,
            value: raw[0].to_string(),
        })?;
        let num = |field: usize| -> Result<f64, ScanCsvError> {
            raw[field]
                .parse::<f64>()
                .map_err(|_| ScanCsvError::BadNumber {
                    line: idx + 1,
                    field: field + 1,
                    value: raw[field].to_string(),
                })
        };
        let (angle_min, angle_increment) = (num(1)?, num(2)?);
        let (range_min, range_max) = (num(3)?, num(4)?);
        let declared = num(5)? as usize;
        let found = raw.len() - 6;
        if declared != found {
            return Err(ScanCsvError::RangeCountMismatch {
                line: idx + 1,
                declared,
                found,
            });
        }
        let mut ranges = Vec::with_capacity(declared);
        for (i, s) in raw[6..].iter().enumerate() {
            ranges.push(s.parse::<f32>().map_err(|_| ScanCsvError::BadNumber {
                line: idx + 1,
                field: i + 7,
                value: (*s).to_string(),
            })?);
        }
        out.push(LaserScan2D {
            stamp,
            frame: FrameId::BASE,
            angle_min,
            angle_increment,
            range_min,
            range_max,
            ranges,
        });
    }
    Ok(out)
}

/// Write scans in the canonical one-scan-per-line format.
pub fn write_scans<W: Write>(scans: &[LaserScan2D], mut writer: W) -> io::Result<()> {
    writeln!(
        writer,
        "# t angle_min angle_increment range_min range_max n r0 .. r(n-1)  (seconds, rad, m)"
    )?;
    for s in scans {
        write!(
            writer,
            "{} {:.9} {:.9} {:.4} {:.4} {}",
            s.stamp,
            s.angle_min,
            s.angle_increment,
            s.range_min,
            s.range_max,
            s.ranges.len()
        )?;
        for r in &s.ranges {
            write!(writer, " {r}")?;
        }
        writeln!(writer)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scan(stamp_s: f64, ranges: Vec<f32>) -> LaserScan2D {
        LaserScan2D {
            stamp: Stamp::from_seconds(stamp_s),
            frame: FrameId::BASE,
            angle_min: -1.5,
            angle_increment: 0.01,
            range_min: 0.1,
            range_max: 25.0,
            ranges,
        }
    }

    #[test]
    fn write_read_roundtrip_preserves_non_finite_ranges() {
        let scans = vec![
            scan(0.0, vec![1.25, f32::INFINITY, 3.5]),
            scan(0.1, vec![f32::NAN, 2.0]),
        ];
        let mut buf = Vec::new();
        write_scans(&scans, &mut buf).unwrap();
        let back = read_scans(buf.as_slice()).unwrap();
        assert_eq!(back.len(), 2);
        assert_eq!(back[0].ranges[0], 1.25);
        assert!(back[0].ranges[1].is_infinite());
        assert!(back[1].ranges[0].is_nan());
        assert_eq!(back[1].ranges[1], 2.0);
        assert_eq!(back[0].angle_min, -1.5);
    }

    #[test]
    fn rejects_range_count_mismatch() {
        let err = read_scans("0.0 -1.5 0.01 0.1 25.0 3 1.0 2.0\n".as_bytes()).unwrap_err();
        assert!(matches!(
            err,
            ScanCsvError::RangeCountMismatch {
                line: 1,
                declared: 3,
                found: 2
            }
        ));
    }

    #[test]
    fn rejects_truncated_header_fields() {
        let err = read_scans("0.0 -1.5 0.01 0.1\n".as_bytes()).unwrap_err();
        assert!(matches!(
            err,
            ScanCsvError::WrongFieldCount { line: 1, found: 4 }
        ));
    }
}

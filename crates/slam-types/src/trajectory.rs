//! Trajectories and TUM-format I/O.
//!
//! TUM is our canonical interchange format (the engine emits it, the `evo`-based harness
//! consumes it). One pose per line:
//!
//! ```text
//! # comment lines start with '#'
//! timestamp tx ty tz qx qy qz qw
//! ```
//!
//! where `timestamp` is in seconds and the quaternion is `(x, y, z, w)`.

use std::io::{self, BufRead, Write};
use std::path::Path;

use crate::geometry::{Pose, Rotation, Vec3};
use crate::time::Stamp;

/// A timestamped pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StampedPose {
    pub stamp: Stamp,
    pub pose: Pose,
}

impl StampedPose {
    #[inline]
    pub fn new(stamp: Stamp, pose: Pose) -> Self {
        StampedPose { stamp, pose }
    }
}

/// A time-ordered sequence of poses.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Trajectory {
    poses: Vec<StampedPose>,
}

/// Errors from parsing a TUM trajectory file.
#[derive(Debug, thiserror::Error)]
pub enum TumParseError {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("line {line}: expected 8 whitespace-separated fields, found {found}")]
    WrongFieldCount { line: usize, found: usize },
    #[error("line {line}: could not parse field {field} ({value:?}) as a number")]
    BadNumber {
        line: usize,
        field: usize,
        value: String,
    },
}

impl Trajectory {
    pub fn new() -> Self {
        Trajectory { poses: Vec::new() }
    }

    /// Append a pose. Callers are responsible for chronological order; [`is_sorted`]
    /// can verify it.
    #[inline]
    pub fn push(&mut self, p: StampedPose) {
        self.poses.push(p);
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    #[inline]
    pub fn poses(&self) -> &[StampedPose] {
        &self.poses
    }

    /// True if timestamps are non-decreasing.
    pub fn is_sorted(&self) -> bool {
        self.poses.windows(2).all(|w| w[0].stamp <= w[1].stamp)
    }

    /// Parse a TUM-format trajectory from any reader. Blank lines and `#` comments are
    /// skipped.
    pub fn read_tum<R: BufRead>(reader: R) -> Result<Trajectory, TumParseError> {
        let mut traj = Trajectory::new();
        for (idx, line) in reader.lines().enumerate() {
            let line = line?;
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            let fields: Vec<&str> = trimmed.split_whitespace().collect();
            if fields.len() != 8 {
                return Err(TumParseError::WrongFieldCount {
                    line: idx + 1,
                    found: fields.len(),
                });
            }
            // Field 1 (timestamp) is parsed exactly from text; epoch-scale values would
            // lose nanoseconds through f64 (see Stamp::from_seconds_str).
            let stamp =
                Stamp::from_seconds_str(fields[0]).map_err(|_| TumParseError::BadNumber {
                    line: idx + 1,
                    field: 1,
                    value: fields[0].to_string(),
                })?;
            let mut vals = [0f64; 7];
            for (i, f) in fields[1..].iter().enumerate() {
                vals[i] = f.parse().map_err(|_| TumParseError::BadNumber {
                    line: idx + 1,
                    field: i + 2,
                    value: (*f).to_string(),
                })?;
            }
            let [tx, ty, tz, qx, qy, qz, qw] = vals;
            traj.push(StampedPose::new(
                stamp,
                Pose::new(Rotation::from_xyzw(qx, qy, qz, qw), Vec3::new(tx, ty, tz)),
            ));
        }
        Ok(traj)
    }

    /// Convenience: read a TUM file from disk.
    pub fn read_tum_file<P: AsRef<Path>>(path: P) -> Result<Trajectory, TumParseError> {
        let file = std::fs::File::open(path)?;
        Trajectory::read_tum(io::BufReader::new(file))
    }

    /// Write this trajectory in TUM format to any writer.
    pub fn write_tum<W: Write>(&self, mut writer: W) -> io::Result<()> {
        writeln!(writer, "# timestamp tx ty tz qx qy qz qw")?;
        for sp in &self.poses {
            let t = sp.pose.translation();
            let [qx, qy, qz, qw] = sp.pose.rotation().to_xyzw();
            // 9 decimals on time (round-trips our nanosecond stamps); generous precision
            // on the pose so we never quantise away accuracy in the interchange.
            writeln!(
                writer,
                "{} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
                sp.stamp, t.x, t.y, t.z, qx, qy, qz, qw
            )?;
        }
        Ok(())
    }

    /// Convenience: write a TUM file to disk.
    pub fn write_tum_file<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        self.write_tum(io::BufWriter::new(file))
    }
}

impl FromIterator<StampedPose> for Trajectory {
    fn from_iter<I: IntoIterator<Item = StampedPose>>(iter: I) -> Self {
        Trajectory {
            poses: iter.into_iter().collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sample() -> Trajectory {
        let mut t = Trajectory::new();
        t.push(StampedPose::new(
            Stamp::from_seconds(1.0),
            Pose::new(Rotation::identity(), Vec3::new(1.0, 2.0, 3.0)),
        ));
        t.push(StampedPose::new(
            Stamp::from_seconds(2.5),
            Pose::new(
                Rotation::exp(Vec3::new(0.0, 0.0, 0.3)),
                Vec3::new(-4.0, 5.0, 6.0),
            ),
        ));
        t
    }

    #[test]
    fn tum_roundtrip_preserves_poses() {
        let traj = sample();
        let mut buf = Vec::new();
        traj.write_tum(&mut buf).unwrap();

        let parsed = Trajectory::read_tum(buf.as_slice()).unwrap();
        assert_eq!(parsed.len(), traj.len());
        for (a, b) in parsed.poses().iter().zip(traj.poses()) {
            assert_eq!(a.stamp, b.stamp);
            assert_relative_eq!(a.pose.translation(), b.pose.translation(), epsilon = 1e-9);
            assert_relative_eq!(
                a.pose.rotation().log(),
                b.pose.rotation().log(),
                epsilon = 1e-9
            );
        }
    }

    #[test]
    fn read_skips_comments_and_blank_lines() {
        let text = "# header\n\n1.0 0 0 0 0 0 0 1\n  \n2.0 1 1 1 0 0 0 1\n";
        let traj = Trajectory::read_tum(text.as_bytes()).unwrap();
        assert_eq!(traj.len(), 2);
        assert!(traj.is_sorted());
    }

    #[test]
    fn read_rejects_wrong_field_count() {
        let err = Trajectory::read_tum("1.0 0 0 0 0 0 1\n".as_bytes()).unwrap_err();
        assert!(matches!(
            err,
            TumParseError::WrongFieldCount { line: 1, found: 7 }
        ));
    }

    #[test]
    fn read_rejects_non_numeric() {
        let err = Trajectory::read_tum("1.0 0 0 0 0 0 0 nope\n".as_bytes()).unwrap_err();
        assert!(matches!(
            err,
            TumParseError::BadNumber {
                line: 1,
                field: 8,
                ..
            }
        ));
    }
}

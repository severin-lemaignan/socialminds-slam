//! Sensor sample types.
//!
//! These are the inputs the engine consumes: IMU samples (M0 baselines) and 2D laser
//! scans (M3 planar front-end). RGB-D frames and wheel odometry land as the roadmap
//! reaches their front-ends.

use crate::geometry::{Vec2, Vec3};
use crate::time::Stamp;

/// Identifies the sensor frame a measurement is expressed in.
///
/// An index into the rig's frame table: `slam-rig` resolves `header.frame_id` strings
/// against the robot's URDF and hands out these ids (ADR 0009). [`FrameId::BASE`] is
/// the body frame itself — the implicit single-sensor default, identity extrinsic.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FrameId(pub u32);

impl FrameId {
    /// The body frame (`base_link`); measurements in it need no extrinsic.
    pub const BASE: FrameId = FrameId(0);
}

/// A single inertial measurement: angular rate (gyroscope) and proper acceleration
/// (accelerometer), both in the IMU body frame, SI units.
///
/// Sign/frame convention: `accel` is the *specific force* measured by the accelerometer,
/// i.e. it includes the reaction to gravity (a stationary, level IMU reads `+g` upward,
/// `≈ (0, 0, 9.81)`). Strapdown integration subtracts gravity explicitly — see
/// `slam-baseline`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub stamp: Stamp,
    /// The sensor frame the rates/forces are expressed in ([`FrameId::BASE`] when
    /// untagged). Robots may carry several IMUs at different mountings (ADR 0009/0010);
    /// consumers rotate into the base frame through the rig extrinsic.
    pub frame: FrameId,
    /// Angular velocity ω (rad/s), sensor frame.
    pub gyro: Vec3,
    /// Specific force / proper acceleration (m/s²), sensor frame.
    pub accel: Vec3,
}

impl ImuSample {
    /// An untagged (base-frame) sample — the single-centred-IMU default.
    #[inline]
    pub fn new(stamp: Stamp, gyro: Vec3, accel: Vec3) -> Self {
        ImuSample {
            stamp,
            frame: FrameId::BASE,
            gyro,
            accel,
        }
    }

    /// The same measurement, tagged with the sensor frame it was taken in.
    #[inline]
    pub fn in_frame(mut self, frame: FrameId) -> Self {
        self.frame = frame;
        self
    }
}

/// One revolution of a planar laser scanner (`sensor_msgs/LaserScan` shape).
///
/// Beam `i` points at angle `angle_min + i * angle_increment` (rad, counter-clockwise,
/// 0 = sensor +X) and measured `ranges[i]` metres. Ranges are kept as recorded — including
/// NaN/inf/out-of-bounds returns — so no information is destroyed at ingest;
/// [`points`](Self::points) applies the validity filtering.
#[derive(Debug, Clone, PartialEq)]
pub struct LaserScan2D {
    pub stamp: Stamp,
    /// The sensor frame the beams are expressed in ([`FrameId::BASE`] when untagged).
    pub frame: FrameId,
    /// Angle of the first beam (rad).
    pub angle_min: f64,
    /// Angular step between consecutive beams (rad).
    pub angle_increment: f64,
    /// Sensor-reported validity window (m): readings outside are not real returns.
    pub range_min: f64,
    pub range_max: f64,
    /// One range per beam (m), as recorded (`f32` on the wire).
    pub ranges: Vec<f32>,
}

impl LaserScan2D {
    /// Cartesian points in the sensor frame (x forward, y left), keeping only valid
    /// returns: finite and inside `[range_min, range_max]`.
    pub fn points(&self) -> Vec<Vec2> {
        let mut out = Vec::new();
        self.points_into(&mut out);
        out
    }

    /// Like [`points`](Self::points), but reusing `out`'s allocation (hot-path variant:
    /// a scan stream converts at sensor rate, so per-scan allocation is avoidable waste).
    pub fn points_into(&self, out: &mut Vec<Vec2>) {
        out.clear();
        out.extend(self.ranges.iter().enumerate().filter_map(|(i, &r)| {
            let r = r as f64;
            if !r.is_finite() || r < self.range_min || r > self.range_max {
                return None;
            }
            let angle = self.angle_min + i as f64 * self.angle_increment;
            Some(Vec2::new(r * angle.cos(), r * angle.sin()))
        }));
    }
}

/// One wheel-odometry sample (`nav_msgs/Odometry` pose): the platform's own pose
/// estimate in its odometry frame. Consumed as *relative* motion between samples —
/// the motion prior, especially on IMU-less robots (ADR 0012).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdomSample {
    pub stamp: Stamp,
    /// The child frame the pose is for ([`FrameId::BASE`] when untagged).
    pub frame: FrameId,
    pub pose: crate::geometry::Pose,
}

/// A 3D point cloud in a sensor frame — e.g. a back-projected, downsampled RGB-D depth
/// frame (the M4 front-end). Points are finite and range-clipped at ingest.
#[derive(Debug, Clone, PartialEq)]
pub struct PointCloud {
    pub stamp: Stamp,
    /// The sensor frame the points are expressed in ([`FrameId::BASE`] when untagged).
    pub frame: FrameId,
    pub points: Vec<Vec3>,
    /// Optional per-point RGB (parallel to `points`; empty = uncoloured). Carried for
    /// visualization and the future voxel-colour channel — never consumed by
    /// registration.
    pub colors: Vec<[u8; 3]>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_points_filters_invalid_returns_and_places_beams() {
        let scan = LaserScan2D {
            stamp: Stamp::from_nanos(0),
            frame: FrameId::BASE,
            angle_min: 0.0,
            angle_increment: std::f64::consts::FRAC_PI_2,
            range_min: 0.1,
            range_max: 10.0,
            // beam 0: +X; beam 1 invalid (inf); beam 2: too short; beam 3 (−X dir at π…
            // actually 3·π/2 = −Y): valid.
            ranges: vec![2.0, f32::INFINITY, 0.05, 3.0],
        };
        let pts = scan.points();
        assert_eq!(pts.len(), 2);
        assert!((pts[0] - Vec2::new(2.0, 0.0)).norm() < 1e-12);
        assert!((pts[1] - Vec2::new(0.0, -3.0)).norm() < 1e-9);
    }
}

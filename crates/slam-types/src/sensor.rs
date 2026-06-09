//! Sensor sample types.
//!
//! These are the inputs the engine consumes: IMU samples (M0 baselines) and 2D laser
//! scans (M3 planar front-end). RGB-D frames and wheel odometry land as the roadmap
//! reaches their front-ends.

use crate::geometry::{Vec2, Vec3};
use crate::time::Stamp;

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
    /// Angular velocity ω (rad/s), body frame.
    pub gyro: Vec3,
    /// Specific force / proper acceleration (m/s²), body frame.
    pub accel: Vec3,
}

impl ImuSample {
    #[inline]
    pub fn new(stamp: Stamp, gyro: Vec3, accel: Vec3) -> Self {
        ImuSample { stamp, gyro, accel }
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
        self.ranges
            .iter()
            .enumerate()
            .filter_map(|(i, &r)| {
                let r = r as f64;
                if !r.is_finite() || r < self.range_min || r > self.range_max {
                    return None;
                }
                let angle = self.angle_min + i as f64 * self.angle_increment;
                Some(Vec2::new(r * angle.cos(), r * angle.sin()))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_points_filters_invalid_returns_and_places_beams() {
        let scan = LaserScan2D {
            stamp: Stamp::from_nanos(0),
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

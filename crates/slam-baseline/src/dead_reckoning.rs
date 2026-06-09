//! IMU strapdown dead-reckoning baseline.
//!
//! Integrates gyroscope and accelerometer directly, with no other sensor and no bias
//! estimation, so it drifts — that is the point (ADR 0005). It demonstrates the harness
//! catching unbounded-but-motion-tracking error, and sets a floor the real RGB-D-inertial
//! front-end (M3) must beat.
//!
//! ## Convention
//!
//! State is the pose `(R, p)` of the body in a fixed reference frame plus velocity `v`.
//! The accelerometer reports specific force `f_b = Rᵀ (a_world − g_vec)` in the body frame,
//! where `g_vec = (0, 0, −g)` points down (so a level, static IMU reads `(0, 0, +g)`). We
//! recover world acceleration as `a_world = R · f_b + g_vec` and integrate:
//!
//! ```text
//! p ← p + v·dt + ½·a_world·dt²
//! v ← v + a_world·dt
//! R ← R · exp(ω·dt)
//! ```
//!
//! First sample only sets the clock (no span to integrate over).

use slam_types::{ImuSample, Pose, Rotation, Stamp, StampedPose, Vec3};

use crate::SlamSystem;

/// Standard gravity (m/s²).
pub const STANDARD_GRAVITY: f64 = 9.80665;

#[derive(Debug, Clone)]
pub struct ImuDeadReckoning {
    rotation: Rotation,
    position: Vec3,
    velocity: Vec3,
    /// Downward gravity vector in the reference frame, `(0, 0, −g)`.
    gravity: Vec3,
    last_stamp: Option<Stamp>,
}

impl ImuDeadReckoning {
    /// Start at the identity pose, at rest, with standard gravity along −Z.
    pub fn new() -> Self {
        ImuDeadReckoning::with_initial_state(Pose::identity(), Vec3::zeros(), STANDARD_GRAVITY)
    }

    /// Start from a known pose, velocity, and gravity magnitude.
    ///
    /// Supplying the ground-truth initial pose and velocity isolates *drift* from
    /// *initialisation error* when benchmarking.
    pub fn with_initial_state(initial: Pose, velocity: Vec3, gravity_magnitude: f64) -> Self {
        ImuDeadReckoning {
            rotation: initial.rotation(),
            position: initial.translation(),
            velocity,
            gravity: Vec3::new(0.0, 0.0, -gravity_magnitude),
            last_stamp: None,
        }
    }

    fn pose(&self) -> Pose {
        Pose::new(self.rotation, self.position)
    }
}

impl Default for ImuDeadReckoning {
    fn default() -> Self {
        ImuDeadReckoning::new()
    }
}

impl SlamSystem for ImuDeadReckoning {
    fn name(&self) -> &str {
        "imu_dead_reckoning"
    }

    fn process_imu(&mut self, sample: &ImuSample) {
        let Some(last) = self.last_stamp else {
            // First sample: nothing to integrate yet, just start the clock.
            self.last_stamp = Some(sample.stamp);
            return;
        };

        let dt = (sample.stamp - last).as_seconds();
        self.last_stamp = Some(sample.stamp);
        if dt <= 0.0 {
            // Duplicate or out-of-order stamp; skip integration but keep the latest clock.
            return;
        }

        // World-frame acceleration from the measured specific force.
        let a_world = self.rotation.rotate(sample.accel) + self.gravity;

        // Translational state (constant-acceleration over the step).
        self.position += self.velocity * dt + 0.5 * a_world * dt * dt;
        self.velocity += a_world * dt;

        // Orientation: integrate the body angular rate.
        self.rotation = self.rotation * Rotation::exp(sample.gyro * dt);
    }

    fn current_estimate(&self) -> Option<StampedPose> {
        self.last_stamp
            .map(|stamp| StampedPose::new(stamp, self.pose()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// A perfectly static, level IMU reads `(0, 0, +g)` and zero rotation; the estimate
    /// must not drift.
    #[test]
    fn static_imu_does_not_move() {
        let mut dr = ImuDeadReckoning::new();
        let accel = Vec3::new(0.0, 0.0, STANDARD_GRAVITY);
        for i in 0..1000 {
            dr.process_imu(&ImuSample::new(
                Stamp::from_seconds(i as f64 * 0.001),
                Vec3::zeros(),
                accel,
            ));
        }
        let est = dr.current_estimate().unwrap();
        assert_relative_eq!(est.pose.translation(), Vec3::zeros(), epsilon = 1e-9);
        assert_relative_eq!(est.pose.rotation().log(), Vec3::zeros(), epsilon = 1e-9);
    }

    /// Constant proper acceleration along +X for 1 s should give p ≈ ½·a·t² under exact
    /// integration. Specific force must include the gravity reaction on Z.
    #[test]
    fn constant_acceleration_matches_kinematics() {
        let mut dr = ImuDeadReckoning::new();
        let a = 2.0; // m/s² along world +X
        let accel = Vec3::new(a, 0.0, STANDARD_GRAVITY);
        let n = 1000;
        let dt = 0.001;
        // i=0 primes the clock; i=1..=n integrate, spanning a full second.
        for i in 0..=n {
            dr.process_imu(&ImuSample::new(
                Stamp::from_seconds(i as f64 * dt),
                Vec3::zeros(),
                accel,
            ));
        }
        let est = dr.current_estimate().unwrap();
        // Euler-integrated position is slightly above the closed form; check order of
        // magnitude and that Y/Z stayed put.
        assert!((est.pose.translation().x - 0.5 * a * 1.0_f64.powi(2)).abs() < 0.01);
        assert_relative_eq!(est.pose.translation().y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(est.pose.translation().z, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn pure_yaw_rate_integrates_to_heading() {
        let mut dr = ImuDeadReckoning::new();
        let rate = 1.0; // rad/s about Z
        let accel = Vec3::new(0.0, 0.0, STANDARD_GRAVITY);
        let n = 1000;
        let dt = 0.001;
        // i=0 primes the clock; i=1..=n integrate, spanning a full second.
        for i in 0..=n {
            dr.process_imu(&ImuSample::new(
                Stamp::from_seconds(i as f64 * dt),
                Vec3::new(0.0, 0.0, rate),
                accel,
            ));
        }
        // ~1 rad of yaw after 1 s.
        let yaw = dr.current_estimate().unwrap().pose.rotation().log().z;
        assert_relative_eq!(yaw, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn none_before_any_input() {
        assert!(ImuDeadReckoning::new().current_estimate().is_none());
    }
}

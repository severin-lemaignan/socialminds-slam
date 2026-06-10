//! Gravity-referenced attitude (roll/pitch) from the IMU — the tilt-compensation
//! prerequisite of ADR 0010: the base is a 3D body whose scan plane tilts under
//! acceleration, and only the IMU can observe that tilt.
//!
//! A complementary filter: gyro integration propagates orientation (fast, drifts),
//! the accelerometer's gravity direction corrects it (slow, but only trusted when the
//! measured specific-force magnitude is close to g — a hard gate against mistaking
//! linear acceleration for tilt). Yaw is unobservable from gravity and deliberately
//! never consumed: [`AttitudeFilter::tilt`] strips it.

use slam_types::{ImuSample, Rotation, Stamp, Vec3};

/// Tuning for [`AttitudeFilter`].
#[derive(Debug, Clone)]
pub struct AttitudeConfig {
    /// Gravity-correction gain (1/s): the inverse time constant of the accelerometer
    /// pulling the attitude towards measured gravity.
    pub accel_gain: f64,
    /// Accept an accel sample as a gravity observation only when its magnitude is
    /// within this of g (m/s²) — otherwise the base is accelerating and the sample
    /// would alias linear acceleration into tilt.
    pub accel_norm_tolerance: f64,
    /// Gravity magnitude (m/s²).
    pub gravity: f64,
}

impl Default for AttitudeConfig {
    fn default() -> Self {
        AttitudeConfig {
            accel_gain: 0.5,
            accel_norm_tolerance: 0.5,
            gravity: 9.80665,
        }
    }
}

/// Complementary roll/pitch filter over an IMU stream.
#[derive(Debug, Clone)]
pub struct AttitudeFilter {
    cfg: AttitudeConfig,
    /// `R_world_body` up to an arbitrary, drifting yaw.
    q: Rotation,
    last_stamp: Option<Stamp>,
    initialized: bool,
}

impl AttitudeFilter {
    pub fn new(cfg: AttitudeConfig) -> Self {
        AttitudeFilter {
            cfg,
            q: Rotation::identity(),
            last_stamp: None,
            initialized: false,
        }
    }

    /// Whether at least one gravity observation has anchored roll/pitch. Before this,
    /// [`tilt`](Self::tilt) is the identity (the planar assumption, unchanged).
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Ingest one IMU sample (must arrive in non-decreasing stamp order).
    pub fn process(&mut self, sample: &ImuSample) {
        let dt = match self.last_stamp {
            // Clamp: a gap in the stream must not integrate one stale rate over seconds.
            Some(prev) => (sample.stamp - prev).as_seconds().clamp(0.0, 0.1),
            None => 0.0,
        };
        self.last_stamp = Some(sample.stamp);

        // Propagate with the gyro: q ← q · exp(ω dt) (body rates).
        if dt > 0.0 {
            self.q = self.q * Rotation::exp(sample.gyro * dt);
        }

        // Gravity correction. A stationary, level IMU reads +g "up" in the body frame
        // (specific force, see slam_types::ImuSample), so the measured body-frame up is
        // accel/|accel|; the predicted one is q⁻¹·ez. The small-rotation correction in
        // the body frame taking predicted to measured is δ = measured × predicted
        // (|δ| = sin∠): q ← q · exp(δ · gain·dt), or the full angle at initialisation.
        let norm = sample.accel.norm();
        if (norm - self.cfg.gravity).abs() > self.cfg.accel_norm_tolerance {
            return;
        }
        let measured = sample.accel / norm;
        let predicted = self.q.inverse().rotate(Vec3::z());
        let cross = measured.cross(&predicted);
        let sin_angle = cross.norm();
        if sin_angle < 1e-12 {
            self.initialized = true;
            return;
        }
        let delta = if self.initialized {
            cross * (self.cfg.accel_gain * dt).min(1.0)
        } else {
            // First gravity observation: snap the full angle.
            cross * (sin_angle.atan2(measured.dot(&predicted)) / sin_angle)
        };
        self.q = self.q * Rotation::exp(delta);
        self.initialized = true;
    }

    /// The gravity-aligned tilt: rotation mapping body coordinates into the
    /// gravity-aligned frame, with yaw stripped (roll/pitch only — yaw drifts here and
    /// belongs to the scan matcher). Identity until initialised.
    pub fn tilt(&self) -> Rotation {
        if !self.initialized {
            return Rotation::identity();
        }
        let (roll, pitch, _yaw) = self.q.to_rpy();
        Rotation::from_rpy(roll, pitch, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use slam_types::Stamp;
    use std::f64::consts::PI;

    const G: f64 = 9.80665;

    fn sample(t: f64, gyro: Vec3, accel: Vec3) -> ImuSample {
        ImuSample::new(Stamp::from_seconds(t), gyro, accel)
    }

    /// Body-frame gravity reading for a body rolled by `roll` and pitched by `pitch`.
    fn tilted_gravity(roll: f64, pitch: f64) -> Vec3 {
        Rotation::from_rpy(roll, pitch, 0.0)
            .inverse()
            .rotate(Vec3::new(0.0, 0.0, G))
    }

    #[test]
    fn level_imu_keeps_identity_tilt() {
        let mut f = AttitudeFilter::new(AttitudeConfig::default());
        for k in 0..100 {
            f.process(&sample(
                k as f64 * 0.005,
                Vec3::zeros(),
                Vec3::new(0.0, 0.0, G),
            ));
        }
        assert!(f.is_initialized());
        let (r, p, _) = f.tilt().to_rpy();
        assert!(r.abs() < 1e-9 && p.abs() < 1e-9);
    }

    #[test]
    fn first_gravity_observation_snaps_to_the_tilt() {
        let mut f = AttitudeFilter::new(AttitudeConfig::default());
        let (roll, pitch) = (0.03, -0.02);
        f.process(&sample(0.0, Vec3::zeros(), tilted_gravity(roll, pitch)));
        let (r, p, _) = f.tilt().to_rpy();
        assert!((r - roll).abs() < 1e-9, "roll {r} != {roll}");
        assert!((p - pitch).abs() < 1e-9, "pitch {p} != {pitch}");
    }

    #[test]
    fn gyro_tracks_a_tilt_ramp_between_gravity_updates() {
        let mut f = AttitudeFilter::new(AttitudeConfig::default());
        f.process(&sample(0.0, Vec3::zeros(), Vec3::new(0.0, 0.0, G)));
        // Roll at 0.1 rad/s for 0.2 s; accel deliberately out-of-tolerance (accelerating)
        // so only the gyro acts.
        let bogus = Vec3::new(0.0, 0.0, G + 2.0);
        for k in 1..=40 {
            f.process(&sample(k as f64 * 0.005, Vec3::new(0.1, 0.0, 0.0), bogus));
        }
        let (r, _, _) = f.tilt().to_rpy();
        assert!((r - 0.02).abs() < 1e-4, "roll {r} != 0.02");
    }

    #[test]
    fn accel_correction_converges_after_gyro_drift() {
        let mut f = AttitudeFilter::new(AttitudeConfig::default());
        f.process(&sample(0.0, Vec3::zeros(), Vec3::new(0.0, 0.0, G)));
        // Biased gyro drifts the roll; steady gravity pulls it back. With gain 0.5 the
        // steady-state error is bias/gain = 0.002/0.5 = 4 mrad.
        for k in 1..=4000 {
            f.process(&sample(
                k as f64 * 0.005,
                Vec3::new(0.002, 0.0, 0.0),
                Vec3::new(0.0, 0.0, G),
            ));
        }
        let (r, _, _) = f.tilt().to_rpy();
        assert!(r.abs() < 0.006, "roll did not stay bounded: {r}");
    }

    #[test]
    fn out_of_tolerance_accel_never_initializes() {
        let mut f = AttitudeFilter::new(AttitudeConfig::default());
        f.process(&sample(0.0, Vec3::zeros(), Vec3::new(5.0, 0.0, G)));
        assert!(!f.is_initialized());
        let (r, p, _) = f.tilt().to_rpy();
        assert_eq!((r, p), (0.0, 0.0));
    }

    #[test]
    fn yaw_is_stripped_from_tilt() {
        let mut f = AttitudeFilter::new(AttitudeConfig::default());
        f.process(&sample(0.0, Vec3::zeros(), Vec3::new(0.0, 0.0, G)));
        // Spin in yaw for a while (gravity unchanged by yaw).
        for k in 1..=100 {
            f.process(&sample(
                k as f64 * 0.005,
                Vec3::new(0.0, 0.0, PI),
                Vec3::new(0.0, 0.0, G),
            ));
        }
        let (r, p, _) = f.tilt().to_rpy();
        assert!(
            r.abs() < 1e-6 && p.abs() < 1e-6,
            "tilt polluted by yaw: {r} {p}"
        );
    }
}

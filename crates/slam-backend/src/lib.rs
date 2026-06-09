//! Safe factor-graph backend over the wrapped GTSAM optimiser.
//!
//! This crate is the **only** consumer of `slam-gtsam-sys` (ADR 0001): the rest of the
//! engine speaks `slam-types` and this API, so the optimiser can be swapped for a
//! pure-Rust one later with no upstream churn.
//!
//! The model is deliberately simple for M2: build a [`FactorGraph`] (values + factors),
//! call [`FactorGraph::optimize`] (Levenberg-Marquardt), read back the optimised values.
//! Incremental smoothing (iSAM2) and robust kernels arrive with the front-ends (M3/M4).
//!
//! # Keys
//! Variables are addressed by typed [`Key`]s in GTSAM `Symbol` convention:
//! [`Key::pose`] (`x`), [`Key::velocity`] (`v`), [`Key::bias`] (`b`).
//!
//! # Example: a two-pose graph
//! ```
//! use slam_backend::{FactorGraph, Key, PoseNoise};
//! use slam_types::geometry::{Pose, Rotation, Vec3};
//!
//! let mut graph = FactorGraph::new();
//! let noise = PoseNoise::isotropic(0.01, 0.05);
//! let step = Pose::new(Rotation::identity(), Vec3::new(1.0, 0.0, 0.0));
//!
//! graph.insert_pose(Key::pose(0), &Pose::identity());
//! graph.insert_pose(Key::pose(1), &step);
//! graph.add_pose_prior(Key::pose(0), &Pose::identity(), &noise);
//! graph.add_between(Key::pose(0), Key::pose(1), &step, &noise);
//! let report = graph.optimize(100).unwrap();
//! assert!(report.final_error < 1e-9);
//! ```

#![forbid(unsafe_code)]

use slam_gtsam_sys::{ffi, symbol};
use slam_types::geometry::{Pose, Rotation, Vec3};

/// Errors surfaced by the backend (GTSAM exceptions crossing the bridge, mostly).
#[derive(Debug, thiserror::Error)]
pub enum BackendError {
    #[error("optimisation failed: {0}")]
    Optimize(String),
    #[error("unknown key {0:?}: {1}")]
    UnknownKey(Key, String),
}

/// A typed factor-graph variable key (GTSAM `Symbol` convention).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Key(u64);

impl Key {
    /// A body pose variable: `x(i)`.
    pub fn pose(index: u64) -> Key {
        Key(symbol(b'x', index))
    }

    /// A linear-velocity variable (world frame): `v(i)`.
    pub fn velocity(index: u64) -> Key {
        Key(symbol(b'v', index))
    }

    /// An IMU-bias variable: `b(i)`.
    pub fn bias(index: u64) -> Key {
        Key(symbol(b'b', index))
    }
}

/// Diagonal noise for an SE(3) factor, as standard deviations.
#[derive(Debug, Clone, Copy)]
pub struct PoseNoise {
    /// Rotation sigmas (rad), one per axis.
    pub rotation: Vec3,
    /// Translation sigmas (m), one per axis.
    pub translation: Vec3,
}

impl PoseNoise {
    /// Same sigma on every rotation axis (rad) and every translation axis (m).
    pub fn isotropic(rotation: f64, translation: f64) -> Self {
        PoseNoise {
            rotation: Vec3::from_element(rotation),
            translation: Vec3::from_element(translation),
        }
    }

    /// GTSAM Pose3 tangent ordering: rotation first, then translation.
    fn sigmas(&self) -> [f64; 6] {
        [
            self.rotation.x,
            self.rotation.y,
            self.rotation.z,
            self.translation.x,
            self.translation.y,
            self.translation.z,
        ]
    }
}

/// IMU noise densities + gravity, the parameters of preintegration.
///
/// Sigmas are continuous-time noise densities as in the sensor datasheet (EuRoC-style):
/// accel in m/s²/√Hz, gyro in rad/s/√Hz.
#[derive(Debug, Clone, Copy)]
pub struct ImuParams {
    pub accel_sigma: f64,
    pub gyro_sigma: f64,
    /// Integration error growth; small (e.g. 1e-8) unless tuned.
    pub integration_sigma: f64,
    /// Gravity magnitude; the world is Z-up with gravity `(0, 0, -gravity)`.
    pub gravity: f64,
}

impl Default for ImuParams {
    fn default() -> Self {
        ImuParams {
            accel_sigma: 1e-3,
            gyro_sigma: 1e-4,
            integration_sigma: 1e-8,
            gravity: 9.81,
        }
    }
}

/// A constant accelerometer + gyroscope bias.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuBias {
    pub accel: Vec3,
    pub gyro: Vec3,
}

/// Pose + velocity: the state an IMU factor connects.
#[derive(Debug, Clone, Copy)]
pub struct NavState {
    pub pose: Pose,
    pub velocity: Vec3,
}

/// Outcome of an optimisation run — instrumented at the source, because the backend is a
/// hot path and "extremely fast" is a project requirement: every solve reports what it
/// cost, so the harness (and later the pipeline telemetry) can regress on it.
#[derive(Debug, Clone, Copy)]
pub struct OptimizeReport {
    /// Total weighted graph error before optimising.
    pub initial_error: f64,
    /// Total weighted graph error at the optimum (0 = all factors satisfied exactly).
    pub final_error: f64,
    /// Levenberg-Marquardt iterations actually performed.
    pub iterations: u64,
    /// Wall time of the solve (measured around the FFI call).
    pub duration: std::time::Duration,
}

fn to_ffi(pose: &Pose) -> ffi::FfiPose {
    let t = pose.translation();
    ffi::FfiPose {
        t: [t.x, t.y, t.z],
        q: pose.rotation().to_xyzw(),
    }
}

fn from_ffi(pose: &ffi::FfiPose) -> Pose {
    Pose::new(
        Rotation::from_xyzw(pose.q[0], pose.q[1], pose.q[2], pose.q[3]),
        Vec3::new(pose.t[0], pose.t[1], pose.t[2]),
    )
}

fn vec3_array(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Accumulates IMU samples between two states into one preintegrated constraint.
///
/// Feed it every IMU sample observed between pose `i` and pose `j`, then either
/// [`predict`](Self::predict) the next state (odometry-style) or hand it to
/// [`FactorGraph::add_imu_factor`] as a graph constraint.
pub struct ImuPreintegrator {
    inner: cxx::UniquePtr<ffi::Preintegrator>,
}

impl ImuPreintegrator {
    pub fn new(params: &ImuParams, bias: &ImuBias) -> Self {
        ImuPreintegrator {
            inner: ffi::new_preintegrator(
                params.accel_sigma,
                params.gyro_sigma,
                params.integration_sigma,
                params.gravity,
                &vec3_array(bias.accel),
                &vec3_array(bias.gyro),
            ),
        }
    }

    /// Accumulate one sample: body-frame specific force (gravity included) and angular
    /// rate, held constant over `dt` seconds. `dt` must be positive.
    pub fn integrate(&mut self, accel: Vec3, gyro: Vec3, dt: f64) {
        self.inner
            .pin_mut()
            .integrate(&vec3_array(accel), &vec3_array(gyro), dt)
            .expect("non-positive dt in IMU preintegration");
    }

    /// Drop everything accumulated so far (for reuse between keyframe pairs).
    pub fn reset(&mut self) {
        self.inner.pin_mut().reset();
    }

    /// Total integrated time (seconds).
    pub fn delta_t(&self) -> f64 {
        self.inner.delta_t()
    }

    /// Propagate `state` through the accumulated IMU delta (dead-reckoning step).
    pub fn predict(&self, state: &NavState) -> NavState {
        let out = self.inner.predict(&ffi::FfiNavState {
            pose: to_ffi(&state.pose),
            velocity: vec3_array(state.velocity),
        });
        NavState {
            pose: from_ffi(&out.pose),
            velocity: Vec3::new(out.velocity[0], out.velocity[1], out.velocity[2]),
        }
    }
}

/// A nonlinear factor graph + initial values, optimised by Levenberg-Marquardt.
pub struct FactorGraph {
    inner: cxx::UniquePtr<ffi::GraphBuilder>,
}

impl Default for FactorGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl FactorGraph {
    pub fn new() -> Self {
        FactorGraph {
            inner: ffi::new_graph_builder(),
        }
    }

    // ---- Initial values ------------------------------------------------------------

    pub fn insert_pose(&mut self, key: Key, pose: &Pose) {
        self.inner.pin_mut().insert_pose(key.0, &to_ffi(pose));
    }

    pub fn insert_velocity(&mut self, key: Key, velocity: Vec3) {
        self.inner
            .pin_mut()
            .insert_velocity(key.0, &vec3_array(velocity));
    }

    pub fn insert_bias(&mut self, key: Key, bias: &ImuBias) {
        self.inner
            .pin_mut()
            .insert_bias(key.0, &vec3_array(bias.accel), &vec3_array(bias.gyro));
    }

    // ---- Factors --------------------------------------------------------------------

    /// Anchor `key` at `pose` (gauge freedom: every graph needs at least one prior).
    pub fn add_pose_prior(&mut self, key: Key, pose: &Pose, noise: &PoseNoise) {
        self.inner
            .pin_mut()
            .add_prior_pose(key.0, &to_ffi(pose), &noise.sigmas());
    }

    /// Relative-pose (odometry / loop-closure) constraint:
    /// `pose(to) = pose(from) * relative`.
    pub fn add_between(&mut self, from: Key, to: Key, relative: &Pose, noise: &PoseNoise) {
        self.inner
            .pin_mut()
            .add_between_pose(from.0, to.0, &to_ffi(relative), &noise.sigmas());
    }

    pub fn add_velocity_prior(&mut self, key: Key, velocity: Vec3, sigma: f64) {
        self.inner.pin_mut().add_prior_velocity(
            key.0,
            &vec3_array(velocity),
            &[sigma, sigma, sigma],
        );
    }

    pub fn add_bias_prior(&mut self, key: Key, bias: &ImuBias, sigma: f64) {
        self.inner.pin_mut().add_prior_bias(
            key.0,
            &vec3_array(bias.accel),
            &vec3_array(bias.gyro),
            &[sigma; 6],
        );
    }

    /// Constrain `(pose_i, vel_i) → (pose_j, vel_j)` by the preintegrated IMU delta;
    /// `bias` is the (shared) bias variable the residual is evaluated at.
    pub fn add_imu_factor(
        &mut self,
        pose_i: Key,
        velocity_i: Key,
        pose_j: Key,
        velocity_j: Key,
        bias: Key,
        preintegrated: &ImuPreintegrator,
    ) -> Result<(), BackendError> {
        self.inner
            .pin_mut()
            .add_imu_factor(
                pose_i.0,
                velocity_i.0,
                pose_j.0,
                velocity_j.0,
                bias.0,
                &preintegrated.inner,
            )
            .map_err(|e| BackendError::Optimize(e.to_string()))
    }

    // ---- Solve & read back ------------------------------------------------------------

    /// Levenberg-Marquardt to convergence (or `max_iterations`). The optimised values
    /// replace the initial estimates.
    pub fn optimize(&mut self, max_iterations: u32) -> Result<OptimizeReport, BackendError> {
        let start = std::time::Instant::now();
        let stats = self
            .inner
            .pin_mut()
            .optimize(max_iterations)
            .map_err(|e| BackendError::Optimize(e.to_string()))?;
        Ok(OptimizeReport {
            initial_error: stats.initial_error,
            final_error: stats.final_error,
            iterations: stats.iterations,
            duration: start.elapsed(),
        })
    }

    pub fn pose(&self, key: Key) -> Result<Pose, BackendError> {
        self.inner
            .pose_at(key.0)
            .map(|p| from_ffi(&p))
            .map_err(|e| BackendError::UnknownKey(key, e.to_string()))
    }

    pub fn velocity(&self, key: Key) -> Result<Vec3, BackendError> {
        self.inner
            .velocity_at(key.0)
            .map(|v| Vec3::new(v.v[0], v.v[1], v.v[2]))
            .map_err(|e| BackendError::UnknownKey(key, e.to_string()))
    }

    pub fn num_factors(&self) -> usize {
        self.inner.num_factors()
    }

    pub fn num_values(&self) -> usize {
        self.inner.num_values()
    }
}

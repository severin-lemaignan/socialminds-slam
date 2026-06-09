//! Raw cxx bridge to GTSAM. **Do not use directly** — `slam-backend` is the safe API.
//!
//! The surface is deliberately narrow (ADR 0001): the Rust side owns graph topology and
//! data; GTSAM is called as a solver. Conventions at the boundary:
//!
//! - Poses cross as translation + quaternion `(x, y, z, w)` — the TUM/ROS ordering used
//!   throughout `slam-types`.
//! - Keys are raw GTSAM `Key`s (`u64`); build them with [`symbol`].
//! - 6-vector sigmas for SE(3) noise follow GTSAM's Pose3 tangent order:
//!   **rotation (rx, ry, rz) first, then translation (tx, ty, tz)**.
//! - C++ exceptions surface as `Err(cxx::Exception)` on the `Result` functions.

#[cxx::bridge(namespace = "slam_gtsam")]
pub mod ffi {
    /// SE(3) pose: translation + unit quaternion in `(x, y, z, w)` order.
    #[derive(Debug, Clone, Copy)]
    pub struct FfiPose {
        pub t: [f64; 3],
        pub q: [f64; 4],
    }

    /// Pose + linear velocity (world frame): GTSAM's `NavState`.
    #[derive(Debug, Clone, Copy)]
    pub struct FfiNavState {
        pub pose: FfiPose,
        pub velocity: [f64; 3],
    }

    /// A bare 3-vector return value (cxx cannot return arrays directly).
    #[derive(Debug, Clone, Copy)]
    pub struct FfiVec3 {
        pub v: [f64; 3],
    }

    /// What a solve cost and achieved — the backend is a hot path, so every call is
    /// instrumented at the source (wall time is measured on the Rust side).
    #[derive(Debug, Clone, Copy)]
    pub struct FfiOptimizeStats {
        pub initial_error: f64,
        pub final_error: f64,
        pub iterations: u64,
    }

    unsafe extern "C++" {
        include!("slam-gtsam-sys/cpp/shim.h");

        // ---- Factor graph + values + Levenberg-Marquardt ------------------------------
        type GraphBuilder;

        fn new_graph_builder() -> UniquePtr<GraphBuilder>;

        fn insert_pose(self: Pin<&mut GraphBuilder>, key: u64, pose: &FfiPose);
        fn insert_velocity(self: Pin<&mut GraphBuilder>, key: u64, velocity: &[f64; 3]);
        fn insert_bias(
            self: Pin<&mut GraphBuilder>,
            key: u64,
            accel_bias: &[f64; 3],
            gyro_bias: &[f64; 3],
        );

        fn add_prior_pose(
            self: Pin<&mut GraphBuilder>,
            key: u64,
            pose: &FfiPose,
            sigmas: &[f64; 6],
        );
        fn add_between_pose(
            self: Pin<&mut GraphBuilder>,
            key_from: u64,
            key_to: u64,
            relative: &FfiPose,
            sigmas: &[f64; 6],
        );
        fn add_prior_velocity(
            self: Pin<&mut GraphBuilder>,
            key: u64,
            velocity: &[f64; 3],
            sigmas: &[f64; 3],
        );
        fn add_prior_bias(
            self: Pin<&mut GraphBuilder>,
            key: u64,
            accel_bias: &[f64; 3],
            gyro_bias: &[f64; 3],
            sigmas: &[f64; 6],
        );
        fn add_imu_factor(
            self: Pin<&mut GraphBuilder>,
            pose_i: u64,
            velocity_i: u64,
            pose_j: u64,
            velocity_j: u64,
            bias: u64,
            preintegrated: &Preintegrator,
        ) -> Result<()>;

        /// Optimise with Levenberg-Marquardt. The optimised values replace the initial
        /// estimates (read back via `pose_at` & co).
        fn optimize(self: Pin<&mut GraphBuilder>, max_iterations: u32) -> Result<FfiOptimizeStats>;

        fn pose_at(self: &GraphBuilder, key: u64) -> Result<FfiPose>;
        fn velocity_at(self: &GraphBuilder, key: u64) -> Result<FfiVec3>;

        fn num_factors(self: &GraphBuilder) -> usize;
        fn num_values(self: &GraphBuilder) -> usize;

        // ---- IMU preintegration --------------------------------------------------------
        type Preintegrator;

        /// `gravity` is the magnitude (m/s²) of a `(0, 0, -gravity)` world gravity vector
        /// (Z-up, matching the engine convention).
        fn new_preintegrator(
            accel_sigma: f64,
            gyro_sigma: f64,
            integration_sigma: f64,
            gravity: f64,
            accel_bias: &[f64; 3],
            gyro_bias: &[f64; 3],
        ) -> UniquePtr<Preintegrator>;

        /// Accumulate one IMU sample: specific force (m/s², gravity included) and angular
        /// rate (rad/s), both in the body frame, over `dt` seconds.
        fn integrate(
            self: Pin<&mut Preintegrator>,
            accel: &[f64; 3],
            gyro: &[f64; 3],
            dt: f64,
        ) -> Result<()>;

        fn reset(self: Pin<&mut Preintegrator>);
        fn delta_t(self: &Preintegrator) -> f64;

        /// Propagate a state through the preintegrated delta (uses the bias fixed at
        /// construction).
        fn predict(self: &Preintegrator, state: &FfiNavState) -> FfiNavState;
    }
}

/// Build a GTSAM `Symbol` key: a one-character kind tag + index, e.g. `symbol(b'x', 7)`.
///
/// Mirrors `gtsam::Symbol`'s layout (tag in the top 8 bits) without crossing the FFI.
#[inline]
pub fn symbol(tag: u8, index: u64) -> u64 {
    debug_assert!(index < (1 << 56), "symbol index overflows 56 bits");
    ((tag as u64) << 56) | index
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symbol_matches_gtsam_layout() {
        assert_eq!(symbol(b'x', 0), 0x7800_0000_0000_0000);
        assert_eq!(symbol(b'x', 42), 0x7800_0000_0000_002A);
    }

    #[test]
    fn graph_builder_smoke() {
        let mut graph = ffi::new_graph_builder();
        let origin = ffi::FfiPose {
            t: [0.0; 3],
            q: [0.0, 0.0, 0.0, 1.0],
        };
        graph.pin_mut().insert_pose(symbol(b'x', 0), &origin);
        graph
            .pin_mut()
            .add_prior_pose(symbol(b'x', 0), &origin, &[0.1; 6]);
        assert_eq!(graph.num_factors(), 1);
        assert_eq!(graph.num_values(), 1);
        let stats = graph.pin_mut().optimize(10).expect("optimise");
        assert!(
            stats.final_error < 1e-9,
            "prior-only graph should reach zero error: {}",
            stats.final_error
        );
        let pose = graph.pose_at(symbol(b'x', 0)).expect("pose");
        assert!(pose.t.iter().all(|v| v.abs() < 1e-9));
    }

    #[test]
    fn missing_key_is_an_error_not_a_crash() {
        let graph = ffi::new_graph_builder();
        assert!(graph.pose_at(symbol(b'x', 99)).is_err());
    }
}

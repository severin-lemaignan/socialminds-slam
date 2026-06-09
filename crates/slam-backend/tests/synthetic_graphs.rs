//! M2 acceptance tests (ROADMAP): synthetic graphs with known solutions must optimise to
//! ground truth within tolerance, CPU-only.
//!
//! Two families: pose graphs (drifted odometry + a loop-closure constraint), and IMU
//! preintegration (analytically known motion → predicted / optimised states).

use approx::assert_relative_eq;
use slam_backend::{FactorGraph, ImuBias, ImuParams, ImuPreintegrator, Key, NavState, PoseNoise};
use slam_types::geometry::{Pose, Rotation, Vec3};
use std::f64::consts::FRAC_PI_2;

fn assert_pose_close(actual: &Pose, expected: &Pose, tol_t: f64, tol_r: f64) {
    let dt = (actual.translation() - expected.translation()).norm();
    let dr = (expected.rotation().inverse() * actual.rotation())
        .log()
        .norm();
    assert!(
        dt < tol_t && dr < tol_r,
        "pose mismatch: |Δt| = {dt:.2e} (tol {tol_t:.0e}), |Δr| = {dr:.2e} (tol {tol_r:.0e})"
    );
}

/// Ground truth: a 10 m × 10 m square loop in the XY plane, one 90° left turn per corner.
/// The relative motion between consecutive corners is identical: advance 10 m along
/// local +X, then yaw +90°.
fn square_loop_ground_truth() -> (Vec<Pose>, Pose) {
    let step = Pose::new(
        Rotation::exp(Vec3::new(0.0, 0.0, FRAC_PI_2)),
        Vec3::new(10.0, 0.0, 0.0),
    );
    let mut poses = vec![Pose::identity()];
    for i in 0..3 {
        poses.push(poses[i] * step);
    }
    (poses, step)
}

#[test]
fn pose_graph_with_exact_odometry_recovers_drifted_initials() {
    let (gt, step) = square_loop_ground_truth();
    let noise = PoseNoise::isotropic(0.01, 0.05);
    let mut graph = FactorGraph::new();

    // Initial estimates: ground truth corrupted by growing translation drift.
    for (i, pose) in gt.iter().enumerate() {
        let drift = Vec3::new(0.3, -0.2, 0.1) * i as f64;
        let drifted = Pose::new(pose.rotation(), pose.translation() + drift);
        graph.insert_pose(Key::pose(i as u64), &drifted);
    }

    graph.add_pose_prior(Key::pose(0), &gt[0], &noise);
    for i in 0..3 {
        graph.add_between(Key::pose(i), Key::pose(i + 1), &step, &noise);
    }

    let report = graph.optimize(100).expect("optimisation must converge");
    assert!(report.final_error < 1e-9, "error: {}", report.final_error);
    for (i, expected) in gt.iter().enumerate() {
        let actual = graph.pose(Key::pose(i as u64)).unwrap();
        assert_pose_close(&actual, expected, 1e-6, 1e-8);
    }
}

#[test]
fn loop_closure_corrects_drifted_odometry() {
    let (gt, step) = square_loop_ground_truth();
    let mut graph = FactorGraph::new();

    // Odometry is *biased*: each between-measurement overshoots by 40 cm and
    // under-rotates by 2°, so the open chain drifts badly.
    let bad_step = Pose::new(
        Rotation::exp(Vec3::new(0.0, 0.0, FRAC_PI_2 - 0.035)),
        Vec3::new(10.4, 0.0, 0.0),
    );
    let odo_noise = PoseNoise::isotropic(0.05, 0.3);

    let mut open_chain = vec![Pose::identity()];
    for i in 0..3 {
        open_chain.push(open_chain[i] * bad_step);
    }
    for (i, pose) in open_chain.iter().enumerate() {
        graph.insert_pose(Key::pose(i as u64), pose);
    }

    graph.add_pose_prior(Key::pose(0), &gt[0], &PoseNoise::isotropic(1e-4, 1e-4));
    for i in 0..3 {
        graph.add_between(Key::pose(i), Key::pose(i + 1), &bad_step, &odo_noise);
    }

    // The open-chain estimate of the last corner is visibly off ground truth.
    let open_error = (open_chain[3].translation() - gt[3].translation()).norm();
    assert!(open_error > 1.0, "drift should be large: {open_error}");

    // Loop closure: an accurate measurement from corner 3 back to corner 0
    // (pose(0) = pose(3) * step, by the square's symmetry).
    graph.add_between(
        Key::pose(3),
        Key::pose(0),
        &step,
        &PoseNoise::isotropic(0.005, 0.02),
    );

    graph.optimize(100).expect("optimisation must converge");

    // Uniformly biased odometry is the worst case for a pose graph: the loop constraint
    // pins the chain's ends and *redistributes* the bias over the middle corners rather
    // than eliminating it. The meaningful claim is that worst-case drift at least halves.
    let worst = gt
        .iter()
        .enumerate()
        .map(|(i, expected)| {
            let actual = graph.pose(Key::pose(i as u64)).unwrap();
            (actual.translation() - expected.translation()).norm()
        })
        .fold(0.0, f64::max);
    assert!(
        worst < open_error / 2.0,
        "worst corner still {worst:.2} m off after loop closure (open-chain: {open_error:.2} m)"
    );
    // And the loop must actually close: corner 3 + step lands on corner 0.
    let closed = graph.pose(Key::pose(3)).unwrap() * step;
    assert_pose_close(&closed, &graph.pose(Key::pose(0)).unwrap(), 0.05, 0.05);
}

/// Stationary IMU, perfect gravity reading: preintegrating must predict "still here".
#[test]
fn preintegration_stationary() {
    let params = ImuParams::default();
    let mut pim = ImuPreintegrator::new(&params, &ImuBias::default());

    // Body level, world Z-up: the accelerometer reads +g on z (specific force).
    let f_gravity = Vec3::new(0.0, 0.0, params.gravity);
    for _ in 0..200 {
        pim.integrate(f_gravity, Vec3::zeros(), 0.005);
    }
    assert_relative_eq!(pim.delta_t(), 1.0, epsilon = 1e-12);

    let state = NavState {
        pose: Pose::identity(),
        velocity: Vec3::zeros(),
    };
    let predicted = pim.predict(&state);
    assert!(predicted.pose.translation().norm() < 1e-9);
    assert!(predicted.velocity.norm() < 1e-9);
}

/// Constant world-frame acceleration, no rotation: p(t) = ½at², v(t) = at.
#[test]
fn preintegration_constant_acceleration() {
    let params = ImuParams::default();
    let mut pim = ImuPreintegrator::new(&params, &ImuBias::default());

    let a = Vec3::new(0.7, -0.3, 0.2);
    let f = a + Vec3::new(0.0, 0.0, params.gravity); // specific force = a - g_world
    let (dt, n) = (0.001, 2000); // 2 s
    for _ in 0..n {
        pim.integrate(f, Vec3::zeros(), dt);
    }

    let predicted = pim.predict(&NavState {
        pose: Pose::identity(),
        velocity: Vec3::zeros(),
    });
    let t = dt * n as f64;
    assert_relative_eq!(predicted.velocity, a * t, epsilon = 1e-6);
    assert_relative_eq!(
        predicted.pose.translation(),
        a * (0.5 * t * t),
        epsilon = 1e-3 // discrete integration error over 2000 steps
    );
}

/// Full graph exercise: two states linked by an IMU factor recover a known motion.
#[test]
fn imu_factor_in_graph_recovers_motion() {
    let params = ImuParams {
        accel_sigma: 1e-3,
        gyro_sigma: 1e-4,
        integration_sigma: 1e-8,
        gravity: 9.81,
    };
    let bias = ImuBias::default();

    // Ground truth: accelerate at 1 m/s² along +X for 1 s → ends at x = 0.5, v = 1.
    let a = Vec3::new(1.0, 0.0, 0.0);
    let mut pim = ImuPreintegrator::new(&params, &bias);
    let (dt, n) = (0.001, 1000);
    for _ in 0..n {
        pim.integrate(a + Vec3::new(0.0, 0.0, params.gravity), Vec3::zeros(), dt);
    }
    let end_t = Vec3::new(0.5, 0.0, 0.0);
    let end_v = Vec3::new(1.0, 0.0, 0.0);

    let mut graph = FactorGraph::new();
    let (x0, v0, x1, v1, b0) = (
        Key::pose(0),
        Key::velocity(0),
        Key::pose(1),
        Key::velocity(1),
        Key::bias(0),
    );

    graph.insert_pose(x0, &Pose::identity());
    graph.insert_velocity(v0, Vec3::zeros());
    graph.insert_bias(b0, &bias);
    // Deliberately wrong initial guesses for the end state.
    graph.insert_pose(
        x1,
        &Pose::new(Rotation::identity(), Vec3::new(2.0, 1.0, -0.5)),
    );
    graph.insert_velocity(v1, Vec3::new(0.0, 2.0, 0.0));

    graph.add_pose_prior(x0, &Pose::identity(), &PoseNoise::isotropic(1e-4, 1e-4));
    graph.add_velocity_prior(v0, Vec3::zeros(), 1e-4);
    graph.add_bias_prior(b0, &bias, 1e-3);
    graph
        .add_imu_factor(x0, v0, x1, v1, b0, &pim)
        .expect("imu factor");

    graph.optimize(100).expect("optimisation must converge");

    let pose1 = graph.pose(x1).unwrap();
    let vel1 = graph.velocity(v1).unwrap();
    assert_relative_eq!(pose1.translation(), end_t, epsilon = 1e-3);
    assert_relative_eq!(vel1, end_v, epsilon = 1e-3);
}

//! The ADR 0010 tilt scenario: a base that pitches while braking/accelerating sweeps
//! its scan plane through the floor, and a planar-naive matcher sees phantom geometry.
//! Tilt compensation (IMU attitude → 3D lift → floor gating) must hold tracking near
//! clean-run quality; running the *same* tilted data without the IMU must be measurably
//! worse — otherwise the compensation isn't doing anything and this test is lying.
//!
//! Clean + noisy variants per the ADR 0010 test-data requirement (range noise +
//! dropouts on top of the tilt).

use slam_frontend_scan::{ScanOdometry, ScanOdometryConfig, Se2};
use slam_types::{FrameId, ImuSample, Pose, Rotation, SlamSystem, Stamp, Vec3};
use std::f64::consts::PI;

mod common;
use common::{pose_error, simulate_scan_25d, world_25d};

const LIDAR_FOV: f64 = 1.5 * PI; // 270°
const LIDAR_BEAMS: usize = 540;
const SENSOR_Z: f64 = 0.18; // mock-rig lidar mounting height
const G: f64 = 9.80665;
const SCAN_DT: f64 = 0.05; // 20 Hz lidar
const IMU_RATE: usize = 10; // IMU samples per scan interval (200 Hz)

/// Pitch profile: two smooth pulses (hard accel, then hard brake) of `amp` radians.
fn pitch_at(t: f64, amp: f64) -> f64 {
    let pulse = |t0: f64, dur: f64, t: f64| {
        if (t0..t0 + dur).contains(&t) {
            (PI * (t - t0) / dur).sin().powi(2)
        } else {
            0.0
        }
    };
    amp * (pulse(2.0, 1.5, t) - pulse(6.0, 1.5, t))
}

/// d(pitch)/dt by central difference — the gyro rate consistent with the profile.
fn pitch_rate_at(t: f64, amp: f64) -> f64 {
    let h = 1e-5;
    (pitch_at(t + h, amp) - pitch_at(t - h, amp)) / (2.0 * h)
}

struct RunResult {
    worst_dt: f64,
    stats: slam_frontend_scan::ScanOdometryStats,
}

/// Drive a straight line with pitch pulses; `feed_imu` selects compensated vs naive,
/// `pitch_amp` = 0 is the clean reference, `noise` adds range noise + dropouts.
fn drive_tilted(pitch_amp: f64, feed_imu: bool, noise: bool) -> RunResult {
    drive_tilted_imu_mount(pitch_amp, feed_imu, noise, Rotation::identity())
}

/// Like [`drive_tilted`], with the IMU mounted at `imu_mount` (its own rig frame):
/// samples are expressed in that frame and tagged with it — the engine must rotate
/// them back through the extrinsic (multi-IMU rigs, ADR 0009).
fn drive_tilted_imu_mount(
    pitch_amp: f64,
    feed_imu: bool,
    noise: bool,
    imu_mount: Rotation,
) -> RunResult {
    let walls = world_25d();
    let t_base_sensor = Pose::new(Rotation::identity(), Vec3::new(0.0, 0.0, SENSOR_Z));
    let lidar = FrameId(1);
    let imu_frame = FrameId(2);
    let extrinsics = vec![
        Pose::identity(),
        t_base_sensor,
        Pose::new(imu_mount, Vec3::new(0.1, -0.05, 0.3)),
    ];

    let step = Se2::new(0.06, 0.0, 0.0); // 1.2 m/s straight drive
    let mut truth = Se2::new(-4.5, -2.5, 0.25);
    let mut odo =
        ScanOdometry::with_extrinsics(truth.to_pose(), ScanOdometryConfig::default(), extrinsics);

    let mut worst_dt: f64 = 0.0;
    for k in 0..140 {
        let t = k as f64 * SCAN_DT;

        // IMU stream between scans: gyro consistent with the pitch profile, accel = the
        // gravity the tilted body would measure (quasi-static, so |accel| = g and the
        // attitude filter trusts it).
        if feed_imu {
            // Causal: IMU samples cover the interval *up to* the scan stamp.
            for j in 0..IMU_RATE {
                let ti = t - SCAN_DT + (j + 1) as f64 * SCAN_DT / IMU_RATE as f64;
                let pitch = pitch_at(ti, pitch_amp);
                let attitude = Rotation::from_rpy(0.0, pitch, 0.0);
                let gyro_base = Vec3::new(0.0, pitch_rate_at(ti, pitch_amp), 0.0);
                let accel_base = attitude.inverse().rotate(Vec3::new(0.0, 0.0, G));
                // Express in the IMU's own frame; the engine rotates it back.
                let inv = imu_mount.inverse();
                odo.process_imu(
                    &ImuSample::new(
                        Stamp::from_seconds(ti),
                        inv.rotate(gyro_base),
                        inv.rotate(accel_base),
                    )
                    .in_frame(imu_frame),
                );
            }
        }

        // The sensor's true world pose: planar truth, tilted by the pitch, lidar on top.
        let pitch = pitch_at(t, pitch_amp);
        let t_world_base =
            truth.to_pose() * Pose::new(Rotation::from_rpy(0.0, pitch, 0.0), Vec3::zeros());
        let t_world_sensor = t_world_base * t_base_sensor;

        let noise_arg = noise.then_some((0.01, k as u64));
        let scan = simulate_scan_25d(
            &t_world_sensor,
            t,
            &walls,
            LIDAR_FOV,
            LIDAR_BEAMS,
            lidar,
            noise_arg,
        );
        odo.process_scan(&scan);

        let est = odo.current_estimate().expect("estimate after first scan");
        let (dt, _) = pose_error(&est.pose, &truth);
        if std::env::var("TILT_DEBUG").is_ok() {
            let st = odo.stats();
            eprintln!(
                "k={k} t={t:.2} pitch={:.2}deg dt={dt:.4} m={} c={} s={} kf={}",
                pitch.to_degrees(),
                st.matched,
                st.coasted,
                st.skipped,
                st.keyframes
            );
        }
        worst_dt = worst_dt.max(dt);
        truth = truth.compose(&step);
    }
    RunResult {
        worst_dt,
        stats: odo.stats(),
    }
}

const PITCH_AMP: f64 = 4.0 * PI / 180.0; // 4° braking pitch

#[test]
fn tilt_compensation_holds_clean_run_quality() {
    let clean = drive_tilted(0.0, false, false);
    let compensated = drive_tilted(PITCH_AMP, true, false);

    assert!(
        clean.worst_dt < 0.05,
        "clean reference drifted: {:.4} m ({:?})",
        clean.worst_dt,
        clean.stats
    );
    assert!(
        compensated.worst_dt < clean.worst_dt.max(0.02) * 3.0 && compensated.worst_dt < 0.10,
        "tilt compensation lost clean-run quality: {:.4} m vs clean {:.4} m ({:?})",
        compensated.worst_dt,
        clean.worst_dt,
        compensated.stats
    );
}

#[test]
fn uncompensated_tilt_is_measurably_worse() {
    let compensated = drive_tilted(PITCH_AMP, true, false);
    let naive = drive_tilted(PITCH_AMP, false, false);

    assert!(
        naive.worst_dt > 2.0 * compensated.worst_dt,
        "tilt did not hurt the naive run (compensation untestable): \
         naive {:.4} m vs compensated {:.4} m",
        naive.worst_dt,
        compensated.worst_dt
    );
}

#[test]
fn tilt_compensation_survives_range_noise_and_dropouts() {
    let clean = drive_tilted(PITCH_AMP, true, false);
    let noisy = drive_tilted(PITCH_AMP, true, true);

    assert!(
        noisy.worst_dt < 0.15 && noisy.worst_dt < 4.0 * clean.worst_dt.max(0.02),
        "noisy variant degraded too far: {:.4} m vs clean-variant {:.4} m ({:?})",
        noisy.worst_dt,
        clean.worst_dt,
        noisy.stats
    );
}

#[test]
fn a_rotated_imu_mount_compensates_identically() {
    let base_mounted = drive_tilted(PITCH_AMP, true, false);
    // A gratuitously awkward mounting: yawed 90°, rolled upside down.
    let mount = Rotation::from_rpy(PI, 0.0, PI / 2.0);
    let rotated = drive_tilted_imu_mount(PITCH_AMP, true, false, mount);
    assert!(
        (rotated.worst_dt - base_mounted.worst_dt).abs() < 1e-6,
        "rotated IMU mount changed compensation: {:.6} vs {:.6}",
        rotated.worst_dt,
        base_mounted.worst_dt
    );
}

//! Loop closure on a circular circuit (ADR 0010 stage 3a).
//!
//! A constant-curvature loop around a central block in a square room: wall normals
//! sweep continuously, so every scan is well-constrained (no degenerate travel axis —
//! the failure mode of straight-corridor synthetic worlds) yet small per-scan
//! prediction + range-noise errors random-walk into real drift over the ~50 m lap.
//! Returning to the start must re-register against the first frozen submap, verify it
//! geometrically, and snap the drifted pose back.

use slam_frontend_scan::{ScanToMapConfig, ScanToMapOdometry, Se2};
use slam_types::{FrameId, LaserScan2D, SlamSystem, Vec2};
use std::f64::consts::{FRAC_PI_2, PI};

mod common;
use common::{noisy_range, pose_error, simulate_scan_at};

/// A 24×24 m room with an 8×8 central block — the circular path sees the block's faces
/// and the outer walls at continuously varying angles.
fn ring_world() -> Vec<(Vec2, Vec2)> {
    let mut segs = Vec::new();
    for hw in [12.0, 4.0] {
        let c = [
            Vec2::new(-hw, -hw),
            Vec2::new(hw, -hw),
            Vec2::new(hw, hw),
            Vec2::new(-hw, hw),
        ];
        for i in 0..4 {
            segs.push((c[i], c[(i + 1) % 4]));
        }
    }
    segs
}

/// A 1080-beam 270° scan with per-beam range noise (the 2.5D world's noise model,
/// applied to the flat raycaster — keeps the shared helper's signature untouched).
fn noisy_scan(truth: &Se2, stamp_s: f64, segments: &[(Vec2, Vec2)], k: u64) -> LaserScan2D {
    let mut scan = simulate_scan_at(truth, stamp_s, segments, 1.5 * PI, 1080, FrameId::BASE);
    for (i, r) in scan.ranges.iter_mut().enumerate() {
        if r.is_finite() {
            *r = noisy_range(*r as f64, i, k, 0.015) as f32;
        }
    }
    scan
}

const RADIUS: f64 = 8.0;
/// ~1.25 laps of the radius-8 circle (≈ 63 m) so the start is comfortably revisited.
const SCANS: usize = 1050;

/// Drive the circular circuit; returns (odometry, final pose error vs truth).
fn drive_ring(cfg: ScanToMapConfig) -> (ScanToMapOdometry, f64) {
    let segments = ring_world();
    // Start on the circle at angle 0: position (R, 0), heading tangent (+y, CCW).
    let mut truth = Se2::new(RADIUS, 0.0, FRAC_PI_2);
    let mut odo = ScanToMapOdometry::anchored_at(truth.to_pose(), cfg);

    let mut last = 0.0f64;
    for k in 0..SCANS {
        // Arc motion: forward, with a heading turn matched to the radius. The step is
        // modulated so the constant-velocity prediction is always slightly wrong.
        let step = 0.06 + 0.015 * (k as f64 / 6.0).sin();
        truth = truth.compose(&Se2::new(step, 0.0, step / RADIUS));
        odo.process_scan(&noisy_scan(&truth, k as f64 * 0.05, &segments, k as u64));
        let est = odo.current_estimate().expect("estimate");
        let (dt, _) = pose_error(&est.pose, &truth);
        last = dt;
    }
    (odo, last)
}

#[test]
fn ring_circuit_closes_the_loop() {
    let (with_loops, final_with) = drive_ring(ScanToMapConfig::default());
    let (without, final_without) = drive_ring(ScanToMapConfig {
        loop_radius: 0.0, // disable closure, everything else identical
        ..ScanToMapConfig::default()
    });

    let closures = with_loops.loop_closures().len();
    assert!(
        with_loops.stats().keyframes >= 2,
        "circuit should span several submaps: {:?}",
        with_loops.stats()
    );
    assert!(
        closures >= 1,
        "returning to the start must close a loop ({:?})",
        with_loops.stats()
    );
    assert!(
        final_with < final_without,
        "closure must reduce final drift: {final_with:.3} m (closed) vs \
         {final_without:.3} m (open), {closures} closures"
    );
    let _ = without;
}

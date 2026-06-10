//! Shared raycast world for front-end integration tests: a rectangular room with a
//! pillar, segment-intersection lidar simulation, and pose-error helpers.

// Each integration-test binary compiles this module separately and uses a subset.
#![allow(dead_code)]

use slam_frontend_scan::Se2;
use slam_types::{FrameId, LaserScan2D, Stamp, Vec2};
use std::f64::consts::PI;

/// Wall segments of the world: a 12×8 m room centred at the origin, with a 1×1 m pillar
/// whose corner sits at (2, 0.5) (the pillar breaks the rectangle's symmetries and adds
/// close-range structure).
pub fn world_segments() -> Vec<(Vec2, Vec2)> {
    let mut segs = Vec::new();
    let (hw, hh) = (6.0, 4.0);
    let room = [
        Vec2::new(-hw, -hh),
        Vec2::new(hw, -hh),
        Vec2::new(hw, hh),
        Vec2::new(-hw, hh),
    ];
    for i in 0..4 {
        segs.push((room[i], room[(i + 1) % 4]));
    }
    let pillar = [
        Vec2::new(2.0, 0.5),
        Vec2::new(3.0, 0.5),
        Vec2::new(3.0, 1.5),
        Vec2::new(2.0, 1.5),
    ];
    for i in 0..4 {
        segs.push((pillar[i], pillar[(i + 1) % 4]));
    }
    segs
}

/// Distance along the ray from `origin` in direction `dir` to segment `(a, b)`, if hit.
fn ray_segment(origin: Vec2, dir: Vec2, a: Vec2, b: Vec2) -> Option<f64> {
    let v = b - a; // segment direction
    let denom = dir.x * (-v.y) - dir.y * (-v.x); // det[dir, -v]
    if denom.abs() < 1e-12 {
        return None; // parallel
    }
    let w = a - origin;
    let t = (w.x * (-v.y) - w.y * (-v.x)) / denom; // along the ray
    let u = (dir.x * w.y - dir.y * w.x) / denom; // along the segment
    (t > 1e-9 && (0.0..=1.0).contains(&u)).then_some(t)
}

/// Simulate one revolution of a `fov`-wide, `beams`-beam scanner at the world pose
/// `sensor`, tagging the scan with `frame`. The FOV is centred on the sensor's +X.
pub fn simulate_scan_at(
    sensor: &Se2,
    stamp_s: f64,
    segments: &[(Vec2, Vec2)],
    fov: f64,
    beams: usize,
    frame: FrameId,
) -> LaserScan2D {
    let angle_min = -fov / 2.0;
    let angle_increment = fov / beams as f64;
    let origin = Vec2::new(sensor.x, sensor.y);
    let ranges = (0..beams)
        .map(|i| {
            let angle = sensor.theta + angle_min + i as f64 * angle_increment;
            let dir = Vec2::new(angle.cos(), angle.sin());
            segments
                .iter()
                .filter_map(|(a, b)| ray_segment(origin, dir, *a, *b))
                .fold(f32::INFINITY, |acc, t| acc.min(t as f32))
        })
        .collect();
    LaserScan2D {
        stamp: Stamp::from_seconds(stamp_s),
        frame,
        angle_min,
        angle_increment,
        range_min: 0.05,
        range_max: 30.0,
        ranges,
    }
}

/// Simulate one full-circle 360-beam revolution from a base-frame sensor pose.
pub fn simulate_scan(pose: &Se2, stamp_s: f64, segments: &[(Vec2, Vec2)]) -> LaserScan2D {
    simulate_scan_at(pose, stamp_s, segments, 2.0 * PI, 360, FrameId::BASE)
}

// ---------------------------------------------------------------------------------
// 2.5D world: finite-height walls + a floor plane, scanned by a *tilted* 3D sensor
// (ADR 0010's noise scenario: the scan plane is not horizontal under acceleration).
// ---------------------------------------------------------------------------------

use slam_types::{Pose, Rotation, Vec3};

/// Vertical wall rectangle: the segment `(a, b)` extruded from z = 0 to `height`.
pub struct Wall {
    pub a: Vec2,
    pub b: Vec2,
    pub height: f64,
}

/// The standard room as a 2.5D world: 4 m outer walls, a deliberately *low* (0.8 m)
/// pillar — beams that dip under tilt land on the floor or skim over it, which is
/// exactly the failure mode planar matching cannot represent.
pub fn world_25d() -> Vec<Wall> {
    world_segments()
        .into_iter()
        .enumerate()
        .map(|(i, (a, b))| Wall {
            a,
            b,
            height: if i < 4 { 4.0 } else { 0.8 },
        })
        .collect()
}

/// Nearest hit of the 3D ray `origin + t·dir` against walls and the z = 0 floor.
fn ray_world_25d(origin: Vec3, dir: Vec3, walls: &[Wall]) -> f64 {
    let mut best = f64::INFINITY;
    if dir.z < -1e-12 {
        best = best.min(-origin.z / dir.z); // floor
    }
    let o2 = Vec2::new(origin.x, origin.y);
    let d2 = Vec2::new(dir.x, dir.y);
    for w in walls {
        if let Some(t) = ray_segment(o2, d2, w.a, w.b) {
            let z = origin.z + t * dir.z;
            if (0.0..=w.height).contains(&z) {
                best = best.min(t);
            }
        }
    }
    best
}

/// Deterministic per-beam range perturbation in `[-amplitude, amplitude]` (m), plus a
/// dropout every `dropout_every`-th beam. A keyed sine hash — no RNG dependency, stable
/// across runs (workflow/CI reproducibility).
pub fn noisy_range(r: f64, beam: usize, scan_seq: u64, amplitude: f64) -> f64 {
    let x = (beam as f64 * 12.9898 + scan_seq as f64 * 78.233).sin() * 43758.5453;
    r + amplitude * 2.0 * (x - x.floor() - 0.5)
}

/// Simulate one revolution from a full 3D sensor pose (`t_world_sensor`): beams sweep
/// the sensor's (tilted) xy-plane. `noise` of `(amplitude_m, scan_seq)` perturbs ranges
/// and drops every 50th beam — the "noisy" half of the ADR 0010 test-data requirement.
#[allow(clippy::too_many_arguments)]
pub fn simulate_scan_25d(
    t_world_sensor: &Pose,
    stamp_s: f64,
    walls: &[Wall],
    fov: f64,
    beams: usize,
    frame: FrameId,
    noise: Option<(f64, u64)>,
) -> LaserScan2D {
    let angle_min = -fov / 2.0;
    let angle_increment = fov / beams as f64;
    let origin = t_world_sensor.translation();
    let rot: Rotation = t_world_sensor.rotation();
    let ranges = (0..beams)
        .map(|i| {
            if let Some((_, seq)) = noise {
                if (i + seq as usize).is_multiple_of(50) {
                    return f32::INFINITY; // dropout
                }
            }
            let angle = angle_min + i as f64 * angle_increment;
            let dir = rot.rotate(Vec3::new(angle.cos(), angle.sin(), 0.0));
            let r = ray_world_25d(origin, dir, walls);
            match noise {
                Some((amp, seq)) if r.is_finite() => noisy_range(r, i, seq, amp) as f32,
                _ => r as f32,
            }
        })
        .collect();
    LaserScan2D {
        stamp: Stamp::from_seconds(stamp_s),
        frame,
        angle_min,
        angle_increment,
        range_min: 0.05,
        range_max: 30.0,
        ranges,
    }
}

/// (translation error (m), |yaw error| (rad)) of an SE(3) estimate vs a planar truth.
pub fn pose_error(estimate: &slam_types::Pose, truth: &Se2) -> (f64, f64) {
    let t = estimate.translation();
    let dt = (t.x - truth.x).hypot(t.y - truth.y);
    let yaw = estimate.rotation().log().z;
    let mut dyaw = (yaw - truth.theta).rem_euclid(2.0 * PI);
    if dyaw > PI {
        dyaw -= 2.0 * PI;
    }
    (dt, dyaw.abs())
}

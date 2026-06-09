//! Micro-benchmark for the PLICP matcher — every optimisation claim is measured here
//! (project rule: reproducible benchmarks, ADR 0005).
//!
//! Workload mirrors the real front-end: a ~720-beam indoor scan against a keyframe,
//! seeded with a realistic inter-scan motion guess. Two shapes:
//! - `cold`: one full match including reference indexing (what scan-to-scan would pay);
//! - `keyframe_reuse`: 10 matches against one reference (what scan-to-keyframe pays).

use criterion::{criterion_group, criterion_main, Criterion};
use slam_frontend_scan::{match_scans, MatchConfig, ScanMatcher, Se2};
use slam_types::Vec2;
use std::hint::black_box;

/// Points on the walls of a 12×8 room with a 1×1 pillar, as seen by a 720-beam scanner
/// at `pose` (raycast, like the integration tests).
fn raycast_scan(pose: &Se2, beams: usize) -> Vec<Vec2> {
    let mut segs: Vec<(Vec2, Vec2)> = Vec::new();
    let room = [
        Vec2::new(-6.0, -4.0),
        Vec2::new(6.0, -4.0),
        Vec2::new(6.0, 4.0),
        Vec2::new(-6.0, 4.0),
    ];
    let pillar = [
        Vec2::new(2.0, 0.5),
        Vec2::new(3.0, 0.5),
        Vec2::new(3.0, 1.5),
        Vec2::new(2.0, 1.5),
    ];
    for i in 0..4 {
        segs.push((room[i], room[(i + 1) % 4]));
        segs.push((pillar[i], pillar[(i + 1) % 4]));
    }

    let origin = Vec2::new(pose.x, pose.y);
    (0..beams)
        .filter_map(|i| {
            let angle = pose.theta + 2.0 * std::f64::consts::PI * i as f64 / beams as f64;
            let dir = Vec2::new(angle.cos(), angle.sin());
            let mut best = f64::INFINITY;
            for (a, b) in &segs {
                let v = *b - *a;
                let denom = dir.x * (-v.y) - dir.y * (-v.x);
                if denom.abs() < 1e-12 {
                    continue;
                }
                let w = *a - origin;
                let t = (w.x * (-v.y) - w.y * (-v.x)) / denom;
                let u = (dir.x * w.y - dir.y * w.x) / -denom;
                if t > 1e-9 && (0.0..=1.0).contains(&u) {
                    best = best.min(t);
                }
            }
            best.is_finite()
                .then(|| pose.inverse().apply(origin + dir * best))
        })
        .collect()
}

fn bench_match(c: &mut Criterion) {
    let reference_pose = Se2::new(-3.0, -2.0, 0.1);
    let motion = Se2::new(0.06, 0.01, 0.015); // a realistic inter-scan step
    let current_pose = reference_pose.compose(&motion);

    let reference = raycast_scan(&reference_pose, 720);
    let current = raycast_scan(&current_pose, 720);
    // Seed slightly off, like a constant-velocity prediction would.
    let initial = Se2::new(0.05, 0.0, 0.01);
    let cfg = MatchConfig::default();

    c.bench_function("match_scans/720pts/cold", |b| {
        b.iter(|| {
            match_scans(
                black_box(&reference),
                black_box(&current),
                black_box(initial),
                &cfg,
            )
            .unwrap()
        })
    });

    // The scan-to-keyframe shape: the reference is indexed once, matched ten times.
    c.bench_function("match_scans/720pts/keyframe_reuse_x10", |b| {
        b.iter(|| {
            let matcher = ScanMatcher::new(reference.clone(), cfg.clone());
            let mut last = Se2::identity();
            for _ in 0..10 {
                last = matcher
                    .match_to(black_box(&current), black_box(initial))
                    .unwrap()
                    .transform;
            }
            black_box(last)
        })
    });
}

criterion_group!(benches, bench_match);
criterion_main!(benches);

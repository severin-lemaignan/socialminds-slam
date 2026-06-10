//! Map-op benchmarks (ADR 0010 performance gate): integrate a lidar-sized fan, sample
//! a registration-sized batch. Budgets: both far above sensor rate on one core.

use criterion::{criterion_group, criterion_main, Criterion};
use slam_map::{SparseTsdf, TsdfConfig, TsdfMap};
use slam_types::Vec3;

/// A 1081-beam planar fan hitting a square room from its centre, at lidar height.
fn fan(origin: Vec3) -> Vec<Vec3> {
    (0..1081)
        .map(|i| {
            let a = -2.36 + i as f64 * (4.71 / 1080.0);
            let (s, c) = a.sin_cos();
            // Range to a 12×8 room wall from the origin.
            let r = (6.0 / c.abs()).min(4.0 / s.abs()).min(12.0);
            origin + Vec3::new(r * c, r * s, 0.0)
        })
        .collect()
}

fn bench(c: &mut Criterion) {
    let origin = Vec3::new(0.0, 0.0, 0.18);
    let points = fan(origin);

    c.bench_function("integrate_1081_beam_fan", |b| {
        let mut map = SparseTsdf::new(TsdfConfig::default());
        b.iter(|| map.integrate_points(origin, &points));
    });

    c.bench_function("sample_batch_1081", |b| {
        let mut map = SparseTsdf::new(TsdfConfig::default());
        for _ in 0..8 {
            map.integrate_points(origin, &points);
        }
        let mut out = Vec::new();
        b.iter(|| map.sample_batch(&points, &mut out));
    });
}

criterion_group!(benches, bench);
criterion_main!(benches);

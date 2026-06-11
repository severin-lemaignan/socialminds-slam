//! Live / progressive 3D visualization through the rerun viewer (ADR 0011).
//!
//! `--rerun spawn` streams to a live viewer **while the engine runs**; `--rerun
//! save:run.rrd` records the same stream for later timeline-scrubbing — the
//! progressive build of the map (accumulating per-chunk point clouds), the current
//! sweep, both trajectories, and the final TSDF surface.
//!
//! Compiled only with `--features viz` (the `rerun` SDK is a heavy dependency); the
//! stub below keeps the CLI surface identical otherwise. Logging happens outside the
//! per-event latency clock but inside wall time: a `--rerun` run is a debugging run,
//! not a benchmark.

use anyhow::Result;
use slam_types::{Pose, Trajectory};

#[cfg(feature = "viz")]
pub use real::Viz;

#[cfg(feature = "viz")]
mod real {
    use super::*;
    use slam_map::TsdfMap;

    /// How many scans accumulate into one progressive map chunk entity.
    const CHUNK_SCANS: usize = 10;

    /// Illumination-normalized chroma for map display: the pixel's CIELAB **a\*b\***
    /// plane re-rendered at a fixed lightness. L\* is the illumination axis by
    /// construction, so exposure and shadows are discarded while hue *and* colour
    /// strength survive; near-grey/dark pixels land near (a\*, b\*) ≈ 0 — the
    /// cube-root compression keeps sensor noise from painting them with phantom hue.
    /// The same plane is the intended storage encoding for the voxel colour channel
    /// (a\*b\* is perceptually uniform, so TSDF-style weighted averaging behaves).
    fn chroma(c: [u8; 3]) -> rerun::Color {
        fn lin(c: u8) -> f32 {
            let c = c as f32 / 255.0;
            if c <= 0.04045 {
                c / 12.92
            } else {
                ((c + 0.055) / 1.055).powf(2.4)
            }
        }
        fn srgb(c: f32) -> u8 {
            let c = c.clamp(0.0, 1.0);
            let s = if c <= 0.0031308 {
                12.92 * c
            } else {
                1.055 * c.powf(1.0 / 2.4) - 0.055
            };
            (s * 255.0 + 0.5) as u8
        }
        fn f(t: f32) -> f32 {
            if t > 216.0 / 24389.0 {
                t.cbrt()
            } else {
                (24389.0 / 27.0 * t + 16.0) / 116.0
            }
        }
        fn f_inv(t: f32) -> f32 {
            let t3 = t * t * t;
            if t3 > 216.0 / 24389.0 {
                t3
            } else {
                (116.0 * t - 16.0) * 27.0 / 24389.0
            }
        }
        // sRGB → XYZ (D65) → (a*, b*).
        let (r, g, b) = (lin(c[0]), lin(c[1]), lin(c[2]));
        let x = 0.4124 * r + 0.3576 * g + 0.1805 * b;
        let y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        let z = 0.0193 * r + 0.1192 * g + 0.9505 * b;
        const WHITE: (f32, f32, f32) = (0.95047, 1.0, 1.08883);
        let (fx, fy, fz) = (f(x / WHITE.0), f(y / WHITE.1), f(z / WHITE.2));
        let a_star = 500.0 * (fx - fy);
        let b_star = 200.0 * (fy - fz);
        // Re-render the chroma at fixed L* = 70 (out-of-gamut clamps in `srgb`).
        let fy2 = (70.0 + 16.0) / 116.0;
        let fx2 = fy2 + a_star / 500.0;
        let fz2 = fy2 - b_star / 200.0;
        let (x2, y2, z2) = (
            WHITE.0 * f_inv(fx2),
            WHITE.1 * f_inv(fy2),
            WHITE.2 * f_inv(fz2),
        );
        rerun::Color::from_rgb(
            srgb(3.2406 * x2 - 1.5372 * y2 - 0.4986 * z2),
            srgb(-0.9689 * x2 + 1.8758 * y2 + 0.0415 * z2),
            srgb(0.0557 * x2 - 0.2040 * y2 + 1.0570 * z2),
        )
    }

    pub struct Viz {
        rec: rerun::RecordingStream,
        trajectory: Vec<[f32; 3]>,
        chunk: Vec<[f32; 3]>,
        scans_seen: usize,
        chunks_logged: usize,
        cloud_chunk: Vec<[f32; 3]>,
        cloud_chunk_colors: Vec<rerun::Color>,
        clouds_seen: usize,
        cloud_chunks_logged: usize,
    }

    impl Viz {
        /// `mode`: `spawn` (live viewer), `connect[:ADDR]` (running viewer), or
        /// `save:FILE.rrd` (record for later scrubbing).
        pub fn new(mode: &str) -> Result<Self> {
            let builder = rerun::RecordingStreamBuilder::new("slam-replay");
            let rec = match mode {
                "spawn" => builder.spawn()?,
                "connect" => builder.connect_grpc()?,
                m if m.starts_with("save:") => builder.save(&m["save:".len()..])?,
                m => anyhow::bail!("unknown --rerun mode {m:?} (spawn | connect | save:FILE)"),
            };
            Ok(Viz {
                rec,
                trajectory: Vec::new(),
                chunk: Vec::new(),
                scans_seen: 0,
                chunks_logged: 0,
                cloud_chunk: Vec::new(),
                cloud_chunk_colors: Vec::new(),
                clouds_seen: 0,
                cloud_chunks_logged: 0,
            })
        }

        /// Ground truth, as one static line strip.
        pub fn log_groundtruth(&self, traj: &Trajectory) {
            let pts: Vec<[f32; 3]> = traj
                .poses()
                .iter()
                .map(|p| {
                    let t = p.pose.translation();
                    [t.x as f32, t.y as f32, t.z as f32]
                })
                .collect();
            let _ = self.rec.log_static(
                "world/groundtruth",
                &rerun::LineStrips3D::new([pts]).with_colors([rerun::Color::from_rgb(60, 200, 60)]),
            );
        }

        /// Advance the timeline and grow the (blue) estimate trajectory — shared by
        /// every exteroceptive modality: a scan-less depth+odom run must still draw
        /// its path.
        fn log_pose(&mut self, stamp_s: f64, pose: &Pose) {
            self.rec.set_duration_secs("sensor_time", stamp_s);
            let t = pose.translation();
            self.trajectory.push([t.x as f32, t.y as f32, t.z as f32]);
            let _ = self.rec.log(
                "world/trajectory",
                &rerun::LineStrips3D::new([self.trajectory.clone()])
                    .with_colors([rerun::Color::from_rgb(80, 120, 230)]),
            );
        }

        /// One processed scan: current sweep, growing estimate path, progressive map.
        pub fn log_scan(&mut self, stamp_s: f64, pose: &Pose, world: Vec<[f32; 3]>) {
            self.log_pose(stamp_s, pose);
            let _ = self.rec.log(
                "world/scan",
                &rerun::Points3D::new(world.iter().copied())
                    .with_colors([rerun::Color::from_rgb(230, 80, 80)])
                    .with_radii([0.02]),
            );

            // Progressive map: every CHUNK_SCANS scans freeze a chunk under its own
            // entity — chunks accumulate in the viewer, replaying the build over time.
            self.chunk.extend(world);
            self.scans_seen += 1;
            if self.scans_seen.is_multiple_of(CHUNK_SCANS) {
                let _ = self.rec.log(
                    format!("world/map/chunk_{:05}", self.chunks_logged),
                    &rerun::Points3D::new(self.chunk.drain(..))
                        .with_colors([rerun::Color::from_rgb(160, 160, 170)])
                        .with_radii([0.012]),
                );
                self.chunks_logged += 1;
            }
        }

        /// One depth cloud: the live frame plus an accumulating 3D map layer
        /// (chunked like the scan layer, under `world/map3d/`). When the cloud
        /// carries RGB (`--color-topic` / config `color:`), points are painted with
        /// their illumination-normalized chroma; otherwise the fixed layer colours.
        pub fn log_cloud(
            &mut self,
            stamp_s: f64,
            pose: &Pose,
            world: Vec<[f32; 3]>,
            colors: Vec<[u8; 3]>,
        ) {
            self.log_pose(stamp_s, pose);
            let colored = colors.len() == world.len() && !world.is_empty();
            let point_colors: Vec<rerun::Color> = if colored {
                colors.iter().map(|&c| chroma(c)).collect()
            } else {
                vec![rerun::Color::from_rgb(240, 170, 60); world.len()]
            };
            let _ = self.rec.log(
                "world/depth",
                &rerun::Points3D::new(world.iter().copied())
                    .with_colors(point_colors.iter().copied())
                    .with_radii([0.01]),
            );
            if colored {
                self.cloud_chunk_colors.extend(point_colors);
            } else {
                self.cloud_chunk_colors.extend(std::iter::repeat_n(
                    rerun::Color::from_rgb(120, 140, 120),
                    world.len(),
                ));
            }
            self.cloud_chunk.extend(world);
            self.clouds_seen += 1;
            if self.clouds_seen.is_multiple_of(CHUNK_SCANS) {
                // Solid cubes at the depth sampling pitch (range-adaptive
                // `target_spacing`, 5 cm default), so the accumulated 3D map reads
                // as surface, consistent with the `world/tsdf` voxel rendering.
                let half = 0.025f32;
                let _ = self.rec.log(
                    format!("world/map3d/chunk_{:05}", self.cloud_chunks_logged),
                    &rerun::Boxes3D::from_centers_and_half_sizes(
                        self.cloud_chunk.drain(..),
                        std::iter::repeat_n([half, half, half], 1),
                    )
                    .with_colors(self.cloud_chunk_colors.drain(..))
                    .with_fill_mode(rerun::FillMode::Solid),
                );
                self.cloud_chunks_logged += 1;
            }
        }

        /// The final TSDF surface (|sdf| below one voxel), coloured by height.
        ///
        /// Submaps are **anchor-local** (stage 3b): each goes under its own entity
        /// with its anchor as the entity transform, so the viewer places it in the
        /// world — and a re-optimised anchor would re-pose voxels without rewrites,
        /// exactly like the engine itself.
        pub fn log_tsdf(
            &self,
            submaps: &[(f64, f64, f64, &dyn TsdfMap)],
            stamp_s: f64,
            announce: bool,
        ) {
            self.rec.set_duration_secs("sensor_time", stamp_s);
            let mut total = 0usize;
            for (i, &(ax, ay, atheta, map)) in submaps.iter().enumerate() {
                let voxel = map.config().voxel_size;
                let mut pts: Vec<[f32; 3]> = Vec::new();
                let mut colors: Vec<rerun::Color> = Vec::new();
                map.visit_voxels(&mut |ix, iy, iz, tsdf, _w| {
                    if (tsdf as f64).abs() > voxel {
                        return;
                    }
                    let z = (iz as f64 + 0.5) * voxel;
                    pts.push([
                        ((ix as f64 + 0.5) * voxel) as f32,
                        ((iy as f64 + 0.5) * voxel) as f32,
                        z as f32,
                    ]);
                    // Height ramp 0..2 m: blue floor → yellow head-height.
                    let t = (z / 2.0).clamp(0.0, 1.0) as f32;
                    colors.push(rerun::Color::from_rgb(
                        (40.0 + 200.0 * t) as u8,
                        (90.0 + 130.0 * t) as u8,
                        (220.0 * (1.0 - t) + 40.0) as u8,
                    ));
                });
                total += pts.len();
                let entity = format!("world/tsdf/submap_{i:03}");
                let _ = self.rec.log(
                    entity.clone(),
                    &rerun::Transform3D::from_translation([ax as f32, ay as f32, 0.0])
                        .with_rotation(rerun::RotationAxisAngle::new(
                            [0.0, 0.0, 1.0],
                            rerun::Angle::from_radians(atheta as f32),
                        )),
                );
                // True-size solid cubes: what the map *is*, not a point-sprite
                // impression.
                let half = (voxel / 2.0) as f32;
                let _ = self.rec.log(
                    entity,
                    &rerun::Boxes3D::from_centers_and_half_sizes(
                        pts,
                        std::iter::repeat_n([half, half, half], 1),
                    )
                    .with_colors(colors)
                    .with_fill_mode(rerun::FillMode::Solid),
                );
            }
            if announce {
                eprintln!(
                    "slam-replay: rerun: TSDF surface {} voxels across {} submaps",
                    total,
                    submaps.len()
                );
            }
        }
    }
}

#[cfg(not(feature = "viz"))]
pub use stub::Viz;

#[cfg(not(feature = "viz"))]
mod stub {
    use super::*;

    /// CLI-compatible stub: `--rerun` without the `viz` feature is a clear error.
    pub struct Viz;

    impl Viz {
        pub fn new(_mode: &str) -> Result<Self> {
            anyhow::bail!(
                "slam-replay was built without visualization; rebuild with \
                 `cargo build --release -p slam-replay --features viz`"
            )
        }

        pub fn log_groundtruth(&self, _traj: &Trajectory) {}

        pub fn log_scan(&mut self, _stamp_s: f64, _pose: &Pose, _world: Vec<[f32; 3]>) {}

        pub fn log_cloud(
            &mut self,
            _stamp_s: f64,
            _pose: &Pose,
            _world: Vec<[f32; 3]>,
            _colors: Vec<[u8; 3]>,
        ) {
        }

        pub fn log_tsdf(
            &self,
            _submaps: &[(f64, f64, f64, &dyn slam_map::TsdfMap)],
            _stamp_s: f64,
            _announce: bool,
        ) {
        }
    }
}

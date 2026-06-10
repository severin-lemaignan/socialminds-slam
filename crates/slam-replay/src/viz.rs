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

    pub struct Viz {
        rec: rerun::RecordingStream,
        trajectory: Vec<[f32; 3]>,
        chunk: Vec<[f32; 3]>,
        scans_seen: usize,
        chunks_logged: usize,
        cloud_chunk: Vec<[f32; 3]>,
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

        /// One processed scan: current sweep, growing estimate path, progressive map.
        pub fn log_scan(&mut self, stamp_s: f64, pose: &Pose, world: Vec<[f32; 3]>) {
            self.rec.set_duration_secs("sensor_time", stamp_s);
            let t = pose.translation();
            self.trajectory.push([t.x as f32, t.y as f32, t.z as f32]);

            let _ = self.rec.log(
                "world/scan",
                &rerun::Points3D::new(world.iter().copied())
                    .with_colors([rerun::Color::from_rgb(230, 80, 80)])
                    .with_radii([0.02]),
            );
            let _ = self.rec.log(
                "world/trajectory",
                &rerun::LineStrips3D::new([self.trajectory.clone()])
                    .with_colors([rerun::Color::from_rgb(80, 120, 230)]),
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
        /// (chunked like the scan layer, under `world/map3d/`).
        pub fn log_cloud(&mut self, stamp_s: f64, world: Vec<[f32; 3]>) {
            self.rec.set_duration_secs("sensor_time", stamp_s);
            let _ = self.rec.log(
                "world/depth",
                &rerun::Points3D::new(world.iter().copied())
                    .with_colors([rerun::Color::from_rgb(240, 170, 60)])
                    .with_radii([0.01]),
            );
            self.cloud_chunk.extend(world);
            self.clouds_seen += 1;
            if self.clouds_seen.is_multiple_of(CHUNK_SCANS) {
                let _ = self.rec.log(
                    format!("world/map3d/chunk_{:05}", self.cloud_chunks_logged),
                    &rerun::Points3D::new(self.cloud_chunk.drain(..))
                        .with_colors([rerun::Color::from_rgb(120, 140, 120)])
                        .with_radii([0.008]),
                );
                self.cloud_chunks_logged += 1;
            }
        }

        /// The final TSDF surface (|sdf| below one voxel), coloured by height.
        pub fn log_tsdf(&self, map: &dyn TsdfMap, stamp_s: f64) {
            self.rec.set_duration_secs("sensor_time", stamp_s);
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
            eprintln!("slam-replay: rerun: TSDF surface {} voxels", pts.len());
            let _ = self.rec.log(
                "world/tsdf",
                &rerun::Points3D::new(pts)
                    .with_colors(colors)
                    .with_radii([0.012]),
            );
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

        pub fn log_cloud(&mut self, _stamp_s: f64, _world: Vec<[f32; 3]>) {}

        pub fn log_tsdf(&self, _map: &dyn slam_map::TsdfMap, _stamp_s: f64) {}
    }
}

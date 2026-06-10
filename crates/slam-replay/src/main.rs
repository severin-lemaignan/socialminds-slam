//! `slam-replay`: drive a [`SlamSystem`] over recorded sensor input, emit a TUM trajectory.
//!
//! Inputs are recorded streams — IMU/scan CSVs, or topics streamed **directly from a
//! ROS1 bag** (`--bag`, no CSV extraction stage). Whatever is provided is merged into
//! one time-ordered event stream and fed to the chosen system — each system consumes the
//! streams it understands and ignores the rest. The "inputs → TUM trajectory" contract
//! the harness depends on never changes. With `--metrics` it also writes a
//! compute-metrics JSON sidecar.

mod config;
mod graph;
mod metrics;
mod viz;

use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{bail, Context, Result};
use clap::{Parser, ValueEnum};
use slam_baseline::{ImuDeadReckoning, Stationary};
use slam_frontend_scan::{
    ScanOdometry, ScanOdometryConfig, ScanToMapConfig, ScanToMapOdometry, Se2,
};
use slam_rig::SensorRig;
use slam_types::{FrameId, ImuSample, LaserScan2D, Pose, SlamSystem, Stamp, Trajectory, Vec3};

use metrics::ProcessingMetrics;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum System {
    /// Fixed pose; the sanity floor.
    Stationary,
    /// IMU strapdown integration.
    DeadReckoning,
    /// 2D scan-to-keyframe odometry (point-to-line ICP, ADR 0007).
    ScanMatching,
    /// Scan-to-submap odometry: 3D fans registered against a TSDF (ADR 0010).
    #[value(name = "scan-matching-3d")]
    ScanMatching3d,
}

#[derive(Parser, Debug)]
#[command(
    name = "slam-replay",
    about = "Run a SLAM system over recorded input and write a TUM trajectory."
)]
struct Args {
    /// Which system to run. (Flag named --baseline for harness compatibility.)
    #[arg(long = "baseline", value_enum)]
    system: System,

    /// IMU CSV input (`t gx gy gz ax ay az`). Repeatable for multi-IMU rigs; prefix
    /// with the sensor's URDF link name (`FRAME=FILE`) — requires `--urdf`.
    #[arg(long, value_name = "[FRAME=]FILE")]
    imu: Vec<String>,

    /// Scan CSV input (`t angle_min angle_increment range_min range_max n r…`).
    /// Repeatable for multi-lidar; prefix with the sensor's URDF link name
    /// (`FRAME=FILE`) to place it on the rig — requires `--urdf`.
    #[arg(long, value_name = "[FRAME=]FILE")]
    scan: Vec<String>,

    /// Robot URDF: resolves sensor frames and extrinsics (ADR 0009). Without it every
    /// scan is treated as taken at the base frame (the single-centred-lidar default).
    #[arg(long, value_name = "FILE")]
    urdf: Option<PathBuf>,

    /// YAML run configuration (ADR 0013): sensor set + rig source + ingest tuning.
    /// Mutually exclusive with the per-topic flags below.
    #[arg(long, value_name = "FILE", requires = "bag", conflicts_with_all = [
        "urdf", "rig_from_bag", "imu_topic", "gyro_topic", "accel_topic",
        "scan_topic", "depth_topic", "camera_info_topic", "color_topic",
    ])]
    config: Option<PathBuf>,

    /// Build the rig from the bag's own `/tf_static` (the recorded counterpart of the
    /// URDF's fixed joints — ADR 0009). Needs `--bag`.
    #[arg(long, requires = "bag", conflicts_with = "urdf")]
    rig_from_bag: bool,

    /// The body frame the rig is anchored at (a URDF link / tf frame name).
    #[arg(long, value_name = "LINK", default_value = "base_link")]
    base_frame: String,

    /// ROS1 bag input: stream topics directly, no CSV extraction stage. Select streams
    /// with `--imu-topic` (or `--gyro-topic` + `--accel-topic`) and/or `--scan-topic`.
    #[arg(long, value_name = "FILE", conflicts_with_all = ["imu", "scan"])]
    bag: Option<PathBuf>,

    /// Single 6-axis `sensor_msgs/Imu` topic to stream from `--bag`.
    #[arg(long, value_name = "TOPIC", requires = "bag", conflicts_with_all = ["gyro_topic", "accel_topic"])]
    imu_topic: Option<String>,

    /// Gyro half of a RealSense-style split IMU (with `--accel-topic`); the streams are
    /// merged with the gyro as time base, accel linearly interpolated.
    #[arg(long, value_name = "TOPIC", requires = "bag")]
    gyro_topic: Option<String>,

    /// Accel half of a RealSense-style split IMU (with `--gyro-topic`).
    #[arg(long, value_name = "TOPIC", requires = "bag")]
    accel_topic: Option<String>,

    /// `sensor_msgs/LaserScan` topic to stream from `--bag`.
    #[arg(long, value_name = "TOPIC", requires = "bag")]
    scan_topic: Option<String>,

    /// `nav_msgs/Odometry` topic to stream from `--bag` (wheel odometry — the motion
    /// prior, especially for IMU-less robots, ADR 0012).
    #[arg(long, value_name = "TOPIC", requires = "bag")]
    odom_topic: Option<String>,

    /// Disable loop closure entirely (open-loop odometry; for A/B comparisons).
    #[arg(long)]
    no_loops: bool,

    /// Override the loop-verification inlier threshold (default 0.55 suits clean
    /// laser fans; people-heavy depth clouds may need lower — experiment knob).
    #[arg(long, value_name = "FRACTION")]
    loop_min_inliers: Option<f64>,

    /// Disable the GTSAM pose graph: verified loops snap instead of optimising
    /// (for A/B comparisons of stage 3b).
    #[arg(long)]
    no_graph: bool,

    /// Depth `sensor_msgs/Image` topic to stream from `--bag` as back-projected point
    /// clouds (M4 RGB-D). Use the aligned depth stream when available.
    #[arg(long, value_name = "TOPIC", requires = "bag")]
    depth_topic: Option<String>,

    /// `CameraInfo` topic carrying the depth stream's intrinsics (ADR 0009). Defaults
    /// to the sibling of `--depth-topic` (`…/image_raw` → `…/camera_info`).
    #[arg(long, value_name = "TOPIC", requires = "depth_topic")]
    camera_info_topic: Option<String>,

    /// Colour image topic riding with an *aligned* depth stream (e.g.
    /// `/d400/color/image_raw`): depth points carry per-pixel RGB → coloured 3D map.
    #[arg(long, value_name = "TOPIC", requires = "depth_topic")]
    color_topic: Option<String>,

    /// Keep every Nth depth frame (30 fps is redundant at ≤ 2 m/s).
    #[arg(long, value_name = "N", default_value_t = 3)]
    depth_every: usize,

    /// Output TUM trajectory file. Defaults to stdout.
    #[arg(long, value_name = "FILE")]
    out: Option<PathBuf>,

    /// Write a compute-metrics JSON sidecar (latency, throughput, real-time factor).
    #[arg(long, value_name = "FILE")]
    metrics: Option<PathBuf>,

    /// Initialise the system's pose (and, for dead-reckoning, velocity) from the first
    /// pose(s) of this TUM ground-truth file. Gives odometry a fair start on real data.
    #[arg(long, value_name = "FILE")]
    init_pose_from_tum: Option<PathBuf>,

    /// Gravity magnitude (m/s²) for dead-reckoning.
    #[arg(long, default_value_t = slam_baseline::STANDARD_GRAVITY)]
    gravity: f64,

    /// Stream the run to the rerun viewer (ADR 0011): `spawn` (live), `connect`
    /// (running viewer), or `save:FILE.rrd` (record for timeline scrubbing).
    /// Needs a build with `--features viz`. Adds overhead — not for benchmarking.
    #[arg(long, value_name = "MODE")]
    rerun: Option<String>,

    /// Write the final TSDF submap (scan-matching-3d only) as a binary voxel dump:
    /// `STSD` magic, u32 version, f64 voxel size, u64 count, then per voxel
    /// `i32 ix, iy, iz; f32 tsdf, weight` (little-endian).
    #[arg(long, value_name = "FILE")]
    map_out: Option<PathBuf>,
}

/// Initial pose + velocity for a run.
struct InitialState {
    pose: Pose,
    velocity: Vec3,
}

impl Default for InitialState {
    fn default() -> Self {
        InitialState {
            pose: Pose::identity(),
            velocity: Vec3::zeros(),
        }
    }
}

/// Derive an initial pose and velocity from the first samples of a TUM trajectory.
/// Velocity is a finite difference of the first two positions (zero if fewer).
fn initial_state_from_tum(path: &Path) -> Result<InitialState> {
    let traj = Trajectory::read_tum_file(path)
        .with_context(|| format!("reading init pose from {}", path.display()))?;
    let poses = traj.poses();
    let first = poses
        .first()
        .with_context(|| format!("{} has no poses", path.display()))?;
    let velocity = match poses.get(1) {
        Some(second) => {
            let dt = (second.stamp - first.stamp).as_seconds();
            if dt > 0.0 {
                (second.pose.translation() - first.pose.translation()) / dt
            } else {
                Vec3::zeros()
            }
        }
        None => Vec3::zeros(),
    };
    Ok(InitialState {
        pose: first.pose,
        velocity,
    })
}

/// An exteroceptive event handed to the visualization hook with its estimate.
enum VizEvent<'a> {
    Scan(&'a LaserScan2D),
    Cloud(&'a slam_types::PointCloud),
}

/// Per-event visualization hook.
type ScanHook<'a> = &'a mut dyn FnMut(&VizEvent<'_>, &slam_types::StampedPose);

/// One time-stamped sensor event from any input stream.
enum Event<'a> {
    Imu(&'a ImuSample),
    Scan(&'a LaserScan2D),
    Cloud(&'a slam_types::PointCloud),
    Odom(&'a slam_types::OdomSample),
}

impl Event<'_> {
    fn stamp(&self) -> Stamp {
        match self {
            Event::Imu(s) => s.stamp,
            Event::Scan(s) => s.stamp,
            Event::Cloud(c) => c.stamp,
            Event::Odom(o) => o.stamp,
        }
    }
}

/// Merge the input streams into one stamp-ordered event sequence (stable two-pointer:
/// equal stamps deliver IMU first, so inertial state is current when a scan lands).
fn merged_events<'a>(
    imu: &'a [ImuSample],
    scans: &'a [LaserScan2D],
    clouds: &'a [slam_types::PointCloud],
    odometry: &'a [slam_types::OdomSample],
) -> Vec<Event<'a>> {
    let mut events: Vec<Event<'a>> =
        Vec::with_capacity(imu.len() + scans.len() + clouds.len() + odometry.len());
    events.extend(imu.iter().map(Event::Imu));
    events.extend(odometry.iter().map(Event::Odom));
    events.extend(scans.iter().map(Event::Scan));
    events.extend(clouds.iter().map(Event::Cloud));
    // Stable: equal stamps keep IMU (pushed first) ahead of exteroceptive events, so
    // inertial state is current when a scan/cloud lands.
    events.sort_by_key(|e| e.stamp());
    events
}

/// Run the system over the events, collecting the trajectory and per-event latencies.
/// An estimate is recorded whenever its stamp advances (no duplicates for ignored events).
fn run_timed(
    system: &mut dyn SlamSystem,
    events: &[Event],
    mut on_scan: Option<ScanHook<'_>>,
) -> (Trajectory, Vec<u64>, std::time::Duration) {
    let mut traj = Trajectory::new();
    let mut latencies = Vec::with_capacity(events.len());
    let mut last_stamp: Option<Stamp> = None;
    let start = Instant::now();
    for event in events {
        let t0 = Instant::now();
        match event {
            Event::Imu(sample) => system.process_imu(sample),
            Event::Scan(scan) => system.process_scan(scan),
            Event::Cloud(cloud) => system.process_points(cloud),
            Event::Odom(odom) => system.process_odometry(odom),
        }
        let est = system.current_estimate();
        latencies.push(t0.elapsed().as_nanos() as u64);
        if let Some(est) = est {
            if last_stamp != Some(est.stamp) {
                traj.push(est);
                last_stamp = Some(est.stamp);
            }
            // Visualization hook, outside the per-event latency clock.
            if let Some(hook) = on_scan.as_mut() {
                match event {
                    Event::Scan(scan) => hook(&VizEvent::Scan(scan), &est),
                    Event::Cloud(cloud) => hook(&VizEvent::Cloud(cloud), &est),
                    Event::Imu(_) | Event::Odom(_) => {}
                }
            }
        }
    }
    (traj, latencies, start.elapsed())
}

/// Write the binary TSDF voxel dump (see `--map-out` help for the format).
fn write_map_dump(map: &dyn slam_map::TsdfMap, path: &Path) -> Result<()> {
    use io::Write as _;
    let mut records: Vec<u8> = Vec::new();
    let mut count: u64 = 0;
    map.visit_voxels(&mut |ix, iy, iz, tsdf, weight| {
        for v in [ix, iy, iz] {
            records.extend_from_slice(&v.to_le_bytes());
        }
        records.extend_from_slice(&tsdf.to_le_bytes());
        records.extend_from_slice(&weight.to_le_bytes());
        count += 1;
    });
    let file = std::fs::File::create(path)
        .with_context(|| format!("creating map dump {}", path.display()))?;
    let mut w = io::BufWriter::new(file);
    w.write_all(b"STSD")?;
    w.write_all(&1u32.to_le_bytes())?;
    w.write_all(&map.config().voxel_size.to_le_bytes())?;
    w.write_all(&count.to_le_bytes())?;
    w.write_all(&records)?;
    eprintln!(
        "slam-replay: wrote {count} voxels ({:.1} cm grid) to {}",
        map.config().voxel_size * 100.0,
        path.display()
    );
    Ok(())
}

fn input_span_seconds(events: &[Event]) -> f64 {
    match (events.first(), events.last()) {
        (Some(a), Some(b)) => (b.stamp() - a.stamp()).as_seconds(),
        _ => 0.0,
    }
}

/// Resolve a frame name against the rig, tolerating tf1's leading slash.
fn resolve_frame(rig: &SensorRig, name: &str) -> Result<FrameId> {
    rig.resolve(name.trim_start_matches('/')).with_context(|| {
        format!("frame {name:?} is not a fixed frame of the rig — check --urdf/--base-frame")
    })
}

/// Stream the requested topics from a ROS1 bag in one pass, merging a split
/// gyro/accel pair into a single 6-axis IMU stream when asked for. Scans are tagged
/// with the frame their `header.frame_id` names when a rig is given.
type BagInputs = (
    Vec<ImuSample>,
    Vec<LaserScan2D>,
    Vec<slam_types::PointCloud>,
    Vec<slam_types::OdomSample>,
);

/// Load every stream named by a run configuration (ADR 0013) from the bag, in one
/// pass for scans+IMUs and one pass per depth camera.
fn load_bag_inputs_from_config(
    bag: &Path,
    cfg: &config::RunConfig,
    rig: Option<&SensorRig>,
) -> Result<BagInputs> {
    let scan_topics: Vec<&str> = cfg
        .sensors
        .scans
        .iter()
        .map(|sc| sc.topic.as_str())
        .collect();
    let mut imu_topics: Vec<&str> = Vec::new();
    for imu in &cfg.sensors.imus {
        match (&imu.topic, &imu.gyro_topic, &imu.accel_topic) {
            (Some(t), _, _) => imu_topics.push(t),
            (None, Some(g), Some(a)) => {
                imu_topics.push(g);
                imu_topics.push(a);
            }
            _ => unreachable!("validated at load"),
        }
    }

    let mut imu: Vec<ImuSample> = Vec::new();
    let mut scans: Vec<LaserScan2D> = Vec::new();
    if !scan_topics.is_empty() || !imu_topics.is_empty() {
        let mut streams = slam_datasets::read_streams_from_bag(bag, &imu_topics, &scan_topics)
            .with_context(|| format!("reading bag {}", bag.display()))?;
        for imu_cfg in &cfg.sensors.imus {
            let (mut stream, frame_topic) = match (&imu_cfg.topic, &imu_cfg.gyro_topic) {
                (Some(t), _) => (streams.imu.remove(t.as_str()).unwrap_or_default(), t),
                (None, Some(g)) => {
                    let accel = imu_cfg.accel_topic.as_ref().expect("validated");
                    (
                        slam_datasets::merge_split_imu(
                            &streams.imu.remove(g.as_str()).unwrap_or_default(),
                            &streams.imu.remove(accel.as_str()).unwrap_or_default(),
                        ),
                        g,
                    )
                }
                _ => unreachable!(),
            };
            if let (Some(rig), Some(frame_id)) = (rig, streams.imu_frames.get(frame_topic.as_str()))
            {
                let frame = resolve_frame(rig, frame_id)?;
                for s in &mut stream {
                    s.frame = frame;
                }
            }
            imu.extend(stream);
        }
        imu.sort_by_key(|s| s.stamp);

        for sc in &cfg.sensors.scans {
            let mut stream = streams.scans.remove(sc.topic.as_str()).unwrap_or_default();
            if let (Some(rig), Some(frame_id)) = (rig, streams.scan_frames.get(sc.topic.as_str())) {
                let frame = resolve_frame(rig, frame_id)?;
                for s in &mut stream {
                    s.frame = frame;
                }
            }
            scans.extend(stream);
        }
        scans.sort_by_key(|s| s.stamp);
    }

    let mut clouds: Vec<slam_types::PointCloud> = Vec::new();
    for d in &cfg.sensors.depth {
        let depth_cfg = slam_datasets::DepthConfig {
            target_spacing: d.target_spacing,
            min_stride: d.min_stride,
            max_points: d.max_points,
            min_range: d.min_range,
            max_range: d.max_range,
        };
        let (mut cs, frame_id) = slam_datasets::read_depth_clouds(
            bag,
            &d.topic,
            &d.info_topic(),
            d.color.as_deref(),
            &depth_cfg,
            d.every_nth,
        )
        .with_context(|| format!("reading depth from {} / {}", d.topic, d.info_topic()))?;
        if let Some(rig) = rig {
            let frame = resolve_frame(rig, &frame_id)?;
            for c in &mut cs {
                c.frame = frame;
            }
        }
        eprintln!(
            "slam-replay: {} depth clouds from {} (frame {frame_id:?}, every {}th)",
            cs.len(),
            d.topic,
            d.every_nth.max(1)
        );
        clouds.extend(cs);
    }
    clouds.sort_by_key(|c| c.stamp);

    let mut odometry: Vec<slam_types::OdomSample> = Vec::new();
    for o in &cfg.sensors.odometry {
        let (samples, child) = slam_datasets::read_odometry(bag, &o.topic)
            .with_context(|| format!("reading odometry from {}", o.topic))?;
        eprintln!(
            "slam-replay: {} odometry samples from {} (child frame {child:?})",
            samples.len(),
            o.topic
        );
        odometry.extend(samples);
    }
    odometry.sort_by_key(|s| s.stamp);
    Ok((imu, scans, clouds, odometry))
}

fn load_bag_inputs(bag: &Path, args: &Args, rig: Option<&SensorRig>) -> Result<BagInputs> {
    let imu_topics: Vec<&str> = match (&args.imu_topic, &args.gyro_topic, &args.accel_topic) {
        (Some(imu), None, None) => vec![imu],
        (None, Some(gyro), Some(accel)) => vec![gyro, accel],
        (None, None, None) => vec![],
        _ => bail!("pass both --gyro-topic and --accel-topic, or neither"),
    };
    let scan_topics: Vec<&str> = args.scan_topic.iter().map(String::as_str).collect();
    if imu_topics.is_empty() && scan_topics.is_empty() {
        bail!(
            "--bag needs at least one stream: --imu-topic (or --gyro-topic + --accel-topic) \
             and/or --scan-topic"
        );
    }

    let mut streams = slam_datasets::read_streams_from_bag(bag, &imu_topics, &scan_topics)
        .with_context(|| format!("reading bag {}", bag.display()))?;
    let (mut imu, imu_topic) = match (&args.imu_topic, &args.gyro_topic, &args.accel_topic) {
        (Some(topic), _, _) => (
            streams.imu.remove(topic.as_str()).unwrap_or_default(),
            Some(topic),
        ),
        (_, Some(gyro), Some(accel)) => (
            slam_datasets::merge_split_imu(
                &streams.imu.remove(gyro.as_str()).unwrap_or_default(),
                &streams.imu.remove(accel.as_str()).unwrap_or_default(),
            ),
            Some(gyro),
        ),
        _ => (Vec::new(), None),
    };
    if let (Some(rig), Some(topic)) = (rig, imu_topic) {
        if let Some(frame_id) = streams.imu_frames.get(topic.as_str()) {
            let frame = resolve_frame(rig, frame_id)?;
            for s in &mut imu {
                s.frame = frame;
            }
        }
    }
    let mut clouds = Vec::new();
    if let Some(depth_topic) = &args.depth_topic {
        let info_topic = args.camera_info_topic.clone().unwrap_or_else(|| {
            // RealSense layout: …/image_raw → …/camera_info.
            match depth_topic.rfind('/') {
                Some(i) => format!("{}/camera_info", &depth_topic[..i]),
                None => format!("{depth_topic}/camera_info"),
            }
        });
        let (mut cs, frame_id) = slam_datasets::read_depth_clouds(
            bag,
            depth_topic,
            &info_topic,
            args.color_topic.as_deref(),
            &slam_datasets::DepthConfig::default(),
            args.depth_every,
        )
        .with_context(|| format!("reading depth from {depth_topic} / {info_topic}"))?;
        if let Some(rig) = rig {
            let frame = resolve_frame(rig, &frame_id)?;
            for c in &mut cs {
                c.frame = frame;
            }
        }
        eprintln!(
            "slam-replay: {} depth clouds from {depth_topic} (frame {frame_id:?}, every {}th)",
            cs.len(),
            args.depth_every.max(1),
        );
        clouds = cs;
    }
    let scans = match &args.scan_topic {
        Some(topic) => {
            let mut scans = streams.scans.remove(topic.as_str()).unwrap_or_default();
            if let Some(rig) = rig {
                // (rig from --urdf or --rig-from-bag alike)
                // The messages name their own frame (ADR 0009).
                let frame_id = streams
                    .scan_frames
                    .get(topic.as_str())
                    .with_context(|| format!("no header.frame_id seen on {topic}"))?;
                let frame = resolve_frame(rig, frame_id)?;
                for s in &mut scans {
                    s.frame = frame;
                }
            }
            scans
        }
        None => Vec::new(),
    };
    let odometry = match &args.odom_topic {
        Some(topic) => {
            let (samples, child) = slam_datasets::read_odometry(bag, topic)
                .with_context(|| format!("reading odometry from {topic}"))?;
            eprintln!(
                "slam-replay: {} odometry samples from {topic} (child frame {child:?})",
                samples.len()
            );
            samples
        }
        None => Vec::new(),
    };
    Ok((imu, scans, clouds, odometry))
}

/// Split a `[FRAME=]FILE` CSV spec and resolve the frame against the rig.
fn frame_and_path(spec: &str, rig: Option<&SensorRig>, flag: &str) -> Result<(FrameId, PathBuf)> {
    match spec.split_once('=') {
        Some((name, file)) => match rig {
            Some(rig) => Ok((resolve_frame(rig, name)?, PathBuf::from(file))),
            None => bail!("--{flag} {name}=… needs --urdf to resolve the frame"),
        },
        None => Ok((FrameId::BASE, PathBuf::from(spec))),
    }
}

/// Load and merge the `--scan [FRAME=]FILE` CSVs into one stamp-sorted stream.
fn load_scan_csvs(specs: &[String], rig: Option<&SensorRig>) -> Result<Vec<LaserScan2D>> {
    let mut scans: Vec<LaserScan2D> = Vec::new();
    for spec in specs {
        let (frame, path) = frame_and_path(spec, rig, "scan")?;
        let file = std::fs::File::open(&path)
            .with_context(|| format!("opening scan file {}", path.display()))?;
        let mut stream = slam_types::read_scans(io::BufReader::new(file))
            .with_context(|| format!("reading scan file {}", path.display()))?;
        for s in &mut stream {
            s.frame = frame;
        }
        scans.extend(stream);
    }
    // Multi-lidar CSVs interleave; the event loop needs one time-ordered stream.
    scans.sort_by_key(|s| s.stamp);
    Ok(scans)
}

/// Load and merge the `--imu [FRAME=]FILE` CSVs into one stamp-sorted stream.
fn load_imu_csvs(specs: &[String], rig: Option<&SensorRig>) -> Result<Vec<ImuSample>> {
    let mut samples: Vec<ImuSample> = Vec::new();
    for spec in specs {
        let (frame, path) = frame_and_path(spec, rig, "imu")?;
        let file = std::fs::File::open(&path)
            .with_context(|| format!("opening IMU file {}", path.display()))?;
        let stream = slam_types::read_imu(io::BufReader::new(file))
            .with_context(|| format!("reading IMU file {}", path.display()))?;
        samples.extend(stream.into_iter().map(|s| s.in_frame(frame)));
    }
    samples.sort_by_key(|s| s.stamp);
    Ok(samples)
}

/// The rig's SE(3) extrinsics table for the scan front-end, warning on lidar frames
/// mounted visibly out of the base's motion plane (the front-end models the lidar as
/// scanning that plane; mounting tilt is not yet compensated — only dynamic IMU tilt).
fn extrinsics_table(rig: &SensorRig, used: &[FrameId]) -> Vec<Pose> {
    const PLANARITY_WARN_RAD: f64 = 0.017; // ≈ 1°
    for &frame in used {
        let (_, out_of_plane) = Se2::planar_projection_of(&rig.extrinsic(frame));
        if out_of_plane > PLANARITY_WARN_RAD {
            eprintln!(
                "slam-replay: warning: lidar frame {:?} is mounted {:.1}° out of the base \
                 plane; the planar front-end assumes ≈ 0°",
                rig.frame_name(frame),
                out_of_plane.to_degrees(),
            );
        }
    }
    rig.extrinsics().to_vec()
}

fn main() -> Result<()> {
    let args = Args::parse();

    let run_cfg = args
        .config
        .as_deref()
        .map(config::RunConfig::load)
        .transpose()?;
    let (urdf_arg, rig_from_bag_arg, base_frame_arg) = match &run_cfg {
        Some(rc) => (
            rc.rig.urdf.clone(),
            rc.rig.source == config::RigSource::Bag,
            rc.rig.base_frame.clone(),
        ),
        None => (
            args.urdf.clone(),
            args.rig_from_bag,
            args.base_frame.clone(),
        ),
    };
    let rig = match (&urdf_arg, rig_from_bag_arg) {
        (Some(path), _) => Some(
            SensorRig::from_urdf_file(path, &base_frame_arg)
                .with_context(|| format!("building rig from {}", path.display()))?,
        ),
        (None, true) => {
            let bag = args
                .bag
                .as_ref()
                .expect("clap: --rig-from-bag requires --bag");
            let tfs = slam_datasets::read_static_transforms(bag)
                .with_context(|| format!("reading /tf_static from {}", bag.display()))?;
            let edges: Vec<(String, String, Pose)> = tfs
                .into_iter()
                .map(|t| (t.parent, t.child, t.transform))
                .collect();
            let rig = SensorRig::from_transforms(&base_frame_arg, &edges)
                .context("building rig from the bag's /tf_static")?;
            eprintln!(
                "slam-replay: rig from /tf_static: {} frames around {:?}",
                rig.len(),
                base_frame_arg
            );
            Some(rig)
        }
        (None, false) => None,
    };

    let (imu, scans, clouds, odometry): BagInputs = if let Some(bag) = &args.bag {
        match &run_cfg {
            Some(rc) => load_bag_inputs_from_config(bag, rc, rig.as_ref())?,
            None => load_bag_inputs(bag, &args, rig.as_ref())?,
        }
    } else {
        (
            load_imu_csvs(&args.imu, rig.as_ref())?,
            load_scan_csvs(&args.scan, rig.as_ref())?,
            Vec::new(),
            Vec::new(),
        )
    };

    // Each system needs its primary stream; running it on silence is a usage error.
    match args.system {
        System::Stationary | System::DeadReckoning if imu.is_empty() => {
            bail!("this system consumes IMU data: pass --imu, or --bag with IMU topics")
        }
        System::ScanMatching if scans.is_empty() => {
            bail!("scan-matching consumes laser scans: pass --scan, or --bag with --scan-topic")
        }
        System::ScanMatching3d if scans.is_empty() && clouds.is_empty() => {
            bail!(
                "scan-matching-3d consumes scans and/or depth clouds: pass --scan, or                  --bag with --scan-topic / --depth-topic"
            )
        }
        _ => {}
    }

    let init = match &args.init_pose_from_tum {
        Some(path) => initial_state_from_tum(path)?,
        None => InitialState::default(),
    };

    let viz_extrinsics = match &rig {
        Some(rig) => rig.extrinsics().to_vec(),
        None => Vec::new(),
    };
    // ScanToMap stays concrete so its TSDF is reachable for --map-out / --rerun.
    enum Engine {
        Dyn(Box<dyn SlamSystem>),
        ScanToMap(Box<ScanToMapOdometry>),
    }
    impl Engine {
        fn as_dyn(&mut self) -> &mut dyn SlamSystem {
            match self {
                Engine::Dyn(b) => b.as_mut(),
                Engine::ScanToMap(s) => s.as_mut(),
            }
        }
    }
    let mut engine = match args.system {
        System::Stationary => Engine::Dyn(Box::new(Stationary::anchored_at(init.pose))),
        System::DeadReckoning => Engine::Dyn(Box::new(ImuDeadReckoning::with_initial_state(
            init.pose,
            init.velocity,
            args.gravity,
        ))),
        System::ScanMatching | System::ScanMatching3d => {
            let extrinsics = match &rig {
                Some(rig) => {
                    let mut used: Vec<FrameId> = scans.iter().map(|s| s.frame).collect();
                    used.sort_unstable();
                    used.dedup();
                    extrinsics_table(rig, &used)
                }
                None => Vec::new(),
            };
            match args.system {
                System::ScanMatching => Engine::Dyn(Box::new(ScanOdometry::with_extrinsics(
                    init.pose,
                    ScanOdometryConfig::default(),
                    extrinsics,
                ))),
                _ => {
                    let mut cfg = ScanToMapConfig::default();
                    if args.no_loops {
                        cfg.loop_radius = 0.0;
                    }
                    if let Some(v) = args.loop_min_inliers {
                        cfg.loop_min_inliers = v;
                    }
                    let mut odo = ScanToMapOdometry::with_extrinsics(init.pose, cfg, extrinsics);
                    // Verified loops feed the GTSAM pose graph (ADR 0010 stage 3b).
                    if !args.no_graph && !args.no_loops {
                        odo.set_graph(Box::new(graph::GtsamAnchorGraph::default()));
                    }
                    Engine::ScanToMap(Box::new(odo))
                }
            }
        }
    };

    let mut viz_sink = match &args.rerun {
        Some(mode) => {
            let v = viz::Viz::new(mode)?;
            if let Some(path) = &args.init_pose_from_tum {
                if let Ok(gt) = Trajectory::read_tum_file(path) {
                    v.log_groundtruth(&gt);
                }
            }
            Some(v)
        }
        None => None,
    };
    // Per-scan viz hook: lift beams through the (static) extrinsic, into world via the
    // estimate. Attitude is not replayed here — close enough for inspection.
    let mut hook;
    let on_scan: Option<ScanHook<'_>> = match viz_sink.as_mut() {
        Some(viz) => {
            hook = move |event: &VizEvent<'_>, est: &slam_types::StampedPose| {
                let frame = match event {
                    VizEvent::Scan(scan) => scan.frame,
                    VizEvent::Cloud(cloud) => cloud.frame,
                };
                let t_bs = viz_extrinsics
                    .get(frame.0 as usize)
                    .copied()
                    .unwrap_or_else(Pose::identity);
                match event {
                    VizEvent::Scan(scan) => {
                        let world: Vec<[f32; 3]> = scan
                            .ranges
                            .iter()
                            .enumerate()
                            .filter_map(|(i, &r)| {
                                let r = r as f64;
                                if !r.is_finite() || r < scan.range_min || r > scan.range_max {
                                    return None;
                                }
                                let a = scan.angle_min + i as f64 * scan.angle_increment;
                                let p = est.pose.transform_point(t_bs.transform_point(Vec3::new(
                                    r * a.cos(),
                                    r * a.sin(),
                                    0.0,
                                )));
                                Some([p.x as f32, p.y as f32, p.z as f32])
                            })
                            .collect();
                        viz.log_scan(est.stamp.as_seconds(), &est.pose, world);
                    }
                    VizEvent::Cloud(cloud) => {
                        // Downsample for display; the engine keeps the full cloud.
                        let world: Vec<[f32; 3]> = cloud
                            .points
                            .iter()
                            .step_by(4)
                            .map(|&q| {
                                let p = est.pose.transform_point(t_bs.transform_point(q));
                                [p.x as f32, p.y as f32, p.z as f32]
                            })
                            .collect();
                        let colors: Vec<[u8; 3]> =
                            cloud.colors.iter().step_by(4).copied().collect();
                        viz.log_cloud(est.stamp.as_seconds(), &est.pose, world, colors);
                    }
                }
            };
            Some(&mut hook)
        }
        None => None,
    };

    let events = merged_events(&imu, &scans, &clouds, &odometry);
    let (traj, latencies, wall) = run_timed(engine.as_dyn(), &events, on_scan);

    if let Engine::ScanToMap(odo) = &engine {
        let st = odo.stats();
        eprintln!(
            "slam-replay: front-end health: {} matched / {} coasted / {} skipped /              {} degenerate; {} submap hand-overs, {} verified loop closures",
            st.matched,
            st.coasted,
            st.skipped,
            st.degenerate,
            st.keyframes,
            odo.loop_closures().len(),
        );
        if let Some(path) = &args.map_out {
            write_map_dump(odo.map(), path)?;
        }
        if let Some(viz) = &viz_sink {
            let end = events.last().map_or(0.0, |e| e.stamp().as_seconds());
            let submaps: Vec<(f64, f64, f64, &dyn slam_map::TsdfMap)> = odo
                .submaps_3d()
                .into_iter()
                .map(|(a, m)| (a.x, a.y, a.theta, m as &dyn slam_map::TsdfMap))
                .collect();
            viz.log_tsdf(&submaps, end);
        }
    } else if args.map_out.is_some() {
        bail!("--map-out needs --baseline scan-matching-3d (the TSDF front-end)");
    }

    let system = engine.as_dyn();
    let span = input_span_seconds(&events);
    let m = ProcessingMetrics::new(system.name(), events.len(), span, wall, latencies);
    eprintln!(
        "slam-replay: ran '{}' over {} events ({} imu, {} scans) -> {} poses ({:.0}x real-time)",
        system.name(),
        events.len(),
        imu.len(),
        scans.len(),
        traj.len(),
        m.real_time_factor,
    );

    match &args.out {
        Some(path) => traj
            .write_tum_file(path)
            .with_context(|| format!("writing {}", path.display()))?,
        None => {
            let stdout = io::stdout();
            traj.write_tum(stdout.lock())?;
            io::stdout().flush()?;
        }
    }

    if let Some(path) = &args.metrics {
        std::fs::write(path, m.to_json())
            .with_context(|| format!("writing metrics {}", path.display()))?;
    }

    Ok(())
}

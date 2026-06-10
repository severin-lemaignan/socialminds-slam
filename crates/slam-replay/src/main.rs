//! `slam-replay`: drive a [`SlamSystem`] over recorded sensor input, emit a TUM trajectory.
//!
//! Inputs are recorded streams — IMU/scan CSVs, or topics streamed **directly from a
//! ROS1 bag** (`--bag`, no CSV extraction stage). Whatever is provided is merged into
//! one time-ordered event stream and fed to the chosen system — each system consumes the
//! streams it understands and ignores the rest. The "inputs → TUM trajectory" contract
//! the harness depends on never changes. With `--metrics` it also writes a
//! compute-metrics JSON sidecar.

mod metrics;

use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{bail, Context, Result};
use clap::{Parser, ValueEnum};
use slam_baseline::{ImuDeadReckoning, Stationary};
use slam_frontend_scan::{ScanOdometry, ScanOdometryConfig};
use slam_types::{ImuSample, LaserScan2D, Pose, SlamSystem, Stamp, Trajectory, Vec3};

use metrics::ProcessingMetrics;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum System {
    /// Fixed pose; the sanity floor.
    Stationary,
    /// IMU strapdown integration.
    DeadReckoning,
    /// 2D scan-to-keyframe odometry (point-to-line ICP, ADR 0007).
    ScanMatching,
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

    /// IMU CSV input (`t gx gy gz ax ay az`).
    #[arg(long, value_name = "FILE")]
    imu: Option<PathBuf>,

    /// Scan CSV input (`t angle_min angle_increment range_min range_max n r…`).
    #[arg(long, value_name = "FILE")]
    scan: Option<PathBuf>,

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

/// One time-stamped sensor event from any input stream.
enum Event<'a> {
    Imu(&'a ImuSample),
    Scan(&'a LaserScan2D),
}

impl Event<'_> {
    fn stamp(&self) -> Stamp {
        match self {
            Event::Imu(s) => s.stamp,
            Event::Scan(s) => s.stamp,
        }
    }
}

/// Merge the input streams into one stamp-ordered event sequence (stable two-pointer:
/// equal stamps deliver IMU first, so inertial state is current when a scan lands).
fn merged_events<'a>(imu: &'a [ImuSample], scans: &'a [LaserScan2D]) -> Vec<Event<'a>> {
    let mut events = Vec::with_capacity(imu.len() + scans.len());
    let (mut i, mut s) = (0, 0);
    while i < imu.len() || s < scans.len() {
        let take_imu = match (imu.get(i), scans.get(s)) {
            (Some(a), Some(b)) => a.stamp <= b.stamp,
            (Some(_), None) => true,
            _ => false,
        };
        if take_imu {
            events.push(Event::Imu(&imu[i]));
            i += 1;
        } else {
            events.push(Event::Scan(&scans[s]));
            s += 1;
        }
    }
    events
}

/// Run the system over the events, collecting the trajectory and per-event latencies.
/// An estimate is recorded whenever its stamp advances (no duplicates for ignored events).
fn run_timed(
    system: &mut dyn SlamSystem,
    events: &[Event],
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
        }
        let est = system.current_estimate();
        latencies.push(t0.elapsed().as_nanos() as u64);
        if let Some(est) = est {
            if last_stamp != Some(est.stamp) {
                traj.push(est);
                last_stamp = Some(est.stamp);
            }
        }
    }
    (traj, latencies, start.elapsed())
}

fn input_span_seconds(events: &[Event]) -> f64 {
    match (events.first(), events.last()) {
        (Some(a), Some(b)) => (b.stamp() - a.stamp()).as_seconds(),
        _ => 0.0,
    }
}

/// Stream the requested topics from a ROS1 bag in one pass, merging a split
/// gyro/accel pair into a single 6-axis IMU stream when asked for.
fn load_bag_inputs(bag: &Path, args: &Args) -> Result<(Vec<ImuSample>, Vec<LaserScan2D>)> {
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
    let imu = match (&args.imu_topic, &args.gyro_topic, &args.accel_topic) {
        (Some(topic), _, _) => streams.imu.remove(topic.as_str()).unwrap_or_default(),
        (_, Some(gyro), Some(accel)) => slam_datasets::merge_split_imu(
            &streams.imu.remove(gyro.as_str()).unwrap_or_default(),
            &streams.imu.remove(accel.as_str()).unwrap_or_default(),
        ),
        _ => Vec::new(),
    };
    let scans = match &args.scan_topic {
        Some(topic) => streams.scans.remove(topic.as_str()).unwrap_or_default(),
        None => Vec::new(),
    };
    Ok((imu, scans))
}

fn main() -> Result<()> {
    let args = Args::parse();

    let (imu, scans): (Vec<ImuSample>, Vec<LaserScan2D>) = if let Some(bag) = &args.bag {
        load_bag_inputs(bag, &args)?
    } else {
        let imu = match &args.imu {
            Some(path) => {
                let file = std::fs::File::open(path)
                    .with_context(|| format!("opening IMU file {}", path.display()))?;
                slam_types::read_imu(io::BufReader::new(file))
                    .with_context(|| format!("reading IMU file {}", path.display()))?
            }
            None => Vec::new(),
        };
        let scans = match &args.scan {
            Some(path) => {
                let file = std::fs::File::open(path)
                    .with_context(|| format!("opening scan file {}", path.display()))?;
                slam_types::read_scans(io::BufReader::new(file))
                    .with_context(|| format!("reading scan file {}", path.display()))?
            }
            None => Vec::new(),
        };
        (imu, scans)
    };

    // Each system needs its primary stream; running it on silence is a usage error.
    match args.system {
        System::Stationary | System::DeadReckoning if imu.is_empty() => {
            bail!("this system consumes IMU data: pass --imu, or --bag with IMU topics")
        }
        System::ScanMatching if scans.is_empty() => {
            bail!("scan-matching consumes laser scans: pass --scan, or --bag with --scan-topic")
        }
        _ => {}
    }

    let init = match &args.init_pose_from_tum {
        Some(path) => initial_state_from_tum(path)?,
        None => InitialState::default(),
    };

    let mut system: Box<dyn SlamSystem> = match args.system {
        System::Stationary => Box::new(Stationary::anchored_at(init.pose)),
        System::DeadReckoning => Box::new(ImuDeadReckoning::with_initial_state(
            init.pose,
            init.velocity,
            args.gravity,
        )),
        System::ScanMatching => Box::new(ScanOdometry::anchored_at(
            init.pose,
            ScanOdometryConfig::default(),
        )),
    };

    let events = merged_events(&imu, &scans);
    let (traj, latencies, wall) = run_timed(system.as_mut(), &events);
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

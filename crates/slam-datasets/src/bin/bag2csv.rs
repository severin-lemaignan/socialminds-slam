//! `slam-bag2csv`: extract several sensor streams from a ROS1 bag in **one pass**.
//!
//! Decompression dominates bag reading (OpenLORIS bags are bz2 inside), so extracting
//! gyro + accel + scan together costs one decompression instead of three. Used by the
//! evaluation harness when materialising OpenLORIS sequences.
//!
//! ```text
//! slam-bag2csv --bag cafe1-1.bag \
//!     --imu /d400/gyro/sample=gyro.csv --imu /d400/accel/sample=accel.csv \
//!     --scan /scan=scan.csv
//! ```

use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "slam-bag2csv",
    about = "Extract multiple topics from a ROS1 bag to CSV files in a single pass."
)]
struct Args {
    /// Input ROS1 `.bag` file.
    #[arg(long, value_name = "FILE")]
    bag: PathBuf,

    /// IMU topic to extract and where to write it; repeatable.
    #[arg(long = "imu", value_name = "TOPIC=FILE")]
    imu: Vec<String>,

    /// Laser-scan topic to extract and where to write it; repeatable.
    #[arg(long = "scan", value_name = "TOPIC=FILE")]
    scan: Vec<String>,
}

/// Split a `TOPIC=FILE` argument.
fn parse_pair(arg: &str) -> Result<(&str, PathBuf)> {
    match arg.split_once('=') {
        Some((topic, file)) if !topic.is_empty() && !file.is_empty() => {
            Ok((topic, PathBuf::from(file)))
        }
        _ => bail!("expected TOPIC=FILE, got {arg:?}"),
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let imu: Vec<(&str, PathBuf)> = args
        .imu
        .iter()
        .map(|s| parse_pair(s))
        .collect::<Result<_>>()?;
    let scan: Vec<(&str, PathBuf)> = args
        .scan
        .iter()
        .map(|s| parse_pair(s))
        .collect::<Result<_>>()?;
    if imu.is_empty() && scan.is_empty() {
        bail!("nothing to extract: pass at least one --imu or --scan TOPIC=FILE");
    }

    let imu_topics: Vec<&str> = imu.iter().map(|(t, _)| *t).collect();
    let scan_topics: Vec<&str> = scan.iter().map(|(t, _)| *t).collect();
    let streams = slam_datasets::read_streams_from_bag(&args.bag, &imu_topics, &scan_topics)
        .with_context(|| format!("reading {}", args.bag.display()))?;

    for (topic, path) in &imu {
        let samples = &streams.imu[*topic];
        let file = std::fs::File::create(path)
            .with_context(|| format!("creating {}", path.display()))?;
        slam_types::write_imu(samples, std::io::BufWriter::new(file))?;
        eprintln!(
            "slam-bag2csv: {} -> {} ({} samples)",
            topic,
            path.display(),
            samples.len()
        );
    }
    for (topic, path) in &scan {
        let scans = &streams.scans[*topic];
        let file = std::fs::File::create(path)
            .with_context(|| format!("creating {}", path.display()))?;
        slam_types::write_scans(scans, std::io::BufWriter::new(file))?;
        eprintln!(
            "slam-bag2csv: {} -> {} ({} scans)",
            topic,
            path.display(),
            scans.len()
        );
    }
    Ok(())
}

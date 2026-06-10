//! Dataset ingestion for the SLAM engine.
//!
//! Reads sensor streams from recorded logs into engine types. The first source is the
//! **ROS1 bag** (used by OpenLORIS-Scene), read by our own indexed reader ([`bag`],
//! ADR 0008) with no ROS install required. Extracted so far: IMU (`sensor_msgs/Imu`)
//! and planar laser scans (`sensor_msgs/LaserScan`); RGB-D extraction lands with the
//! visual front-end.
//!
//! Design: the engine consumes the simple [`slam_types`] formats, so this crate's job is
//! purely *log format → engine types*. The `slam-bag2csv` (multi-topic, one pass) and
//! `slam-bag2imu` / `slam-bag2scan` binaries expose the readers for the evaluation
//! harness.

#![forbid(unsafe_code)]

pub mod bag;
mod imu_msg;
mod scan_msg;
mod tf_msg;

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use slam_types::{ImuSample, LaserScan2D};

use bag::BagFile;
pub use imu_msg::parse_imu;
pub use scan_msg::parse_scan;
pub use tf_msg::{parse_tf_message, StaticTransform};

/// ROS message type string for IMU data.
pub const IMU_MSG_TYPE: &str = "sensor_msgs/Imu";
/// ROS message type string for planar laser scans.
pub const SCAN_MSG_TYPE: &str = "sensor_msgs/LaserScan";

/// Errors from reading a dataset log.
#[derive(Debug, thiserror::Error)]
pub enum BagError {
    #[error("opening bag {path:?}: {source}")]
    Open {
        path: String,
        source: std::io::Error,
    },
    #[error("I/O reading bag: {0}")]
    Io(#[from] std::io::Error),
    #[error("malformed bag: {0}")]
    Format(&'static str),
    #[error("decompressing chunk: {0}")]
    Decompress(String),
    #[error("malformed sensor_msgs/Imu message: {0}")]
    ImuDecode(&'static str),
    #[error("malformed sensor_msgs/LaserScan message: {0}")]
    ScanDecode(&'static str),
    #[error("no {0} topic found in bag")]
    NoTopic(&'static str),
    #[error("multiple {0} topics present ({1}); pass one explicitly")]
    AmbiguousTopic(&'static str, String),
    #[error("topic {0:?} not found in bag, or it is not {1}")]
    TopicNotFound(String, &'static str),
}

/// A topic and its ROS message type.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct TopicInfo {
    pub topic: String,
    pub message_type: String,
}

/// Map connection id → (topic, message type), from the bag's index.
fn connection_map(bag: &BagFile) -> BTreeMap<u32, (String, String)> {
    bag.connections()
        .iter()
        .map(|c| (c.id, (c.topic.clone(), c.message_type.clone())))
        .collect()
}

/// List the topics (and message types) present in a bag.
pub fn list_topics<P: AsRef<Path>>(path: P) -> Result<Vec<TopicInfo>, BagError> {
    let bag = BagFile::open(path)?;
    let mut topics: BTreeMap<String, String> = BTreeMap::new();
    for c in bag.connections() {
        topics.insert(c.topic.clone(), c.message_type.clone());
    }
    Ok(topics
        .into_iter()
        .map(|(topic, message_type)| TopicInfo {
            topic,
            message_type,
        })
        .collect())
}

/// All sensor streams pulled from a bag, keyed by topic, each time-sorted.
#[derive(Debug, Default)]
pub struct BagStreams {
    pub imu: BTreeMap<String, Vec<ImuSample>>,
    pub scans: BTreeMap<String, Vec<LaserScan2D>>,
    /// `header.frame_id` of each scan topic (first message wins) — the URDF link name
    /// to resolve against the rig (ADR 0009).
    pub scan_frames: BTreeMap<String, String>,
    /// Likewise for each IMU topic (multi-IMU rigs, ADR 0009).
    pub imu_frames: BTreeMap<String, String>,
}

/// Resolve a (possibly auto-selected) topic request against the bag's connections.
///
/// `None` auto-selects the unique topic of `msg_type`; an error if zero or several.
fn resolve_topic(
    conns: &BTreeMap<u32, (String, String)>,
    requested: Option<&str>,
    msg_type: &'static str,
) -> Result<String, BagError> {
    let candidates: Vec<&str> = {
        let mut t: Vec<&str> = conns
            .values()
            .filter(|(_, tp)| tp == msg_type)
            .map(|(topic, _)| topic.as_str())
            .collect();
        t.sort_unstable();
        t.dedup();
        t
    };
    match requested {
        Some(topic) if candidates.contains(&topic) => Ok(topic.to_string()),
        Some(topic) => Err(BagError::TopicNotFound(topic.to_string(), msg_type)),
        None => match candidates.len() {
            0 => Err(BagError::NoTopic(msg_type)),
            1 => Ok(candidates[0].to_string()),
            _ => Err(BagError::AmbiguousTopic(msg_type, candidates.join(", "))),
        },
    }
}

/// What a connection id contributes to which output stream.
#[derive(Clone)]
enum Target {
    Imu(String),
    Scan(String),
}

/// Read several topics in **one pass** over the bag.
///
/// Decompression dominates extraction cost (OpenLORIS bags are bz2 inside), and the
/// reader is index-driven (ADR 0008): only chunks containing at least one requested
/// connection are decompressed, so the cost is proportional to the requested data, not
/// the bag size. Every requested topic must exist with the right message type.
pub fn read_streams_from_bag<P: AsRef<Path>>(
    path: P,
    imu_topics: &[&str],
    scan_topics: &[&str],
) -> Result<BagStreams, BagError> {
    let mut bag = BagFile::open(path)?;
    let conns = connection_map(&bag);

    // Validate requests and map connection ids to output streams. (A topic may be
    // carried by several connections.)
    let mut targets: BTreeMap<u32, Target> = BTreeMap::new();
    let mut out = BagStreams::default();
    for (&requested, msg_type) in imu_topics
        .iter()
        .map(|t| (t, IMU_MSG_TYPE))
        .chain(scan_topics.iter().map(|t| (t, SCAN_MSG_TYPE)))
    {
        let ids: Vec<u32> = conns
            .iter()
            .filter(|(_, (topic, tp))| topic == requested && tp == msg_type)
            .map(|(&id, _)| id)
            .collect();
        if ids.is_empty() {
            return Err(BagError::TopicNotFound(requested.to_string(), msg_type));
        }
        for id in ids {
            targets.insert(
                id,
                if msg_type == IMU_MSG_TYPE {
                    out.imu.entry(requested.to_string()).or_default();
                    Target::Imu(requested.to_string())
                } else {
                    out.scans.entry(requested.to_string()).or_default();
                    Target::Scan(requested.to_string())
                },
            );
        }
    }

    let wanted: BTreeSet<u32> = targets.keys().copied().collect();
    bag.for_each_message(&wanted, |conn, data| {
        match targets.get(&conn) {
            Some(Target::Imu(topic)) => {
                let (sample, frame_id) = parse_imu(data)?;
                out.imu
                    .get_mut(topic)
                    .expect("stream pre-created")
                    .push(sample);
                out.imu_frames.entry(topic.clone()).or_insert(frame_id);
            }
            Some(Target::Scan(topic)) => {
                let (scan, frame_id) = parse_scan(data)?;
                out.scans
                    .get_mut(topic)
                    .expect("stream pre-created")
                    .push(scan);
                out.scan_frames.entry(topic.clone()).or_insert(frame_id);
            }
            None => {}
        }
        Ok(())
    })?;

    for samples in out.imu.values_mut() {
        samples.sort_by_key(|s| s.stamp);
    }
    for scans in out.scans.values_mut() {
        scans.sort_by_key(|s| s.stamp);
    }
    Ok(out)
}

/// Merge RealSense-style split IMU streams into one 6-axis stream.
///
/// OpenLORIS (and RealSense devices generally) publish gyro and accel as *separate*
/// `sensor_msgs/Imu` topics at different rates. The gyro is the denser stream, so its
/// samples are the time base and pass through verbatim; accel is linearly interpolated
/// at each gyro stamp. Gyro samples outside the accel time span are dropped (no
/// extrapolation). Both inputs must be time-sorted (bag readers return them sorted).
/// Mirrors `harness.datasets.merge_split_imu` on the Python side.
pub fn merge_split_imu(gyro: &[ImuSample], accel: &[ImuSample]) -> Vec<ImuSample> {
    let (first, last) = match (accel.first(), accel.last()) {
        (Some(f), Some(l)) => (f.stamp, l.stamp),
        _ => return Vec::new(),
    };
    let mut out = Vec::with_capacity(gyro.len());
    let mut i = 0;
    for g in gyro {
        if g.stamp < first || g.stamp > last {
            continue;
        }
        while i + 1 < accel.len() && accel[i + 1].stamp < g.stamp {
            i += 1;
        }
        let a0 = &accel[i];
        let a1 = &accel[(i + 1).min(accel.len() - 1)];
        let dt = (a1.stamp - a0.stamp).as_seconds();
        let w = if dt == 0.0 {
            0.0
        } else {
            (g.stamp - a0.stamp).as_seconds() / dt
        };
        out.push(ImuSample::new(
            g.stamp,
            g.gyro,
            a0.accel + (a1.accel - a0.accel) * w,
        ));
    }
    out
}

/// Read the rigid sensor extrinsics a bag carries on `/tf_static` (ADR 0009: the
/// recorded counterpart of the URDF's fixed joints). Every `tf2_msgs/TFMessage` on the
/// topic is decoded; duplicate parent→child pairs keep the first occurrence.
pub fn read_static_transforms<P: AsRef<Path>>(path: P) -> Result<Vec<StaticTransform>, BagError> {
    let mut bag = BagFile::open(path)?;
    let wanted: BTreeSet<u32> = bag
        .connections()
        .iter()
        .filter(|c| c.topic == "/tf_static" && c.message_type == "tf2_msgs/TFMessage")
        .map(|c| c.id)
        .collect();
    if wanted.is_empty() {
        return Err(BagError::NoTopic("tf2_msgs/TFMessage (/tf_static)"));
    }
    let mut out: Vec<StaticTransform> = Vec::new();
    bag.for_each_message(&wanted, |_, data| {
        for tf in parse_tf_message(data)? {
            if !out
                .iter()
                .any(|t| t.parent == tf.parent && t.child == tf.child)
            {
                out.push(tf);
            }
        }
        Ok(())
    })?;
    Ok(out)
}

/// Read the IMU stream from a ROS1 bag, returning time-sorted samples.
///
/// Topic selection as in [`resolve_topic`].
pub fn read_imu_from_bag<P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
) -> Result<Vec<ImuSample>, BagError> {
    let chosen = resolve_topic(&connection_map(&BagFile::open(&path)?), topic, IMU_MSG_TYPE)?;
    let mut streams = read_streams_from_bag(path, &[&chosen], &[])?;
    Ok(streams.imu.remove(&chosen).unwrap_or_default())
}

/// Read a planar laser-scan stream from a ROS1 bag, returning time-sorted scans.
///
/// Topic selection as in [`resolve_topic`].
pub fn read_scans_from_bag<P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
) -> Result<Vec<LaserScan2D>, BagError> {
    let chosen = resolve_topic(
        &connection_map(&BagFile::open(&path)?),
        topic,
        SCAN_MSG_TYPE,
    )?;
    let mut streams = read_streams_from_bag(path, &[], &[&chosen])?;
    Ok(streams.scans.remove(&chosen).unwrap_or_default())
}

#[cfg(test)]
mod tests {
    use super::*;
    use slam_types::{Stamp, Vec3};

    fn imu(t: f64, g: f64, a: f64) -> ImuSample {
        ImuSample::new(
            Stamp::from_seconds(t),
            Vec3::new(g, g, g),
            Vec3::new(a, a, a),
        )
    }

    #[test]
    fn merge_interpolates_accel_at_gyro_stamps() {
        let gyro = vec![imu(1.0, 0.1, 0.0), imu(1.5, 0.2, 0.0), imu(2.0, 0.3, 0.0)];
        let accel = vec![imu(1.0, 0.0, 10.0), imu(2.0, 0.0, 20.0)];
        let merged = merge_split_imu(&gyro, &accel);
        assert_eq!(merged.len(), 3);
        // Gyro columns (stamp included) pass through verbatim.
        assert_eq!(merged[1].stamp, gyro[1].stamp);
        assert_eq!(merged[1].gyro, gyro[1].gyro);
        // Accel is lerped at the gyro stamp: midway between 10 and 20.
        assert!((merged[1].accel.x - 15.0).abs() < 1e-12);
        assert!((merged[2].accel.x - 20.0).abs() < 1e-12);
    }

    #[test]
    fn merge_drops_gyro_outside_accel_span() {
        let gyro = vec![imu(0.5, 0.1, 0.0), imu(1.5, 0.2, 0.0), imu(2.5, 0.3, 0.0)];
        let accel = vec![imu(1.0, 0.0, 10.0), imu(2.0, 0.0, 20.0)];
        let merged = merge_split_imu(&gyro, &accel);
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].stamp, Stamp::from_seconds(1.5));
    }

    #[test]
    fn merge_handles_empty_streams() {
        assert!(merge_split_imu(&[], &[imu(1.0, 0.0, 1.0)]).is_empty());
        assert!(merge_split_imu(&[imu(1.0, 0.1, 0.0)], &[]).is_empty());
    }

    #[test]
    fn merge_single_accel_sample_is_constant() {
        let gyro = vec![imu(1.0, 0.1, 0.0)];
        let accel = vec![imu(1.0, 0.0, 10.0)];
        let merged = merge_split_imu(&gyro, &accel);
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].accel.x, 10.0);
    }
}

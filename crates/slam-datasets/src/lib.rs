//! Dataset ingestion for the SLAM engine.
//!
//! Reads sensor streams from recorded logs into engine types. The first source is the
//! **ROS1 bag** (used by OpenLORIS-Scene), read via the `rosbag` crate with no ROS install
//! required. Extracted so far: IMU (`sensor_msgs/Imu`) and planar laser scans
//! (`sensor_msgs/LaserScan`); RGB-D extraction lands with the visual front-end.
//!
//! Design: the engine consumes the simple [`slam_types`] formats, so this crate's job is
//! purely *log format → engine types*. The `slam-bag2imu` / `slam-bag2scan` binaries
//! expose [`read_imu_from_bag`] / [`read_scans_from_bag`] for the evaluation harness.

#![forbid(unsafe_code)]

mod imu_msg;
mod scan_msg;

use std::collections::BTreeMap;
use std::path::Path;

use rosbag::{ChunkRecord, IndexRecord, MessageRecord, RosBag};
use slam_types::{ImuSample, LaserScan2D};

pub use imu_msg::parse_imu;
pub use scan_msg::parse_scan;

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
    #[error("parsing bag: {0}")]
    Bag(#[from] rosbag::Error),
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

fn open(path: &Path) -> Result<RosBag, BagError> {
    RosBag::new(path).map_err(|source| BagError::Open {
        path: path.display().to_string(),
        source,
    })
}

/// Map connection id → (topic, message type), from the bag's index section.
fn connection_map(bag: &RosBag) -> Result<BTreeMap<u32, (String, String)>, BagError> {
    let mut map = BTreeMap::new();
    for rec in bag.index_records() {
        if let IndexRecord::Connection(conn) = rec? {
            map.insert(conn.id, (conn.topic.to_string(), conn.tp.to_string()));
        }
    }
    Ok(map)
}

/// List the topics (and message types) present in a bag.
pub fn list_topics<P: AsRef<Path>>(path: P) -> Result<Vec<TopicInfo>, BagError> {
    let bag = open(path.as_ref())?;
    let mut topics: BTreeMap<String, String> = BTreeMap::new();
    for (_, (topic, tp)) in connection_map(&bag)? {
        topics.insert(topic, tp);
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
/// Decompression dominates extraction cost (OpenLORIS bags are bz2 inside), so the
/// number of passes *is* the cost: extracting gyro + accel + scan together costs one
/// decompression instead of three. Every requested topic must exist with the right
/// message type.
pub fn read_streams_from_bag<P: AsRef<Path>>(
    path: P,
    imu_topics: &[&str],
    scan_topics: &[&str],
) -> Result<BagStreams, BagError> {
    let bag = open(path.as_ref())?;
    let conns = connection_map(&bag)?;

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

    for chunk_rec in bag.chunk_records() {
        if let ChunkRecord::Chunk(chunk) = chunk_rec? {
            for msg in chunk.messages() {
                if let MessageRecord::MessageData(data) = msg? {
                    match targets.get(&data.conn_id) {
                        Some(Target::Imu(topic)) => out
                            .imu
                            .get_mut(topic)
                            .expect("stream pre-created")
                            .push(parse_imu(data.data)?),
                        Some(Target::Scan(topic)) => out
                            .scans
                            .get_mut(topic)
                            .expect("stream pre-created")
                            .push(parse_scan(data.data)?),
                        None => {}
                    }
                }
            }
        }
    }

    for samples in out.imu.values_mut() {
        samples.sort_by_key(|s| s.stamp);
    }
    for scans in out.scans.values_mut() {
        scans.sort_by_key(|s| s.stamp);
    }
    Ok(out)
}

/// Read the IMU stream from a ROS1 bag, returning time-sorted samples.
///
/// Topic selection as in [`resolve_topic`].
pub fn read_imu_from_bag<P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
) -> Result<Vec<ImuSample>, BagError> {
    let bag = open(path.as_ref())?;
    let chosen = resolve_topic(&connection_map(&bag)?, topic, IMU_MSG_TYPE)?;
    drop(bag);
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
    let bag = open(path.as_ref())?;
    let chosen = resolve_topic(&connection_map(&bag)?, topic, SCAN_MSG_TYPE)?;
    drop(bag);
    let mut streams = read_streams_from_bag(path, &[], &[&chosen])?;
    Ok(streams.scans.remove(&chosen).unwrap_or_default())
}

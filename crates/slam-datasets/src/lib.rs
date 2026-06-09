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

/// Read every message of one topic from a ROS1 bag through `parse`.
///
/// If `topic` is `None`, the unique topic of `msg_type` is auto-selected; an error is
/// returned if there are zero or several. If `topic` is given, it must exist and carry
/// `msg_type` messages.
fn read_topic_from_bag<T, P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
    msg_type: &'static str,
    parse: impl Fn(&[u8]) -> Result<T, BagError>,
) -> Result<Vec<T>, BagError> {
    let bag = open(path.as_ref())?;
    let conns = connection_map(&bag)?;

    // Topics that carry messages of the requested type.
    let candidates: BTreeMap<&str, ()> = conns
        .values()
        .filter(|(_, tp)| tp == msg_type)
        .map(|(topic, _)| (topic.as_str(), ()))
        .collect();

    let chosen: String = match topic {
        Some(requested) => {
            if candidates.contains_key(requested) {
                requested.to_string()
            } else {
                return Err(BagError::TopicNotFound(requested.to_string(), msg_type));
            }
        }
        None => match candidates.len() {
            0 => return Err(BagError::NoTopic(msg_type)),
            1 => candidates.keys().next().unwrap().to_string(),
            _ => {
                let names: Vec<&str> = candidates.keys().copied().collect();
                return Err(BagError::AmbiguousTopic(msg_type, names.join(", ")));
            }
        },
    };

    // Connection ids mapping to the chosen topic (a topic may have several connections).
    let target_ids: std::collections::BTreeSet<u32> = conns
        .iter()
        .filter(|(_, (t, _))| *t == chosen)
        .map(|(id, _)| *id)
        .collect();

    let mut samples = Vec::new();
    for chunk_rec in bag.chunk_records() {
        if let ChunkRecord::Chunk(chunk) = chunk_rec? {
            for msg in chunk.messages() {
                if let MessageRecord::MessageData(data) = msg? {
                    if target_ids.contains(&data.conn_id) {
                        samples.push(parse(data.data)?);
                    }
                }
            }
        }
    }
    Ok(samples)
}

/// Read the IMU stream from a ROS1 bag, returning time-sorted samples.
///
/// Topic selection as in [`read_topic_from_bag`].
pub fn read_imu_from_bag<P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
) -> Result<Vec<ImuSample>, BagError> {
    let mut samples = read_topic_from_bag(path, topic, IMU_MSG_TYPE, parse_imu)?;
    samples.sort_by_key(|s| s.stamp);
    Ok(samples)
}

/// Read a planar laser-scan stream from a ROS1 bag, returning time-sorted scans.
///
/// Topic selection as in [`read_topic_from_bag`].
pub fn read_scans_from_bag<P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
) -> Result<Vec<LaserScan2D>, BagError> {
    let mut scans = read_topic_from_bag(path, topic, SCAN_MSG_TYPE, parse_scan)?;
    scans.sort_by_key(|s| s.stamp);
    Ok(scans)
}

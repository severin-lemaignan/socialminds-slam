//! Dataset ingestion for the SLAM engine.
//!
//! Reads sensor streams from recorded logs into engine types. The first source is the
//! **ROS1 bag** (used by OpenLORIS-Scene), read via the `rosbag` crate with no ROS install
//! required. For now we extract the IMU stream (`sensor_msgs/Imu`); lidar/RGB-D extraction
//! lands with their front-ends (M3+).
//!
//! Design: the engine consumes the simple [`slam_types`] IMU/trajectory formats, so this
//! crate's job is purely *log format → engine types*. The `slam-bag2imu` binary exposes
//! [`read_imu_from_bag`] on the command line for the evaluation harness.

#![forbid(unsafe_code)]

mod imu_msg;

use std::collections::BTreeMap;
use std::path::Path;

use rosbag::{ChunkRecord, IndexRecord, MessageRecord, RosBag};
use slam_types::ImuSample;

pub use imu_msg::parse_imu;

/// ROS message type string for IMU data.
pub const IMU_MSG_TYPE: &str = "sensor_msgs/Imu";

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
    #[error("no {IMU_MSG_TYPE} topic found in bag")]
    NoImuTopic,
    #[error("multiple {IMU_MSG_TYPE} topics present ({0}); pass one explicitly")]
    AmbiguousImuTopic(String),
    #[error("topic {0:?} not found in bag, or it is not {IMU_MSG_TYPE}")]
    TopicNotFound(String),
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

/// Read the IMU stream from a ROS1 bag, returning time-sorted samples.
///
/// If `topic` is `None`, the unique `sensor_msgs/Imu` topic is auto-selected; an error is
/// returned if there are zero or several. If `topic` is given, it must exist and be an IMU
/// topic.
pub fn read_imu_from_bag<P: AsRef<Path>>(
    path: P,
    topic: Option<&str>,
) -> Result<Vec<ImuSample>, BagError> {
    let bag = open(path.as_ref())?;
    let conns = connection_map(&bag)?;

    // Topics that carry IMU messages.
    let imu_topics: BTreeMap<&str, ()> = conns
        .values()
        .filter(|(_, tp)| tp == IMU_MSG_TYPE)
        .map(|(topic, _)| (topic.as_str(), ()))
        .collect();

    let chosen: String = match topic {
        Some(requested) => {
            if imu_topics.contains_key(requested) {
                requested.to_string()
            } else {
                return Err(BagError::TopicNotFound(requested.to_string()));
            }
        }
        None => match imu_topics.len() {
            0 => return Err(BagError::NoImuTopic),
            1 => imu_topics.keys().next().unwrap().to_string(),
            _ => {
                let names: Vec<&str> = imu_topics.keys().copied().collect();
                return Err(BagError::AmbiguousImuTopic(names.join(", ")));
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
                        samples.push(parse_imu(data.data)?);
                    }
                }
            }
        }
    }

    samples.sort_by_key(|s| s.stamp);
    Ok(samples)
}

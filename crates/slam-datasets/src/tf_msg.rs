//! Decoding `tf2_msgs/TFMessage` (the `/tf_static` topic) from ROS1 wire bytes.
//!
//! `/tf_static` is where a ROS system publishes its rigid (calibrated) sensor
//! extrinsics — the recorded counterpart of the URDF's fixed joints (ADR 0009). Reading
//! it lets a bag carry its own rig: OpenLORIS bags, for instance, place the IMUs on the
//! camera bodies (`d400_imu`), not on `base_link`.
//!
//! Layout: `geometry_msgs/TransformStamped[]` — u32 count, then per element:
//!
//! ```text
//! std_msgs/Header header        # uint32 seq; uint32 stamp.sec; uint32 stamp.nsec;
//!                               # uint32 frame_id_len; frame_id bytes   (parent)
//! string child_frame_id         # u32 len + bytes
//! geometry_msgs/Transform       # translation (3 × f64), rotation quaternion (4 × f64, xyzw)
//! ```

use slam_types::{Pose, Rotation, Vec3};

use crate::BagError;

/// One rigid parent→child transform from `/tf_static`.
#[derive(Debug, Clone)]
pub struct StaticTransform {
    pub parent: String,
    pub child: String,
    /// `T_parent_child`.
    pub transform: Pose,
}

struct LeCursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> LeCursor<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], BagError> {
        let end = self
            .pos
            .checked_add(n)
            .ok_or(BagError::Format("tf message length overflow"))?;
        let slice = self
            .data
            .get(self.pos..end)
            .ok_or(BagError::Format("tf message truncated"))?;
        self.pos = end;
        Ok(slice)
    }

    fn u32(&mut self) -> Result<u32, BagError> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f64(&mut self) -> Result<f64, BagError> {
        let b = self.take(8)?;
        Ok(f64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }

    fn string(&mut self) -> Result<String, BagError> {
        let len = self.u32()? as usize;
        Ok(String::from_utf8_lossy(self.take(len)?).into_owned())
    }
}

/// Decode one `tf2_msgs/TFMessage` body into its transforms.
pub fn parse_tf_message(data: &[u8]) -> Result<Vec<StaticTransform>, BagError> {
    let mut c = LeCursor { data, pos: 0 };
    let count = c.u32()? as usize;
    if count > data.len() / 60 {
        // Each TransformStamped is ≥ 60 bytes on the wire; reject bogus counts.
        return Err(BagError::Format("tf transform count exceeds message size"));
    }
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        let _seq = c.u32()?;
        let _secs = c.u32()?;
        let _nsecs = c.u32()?;
        let parent = c.string()?;
        let child = c.string()?;
        let t = Vec3::new(c.f64()?, c.f64()?, c.f64()?);
        let (qx, qy, qz, qw) = (c.f64()?, c.f64()?, c.f64()?, c.f64()?);
        out.push(StaticTransform {
            parent,
            child,
            transform: Pose::new(Rotation::from_xyzw(qx, qy, qz, qw), t),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode(transforms: &[(&str, &str, [f64; 3], [f64; 4])]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(transforms.len() as u32).to_le_bytes());
        for (parent, child, t, q) in transforms {
            v.extend_from_slice(&1u32.to_le_bytes()); // seq
            v.extend_from_slice(&0u32.to_le_bytes()); // sec
            v.extend_from_slice(&0u32.to_le_bytes()); // nsec
            v.extend_from_slice(&(parent.len() as u32).to_le_bytes());
            v.extend_from_slice(parent.as_bytes());
            v.extend_from_slice(&(child.len() as u32).to_le_bytes());
            v.extend_from_slice(child.as_bytes());
            for x in t.iter().chain(q.iter()) {
                v.extend_from_slice(&x.to_le_bytes());
            }
        }
        v
    }

    #[test]
    fn decodes_a_two_transform_message() {
        let body = encode(&[
            ("base_link", "laser", [0.2, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0]),
            (
                "d400_color",
                "d400_imu",
                [0.015, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ),
        ]);
        let tfs = parse_tf_message(&body).unwrap();
        assert_eq!(tfs.len(), 2);
        assert_eq!(tfs[0].parent, "base_link");
        assert_eq!(tfs[0].child, "laser");
        assert!((tfs[0].transform.translation().z - 0.3).abs() < 1e-12);
        assert_eq!(tfs[1].child, "d400_imu");
    }

    #[test]
    fn rejects_truncated_message() {
        let body = encode(&[("a", "b", [0.0; 3], [0.0, 0.0, 0.0, 1.0])]);
        assert!(parse_tf_message(&body[..body.len() - 8]).is_err());
    }

    #[test]
    fn rejects_bogus_count() {
        let mut body = encode(&[]);
        body[0..4].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(parse_tf_message(&body).is_err());
    }
}

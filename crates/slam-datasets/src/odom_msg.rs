//! Decoding `nav_msgs/Odometry` from ROS1 wire bytes (the wheel-odometry motion
//! prior — ADR 0012). Layout: Header; string child_frame_id;
//! PoseWithCovariance (3×f64 position, 4×f64 quaternion xyzw, f64[36] covariance);
//! TwistWithCovariance (skipped).

use slam_types::{OdomSample, Pose, Rotation, Stamp, Vec3};

use crate::BagError;

struct LeCursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> LeCursor<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], BagError> {
        let end = self
            .pos
            .checked_add(n)
            .ok_or(BagError::Format("odometry message length overflow"))?;
        let slice = self
            .data
            .get(self.pos..end)
            .ok_or(BagError::Format("odometry message truncated"))?;
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

/// Decode one `nav_msgs/Odometry` body. Returns the sample plus the `child_frame_id`
/// (the frame the pose is *of* — normally the base link).
pub fn parse_odometry(data: &[u8]) -> Result<(OdomSample, String), BagError> {
    let mut c = LeCursor { data, pos: 0 };
    let _seq = c.u32()?;
    let secs = c.u32()?;
    let nsecs = c.u32()?;
    let _frame_id = c.string()?; // the odometry (parent) frame
    let child = c.string()?;
    let t = Vec3::new(c.f64()?, c.f64()?, c.f64()?);
    let (qx, qy, qz, qw) = (c.f64()?, c.f64()?, c.f64()?, c.f64()?);
    Ok((
        OdomSample {
            stamp: Stamp::from_nanos(secs as i64 * 1_000_000_000 + nsecs as i64),
            frame: slam_types::FrameId::BASE,
            pose: Pose::new(Rotation::from_xyzw(qx, qy, qz, qw), t),
        },
        child,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_well_formed_message() {
        let mut v = Vec::new();
        v.extend_from_slice(&1u32.to_le_bytes());
        v.extend_from_slice(&100u32.to_le_bytes());
        v.extend_from_slice(&0u32.to_le_bytes());
        for s in ["odom", "base_link"] {
            v.extend_from_slice(&(s.len() as u32).to_le_bytes());
            v.extend_from_slice(s.as_bytes());
        }
        for x in [1.5f64, -0.25, 0.0, 0.0, 0.0, 0.0, 1.0] {
            v.extend_from_slice(&x.to_le_bytes());
        }
        for _ in 0..36 {
            v.extend_from_slice(&0f64.to_le_bytes());
        }
        let (s, child) = parse_odometry(&v).unwrap();
        assert_eq!(child, "base_link");
        assert!((s.pose.translation().x - 1.5).abs() < 1e-12);
        assert!((s.pose.translation().y + 0.25).abs() < 1e-12);
    }

    #[test]
    fn rejects_truncated() {
        assert!(parse_odometry(&[0u8; 10]).is_err());
    }
}

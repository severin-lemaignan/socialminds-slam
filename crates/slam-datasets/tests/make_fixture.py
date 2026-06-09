#!/usr/bin/env python3
"""Generate the committed `mini.bag` fixture for slam-datasets' bag-reader tests.

A tiny, uncompressed ROS1 bag with three `sensor_msgs/Imu` messages on `/d400/imu`, two
`sensor_msgs/LaserScan` messages on `/scan`, plus a non-IMU topic (`/robot/info`), so the
Rust tests exercise auto-topic-selection and type-based filtering. Re-run only when the
fixture needs regenerating:

    pip install rosbags numpy
    python crates/slam-datasets/tests/make_fixture.py

The values here are asserted verbatim in `tests/read_bag.rs`.
"""
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

OUT = Path(__file__).parent / "fixtures" / "mini.bag"

# (sec, nanosec, gyro xyz, accel xyz) — asserted in the Rust test.
SAMPLES = [
    (1560000083, 920771360, (0.10, -0.20, 0.30), (8.10, -0.30, 4.50)),
    (1560000083, 925771360, (0.11, -0.21, 0.31), (8.11, -0.31, 4.51)),
    (1560000083, 930771360, (0.12, -0.22, 0.32), (8.12, -0.32, 4.52)),
]

# (sec, nanosec, ranges) — asserted in the Rust test; one invalid (inf) return.
SCANS = [
    (1560000083, 922000000, (1.0, float("inf"), 2.5)),
    (1560000083, 947000000, (1.1, 1.2, 2.6)),
]
SCAN_META = dict(
    angle_min=-1.5, angle_max=1.5, angle_increment=1.5, time_increment=0.0001,
    scan_time=0.025, range_min=0.1, range_max=25.0,
)


def main() -> None:
    ts = get_typestore(Stores.ROS1_NOETIC)
    Imu = ts.types["sensor_msgs/msg/Imu"]
    LaserScan = ts.types["sensor_msgs/msg/LaserScan"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    Quaternion = ts.types["geometry_msgs/msg/Quaternion"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    String = ts.types["std_msgs/msg/String"]

    OUT.parent.mkdir(parents=True, exist_ok=True)
    if OUT.exists():
        OUT.unlink()

    with Writer(OUT) as writer:
        imu_conn = writer.add_connection("/d400/imu", Imu.__msgtype__, typestore=ts)
        info_conn = writer.add_connection("/robot/info", String.__msgtype__, typestore=ts)

        for sec, nsec, gyro, accel in SAMPLES:
            t_ns = sec * 1_000_000_000 + nsec
            msg = Imu(
                header=Header(seq=0, stamp=Time(sec=sec, nanosec=nsec), frame_id="imu"),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=np.zeros(9, dtype=np.float64),
                angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                angular_velocity_covariance=np.zeros(9, dtype=np.float64),
                linear_acceleration=Vector3(x=accel[0], y=accel[1], z=accel[2]),
                linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
            )
            writer.write(imu_conn, t_ns, ts.serialize_ros1(msg, Imu.__msgtype__))

        scan_conn = writer.add_connection("/scan", LaserScan.__msgtype__, typestore=ts)
        for sec, nsec, ranges in SCANS:
            t_ns = sec * 1_000_000_000 + nsec
            msg = LaserScan(
                header=Header(seq=0, stamp=Time(sec=sec, nanosec=nsec), frame_id="laser"),
                ranges=np.array(ranges, dtype=np.float32),
                intensities=np.zeros(len(ranges), dtype=np.float32),
                **SCAN_META,
            )
            writer.write(scan_conn, t_ns, ts.serialize_ros1(msg, LaserScan.__msgtype__))

        info = String(data="openloris-mini")
        writer.write(info_conn, SAMPLES[0][0] * 1_000_000_000 + SAMPLES[0][1],
                     ts.serialize_ros1(info, String.__msgtype__))

    print(f"wrote {OUT} ({OUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()

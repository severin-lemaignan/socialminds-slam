#!/usr/bin/env bash
# Run RTAB-Map over an OpenLORIS-Scene ROS1 bag in Docker and export a TUM trajectory.
#
# OPERATOR STEP — NOT run in CI. Needs Docker and a downloaded OpenLORIS bag. RTAB-Map and
# the dataset's exact topic names mean this is a STARTING POINT you will adapt per sequence,
# not a turnkey script. See ./README.md.
#
# Usage:
#   ./run_rtabmap.sh <path/to/sequence.bag> <output.tum>
set -euo pipefail

BAG="${1:?usage: run_rtabmap.sh <bag> <output.tum>}"
OUT="${2:?usage: run_rtabmap.sh <bag> <output.tum>}"

# OpenLORIS D435i topics (verify with: slam-bag2imu --bag <bag> --list, or rosbag info).
# These are the typical RealSense topics; adjust to the sequence if they differ.
RGB_TOPIC="${RGB_TOPIC:-/d400/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/d400/aligned_depth_to_color/image_raw}"
CAMERA_INFO="${CAMERA_INFO:-/d400/color/camera_info}"

IMAGE="${RTABMAP_IMAGE:-introlab3it/rtabmap_ros:noetic-latest}"

echo "RTAB-Map reference run"
echo "  bag:    $BAG"
echo "  output: $OUT"
echo "  topics: rgb=$RGB_TOPIC depth=$DEPTH_TOPIC info=$CAMERA_INFO"
echo
cat <<'NOTE'
This template launches RTAB-Map against the bag, then exports poses to TUM. Inside the
container you typically:
  1) roscore &
  2) rosparam set use_sim_time true
  3) roslaunch rtabmap_launch rtabmap.launch \
        rgb_topic:=$RGB_TOPIC depth_topic:=$DEPTH_TOPIC camera_info_topic:=$CAMERA_INFO \
        rtabmap_args:="--delete_db_on_start" &
  4) rosbag play --clock "$BAG"
  5) rtabmap-export --poses --poses_format 11 ~/.ros/rtabmap.db   # 11 = TUM format
     (then copy the exported *.txt out as "$OUT")

Wire these into the `docker run` below once topic remaps are confirmed for your sequence.
NOTE

# Skeleton invocation (left non-executing on purpose until topics are confirmed):
#   docker run --rm -it -v "$(dirname "$BAG")":/data -v "$(dirname "$OUT")":/out "$IMAGE" \
#       bash -lc '<the steps above, writing /out/$(basename "$OUT")>'

echo "Edit this script to enable the docker run for your environment (see ./README.md)."
exit 0

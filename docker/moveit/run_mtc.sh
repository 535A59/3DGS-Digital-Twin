#!/bin/bash
# MoveIt 2 + MTC container of the digital twin (contract §6): builds the image (cached), then runs
# mtc_dynamic's launch/isaac.launch.py against the Isaac Sim container (same ROS_DOMAIN_ID, --network host).
#
#   ROS_DOMAIN_ID=64 docker/moveit/run_mtc.sh /abs/path/output/twin_res5/mtc_task.yaml
#
# Exit code 0 only if pick_place_demo logged "Execution complete: <n> objects".
# Env: MTC_IMAGE (twin-moveit), MTC_CONTAINER (twin_mtc), MTC_TIMEOUT (s, 1500).
# The task file's directory (the sim asset out_dir) is mounted read-only at its host path, so the absolute
# mesh paths in it are valid inside the container.
set -euo pipefail
: "${ROS_DOMAIN_ID:?set an explicit non-zero ROS_DOMAIN_ID (domain 0 is shared with other ROS graphs)}"
[ "$ROS_DOMAIN_ID" != 0 ] || { echo "[run_mtc] refusing ROS_DOMAIN_ID=0"; exit 1; }
TASK=$(realpath "${1:?usage: run_mtc.sh <mtc_task.yaml>}")
REPO=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=${MTC_IMAGE:-twin-moveit}

echo "[run_mtc] docker build $IMAGE"
docker build -q -t "$IMAGE" -f "$REPO/docker/moveit/Dockerfile" "$REPO/ros2_ws/mtc_controller_final/src"

LOG=$(mktemp)
trap 'rm -f "$LOG"' EXIT
docker run --rm --name "${MTC_CONTAINER:-twin_mtc}" --network host \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$(dirname "$TASK"):$(dirname "$TASK"):ro" "$IMAGE" \
  bash -c "source /ws/install/setup.bash && timeout ${MTC_TIMEOUT:-1500} ros2 launch mtc_dynamic isaac.launch.py task:=$TASK" \
  2>&1 | tee "$LOG" || true
grep -q "Execution complete:" "$LOG"

#!/bin/bash
# Isaac Sim 6.1 container for the 3DGS digital twin (contract §0, §4, §5).
#
#   ROS_DOMAIN_ID=62 simulation/isaac/run_isaac.sh [run_twin.py args ...]
#   ROS_DOMAIN_ID=62 SCRIPT=build_scene.py simulation/isaac/run_isaac.sh        # only author + save the stage
#
# Env: OUT_DIR (default <repo>/output/twin_res5), SCRIPT (run_twin.py), ISAAC_CONTAINER (twin_isaac),
#      ISAAC_IMAGE (nvcr.io/nvidia/isaac-sim:6.1.0), ISAAC_CACHE (~/.cache/isaac-sim-6.1-docker).
# The repo (read-only) and OUT_DIR (read-write) are mounted at their host paths, so the absolute paths
# in mtc_task.yaml and the Marker mesh resources are valid on both sides.
set -euo pipefail
: "${ROS_DOMAIN_ID:?set an explicit non-zero ROS_DOMAIN_ID (domain 0 is shared with other ROS graphs)}"
[ "$ROS_DOMAIN_ID" != 0 ] || { echo "[run_isaac] refusing ROS_DOMAIN_ID=0"; exit 1; }

REPO=$(cd "$(dirname "$0")/../.." && pwd)
OUT_DIR=$(realpath "${OUT_DIR:-$REPO/output/twin_res5}")
SCRIPT=${SCRIPT:-run_twin.py}
CACHE=${ISAAC_CACHE:-$HOME/.cache/isaac-sim-6.1-docker}
[ -f "$OUT_DIR/manifest.json" ] || { echo "[run_isaac] no $OUT_DIR/manifest.json — run src.sim_export.export first"; exit 1; }

# Container user is uid 1234: every writable mount must be world-writable.
mkdir -p "$CACHE"/{kitcache,kitdata,main,computecache,logs,config,data,pkg}
chmod 777 "$CACHE" "$CACHE"/* "$OUT_DIR"

exec docker run --rm --name "${ISAAC_CONTAINER:-twin_isaac}" --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$CACHE/kitcache:/isaac-sim/kit/cache:rw" -v "$CACHE/kitdata:/isaac-sim/kit/data:rw" \
  -v "$CACHE/main:/isaac-sim/.cache:rw" -v "$CACHE/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw" -v "$CACHE/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw" -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$REPO:$REPO:ro" -v "$OUT_DIR:$OUT_DIR:rw" -w "$REPO" \
  --entrypoint /isaac-sim/python.sh "${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.1.0}" \
  "$REPO/simulation/isaac/$SCRIPT" --out_dir "$OUT_DIR" "$@"

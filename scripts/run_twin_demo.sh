#!/bin/bash
# End-to-end digital-twin demo (contract §7): export -> Isaac Sim -> MoveIt/MTC -> recording + report.
#
#   ROS_DOMAIN_ID=64 PYTHON=<venv>/bin/python scripts/run_twin_demo.sh
#
# Env: ROS_DOMAIN_ID (required, non-zero), PYTHON (host python with the repo deps, default python3),
#      CONFIG (configs/twin_res5.yaml), OUT_DIR (from the config's out_dir), SIM_SECONDS (1800, Isaac upper bound),
#      MTC_TIMEOUT (1500 s); extra args are passed to run_twin.py (e.g. --render_hz 60).
# Outputs in OUT_DIR: isaac.log, mtc.log, final_state.json, recording/{unity_main,top}.mp4, report.json.
set -euo pipefail
: "${ROS_DOMAIN_ID:?set an explicit non-zero ROS_DOMAIN_ID}"
[ "$ROS_DOMAIN_ID" != 0 ] || { echo "[Demo] refusing ROS_DOMAIN_ID=0"; exit 1; }
REPO=$(cd "$(dirname "$0")/.." && pwd)
cd "$REPO"
PYTHON=${PYTHON:-python3}
CONFIG=${CONFIG:-configs/twin_res5.yaml}
export OUT_DIR=$(realpath -m "${OUT_DIR:-$("$PYTHON" -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['out_dir'])")}")
export MTC_TIMEOUT=${MTC_TIMEOUT:-1500}
T0=$SECONDS

echo "[Demo] 1/5 sim asset export -> $OUT_DIR"
"$PYTHON" -m src.sim_export.export --config "$CONFIG" --out_dir "$OUT_DIR"
rm -f "$OUT_DIR/final_state.json" "$OUT_DIR/report.json"

echo "[Demo] 2/5 Isaac Sim (log: $OUT_DIR/isaac.log)"
simulation/isaac/run_isaac.sh --seconds "${SIM_SECONDS:-1800}" "$@" > "$OUT_DIR/isaac.log" 2>&1 &
ISAAC=$!
trap 'touch "$OUT_DIR/STOP"; wait $ISAAC 2>/dev/null' EXIT
timeout 600 bash -c "until grep -q '\[Twin\] settled' '$OUT_DIR/isaac.log'; do kill -0 $ISAAC || exit 1; sleep 2; done" \
  || { echo "[Demo] Isaac did not come up, see $OUT_DIR/isaac.log"; exit 1; }
grep "\[Twin\] settled" "$OUT_DIR/isaac.log"
T_ISAAC=$SECONDS

echo "[Demo] 3/5 MoveIt + MTC (log: $OUT_DIR/mtc.log)"
# spawns the controllers in Isaac's controller_manager, starts move_group and pick_place_demo with mtc_task.yaml,
# returns when pick_place_demo exits (0 = "Execution complete")
MTC_RC=0
docker/moveit/run_mtc.sh "$OUT_DIR/mtc_task.yaml" > "$OUT_DIR/mtc.log" 2>&1 || MTC_RC=$?
grep -E "Object '.*' done|Task failed|Execution complete" "$OUT_DIR/mtc.log" || true
echo "[Demo] MTC exit code $MTC_RC after $((SECONDS - T_ISAAC)) s"
sleep 3   # let the last placed object settle in the recording

echo "[Demo] 4/5 stop Isaac (writes $OUT_DIR/final_state.json)"
touch "$OUT_DIR/STOP"
wait $ISAAC || true
trap - EXIT
grep "\[Twin\]" "$OUT_DIR/isaac.log" | tail -4
"$PYTHON" simulation/isaac/frames_to_mp4.py --out_dir "$OUT_DIR"

echo "[Demo] 5/5 report"
"$PYTHON" simulation/isaac/twin_report.py --out_dir "$OUT_DIR" --config "$CONFIG" --total_s $((SECONDS - T0))

"""
Isaac Sim runtime of the 3DGS digital twin (contract §4, §5, §7).

Builds the stage with build_scene.py and plays it headless with
  * /clock + in-process ros2_control: isaacsim.ros2.control ROS2ControlManager on
    /World/panda with franka_controllers.yaml (controllers are spawned from the
    MoveIt container); robot_state_publisher on the MoveIt side owns TF and
    /robot_description, so neither is published here;
  * once the scene has settled (--settle sim seconds), per object at 10 Hz (sim):
        /<obj>_position          visualization_msgs/Marker    body pose in world
        /<obj>_target_position   geometry_msgs/PoseStamped    target body pose (manifest)
  * /twin/status  std_msgs/String JSON at 1 Hz (sim): sim time, settled flag, body poses;
  * PNG frames of every manifest camera to out_dir/recording/<cam>/%05d.png at
    --record_fps (sim time).
Physics at --physics_hz, app updates (render + /clock) at --render_hz; sim time is
throttled to wall time (RTF <= 1, like the real robot). Rendering two 960x540 cameras
on every update is the main cost: ~RTF 1.0 at 30 Hz on the GB10, ~0.5 at 60 Hz.
Stops after --seconds of sim time, when out_dir/STOP appears, or on a message on
/twin/stop (std_msgs/Empty); then writes out_dir/final_state.json (final body poses,
drift, distance to target, highest lift of each body, and every contact pair with a
non-zero impulse that involved a robot link or an object, from PhysX contact reports),
out_dir/body_poses.json (10 Hz body poses) and prints a summary.

Usage (inside the Isaac Sim 6.1 container, see run_isaac.sh)
-----
    ROS_DOMAIN_ID=62 simulation/isaac/run_isaac.sh --seconds 600
    ROS_DOMAIN_ID=62 simulation/isaac/run_isaac.sh --no_ros --drop 0.05 --seconds 4   # physics/render check
"""

import argparse
import json
import os
import shutil
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

ap = argparse.ArgumentParser(description="Play the 3DGS digital twin in Isaac Sim (headless) with the ROS 2 interface")
ap.add_argument("--out_dir", required=True, help="sim asset folder with manifest.json (src.sim_export.export)")
ap.add_argument("--seconds", type=float, default=600.0, help="sim time to run")
ap.add_argument("--wall_timeout", type=float, default=3600.0, help="hard stop after this many wall seconds")
ap.add_argument("--settle", type=float, default=1.5, help="sim seconds before object poses are published")
ap.add_argument("--physics_hz", type=int, default=120)
ap.add_argument("--render_hz", type=int, default=30, help="app updates (render, /clock) per sim second")
ap.add_argument("--width", type=int, default=960)
ap.add_argument("--height", type=int, default=540)
ap.add_argument("--record_fps", type=float, default=10.0, help="frames per sim second per camera, 0 = off")
ap.add_argument("--faster_than_realtime", action="store_true", help="do not throttle sim time to wall time")
ap.add_argument("--drop", type=float, default=0.0, help="start every object this much higher (physics test)")
ap.add_argument("--no_ros", action="store_true", help="no ROS graph / publishers (physics + render only)")
args, _ = ap.parse_known_args()
os.umask(0)  # files written as uid 1234 stay editable by the host user

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": args.width, "height": args.height})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.usd
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Usd, UsdGeom

import build_scene

for ext in ["omni.kit.converter.gsplat"] + ([] if args.no_ros else
                                            ["isaacsim.ros2.core", "isaacsim.ros2.bridge", "isaacsim.ros2.control"]):
    app_utils.enable_extension(ext)
app.update()

manifest = build_scene.load_manifest(args.out_dir)
names = [o["name"] for o in manifest["objects"]]
stage_utils.create_new_stage()
stage = omni.usd.get_context().get_stage()
build_scene.build(stage, manifest, args.out_dir, build_scene.franka_url(app, manifest), drop=args.drop,
                  aspect=args.width / args.height)
app.update()


def ros_graph(robot):
    """OnPlaybackTick -> /clock + ROS2ControlManager (contract §4 /World/ActionGraph)."""
    import isaacsim.ros2.control.urdf_synth as urdf_synth
    import omni.graph.core as og
    import usdrt.Sdf

    # ponytail: isaacsim.ros2.control 0.1.6 urdf_synth._prune_to_control_tree treats panda_hand (parent of the
    # finger joint, reached via the fixed panda_hand_joint) as a 2nd root -> "Two root links found" and the
    # ControllerManager fails to init. Counting fixed joints as tree edges keeps one root. Drop once fixed upstream.
    prune = urdf_synth._prune_to_control_tree
    urdf_synth._prune_to_control_tree = lambda root, names: prune(
        root, names + [j.get("name") for j in root.findall("joint") if j.get("type") == "fixed"])

    k = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        {
            k.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("CM", "isaacsim.ros2.control.ROS2ControlManager"),
            ],
            k.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("Tick.outputs:tick", "CM.inputs:execIn"),
            ],
            k.SET_VALUES: [
                ("CM.inputs:targetPrim", [usdrt.Sdf.Path(robot)]),
                ("CM.inputs:controllerConfig",
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), "franka_controllers.yaml")),
                ("CM.inputs:namespace", ""),
                ("CM.inputs:publishRobotDescription", False),
            ],
        },
    )


def body_poses():
    out = {}
    for n in names:
        m = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Objects/{n}")).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        q = m.ExtractRotationQuat()
        out[n] = {"p": list(m.ExtractTranslation()), "q": [q.GetReal(), *q.GetImaginary()]}
    return out


class RosIO:
    """rclpy (Isaac's bundled Jazzy build) publishers of contract §5 and the /twin/stop subscriber."""

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Empty, String
        from visualization_msgs.msg import Marker
        self.rclpy, self.PoseStamped, self.String, self.Marker = rclpy, PoseStamped, String, Marker
        rclpy.init()
        self.node = rclpy.create_node("twin_isaac")
        self.pose_pub = {n: self.node.create_publisher(Marker, f"/{n}_position", 10) for n in names}
        self.target_pub = {n: self.node.create_publisher(PoseStamped, f"/{n}_target_position", 10) for n in names}
        self.status_pub = self.node.create_publisher(String, "/twin/status", 10)
        self.stop = False
        self.node.create_subscription(Empty, "/twin/stop", lambda _: setattr(self, "stop", True), 10)

    @staticmethod
    def _fill(msg_pose, pose):
        msg_pose.position.x, msg_pose.position.y, msg_pose.position.z = map(float, pose["p"])
        msg_pose.orientation.w, msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z = \
            map(float, pose["q"])

    def _header(self, h, t):
        h.frame_id = "world"
        h.stamp.sec, h.stamp.nanosec = int(t), int((t - int(t)) * 1e9)

    def publish_objects(self, t, poses):
        for o in manifest["objects"]:
            n = o["name"]
            m = self.Marker()
            self._header(m.header, t)
            m.ns, m.id, m.type, m.action = n, 0, self.Marker.MESH_RESOURCE, self.Marker.ADD
            m.mesh_resource = "file://" + os.path.abspath(os.path.join(args.out_dir, o["collision"]))
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.6, 0.2, 1.0
            self._fill(m.pose, poses[n])
            self.pose_pub[n].publish(m)
            ps = self.PoseStamped()
            self._header(ps.header, t)
            self._fill(ps.pose, o["target_pose_world"])
            self.target_pub[n].publish(ps)

    def publish_status(self, status):
        self.status_pub.publish(self.String(data=json.dumps(status)))

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


class Recorder:
    """LdrColor annotator per manifest camera; PNGs written by a thread pool so the sim loop keeps going."""

    def __init__(self):
        import omni.replicator.core as rep
        self.annot, self.n, self.pool = {}, 0, ThreadPoolExecutor(max_workers=2)
        for cam in build_scene.cameras(manifest):
            rp = rep.create.render_product(f"/World/Camera_{cam}", (args.width, args.height))
            a = rep.AnnotatorRegistry.get_annotator("LdrColor")
            a.attach(rp)
            self.annot[cam] = a
            d = os.path.join(args.out_dir, "recording", cam)
            shutil.rmtree(d, ignore_errors=True)  # no stale frames from a longer earlier run
            os.makedirs(d)

    def capture(self):
        from PIL import Image
        frames = {cam: np.asarray(a.get_data()) for cam, a in self.annot.items()}
        if any(f.ndim != 3 for f in frames.values()):  # renderer not ready yet
            return
        for cam, f in frames.items():
            path = os.path.join(args.out_dir, "recording", cam, f"{self.n:05d}.png")
            self.pool.submit(lambda f=f, path=path: Image.fromarray(f[..., :3].copy()).save(path, compress_level=1))
        self.n += 1

    def close(self):
        self.pool.shutdown(wait=True)


class Contacts:
    """PhysX contact reports (every physics step) of the robot links and the objects, summarised per
    body pair: first/last sim time, steps in contact, number of separate touches, peak impulse."""

    def __init__(self):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysxSchema, PhysicsSchemaTools, UsdPhysics
        self.to_path = PhysicsSchemaTools.intToSdfPath
        self.pairs = {}
        for root in [manifest["robot"]["prim"]] + [f"/World/Objects/{n}" for n in names]:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        self.sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_report)

    def _on_report(self, headers, data):
        from omni.physx.bindings._physx import ContactEventType
        t = SimulationManager.get_simulation_time()
        for h in headers:
            imp = max((float(np.linalg.norm(data[i].impulse))
                       for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data)), default=0.0)
            if h.type == ContactEventType.CONTACT_LOST or imp <= 0.0:  # 0 = shapes only within contact offset
                continue
            key = " | ".join(sorted([str(self.to_path(h.actor0)), str(self.to_path(h.actor1))]))
            c = self.pairs.setdefault(key, {"first_t": t, "last_t": -1.0, "steps": 0, "touches": 0, "max_impulse": 0.0})
            c["touches"] += t - c["last_t"] > 0.05  # a new touch after a gap without force
            c["last_t"], c["steps"], c["max_impulse"] = t, c["steps"] + 1, max(c["max_impulse"], imp)


if not args.no_ros:
    ros_graph(manifest["robot"]["prim"])
ros = None if args.no_ros else RosIO()
rec = Recorder() if args.record_fps > 0 else None
contacts = Contacts()
SimulationManager.setup_simulation(dt=1.0 / args.physics_hz, device="cpu")
RenderingManager.set_dt(1.0 / args.render_hz)  # every update renders both cameras: the main cost of a step

from omni.kit.viewport.utility import get_active_viewport

if get_active_viewport():  # headless: the default viewport only costs GPU time
    get_active_viewport().updates_enabled = False
for _ in range(60):  # let geometry streaming load the splats before the first recorded frame
    app.update()

stop_file = os.path.join(args.out_dir, "STOP")
if os.path.exists(stop_file):
    os.remove(stop_file)
initial = body_poses()
max_z = {n: initial[n]["p"][2] for n in names}
track = {"t": [], **{n: [] for n in names}}  # 10 Hz body poses [x, y, z, qw, qx, qy, qz] -> body_poses.json
app_utils.play()
print(f"[Twin] playing: {len(names)} objects, ROS {'off' if args.no_ros else os.environ.get('ROS_DOMAIN_ID')}, "
      f"record {args.record_fps} fps x {len(rec.annot) if rec else 0} cams", flush=True)

t0, t, steps = time.time(), 0.0, 0
next_pub, next_status, next_rec, settled_logged = args.settle, 0.0, 0.0, False
while app.is_running():
    app.update()
    steps += 1
    t = SimulationManager.get_simulation_time()
    if ros:
        ros.spin()
    if t >= next_status or t >= next_pub:
        poses = body_poses()
        max_z = {n: max(max_z[n], poses[n]["p"][2]) for n in names}
    if t >= next_pub:
        if ros:
            ros.publish_objects(t, poses)
        track["t"].append(round(t, 4))
        for n in names:
            track[n].append([round(v, 5) for v in poses[n]["p"] + poses[n]["q"]])
        next_pub += 0.1
    if t >= next_status:
        settled = t >= args.settle
        if ros:
            ros.publish_status({"t": round(t, 4), "settled": settled, "objects": poses})
        if settled and not settled_logged:
            print(f"[Twin] settled at t={t:.2f}s: " + ", ".join(
                f"{n} z={poses[n]['p'][2]:.4f}" for n in names), flush=True)
            settled_logged = True
        if os.path.exists(stop_file) or (ros and ros.stop):
            print("[Twin] stop requested", flush=True)
            break
        next_status += 1.0
    if rec and t >= next_rec:
        rec.capture()
        next_rec += 1.0 / args.record_fps
    if t >= args.seconds or time.time() - t0 > args.wall_timeout:
        break
    if not args.faster_than_realtime and t > time.time() - t0:  # behave like the real robot: RTF <= 1
        time.sleep(t - (time.time() - t0))

wall = time.time() - t0
final = body_poses()
app_utils.stop()
state = {"sim_time": t, "wall_time": wall, "steps": steps, "drop": args.drop, "objects": {},
         "contacts": contacts.pairs}
for o in manifest["objects"]:
    n, p = o["name"], np.asarray(final[o["name"]]["p"])
    p_init = np.asarray(o["body_pose_world"]["p"])
    state["objects"][n] = dict(final[n], initial_p=initial[n]["p"], target_p=o["target_pose_world"]["p"],
                               drift_m=float(np.linalg.norm(p - p_init)),
                               max_lift_m=float(max_z[n] - initial[n]["p"][2]),
                               target_err_m=float(np.linalg.norm(p - o["target_pose_world"]["p"])))
with open(os.path.join(args.out_dir, "final_state.json"), "w") as f:
    json.dump(state, f, indent=2)
with open(os.path.join(args.out_dir, "body_poses.json"), "w") as f:
    json.dump(track, f)
print(f"[Twin] {t:.2f}s sim in {wall:.1f}s wall (RTF {t / max(wall, 1e-9):.2f}, {steps} updates, "
      f"{rec.n if rec else 0} frames/cam)", flush=True)
for n, s in state["objects"].items():
    print(f"[Twin] {n:7s} p={np.round(s['p'], 4).tolist()}  drift from manifest pose {s['drift_m'] * 1000:6.1f} mm  "
          f"to target {s['target_err_m'] * 1000:6.1f} mm", flush=True)
if rec:
    rec.close()
if ros:
    ros.close()
app.close()

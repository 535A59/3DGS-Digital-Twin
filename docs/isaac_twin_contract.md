# Isaac Sim digital twin — design & interface contract (v1)

Replaces the Unity runtime of the paper with Isaac Sim 6.1.0 (aarch64 DGX Spark). The 3DGS reconstruction is the
visual; pipeline meshes are invisible PhysX proxies (rigid bodies with gravity + collision; table/cardboard box
static); each object's splats move with its rigid body; a Franka Panda is driven by the repo's MTC controller
(`ros2_ws/mtc_controller_final`, package `mtc_dynamic`) through MoveIt 2 — full closed-loop pick & place.

Components: export (`src/sim_export`), Isaac scene (`simulation/isaac`), MTC controller
(`ros2_ws/mtc_controller_final`, `docker/moveit`), orchestration (`scripts/run_twin_demo.sh`). Code comments
cite this file as "contract §N".

## 0. Runtime rules
- `old_code-data/3dgs_resource` (the RES5 reconstruction and the paper's Unity project) is an input, read only
  and gitignored; outputs go to `output/` (gitignored).
- Isaac Sim runs in the `nvcr.io/nvidia/isaac-sim:6.1.0` container as uid 1234: every mounted writable dir must be
  `chmod 777`. Mount persistent kit caches (`~/.cache/isaac-sim-6.1-docker/{kitcache,kitdata,main,computecache,
  logs,config,data,pkg}`), otherwise the first frame takes ~200 s. `simulation/isaac/run_isaac.sh` does both.
- ROS: both containers `--network host`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (SHM fails across containers),
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and an explicit non-zero `ROS_DOMAIN_ID`; the scripts refuse domain 0.
- One Isaac container at a time on a shared GPU, always `--rm` and bounded by a timeout.

## 1. Scene / dataset (decided)
The original Unity demo scene (= the paper's Fig. 2/4 scene), "RES5" set:
| role | name | source PLY (3DGS, source frame) |
|---|---|---|
| object (paper "blue box", actually SPAM can) | `box` | `old_code-data/3dgs_resource/mesh_source/object1-copy.ply` (7,887) |
| object | `hammer` | `.../mesh_source/object2-copy.ply` (4,659) |
| object (2.5 cm yellow cube) | `cube` | `.../mesh_source/object3-copy.ply` (1,346) |
| static background (table, cardboard box, target frame) | `background` | decoded from Unity asset `.../GSreal2sim2real/Assets/GaussianAssets/background-copy-cl.*` with `tools/unity_gs_to_ply.py` (rotation repair on). No source PLY exists. |
| robot-base splats (calibration check only, NOT rendered) | `arm` | `.../3dgs/RES5/arm.ply` |
Task (paper §4.1.1): (1) box → on top of the cardboard box, (2) cube → on top of the placed box, (3) hammer → into
the target frame. Unity target transforms (target_box/target_hammer; target_cube == target_box in Unity) are known
in the source frame via the Unity calibration.

## 2. Frames & calibration
- **World frame W = robot base `panda_link0`** (MoveIt `world`, identity static TF world→panda_link0), Z-up, metres.
  Isaac stage: `metersPerUnit=1`, `upAxis=Z`; Franka placed at the identity pose at `/World/panda`.
- `T_W_src` (4×4 similarity, det(R)=+1): source-PLY frame → W. Built from the Unity calibration (rotation R_u, base
  origin in source frame p0) BUT with a corrected metric scale `s` (Unity's 2.27 oversizes objects: the 2.5 cm cube
  would be ~4 cm and the SPAM can ungraspable) and a ≤2° tilt correction so the RANSAC table plane is exactly
  horizontal. Anchor for `s`: the paper's 2.5 cm cube edge; cross-check with FR3/Panda link0 footprint vs `arm.ply`
  and with the SPAM can dimensions. Robot base stays where Unity put it (p0 ↦ W origin). Document all evidence.
- Splat PLYs are NEVER modified (SH must not be rotated); transforms live on USD prims.

## 3. Sim asset export — `out_dir/` (default `output/twin_res5/`, gitignored)
```
manifest.json
splats/{box,hammer,cube,background}.ply        # 3DGS PLYs in SOURCE frame (copies/decoded), untouched attrs
collision/{box,hammer,cube}.obj                # watertight-ish, metres, vertices in the BODY frame (see below)
collision/env_{table,cardboard}.obj            # static obstacles, metres, vertices in WORLD frame
mtc_task.yaml                                  # ROS params for mtc_dynamic (see §6), generated
```
`manifest.json` (all poses: position xyz metres + quaternion **wxyz**; matrices row-major 4×4):
```json
{
  "version": 1, "units": "m", "up_axis": "Z", "world_frame": "panda_link0",
  "T_world_from_source": [[...4x4...]], "scale_m_per_unit": s,
  "table": {"height": z_table, "normal_src": [..], "d_src": d},
  "background": {"splats": "splats/background.ply"},
  "static_colliders": [{"name": "table", "mesh": "collision/env_table.obj", "approximation": "convexHull"},
                       {"name": "cardboard", "mesh": "collision/env_cardboard.obj", "approximation": "convexHull"}],
  "objects": [
    {"name": "box", "splats": "splats/box.ply", "collision": "collision/box.obj",
     "approximation": "convexHull" | "convexDecomposition", "max_convex_hulls": 16,
     "mass_kg": 0.2, "body_pose_world": {"p": [x,y,z], "q": [w,x,y,z]},
     "T_body_from_source": [[...4x4 similarity incl. scale s...]],
     "size_m": [dx,dy,dz], "target_pose_world": {"p": [...], "q": [...]}}
  ],
  "camera": {"name": "unity_main", "pose_world": {"p": [...], "q": [...]}, "vertical_fov_deg": 39.1,
             "convention": "USD camera (-Z forward, +Y up)"},
  "robot": {"usd": "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", "prim": "/World/panda",
            "base_pose_world": {"p": [0,0,0], "q": [1,0,0,0]}}
}
```
- Body frame: origin = object's collision-mesh centroid in W at rest (lifted so the mesh's lowest point sits 1 mm
  above its support surface), axes = world-aligned at export. So `T_body_from_source = inv(T_W_body) @ T_W_src`,
  collision OBJ vertices = `T_body_from_source` applied to the source-frame mesh. Splats under the body get the
  xformOp `T_body_from_source` (one matrix op incl. uniform scale). Both line up by construction.
- Collision meshes are generated by the repo pipeline (`src/geometry_refinement`) from the source PLYs with
  `center_the_cloud=False`, `transform_coords=False` (the latter is a MIRROR, never use for Isaac), then
  transformed. Approximation: convexHull if hull/mesh volume ≤ 1.25 else convexDecomposition.
- Masses: box(SPAM can) 0.15 kg, hammer 0.12 kg, cube 0.01 kg (configurable).

## 4. Isaac scene — USD layout
```
/World (Xform)                         metersPerUnit=1, upAxis=Z, physicsScene gravity -Z 9.81
  /World/panda                         Franka franka.usd reference (asset root S3 6.1); finger drive maxForce ↑ (~70 N)
  /World/Background (Xform, static)    /Splats (ParticleField3DGaussianSplat ref), xformOp = T_world_from_source
  /World/Env/<name>                    invisible static collider meshes (CollisionAPI), physics material
  /World/Objects/<name> (Xform)        RigidBodyAPI + MassAPI, xform = body_pose_world
      /Collision (Mesh)                visibility=invisible, CollisionAPI + MeshCollisionAPI(approximation), material
      /Splats                          ParticleField ref, xformOp = T_body_from_source
  /World/GroundPlane?                  optional invisible static plane under table
  /World/Camera_unity_main             recording camera
  /World/ActionGraph                   OnPlaybackTick → ROS2PublishClock(/clock) + ROS2ControlManager(targetPrim=/World/panda, ...)
```
Splat USD: produced with the image's converter (`omni.kit.converter.gsplat`: `read_ply`, `write_gaussian_splat_usd`, or
CLI `python3 -m usd_convert_gsplat`), cached as `out_dir/usd/<name>.usdc`. Physics materials: friction static/dynamic
≈1.0 on objects + finger pads. Physics: `/physics/updateToUsd` default True (splats must follow bodies).

## 5. ROS 2 interface between Isaac and MoveIt/MTC
| name | type | pub → sub | semantics |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Isaac → all | sim time; every MoveIt/MTC node `use_sim_time:=true` |
| controller_manager (in Isaac) | ros2_control | — | `joint_state_broadcaster`, `panda_arm_controller` (JTC, 7 arm joints), `panda_hand_controller` (JTC on `panda_finger_joint1`; finger2 is a PhysX mimic). Spawned from the ROS container with `controller_manager spawner`. |
| `/joint_states` | sensor_msgs/JointState | Isaac (JSB) → MoveIt | 9 joints |
| `/panda_arm_controller/follow_joint_trajectory`, `/panda_hand_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | MoveIt → Isaac | MoveIt controllers yaml: both `type: FollowJointTrajectory`, `action_ns: follow_joint_trajectory` |
| `/<obj>_position` for obj in {box, hammer, cube} | visualization_msgs/Marker | Isaac → MTC | header.frame_id `world`; pose = rigid-body (body frame) pose in W; `scale=(1,1,1)`; `type=MESH_RESOURCE`; published at 10 Hz, starting only AFTER physics has settled (≥1.5 s sim) |
| `/<obj>_target_position` | geometry_msgs/PoseStamped | Isaac → MTC | target pose of the BODY origin in W (manifest `target_pose_world`), 10 Hz, same start rule |
| `/twin/status` | std_msgs/String (JSON) | Isaac → anyone | per-second JSON: sim time, object body poses, "settled" flag — used for verification |
QoS: RELIABLE, VOLATILE, depth 10. Isaac must NOT publish TF for world/panda links (robot_state_publisher on the
MoveIt side owns TF and `/robot_description`).

## 6. MTC controller — generalized `mtc_dynamic`
- Fix Jazzy build (CUBE→BOX / delete dead helper; remove removed include; `.hpp` headers where needed).
- Generalize the hard-coded box+hammer sequence into an ordered list of K objects from parameters (keeping
  `pick_place_demo` executable & `pick_place_task` library names), each with: `name`, `mesh_path`, `pose_topic`,
  `target_topic`, grasp params (`grasp_offset_xyz`, `grasp_rpy`), `finger_close` (per-finger m), `place_offset_z`
  (or derive from mesh height), approach/lift/lower/retreat ranges. Default sequence (paper): box, cube, hammer.
  Remove the ×0.7 mesh-scale hack (`mesh_scale_factor` param default 1.0). Optional static obstacles list
  (`env_names`, `env_mesh_paths`, poses identity-in-world since env meshes are in W). Add a task timeout; Cartesian
  planner for approach/lift where it helps. Place: the object body origin must end at `target_pose_world`
  (convert body target → TCP target using the grasp transform the solution actually used, or attach-frame IK).
- Launch `launch/isaac.launch.py`: robot_state_publisher, static TF world→panda_link0, move_group with
  ExecuteTaskSolutionCapability, MoveIt controllers yaml for Isaac (both FJT), `use_sim_time:=true` everywhere,
  NO ros2_control_node/mock hardware; spawners for the 3 controllers (talking to Isaac's controller_manager);
  then `pick_place_demo` with params from `mtc_task.yaml` (path argument). Keep `demo.launch.py` (mock) working.
- Docker: `docker/moveit/Dockerfile` (ros:jazzy arm64 + MoveIt + MTC + ros2_controllers + panda config +
  generate_parameter_library ...), builds the repo package; `docker/moveit/run_mtc.sh`.
- `mtc_task.yaml` (generated by the export, consumed by the controller) — fixed schema; example at `ros2_ws/mtc_controller_final/src/config/twin_task.example.yaml`:
```yaml
/**:
  ros__parameters:
    object_names: [box, cube, hammer]        # execution order (paper: box→cardboard, cube→box, hammer→frame)
    mesh_scale_factor: 1.0                   # replaces the old hard-coded 0.7
    task_timeout: 180.0                      # s, planning timeout for each pick&place task
    objects:                                 # generate_parameter_library mapped params (__map_object_names)
      box:
        mesh_path: /abs/path/output/twin_res5/collision/box.obj   # vertices in BODY frame (metres)
        pose_topic: /box_position            # visualization_msgs/Marker, body pose in world
        target_topic: /box_target_position   # geometry_msgs/PoseStamped, target BODY pose in world
        grasp_offset_xyz: [0.0, 0.0, 0.0]    # TCP position relative to body origin, world axes (m)
        grasp_yaw: 0.0                       # TCP yaw about world Z for a top-down grasp (rad); fingers close along TCP y
        finger_close: 0.02                   # per-finger joint target when closed (m), < half object width → squeeze
        approach_min_dist: 0.05
        approach_max_dist: 0.15
        lift_min_dist: 0.05
        lift_max_dist: 0.15
    env_names: [table, cardboard]            # static collision objects added to the planning scene (pose = identity in world)
    env_mesh_paths: [/abs/.../collision/env_table.obj, /abs/.../collision/env_cardboard.obj]
```
  TCP convention: top-down grasp = TCP z pointing −Z_world; "TCP" = the MTC `ik_frame` (panda_link8 + 0.1 m along
  its z, i.e. between the fingertips; keep the existing `grasp_frame_transform` semantics). Place: TCP target =
  target_body_pose ∘ T_body_tcp (same relative grasp as at pick). Paths are absolute HOST paths; the MoveIt
  container mounts the output dir at the identical absolute path.

## 7. Orchestration (integration)
`scripts/run_twin_demo.sh`: (1) `python -m src.sim_export.export --config configs/twin_res5.yaml` (host venv),
(2) start Isaac container `simulation/isaac/run_isaac.sh` (headless, records `out_dir/recording/*.png` + mp4 from
the unity_main camera and a top view), (3) start MoveIt container `docker/moveit/run_mtc.sh`, (4) wait for MTC
"Execution complete"/failure, (5) collect `/twin/status` final object poses, compute success metrics (object body
position error to target, height checks, collisions), write `out_dir/report.json`.
Success criteria for the demo (paper): all three objects end within their goal regions (box on cardboard, cube on
box, hammer inside the frame), robot executes without collision, splats visibly follow objects in the recording.

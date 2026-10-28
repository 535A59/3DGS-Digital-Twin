# 3DGS Digital Twin

A high-fidelity digital twin framework for robotic manipulation, built on **3D Gaussian Splatting (3DGS)**. This repository implements the full perception-to-planning-to-execution pipeline described in:

> **"A High-Fidelity Digital Twin for Robotic Manipulation Based on 3D Gaussian Splatting"**

---

## Overview

The system reconstructs photorealistic, collision-ready digital twins from sparse RGB images in minutes, enabling zero-shot pick-and-place planning on real robots.

```
Sparse RGB Images
       │
       ▼
┌─────────────────────┐
│  InstantSplat (ext) │  ← 3DGS reconstruction (~229 s)
└─────────────────────┘
       │  point_cloud.ply + cameras.json
       ▼
┌─────────────────────┐
│  Step 1 · SAM       │  ← Interactive 2D segmentation
│  Segmentation       │     saves JSON polygon masks
└─────────────────────┘
       │  masks_json/
       ▼
┌─────────────────────┐
│  Step 2 · Semantic  │  ← Multi-view voting + KNN
│  Fusion             │     boundary refinement
└─────────────────────┘
       │  object.ply
       ▼
┌─────────────────────┐
│  Step 3a · Filter   │  ← Opacity + scale pruning
│  Step 3b · DBSCAN   │  ← Connectivity pruning
│  Step 3c · Mesh     │  ← Alpha Shapes → .obj
└─────────────────────┘
       │  object.obj (collision mesh)
       ▼
┌─────────────────────┐
│  Unity Simulation   │  ← MeshCollider + Rigidbody
│  + ROS 2 / MoveIt 2 │     collision-aware planning
└─────────────────────┘
```

---

## Repository Structure

```
3DGS-Digital-Twin/
├── pipeline.py                   # End-to-end orchestration script
├── requirements.txt
├── configs/
│   ├── default.yaml              # All tuneable parameters
│   └── twin_res5.yaml            # Isaac twin export: calibration, objects, targets
├── src/
│   ├── semantic_segmentation/
│   │   └── sam_segmentation.py   # Interactive SAM segmentation → JSON masks
│   ├── semantic_fusion/
│   │   └── fusion.py             # Multi-view voting + KNN 3D lifting
│   ├── geometry_refinement/
│   │   ├── filter.py             # Opacity + scale attribute filtering
│   │   ├── dbscan_clean.py       # DBSCAN connectivity pruning / splitting
│   │   └── meshing.py            # Alpha Shapes watertight meshing
│   └── sim_export/
│       └── export.py             # 3DGS scene → Isaac Sim assets + MTC task file
├── tools/
│   └── unity_gs_to_ply.py        # Unity GaussianSplatting .asset → 3DGS PLY
├── ros2_ws/
│   └── mtc_controller_final/     # MoveIt 2 Task Constructor ROS 2 package
├── simulation/
│   ├── isaac/                    # Isaac Sim 6.1 scene, runtime, report
│   ├── Scripts/                  # Unity C# scripts (see below)
│   └── Scenes/                   # Unity scene files
├── docker/moveit/                # ROS 2 Jazzy + MoveIt 2 + MTC image for the twin
├── scripts/run_twin_demo.sh      # Isaac twin end-to-end demo
├── tests/                        # Self-checks (python -m tests.<name>)
├── docs/isaac_twin_contract.md   # Isaac twin frames, file formats, ROS interface
├── external/
│   ├── InstantSplat/             # 3DGS reconstruction (submodule)
│   ├── UnityGaussianSplatting/   # Unity 3DGS renderer (submodule)
│   └── ros2-for-unity/           # ROS2ForUnity plugin (submodule)
└── data/
    └── real_env_video.mp4        # Sample real-robot environment video
```

---

## Quick Start

### 1. Clone (with submodules)

```bash
git clone --recurse-submodules https://github.com/your-username/3DGS-Digital-Twin.git
cd 3DGS-Digital-Twin
```

### 2. Install Python dependencies

```bash
pip install -r requirements.txt

# Install SAM separately:
pip install git+https://github.com/facebookresearch/segment-anything.git

# Download SAM ViT-H weights:
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

### 3. Reconstruct with InstantSplat (Step 0)

Follow the [InstantSplat README](external/InstantSplat/README.md) to reconstruct a scene from sparse RGB images. This produces:
- `point_cloud.ply` — 3DGS Gaussian point cloud
- `cameras.json`    — Calibrated camera parameters

### 4. Configure the pipeline

Copy `configs/default.yaml` and fill in your paths:

```yaml
ply:          /path/to/point_cloud.ply
cameras_json: /path/to/cameras.json
images_dir:   /path/to/images/
mask_dir:     /path/to/masks_json/
out_dir:      /path/to/output/
object_name:  hammer
sam_checkpoint: sam_vit_h_4b8939.pth
```

### 5. Run the full pipeline

```bash
python pipeline.py --config configs/default.yaml
```

This runs all three steps and saves:
- `output/segmented/hammer.ply`       — Segmented 3DGS point cloud
- `output/refined/hammer_filtered.ply` — After opacity/scale filtering
- `output/refined/hammer_clean.ply`    — After DBSCAN pruning
- `output/refined/hammer.obj`          — Final collision mesh

### 6. Run individual steps

```bash
# Step 1 only (interactive segmentation window will open):
python pipeline.py --config configs/default.yaml --steps 1

# Steps 2 and 3 only (if masks already exist):
python pipeline.py --config configs/default.yaml --steps 2,3

# Step 3 only (if segmented PLY already exists):
python pipeline.py --config configs/default.yaml --steps 3
```

---

## Module Reference

### `src/semantic_segmentation`

Interactive SAM-based 2D segmentation. Opens a window per image:
- **Left click** — add foreground point
- **Right click** — add background / exclusion point
- **`s` / Space** — save current mask as JSON polygon file
- **`z`** — undo last point
- **`n`** — next image
- **`q`** — quit

```bash
python -m src.semantic_segmentation.sam_segmentation \
    --input_folder  /path/to/images \
    --output_folder /path/to/masks_json \
    --sam_checkpoint sam_vit_h_4b8939.pth
```

### `src/semantic_fusion`

Lifts 2D masks into 3D via multi-view voting and iterative KNN refinement.

```bash
python -m src.semantic_fusion.fusion \
    --ply          /path/to/point_cloud.ply \
    --cameras_json /path/to/cameras.json \
    --images_dir   /path/to/images \
    --mask_dir     /path/to/masks_json \
    --out_dir      /path/to/output \
    --object_name  hammer
```

### `src/geometry_refinement`

**Stage 3a — Attribute filter:**

```bash
python -m src.geometry_refinement.filter \
    --input  /path/to/object.ply \
    --output /path/to/object_filtered.ply \
    --min_alpha 0.05 --max_scale 0.007
```

**Stage 3b — DBSCAN clean** (keep largest cluster):

```bash
python -m src.geometry_refinement.dbscan_clean clean \
    --input  /path/to/object_filtered.ply \
    --output /path/to/object_clean.ply \
    --eps 0.01 --min_samples 50
```

**Stage 3b — DBSCAN split** (save each cluster separately):

```bash
python -m src.geometry_refinement.dbscan_clean split \
    --input   /path/to/object_filtered.ply \
    --out_dir /path/to/clusters/ \
    --eps 0.05 --min_samples 50
```

**Stage 3c — Meshing:**

```bash
python -m src.geometry_refinement.meshing \
    --input  /path/to/object_clean.ply \
    --output /path/to/object.obj \
    --alpha  0.01
```

---

## ROS 2 / MoveIt 2 Integration

The `ros2_ws/mtc_controller_final/` package implements collision-aware motion planning using MoveIt 2 Task Constructor (MTC).

```bash
cd ros2_ws/mtc_controller_final
colcon build
source install/setup.bash

# Launch planning scene + demo:
ros2 launch mtc_dynamic demo.launch.py

# Run pick-and-place with mesh collision object:
ros2 launch mtc_dynamic run.launch.py
```

The generated `.obj` mesh can be loaded as a `CollisionObject` in MoveIt 2's planning scene. See `ros2_ws/mtc_controller_final/src/scripts/` for utility scripts.

---

## Unity Simulation

The collision mesh is imported into Unity alongside the [UnityGaussianSplatting](external/UnityGaussianSplatting/) renderer for visual fidelity:

1. Import `object.obj` into a Unity project.
2. Add a `MeshCollider` component (enable **Convex** for physics simulation).
3. Add a `Rigidbody` component.
4. Use the [ROS2ForUnity](external/ros2-for-unity/) plugin to bridge Unity ↔ ROS 2 for sensor simulation and command execution.

---

## Isaac Sim Digital Twin

Isaac Sim 6.1 replaces the Unity runtime for the closed-loop pick-and-place demo (paper Sec. 4.1.1, the
RES5 scene): **box → on the cardboard box, cube → on the placed box, hammer → into the tape frame**.

- The 3DGS splats are what you see; the collision meshes from `src/geometry_refinement` are invisible PhysX
  rigid bodies (gravity + contact). Each object's splats are parented to its rigid body and move with it.
- The Franka runs on `isaacsim.ros2.control` inside Isaac. MoveIt 2 + MTC (`mtc_dynamic`) run in a second
  container and plan against the live object poses Isaac publishes. Grasps are physical (friction only, no attach).
- World frame = `panda_link0`, Z-up, metres. Frames, file formats and the ROS interface are in
  [docs/isaac_twin_contract.md](docs/isaac_twin_contract.md).

### Requirements

- Docker with an NVIDIA GPU, the `nvcr.io/nvidia/isaac-sim:6.1.0` image (NGC), and network access for the
  `ros:jazzy` build of `docker/moveit/Dockerfile` (`run_mtc.sh` builds it on first use).
- Host Python with `requirements.txt` (numpy, scipy, scikit-learn, plyfile, open3d, opencv, PyYAML, trimesh).
- Input data in `old_code-data/3dgs_resource/` (not in the repo): `mesh_source/object{1,2,3}-copy.ply`,
  `GSreal2sim2real/Assets/GaussianAssets/background-copy-cl.*` and `3dgs/RES5/arm.ply`.

### Run

```bash
# export -> Isaac Sim -> MoveIt/MTC -> recording + report, about 2.5 min
ROS_DOMAIN_ID=64 PYTHON=python3 scripts/run_twin_demo.sh

# the two recorded cameras side by side (Unity main camera | top view)
ffmpeg -i output/twin_res5/recording/unity_main.mp4 -i output/twin_res5/recording/top.mp4 \
    -filter_complex hstack -c:v libx264 -pix_fmt yuv420p output/twin_res5/side_by_side.mp4
```

`ROS_DOMAIN_ID` must be set and non-zero. Outputs in `output/twin_res5/` are `manifest.json`, `collision/`,
`mtc_task.yaml`, `isaac.log`, `mtc.log`, `recording/{unity_main,top}.mp4`, `final_state.json` and `report.json`.

The stages can also run on their own:

```bash
python -m src.sim_export.export --config configs/twin_res5.yaml               # assets + mtc_task.yaml
ROS_DOMAIN_ID=64 simulation/isaac/run_isaac.sh --seconds 600                  # Isaac container
ROS_DOMAIN_ID=64 docker/moveit/run_mtc.sh $PWD/output/twin_res5/mtc_task.yaml # MoveIt + MTC container
python simulation/isaac/twin_report.py --out_dir output/twin_res5 --config configs/twin_res5.yaml
```

### Calibration and physics settings

- `configs/twin_res5.yaml` keeps the robot base and rotation of the paper's Unity alignment, levels the
  table to +Z and replaces Unity's scale 2.27 with 1.75 m/unit (the 2.5 cm cube as anchor). The evidence is
  in `calibration_notes` there.
- Added for a stable simulation (not part of the paper's method):
  - a torsional friction patch on the object colliders;
  - each body's origin and centre of mass at its collider centroid, with uniform density;
  - the cardboard lid (5 mm) and the box bottom (10 mm) flattened;
  - masses box 0.15 kg, cube 0.01 kg, hammer 0.12 kg;
  - a raised finger drive force.

### Results

Three consecutive full runs, each with physical grasps and no unexpected contacts. Values are the horizontal
error to the target and the final tilt. Pass thresholds: 30 mm horizontal, 15 mm vertical, 15° tilt.

| Run | box → cardboard | cube → box | hammer → frame |
|-----|-----------------|------------|----------------|
| I | 6.6 mm, 0° | 1.4 mm, 2.2° | 3.2 mm |
| J | 5.5 mm, 0° | 1.6 mm, 2.2° | 3.5 mm |
| K | 2.4 mm, 0° | 1.0 mm, 2.2° | 3.3 mm |

### Known limitations

- The cube ends 5–6 mm low with a 2.2° tilt, and the box can rotate up to about 5.7° in the gripper.
- The hammer is judged by the 30 mm horizontal tolerance, not by the frame outline.
- `move_group` segfaults at shutdown, after "Execution complete"; this does not affect the run.
- `launch/run.launch.py` (the Unity entry point) still uses the old `panda_config.yaml` parameters; the
  twin uses `launch/isaac.launch.py`.

### Self-checks

```bash
python -m tests.test_sim_export    # the last two checks need old_code-data/
python -m tests.test_twin_report
```

---

## Parameter Tuning Guide

| Parameter | Default | Effect |
|-----------|---------|--------|
| `vote_fraction` | 0.5 | Lower → more permissive object mask |
| `knn_k` | 15 | Higher → smoother boundary |
| `knn_threshold` | 0.6 | Lower → more boundary updates |
| `filter.min_alpha` | 0.05 | Lower → keep more transparent Gaussians |
| `filter.max_scale` | 0.007 | Higher → keep larger Gaussians |
| `dbscan.eps` | 0.01 | Higher → merge nearby clusters |
| `meshing.alpha_value` | 0.01 | Higher → coarser / fewer holes in mesh |

---

## Performance (from paper)

| Metric | This system | NeRF baseline |
|--------|------------|---------------|
| Reconstruction time | ~229 s | ~1123 s |
| PSNR | 37.03 dB | 29.14 dB |
| SSIM | 0.9821 | 0.9037 |
| Pick-and-place success | **90%** (9/10) | — |

---

## License

This project builds on several open-source works:
- [3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting) — Inria / MPI
- [InstantSplat](https://github.com/NVlabs/InstantSplat) — NVIDIA
- [Segment Anything Model](https://github.com/facebookresearch/segment-anything) — Meta AI
- [Open3D](https://github.com/isl-org/Open3D) — Intel ISL
- [UnityGaussianSplatting](https://github.com/aras-p/UnityGaussianSplatting) — Aras Pranckevičius

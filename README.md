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
│   └── default.yaml              # All tuneable parameters
├── src/
│   ├── semantic_segmentation/
│   │   └── sam_segmentation.py   # Interactive SAM segmentation → JSON masks
│   ├── semantic_fusion/
│   │   └── fusion.py             # Multi-view voting + KNN 3D lifting
│   └── geometry_refinement/
│       ├── filter.py             # Opacity + scale attribute filtering
│       ├── dbscan_clean.py       # DBSCAN connectivity pruning / splitting
│       └── meshing.py            # Alpha Shapes watertight meshing
├── ros2_ws/
│   └── mtc_controller_final/     # MoveIt 2 Task Constructor ROS 2 package
├── simulation/
│   ├── Scripts/                  # Unity C# scripts (see below)
│   └── Scenes/                   # Unity scene files
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

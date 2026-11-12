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
│  Step 2 · Semantic  │  ← Z-buffer visibility + depth-
│  Fusion             │     gated consensus + KNN (×3)
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
│   │   └── fusion.py             # Visibility-aware, depth-gated 3D lifting (Sec. 3.2)
│   ├── geometry_refinement/
│   │   ├── filter.py             # Opacity + scale attribute filtering
│   │   ├── dbscan_clean.py       # DBSCAN connectivity pruning / splitting
│   │   └── meshing.py            # Alpha Shapes watertight meshing
│   └── evaluation/
│       └── metrics.py            # Table 4 consistency / ghost index, Table 5 geometry, mIoU
├── tests/
│   └── test_semantic_fusion.py   # Synthetic-scene self-checks
├── ros2_ws/
│   ├── mtc_controller_final/     # MoveIt 2 Task Constructor ROS 2 package
│   ├── point_cloud/              # Publishes background PLY (Unity → ROS frame) as PointCloud2
│   └── cloud_to_octomap/         # PointCloud2 → OctoMap for the MoveIt planning scene
├── simulation/
│   ├── Scripts/                  # Unity C# scripts (see below)
│   └── Scenes/GSTestScene.unity  # Unity scene used in the paper
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
git clone --recurse-submodules https://github.com/535A59/3DGS-Digital-Twin.git
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

Lifts 2D masks into 3D (paper Sec. 3.2, Eqs. 2–3). For up to `max_iter` feedback iterations:

1. **Visibility** — per view, a z-buffer of opaque Gaussians (`occluder_min_alpha`) on
   `zbuf_cell`-pixel cells; a Gaussian is visible if within `visibility_tol` of the front surface → `V'_i`.
2. **Depth clustering** — per view, 1-D DBSCAN (`eps_depth`, `min_samples`) on the depths of the
   visible in-mask Gaussians. Primary cluster → `w_cluster = 1`; detached points decay as
   `exp(-½ (gap / eps_depth)²)` → `W_ij`.
3. **Consensus** — object iff `Σ_j W_ij · 1[(u,v) ∈ M_j] ≥ |V'_i| / κ` (κ = 1.5).
4. **KNN boundary refinement.**
5. **Feedback** — in-mask Gaussians rejected by the depth gate are dropped from the next
   iteration's z-buffer, and the primary depth cluster is chosen by overlap with the current 3D labels.

Surfaces never seen in any view (e.g. an object's underside resting on the table) receive no
observations and are not completed explicitly; the Alpha Shapes mesh closes them with the hull
of the observed points.

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

### `src/evaluation`

```bash
# Table 4: consistency / ghost index sweep (every κ reruns the full feedback + KNN pipeline)
python -m src.evaluation.metrics table4 \
    --ply point_cloud.ply --cameras_json cameras.json \
    --images_dir images/ --mask_dir masks_json/ --kappas 2.0,1.8,1.5,1.2,1.0

# Table 5: Chamfer / precision / recall / F1 vs. a manually cleaned GT cloud
python -m src.evaluation.metrics geometry --pred refined/hammer_clean.ply --gt gt/hammer.ply

# 2D mIoU of the projected 3D segmentation vs. annotated GT masks (same JSON format)
python -m src.evaluation.metrics miou --scene_ply point_cloud.ply \
    --object_ply segmented/hammer.ply --cameras_json cameras.json \
    --images_dir images/ --gt_mask_dir gt_masks_json/
```

Metric definitions (Ghost Index = % of mask pixels whose front-most point is left in the background) are in the
[metrics.py](src/evaluation/metrics.py) docstring.

### Tests

```bash
python -m tests.test_semantic_fusion
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

The static background reaches MoveIt as an OctoMap:

```bash
ros2 run point_cloud public_ply /path/to/background.ply   # → /background_cloud_reliable
ros2 launch cloud_to_octomap cloud_to_octomap.launch.py   # → octomap (frame: world)
```

`public_ply` waits for the background pose on `/background_tranform` (a `Marker`
published from Unity by `MarkerPublisher.cs`) and applies scale → rotation → translation.

---

## Unity Simulation

The collision mesh is imported into Unity alongside the [UnityGaussianSplatting](external/UnityGaussianSplatting/) renderer for visual fidelity:

1. Import `object.obj` into a Unity project.
2. Add a `MeshCollider` component (enable **Convex** for physics simulation).
3. Add a `Rigidbody` component.
4. Use the [ROS2ForUnity](external/ros2-for-unity/) plugin to bridge Unity ↔ ROS 2 for sensor simulation and command execution.

`simulation/` contains the scripts and scene from the paper's Unity (HDRP) project. Copy them into
`Assets/` of a project that already has UnityGaussianSplatting, ROS2ForUnity and a Franka URDF
import; the scene's references to the Gaussian assets and robot model must then be re-linked.

| Script | Role |
|--------|------|
| `JointStatePublisher.cs` / `ROS_Subscriber.cs` / `InitJoint.cs` | Robot joint state Unity ↔ `/joint_states` |
| `BackgroundCloudPublisher.cs` | Background point cloud → `/background_cloud` |
| `MarkerPublisher.cs` | Object / background poses as `Marker`s |
| `GraphMeshPublisher.cs` | Grasp-object mesh → `/grasp_mesh` |
| `ObjectPositionSubscriber.cs` | Object pose ← ROS |
| `DestinationPub.cs` | Placement target → ROS |
| `GSRigidBodySpawner.cs` | Convex-hull `MeshCollider` + `Rigidbody` from a Gaussian asset |
| `Transformation.cs` | Applies a 4×4 alignment matrix to a GameObject |
| `GS2Mesh.cs` | (legacy, commented out) in-Unity Gaussian → mesh conversion |

**Coordinate conventions.** 3DGS/Unity points are Y-up; ROS is Z-up. Both
`meshing.py --transform_coords` and `public_ply.py` map `(x, y, z)_unity → (z, -x, y)_ros`
after centring the cloud (`public_ply` centres on the AABB centre, `meshing.py` on the centroid).
Units are metres; InstantSplat/MASt3R reconstructions are only approximately metric, so the
Unity ↔ robot alignment (scale included) is set with `Transformation.cs`.

---

## Parameter Tuning Guide

| Parameter | Default | Effect |
|-----------|---------|--------|
| `kappa` | 1.5 | Higher → more permissive object mask (τ = visible views / κ) |
| `eps_depth` | 0.02 | Depth gap (m) separating object from bleeding background |
| `min_samples` | 10 | Minimum depth-cluster size |
| `visibility_tol` | 0.02 | Depth slack (m) for the z-buffer test |
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
| PSNR | 37.03 dB | 28.33 dB |
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

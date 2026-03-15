"""
3DGS Digital Twin — End-to-End Pipeline

Orchestrates the full reconstruction-to-collision workflow:

    Step 0 (external)  InstantSplat 3DGS reconstruction
    Step 1             Semantic segmentation  (SAM interactive masks → JSON)
    Step 2             Semantic fusion        (multi-view voting + KNN lifting)
    Step 3a            Geometry filter        (opacity + scale pruning)
    Step 3b            DBSCAN clean           (connectivity pruning)
    Step 3c            Watertight meshing     (Alpha Shapes → .obj)

Usage
-----
    # Full pipeline (skips step 1 if masks already exist):
    python pipeline.py --config configs/default.yaml

    # Or pass every argument explicitly:
    python pipeline.py \\
        --ply          /path/to/point_cloud.ply \\
        --cameras_json /path/to/cameras.json \\
        --images_dir   /path/to/images \\
        --mask_dir     /path/to/masks_json \\
        --out_dir      /path/to/output \\
        --object_name  hammer \\
        --sam_checkpoint /path/to/sam_vit_h_4b8939.pth

    # Run only specific steps (comma-separated):
    python pipeline.py --config configs/default.yaml --steps 2,3
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import yaml


# ---------------------------------------------------------------------------
# Step runners
# ---------------------------------------------------------------------------

def step1_segment(cfg: dict) -> None:
    """Interactive SAM segmentation of training images."""
    from src.semantic_segmentation import run_segmentation

    sam_ckpt = cfg.get("sam_checkpoint", "sam_vit_h_4b8939.pth")
    if not os.path.exists(sam_ckpt):
        print(
            f"[Step 1] WARNING: SAM checkpoint not found at '{sam_ckpt}'.\n"
            "         Skipping segmentation step — ensure masks already exist in mask_dir."
        )
        return

    print("\n" + "=" * 60)
    print("STEP 1: Semantic Segmentation (SAM)")
    print("=" * 60)
    run_segmentation(
        input_folder=cfg["images_dir"],
        output_folder=cfg["mask_dir"],
        sam_checkpoint=sam_ckpt,
        model_type=cfg.get("sam_model_type", "vit_h"),
        device=cfg.get("device", None),
    )


def step2_fuse(cfg: dict) -> str:
    """Lift 2D masks into 3D via multi-view voting + KNN refinement."""
    from src.semantic_fusion import fuse_masks_into_3d

    print("\n" + "=" * 60)
    print("STEP 2: Semantic Fusion (3D Lifting)")
    print("=" * 60)

    seg_cfg = cfg.get("semantic_fusion", {})
    out_dir = os.path.join(cfg["out_dir"], "segmented")

    obj_path = fuse_masks_into_3d(
        ply_path=cfg["ply"],
        cameras_json=cfg["cameras_json"],
        images_dir=cfg["images_dir"],
        mask_dir=cfg["mask_dir"],
        out_dir=out_dir,
        object_name=cfg.get("object_name", "object"),
        vote_fraction=seg_cfg.get("vote_fraction", 0.5),
        max_iter=seg_cfg.get("max_iter", 3),
        knn_k=seg_cfg.get("knn_k", 15),
        knn_threshold=seg_cfg.get("knn_threshold", 0.6),
        save_background=seg_cfg.get("save_background", False),
    )
    return obj_path


def step3_refine(cfg: dict, ply_path: str) -> str:
    """Three-stage geometry refinement: filter → DBSCAN → mesh."""
    from src.geometry_refinement import filter_gaussians, dbscan_clean, gaussians_to_mesh

    geo_cfg = cfg.get("geometry_refinement", {})
    filter_cfg = geo_cfg.get("filter", {})
    dbscan_cfg = geo_cfg.get("dbscan", {})
    mesh_cfg = geo_cfg.get("meshing", {})

    refine_dir = os.path.join(cfg["out_dir"], "refined")
    os.makedirs(refine_dir, exist_ok=True)

    object_name = cfg.get("object_name", "object")

    # --- 3a: Intrinsic attribute filtering ---
    print("\n" + "=" * 60)
    print("STEP 3a: Geometry Filter (opacity + scale)")
    print("=" * 60)
    filtered_path = os.path.join(refine_dir, f"{object_name}_filtered.ply")
    filter_gaussians(
        input_path=ply_path,
        output_path=filtered_path,
        min_alpha=filter_cfg.get("min_alpha", 0.05),
        min_scale=filter_cfg.get("min_scale", 1e-5),
        max_scale=filter_cfg.get("max_scale", 0.007),
    )

    # --- 3b: DBSCAN connectivity pruning ---
    print("\n" + "=" * 60)
    print("STEP 3b: DBSCAN Clean (connectivity pruning)")
    print("=" * 60)
    cleaned_path = os.path.join(refine_dir, f"{object_name}_clean.ply")
    dbscan_clean(
        input_path=filtered_path,
        output_path=cleaned_path,
        eps=dbscan_cfg.get("eps", 0.01),
        min_samples=dbscan_cfg.get("min_samples", 50),
    )

    # --- 3c: Alpha Shapes meshing ---
    print("\n" + "=" * 60)
    print("STEP 3c: Watertight Meshing (Alpha Shapes)")
    print("=" * 60)
    mesh_ext = mesh_cfg.get("format", "obj")
    mesh_path = os.path.join(refine_dir, f"{object_name}.{mesh_ext}")
    gaussians_to_mesh(
        input_ply_path=cleaned_path,
        output_mesh_path=mesh_path,
        min_opacity=mesh_cfg.get("min_opacity", 0.02),
        white_threshold=mesh_cfg.get("white_threshold", 1.0),
        alpha_value=mesh_cfg.get("alpha_value", 0.01),
        center_the_cloud=mesh_cfg.get("center_the_cloud", True),
        transform_coords=mesh_cfg.get("transform_coords", False),
        nb_neighbors=mesh_cfg.get("nb_neighbors", 20),
        std_ratio=mesh_cfg.get("std_ratio", 2.0),
    )
    return mesh_path


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def _load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _merge_cli_into_cfg(cfg: dict, args: argparse.Namespace) -> dict:
    """CLI arguments override config-file values when explicitly provided."""
    overrides = {
        "ply": args.ply,
        "cameras_json": args.cameras_json,
        "images_dir": args.images_dir,
        "mask_dir": args.mask_dir,
        "out_dir": args.out_dir,
        "object_name": args.object_name,
        "sam_checkpoint": args.sam_checkpoint,
    }
    for key, val in overrides.items():
        if val is not None:
            cfg[key] = val
    return cfg


def _validate_cfg(cfg: dict) -> None:
    required = ["ply", "cameras_json", "images_dir", "mask_dir", "out_dir"]
    missing = [k for k in required if not cfg.get(k)]
    if missing:
        print(
            f"[ERROR] Missing required configuration keys: {missing}\n"
            "        Provide them via --config or as CLI flags."
        )
        sys.exit(1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="3DGS Digital Twin — end-to-end reconstruction pipeline."
    )
    parser.add_argument(
        "--config", default="configs/default.yaml",
        help="YAML config file (default: configs/default.yaml)"
    )
    parser.add_argument("--ply", help="Override: input 3DGS PLY path")
    parser.add_argument("--cameras_json", help="Override: cameras.json path")
    parser.add_argument("--images_dir", help="Override: training images folder")
    parser.add_argument("--mask_dir", help="Override: JSON mask folder")
    parser.add_argument("--out_dir", help="Override: output directory")
    parser.add_argument("--object_name", help="Override: object name / file stem")
    parser.add_argument("--sam_checkpoint", help="Override: SAM checkpoint path")
    parser.add_argument(
        "--steps",
        default="1,2,3",
        help="Comma-separated steps to run: 1=segment, 2=fuse, 3=refine (default: 1,2,3)"
    )
    args = parser.parse_args()

    # --- Load and merge config ---
    if os.path.exists(args.config):
        cfg = _load_config(args.config)
    else:
        print(f"[INFO] Config file not found ({args.config}), using CLI args only.")
        cfg = {}

    cfg = _merge_cli_into_cfg(cfg, args)
    _validate_cfg(cfg)

    steps = {int(s.strip()) for s in args.steps.split(",")}

    print("\n" + "=" * 60)
    print("3DGS Digital Twin Pipeline")
    print(f"Object : {cfg.get('object_name', 'object')}")
    print(f"Steps  : {sorted(steps)}")
    print("=" * 60)

    t0 = time.time()

    # Step 1 — segmentation
    if 1 in steps:
        step1_segment(cfg)

    # Step 2 — semantic fusion
    segmented_ply = os.path.join(
        cfg["out_dir"], "segmented", cfg.get("object_name", "object") + ".ply"
    )
    if 2 in steps:
        segmented_ply = step2_fuse(cfg)
    elif not os.path.exists(segmented_ply):
        # Fall back to the raw PLY if no segmented file is found
        segmented_ply = cfg["ply"]
        print(f"[INFO] Skipping fusion — using {segmented_ply} directly.")

    # Step 3 — geometry refinement
    if 3 in steps:
        mesh_path = step3_refine(cfg, segmented_ply)

        elapsed = time.time() - t0
        print("\n" + "=" * 60)
        print(f"Pipeline complete in {elapsed:.1f}s")
        print(f"Final mesh: {mesh_path}")
        print("=" * 60)
    else:
        elapsed = time.time() - t0
        print(f"\nPipeline complete in {elapsed:.1f}s")


if __name__ == "__main__":
    main()

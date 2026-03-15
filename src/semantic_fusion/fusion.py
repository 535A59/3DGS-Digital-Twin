"""
Visibility-Aware Semantic Fusion.

Lifts 2D segmentation masks into 3D by projecting 3DGS Gaussian centres
onto each calibrated camera view and accumulating votes, then refining
label boundaries with iterative KNN smoothing.

Pipeline
--------
1. Load 3DGS point cloud (.ply) — all Gaussian attributes preserved.
2. Load camera parameters from cameras.json (InstantSplat / COLMAP format).
3. For every camera-mask pair:
   - Project all 3-D Gaussian centres to 2-D.
   - Increment votes for visible in-mask points.
4. Threshold votes → initial object / background labels + confidence.
5. Iterative KNN boundary refinement (max 3 passes).
6. Save segmented object and (optionally) background as .ply files.

Usage
-----
    python -m src.semantic_fusion.fusion \\
        --ply          /path/to/point_cloud.ply \\
        --cameras_json /path/to/cameras.json \\
        --images_dir   /path/to/images \\
        --mask_dir     /path/to/masks_json \\
        --out_dir      /path/to/output \\
        --object_name  my_object
"""

from __future__ import annotations

import json
import os
import sys
import argparse
from collections import Counter
from typing import Sequence

import cv2
import numpy as np
from PIL import Image
from plyfile import PlyData, PlyElement
from sklearn.neighbors import NearestNeighbors


# ---------------------------------------------------------------------------
# PLY I/O
# ---------------------------------------------------------------------------

def read_ply(path: str) -> tuple[np.ndarray, tuple]:
    """Load a 3DGS PLY file, returning (N×F float32 array, field names)."""
    data = PlyData.read(path)["vertex"].data
    fields = data.dtype.names
    if len(data) == 0:
        return np.empty((0, len(fields)), dtype=np.float32), fields
    pts = np.stack([data[f] for f in fields], axis=-1).astype(np.float32)
    return pts, fields


def write_ply(path: str, pts: np.ndarray, fields: Sequence[str]) -> None:
    """Write a 3DGS point cloud array back to a PLY file."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    dtype = [(f, "<f4") for f in fields]
    arr = np.empty(len(pts), dtype=dtype)
    for i, f in enumerate(fields):
        arr[f] = pts[:, i]
    PlyData([PlyElement.describe(arr, "vertex")], text=False).write(path)


# ---------------------------------------------------------------------------
# Camera utilities
# ---------------------------------------------------------------------------

def load_cameras(cameras_json: str, img_dir: str) -> list[dict]:
    """
    Parse cameras.json (InstantSplat / COLMAP export format).

    Expected per-frame keys: img_name, width, height, fx, fy,
    rotation (3×3 row-major list), position (3-vector world position).
    """
    frames = json.load(open(cameras_json))
    cameras = []
    for fr in frames:
        name = fr["img_name"]
        img_p = next(
            (
                os.path.join(img_dir, name + ext)
                for ext in (".jpg", ".png", ".jpeg")
                if os.path.exists(os.path.join(img_dir, name + ext))
            ),
            None,
        )
        if img_p is None:
            print(f"[WARN] Image {name}.* not found, skipping.")
            continue

        w, h = fr["width"], fr["height"]
        fx, fy = fr["fx"], fr["fy"]
        K = np.array([[fx, 0, w / 2], [0, fy, h / 2], [0, 0, 1]], dtype=np.float32)
        R = np.array(fr["rotation"], dtype=np.float32)   # world-to-camera rotation
        C = np.array(fr["position"], dtype=np.float32)   # camera centre in world
        # Build [R | t] extrinsic matrix: t = -R^T * C
        RT = np.hstack([R.T, (-R.T @ C.reshape(3, 1))])
        cameras.append(dict(path=img_p, K=K, RT=RT, shape=(h, w)))
    return cameras


def project(pts: np.ndarray, K: np.ndarray, RT: np.ndarray) -> np.ndarray:
    """Project N×3 world points to N×2 integer pixel coordinates."""
    pts_h = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float32)])  # (N,4)
    uvw = (K @ RT @ pts_h.T).T                                           # (N,3)
    return (uvw[:, :2] / uvw[:, 2:]).astype(int)                         # (N,2)


# ---------------------------------------------------------------------------
# Mask loading  (JSON polygon format)
# ---------------------------------------------------------------------------

def load_masks_from_json(cameras: list[dict], mask_dir: str) -> list[np.ndarray]:
    """
    Reconstruct binary masks from polygon JSON files saved by sam_segmentation.

    Each JSON file must contain a 'segmentation_polygons' key with a list of
    [[x,y], ...] polygon lists.

    Returns a list of bool arrays of shape (H, W).
    """
    masks: list[np.ndarray] = []
    for cam in cameras:
        fname = os.path.splitext(os.path.basename(cam["path"]))[0] + "_mask.json"
        mpath = os.path.join(mask_dir, fname)

        if not os.path.exists(mpath):
            raise FileNotFoundError(f"JSON mask not found: {mpath}")

        with open(mpath, "r") as f:
            data = json.load(f)

        h, w = cam["shape"]
        polygons_list = data["segmentation_polygons"]

        mask_image = np.zeros((h, w), dtype=np.uint8)
        contours_to_draw = [
            np.array(poly, dtype=np.int32) for poly in polygons_list if poly
        ]
        if contours_to_draw:
            cv2.fillPoly(mask_image, contours_to_draw, 255)

        masks.append(mask_image == 255)
    return masks


# ---------------------------------------------------------------------------
# Core fusion
# ---------------------------------------------------------------------------

def _voting_pass(
    pts_xyz: np.ndarray,
    cameras: list[dict],
    masks: list[np.ndarray],
) -> np.ndarray:
    """Return per-point vote counts (how many camera views see it as object)."""
    votes = np.zeros(len(pts_xyz), dtype=np.int32)
    for cam, mask in zip(cameras, masks):
        uv = project(pts_xyz, cam["K"], cam["RT"])
        h, w = cam["shape"]
        in_img = (
            (uv[:, 0] >= 0) & (uv[:, 0] < w) &
            (uv[:, 1] >= 0) & (uv[:, 1] < h)
        )
        idx = np.where(in_img)[0]
        votes[idx] += mask[uv[idx, 1], uv[idx, 0]].astype(np.int32)
    return votes


def _knn_refinement(
    labels: np.ndarray,
    confidences: np.ndarray,
    positions: np.ndarray,
    *,
    max_iter: int = 3,
    knn_k: int = 15,
    knn_threshold: float = 0.6,
    confidence_gate: float = 0.8,
) -> np.ndarray:
    """
    Iterative KNN boundary refinement.

    A point's label is updated if its KNN neighbourhood is consistent
    (>= knn_threshold fraction agree) AND the point's current confidence
    is below confidence_gate.
    """
    print(f"\n[Refinement] max_iter={max_iter}, K={knn_k}, threshold={knn_threshold}")

    nbrs = NearestNeighbors(n_neighbors=knn_k + 1, algorithm="kd_tree", n_jobs=-1)
    nbrs.fit(positions)
    _, indices = nbrs.kneighbors(positions)
    neighbor_indices = indices[:, 1:]  # exclude self

    for iteration in range(1, max_iter + 1):
        changed = 0
        for i in range(len(positions)):
            nb_labels = labels[neighbor_indices[i]]
            label_counts = Counter(nb_labels)
            top_label, top_count = label_counts.most_common(1)[0]
            ratio = top_count / knn_k

            if ratio >= knn_threshold:
                if labels[i] != top_label and confidences[i] < confidence_gate:
                    labels[i] = top_label
                    confidences[i] = ratio
                    changed += 1
                elif labels[i] == top_label:
                    confidences[i] = max(confidences[i], ratio)

        print(f"  [Iter {iteration}] {changed} labels changed")
        if changed < len(positions) * 0.001:
            print("  [Converged] < 0.1 % changed — stopping early.")
            break

    return labels


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def fuse_masks_into_3d(
    ply_path: str,
    cameras_json: str,
    images_dir: str,
    mask_dir: str,
    out_dir: str,
    object_name: str = "object",
    vote_fraction: float = 0.5,
    max_iter: int = 3,
    knn_k: int = 15,
    knn_threshold: float = 0.6,
    save_background: bool = False,
) -> str:
    """
    Lift 2D SAM masks into 3D and save the segmented point cloud.

    Parameters
    ----------
    ply_path        Path to the full 3DGS point cloud.
    cameras_json    Path to cameras.json.
    images_dir      Directory containing the training images.
    mask_dir        Directory containing the JSON mask files.
    out_dir         Output directory for the segmented PLY files.
    object_name     Stem for the output file (e.g. "hammer" → hammer.ply).
    vote_fraction   Fraction of cameras that must agree for initial label=1.
    max_iter        Maximum KNN refinement iterations.
    knn_k           Number of neighbours for KNN refinement.
    knn_threshold   Neighbourhood consistency threshold for label update.
    save_background Whether to also save the background point cloud.

    Returns
    -------
    Path to the saved object PLY file.
    """
    # --- Load data ---
    cameras = load_cameras(cameras_json, images_dir)
    if not cameras:
        sys.exit("[ERROR] No cameras loaded from cameras.json.")

    pts_all, fields = read_ply(ply_path)
    if pts_all.size == 0:
        sys.exit("[ERROR] Empty point cloud.")

    masks = load_masks_from_json(cameras, mask_dir)
    print(f"[INFO] {len(cameras)} views | {pts_all.shape[0]} points | masks OK")

    # --- Multi-view voting ---
    pts_xyz = pts_all[:, :3]
    votes = _voting_pass(pts_xyz, cameras, masks)

    vote_threshold = int(len(cameras) * vote_fraction)
    labels = (votes >= vote_threshold).astype(np.int32)
    confidences = (
        np.abs(votes - vote_threshold) / max(len(cameras), 1)
    ).clip(0.0, 1.0).astype(np.float32)

    print(
        f"[INFO] Initial segmentation: "
        f"object={np.sum(labels == 1)} | background={np.sum(labels == 0)}"
    )

    # --- KNN boundary refinement ---
    labels = _knn_refinement(
        labels, confidences, pts_xyz,
        max_iter=max_iter,
        knn_k=knn_k,
        knn_threshold=knn_threshold,
    )

    obj_mask = labels == 1
    obj_pts = pts_all[obj_mask]
    bg_pts = pts_all[~obj_mask]
    print(
        f"[INFO] After refinement: "
        f"object={len(obj_pts)} | background={len(bg_pts)}"
    )

    # --- Save ---
    os.makedirs(out_dir, exist_ok=True)
    obj_path = os.path.join(out_dir, f"{object_name}.ply")
    write_ply(obj_path, obj_pts, fields)
    print(f"[OK] Object saved → {obj_path}")

    if save_background:
        bg_path = os.path.join(out_dir, "background.ply")
        write_ply(bg_path, bg_pts, fields)
        print(f"[OK] Background saved → {bg_path}")

    return obj_path


class SemanticFusion:
    """Configurable wrapper around fuse_masks_into_3d."""

    def __init__(
        self,
        vote_fraction: float = 0.5,
        max_iter: int = 3,
        knn_k: int = 15,
        knn_threshold: float = 0.6,
        save_background: bool = False,
    ):
        self.vote_fraction = vote_fraction
        self.max_iter = max_iter
        self.knn_k = knn_k
        self.knn_threshold = knn_threshold
        self.save_background = save_background

    def run(
        self,
        ply_path: str,
        cameras_json: str,
        images_dir: str,
        mask_dir: str,
        out_dir: str,
        object_name: str = "object",
    ) -> str:
        return fuse_masks_into_3d(
            ply_path=ply_path,
            cameras_json=cameras_json,
            images_dir=images_dir,
            mask_dir=mask_dir,
            out_dir=out_dir,
            object_name=object_name,
            vote_fraction=self.vote_fraction,
            max_iter=self.max_iter,
            knn_k=self.knn_k,
            knn_threshold=self.knn_threshold,
            save_background=self.save_background,
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    parser = argparse.ArgumentParser(
        description="Lift 2D SAM masks into 3D via multi-view voting + KNN refinement."
    )
    parser.add_argument("--ply", required=True, help="Input 3DGS point cloud (.ply)")
    parser.add_argument("--cameras_json", required=True, help="cameras.json path")
    parser.add_argument("--images_dir", required=True, help="Training images folder")
    parser.add_argument("--mask_dir", required=True, help="JSON mask folder")
    parser.add_argument("--out_dir", required=True, help="Output folder")
    parser.add_argument("--object_name", default="object", help="Output file stem")
    parser.add_argument(
        "--vote_fraction", type=float, default=0.5,
        help="Fraction of cameras required for object label (default: 0.5)"
    )
    parser.add_argument("--max_iter", type=int, default=3, help="KNN refinement iterations")
    parser.add_argument("--knn_k", type=int, default=15, help="Number of KNN neighbours")
    parser.add_argument(
        "--knn_threshold", type=float, default=0.6, help="KNN consistency threshold"
    )
    parser.add_argument(
        "--save_background", action="store_true", help="Also save background.ply"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    fuse_masks_into_3d(
        ply_path=args.ply,
        cameras_json=args.cameras_json,
        images_dir=args.images_dir,
        mask_dir=args.mask_dir,
        out_dir=args.out_dir,
        object_name=args.object_name,
        vote_fraction=args.vote_fraction,
        max_iter=args.max_iter,
        knn_k=args.knn_k,
        knn_threshold=args.knn_threshold,
        save_background=args.save_background,
    )

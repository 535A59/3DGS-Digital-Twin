"""
Visibility-Aware Semantic Fusion (paper Sec. 3.2).

Lifts 2D segmentation masks into 3D by projecting 3DGS Gaussian centres
onto each calibrated view, gating every in-mask observation by depth
consistency, and accepting a point as object when its weighted votes reach
a fraction of the views in which it is actually visible.

Pipeline
--------
1. Load 3DGS point cloud (.ply) — all Gaussian attributes preserved.
2. Load camera parameters from cameras.json (InstantSplat / 3DGS format).
3. Repeat for up to ``max_iter`` feedback iterations:
   a. Per view, build a z-buffer from the current occluder set and keep only
      Gaussians that pass the depth/visibility test   →  V'_i
   b. Per view, run 1-D DBSCAN on the depths of the visible in-mask
      Gaussians (eps_depth, min_samples).  The primary cluster gets
      w_cluster ≈ 1; detached clusters (bleeding background, floaters)
      decay as exp(-½ (gap / eps_depth)²)                →  W_ij  (Eq. 2)
   c. p_i is object iff  Σ_j W_ij · 1[(u,v) ∈ M_j]  ≥  |V'_i| / κ   (Eq. 3)
   d. KNN boundary refinement of the labels.
   e. Feedback: in-mask Gaussians rejected by the depth gate are removed from
      the next iteration's z-buffer (cleaner depth maps), and the primary
      depth cluster is chosen by overlap with the current object labels.
4. Save segmented object and (optionally) background as .ply files.

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
from typing import Sequence

import cv2
import numpy as np
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
    Parse cameras.json (InstantSplat / 3DGS export format).

    Expected per-frame keys: img_name, width, height, fx, fy,
    rotation (3×3 camera-to-world rotation), position (camera centre in world).
    """
    frames = json.load(open(cameras_json))
    cameras = []
    for fr in frames:
        name = fr["img_name"]
        stem, suffix = os.path.splitext(name)
        if suffix.lower() in (".jpg", ".png", ".jpeg"):   # img_name may already carry the extension
            name = stem
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
        R = np.array(fr["rotation"], dtype=np.float32)   # camera-to-world rotation
        C = np.array(fr["position"], dtype=np.float32)   # camera centre in world
        # World-to-camera extrinsic [R^T | -R^T C]
        RT = np.hstack([R.T, (-R.T @ C.reshape(3, 1))])
        cameras.append(dict(path=img_p, K=K, RT=RT, shape=(h, w)))
    return cameras


def project(pts: np.ndarray, K: np.ndarray, RT: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Project N×3 world points.

    Returns (N×2 integer pixel coords, N camera-space depths).  Pixel coords
    of points with depth <= 0 are meaningless — filter them with ``in_view``.
    """
    cam = pts @ RT[:, :3].T + RT[:, 3]
    z = cam[:, 2]
    uv = (cam @ K.T)[:, :2] / np.where(z > 1e-6, z, 1.0)[:, None]
    return np.floor(uv).astype(np.int64), z


def in_view(uv: np.ndarray, z: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Points in front of the camera that land inside the image."""
    h, w = shape
    return (
        (z > 1e-6) &
        (uv[:, 0] >= 0) & (uv[:, 0] < w) &
        (uv[:, 1] >= 0) & (uv[:, 1] < h)
    )


def visible_points(
    pts_xyz: np.ndarray,
    cam: dict,
    occluders: np.ndarray,
    *,
    visibility_tol: float = 0.02,
    zbuf_cell: int = 4,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Depth/visibility test against a z-buffer of the occluder Gaussians.

    The z-buffer is built on ``zbuf_cell``×``zbuf_cell`` pixel cells so that
    sparse Gaussian centres still form a closed surface.

    Returns (indices of visible points, their pixel coords, their depths).
    """
    uv, z = project(pts_xyz, cam["K"], cam["RT"])
    idx = np.flatnonzero(in_view(uv, z, cam["shape"]))
    uv, z = uv[idx], z[idx]

    h, w = cam["shape"]
    wc = -(-w // zbuf_cell)
    cell = (uv[:, 1] // zbuf_cell) * wc + uv[:, 0] // zbuf_cell
    zbuf = np.full((-(-h // zbuf_cell)) * wc, np.inf, dtype=np.float32)
    occ = occluders[idx]
    np.minimum.at(zbuf, cell[occ], z[occ])

    vis = z <= zbuf[cell] + visibility_tol
    return idx[vis], uv[vis], z[vis]


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


def load_scene(
    ply_path: str,
    cameras_json: str,
    images_dir: str,
    mask_dir: str,
    occluder_min_alpha: float = 0.1,
):
    """
    Load everything fusion needs.

    Returns (pts_all, fields, cameras, masks, occluders) where ``occluders``
    marks Gaussians opaque enough (sigmoid(opacity) >= occluder_min_alpha) to
    block the view in the z-buffer.
    """
    cameras = load_cameras(cameras_json, images_dir)
    if not cameras:
        sys.exit("[ERROR] No cameras loaded from cameras.json.")

    pts_all, fields = read_ply(ply_path)
    if pts_all.size == 0:
        sys.exit("[ERROR] Empty point cloud.")

    masks = load_masks_from_json(cameras, mask_dir)

    if "opacity" in fields:
        alpha = 1.0 / (1.0 + np.exp(-pts_all[:, fields.index("opacity")]))
        occluders = alpha >= occluder_min_alpha
    else:
        occluders = np.ones(len(pts_all), dtype=bool)

    print(
        f"[INFO] {len(cameras)} views | {pts_all.shape[0]} points "
        f"({occluders.sum()} occluders) | masks OK"
    )
    return pts_all, fields, cameras, masks, occluders


# ---------------------------------------------------------------------------
# Core fusion
# ---------------------------------------------------------------------------

def dbscan_1d(values: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """
    Exact DBSCAN on scalar values (same core/noise semantics as sklearn).

    Sort-based O(n log n) — sklearn on dense 1-D depths needs O(n²) memory.
    Border points go to the nearest core point's cluster.  Noise = -1.
    """
    labels = np.full(len(values), -1, dtype=np.int64)
    if len(values) == 0:
        return labels
    order = np.argsort(values)
    s = values[order]
    counts = np.searchsorted(s, s + eps, "right") - np.searchsorted(s, s - eps, "left")
    core = s[counts >= min_samples]
    if core.size == 0:
        return labels

    core_cluster = np.concatenate([[0], np.cumsum(np.diff(core) > eps)])
    right = np.searchsorted(core, s).clip(max=len(core) - 1)
    left = (right - 1).clip(min=0)
    nearest = np.where(np.abs(s - core[left]) <= np.abs(s - core[right]), left, right)
    labels[order] = np.where(
        np.abs(s - core[nearest]) <= eps, core_cluster[nearest], -1
    )
    return labels


def _cluster_weights(
    depths: np.ndarray,
    prev_object: np.ndarray | None,
    eps_depth: float,
    min_samples: int,
) -> np.ndarray:
    """
    Soft spatial gate w_cluster for the in-mask Gaussians of one view.

    The primary depth cluster is the largest one on the first pass and, once
    3D labels exist, the one containing the most current object points.
    Points inside the primary cluster's depth span get 1; others decay with
    their depth gap to that span.
    """
    lab = dbscan_1d(depths, eps_depth, min_samples)
    clustered = lab >= 0
    if not clustered.any():
        # Too few visible in-mask points to cluster — trust the mask.
        return np.ones(len(depths), dtype=np.float32)

    score = np.bincount(lab[clustered])
    if prev_object is not None:
        overlap = np.bincount(lab[clustered & prev_object], minlength=len(score))
        if overlap.any():
            score = overlap
    primary = lab == np.argmax(score)

    lo, hi = depths[primary].min(), depths[primary].max()
    gap = np.maximum(0.0, np.maximum(lo - depths, depths - hi))
    return np.exp(-0.5 * (gap / eps_depth) ** 2).astype(np.float32)


def _knn_refinement(
    labels: np.ndarray,
    confidences: np.ndarray,
    neighbors: np.ndarray,
    *,
    knn_threshold: float = 0.6,
    confidence_gate: float = 0.8,
) -> np.ndarray:
    """
    KNN boundary refinement (one pass, vectorised).

    A point takes its neighbourhood's majority label when >= knn_threshold of
    its K neighbours agree AND its own vote confidence is below
    confidence_gate.
    """
    frac = labels[neighbors].mean(axis=1)
    top = (frac >= 0.5).astype(labels.dtype)
    ratio = np.maximum(frac, 1.0 - frac)
    flip = (ratio >= knn_threshold) & (top != labels) & (confidences < confidence_gate)
    out = labels.copy()
    out[flip] = top[flip]
    return out


def lift_labels(
    pts_xyz: np.ndarray,
    cameras: list[dict],
    masks: list[np.ndarray],
    occluders: np.ndarray | None = None,
    *,
    kappa: float = 1.5,
    eps_depth: float = 0.02,
    min_samples: int = 10,
    visibility_tol: float = 0.02,
    zbuf_cell: int = 4,
    max_iter: int = 3,
    knn_k: int = 15,
    knn_threshold: float = 0.6,
    confidence_gate: float = 0.8,
) -> tuple[np.ndarray, dict]:
    """
    Visibility-aware, depth-gated multi-view consensus (Eqs. 2–3).

    ``kappa`` sets the adaptive threshold τ_i = |V'_i| / κ.  ``kappa=inf``
    degenerates to naive projection (any positive weighted vote).

    Returns (bool object labels, stats) where stats holds the final
    per-point weighted votes ``votes``, visible-view counts ``visible`` and
    in-mask observation counts ``in_mask``.
    """
    n = len(pts_xyz)
    base_occ = np.ones(n, dtype=bool) if occluders is None else occluders
    occ = base_occ
    neighbors = (
        NearestNeighbors(n_neighbors=knn_k, n_jobs=-1)
        .fit(pts_xyz)
        .kneighbors(return_distance=False)
    )

    labels = None
    for it in range(1, max_iter + 1):
        votes = np.zeros(n, dtype=np.float32)
        visible = np.zeros(n, dtype=np.int32)
        in_mask = np.zeros(n, dtype=np.int32)
        prev_obj = None if labels is None else labels.astype(bool)

        for cam, mask in zip(cameras, masks):
            idx, uv, z = visible_points(
                pts_xyz, cam, occ,
                visibility_tol=visibility_tol, zbuf_cell=zbuf_cell,
            )
            m = mask[uv[:, 1], uv[:, 0]]
            hit = idx[m]
            visible[idx] += 1
            in_mask[hit] += 1
            votes[hit] += _cluster_weights(
                z[m], None if prev_obj is None else prev_obj[hit],
                eps_depth, min_samples,
            )

        tau = visible / kappa
        new = ((visible > 0) & (votes > 0) & (votes >= tau)).astype(np.int8)
        conf = (np.abs(votes - tau) / np.maximum(visible, 1)).clip(0.0, 1.0)
        new = _knn_refinement(
            new, conf, neighbors,
            knn_threshold=knn_threshold, confidence_gate=confidence_gate,
        )

        changed = n if labels is None else int((new != labels).sum())
        labels = new
        print(
            f"  [Iter {it}] object={int(labels.sum())} | "
            f"never visible={int((visible == 0).sum())} | changed={changed}"
        )

        # Feedback: Gaussians that fall inside masks but were rejected by the
        # depth gate (floaters, ghosts) no longer occlude in the next pass.
        rejected = (labels == 0) & (in_mask > 0) & (votes < 0.5 * in_mask)
        occ = base_occ & ~rejected

        if it > 1 and changed < n * 0.001:
            print("  [Converged] < 0.1 % changed — stopping early.")
            break

    return labels.astype(bool), dict(votes=votes, visible=visible, in_mask=in_mask)


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
    kappa: float = 1.5,
    eps_depth: float = 0.02,
    min_samples: int = 10,
    visibility_tol: float = 0.02,
    zbuf_cell: int = 4,
    occluder_min_alpha: float = 0.1,
    max_iter: int = 3,
    knn_k: int = 15,
    knn_threshold: float = 0.6,
    save_background: bool = False,
) -> str:
    """
    Lift 2D SAM masks into 3D and save the segmented point cloud.

    Parameters
    ----------
    ply_path            Path to the full 3DGS point cloud.
    cameras_json        Path to cameras.json.
    images_dir          Directory containing the training images.
    mask_dir            Directory containing the JSON mask files.
    out_dir             Output directory for the segmented PLY files.
    object_name         Stem for the output file (e.g. "hammer" → hammer.ply).
    kappa               Consensus divisor: object iff votes >= |V'_i| / kappa.
    eps_depth           DBSCAN eps on per-view depths (metres).
    min_samples         DBSCAN min_samples for depth clustering.
    visibility_tol      Depth slack for the z-buffer visibility test (metres).
    zbuf_cell           Z-buffer cell size in pixels.
    occluder_min_alpha  Gaussians below this opacity never occlude.
    max_iter            Depth-feedback iterations.
    knn_k               Number of neighbours for KNN refinement.
    knn_threshold       Neighbourhood consistency threshold for label update.
    save_background     Whether to also save the background point cloud.

    Returns
    -------
    Path to the saved object PLY file.
    """
    pts_all, fields, cameras, masks, occluders = load_scene(
        ply_path, cameras_json, images_dir, mask_dir, occluder_min_alpha
    )

    print(f"\n[Fusion] kappa={kappa}, eps_depth={eps_depth}, min_samples={min_samples}")
    obj_mask, _ = lift_labels(
        pts_all[:, :3], cameras, masks, occluders,
        kappa=kappa,
        eps_depth=eps_depth,
        min_samples=min_samples,
        visibility_tol=visibility_tol,
        zbuf_cell=zbuf_cell,
        max_iter=max_iter,
        knn_k=knn_k,
        knn_threshold=knn_threshold,
    )

    obj_pts = pts_all[obj_mask]
    bg_pts = pts_all[~obj_mask]
    print(f"[INFO] Final: object={len(obj_pts)} | background={len(bg_pts)}")

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
    """Configurable wrapper around fuse_masks_into_3d (kwargs as there)."""

    def __init__(self, **params):
        self.params = params

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
            **self.params,
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    parser = argparse.ArgumentParser(
        description="Lift 2D SAM masks into 3D via visibility-aware, depth-gated consensus."
    )
    parser.add_argument("--ply", required=True, help="Input 3DGS point cloud (.ply)")
    parser.add_argument("--cameras_json", required=True, help="cameras.json path")
    parser.add_argument("--images_dir", required=True, help="Training images folder")
    parser.add_argument("--mask_dir", required=True, help="JSON mask folder")
    parser.add_argument("--out_dir", required=True, help="Output folder")
    parser.add_argument("--object_name", default="object", help="Output file stem")
    parser.add_argument(
        "--kappa", type=float, default=1.5,
        help="Object iff weighted votes >= visible views / kappa (default: 1.5)"
    )
    parser.add_argument("--eps_depth", type=float, default=0.02, help="Depth DBSCAN eps (m)")
    parser.add_argument("--min_samples", type=int, default=10, help="Depth DBSCAN min_samples")
    parser.add_argument("--visibility_tol", type=float, default=0.02, help="Z-buffer slack (m)")
    parser.add_argument("--zbuf_cell", type=int, default=4, help="Z-buffer cell size (px)")
    parser.add_argument(
        "--occluder_min_alpha", type=float, default=0.1,
        help="Gaussians below this opacity never occlude"
    )
    parser.add_argument("--max_iter", type=int, default=3, help="Depth-feedback iterations")
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
        kappa=args.kappa,
        eps_depth=args.eps_depth,
        min_samples=args.min_samples,
        visibility_tol=args.visibility_tol,
        zbuf_cell=args.zbuf_cell,
        occluder_min_alpha=args.occluder_min_alpha,
        max_iter=args.max_iter,
        knn_k=args.knn_k,
        knn_threshold=args.knn_threshold,
        save_background=args.save_background,
    )

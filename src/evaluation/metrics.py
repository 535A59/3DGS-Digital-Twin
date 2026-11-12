"""
Evaluation metrics (paper Sec. 4.1.3, Tables 4 and 5).

table4    Sweep the consensus divisor κ and report, per setting,
          3D Projection Consistency and Ghost Index.
geometry  Chamfer Distance / Precision / Recall / F1 of a refined point cloud
          against a manually cleaned ground-truth cloud.
miou      2D IoU between projected 3D object points and annotated GT masks.

Definitions
-----------
Consistency(p)  = N_fg(p) / N(p), with N(p) the views where p projects inside
                  the image (no occlusion test) and N_fg(p) those whose pixel
                  is in the mask.  Reported: % of evaluated points with
                  N(p) > 0 and Consistency(p) >= 0.8.  Evaluated on the
                  foreground points retained at each κ.
Ghost Index(κ)  = residue of the object left in the background after
                  segmentation.  Per view, z-buffer all points per pixel; among
                  mask pixels hit by a point, the % whose front-most point is
                  NOT foreground at κ.  Averaged over views.  A strict κ drops
                  object points into the background, where they remain "in
                  place" as a ghost of the object (cf. the residual shadow in
                  background renders).  Points only, no rasterisation.
Artefact rate   (needs --gt) % of foreground points farther than δ from the
                  manually cleaned GT cloud — the GT-grounded ghost measure.
GT recall       (needs --gt) % of GT points with a foreground point within δ.

Usage
-----
    python -m src.evaluation.metrics table4 \\
        --ply point_cloud.ply --cameras_json cameras.json \\
        --images_dir images/ --mask_dir masks_json/ \\
        --kappas 2.0,1.8,1.5,1.2,1.0

    python -m src.evaluation.metrics geometry \\
        --pred refined/hammer_clean.ply --gt gt/hammer_gt.ply

    python -m src.evaluation.metrics miou \\
        --scene_ply point_cloud.ply --object_ply segmented/hammer.ply \\
        --cameras_json cameras.json --images_dir images/ --gt_mask_dir gt_masks_json/
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np
from scipy.spatial import cKDTree

from src.semantic_fusion.fusion import (
    in_view, lift_labels, load_cameras, load_masks_from_json, load_scene,
    project, read_ply, visible_points,
)


# ---------------------------------------------------------------------------
# Table 4 — consistency / ghost index
# ---------------------------------------------------------------------------

def projection_consistency(
    pts_xyz: np.ndarray,
    cameras: list[dict],
    masks: list[np.ndarray],
    min_score: float = 0.8,
) -> float:
    """% of points with N(p) > 0 whose in-mask fraction is >= min_score."""
    n_valid = np.zeros(len(pts_xyz), dtype=np.int32)
    n_fg = np.zeros(len(pts_xyz), dtype=np.int32)
    for cam, mask in zip(cameras, masks):
        uv, z = project(pts_xyz, cam["K"], cam["RT"])
        idx = np.flatnonzero(in_view(uv, z, cam["shape"]))
        n_valid[idx] += 1
        n_fg[idx] += mask[uv[idx, 1], uv[idx, 0]]
    valid = n_valid > 0
    if not valid.any():
        return float("nan")
    return 100.0 * float(np.mean(n_fg[valid] / n_valid[valid] >= min_score))


def ghost_index(
    pts_xyz: np.ndarray,
    fg: np.ndarray,
    cameras: list[dict],
    masks: list[np.ndarray],
) -> float:
    """% of in-mask pixels whose front-most point is background (view mean)."""
    per_view = []
    for cam, mask in zip(cameras, masks):
        uv, z = project(pts_xyz, cam["K"], cam["RT"])
        idx = np.flatnonzero(in_view(uv, z, cam["shape"]))
        idx = idx[mask[uv[idx, 1], uv[idx, 0]]]
        if not len(idx):
            continue
        pix = uv[idx, 1] * cam["shape"][1] + uv[idx, 0]
        order = np.lexsort((z[idx], pix))
        first = np.r_[True, pix[order][1:] != pix[order][:-1]]
        per_view.append(100.0 * float(np.mean(~fg[idx[order][first]])))
    return float(np.mean(per_view)) if per_view else float("nan")


def table4(
    ply_path: str,
    cameras_json: str,
    images_dir: str,
    mask_dir: str,
    kappas: list[float],
    gt_xyz: np.ndarray | None = None,
    delta: float = 0.005,
    **fusion_kwargs,
) -> list[dict]:
    pts_all, _, cameras, masks, occluders = load_scene(
        ply_path, cameras_json, images_dir, mask_dir,
        fusion_kwargs.pop("occluder_min_alpha", 0.1),
    )
    xyz = pts_all[:, :3]

    def run(k):
        print(f"\n[Table4] kappa={k}")
        return lift_labels(xyz, cameras, masks, occluders, kappa=k, **fusion_kwargs)[0]

    gt_tree = None if gt_xyz is None else cKDTree(gt_xyz)
    rows = []
    for k in kappas:
        fg = run(k)
        rows.append(dict(
            kappa=k,
            n_fg=int(fg.sum()),
            n_total=len(xyz),
            consistency=projection_consistency(xyz[fg], cameras, masks),
            ghost_index=ghost_index(xyz, fg, cameras, masks),
        ))
        if gt_tree is not None:
            d_fg, _ = gt_tree.query(xyz[fg])
            d_gt, _ = cKDTree(xyz[fg]).query(gt_xyz)
            rows[-1]["artefact_rate"] = 100.0 * float(np.mean(d_fg > delta))
            rows[-1]["gt_recall"] = 100.0 * float(np.mean(d_gt <= delta))
    return rows


# ---------------------------------------------------------------------------
# Table 5 — geometric fidelity
# ---------------------------------------------------------------------------

def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    _, keep = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
    return pts[np.sort(keep)]


def geometry_metrics(
    pred: np.ndarray,
    gt: np.ndarray,
    threshold: float = 0.005,
    voxel: float = 0.001,
) -> dict:
    """
    Chamfer = ½ (mean d(pred→gt) + mean d(gt→pred)), both clouds voxelised
    at ``voxel``.  Precision / recall = fraction of pred / gt points within
    ``threshold`` of the other cloud; F1 is their harmonic mean.
    """
    pred, gt = voxel_downsample(pred, voxel), voxel_downsample(gt, voxel)
    d_pg, _ = cKDTree(gt).query(pred)
    d_gp, _ = cKDTree(pred).query(gt)
    precision = float(np.mean(d_pg < threshold))
    recall = float(np.mean(d_gp < threshold))
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return dict(
        chamfer=0.5 * float(d_pg.mean() + d_gp.mean()),
        precision=precision, recall=recall, f1=f1,
    )


# ---------------------------------------------------------------------------
# 2D mIoU of projected 3D segmentation
# ---------------------------------------------------------------------------

def projected_miou(
    scene_xyz: np.ndarray,
    object_xyz: np.ndarray,
    cameras: list[dict],
    gt_masks: list[np.ndarray],
    *,
    close_kernel: int = 5,
    visibility_tol: float = 0.02,
    zbuf_cell: int = 4,
) -> float:
    """
    Rasterise the visible object Gaussians per view (z-buffer from the whole
    scene), close holes with a ``close_kernel`` morphological closing, and
    average IoU against the GT masks.
    """
    xyz = np.vstack([scene_xyz, object_xyz])
    is_obj = np.r_[np.zeros(len(scene_xyz), bool), np.ones(len(object_xyz), bool)]
    kernel = np.ones((close_kernel, close_kernel), np.uint8)
    ious = []
    for cam, gt in zip(cameras, gt_masks):
        idx, uv, _ = visible_points(
            xyz, cam, np.ones(len(xyz), bool),
            visibility_tol=visibility_tol, zbuf_cell=zbuf_cell,
        )
        pred = np.zeros(cam["shape"], np.uint8)
        o = is_obj[idx]
        pred[uv[o, 1], uv[o, 0]] = 1
        pred = cv2.morphologyEx(pred, cv2.MORPH_CLOSE, kernel).astype(bool)
        union = (pred | gt).sum()
        if union:
            ious.append((pred & gt).sum() / union)
    return float(np.mean(ious))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    p = argparse.ArgumentParser(description="Evaluation metrics for Tables 4 and 5.")
    sub = p.add_subparsers(dest="mode", required=True)

    t4 = sub.add_parser("table4", help="Consistency / Ghost Index sweep over kappa.")
    t4.add_argument("--ply", required=True)
    t4.add_argument("--cameras_json", required=True)
    t4.add_argument("--images_dir", required=True)
    t4.add_argument("--mask_dir", required=True)
    t4.add_argument("--kappas", default="2.0,1.8,1.5,1.2,1.0")
    t4.add_argument("--max_iter", type=int, default=3)
    t4.add_argument("--gt", help="Manually cleaned GT cloud (.ply) → artefact rate + GT recall")
    t4.add_argument("--delta", type=float, default=0.005, help="GT match distance (m)")

    geo = sub.add_parser("geometry", help="Chamfer / Precision / F1 vs ground truth.")
    geo.add_argument("--pred", required=True, help="Refined point cloud (.ply)")
    geo.add_argument("--gt", required=True, help="Manually cleaned GT cloud (.ply)")
    geo.add_argument("--threshold", type=float, default=0.005, help="F-score distance (m)")
    geo.add_argument("--voxel", type=float, default=0.001, help="Voxel size (m)")

    mi = sub.add_parser("miou", help="2D IoU of projected 3D segmentation.")
    mi.add_argument("--scene_ply", required=True)
    mi.add_argument("--object_ply", required=True)
    mi.add_argument("--cameras_json", required=True)
    mi.add_argument("--images_dir", required=True)
    mi.add_argument("--gt_mask_dir", required=True)
    mi.add_argument("--close_kernel", type=int, default=5)
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    if args.mode == "table4":
        rows = table4(
            args.ply, args.cameras_json, args.images_dir, args.mask_dir,
            [float(k) for k in args.kappas.split(",")],
            max_iter=args.max_iter,
            gt_xyz=None if args.gt is None else read_ply(args.gt)[0][:, :3],
            delta=args.delta,
        )
        gt_cols = f" {'Artefact%':>10} {'GTRecall%':>10}" if args.gt else ""
        print(f"\n{'threshold':>10} {'|FG|':>9} {'Consistency%':>13} {'GhostIndex%':>12}{gt_cols}")
        for r in rows:
            extra = (f" {r['artefact_rate']:>10.2f} {r['gt_recall']:>10.2f}"
                     if args.gt else "")
            print(f"{'N/' + str(r['kappa']):>10} {r['n_fg']:>9} "
                  f"{r['consistency']:>13.2f} {r['ghost_index']:>12.2f}{extra}")
        print(f"(N_total={rows[0]['n_total']})")
    elif args.mode == "geometry":
        m = geometry_metrics(
            read_ply(args.pred)[0][:, :3], read_ply(args.gt)[0][:, :3],
            threshold=args.threshold, voxel=args.voxel,
        )
        print(" | ".join(f"{k}={v:.4f}" for k, v in m.items()))
    else:
        cams = load_cameras(args.cameras_json, args.images_dir)
        miou = projected_miou(
            read_ply(args.scene_ply)[0][:, :3], read_ply(args.object_ply)[0][:, :3],
            cams, load_masks_from_json(cams, args.gt_mask_dir),
            close_kernel=args.close_kernel,
        )
        print(f"mIoU={miou:.4f}")

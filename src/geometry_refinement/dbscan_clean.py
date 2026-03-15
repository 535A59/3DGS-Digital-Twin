"""
Stage 2 — Semantic-Guided Connectivity Pruning via DBSCAN.

Two modes
---------
dbscan_clean  — Keep only the single largest connected cluster (removes
                outlier patches and floating noise).
dbscan_split  — Save every cluster as an individual PLY file.
                Useful when the scene contains multiple distinct objects.

Usage
-----
    # Clean (keep largest cluster):
    python -m src.geometry_refinement.dbscan_clean clean \\
        --input  /path/to/object.ply \\
        --output /path/to/object_clean.ply \\
        --eps 0.01 --min_samples 50

    # Split (save each cluster):
    python -m src.geometry_refinement.dbscan_clean split \\
        --input     /path/to/object.ply \\
        --out_dir   /path/to/output/ \\
        --eps 0.05 --min_samples 50
"""

from __future__ import annotations

import argparse
import os
import sys
from collections import Counter

import numpy as np
from plyfile import PlyData, PlyElement
from sklearn.cluster import DBSCAN


# ---------------------------------------------------------------------------
# PLY helpers
# ---------------------------------------------------------------------------

def _read_ply(path: str):
    data = PlyData.read(path)
    vertices = data["vertex"].data
    return data, vertices


def _write_ply(output_path: str, vertices, plydata):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    PlyData(
        [PlyElement.describe(vertices, "vertex")], text=plydata.text
    ).write(output_path)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def dbscan_clean(
    input_path: str,
    output_path: str,
    eps: float = 0.01,
    min_samples: int = 50,
) -> int:
    """
    Keep only the largest DBSCAN cluster; discard all others and noise.

    Parameters
    ----------
    input_path   Input PLY file.
    output_path  Output PLY file.
    eps          DBSCAN neighbourhood radius.
    min_samples  Minimum neighbours to form a core point.

    Returns
    -------
    Number of points kept.
    """
    print(f"[DBSCAN-Clean] Loading: {input_path}")
    plydata, vertices = _read_ply(input_path)
    n_original = len(vertices)
    print(f"[DBSCAN-Clean] Points: {n_original} | eps={eps} | min_samples={min_samples}")

    positions = np.vstack(
        [vertices["x"], vertices["y"], vertices["z"]]
    ).T.astype(np.float64)

    print("[DBSCAN-Clean] Running DBSCAN (n_jobs=-1) ...")
    labels = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=-1).fit_predict(positions)

    label_counts = Counter(labels)
    noise_count = label_counts.pop(-1, 0)
    n_clusters = len(label_counts)

    print(
        f"[DBSCAN-Clean] Clusters found: {n_clusters} | noise: {noise_count}"
    )

    if n_clusters == 0:
        raise RuntimeError(
            "No clusters found. Try increasing eps or decreasing min_samples."
        )

    main_label = max(label_counts, key=label_counts.get)
    cleaned = vertices[labels == main_label]
    n_kept = len(cleaned)

    _write_ply(output_path, cleaned, plydata)
    print(
        f"[DBSCAN-Clean] Kept cluster #{main_label} ({n_kept} pts) → {output_path}"
    )
    return n_kept


def dbscan_split(
    input_path: str,
    out_dir: str,
    eps: float = 0.05,
    min_samples: int = 50,
    save_noise: bool = True,
) -> int:
    """
    Split a point cloud into per-cluster PLY files using DBSCAN.

    Parameters
    ----------
    input_path   Input PLY file.
    out_dir      Output directory; files are named object1.ply, object2.ply, …
    eps          DBSCAN neighbourhood radius.
    min_samples  Minimum neighbours for a core point.
    save_noise   Whether to write a noise.ply for outlier points.

    Returns
    -------
    Number of clusters saved.
    """
    print(f"[DBSCAN-Split] Loading: {input_path}")
    plydata, vertices = _read_ply(input_path)
    n_original = len(vertices)
    print(f"[DBSCAN-Split] Points: {n_original} | eps={eps} | min_samples={min_samples}")

    positions = np.vstack(
        [vertices["x"], vertices["y"], vertices["z"]]
    ).T.astype(np.float64)

    # We need a full numpy array for indexing across all fields
    fields = vertices.dtype.names
    pts_array = np.stack([vertices[f] for f in fields], axis=-1).astype(np.float32)

    print("[DBSCAN-Split] Running DBSCAN (n_jobs=-1) ...")
    labels = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=-1).fit_predict(positions)

    unique_labels = set(labels)
    n_clusters = len(unique_labels) - (1 if -1 in unique_labels else 0)
    n_noise = int(np.sum(labels == -1))

    print(
        f"[DBSCAN-Split] Clusters: {n_clusters} | noise: {n_noise}"
    )

    if n_clusters == 0:
        print("[WARN] No clusters found — all points are noise.")
        if save_noise and n_noise > 0:
            _save_array(
                pts_array, fields,
                os.path.join(out_dir, "noise.ply"),
                plydata,
            )
        return 0

    os.makedirs(out_dir, exist_ok=True)
    saved = 0
    for lbl in sorted(unique_labels):
        if lbl == -1:
            if save_noise and n_noise > 0:
                _save_array(
                    pts_array[labels == -1], fields,
                    os.path.join(out_dir, "noise.ply"),
                    plydata,
                )
            continue

        cluster_pts = pts_array[labels == lbl]
        out_path = os.path.join(out_dir, f"object{lbl + 1}.ply")
        _save_array(cluster_pts, fields, out_path, plydata)
        print(f"  Cluster {lbl + 1}: {len(cluster_pts)} pts → {out_path}")
        saved += 1

    print(f"[DBSCAN-Split] {saved} clusters saved to {out_dir}")
    return saved


def _save_array(pts: np.ndarray, fields, path: str, plydata):
    """Write a float32 array back to PLY using the original field layout."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    dtype = [(f, "<f4") for f in fields]
    arr = np.empty(len(pts), dtype=dtype)
    for i, f in enumerate(fields):
        arr[f] = pts[:, i]
    PlyData([PlyElement.describe(arr, "vertex")], text=plydata.text).write(path)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    p = argparse.ArgumentParser(
        description="DBSCAN-based point cloud cleaning and splitting."
    )
    sub = p.add_subparsers(dest="mode", required=True)

    clean = sub.add_parser("clean", help="Keep only the largest cluster.")
    clean.add_argument("--input", required=True)
    clean.add_argument("--output", required=True)
    clean.add_argument("--eps", type=float, default=0.01)
    clean.add_argument("--min_samples", type=int, default=50)

    split = sub.add_parser("split", help="Save each cluster as a separate PLY.")
    split.add_argument("--input", required=True)
    split.add_argument("--out_dir", required=True)
    split.add_argument("--eps", type=float, default=0.05)
    split.add_argument("--min_samples", type=int, default=50)
    split.add_argument("--no_noise", action="store_true", help="Discard noise points")

    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    if args.mode == "clean":
        dbscan_clean(args.input, args.output, args.eps, args.min_samples)
    else:
        dbscan_split(
            args.input, args.out_dir, args.eps, args.min_samples,
            save_noise=not args.no_noise
        )

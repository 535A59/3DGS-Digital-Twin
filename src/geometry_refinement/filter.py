"""
Stage 1 — Intrinsic Attribute Filtering.

Removes ghost / floater Gaussians by thresholding their physical
opacity (alpha) and geometric scale (max axis length).

Usage
-----
    python -m src.geometry_refinement.filter \\
        --input  /path/to/object.ply \\
        --output /path/to/object_filtered.ply \\
        --min_alpha 0.05 \\
        --min_scale 1e-5 \\
        --max_scale 0.007
"""

from __future__ import annotations

import argparse
import os

import numpy as np
from plyfile import PlyData, PlyElement


def filter_gaussians(
    input_path: str,
    output_path: str,
    min_alpha: float = 0.05,
    min_scale: float = 1e-5,
    max_scale: float = 0.007,
) -> int:
    """
    Filter a 3DGS PLY file by opacity (alpha) and geometric scale.

    Parameters
    ----------
    input_path  Path to input 3DGS PLY file.
    output_path Path to write the filtered PLY file.
    min_alpha   Discard points with sigmoid(opacity) < min_alpha.
    min_scale   Discard points with max-axis scale < min_scale (noise).
    max_scale   Discard points with max-axis scale > max_scale (ghosts).

    Returns
    -------
    Number of points kept.
    """
    print(f"[Filter] Loading: {input_path}")
    plydata = PlyData.read(input_path)
    vertices = plydata["vertex"].data
    n_original = len(vertices)
    print(f"[Filter] Original point count: {n_original}")

    required = {"opacity", "scale_0", "scale_1", "scale_2"}
    if not required.issubset(vertices.dtype.names):
        raise ValueError(
            f"PLY file missing required 3DGS attributes. "
            f"Found: {vertices.dtype.names}"
        )

    # Physical opacity via sigmoid
    alphas = 1.0 / (1.0 + np.exp(-vertices["opacity"].astype(np.float64)))

    # Physical scale: exp(log_scale) for each axis, take the max
    scales = np.exp(
        np.stack(
            [vertices["scale_0"], vertices["scale_1"], vertices["scale_2"]], axis=1
        ).astype(np.float64)
    )
    max_scales = scales.max(axis=1)

    print(
        f"[Filter] Alpha range  : [{alphas.min():.4f}, {alphas.max():.4f}]\n"
        f"[Filter] Scale range  : [{max_scales.min():.6f}, {max_scales.max():.6f}]"
    )
    print(
        f"[Filter] Thresholds   : alpha > {min_alpha} | "
        f"{min_scale} < scale < {max_scale}"
    )

    mask = (alphas > min_alpha) & (max_scales > min_scale) & (max_scales < max_scale)
    cleaned = vertices[mask]
    n_kept = len(cleaned)

    if n_kept == 0:
        raise RuntimeError(
            "All points were filtered out — try relaxing the thresholds."
        )

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    PlyData(
        [PlyElement.describe(cleaned, "vertex")], text=plydata.text
    ).write(output_path)

    removed = n_original - n_kept
    print(
        f"[Filter] Removed {removed} / {n_original} points "
        f"({removed / n_original:.1%})\n"
        f"[Filter] Saved {n_kept} points → {output_path}"
    )
    return n_kept


def _parse_args():
    p = argparse.ArgumentParser(description="Filter 3DGS Gaussians by opacity and scale.")
    p.add_argument("--input", required=True, help="Input PLY path")
    p.add_argument("--output", required=True, help="Output PLY path")
    p.add_argument("--min_alpha", type=float, default=0.05)
    p.add_argument("--min_scale", type=float, default=1e-5)
    p.add_argument("--max_scale", type=float, default=0.007)
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    filter_gaussians(args.input, args.output, args.min_alpha, args.min_scale, args.max_scale)

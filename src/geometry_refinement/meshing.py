"""
Stage 3 — Watertight Meshing via Alpha Shapes.

Converts a filtered 3DGS point cloud into a watertight triangle mesh
suitable for use as a physics collision primitive in Unity / MoveIt 2.

Pipeline inside this module
---------------------------
1. Read PLY, decode SH DC coefficients → RGB colours.
2. Opacity filter + white-background removal.
3. Statistical outlier removal (Open3D).
4. Optional: centre point cloud at origin.
5. Optional: apply coordinate-system transform (3D-model → ROS convention).
6. Alpha Shapes → TriangleMesh.
7. Save as .obj / .ply / .stl (format inferred from extension).

Usage
-----
    python -m src.geometry_refinement.meshing \\
        --input  /path/to/object_clean.ply \\
        --output /path/to/object.obj \\
        --alpha  0.01
"""

from __future__ import annotations

import argparse
import os

import numpy as np

try:
    import open3d as o3d
    O3D_AVAILABLE = True
except ImportError:
    O3D_AVAILABLE = False

from plyfile import PlyData


def gaussians_to_mesh(
    input_ply_path: str,
    output_mesh_path: str,
    *,
    min_opacity: float = 0.02,
    white_threshold: float = 1.0,
    alpha_value: float = 0.01,
    center_the_cloud: bool = True,
    transform_coords: bool = False,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0,
) -> None:
    """
    Convert a 3DGS PLY file into a watertight triangle mesh.

    Parameters
    ----------
    input_ply_path    Input 3DGS PLY (must contain x/y/z, opacity, f_dc_*).
    output_mesh_path  Output mesh file (.obj / .ply / .stl).
    min_opacity       Discard Gaussians with sigmoid(opacity) < this value.
    white_threshold   Discard Gaussians whose RGB colour channels are all
                      above this value (background removal).
    alpha_value       Alpha radius for the Alpha Shapes algorithm.
                      Smaller → tighter / more detailed mesh.
                      Larger  → smoother / fewer holes.
    center_the_cloud  Translate the cloud so its geometric centre = origin.
    transform_coords  Apply model→ROS coordinate convention:
                        Z → X,  X → -Y,  Y → Z
    nb_neighbors      Statistical outlier removal: number of neighbours.
    std_ratio         Statistical outlier removal: standard deviation ratio.
    """
    if not O3D_AVAILABLE:
        raise ImportError(
            "open3d is required for meshing. Run: pip install open3d"
        )

    print(f"[Mesh] Processing: {input_ply_path}")

    if not os.path.exists(input_ply_path):
        raise FileNotFoundError(f"Input file not found: {input_ply_path}")

    # --- 1. Load 3DGS data ---
    plydata = PlyData.read(input_ply_path)
    vert = plydata["vertex"]

    points = np.vstack([vert["x"], vert["y"], vert["z"]]).T
    opacities = 1.0 / (1.0 + np.exp(-vert["opacity"]))

    # --- 2. Decode SH DC term → colour ---
    sh_c0 = 0.28209479177387814
    f_dc = np.vstack([vert["f_dc_0"], vert["f_dc_1"], vert["f_dc_2"]]).T
    colors = np.clip(0.5 + sh_c0 * f_dc, 0.0, 1.0)

    print(f"[Mesh] Loaded {len(points)} Gaussians.")

    # --- 3. Opacity filter ---
    mask = opacities > min_opacity
    print(f"[Mesh] After opacity filter  : {mask.sum()} pts")

    # --- 4. White-background removal ---
    white = np.all(colors[mask] > white_threshold, axis=1)
    valid_idx = np.where(mask)[0]
    mask[valid_idx[white]] = False
    print(f"[Mesh] After colour filter   : {mask.sum()} pts")

    if mask.sum() < 100:
        raise RuntimeError(
            f"Too few points ({mask.sum()}) after filtering — "
            "try relaxing min_opacity or white_threshold."
        )

    # --- 5. Build Open3D point cloud ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[mask])
    pcd.colors = o3d.utility.Vector3dVector(colors[mask])

    # --- 6. Statistical outlier removal ---
    _cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pcd = pcd.select_by_index(ind)
    print(f"[Mesh] After outlier removal : {len(pcd.points)} pts")

    # --- 7. Centre ---
    if center_the_cloud:
        centre = pcd.get_center()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) - centre)
        print(f"[Mesh] Centred at {centre}")

    # --- 8. Coordinate transform (model → ROS) ---
    if transform_coords:
        orig = np.asarray(pcd.points)
        transformed = np.zeros_like(orig)
        transformed[:, 0] = orig[:, 2]    # Z → X
        transformed[:, 1] = -orig[:, 0]   # X → -Y
        transformed[:, 2] = orig[:, 1]    # Y → Z
        pcd.points = o3d.utility.Vector3dVector(transformed)
        print("[Mesh] Coordinate transform applied (model → ROS).")

    # --- 9. Alpha Shapes → mesh ---
    print(f"[Mesh] Computing Alpha Shape (alpha={alpha_value}) ...")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        pcd, alpha_value
    )

    if mesh.is_empty():
        raise RuntimeError(
            "Alpha Shape produced an empty mesh. "
            "Try a larger alpha value or denser point cloud."
        )

    mesh.compute_vertex_normals()
    print(
        f"[Mesh] Mesh: {len(mesh.vertices)} vertices, "
        f"{len(mesh.triangles)} triangles."
    )

    # --- 10. Save ---
    os.makedirs(os.path.dirname(os.path.abspath(output_mesh_path)), exist_ok=True)
    o3d.io.write_triangle_mesh(output_mesh_path, mesh, write_vertex_colors=True)
    print(f"[Mesh] Saved → {output_mesh_path}")


def _parse_args():
    p = argparse.ArgumentParser(
        description="Convert a 3DGS PLY to a watertight mesh via Alpha Shapes."
    )
    p.add_argument("--input", required=True, help="Input 3DGS PLY")
    p.add_argument("--output", required=True, help="Output mesh file (.obj/.ply/.stl)")
    p.add_argument("--alpha", type=float, default=0.01, help="Alpha radius")
    p.add_argument("--min_opacity", type=float, default=0.02)
    p.add_argument("--white_threshold", type=float, default=1.0)
    p.add_argument("--no_center", action="store_true", help="Disable centring")
    p.add_argument(
        "--transform_coords", action="store_true",
        help="Apply model→ROS coordinate transform"
    )
    p.add_argument("--nb_neighbors", type=int, default=20)
    p.add_argument("--std_ratio", type=float, default=2.0)
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    gaussians_to_mesh(
        input_ply_path=args.input,
        output_mesh_path=args.output,
        min_opacity=args.min_opacity,
        white_threshold=args.white_threshold,
        alpha_value=args.alpha,
        center_the_cloud=not args.no_center,
        transform_coords=args.transform_coords,
        nb_neighbors=args.nb_neighbors,
        std_ratio=args.std_ratio,
    )

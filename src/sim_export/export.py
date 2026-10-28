"""
Sim Asset Export for the Isaac Sim digital twin.

Turns the 3DGS reconstruction (object PLYs + the decoded Unity background)
into the asset folder read by the Isaac scene and the MTC controller:

    out_dir/
      manifest.json                         frames, poses, masses, cameras
      splats/{<objects>,background}.ply     3DGS PLYs in the SOURCE frame (untouched)
      collision/<object>.obj                collision mesh, BODY frame, metres
      collision/env_{table,cardboard}.obj   static colliders, WORLD frame, metres
      mtc_task.yaml                         ROS parameters for mtc_dynamic

Frames
------
World W = robot base (panda_link0), Z-up, metres.  T_W_src is a similarity

    p_W = s * R (p_src - p0)

with p0 / R from the Unity calibration (the robot base stays where Unity put
it), R tilted minimally so the RANSAC table normal is exactly +Z, and s the
metric scale from the config.  Splat PLYs are never modified (their SH would
have to be rotated) — transforms live on the USD prims.

Body frame of an object: origin = centre of mass (uniform density) of its
collider — the convex-hull centroid for convexHull, the mesh centroid for
convexDecomposition — in W at rest (splat bleed below the table cut, then
lifted so the lowest point sits ``lift`` above the table), axes world-aligned.
``T_body_from_source = inv(T_W_body) @ T_lift @ T_W_src`` is applied to the
collision vertices here and to the splats (one USD xformOp) in Isaac, so the
two line up by construction.

Collision meshes come from the repo refinement (filter → DBSCAN clean → alpha
shape) with ``center_the_cloud=False`` and ``transform_coords=False``.

Usage (from the repo root; needs ``old_code-data/3dgs_resource``)
-----
    python -m src.sim_export.export --config configs/twin_res5.yaml
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import tempfile

import cv2
import numpy as np
import open3d as o3d
import yaml
from plyfile import PlyData
from scipy.spatial import ConvexHull, Delaunay
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN

from src.geometry_refinement import dbscan_clean, filter_gaussians, gaussians_to_mesh
from tools.unity_gs_to_ply import decode_asset, write_ply

FRANKA_USD = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
LINK0 = {"extent_xy": (0.2256, 0.1893), "center_xy": (-0.0413, 0.0), "height": 0.14}  # link0 collision mesh


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def read_xyz(path: str, min_opacity: float = 0.0) -> np.ndarray:
    """Gaussian centres of a 3DGS PLY with sigmoid(opacity) > min_opacity."""
    v = PlyData.read(path)["vertex"].data
    keep = 1.0 / (1.0 + np.exp(-v["opacity"].astype(np.float64))) > min_opacity
    return np.c_[v["x"], v["y"], v["z"]].astype(np.float64)[keep]


def apply(T: np.ndarray, P: np.ndarray) -> np.ndarray:
    return P @ T[:3, :3].T + T[:3, 3]


def translation(t) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = t
    return T


def pose(p, R=np.eye(3)) -> dict:
    """Position + quaternion wxyz (manifest convention)."""
    q = Rotation.from_matrix(R).as_quat()                    # xyzw
    x, y, z, w = q if q[3] >= 0 else -q
    return {"p": [float(v) for v in p], "q": [float(w), float(x), float(y), float(z)]}


def hull_mesh(P: np.ndarray):
    """Convex hull of P -> (vertices, outward triangles, volume, centroid)."""
    h = ConvexHull(P)
    V = P[h.vertices]
    remap = np.full(len(P), -1)
    remap[h.vertices] = np.arange(len(V))
    F = remap[h.simplices]
    inner = V.mean(0)
    a, b, c = (V[F[:, i]] - inner for i in range(3))
    flip = np.einsum("ij,ij->i", np.cross(b - a, c - a), a) < 0
    F[flip] = F[flip][:, ::-1]
    a, b, c = (V[F[:, i]] - inner for i in range(3))
    tet = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    centroid = inner + ((a + b + c) / 4.0 * tet[:, None]).sum(0) / tet.sum()
    return V, F, tet.sum(), centroid


def mesh_volume(V: np.ndarray, F: np.ndarray) -> float:
    """|signed volume| — exact for closed meshes, approximate for small holes."""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    return abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0


def mesh_centroid(V: np.ndarray, F: np.ndarray) -> np.ndarray:
    """Volume centroid (uniform-density centre of mass) of a closed mesh."""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    tet = np.einsum("ij,ij->i", a, np.cross(b, c))
    return ((a + b + c) / 4.0 * tet[:, None]).sum(0) / tet.sum()


def write_obj(path: str, V: np.ndarray, F: np.ndarray) -> None:
    with open(path, "w") as f:
        f.writelines(f"v {x:.6f} {y:.6f} {z:.6f}\n" for x, y, z in V)
        f.writelines(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in F)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def fit_table(bg_src: np.ndarray, obj_src: np.ndarray, threshold: float, iters: int = 2000):
    """Seeded RANSAC + least-squares refit of the table plane n·p + d = 0 (source
    frame), n towards the objects.  Returns (n, d, inlier mask)."""
    rng = np.random.default_rng(0)
    S = bg_src[rng.choice(len(bg_src), min(len(bg_src), 50000), replace=False)]
    best = (-1, None, None)
    for a, b, c in S[rng.choice(len(S), (iters, 3))]:
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) > 1e-12:
            n /= np.linalg.norm(n)
            cnt = np.count_nonzero(np.abs((S - a) @ n) < threshold)
            best = max(best, (cnt, n, a), key=lambda t: t[0])
    _, n, a = best
    inliers = np.abs((bg_src - a) @ n) < threshold
    Q = bg_src[inliers]
    n = np.linalg.svd(Q - Q.mean(0), full_matrices=False)[2][2]
    d = -n @ Q.mean(0)
    if np.median(obj_src @ n + d) < 0:
        n, d = -n, -d
    return n, d, inliers


def world_from_source(T_unity: np.ndarray, n_src: np.ndarray, s: float):
    """T_W_src (p_W = s R (p - p0)) and the tilt (deg) added to Unity's rotation."""
    s_u = np.cbrt(np.linalg.det(T_unity[:3, :3]))
    assert s_u > 0, "Unity calibration must be a proper similarity (det > 0)"
    R_u = T_unity[:3, :3] / s_u
    p0 = np.linalg.solve(T_unity[:3, :3], -T_unity[:3, 3])   # robot base in the source frame
    m = R_u @ n_src
    axis = np.cross(m, [0.0, 0.0, 1.0])
    angle = np.arctan2(np.linalg.norm(axis), m[2])
    tilt = Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle if angle > 1e-12 else np.zeros(3))
    R = tilt.as_matrix() @ R_u
    T = np.eye(4)
    T[:3, :3] = s * R
    T[:3, 3] = -s * R @ p0
    return T, float(np.degrees(angle))


def usd_camera(forward, up) -> np.ndarray:
    """Rotation of a USD camera (-Z forward, +Y up) looking along ``forward``."""
    f = np.asarray(forward, float) / np.linalg.norm(forward)
    u = np.asarray(up, float) - (np.dot(up, f)) * f
    u /= np.linalg.norm(u)
    return np.c_[np.cross(f, u), u, -f]


# ---------------------------------------------------------------------------
# Static colliders (WORLD frame)
# ---------------------------------------------------------------------------

def table_collider(inliers_W: np.ndarray, z_table: float, thickness: float):
    """Box under the table top, fitted to the main cluster of the plane inliers."""
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(
        np.c_[inliers_W[:, :2], np.zeros(len(inliers_W))])).voxel_down_sample(0.01)
    xy = np.asarray(pc.points)[:, :2]
    lab = DBSCAN(eps=0.03, min_samples=5).fit_predict(xy)   # drop other coplanar desks
    xy = xy[lab == np.bincount(lab[lab >= 0]).argmax()]
    corners = cv2.boxPoints(cv2.minAreaRect(xy.astype(np.float32))).astype(np.float64)
    box = np.vstack([np.c_[corners, np.full(4, z_table)],
                     np.c_[corners, np.full(4, z_table - thickness)]])
    V, F, _, _ = hull_mesh(box)
    return V, F


def cardboard_collider(bg_W: np.ndarray, z_table: float, near_xy, cc: dict):
    """Convex hull (down to the table) of the splat cluster nearest ``near_xy``."""
    lo, hi = cc["z_band"]
    P = bg_W[(bg_W[:, 2] > z_table + lo) & (bg_W[:, 2] < z_table + hi)
             & (np.linalg.norm(bg_W[:, :2] - near_xy, axis=1) < cc["search_radius"])]
    lab = DBSCAN(eps=cc["dbscan_eps"], min_samples=cc["dbscan_min_samples"]).fit_predict(P)
    best = min(set(lab) - {-1}, key=lambda l: np.linalg.norm(P[lab == l, :2].mean(0) - near_xy))
    C = P[lab == best]
    top = np.percentile(C[:, 2], cc["top_percentile"])
    C = C[C[:, 2] <= top]
    # one lid plane: PhysX contacts a mosaic of near-coplanar hull facets through a single small facet, and an
    # object placed on it rocks / topples from ~1 deg
    C[C[:, 2] > top - cc.get("flat_top", 0.0), 2] = top
    V, F, _, _ = hull_mesh(np.vstack([C, np.c_[C[:, :2], np.full(len(C), z_table)]]))
    print(f"[Export] Cardboard: {len(C)} splats, top {top - z_table:.3f} m above the table")
    return V, F, float(top)


# ---------------------------------------------------------------------------
# Objects (BODY frame)
# ---------------------------------------------------------------------------

def object_mesh(ply: str, mc: dict, s: float, tmp_dir: str):
    """Repo refinement (filter → DBSCAN clean → alpha shape) in the SOURCE frame;
    no centring and no mirror, so the mesh stays aligned with the splats."""
    stem = os.path.join(tmp_dir, os.path.splitext(os.path.basename(ply))[0])
    filter_gaussians(ply, stem + "_filtered.ply", min_alpha=mc["min_alpha"], max_scale=mc["max_scale"] / s)
    dbscan_clean(stem + "_filtered.ply", stem + "_clean.ply",
                 eps=mc["dbscan_eps"] / s, min_samples=mc["dbscan_min_samples"])
    gaussians_to_mesh(stem + "_clean.ply", stem + ".obj", min_opacity=mc["min_alpha"],
                      white_threshold=mc["white_threshold"], alpha_value=mc["alpha"] / s,
                      nb_neighbors=mc["nb_neighbors"], std_ratio=mc["std_ratio"],
                      center_the_cloud=False, transform_coords=False)
    m = o3d.io.read_triangle_mesh(stem + ".obj")
    return np.asarray(m.vertices, np.float64), np.asarray(m.triangles)


def body_from_mesh(V_src, F, T_ws, z_support, mc, approximation=None, flat_bottom=0.0):
    """Rest on the support, choose the collider, and express it in the body frame.
    ``flat_bottom`` > 0 extrudes the lowest band of that height down to the base, so a
    rounded / tilted reconstructed bottom stands upright (centre of mass over the base)."""
    V = apply(T_ws, V_src)
    V[:, 2] = np.maximum(V[:, 2], z_support)                 # cut splat bleed below the table
    lift = z_support + mc["lift"] - V[:, 2].min()
    V[:, 2] += lift
    V[V[:, 2] < V[:, 2].min() + flat_bottom, 2] = V[:, 2].min()
    hV, hF, hull_vol, centroid = hull_mesh(V)
    ratio = hull_vol / mesh_volume(V, F)
    approximation = approximation or ("convexHull" if ratio <= mc["hull_ratio"] else "convexDecomposition")
    if approximation != "convexHull":   # origin = centre of mass of the collider, where the grasp goes through
        centroid = mesh_centroid(V, F)
    T_body_src = translation(-centroid) @ translation([0, 0, lift]) @ T_ws
    # convexHull: write the hull itself so PhysX and MoveIt collide with the same solid
    Vc, Fc = (hV, hF) if approximation == "convexHull" else (V, F)
    return dict(T_body_src=T_body_src, centroid=centroid, lift=lift, hull_ratio=ratio,
                approximation=approximation, V=Vc - centroid, F=Fc, V_dense=V - centroid)


HAND = {"half_x": 0.032, "half_y": 0.104, "tip_below_tcp": 0.0122}   # MoveIt panda hand.stl / finger.stl, TCP


def hand_clear(tcp_W, yaw: float, obstacles, margin: float) -> bool:
    """Is the top-down hand's footprint (hand y = finger axis = TCP y), inflated by ``margin``, clear of every
    obstacle (world-frame hull vertices) whose top reaches above the fingertips?"""
    x_ax, y_ax = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
    u, v = np.meshgrid(np.linspace(-1, 1, 7) * (HAND["half_x"] + margin), np.linspace(-1, 1, 25) * (HAND["half_y"] + margin))
    pts = tcp_W[:2] + u.reshape(-1, 1) * x_ax + v.reshape(-1, 1) * y_ax
    return not any(O[:, 2].max() > tcp_W[2] - HAND["tip_below_tcp"] and (Delaunay(O[:, :2]).find_simplex(pts) >= 0).any()
                   for O in obstacles)


def grasp_params(V: np.ndarray, grip_from_top: float, mtc: dict, body_W=(), obstacles=()) -> dict:
    """Top-down grasp through the body origin (body-frame vertices V).

    The fingers close along TCP y = (-sin yaw, cos yaw); yaw is chosen so the
    object's cross-section between the finger pads (|x_tcp| < slab_half_width)
    is narrowest, and the TCP is centred on that cross-section.  Yaws whose hand
    footprint would hit an obstacle with the body at any of the world positions
    ``body_W`` (rest, target) are skipped.
    """
    cands = []
    for yaw in np.radians(np.arange(-90.0, 90.0, 1.0)):     # gripper is symmetric under pi
        x_ax, y_ax = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
        slab = V[np.abs(V[:, :2] @ x_ax) < mtc["slab_half_width"]]
        if len(slab) >= 3:
            across = slab[:, :2] @ y_ax
            cands.append((np.ptp(across), yaw, y_ax * (across.min() + across.max()) / 2, slab[:, 2].max()))
    bottom = V[:, 2].min()
    tcp_z = lambda top: max(top - grip_from_top * (top - bottom), bottom + mtc["tip_clearance"])
    clear = [c for c in sorted(cands, key=lambda c: c[0])
             if all(hand_clear(np.r_[np.asarray(p)[:2] + c[2], p[2] + tcp_z(c[3])], c[1], obstacles,
                               mtc["hand_clearance"]) for p in body_W)]
    if not clear:
        print("[Export] WARNING: no top-down grasp clears the obstacles, using the narrowest")
    width, yaw, xy, top = (clear or sorted(cands, key=lambda c: c[0]))[0]
    if width > mtc["max_open_width"]:
        print(f"[Export] WARNING: grasp width {width:.3f} m > gripper opening {mtc['max_open_width']} m")
    return {"grasp_offset_xyz": [round(float(v), 4) for v in (xy[0], xy[1], tcp_z(top))],
            "grasp_yaw": round(float(yaw), 4),
            "finger_close": round(float(max(0.0, width / 2 - mtc["finger_margin"])), 4),
            "grasp_width": round(float(width), 4)}


def _inside(points: np.ndarray, hull_V: np.ndarray) -> np.ndarray:
    return Delaunay(hull_V).find_simplex(points) >= 0


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

def export_scene(cfg: dict) -> str:
    """Build ``cfg['out_dir']`` (see module docstring); returns the manifest path."""
    root = cfg.get("data_root", ".")
    data = lambda p: p if os.path.isabs(p) else os.path.join(root, p)
    out = os.path.abspath(cfg["out_dir"])
    for sub in ("splats", "collision"):
        os.makedirs(os.path.join(out, sub), exist_ok=True)
    s, mc, mtc = cfg["scale_m_per_unit"], cfg["meshing"], cfg["mtc"]
    objects = cfg["objects"]

    # --- Background: decode the Unity asset (only copy of it) ---
    bg_ply = os.path.join(out, "splats", "background.ply")
    write_ply(bg_ply, decode_asset(data(cfg["background_asset"])))
    bg_src = read_xyz(bg_ply, 0.1)
    obj_src = {o["name"]: read_xyz(data(o["ply"])) for o in objects}

    # --- Calibration ---
    T_unity = np.array(cfg["unity_T_base_from_source"], float)
    n, d, inl = fit_table(bg_src, np.vstack(list(obj_src.values())), cfg["table"]["ransac_threshold"] / s)
    T_ws, tilt = world_from_source(T_unity, n, s)
    p0 = np.linalg.solve(T_ws[:3, :3], -T_ws[:3, 3])
    z_table = float(-s * (d + n @ p0))
    bg_W = apply(T_ws, bg_src)
    print(f"[Export] s={s} m/unit, tilt vs Unity {tilt:.2f} deg, table z={z_table:+.4f} m, "
          f"plane inliers {inl.sum()}/{len(bg_src)}")
    T_W_base = T_ws @ np.linalg.inv(T_unity)                 # Unity base frame (scale 2.27) -> W
    unity_W = {k: apply(T_W_base, np.array(v, float)) for k, v in cfg["unity_targets_base"].items()}

    # --- Robot-base splats: calibration check only ---
    notes = list(cfg.get("calibration_notes", []))
    if cfg.get("arm_ply"):
        A = apply(T_ws, read_xyz(data(cfg["arm_ply"]), 0.3))
        A = A[A[:, 2] > z_table + 0.005]
        lo, hi = np.percentile(A, [2, 98], axis=0)
        notes.append(f"check: arm splats p2-p98 xy box centre ({(lo[0] + hi[0]) / 2:+.3f}, {(lo[1] + hi[1]) / 2:+.3f}) m, "
                     f"extent {hi[0] - lo[0]:.3f} x {hi[1] - lo[1]:.3f} m, top {hi[2] - z_table:.3f} m above table "
                     f"(link0: centre {LINK0['center_xy']}, {LINK0['extent_xy'][0]} x {LINK0['extent_xy'][1]} m, "
                     f"{LINK0['height']} m tall); table {-z_table:.3f} m below the base origin; tilt {tilt:.2f} deg")
        print(f"[Export] {notes[-1]}")

    # --- Static colliders ---
    tV, tF = table_collider(apply(T_ws, bg_src[inl]), z_table, cfg["table"]["thickness"])
    cV, cF, card_top = cardboard_collider(bg_W, z_table, unity_W["box"][:2], cfg["cardboard"])
    write_obj(os.path.join(out, "collision", "env_table.obj"), tV, tF)
    write_obj(os.path.join(out, "collision", "env_cardboard.obj"), cV, cF)

    # --- Objects ---
    entries, placed, grasps = [], {}, {}
    with tempfile.TemporaryDirectory() as tmp:
        for o in objects:
            name = o["name"]
            print(f"\n[Export] Object '{name}'")
            shutil.copyfile(data(o["ply"]), os.path.join(out, "splats", f"{name}.ply"))
            b = body_from_mesh(*object_mesh(data(o["ply"]), mc, s, tmp), T_ws, z_table, mc,
                               o.get("approximation"), o.get("flat_bottom", 0.0))
            write_obj(os.path.join(out, "collision", f"{name}.obj"), b["V"], b["F"])

            # target: on the table / cardboard at a Unity target xy, or on another placed object
            on = o["target"]["support"]
            zmin, zmax = b["V"][:, 2].min(), b["V"][:, 2].max()
            if on in placed:
                xy, support = placed[on]["p"][:2], placed[on]["p"][2] + placed[on]["zmax"]
            else:
                xy, support = unity_W[o["target"]["unity"]][:2], {"table": z_table, "cardboard": card_top}[on]
            p_t = np.r_[xy, support + cfg["placement_clearance"] - zmin]
            placed[name] = {"p": p_t, "zmax": zmax}

            # checks: reach (Panda ~0.85 m), mesh vertices inside the statics at rest / at the target
            r = np.linalg.norm(p_t[:2])
            hits = [int(_inside(b["V_dense"] + p, cV).sum() + (b["V_dense"][:, 2] + p[2] < z_table).sum())
                    for p in (b["centroid"], p_t)]
            on_top = on != "cardboard" or _inside(np.r_[p_t[:2], card_top - 1e-3][None], cV)[0]
            print(f"[Check] {name}: target reach {r:.3f} m {'OK' if 0.3 <= r <= 0.75 else 'OUT OF 0.3-0.75 m'}, "
                  f"vertices in statics at rest / target {hits}, target over its support: {bool(on_top)}")

            grasps[name] = grasp_params(b["V_dense"], o.get("grip_from_top", 1 / 3), mtc, (b["centroid"], p_t), [cV])
            size = np.ptp(b["V"], axis=0)
            print(f"[Export] {name}: size {np.round(size * 100, 1).tolist()} cm, hull/mesh volume "
                  f"{b['hull_ratio']:.2f} -> {b['approximation']}, lifted {b['lift'] * 1000:+.1f} mm, grasp {grasps[name]}")
            entries.append({
                "name": name,
                "splats": f"splats/{name}.ply",
                "collision": f"collision/{name}.obj",
                "approximation": b["approximation"],
                "max_convex_hulls": mc["max_convex_hulls"],
                "mass_kg": o["mass_kg"],
                "body_pose_world": pose(b["centroid"]),
                "T_body_from_source": b["T_body_src"].tolist(),
                "size_m": size.tolist(),
                "target_pose_world": pose(p_t),
            })

    # --- Cameras ---
    cam = cfg["unity_camera_base"]
    R_wb = T_W_base[:3, :3] / np.cbrt(np.linalg.det(T_W_base[:3, :3]))
    cam_p = apply(T_W_base, np.array(cam["position"], float))
    xy = np.array([e[k]["p"][:2] for e in entries for k in ("body_pose_world", "target_pose_world")] + [[0, 0]])
    centre = (xy.min(0) + xy.max(0)) / 2                       # objects, targets and the robot base
    top_p = [centre[0], centre[1], z_table + cfg["top_camera_height"]]
    top_R = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # looks down, image up = +X

    manifest = {
        "version": 1, "units": "m", "up_axis": "Z", "world_frame": "panda_link0",
        "T_world_from_source": T_ws.tolist(), "scale_m_per_unit": s,
        "calibration_notes": notes,
        "table": {"height": z_table, "normal_src": n.tolist(), "d_src": float(d)},
        "background": {"splats": "splats/background.ply"},
        "static_colliders": [
            {"name": "table", "mesh": "collision/env_table.obj", "approximation": "convexHull"},
            {"name": "cardboard", "mesh": "collision/env_cardboard.obj", "approximation": "convexHull",
             "top_z": card_top},
        ],
        "objects": entries,
        "camera": {"name": "unity_main",
                   "pose_world": pose(cam_p, usd_camera(R_wb @ cam["forward"], R_wb @ cam["up"])),
                   "vertical_fov_deg": cam["vertical_fov_deg"], "convention": "USD camera (-Z forward, +Y up)"},
        "camera_top": {"name": "top_down", "pose_world": pose(top_p, top_R),
                       "vertical_fov_deg": cam["vertical_fov_deg"], "convention": "USD camera (-Z forward, +Y up)"},
        "robot": {"usd": FRANKA_USD, "prim": "/World/panda", "base_pose_world": pose([0, 0, 0])},
    }
    manifest_path = os.path.join(out, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    # --- MTC task parameters (absolute host paths) ---
    names = [o["name"] for o in objects]
    params = {
        "object_names": names,
        "mesh_scale_factor": 1.0,
        "task_timeout": float(mtc["task_timeout"]),
        "objects": {name: {
            "mesh_path": os.path.join(out, "collision", f"{name}.obj"),
            "pose_topic": f"/{name}_position",
            "target_topic": f"/{name}_target_position",
            "grasp_offset_xyz": grasps[name]["grasp_offset_xyz"],
            "grasp_yaw": grasps[name]["grasp_yaw"],
            "finger_close": grasps[name]["finger_close"],
            "approach_min_dist": mtc["approach"][0], "approach_max_dist": mtc["approach"][1],
            "lift_min_dist": mtc["lift"][0], "lift_max_dist": mtc["lift"][1],
        } for name in names},
        "env_names": ["table", "cardboard"],
        "env_mesh_paths": [os.path.join(out, "collision", f"env_{e}.obj") for e in ("table", "cardboard")],
    }
    with open(os.path.join(out, "mtc_task.yaml"), "w") as f:
        f.write("# Generated by src/sim_export/export.py — do not edit by hand.\n")
        yaml.safe_dump({"/**": {"ros__parameters": params}}, f, sort_keys=False, default_flow_style=None)

    print(f"\n[Export] Manifest → {manifest_path}")
    return manifest_path


def _parse_args():
    p = argparse.ArgumentParser(description="Export Isaac Sim twin assets (splats, colliders, manifest, MTC params).")
    p.add_argument("--config", default="configs/twin_res5.yaml", help="Export config YAML")
    p.add_argument("--out_dir", help="Override: output directory")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    if args.out_dir:
        cfg["out_dir"] = args.out_dir
    export_scene(cfg)

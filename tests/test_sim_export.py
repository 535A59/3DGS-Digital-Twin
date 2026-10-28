"""
Self-checks for the Isaac Sim asset export and the Unity asset decoder.

    python -m tests.test_sim_export

Synthetic: a cube-on-table scene is put into a random
"source" frame (similarity); a Unity-like calibration with a wrong scale and a
1.5 deg tilt error must give back a Z-up, det>0 world frame.  The last two
checks run on the real RES5 data when old_code-data/ is present.
"""

import itertools
import json
import os
import tempfile

import numpy as np
import trimesh
import yaml
from scipy.spatial import Delaunay, cKDTree
from scipy.spatial.transform import Rotation

from src.sim_export.export import (apply, body_from_mesh, export_scene, fit_table, grasp_params, hand_clear,
                                   hull_mesh, translation, world_from_source)
from tools.unity_gs_to_ply import compare_to_source, decode_asset, decode_rot, write_ply

DATA = "old_code-data/3dgs_resource"
Z_TABLE = -0.02                      # table 2 cm below the robot base, like RES5
MC = {"lift": 0.001, "hull_ratio": 1.25}


def _grid(a, b, step):
    u, v = np.meshgrid(np.arange(a[0], a[1], step), np.arange(b[0], b[1], step))
    return u.ravel(), v.ravel()


def _scene(step=0.003, seed=0):
    """10 cm cube on a table plus a floater cluster above it; kind 0 table, 1 cube, 2 floater."""
    rng = np.random.default_rng(seed)
    x, y = _grid((-0.3, 0.3), (-0.3, 0.3), step)
    table = np.c_[x, y, np.zeros_like(x)]
    table = table[~((np.abs(x) < 0.05) & (np.abs(y) < 0.05))]  # hidden under cube

    s, t = _grid((-0.05, 0.05), (0.0, 0.1), step)
    faces = [
        np.c_[s, np.full_like(s, -0.05), t], np.c_[s, np.full_like(s, 0.05), t],
        np.c_[np.full_like(s, -0.05), s, t], np.c_[np.full_like(s, 0.05), s, t],
    ]
    a, b = _grid((-0.05, 0.05), (-0.05, 0.05), step)
    faces.append(np.c_[a, b, np.full_like(a, 0.1)])
    cube = np.vstack(faces)

    floater = rng.normal([0.0, 0.0, 0.2], 0.005, size=(60, 3))
    pts = np.vstack([table, cube, floater]).astype(np.float32)
    kind = np.r_[np.zeros(len(table)), np.ones(len(cube)), np.full(len(floater), 2)]
    return pts, kind


def _source_scene(seed=0):
    """Scene in W (metres), the true T_W_src, and the scene in the source frame."""
    pts, kind = _scene()
    P_W = pts.astype(np.float64) + [0.4, 0.0, Z_TABLE]
    rng = np.random.default_rng(seed)
    s, R = 1.75, Rotation.random(random_state=seed).as_matrix()
    p0 = rng.normal(size=3)
    P_src = P_W @ R / s + p0                                 # inverse of p_W = s R (p - p0)
    return P_W, P_src, kind, s, R, p0


def test_world_from_source_recovers_similarity():
    P_W, P_src, kind, s, R, p0 = _source_scene()
    tilt_err = Rotation.from_rotvec(np.radians(1.5) * np.array([0.6, 0.8, 0.0])).as_matrix()
    T_unity = np.eye(4)
    T_unity[:3, :3] = 2.27 * tilt_err @ R                    # wrong scale + tilted up axis
    T_unity[:3, 3] = -2.27 * tilt_err @ R @ p0
    n, d, inl = fit_table(P_src[kind != 1], P_src[kind == 1], threshold=1e-4)
    T, tilt = world_from_source(T_unity, n, s)

    Rs = T[:3, :3] / s
    assert np.linalg.det(Rs) > 0 and np.allclose(Rs @ Rs.T, np.eye(3), atol=1e-9)
    assert np.allclose(Rs @ n, [0, 0, 1], atol=1e-9)        # table normal -> +Z exactly
    assert np.allclose(Rs, R, atol=1e-6)                     # minimal tilt undoes the tilt error
    assert abs(tilt - 1.5) < 0.05
    assert np.allclose(apply(T, p0[None]), 0, atol=1e-9)     # robot base stays at the origin
    Q = apply(T, P_src)
    assert np.abs(Q[kind == 0, 2] - Z_TABLE).max() < 1e-6    # table flat at the right height
    assert abs(Q[kind == 1, 2].max() - (Z_TABLE + 0.1)) < 1e-6   # cube height kept (metric)
    print(f"tilt recovered {tilt:.3f} deg, table inliers {inl.sum()}/{(kind == 0).sum()}")


def test_body_alignment_and_rest():
    P_W, P_src, kind, s, R, p0 = _source_scene(1)
    T_ws = np.eye(4)
    T_ws[:3, :3], T_ws[:3, 3] = s * R, -s * R @ p0
    cube = P_src[kind == 1]
    V_src, F, _, _ = hull_mesh(cube)
    for drop in (0.0, 0.003):                                # resting on / 3 mm into the table (bleed)
        Vd = V_src - (R.T @ [0, 0, drop]) / s
        b = body_from_mesh(Vd, F, T_ws, Z_TABLE, MC)
        T_W_body = translation(b["centroid"])
        # collider vertices = T_body_from_source applied to the source mesh (minus the clamped bleed)
        d, _ = cKDTree(apply(b["T_body_src"], Vd)).query(b["V"])
        assert drop > 0 or d.max() < 1e-9
        assert abs(apply(T_W_body, b["V"])[:, 2].min() - (Z_TABLE + 0.001)) < 1e-9
        assert np.allclose(b["T_body_src"][:3, :3], T_ws[:3, :3])   # world-aligned body axes
        assert b["approximation"] == "convexHull"
    print(f"cube lift {b['lift'] * 1000:.2f} mm, hull/mesh {b['hull_ratio']:.3f}")


def test_grasp_params_box():
    yaw = np.radians(30)
    c, sn = np.cos(yaw), np.sin(yaw)
    V = np.random.default_rng(0).uniform([-0.04, -0.02, -0.03], [0.04, 0.02, 0.03], (5000, 3))
    V = V @ np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]).T   # 8 x 4 x 6 cm box, yawed 30 deg
    g = grasp_params(V, 1 / 3, {"slab_half_width": 0.01, "tip_clearance": 0.016,
                                "max_open_width": 0.07, "finger_margin": 0.004})
    assert abs(g["grasp_yaw"] - yaw) < np.radians(1.5)       # TCP x along the long side
    assert abs(g["grasp_width"] - 0.04) < 2e-3 and abs(g["finger_close"] - 0.016) < 1e-3
    assert np.allclose(g["grasp_offset_xyz"], [0, 0, 0.03 - 0.02], atol=1e-3)   # 1/3 below the top


def test_grasp_avoids_obstacle():
    """4 x 3 cm block: the narrowest grasp (fingers along y) would put the 21 cm long hand into a tall
    obstacle 8 cm away in +y, so a yaw with the hand turned away from it is chosen."""
    V = np.random.default_rng(1).uniform([-0.02, -0.015, -0.02], [0.02, 0.015, 0.02], (4000, 3))
    wall = np.array(list(itertools.product([0.3, 0.5], [0.08, 0.2], [-0.03, 0.15])))
    mtc = {"slab_half_width": 0.01, "tip_clearance": 0.016, "max_open_width": 0.07, "finger_margin": 0.004,
           "hand_clearance": 0.0}
    body = np.array([0.4, 0.0, 0.0])
    assert abs(grasp_params(V, 1 / 3, mtc)["grasp_yaw"]) < np.radians(2)     # free: narrowest, fingers along y
    g = grasp_params(V, 1 / 3, mtc, [body], [wall])
    tcp = body + g["grasp_offset_xyz"]
    assert abs(g["grasp_yaw"]) > np.radians(60) and hand_clear(tcp, g["grasp_yaw"], [wall], 0.0)
    assert not hand_clear(tcp, 0.0, [wall], 0.0)


def test_decode_rot_roundtrip():
    """Unity smallest-three rotation packing (GaussianUtils.PackSmallest3Rotation) round trip."""
    rng = np.random.default_rng(0)
    q = rng.normal(size=(1000, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    im = np.abs(q).argmax(1)
    perm = {0: [1, 2, 3, 0], 1: [0, 2, 3, 1], 2: [0, 1, 3, 2], 3: [0, 1, 2, 3]}
    qq = np.stack([q[i][perm[im[i]]] for i in range(len(q))])
    three = qq[:, :3] * np.where(qq[:, 3:] >= 0, 1, -1) * np.sqrt(2) * 0.5 + 0.5
    enc = ((three[:, 0] * 1023.5).astype(np.uint32) | ((three[:, 1] * 1023.5).astype(np.uint32) << 10)
           | ((three[:, 2] * 1023.5).astype(np.uint32) << 20) | ((im / 3.0 * 3.5).astype(np.uint32) << 30))
    assert np.all(np.abs(np.abs((decode_rot(enc) * q).sum(1)) - 1) < 2e-3)


def test_unity_decoder_matches_source():
    asset = f"{DATA}/GSreal2sim2real/Assets/GaussianAssets/object1-copy.asset"
    if not os.path.exists(asset):
        print("skip: old_code-data not present")
        return
    with tempfile.TemporaryDirectory() as tmp:
        write_ply(f"{tmp}/d.ply", decode_asset(asset))
        rep = compare_to_source(f"{tmp}/d.ply", f"{DATA}/mesh_source/object1-copy.ply")
    print(rep)
    assert rep["ok"] and rep["pos_err_max"] <= 1e-5


def test_export_res5_schema():
    if not os.path.isdir(DATA):
        print("skip: old_code-data not present")
        return
    cfg = yaml.safe_load(open("configs/twin_res5.yaml"))
    with tempfile.TemporaryDirectory() as tmp:
        cfg["out_dir"] = tmp
        m = json.load(open(export_scene(cfg)))
        task = yaml.safe_load(open(f"{tmp}/mtc_task.yaml"))["/**"]["ros__parameters"]
        T_ws = np.array(m["T_world_from_source"])
        for k in ("version", "units", "up_axis", "world_frame", "T_world_from_source", "scale_m_per_unit",
                  "table", "background", "static_colliders", "objects", "camera", "robot"):
            assert k in m, k
        for o in m["objects"]:
            for k in ("name", "splats", "collision", "approximation", "max_convex_hulls", "mass_kg",
                      "body_pose_world", "T_body_from_source", "size_m", "target_pose_world"):
                assert k in o, k
            T = translation(o["body_pose_world"]["p"]) @ np.array(o["T_body_from_source"])
            assert np.allclose(T[:3, :3], T_ws[:3, :3])      # splats and collider share one transform
            assert os.path.exists(f"{tmp}/{o['collision']}") and os.path.exists(f"{tmp}/{o['splats']}")
            assert len(o["body_pose_world"]["q"]) == 4 and o["body_pose_world"]["q"][0] == 1.0
            mesh = trimesh.load(f"{tmp}/{o['collision']}")          # body origin = collider centre of mass
            com = mesh.center_mass if o["approximation"] == "convexDecomposition" else mesh.convex_hull.center_mass
            assert np.abs(com).max() < 1e-4, (o["name"], com)
        card = trimesh.load(f"{tmp}/collision/env_cardboard.obj")          # one flat lid under the box target
        lid = card.triangles[card.face_normals[:, 2] > 1 - 1e-9].reshape(-1, 3)
        box_t = next(o for o in m["objects"] if o["name"] == "box")["target_pose_world"]["p"]
        assert np.ptp(lid[:, 2]) < 1e-9 and Delaunay(lid[:, :2]).find_simplex(np.array([box_t[:2]]))[0] >= 0
        assert task["object_names"] == [o["name"] for o in m["objects"]]
        for name in task["object_names"]:
            assert set(task["objects"][name]) == {
                "mesh_path", "pose_topic", "target_topic", "grasp_offset_xyz", "grasp_yaw", "finger_close",
                "approach_min_dist", "approach_max_dist", "lift_min_dist", "lift_max_dist"}
            assert os.path.isabs(task["objects"][name]["mesh_path"])
        assert task["env_names"] == ["table", "cardboard"] and len(task["env_mesh_paths"]) == 2


if __name__ == "__main__":
    test_world_from_source_recovers_similarity()
    test_body_alignment_and_rest()
    test_grasp_params_box()
    test_grasp_avoids_obstacle()
    test_decode_rot_roundtrip()
    test_unity_decoder_matches_source()
    test_export_res5_schema()
    print("all checks passed")

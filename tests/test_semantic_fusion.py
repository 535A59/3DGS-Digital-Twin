"""
Self-checks for semantic fusion on a synthetic scene.

    python -m tests.test_semantic_fusion      (or: pytest tests/)

Scene: 10 cm cube on a table, a floater cluster hovering above it, 12 views
on a ring, and SAM-like masks dilated by 15 px so they bleed onto the table.
"""

import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.metrics import adjusted_rand_score

from src.evaluation.metrics import geometry_metrics, ghost_index, projection_consistency
from src.semantic_fusion.fusion import dbscan_1d, in_view, lift_labels, project

import cv2


def _grid(a, b, step):
    u, v = np.meshgrid(np.arange(a[0], a[1], step), np.arange(b[0], b[1], step))
    return u.ravel(), v.ravel()


def _scene(step=0.003, seed=0):
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


def _cameras(n=12, radius=0.6, height=0.5, size=(480, 640)):
    h, w = size
    K = np.array([[500, 0, w / 2], [0, 500, h / 2], [0, 0, 1]], np.float32)
    target = np.array([0, 0, 0.05])
    cams = []
    for a in np.linspace(0, 2 * np.pi, n, endpoint=False):
        c = np.array([radius * np.cos(a), radius * np.sin(a), height])
        fwd = (target - c) / np.linalg.norm(target - c)
        right = np.cross(fwd, [0, 0, 1]); right /= np.linalg.norm(right)
        down = np.cross(fwd, right)
        R = np.stack([right, down, fwd])  # world-to-camera, OpenCV axes
        RT = np.hstack([R, -R @ c[:, None]]).astype(np.float32)
        cams.append(dict(K=K, RT=RT, shape=size, path=""))
    return cams


def _masks(cams, cube, dilate_px=15):
    kernel = np.ones((2 * dilate_px + 1,) * 2, np.uint8)
    masks = []
    for cam in cams:
        uv, z = project(cube, cam["K"], cam["RT"])
        uv = uv[in_view(uv, z, cam["shape"])]
        m = np.zeros(cam["shape"], np.uint8)
        m[uv[:, 1], uv[:, 0]] = 1
        masks.append(cv2.dilate(m, kernel).astype(bool))
    return masks


def test_dbscan_1d_matches_sklearn():
    rng = np.random.default_rng(1)
    v = np.concatenate([rng.normal(0.4, 0.01, 300), rng.normal(0.6, 0.02, 200),
                        rng.uniform(0.0, 1.0, 40)])
    ref = DBSCAN(eps=0.02, min_samples=10).fit_predict(v[:, None])
    ours = dbscan_1d(v, 0.02, 10)
    assert np.array_equal(ours == -1, ref == -1)
    assert adjusted_rand_score(ref, ours) == 1.0


def test_fusion_rejects_bleeding_and_floaters():
    pts, kind = _scene()
    cams = _cameras()
    masks = _masks(cams, pts[kind == 1])

    naive, _ = lift_labels(pts, cams, masks, kappa=float("inf"))
    ours, stats = lift_labels(pts, cams, masks, kappa=1.5)

    recall = ours[kind == 1].mean()
    table_fp, naive_fp = ours[kind == 0].sum(), naive[kind == 0].sum()
    print(f"cube recall={recall:.3f} | table FP ours={table_fp} naive={naive_fp} "
          f"| floater FP={ours[kind == 2].sum()}")

    assert recall > 0.95
    assert ours[kind == 2].sum() == 0
    assert table_fp < 0.5 * naive_fp
    assert projection_consistency(pts[ours], cams, masks) > 90.0


def test_geometry_metrics_identity():
    pts, kind = _scene()
    m = geometry_metrics(pts[kind == 1], pts[kind == 1])
    assert m["chamfer"] == 0.0 and m["f1"] == 1.0


def test_ghost_index_counts_background_residue():
    # one camera at the origin looking down +z; 2x2 image, all pixels in mask
    cam = dict(K=np.array([[1.0, 0, 1], [0, 1, 1], [0, 0, 1]]),
               RT=np.hstack([np.eye(3), np.zeros((3, 1))]), shape=(2, 2))
    front = np.array([[-.5, -.5, 1], [.5, -.5, 1], [-.5, .5, 1], [.5, .5, 1]])
    pts = np.vstack([front, front * [1, 1, 2]])          # same pixels, farther away
    fg = np.r_[True, True, True, False, False, False, False, False]
    # 1 of 4 mask pixels has a background point in front -> 25%; hidden points don't count
    assert ghost_index(pts, fg, [cam], [np.ones((2, 2), bool)]) == 25.0


if __name__ == "__main__":
    test_ghost_index_counts_background_residue()
    test_dbscan_1d_matches_sklearn()
    test_geometry_metrics_identity()
    test_fusion_rejects_bleeding_and_floaters()
    print("all checks passed")

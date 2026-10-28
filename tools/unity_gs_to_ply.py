"""
Decode a UnityGaussianSplatting asset back to a standard 3DGS PLY.

The original Unity demo (``GSreal2sim2real/Assets/GaussianAssets``) only keeps
the background as a Unity asset; no source PLY exists.  This tool rebuilds it.

Supported: format 20231020, "High" quality preset (chunks of 256 splats;
position/scale Norm16, colour Float16x4, SH Norm11) — what every asset in the
demo uses.  Decode math mirrors ``LoadSplatData`` in GaussianSplatting.hlsl.

The demo's modified importer also
  (a) centred the splats on their bbox centre (stored as ``m_Pivot``),
  (b) mirrored x  (pos.x -> -x, quat (x,y,z,w) -> (x,-y,-z,w)), SH untouched,
  (c) lost the 3rd packed rotation component and the largest-component index
      (bits 20..31 of every rotation word are 0xFFF).
(a) and (b) are undone, so the output is in the ORIGINAL source-PLY frame.
For (c) the default repair assumes w is the largest component and z = 0
(median rotation error ~20-25 deg vs ~100 deg for what Unity rendered);
``--raw_rot`` keeps the raw Unity decode.

Usage
-----
    python tools/unity_gs_to_ply.py \\
        --asset  .../GaussianAssets/background-copy-cl.asset \\
        --output /path/to/background.ply

    # validate against an asset whose source PLY exists (positions <= 1e-5):
    python tools/unity_gs_to_ply.py \\
        --asset  .../GaussianAssets/object1-copy.asset \\
        --output /tmp/object1-decoded.ply \\
        --check  .../mesh_source/object1-copy.ply
"""

from __future__ import annotations

import argparse
import os
import re
import sys

import numpy as np
from plyfile import PlyData
from scipy.spatial import cKDTree

SH_C0 = 0.28209479177387814
CHUNK = 256
CHUNK_DT = np.dtype([("col", "<u4", 4), ("posX", "<f4", 2), ("posY", "<f4", 2),
                     ("posZ", "<f4", 2), ("scl", "<u4", 3), ("sh", "<u4", 3)])
# m_PosFormat, m_ScaleFormat, m_ColorFormat, m_SHFormat of the "High" preset
HIGH_PRESET = {"m_PosFormat": 1, "m_ScaleFormat": 1, "m_ColorFormat": 1, "m_SHFormat": 2}


def read_asset(path: str) -> dict:
    """Parse the asset YAML header (splat count, formats, pivot)."""
    text = open(path).read()
    get = lambda k: re.search(rf"^\s*{k}: (.+)$", text, re.M).group(1).strip()
    for k, v in HIGH_PRESET.items():
        if int(get(k)) != v:
            raise NotImplementedError(f"{k}={get(k)}; only the High preset {HIGH_PRESET} is supported")
    pivot = np.zeros(3)
    if "m_Pivot" in text:
        pivot = np.array([float(x) for x in re.findall(r"[xyz]: ([-0-9.eE]+)", get("m_Pivot"))])
    base = os.path.splitext(path)[0]
    return {"n": int(get("m_SplatCount")), "pivot": pivot,
            "files": {k: f"{base}_{k}.bytes" for k in ("pos", "oth", "col", "shs", "chk")}}


def _half_pair(u):
    """uint32 packing (min | max << 16) of two float16 -> (min, max)."""
    u = np.asarray(u, np.uint32)
    lo = (u & 0xFFFF).astype(np.uint16).view(np.float16).astype(np.float64)
    hi = (u >> 16).astype(np.uint16).view(np.float16).astype(np.float64)
    return lo, hi


def _norm11(words):
    """11.10.11 packed unit vector."""
    w = np.asarray(words, np.uint64)
    return np.stack([(w & 2047) / 2047.0, ((w >> 11) & 1023) / 1023.0, ((w >> 21) & 2047) / 2047.0], 1)


def decode_rot(u):
    """10.10.10.2 smallest-three -> quaternion xyzw (DecodeRotation in GaussianSplatting.hlsl)."""
    u = np.asarray(u, np.uint64)
    idx = (u >> 30).astype(int)
    xyz = np.stack([(u & 1023), (u >> 10) & 1023, (u >> 20) & 1023], 1) / 1023.0
    xyz = xyz * np.sqrt(2.0) - 1.0 / np.sqrt(2.0)
    w = np.sqrt(1.0 - np.clip((xyz ** 2).sum(1), 0, 1))
    q = np.c_[xyz, w]                       # (a, b, c, largest)
    out = q.copy()
    out[idx == 0] = q[idx == 0][:, [3, 0, 1, 2]]
    out[idx == 1] = q[idx == 1][:, [0, 3, 1, 2]]
    out[idx == 2] = q[idx == 2][:, [0, 1, 3, 2]]
    return out


def _splat_to_pixel(idx, width=2048):
    """Morton 16x16 tile swizzle of the colour texture."""
    t = idx.astype(np.uint32)
    t = (t & 0xFF) | ((t & 0xFE) << 7)
    t &= 0x5555
    t = (t ^ (t >> 1)) & 0x3333
    t = (t ^ (t >> 2)) & 0x0F0F
    x16, y16 = t & 0xF, t >> 8
    blk = idx >> 8
    return ((blk // (width // 16)) * 16 + y16) * width + (blk % (width // 16)) * 16 + x16


def decode_asset(path: str, raw_rot: bool = False) -> dict:
    """Decode an asset into standard 3DGS PLY attributes in the source frame."""
    a = read_asset(path)
    n = a["n"]
    rb = lambda k: open(a["files"][k], "rb").read()
    ch = np.frombuffer(rb("chk"), CHUNK_DT)[np.arange(n) // CHUNK]

    # Position (Norm16, chunk range)
    pos = np.frombuffer(rb("pos"), "<u2", n * 3).reshape(n, 3) / 65535.0
    pmin = np.stack([ch["posX"][:, 0], ch["posY"][:, 0], ch["posZ"][:, 0]], 1)
    pmax = np.stack([ch["posX"][:, 1], ch["posY"][:, 1], ch["posZ"][:, 1]], 1)
    pos = pmin + (pmax - pmin) * pos

    # Rotation word + scale (Norm16, chunk range, ^8)
    oth = np.frombuffer(rb("oth"), np.uint8, n * 10).reshape(n, 10)
    rword = oth[:, :4].copy().view("<u4")[:, 0]
    q = decode_rot(rword)
    scl = oth[:, 4:].copy().view("<u2") / 65535.0
    smin, smax = _half_pair(ch["scl"])
    scl = (smin + (smax - smin) * scl) ** 8

    # Colour + opacity (Float16x4 texture, chunk range, inverse square-centring)
    tex = np.frombuffer(rb("col"), "<f2").reshape(-1, 4).astype(np.float64)
    col = tex[_splat_to_pixel(np.arange(n))]
    cmin, cmax = _half_pair(ch["col"])
    col = cmin + (cmax - cmin) * col
    x = (col[:, 3] - 0.5) * 0.5
    alpha = np.sqrt(np.abs(x)) * np.sign(x) + 0.5

    # SH bands 1-3 (Norm11, chunk range)
    sh = _norm11(np.frombuffer(rb("shs"), "<u4", n * 15)).reshape(n, 15, 3)
    hmin, hmax = _half_pair(ch["sh"])
    sh = hmin[:, None, :] + (hmax - hmin)[:, None, :] * sh

    broken = (rword >> 20) == 0xFFF
    if not raw_rot and broken.any():
        # ponytail: w-largest / z=0 guess; the lost component is unrecoverable from the asset.
        q[broken, 2] = 0.0
        q[broken, 3] = np.sqrt(np.clip(1 - q[broken, 0] ** 2 - q[broken, 1] ** 2, 0, 1))

    # Undo centring + x-mirror (SH were not mirrored by the importer)
    p = pos + a["pivot"]
    p[:, 0] *= -1
    q[:, 1] *= -1
    q[:, 2] *= -1
    q /= np.linalg.norm(q, axis=1, keepdims=True)

    alpha = np.clip(alpha, 1e-6, 1 - 1e-6)
    print(f"[Unity] {os.path.basename(path)}: {n} splats, pivot={np.round(a['pivot'], 6).tolist()}, "
          f"broken rotations={broken.mean():.1%} ({'raw' if raw_rot else 'repaired'})")
    return {
        "xyz": p,
        "f_dc": (col[:, :3] - 0.5) / SH_C0,
        "f_rest": sh.transpose(0, 2, 1).reshape(n, 45),   # all R coeffs, then G, then B
        "opacity": np.log(alpha / (1 - alpha)),
        "scale": np.log(np.maximum(scl, 1e-12)),
        "rot": q[:, [3, 0, 1, 2]],                         # PLY rot_0..3 = w, x, y, z
    }


def write_ply(path: str, g: dict) -> None:
    """Write the standard INRIA 3DGS layout (binary little endian, float32)."""
    n = len(g["xyz"])
    names = (["x", "y", "z", "nx", "ny", "nz"] + [f"f_dc_{i}" for i in range(3)]
             + [f"f_rest_{i}" for i in range(45)] + ["opacity"]
             + [f"scale_{i}" for i in range(3)] + [f"rot_{i}" for i in range(4)])
    data = np.concatenate([g["xyz"], np.zeros((n, 3)), g["f_dc"], g["f_rest"],
                           g["opacity"][:, None], g["scale"], g["rot"]], 1).astype("<f4")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as f:
        f.write(f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\n".encode())
        f.write("".join(f"property float {nm}\n" for nm in names).encode())
        f.write(b"end_header\n")
        f.write(data.tobytes())
    print(f"[Unity] Saved → {path}")


def compare_to_source(decoded_ply: str, source_ply: str) -> dict:
    """Match splats by nearest position (import reorders them) and report errors."""
    da, sa = PlyData.read(decoded_ply)["vertex"].data, PlyData.read(source_ply)["vertex"].data
    cols = lambda d, ks: np.stack([d[k] for k in ks], 1).astype(np.float64)
    A, B = cols(da, "xyz"), cols(sa, "xyz")
    dist, nn = cKDTree(B).query(A)
    sig = lambda x: 1 / (1 + np.exp(-x))
    dc = [f"f_dc_{i}" for i in range(3)]
    rest = [f"f_rest_{i}" for i in range(45)]
    rot = [f"rot_{i}" for i in range(4)]
    q2 = cols(sa, rot)[nn]
    q2 /= np.linalg.norm(q2, axis=1, keepdims=True)
    rep = {
        "n": (len(A), len(B)),
        "unique_matches": int(len(np.unique(nn))),
        "pos_err_max": float(dist.max()),
        "rgb_err_max": float(np.abs(cols(da, dc) - cols(sa, dc)[nn]).max() * SH_C0),
        "alpha_err_max": float(np.abs(sig(cols(da, ["opacity"])) - sig(cols(sa, ["opacity"])[nn])).max()),
        "log_scale_err_median": float(np.median(np.abs(cols(da, [f"scale_{i}" for i in range(3)])
                                                       - cols(sa, [f"scale_{i}" for i in range(3)])[nn]))),
        "sh_err_median": float(np.median(np.abs(cols(da, rest) - cols(sa, rest)[nn]))),
        "rot_err_deg_median": float(np.median(2 * np.degrees(np.arccos(
            np.clip(np.abs((cols(da, rot) * q2).sum(1)), 0, 1))))),
    }
    rep["ok"] = (rep["unique_matches"] == len(A) == len(B) and rep["pos_err_max"] <= 1e-5
                 and rep["alpha_err_max"] < 0.02 and rep["rgb_err_max"] < 0.01
                 and rep["sh_err_median"] < 0.01)
    return rep


def _parse_args():
    p = argparse.ArgumentParser(description="Decode a UnityGaussianSplatting asset to a 3DGS PLY.")
    p.add_argument("--asset", required=True, help="Unity <name>.asset (the <name>_*.bytes must sit next to it)")
    p.add_argument("--output", required=True, help="Output 3DGS PLY")
    p.add_argument("--raw_rot", action="store_true", help="Keep the broken rotations as Unity rendered them")
    p.add_argument("--check", help="Source PLY to validate against (exit 1 on failure)")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    write_ply(args.output, decode_asset(args.asset, raw_rot=args.raw_rot))
    if args.check:
        rep = compare_to_source(args.output, args.check)
        for k, v in rep.items():
            print(f"[Check] {k}: {v}")
        print("[Check] PASS" if rep["ok"] else "[Check] FAIL")
        sys.exit(0 if rep["ok"] else 1)

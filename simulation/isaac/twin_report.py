"""
Success metrics of a digital-twin pick & place run (contract §7), host side.

Reads ``out_dir/manifest.json``, ``out_dir/final_state.json`` (run_twin.py), the
export config (which support each object is placed on) and the MTC log, and
writes ``out_dir/report.json``:

  * per object: final body position error to its target (xy and z), tilt,
    highest lift during the run (physical carry), whether it rests on the
    intended support (bottom of its collider within --z_tol of the support top
    and its centre over the support's footprint), MTC status, grasp mode,
    success = on support and |err_xy| <= --xy_tol and |err_z| <= --z_tol and
    tilt <= --tilt_tol (upright);
  * contacts (PhysX reports): finger-object grasp contacts, and unexpected ones:
    robot vs static env, non-finger robot links vs objects, objects vs env /
    objects that are not their rest or target support;
  * timing and overall success (MTC complete, every object placed, no
    unexpected robot contact).

Usage
-----
    python simulation/isaac/twin_report.py --out_dir output/twin_res5 --config configs/twin_res5.yaml \\
        --mtc_log output/twin_res5/mtc.log --total_s 512
"""

import argparse
import json
import os
import re

import numpy as np
import yaml
from scipy.spatial import Delaunay

from build_scene import read_obj


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def inside_xy(p, V):
    """Is the xy point p inside the 2D convex hull of V's xy?"""
    return bool(Delaunay(V[:, :2]).find_simplex(np.asarray(p[:2])[None])[0] >= 0)


def mtc_status(log_text, names):
    """Per-object MTC status from pick_place_demo's log lines."""
    out = {}
    for n in names:
        done = re.search(rf"Object '{n}' done in ([0-9.]+) s", log_text)
        if done:
            out[n] = {"status": "done", "sim_s": float(done.group(1))}
        elif re.search(rf"Task failed: planning '{n}'", log_text):
            out[n] = {"status": "planning failed"}
        elif re.search(rf"Task failed: executing '{n}'", log_text):
            out[n] = {"status": "execution failed"}
        else:
            out[n] = {"status": "not run"}
    return out


def classify_contacts(pairs, robot, supports):
    """Split PhysX contact pairs into grasp contacts and unexpected ones.
    ``supports``: object -> set of body/env names it may rest on (rest + target support)."""
    grasp, unexpected, resting = [], [], []
    name = lambda p: p.rsplit("/", 1)[-1]
    for key, c in pairs.items():
        a, b = key.split(" | ")
        ra, rb = a.startswith(robot + "/"), b.startswith(robot + "/")
        entry = dict(c, pair=f"{name(a)} - {name(b)}", robot=ra or rb)
        if ra and rb:
            continue
        if ra or rb:
            link, other = (a, b) if ra else (b, a)
            if other.startswith("/World/Objects/") and "finger" in name(link):
                grasp.append(entry)
            else:
                unexpected.append(entry)
            continue
        oa = name(a) if a.startswith("/World/Objects/") else None
        ob = name(b) if b.startswith("/World/Objects/") else None
        if (oa and name(b) in supports.get(oa, ())) or (ob and name(a) in supports.get(ob, ())):
            resting.append(entry)
        else:
            unexpected.append(entry)
    return grasp, unexpected, resting


def report(out_dir, cfg, state, log_text, xy_tol=0.03, z_tol=0.015, tilt_tol=15.0, total_s=None):
    with open(os.path.join(out_dir, "manifest.json")) as f:
        manifest = json.load(f)
    objs = {o["name"]: o for o in manifest["objects"]}
    support_of = {o["name"]: o["target"]["support"] for o in cfg["objects"]}
    world_V = {}
    for o in manifest["objects"]:
        s = state["objects"][o["name"]]
        V, _ = read_obj(os.path.join(out_dir, o["collision"]))
        world_V[o["name"]] = V @ rot(s["q"]).T + np.asarray(s["p"])
    for env in manifest["static_colliders"]:
        world_V[env["name"]] = read_obj(os.path.join(out_dir, env["mesh"]))[0]

    mtc = mtc_status(log_text, list(objs))
    modes = state.get("grasp_mode", {})
    rows = {}
    for n, o in objs.items():
        s, sup = state["objects"][n], support_of[n]
        p, target = np.asarray(s["p"]), np.asarray(o["target_pose_world"]["p"])
        sup_top = manifest["table"]["height"] if sup == "table" else world_V[sup][:, 2].max()
        gap = float(world_V[n][:, 2].min() - sup_top)
        on_support = abs(gap) <= z_tol and inside_xy(p, world_V[sup])
        err_xy, err_z = float(np.linalg.norm(p[:2] - target[:2])), float(p[2] - target[2])
        tilt = float(np.degrees(np.arccos(np.clip(rot(s["q"])[2, 2], -1, 1))))
        rows[n] = {
            "final_p": p.round(4).tolist(), "target_p": target.round(4).tolist(),
            "err_xy_m": round(err_xy, 4), "err_z_m": round(err_z, 4),
            "tilt_deg": round(tilt, 1),
            "max_lift_m": round(s.get("max_lift_m", 0.0), 4),
            "support": sup, "gap_to_support_m": round(gap, 4), "on_support": bool(on_support),
            "grasp_mode": modes.get(n, "physical"), "mtc": mtc[n],
            "success": bool(on_support and err_xy <= xy_tol and abs(err_z) <= z_tol and tilt <= tilt_tol),
        }
    supports = {n: {"table", support_of[n]} for n in objs}
    grasp, unexpected, resting = classify_contacts(state.get("contacts", {}), manifest["robot"]["prim"], supports)
    robot_unexpected = [c for c in unexpected if c["robot"]]
    complete = "Execution complete:" in log_text
    return {
        "success": bool(complete and all(r["success"] for r in rows.values()) and not robot_unexpected),
        "mtc_complete": complete,
        "tolerances": {"xy_m": xy_tol, "z_m": z_tol, "tilt_deg": tilt_tol},
        "objects": rows,
        "contacts": {"unexpected": unexpected, "grasp": grasp, "resting": resting},
        "timing": {"total_wall_s": total_s, "sim_s": round(state["sim_time"], 2),
                   "isaac_wall_s": round(state["wall_time"], 2),
                   "rtf": round(state["sim_time"] / max(state["wall_time"], 1e-9), 3)},
    }


def main():
    ap = argparse.ArgumentParser(description="final_state.json + MTC log -> out_dir/report.json")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--config", required=True, help="export config (objects[].target.support)")
    ap.add_argument("--mtc_log", default=None, help="default: <out_dir>/mtc.log")
    ap.add_argument("--xy_tol", type=float, default=0.03)
    ap.add_argument("--z_tol", type=float, default=0.015)
    ap.add_argument("--tilt_tol", type=float, default=15.0, help="deg from the upright rest orientation")
    ap.add_argument("--total_s", type=float, default=None, help="wall time of the whole demo")
    args = ap.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    with open(os.path.join(args.out_dir, "final_state.json")) as f:
        state = json.load(f)
    log_path = args.mtc_log or os.path.join(args.out_dir, "mtc.log")
    log_text = open(log_path, errors="replace").read() if os.path.exists(log_path) else ""
    rep = report(args.out_dir, cfg, state, log_text, args.xy_tol, args.z_tol, args.tilt_tol, args.total_s)
    with open(os.path.join(args.out_dir, "report.json"), "w") as f:
        json.dump(rep, f, indent=2)
    for n, r in rep["objects"].items():
        print(f"[Report] {n:7s} {'OK  ' if r['success'] else 'FAIL'} err_xy {r['err_xy_m'] * 1000:6.1f} mm  "
              f"err_z {r['err_z_m'] * 1000:+6.1f} mm  tilt {r['tilt_deg']:5.1f} deg  lift {r['max_lift_m']:.3f} m  "
              f"on {r['support']}: {r['on_support']}  mtc: {r['mtc']['status']}  grasp: {r['grasp_mode']}")
    for c in rep["contacts"]["unexpected"]:
        print(f"[Report] unexpected contact {c['pair']}: t {c['first_t']:.1f}-{c['last_t']:.1f} s, "
              f"{c['touches']} touches, peak impulse {c['max_impulse']:.3g} N s")
    print(f"[Report] success={rep['success']} (MTC complete {rep['mtc_complete']}) -> "
          f"{os.path.join(args.out_dir, 'report.json')}")


if __name__ == "__main__":
    main()

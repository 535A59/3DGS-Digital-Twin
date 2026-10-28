"""
Self-check for the digital-twin run report (simulation/isaac/twin_report.py).

    python -m tests.test_twin_report

Synthetic out_dir: a box placed on the cardboard (success) and a cube that fell
next to the box instead of onto it (not on its support), plus PhysX contact
pairs of every kind and an MTC log.
"""

import itertools
import json
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation", "isaac"))
import twin_report  # noqa: E402


def _cuboid_obj(path, lo, hi):
    V = np.array(list(itertools.product(*zip(lo, hi))))
    F = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    with open(path, "w") as f:
        f.writelines(f"v {x} {y} {z}\n" for x, y, z in V)
        f.writelines("f " + " ".join(str(i + 1) for i in face) + "\n" for face in F)


def test_report():
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(f"{d}/collision")
        _cuboid_obj(f"{d}/collision/env_table.obj", (-1, -1, -0.02), (1, 1, 0.0))
        _cuboid_obj(f"{d}/collision/env_cardboard.obj", (0.4, -0.1, 0.0), (0.6, 0.1, 0.1))
        _cuboid_obj(f"{d}/collision/box.obj", (-0.03, -0.03, -0.04), (0.03, 0.03, 0.04))
        _cuboid_obj(f"{d}/collision/cube.obj", (-0.0125,) * 3, (0.0125,) * 3)
        I = [1.0, 0.0, 0.0, 0.0]
        box_t, cube_t = [0.5, 0.0, 0.141], [0.5, 0.0, 0.1935]
        manifest = {
            "table": {"height": 0.0}, "robot": {"prim": "/World/panda"},
            "static_colliders": [{"name": "table", "mesh": "collision/env_table.obj"},
                                 {"name": "cardboard", "mesh": "collision/env_cardboard.obj"}],
            "objects": [{"name": "box", "collision": "collision/box.obj", "target_pose_world": {"p": box_t, "q": I}},
                        {"name": "cube", "collision": "collision/cube.obj", "target_pose_world": {"p": cube_t, "q": I}}],
        }
        json.dump(manifest, open(f"{d}/manifest.json", "w"))
        cfg = {"objects": [{"name": "box", "target": {"support": "cardboard"}},
                           {"name": "cube", "target": {"support": "box"}}]}
        c = {"first_t": 1.0, "last_t": 2.0, "steps": 10, "touches": 1, "max_impulse": 0.01}
        state = {
            "sim_time": 100.0, "wall_time": 110.0,
            "objects": {"box": {"p": [0.505, 0.004, 0.142], "q": I, "max_lift_m": 0.2},
                        "cube": {"p": [0.3, 0.0, 0.0135], "q": I, "max_lift_m": 0.25}},
            "contacts": {"/World/Objects/box | /World/panda/panda_leftfinger": c,      # grasp
                         "/World/Env/cardboard | /World/panda/panda_link7": c,        # unexpected (robot)
                         "/World/Objects/cube | /World/panda/panda_hand": c,          # unexpected (palm)
                         "/World/Env/cardboard | /World/Objects/box": c,              # box rests on its target
                         "/World/Env/table | /World/Objects/cube": c,                 # rest support
                         "/World/Objects/box | /World/Objects/cube": c,               # cube's target support
                         "/World/Env/cardboard | /World/Objects/cube": c},            # unexpected (knocked)
        }
        log = "Object 'box' done in 41.5 s (sim)\nTask failed: executing 'cube'\n"
        r = twin_report.report(d, cfg, state, log)

        box, cube = r["objects"]["box"], r["objects"]["cube"]
        assert box["on_support"] and box["success"] and abs(box["err_xy_m"] - np.hypot(0.005, 0.004)) < 1e-4
        assert abs(box["gap_to_support_m"] - 0.002) < 1e-6 and abs(box["err_z_m"] - 0.001) < 1e-6
        assert not cube["on_support"] and not cube["success"]               # on the table, not on the box
        assert box["mtc"] == {"status": "done", "sim_s": 41.5} and cube["mtc"]["status"] == "execution failed"
        assert box["grasp_mode"] == "physical" and box["tilt_deg"] == 0.0
        pairs = lambda k: sorted(x["pair"] for x in r["contacts"][k])
        assert pairs("grasp") == ["box - panda_leftfinger"]
        assert pairs("unexpected") == ["cardboard - cube", "cardboard - panda_link7", "cube - panda_hand"]
        assert pairs("resting") == ["box - cube", "cardboard - box", "table - cube"]
        assert not r["success"] and not r["mtc_complete"] and r["timing"]["rtf"] == round(100 / 110, 3)

        # the same box tilted 20 deg: still on the cardboard at its target, but not upright
        state["objects"]["box"]["q"] = [np.cos(np.radians(10)), np.sin(np.radians(10)), 0.0, 0.0]
        box = twin_report.report(d, cfg, state, log)["objects"]["box"]
        assert box["tilt_deg"] == 20.0 and box["on_support"] and abs(box["err_z_m"]) < 0.015 and not box["success"]


if __name__ == "__main__":
    test_report()
    print("all checks passed")

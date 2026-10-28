"""
Isaac Sim 6.1 stage of the 3DGS digital twin (contract §4).

Reads ``out_dir/manifest.json`` (written by src/sim_export/export.py) and
authors, in the world frame W = panda_link0 (Z-up, metres):

    /World/physicsScene               gravity -Z 9.81, 120 Hz
    /World/panda                      Franka (franka.usd from the 6.1 asset root), starts in MoveIt's
                                      "ready" pose; finger drive stiffer/stronger, finger pads high friction
    /World/Background/Splats          background 3DGS, xform = T_world_from_source
    /World/Env/<name>                 invisible static collider meshes (world-frame OBJs)
    /World/Objects/<name>             rigid body at body_pose_world, centre of mass at the body origin
        /Collision                    invisible mesh (body-frame OBJ), convexHull / convexDecomposition,
                                      torsional friction patch
        /Splats                       object 3DGS, xform = T_body_from_source
    /World/Camera_<name>              recording cameras (unity_main, top)
    /World/DomeLight                  light for the robot meshes (splats carry baked radiance)

Splat PLYs are converted once with the image's gsplat converter
(ParticleField3DGaussianSplat) and cached in ``out_dir/usd/<name>.usdc``.
The PLYs themselves are never modified; all transforms live on the prims.

Usage (inside the Isaac Sim 6.1 container, see run_isaac.sh)
-----
    SCRIPT=build_scene.py ROS_DOMAIN_ID=62 simulation/isaac/run_isaac.sh   # -> out_dir/twin_scene.usda
"""

import argparse
import json
import math
import os

import numpy as np

FINGER_MAX_FORCE = 70.0      # N, Franka Hand continuous force (asset default 7.2 N)
FINGER_STIFFNESS = 2000.0    # N/m, asset default 400 squeezes with only ~1.6 N at 4 mm
FRICTION = 1.0               # static = dynamic, objects + env + finger pads
# m, PhysX torsional friction patch on the object colliders (pair = max of both shapes): a finger pad touching a
# reconstructed (slightly slanted) side at one point is otherwise a frictionless pivot, and a can gripped above its
# centre of mass swings ~10 deg about the finger axis when lifted. ~ half the Franka pad width.
TORSION_RADIUS = 0.01
H_APERTURE = 20.955          # mm, USD default horizontal aperture
# deg, start in MoveIt's "ready" pose (= the MTC home pose). The asset's folded default puts the hand next to link5,
# where MoveIt already sees the opened right finger touching link5 and rejects the task's first "open hand".
READY_DEG = {"panda_joint1": 0.0, "panda_joint2": -45.0, "panda_joint3": 0.0, "panda_joint4": -135.0,
             "panda_joint5": 0.0, "panda_joint6": 90.0, "panda_joint7": 45.0}


def load_manifest(out_dir):
    with open(os.path.join(out_dir, "manifest.json")) as f:
        return json.load(f)


def read_obj(path):
    """Vertices (N,3) and faces (lists of 0-based indices) of a plain OBJ."""
    V, F = [], []
    with open(path) as f:
        for line in f:
            t = line.split()
            if t and t[0] == "v":
                V.append([float(x) for x in t[1:4]])
            elif t and t[0] == "f":
                F.append([int(x.split("/")[0]) - 1 for x in t[1:]])
    return np.asarray(V), F


def splat_usd(out_dir, ply_rel, name):
    """Convert a 3DGS PLY to a ParticleField .usdc once (cached in out_dir/usd/)."""
    ply = os.path.join(out_dir, ply_rel)
    usd = os.path.join(out_dir, "usd", f"{name}.usdc")
    if os.path.exists(usd) and os.path.getmtime(usd) >= os.path.getmtime(ply):
        return usd
    from omni.kit.converter.gsplat import read_ply, write_gaussian_splat_usd
    os.makedirs(os.path.dirname(usd), exist_ok=True)
    write_gaussian_splat_usd(read_ply(ply), usd, source_file=ply, prim_name=name, up_axis="Z")
    print(f"[Scene] converted {ply_rel} -> {usd}", flush=True)
    return usd


def _mat(T):
    from pxr import Gf
    return Gf.Matrix4d(np.asarray(T, dtype=float).T.tolist())  # USD: row vectors, p' = p M


def _pose_ops(prim, pose, dz=0.0):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xformable(prim)
    p, q = pose["p"], pose["q"]
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(p[0], p[1], p[2] + dz))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q[0], q[1], q[2], q[3]))


def _collider_mesh(stage, path, obj, approximation, material, max_hulls=16):
    from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    V, F = read_obj(obj)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(V.tolist())
    m.CreateFaceVertexCountsAttr([len(f) for f in F])
    m.CreateFaceVertexIndicesAttr([i for f in F for i in f])
    m.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(approximation)
    if approximation == "convexDecomposition":
        PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(m.GetPrim()).CreateMaxConvexHullsAttr(max_hulls)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
    return m


def _splats(stage, path, usd, T):
    from pxr import UsdGeom
    prim = stage.DefinePrim(path)
    prim.GetReferences().AddReference(usd)
    UsdGeom.Xformable(prim).AddTransformOp().Set(_mat(T))
    return prim


def _camera(stage, path, cam, aspect):
    from pxr import Gf, UsdGeom
    c = UsdGeom.Camera.Define(stage, path)
    _pose_ops(c.GetPrim(), cam["pose_world"])
    v_ap = H_APERTURE / aspect
    c.CreateHorizontalApertureAttr(H_APERTURE)
    c.CreateVerticalApertureAttr(v_ap)
    c.CreateFocalLengthAttr(v_ap / (2.0 * math.tan(math.radians(cam["vertical_fov_deg"]) / 2.0)))
    c.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    return path


def cameras(manifest):
    """{name: camera dict} for the recording cameras in the manifest."""
    cams = {manifest["camera"]["name"]: manifest["camera"]}
    if "camera_top" in manifest:
        cams["top"] = manifest["camera_top"]
    return cams


def build(stage, manifest, out_dir, franka_usd, drop=0.0, aspect=16 / 9):
    """Author the twin onto ``stage``. ``drop`` lifts every object (physics test)."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(120)

    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/grip")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(FRICTION)
    pm.CreateDynamicFrictionAttr(FRICTION)
    pm.CreateRestitutionAttr(0.0)

    # Robot at the identity pose (W = panda_link0)
    robot = manifest["robot"]["prim"]
    stage_utils.add_reference_to_stage(usd_path=franka_usd, path=robot)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot)):
        if prim.GetName() in READY_DEG:
            UsdPhysics.DriveAPI.Get(prim, "angular").CreateTargetPositionAttr(READY_DEG[prim.GetName()])
            prim.ApplyAPI("PhysicsJointStateAPI", "angular")
            prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(READY_DEG[prim.GetName()])
        elif prim.GetName() == "panda_finger_joint1":
            drive = UsdPhysics.DriveAPI.Get(prim, "linear")
            drive.GetMaxForceAttr().Set(FINGER_MAX_FORCE)
            drive.GetStiffnessAttr().Set(FINGER_STIFFNESS)
        elif prim.GetName() in ("panda_leftfinger", "panda_rightfinger"):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")

    # Static background splats + invisible static colliders
    UsdGeom.Xform.Define(stage, "/World/Background")
    _splats(stage, "/World/Background/Splats", splat_usd(out_dir, manifest["background"]["splats"], "background"),
            manifest["T_world_from_source"])
    UsdGeom.Xform.Define(stage, "/World/Env")
    for env in manifest["static_colliders"]:
        _collider_mesh(stage, f"/World/Env/{env['name']}", os.path.join(out_dir, env["mesh"]), env["approximation"], mat)

    # Objects: rigid body = invisible collider + splats, both in the body frame
    UsdGeom.Xform.Define(stage, "/World/Objects")
    for o in manifest["objects"]:
        body = UsdGeom.Xform.Define(stage, f"/World/Objects/{o['name']}").GetPrim()
        _pose_ops(body, o["body_pose_world"], dz=drop)
        UsdPhysics.RigidBodyAPI.Apply(body)
        mass = UsdPhysics.MassAPI.Apply(body)
        mass.CreateMassAttr(o["mass_kg"])
        # COM = body origin = the collider's centre of mass (export), which the grasp goes through. For
        # convexDecomposition PhysX would use the centroid of the overlapping hulls (hammer: 3 mm off the grasp,
        # enough to swing it ~40 deg in the fingers).
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        col = _collider_mesh(stage, f"{body.GetPath()}/Collision", os.path.join(out_dir, o["collision"]),
                             o["approximation"], mat, o.get("max_convex_hulls", 16))
        PhysxSchema.PhysxCollisionAPI.Apply(col.GetPrim()).CreateMinTorsionalPatchRadiusAttr(TORSION_RADIUS)
        _splats(stage, f"{body.GetPath()}/Splats", splat_usd(out_dir, o["splats"], o["name"]), o["T_body_from_source"])

    for name, cam in cameras(manifest).items():
        _camera(stage, f"/World/Camera_{name}", cam, aspect)

    # The light only shades the robot meshes (splats carry baked radiance). Dome only: a DistantLight casts
    # the arm's shadow onto the splats as a dark smear, even with shadow:enable = False.
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
    check_stage(stage, manifest, drop)


def check_stage(stage, manifest, drop=0.0):
    """Self-check: authored world transforms == manifest (catches USD row-vector / quaternion slips)."""
    from pxr import Usd, UsdGeom

    def world(path):
        m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(m).T  # back to column-vector convention

    def pose(p, dz=0.0):
        w, x, y, z = p["q"]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, np.add(p["p"], [0, 0, dz])
        return T

    assert np.allclose(world("/World/Background/Splats"), manifest["T_world_from_source"], atol=1e-6)
    for o in manifest["objects"]:
        T_body = pose(o["body_pose_world"], drop)
        assert np.allclose(world(f"/World/Objects/{o['name']}"), T_body, atol=1e-6)
        assert np.allclose(world(f"/World/Objects/{o['name']}/Splats"), T_body @ np.asarray(o["T_body_from_source"]),
                           atol=1e-6)
    for name, cam in cameras(manifest).items():
        assert np.allclose(world(f"/World/Camera_{name}"), pose(cam["pose_world"]), atol=1e-6)
    print("[Check] stage transforms match the manifest", flush=True)


def franka_url(app, manifest):
    from isaacsim.storage.native import get_assets_root_path_async
    return app.run_coroutine(get_assets_root_path_async()) + manifest["robot"]["usd"]


def main():
    ap = argparse.ArgumentParser(description="Author the twin stage and save it as out_dir/twin_scene.usda")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--output", default=None, help="default: <out_dir>/twin_scene.usda")
    args, _ = ap.parse_known_args()
    os.umask(0)  # files written for uid 1234 stay editable by the host user

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.usd

    app_utils.enable_extension("omni.kit.converter.gsplat")
    app.update()
    manifest = load_manifest(args.out_dir)
    stage_utils.create_new_stage()
    stage = omni.usd.get_context().get_stage()
    build(stage, manifest, args.out_dir, franka_url(app, manifest))
    out = args.output or os.path.join(args.out_dir, "twin_scene.usda")
    stage.GetRootLayer().Export(out)
    print(f"[Scene] saved {out}", flush=True)
    app.close()


if __name__ == "__main__":
    main()

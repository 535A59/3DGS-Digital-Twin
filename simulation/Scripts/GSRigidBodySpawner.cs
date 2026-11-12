// Assets/Editor/GS_HullRigidBodyFromAsset.cs
// 依赖：GaussianSplatting.Runtime   GK.ConvexHullCalculator (unity-quickhull)

using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using Unity.Mathematics;
using GaussianSplatting.Runtime;
using GK;

[ExecuteAlways, RequireComponent(typeof(GaussianSplatRenderer))]
public class GS_HullRigidBodyFromAsset : MonoBehaviour
{
    [Range(1024,20000)] public int samplePoints = 4000;
    public float mass = 1;
    public bool useGravity = false, isKinematic = true;

    const string kHullName = "_HullRigidBody";
    static readonly System.Random rng = new(2025);

#if UNITY_EDITOR
    void OnEnable()
    {
        if (Application.isPlaying) return;
        if (transform.parent && transform.parent.name.EndsWith(kHullName)) return;
        BuildHull();
    }
#endif

    void BuildHull()
    {
        var gs = GetComponent<GaussianSplatRenderer>();
        var asset = FindAsset(gs);
        if (asset == null) { Debug.LogError("GS asset not found"); return; }

        List<Vector3> pts = SamplePoints(asset, samplePoints);
        if (pts.Count < 4) { Debug.LogError("too few points"); return; }

        var verts = new List<Vector3>();
        var tris  = new List<int>();
        var norms = new List<Vector3>();
        new ConvexHullCalculator().GenerateHull(pts, true, ref verts, ref tris, ref norms);

        Mesh m = new() { name = asset.name + "_Hull" };
        m.SetVertices(verts); m.SetTriangles(tris, 0); m.SetNormals(norms);

        var host = new GameObject(name + kHullName);
        Undo.RegisterCreatedObjectUndo(host, "GS Hull");
        host.transform.SetParent(transform.parent, true);

        var mc = host.AddComponent<MeshCollider>();
        mc.sharedMesh = m;  mc.convex = true;

        var rb = host.AddComponent<Rigidbody>();
        rb.mass = mass; rb.useGravity = useGravity; rb.isKinematic = isKinematic;

        transform.SetParent(host.transform, true);
        Debug.Log($"Hull created  verts:{verts.Count}  tris:{tris.Count/3}");
    }

    // ───── helpers ─────
    static GaussianSplatAsset FindAsset(GaussianSplatRenderer r)
    {
        foreach (var f in r.GetType().GetFields(
                     BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic))
            if (f.FieldType == typeof(GaussianSplatAsset))
                return f.GetValue(r) as GaussianSplatAsset;

        foreach (var p in r.GetType().GetProperties(
                     BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic))
            if (p.PropertyType == typeof(GaussianSplatAsset) && p.GetIndexParameters().Length == 0)
                return p.GetValue(r) as GaussianSplatAsset;

        return null;
    }

    static List<Vector3> SamplePoints(GaussianSplatAsset a, int want)
    {
        int total = a.splatCount;
        int stride = GaussianSplatAsset.GetVectorSize(a.posFormat);
        byte[] buf = a.posData.bytes;
        float3 min = a.boundsMin, max = a.boundsMax;

        var res = new List<Vector3>(want);
        int step = Math.Max(1, total / want);

        for (int idx = 0; idx < total && res.Count < want; idx += step)
        {
            int i   = (idx + rng.Next(step)) % total;
            int off = i * stride;
            float3 n = Decode(buf, off, a.posFormat);
            res.Add(math.lerp(min, max, n));
        }
        return res;
    }

    static float3 Decode(byte[] d, int o, GaussianSplatAsset.VectorFormat f)
    {
        switch (f)
        {
            case GaussianSplatAsset.VectorFormat.Float32:
                return new float3(BitConverter.ToSingle(d,o),
                                  BitConverter.ToSingle(d,o+4),
                                  BitConverter.ToSingle(d,o+8));
            case GaussianSplatAsset.VectorFormat.Norm16:
                ulong v = BitConverter.ToUInt32(d,o) |
                         ((ulong)BitConverter.ToUInt16(d,o+4)<<32);
                return new float3((v&0xFFFF)/65535f,
                                  ((v>>16)&0xFFFF)/65535f,
                                  (v>>32)/65535f);
            case GaussianSplatAsset.VectorFormat.Norm11:
                uint w = BitConverter.ToUInt32(d,o);
                return new float3((w & 0x7FF)        /2047f,
                                  ((w>>11)&0x3FF)    /1023f,
                                  (w>>21)            /2047f);
            default: return float3.zero;
        }
    }
}

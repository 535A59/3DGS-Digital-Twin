// // GS2Mesh.cs – refined reconstruction
// // -------------------------------------------------------------
// // 目标：在仅依赖 GaussianSplatAsset(posData) 的情况下，尽量「干净」
// //      地把 3DGS 点云转为 Unity MeshCollider / 调试 Mesh。
// // 改进点：
// //   1. 恢复并可选启用三种过滤：Radius、AABB、质心距 K 剔除。
// //   2. 统计三维邻域密度，移除低密度离群点 (Statistical Outlier Removal)。
// //   3. 把体素下采样和其它过滤顺序调整：
// //      原始 → Radius/AABB/SOR → 质心距 K → Voxel → Mesh。
// //   4. 所有步骤都在同一 referenceSpace 局部坐标完成。
// // -------------------------------------------------------------
// using System.Collections.Generic;
// using UnityEngine;
// using Unity.Collections;
// using GaussianSplatting.Runtime;
// using UnityEngine.Rendering;

// [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
// public class GS2Mesh : MonoBehaviour
// {
//     [Header("Gaussian Splat Asset (.3dgs)")]
//     public GaussianSplatAsset gsAsset;

//     [Header("Reference Space (local coords)")]
//     public Transform referenceSpace;                 // null ⇒ this.transform

//     [Header("Radius Filter")]
//     public bool  useRadiusFilter = true;
//     public float minRadius = 0f;
//     public float maxRadius = 0.03f;

//     [Header("AABB Filter (ref-local)")]
//     public bool   useBoundsFilter = false;
//     public Vector3 boundsMin = new(-.15f,-.15f,-.15f);
//     public Vector3 boundsMax = new( .15f, .15f, .15f);

//     [Header("Statistical Outlier Removal")]
//     public bool useSOR = true;
//     [Tooltip("Number of neighbours for SOR")] public int sorK = 16;
//     [Tooltip("Distance σ multiplier")]       public float sorSigma = 2.5f;

//     [Header("Trim farthest from centroid")]  
//     public bool enableRemoveKFarthest = false;
//     public int  kRemove = 100;

//     [Header("Voxel Down-sample (ref-local)")]
//     public float voxelSize = 0.01f;          // ≤0 ⇒ off

//     [Header("Material")]
//     public Material meshMaterial;

//     /* ------------------------------------------------------- */
//     void Start()
//     {
//         if (!gsAsset) { Debug.LogError("[GS2Mesh] Missing GS Asset"); return; }
//         var refTf      = referenceSpace ? referenceSpace : transform;
//         var assetToRef = refTf.worldToLocalMatrix * transform.localToWorldMatrix;

//         // 1. Load raw points
//         NativeArray<float> posRaw = gsAsset.posData.GetData<float>();
//         int N = posRaw.Length / 4;

//         var pts  = new List<Vector3>(N);
//         for (int i=0;i<N;i++)
//         {
//             // basic radius filter
//             float radius = posRaw[4*i+3];
//             if (useRadiusFilter && (radius<minRadius || radius>maxRadius)) continue;
//             Vector3 pAsset = new Vector3(posRaw[4*i], posRaw[4*i+1], posRaw[4*i+2]);
//             Vector3 pRef   = assetToRef.MultiplyPoint3x4(pAsset);
//             pts.Add(pRef);
//         }
//         Debug.Log($"[GS2Mesh] after radius : {pts.Count}/{N}");

//         // 2. AABB filter
//         if (useBoundsFilter)
//         {
//             pts.RemoveAll(p => p.x<boundsMin.x || p.x>boundsMax.x ||
//                                p.y<boundsMin.y || p.y>boundsMax.y ||
//                                p.z<boundsMin.z || p.z>boundsMax.z);
//             Debug.Log($"[GS2Mesh] after AABB  : {pts.Count}");
//         }

//         // 3. Statistical Outlier Removal (简单实现：均值+方差半径)
//         if (useSOR && pts.Count> sorK)
//         {
//             RemoveStatisticalOutlier(pts, sorK, sorSigma);
//             Debug.Log($"[GS2Mesh] after SOR   : {pts.Count}");
//         }

//         // 4. remove farthest K from centroid
//         if (enableRemoveKFarthest && kRemove>0 && pts.Count>kRemove)
//         {
//             TrimKFarthest(pts, kRemove);
//             Debug.Log($"[GS2Mesh] after Ktrim : {pts.Count}");
//         }

//         // 5. voxel downsample
//         if (voxelSize>0f)
//         {
//             pts = VoxelGrid(pts, voxelSize);
//             Debug.Log($"[GS2Mesh] after voxel : {pts.Count}");
//         }

//         // 6. build colors (white) & mesh
//         var cols = new List<Color32>(pts.Count);
//         for(int i=0;i<pts.Count;i++) cols.Add(new Color32(255,255,255,255));
//         Mesh mesh = BuildMesh(pts, cols);
//         GetComponent<MeshFilter>().mesh = mesh;
//         if (meshMaterial) GetComponent<MeshRenderer>().material = meshMaterial;
//     }

//     /* -------------------- helpers ------------------------- */
//     static void RemoveStatisticalOutlier(List<Vector3> points, int k, float sigma)
//     {
//         // 简易 SOR：计算每点到最近 k 邻居距离均值，再按均值的
//         // 全局均值+sigma*std 阈值剔除。O(n^2) 但 n<50k 时够用。
//         int n = points.Count;
//         var distMeans = new float[n];
//         for(int i=0;i<n;i++)
//         {
//             // 收集 k 最近距离 (线性扫描)
//             var dists = new List<float>(n);
//             Vector3 pi = points[i];
//             for(int j=0;j<n;j++) if(i!=j) dists.Add((pi-points[j]).sqrMagnitude);
//             dists.Sort();
//             float sum=0; for(int t=0;t<k;t++) sum+=Mathf.Sqrt(dists[t]);
//             distMeans[i] = sum/k;
//         }
//         // 统计
//         float mean=0; foreach(float d in distMeans) mean+=d; mean/=n;
//         float var=0; foreach(float d in distMeans) var+=(d-mean)*(d-mean); var/=n;
//         float std = Mathf.Sqrt(var);
//         float thresh = mean + sigma*std;
//         // 去除
//         for(int i=n-1;i>=0;i--) if(distMeans[i]>thresh) points.RemoveAt(i);
//     }

//     static void TrimKFarthest(List<Vector3> pts, int k)
//     {
//         Vector3 centroid = Vector3.zero; foreach(var p in pts) centroid+=p; centroid/=pts.Count;
//         var tuples = new List<(float,int)>(pts.Count);
//         for(int i=0;i<pts.Count;i++) tuples.Add(((pts[i]-centroid).sqrMagnitude,i));
//         tuples.Sort((a,b)=>b.Item1.CompareTo(a.Item1));
//         var removeSet = new HashSet<int>();
//         for(int i=0;i<k;i++) removeSet.Add(tuples[i].Item2);
//         for(int i=pts.Count-1;i>=0;i--) if(removeSet.Contains(i)) pts.RemoveAt(i);
//     }

//     static List<Vector3> VoxelGrid(List<Vector3> src, float size)
//     {
//         var map=new Dictionary<(int,int,int),(Vector3 sum,int cnt)>(src.Count);
//         foreach(var p in src)
//         {
//             var key=((int)Mathf.Floor(p.x/size),(int)Mathf.Floor(p.y/size),(int)Mathf.Floor(p.z/size));
//             if(!map.TryGetValue(key,out var agg)) agg=(Vector3.zero,0);
//             agg.sum+=p; agg.cnt++; map[key]=agg;
//         }
//         var dst=new List<Vector3>(map.Count);
//         foreach(var kv in map){var (s,c)=kv.Value; dst.Add(s/c);}return dst;
//     }

//     static Mesh BuildMesh(List<Vector3> verts, List<Color32> cols)
//     {
//         Mesh m=new(); m.indexFormat=verts.Count>65000?IndexFormat.UInt32:IndexFormat.UInt16;
//         m.SetVertices(verts); m.SetColors(cols);
//         int[] idx=new int[verts.Count]; for(int i=0;i<idx.Length;i++) idx[i]=i;
//         m.SetIndices(idx,MeshTopology.Points,0); m.RecalculateBounds(); return m;
//     }
// }


// GS2MeshSimple.cs  —— 纯『点云→Mesh』
// -------------------------------------------
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using GaussianSplatting.Runtime;
using UnityEngine.Rendering;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
[ExecuteAlways]  // ← 在编辑器和运行时都执行
public class GS2MeshSimple : MonoBehaviour
{
    public enum SourceType { Gaussian, MeshAsset }

    [Header("输入类型 (二选一)")]
    public SourceType source = SourceType.MeshAsset;

    [Header("Gaussian Splat Asset (.3dgs)")]
    public GaussianSplatAsset gsAsset;

    [Header("Mesh 资源 (.obj / FBX / Prefab)")]
    public Mesh meshAsset;

    [Header("Optional Offsets")]
    public Transform offsetTransform;
    public Transform referenceSpace;

    [Header("Material (Scene 视图可见)")]
    public Material pointMaterial;

    // 每次在 Inspector 值改变时，会自动调用
    void OnValidate()
    {
        // 防止脚本在销毁时跑出错
        if (!this) return;
        BuildPointCloud();
    }

    // Play 模式下首次 Awake 也要生成一次
    void Awake()
    {
        BuildPointCloud();
    }

    void BuildPointCloud()
    {
        var mf = GetComponent<MeshFilter>();
        var mr = GetComponent<MeshRenderer>();

        // 1. 读取顶点
        var vertices = new List<Vector3>();
        if (source == SourceType.Gaussian)
        {
            if (!gsAsset) return;
            NativeArray<float> posRaw = gsAsset.posData.GetData<float>();
            int N = posRaw.Length / 4;
            vertices.Capacity = N;
            for (int i = 0; i < N; ++i)
                vertices.Add(new Vector3(
                    posRaw[4*i+0],
                    posRaw[4*i+1],
                    posRaw[4*i+2]));
        }
        else
        {
            if (!meshAsset) return;
            vertices.AddRange(meshAsset.vertices);
        }

        // 2. 坐标转换
        var refTf = referenceSpace ? referenceSpace : transform;
        Matrix4x4 a2w = offsetTransform
            ? offsetTransform.localToWorldMatrix
            : Matrix4x4.identity;
        Matrix4x4 w2r = refTf.worldToLocalMatrix;

        for (int i = 0; i < vertices.Count; ++i)
        {
            var pw = a2w.MultiplyPoint3x4(vertices[i]);
            vertices[i] = w2r.MultiplyPoint3x4(pw);
        }

        // 3. 构造点云 Mesh
        var cols = new List<Color32>(vertices.Count);
        for (int i = 0; i < vertices.Count; ++i)
            cols.Add(new Color32(255,255,255,255));

        var m = new Mesh {
            indexFormat = vertices.Count > 65_000 ? IndexFormat.UInt32 : IndexFormat.UInt16
        };
        m.SetVertices(vertices);
        m.SetColors(cols);
        var idx = new int[vertices.Count];
        for (int i = 0; i < idx.Length; ++i) idx[i] = i;
        m.SetIndices(idx, MeshTopology.Points, 0, false);
        m.RecalculateBounds();

        // 4. 绑定
        mf.sharedMesh = m;
        if (pointMaterial) mr.sharedMaterial = pointMaterial;
    }
}

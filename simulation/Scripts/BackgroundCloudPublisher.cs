// // Assets/Scripts/BackgroundCloudPublisher.cs
// using System;
// using System.Collections.Generic;
// using System.Threading;
// using Unity.Collections;
// using UnityEngine;
// using ROS2;
// using sensor_msgs.msg;
// using builtin_interfaces.msg;
// using std_msgs.msg;
// using GaussianSplatting.Runtime;

// [RequireComponent(typeof(ROS2UnityComponent))]
// public class BackgroundCloudPublisher : MonoBehaviour
// {
//     [Header("ROS2 参数")]
//     public string topicName   = "/background_cloud";
//     public string frameID     = "world";
//     [Tooltip("发布频率 (Hz)")]
//     public int    publishRate = 1;

//     [Header("Gaussian Splat Asset (优先)")]
//     public GaussianSplatAsset gsAsset;

//     [Header("坐标变换")]
//     [Tooltip("Splat 所在的 Table/GameObject 本身（脚本挂载对象）的 Transform")]
//     public Transform tableTransform;
//     [Tooltip("FR3 机器人 根节点 Transform，用于计算相对坐标")]
//     public Transform fr3Transform;
//     [Header("点云降采样（可选）")]
//     [Tooltip("体素网格的边长，单位同 Unity 世界坐标。若为 0 或者小于等于 0，则不做降采样。")]
//     public float voxelSize = 0.02f;
//     [Header("offset")]
//     [Tooltip("ROS 坐标系下的初始偏移量，单位同 Unity 世界坐标。")]
//     public float offset = 0.3f;

//     /* ---------- 运行时 ---------- */
//     ROS2UnityComponent      ros2;
//     ROS2Node                node;
//     IPublisher<PointCloud2> pub;
//     PointCloud2             msg;
//     Thread                  pubThread;

//     void Start()
//     {
//         ros2 = GetComponent<ROS2UnityComponent>();

//         // 1) 从资产提取 splat 的本地位置 & 颜色
//         List<Vector3> localPts;
//         List<Color32> cols;
//         ExtractFromGaussian(gsAsset, out localPts, out cols);

//         // 2) 坐标变换：本地 -> 世界 -> 相对 FR3
//         var relativePts = new List<Vector3>(localPts.Count);
//         for (int i = 0; i < localPts.Count; i++)
//         {
//             // (A) 本地(相对于 Table) → 世界坐标
//             Vector3 worldPt = tableTransform.TransformPoint(localPts[i]);

//             // (B) 世界 → 相对 FR3(root) 坐标
//             //     如果 fr3Transform 不为 null，就做 InverseTransformPoint；否则默认不变
//             Vector3 rel = fr3Transform != null
//                 ? fr3Transform.InverseTransformPoint(worldPt)
//                 : worldPt;

//             relativePts.Add(rel);
//         }

//         List<Vector3> dsPts = relativePts;
//         List<Color32> dsCols = cols;
//         if (voxelSize > 0f)
//         {
//             VoxelDownsample(relativePts, cols, voxelSize, out dsPts, out dsCols);
//             Debug.Log($"[BGCloud] 降采样后点数：{dsPts.Count}（voxelSize = {voxelSize}）");
//         }
//         else
//         {
//             Debug.Log($"[BGCloud] 未指定 voxelSize，跳过降采样，共 {relativePts.Count} pts");
//         }
//         // 3) 构造 PointCloud2
//         msg = BuildPointCloud2(dsPts, cols);

//         // 4) 后台发布
//         pubThread = new Thread(PublishLoop) { IsBackground = true };
//         pubThread.Start();

//         Debug.Log($"[BGCloud] 发布 {dsPts.Count} pts (相对 FR3) → {topicName} @{publishRate} Hz");
//     }

//     void OnApplicationQuit() => pubThread?.Abort();

//     void PublishLoop()
//     {
//         int intervalMs = Mathf.Max(1, 1000 / publishRate);
//         while (true)
//         {
//             if (ros2.Ok())
//             {
//                 if (node == null)
//                 {
//                     node = ros2.CreateNode("bg_cloud_node");
//                     pub  = node.CreateSensorPublisher<PointCloud2>(topicName);
//                 }
//                 var withHeader = msg as MessageWithHeader;
//                 node.clock.UpdateROSTimestamp(ref withHeader);
//                 pub.Publish(msg);
//             }
//             Thread.Sleep(intervalMs);
//         }
//     }

//         void VoxelDownsample(
//         List<Vector3> srcPts, 
//         List<Color32> srcCols, 
//         float voxelSize,
//         out List<Vector3> dstPts, 
//         out List<Color32> dstCols)
//     {
//         // 使用 Dictionary<顺序索引, 聚合数据> 将每个 voxel 内的所有点归并
//         var voxelMap = new Dictionary<Val3, VoxelAggregate>(capacity: srcPts.Count);
//         int N = srcPts.Count;

//         for (int i = 0; i < N; i++)
//         {
//             Vector3 p = srcPts[i];
//             // 计算当前点所在的体素索引 (ix,iy,iz)：
//             int ix = Mathf.FloorToInt(p.x / voxelSize);
//             int iy = Mathf.FloorToInt(p.y / voxelSize);
//             int iz = Mathf.FloorToInt(p.z / voxelSize);
//             var key = new Val3(ix, iy, iz);

//             Color32 c = srcCols[i];

//             if (!voxelMap.TryGetValue(key, out var agg))
//             {
//                 // 第一次遇到这个 voxel
//                 agg = new VoxelAggregate();
//                 agg.sumPos = p;
//                 agg.sumColor = new Vector3(c.r, c.g, c.b);
//                 agg.count = 1;
//                 voxelMap[key] = agg;
//             }
//             else
//             {
//                 // 累加到已有 aggregator
//                 agg.sumPos += p;
//                 agg.sumColor += new Vector3(c.r, c.g, c.b);
//                 agg.count += 1;
//                 voxelMap[key] = agg;
//             }
//         }

//         // 遍历字典，每个 voxel 生成一个平均值
//         dstPts  = new List<Vector3>(voxelMap.Count);
//         dstCols = new List<Color32>(voxelMap.Count);
//         foreach (var kv in voxelMap)
//         {
//             VoxelAggregate agg = kv.Value;
//             float invCnt = 1.0f / agg.count;
//             // 平均坐标
//             Vector3 avgPos = agg.sumPos * invCnt;
//             // 平均颜色 (需要四舍五入到 byte)
//             Vector3 avgColF = agg.sumColor * invCnt;
//             byte rc = (byte)Mathf.Clamp(Mathf.RoundToInt(avgColF.x), 0, 255);
//             byte gc = (byte)Mathf.Clamp(Mathf.RoundToInt(avgColF.y), 0, 255);
//             byte bc = (byte)Mathf.Clamp(Mathf.RoundToInt(avgColF.z), 0, 255);

//             dstPts.Add(avgPos);
//             dstCols.Add(new Color32(rc, gc, bc, 255));
//         }
//     }

//     /// <summary>
//     /// 用来作体素索引的三元组 (ix,iy,iz)。必须重写 Equals/GetHashCode 才能当字典 Key。
//     /// </summary>
//     struct Val3 : IEquatable<Val3>
//     {
//         public int x, y, z;
//         public Val3(int _x, int _y, int _z) { x = _x; y = _y; z = _z; }

//         public bool Equals(Val3 other)
//         {
//             return x == other.x && y == other.y && z == other.z;
//         }

//         public override bool Equals(object obj)
//         {
//             return obj is Val3 other && Equals(other);
//         }

//         public override int GetHashCode()
//         {
//             // 简单合并三个 int——如果 voxels 很多，必要时可以换成更好的哈希函数
//             unchecked
//             {
//                 int hash = 17;
//                 hash = hash * 31 + x;
//                 hash = hash * 31 + y;
//                 hash = hash * 31 + z;
//                 return hash;
//             }
//         }
//     }

//     /// <summary>
//     /// 存储一个 voxel 内所有点的聚合信息，最后求平均位置 & 平均颜色
//     /// </summary>
//     struct VoxelAggregate
//     {
//         public Vector3 sumPos;     // 所有点坐标之和
//         public Vector3 sumColor;   // 存成 Vector3(r,g,b) 之和，最后除以 count 再取整
//         public int count;          // 点的数量
//     }

//     // 把 Asset posData 解包成本地坐标点
//     static void ExtractFromGaussian(GaussianSplatAsset asset,
//                                     out List<Vector3> pts,
//                                     out List<Color32> cols)
//     {
//         var flat = asset.posData.GetData<float>();
//         if (flat.Length % 4 != 0)
//             throw new Exception($"posData 长度 {flat.Length} 不是 4 的倍数！");
//         int N = flat.Length / 4;
//         pts  = new List<Vector3>(N);
//         cols = new List<Color32>(N);
//         for (int i = 0; i < N; i++)
//         {
//             float x = flat[4*i + 0];
//             float y = flat[4*i + 1];
//             float z = flat[4*i + 2];
//             pts.Add(new Vector3(x, y, z));
//             cols.Add(new Color32(255,255,255,255));
//         }
//         Debug.Log($"[BGCloud] 从资产“{asset.name}”读取 {N} pts");
//     }

//     // 构造 PointCloud2 消息（XYZ + rgb）
//     PointCloud2 BuildPointCloud2(List<Vector3> pts, List<Color32> cols)
//     {
//         uint N        = (uint)pts.Count;
//         const uint ptBytes  = 16;
//         uint rowBytes = ptBytes * N;

//         var cloud = new PointCloud2
//         {
//             Height       = 1,
//             Width        = N,
//             Is_bigendian = false,
//             Is_dense     = false,
//             Point_step   = ptBytes,
//             Row_step     = rowBytes,
//             Data         = new byte[rowBytes],
//             Header       = new Header { Frame_id = frameID }
//         };
//         cloud.Fields = new[]
//         {
//             new PointField { Name="x",   Offset=0,  Datatype=7, Count=1 },
//             new PointField { Name="y",   Offset=4,  Datatype=7, Count=1 },
//             new PointField { Name="z",   Offset=8,  Datatype=7, Count=1 },
//             new PointField { Name="rgb", Offset=12, Datatype=7, Count=1 }
//         };
//        	Vector3 initialOffsetROS = new Vector3(0.0f, 0.0f, offset); 
//         for (int i = 0; i < pts.Count; ++i)
//         {
//             int b = i * (int)ptBytes;

//             Vector3 relU = pts[i];

//             // Unity → ROS 坐标重映射
//             float ux = relU.x, uy = relU.y, uz = relU.z;
//             float rx = uz;   // ROS x
//             float ry = -ux;  // ROS y
//             float rz = uy; 

//             rx = rx + initialOffsetROS.x;
//             ry = ry + initialOffsetROS.y;
//             rz = rz + initialOffsetROS.z;

//             Buffer.BlockCopy(BitConverter.GetBytes(rx), 0, cloud.Data, b + 0, 4);
//             Buffer.BlockCopy(BitConverter.GetBytes(ry), 0, cloud.Data, b + 4, 4);
//             Buffer.BlockCopy(BitConverter.GetBytes(rz), 0, cloud.Data, b + 8, 4);

//             uint rgb = ((uint)cols[i].r << 16) | ((uint)cols[i].g << 8) | cols[i].b;
//             float rgbF = BitConverter.ToSingle(BitConverter.GetBytes(rgb), 0);
//             Buffer.BlockCopy(BitConverter.GetBytes(rgbF), 0, cloud.Data, b + 12, 4);
//         }
//         return cloud;
//     }
// }
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;
using std_msgs.msg;

[RequireComponent(typeof(ROS2UnityComponent))]
public class BackgroundCloudPublisher : MonoBehaviour
{
    [Header("ROS2")]
    public string topicName = "/background_cloud";
    public string frameID = "world";
    public float publishRate = 1f; // Hz

    [Header("PLY source")]
    public TextAsset plyFile;
    public string plyPathOnDisk = "";

    private ROS2UnityComponent ros2;
    private ROS2Node node;
    private IPublisher<PointCloud2> pub;
    private PointCloud2 cloud;
    private float acc;

    void Start()
    {
        ros2 = GetComponent<ROS2UnityComponent>();

        var points = new List<Vector3>();
        if (!LoadPlyXyz(points))
        {
            enabled = false;
            return;
        }

        cloud = BuildCloudXyz(points);
        
        node = ros2.CreateNode("bg_cloud_node");
        pub = node.CreateSensorPublisher<PointCloud2>(topicName);
        Debug.Log($"[BGCloud] Loaded {points.Count} points. Publishing to '{topicName}' for RViz.");
    }

    void Update()
    {
        if (pub == null) return;
        acc += Time.deltaTime;
        if (acc >= 1f / Mathf.Max(publishRate, 0.01f))
        {
            acc = 0f;
            var hdr = cloud as MessageWithHeader;
            node.clock.UpdateROSTimestamp(ref hdr);
            pub.Publish(cloud);
        }
    }

    private bool LoadPlyXyz(List<Vector3> points)
    {
        byte[] raw;
        if (plyFile != null) raw = plyFile.bytes;
        else if (File.Exists(plyPathOnDisk)) raw = File.ReadAllBytes(plyPathOnDisk);
        else { Debug.LogError("[BGCloud] PLY file not found."); return false; }

        using var mem = new MemoryStream(raw);
        using var br = new BinaryReader(mem);

        bool binLE = false;
        int vertexCount = 0;
        string line;
        while ((line = ReadLine(br)) != null)
        {
            if (line.StartsWith("format binary_little_endian")) binLE = true;
            else if (line.StartsWith("element vertex")) vertexCount = int.Parse(line.Split(' ')[2]);
            else if (line == "end_header") break;
        }

        if (vertexCount == 0) { Debug.LogError("[BGCloud] Invalid PLY header."); return false; }
        
        var ci = CultureInfo.InvariantCulture;
        for (int i = 0; i < vertexCount; i++)
        {
            if (binLE)
            {
                if (mem.Position + 12 > mem.Length) break;
                points.Add(new Vector3(br.ReadSingle(), br.ReadSingle(), br.ReadSingle()));
            }
            else // ASCII
            {
                line = ReadLine(br);
                if (line == null) break;
                line = line.Trim();
                if (line.Length == 0 || line.StartsWith("#")) { i--; continue; }
                var tk = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (tk.Length < 3) continue;
                if (float.TryParse(tk[0], NumberStyles.Float, ci, out float x) &&
                    float.TryParse(tk[1], NumberStyles.Float, ci, out float y) &&
                    float.TryParse(tk[2], NumberStyles.Float, ci, out float z))
                {
                    points.Add(new Vector3(x, y, z));
                }
            }
        }
        return true;
    }
    
    private PointCloud2 BuildCloudXyz(List<Vector3> points)
    {
        const uint POINT_STEP = 12;
        uint N = (uint)points.Count;
        uint ROW_STEP = N * POINT_STEP;

        var pc = new PointCloud2
        {
            Height = 1,
            Width = N,
            Is_bigendian = false,
            Is_dense = true,
            Point_step = POINT_STEP,
            Row_step = ROW_STEP,
            Data = new byte[ROW_STEP],
            Header = new Header { Frame_id = frameID }
        };

        pc.Fields = new[] {
            new PointField { Name="x", Offset=0, Datatype=PointField.FLOAT32, Count=1 },
            new PointField { Name="y", Offset=4, Datatype=PointField.FLOAT32, Count=1 },
            new PointField { Name="z", Offset=8, Datatype=PointField.FLOAT32, Count=1 }
        };
        
        for (int i = 0; i < points.Count; i++)
        {
            int baseIndex = i * (int)POINT_STEP;
            Vector3 p = points[i];

            // ======================== 最终坐标系转换 ========================
            // 根据 Open3D 的正确显示，我们确认源文件是 Y-up。
            // 此转换为从 Y-up (PLY) 转到 Z-up (ROS)。
            float ros_x =  p.z;
            float ros_y = -p.x;
            float ros_z =  p.y;
            // ==========================================================

            Buffer.BlockCopy(BitConverter.GetBytes(ros_x), 0, pc.Data, baseIndex + 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(ros_y), 0, pc.Data, baseIndex + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(ros_z), 0, pc.Data, baseIndex + 8, 4);
        }
        return pc;
    }
    
    private static string ReadLine(BinaryReader br)
    {
        var bytes = new List<byte>();
        try
        {
            while (true)
            {
                byte c = br.ReadByte();
                if (c == '\n') break;
                if (c != '\r') bytes.Add(c);
            }
        }
        catch (EndOfStreamException) { }
        return bytes.Count == 0 ? null : System.Text.Encoding.ASCII.GetString(bytes.ToArray());
    }
}

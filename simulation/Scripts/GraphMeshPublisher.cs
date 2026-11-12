// // Assets/Scripts/GraphMeshPublisher.cs
// //-----------------------------------------------------------------
// // 动态把 MeshFilter.mesh 转成 ROS shape_msgs/Mesh，
// // 每帧或按速率在主线程里重新计算并发布
// //-----------------------------------------------------------------
// using System.Collections.Generic;
// using ROS2;
// using UnityEngine;

// // ROS 消息类型别名
// using RosMesh     = shape_msgs.msg.Mesh;
// using RosTriangle = shape_msgs.msg.MeshTriangle;
// using RosPoint    = geometry_msgs.msg.Point;

// [RequireComponent(typeof(ROS2UnityComponent))]
// [RequireComponent(typeof(MeshFilter))]
// public class GraphMeshPublisher : MonoBehaviour
// {
//     [Header("ROS2")]
//     public string topic = "/grasp_mesh";
//     [Tooltip("发布频率 (Hz)，0 表示每帧")]
//     public float publishHz = 10f;

//     [Header("坐标系")]
//     [Tooltip("Mesh 顶点在此坐标系下为本地坐标")]
//     public Transform baseFrame;
//     [Tooltip("世界→此坐标系后，再做 Unity→ROS 轴映射")]
//     public Transform targetFrame;

//     ROS2UnityComponent ros2;
//     ROS2Node           node;
//     IPublisher<RosMesh> publisher;

//     float publishInterval;
//     float timeSinceLast;

//     void Start()
//     {
//         if (baseFrame   == null) baseFrame   = transform;
//         if (targetFrame == null) targetFrame = transform;

//         ros2 = GetComponent<ROS2UnityComponent>();
//         node = ros2.CreateNode("grasp_mesh_pub_node");
//         publisher = node.CreatePublisher<RosMesh>(topic);

//         publishInterval = publishHz > 0 ? 1f / publishHz : 0f;
//         timeSinceLast = 0f;
//     }

//     void Update()
//     {
//         timeSinceLast += Time.deltaTime;
//         if (publishInterval == 0f || timeSinceLast >= publishInterval)
//         {
//             PublishMesh();
//             timeSinceLast = 0f;
//         }
//     }

//     void PublishMesh()
//     {
//         var mesh = GetComponent<MeshFilter>().mesh;
//         if (mesh == null || mesh.vertexCount == 0)
//             return;

//         var msg  = UnityToRosMesh(mesh, baseFrame, targetFrame);
//         publisher.Publish(msg);
//     }

//     static RosMesh UnityToRosMesh(Mesh mesh, Transform baseFrame, Transform targetFrame)
//     {
//         var verts    = mesh.vertices;
//         var rosVerts = new RosPoint[verts.Length];
//         for (int i = 0; i < verts.Length; i++)
//         {
//             // 1) local(baseFrame) → world
//             Vector3 w = baseFrame.TransformPoint(verts[i]);
//             // 2) world → local(targetFrame)
//             Vector3 t = targetFrame.InverseTransformPoint(w);
//             // 3) Unity→ROS 轴映射
//             rosVerts[i] = new RosPoint {
//                 X =  t.z,
//                 Y = -t.x,
//                 Z =  t.y
//             };
//         }

//         var idx  = mesh.GetIndices(0);
//         var tris = new List<RosTriangle>(idx.Length / 3);
//         for (int i = 0; i + 2 < idx.Length; i += 3)
//         {
//             var mt = new RosTriangle();
//             mt.Vertex_indices[0] = (uint)idx[i];
//             mt.Vertex_indices[1] = (uint)idx[i + 1];
//             mt.Vertex_indices[2] = (uint)idx[i + 2];
//             tris.Add(mt);
//         }

//         return new RosMesh {
//             Vertices  = rosVerts,
//             Triangles = tris.ToArray()
//         };
//     }
// }


// Assets/Scripts/GraphMeshPublisher.cs
using System.Collections.Generic;
using ROS2;
using UnityEngine;

// ROS 消息类型别名
using RosMesh     = shape_msgs.msg.Mesh;
using RosTriangle = shape_msgs.msg.MeshTriangle;
using RosPoint    = geometry_msgs.msg.Point;

[RequireComponent(typeof(ROS2UnityComponent))]
public class GraphMeshPublisher : MonoBehaviour
{
    [Header("ROS2")]
    public string topic = "/grasp_mesh";
    [Tooltip("发布频率 (Hz)，0 或负数 表示每帧")]
    public float publishHz = 10f;

    [Header("Mesh 来源")]
    [Tooltip("直接在项目中选择一个 Mesh (obj/fbx/imported MeshAsset)")]
    public Mesh   meshAsset;
    [Tooltip("对 Mesh 做位置/旋转/缩放补偿 (可选)")]
    public Transform meshOffset;

    [Header("杯子坐标 (几何中心将对齐到这里)")]
    [Tooltip("点云/网格的中心将被放到这个 Transform 的位置")]
    public Transform cup;

    [Header("机械臂 Base Link")]
    [Tooltip("通常对应 ROS 中的 base_link")]
    public Transform baseFrame;

    [Header("发布时的目标坐标系")]
    [Tooltip("计算 ROS 顶点坐标时，用来做轴映射的坐标系")]
    public Transform targetFrame;

    ROS2UnityComponent ros2;
    ROS2Node           node;
    IPublisher<RosMesh> publisher;
    float               publishInterval;
    float               timeSinceLast;

    void Start()
    {
        ros2 = GetComponent<ROS2UnityComponent>();
        node = ros2.CreateNode("grasp_mesh_pub_node");
        publisher = node.CreatePublisher<RosMesh>(topic);

        publishInterval = publishHz > 0 ? 1f / publishHz : 0f;
        timeSinceLast = 0f;

        if (baseFrame   == null) baseFrame   = transform;
        if (targetFrame == null) targetFrame = baseFrame;
    }

    void Update()
    {
        timeSinceLast += Time.deltaTime;
        if (publishInterval <= 0f || timeSinceLast >= publishInterval)
        {
            PublishMeshWithCentroidShift();
            timeSinceLast = 0f;
        }
    }

    private void PublishMeshWithCentroidShift()
    {
        if (meshAsset == null || cup == null) return;

        // 1) 把 meshAsset 顶点先按 meshOffset 变换到世界坐标
        var rawVerts   = meshAsset.vertices;
        int  nVerts    = rawVerts.Length;
        var worldVerts = new Vector3[nVerts];
        Matrix4x4 m2w  = meshOffset
            ? meshOffset.localToWorldMatrix
            : transform.localToWorldMatrix;
        for (int i = 0; i < nVerts; i++)
            worldVerts[i] = m2w.MultiplyPoint3x4(rawVerts[i]);

        // 2) 计算几何质心
        Vector3 centroidWorld = Vector3.zero;
        foreach (var v in worldVerts) centroidWorld += v;
        centroidWorld /= nVerts;

        // 3) 把几何质心对齐到 cup.position → 得到一个平移向量 t = cup - centroid
        Vector3 shift = cup.position - centroidWorld;

        // 4) 计算“杯子质心”在 baseFrame 下的位置（打印调试）
        Vector3 cupInBase = baseFrame.InverseTransformPoint(cup.position);
        Debug.Log($"[GraphMeshPublisher] 杯子质心在 FR3 base_link 下的坐标：{cupInBase}");

        // 5) 对所有顶点应用 shift（把点云/网格中心移到 cup），然后映射到 ROS 坐标系
        var rosVerts = new RosPoint[nVerts];
        for (int i = 0; i < nVerts; i++)
        {
            // world 顶点 → world + shift → targetFrame 空间 → ROS 轴映射
            Vector3 moved = worldVerts[i] + shift;
            Vector3 inTarget = targetFrame.InverseTransformPoint(moved);
            rosVerts[i] = new RosPoint {
                X =  inTarget.z,
                Y = -inTarget.x,
                Z =  inTarget.y
            };
        }

        // 6) 构造三角形索引（假设 MeshTopology = Triangles）
        var indices = meshAsset.GetIndices(0);
        var tris = new List<RosTriangle>(indices.Length / 3);
        for (int i = 0; i + 2 < indices.Length; i += 3)
        {
            var tri = new RosTriangle();
            tri.Vertex_indices[0] = (uint)indices[i];
            tri.Vertex_indices[1] = (uint)indices[i + 1];
            tri.Vertex_indices[2] = (uint)indices[i + 2];
            tris.Add(tri);
        }

        // 7) 发布消息
        var msg = new RosMesh {
            Vertices  = rosVerts,
            Triangles = tris.ToArray()
        };
        publisher.Publish(msg);
    }
}

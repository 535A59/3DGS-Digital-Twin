// using UnityEngine;
// using ROS2;

// // 1. 引入所有需要的消息类型
// using std_msgs.msg;
// using geometry_msgs.msg;
// using visualization_msgs.msg;

// // 2. 为所有可能冲突的类型创建别名，这是最稳妥的做法
// using UnityTransform = UnityEngine.Transform;
// using UnityVector3 = UnityEngine.Vector3;
// using UnityQuaternion = UnityEngine.Quaternion;
// using RosPose = geometry_msgs.msg.Pose;
// using RosPoint = geometry_msgs.msg.Point;
// using RosQuaternion = geometry_msgs.msg.Quaternion;
// using RosVector3 = geometry_msgs.msg.Vector3;


// [RequireComponent(typeof(ROS2UnityComponent))]
// public class MarkerPublisher : MonoBehaviour
// {
//     [Header("ROS 设置")]
//     [Tooltip("Marker发布到的话题名")]
//     public string topicName = "/my_marker";
//     [Tooltip("Marker所在的ROS坐标系名称")]
//     public string frameId = "world";

//     [Header("Unity 场景关联")]
//     [Tooltip("(可选) 如果Marker的位姿是相对另一个物体而言的，请把那个物体拖到这里。留空则使用世界坐标。")]
//     public UnityTransform parentFrameObject;

//     [Header("Marker 属性")]
//     [Tooltip("Marker的类型 (1=CUBE, 2=SPHERE, 3=ARROW...)")]
//     public int markerType = Marker.CUBE;

//     // --- 运行时变量 ---
//     private ROS2UnityComponent ros2Unity;
//     private ROS2Node ros2Node;
//     private IPublisher<Marker> pub;
//     private Marker markerMsg;

//     void Start()
//     {
//         // 在Start中只获取ROS2UnityComponent的引用
//         ros2Unity = GetComponent<ROS2UnityComponent>();
//     }

//     void Update()
//     {
//         // 3. 遵循官方示例的初始化模式：在Update中检查ros2是否就绪
//         if (ros2Unity.Ok() && ros2Node == null)
//         {
//             // 当ROS连接成功后，才创建Node、Publisher和消息对象
//             ros2Node = ros2Unity.CreateNode("unity_marker_publisher");
//             pub = ros2Node.CreatePublisher<Marker>(topicName);

//             markerMsg = new Marker
//             {
//                 Header = new Header { Frame_id = frameId },
//                 Ns = "unity_markers",
//                 Id = 0,
//                 Type = markerType,
//                 Action = Marker.ADD,
//                 Color = new ColorRGBA { R = 0.0f, G = 1.0f, B = 0.0f, A = 0.8f },
//                 Lifetime = new builtin_interfaces.msg.Duration { Sec = 0 }
//             };
//             Debug.Log($"[{ros2Node.name}] 将向 '{topicName}' 发布Marker。");
//         }

//         // 确保节点和发布者都已创建成功再执行后续逻辑
//         if (ros2Node == null || pub == null)
//         {
//             return;
//         }

//         // --- 计算相对位姿 ---
//         UnityVector3 posInFrame;
//         UnityQuaternion rotInFrame;

//         if (parentFrameObject != null)
//         {
//             posInFrame = parentFrameObject.InverseTransformPoint(transform.position);
//             rotInFrame = UnityQuaternion.Inverse(parentFrameObject.rotation) * transform.rotation;
            
//             Debug.Log($"the transform after parent pose is{posInFrame}");
//         }
//         else
//         {
//             posInFrame = transform.position;
//             rotInFrame = transform.rotation;
//         }
        
//         UnityVector3 unityScale = transform.lossyScale;

//         // --- 进行坐标系转换并填充消息 ---
//         // 使用别名来避免所有命名冲突
//         markerMsg.Pose = new RosPose
//         {
//             Position = new RosPoint
//             {
//                 X = posInFrame.z,
//                 Y = -posInFrame.x,
//                 Z = posInFrame.y
//             },
//             Orientation = new RosQuaternion
//             {
//                    X = rotInFrame.z, 
//                    Y = -rotInFrame.x, 
//                    Z = rotInFrame.y, 
//                    W = -rotInFrame.w
//             }
//         };

//         markerMsg.Scale = new RosVector3
//         {
//             X = unityScale.x,
//             Y = unityScale.y,
//             Z = unityScale.z
//         };
        
//         // 4. 使用正确的API来更新时间戳
//         ros2Node.clock.UpdateROSClockTime(markerMsg.Header.Stamp);
        
//         pub.Publish(markerMsg);
//     }
// }


using UnityEngine;
using ROS2;

// 1. 引入所有需要的消息类型
using std_msgs.msg;
using geometry_msgs.msg;
using visualization_msgs.msg;

// 2. 为所有可能冲突的类型创建别名
using UnityTransform = UnityEngine.Transform;
using UnityVector3 = UnityEngine.Vector3;
using UnityQuaternion = UnityEngine.Quaternion;
using RosPose = geometry_msgs.msg.Pose;
using RosPoint = geometry_msgs.msg.Point;
using RosQuaternion = geometry_msgs.msg.Quaternion;
using RosVector3 = geometry_msgs.msg.Vector3;

[RequireComponent(typeof(ROS2UnityComponent))]
public class MarkerPublisher : MonoBehaviour
{
    [Header("ROS 设置")]
    [Tooltip("Marker Node 名称")]
    public string node = "unity_marker_publisher";
    [Tooltip("Marker发布到的话题名")]
    public string topicName = "/my_marker";
    [Tooltip("Marker所在的ROS坐标系名称")]
    public string frameId = "world";

    [Header("Unity 场景关联")]
    [Tooltip("(可选) 如果Marker的位姿是相对另一个物体而言的，请把那个物体拖到这里。留空则使用世界坐标。")]
    public UnityTransform parentFrameObject;

    [Header("Marker 属性")]
    [Tooltip("Marker的类型 (1=CUBE, 2=SPHERE, 3=ARROW...)")]
    public int markerType = Marker.CUBE;

    // --- 运行时变量 ---
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<Marker> pub;
    private Marker markerMsg;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode(node);
            pub = ros2Node.CreatePublisher<Marker>(topicName);

            markerMsg = new Marker
            {
                Header = new Header { Frame_id = frameId },
                Ns = "unity_markers",
                Id = 0,
                Type = markerType,
                Action = Marker.ADD,
                Color = new ColorRGBA { R = 0.0f, G = 1.0f, B = 0.0f, A = 0.8f },
                Lifetime = new builtin_interfaces.msg.Duration { Sec = 0 }
            };
            Debug.Log($"[{ros2Node.name}] 将向 '{topicName}' 发布Marker。");
        }

        if (ros2Node == null || pub == null)
        {
            return;
        }

        // --- 获取 Unity 中的位姿和缩放 ---
        UnityVector3 localPosition;
        UnityQuaternion localRotation;
        UnityVector3 lossyScale = transform.lossyScale;

        if (parentFrameObject != null)
        {
            // 计算相对于父物体的局部坐标
            Matrix4x4 parentInverse = parentFrameObject.worldToLocalMatrix;
            Matrix4x4 localMatrix = parentInverse * transform.localToWorldMatrix;
            
            localPosition = localMatrix.GetColumn(3);
            localRotation = localMatrix.rotation;
        }
        else
        {
            // 使用世界坐标
            localPosition = transform.position;
            localRotation = transform.rotation;
        }

        // --- 进行坐标系转换并填充消息 ---
        // 这是最关键的一步：将 Unity 的位姿和缩放转换为 ROS 标准
        (markerMsg.Pose.Position, markerMsg.Pose.Orientation) = Unity2Ros(localPosition, localRotation);
        markerMsg.Scale = Unity2Ros(lossyScale);

        ros2Node.clock.UpdateROSClockTime(markerMsg.Header.Stamp);
        pub.Publish(markerMsg);
    }
    
    #region Coordinate System Conversion Helpers

    // 核心转换逻辑：从 Unity (左手, Y-up) 到 ROS (右手, Z-up)

    /// <summary>
    /// 转换 Vector3：位置或缩放
    /// </summary>
    public static RosVector3 Unity2Ros(UnityVector3 vector)
    {
        return new RosVector3
        {
            X = vector.z,
            Y = -vector.x,
            Z = vector.y
        };
    }
    
    /// <summary>
    /// 转换点
    /// </summary>
    public static RosPoint Unity2RosPoint(UnityVector3 vector)
    {
        return new RosPoint
        {
            X = vector.z,
            Y = -vector.x,
            Z = vector.y
        };
    }

    /// <summary>
    /// 转换 Quaternion：旋转
    /// </summary>
    public static RosQuaternion Unity2Ros(UnityQuaternion q)
    {
        // 这个转换同时改变了轴向和手性，是最可靠的方法
        return new RosQuaternion
        {
            X = -q.z,
            Y = q.x,
            Z = -q.y,
            W = q.w
        };
    }

    /// <summary>
    /// 一次性转换位姿 (位置和旋转)
    /// </summary>
    public static (RosPoint, RosQuaternion) Unity2Ros(UnityVector3 position, UnityQuaternion rotation)
    {
        return (Unity2RosPoint(position), Unity2Ros(rotation));
    }
    
    #endregion
}
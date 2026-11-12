// Assets/Ros2ForUnity/Scripts/ObjectPositionPublisher.cs
using UnityEngine;
using ROS2;
using std_msgs.msg;               // Header
using geometry_msgs.msg;          // PoseStamped, Pose, Point, Quaternion
using builtin_interfaces.msg;     // Time

// —— Unity 类型起别名，防止和 ROS 名字冲突 ——
using UnityVector3    = UnityEngine.Vector3;
using UnityQuaternion = UnityEngine.Quaternion;
using UnityTransform  = UnityEngine.Transform;

// —— ROS 消息类型起别名 ——
using RosPoint        = geometry_msgs.msg.Point;
using RosQuaternion   = geometry_msgs.msg.Quaternion;
using RosPose         = geometry_msgs.msg.Pose;
using RosPoseStamped  = geometry_msgs.msg.PoseStamped;
using RosHeader       = std_msgs.msg.Header;
using RosTime         = builtin_interfaces.msg.Time;

[RequireComponent(typeof(ROS2UnityComponent))]
public class DestinationPub : MonoBehaviour
{
    [Tooltip("参考坐标系 (想发相对 fr3_link0，直接把 fr3_link0 拖进来)")]
    public UnityTransform world;          // 只用来做变换

    ROS2UnityComponent ros;
    ROS2Node           node;
    IPublisher<RosPoseStamped> pub;

    void Start()
    {
        ros = GetComponent<ROS2UnityComponent>();
        if (world == null)
        {
            Debug.LogError("请把参考坐标系( world / fr3_link0 )拖到 Inspector！");
            enabled = false;
        }
    }

    void Update()
    {
        if (ros == null || !ros.Ok()) return;

        // 懒加载 Node + Publisher
        if (node == null)
        {
            node = ros.CreateNode("unity_destination_pose_pub");
            pub  = node.CreatePublisher<RosPoseStamped>("destination_position");
        }

        /*──────────── ① 把物体位姿变到“world”坐标系下 ────────────*/
        UnityVector3    pLocal = world.InverseTransformPoint(transform.position);
        UnityQuaternion qLocal = UnityQuaternion.Inverse(world.rotation) *
                                 transform.rotation;

        /*──────────── ② 填写 PoseStamped ────────────*/
        var stamp = new RosTime();
        node.clock.UpdateROSClockTime(stamp);

        var msg = new RosPoseStamped
        {
            Header = new RosHeader
            {
                Frame_id = world.name,   // 例如 “fr3_link0” 或 “world”
                Stamp    = stamp
            },
            Pose = new RosPose
            {
                Position = new RosPoint
                {
                    X = pLocal.z,
                    Y = -pLocal.x,
                    Z = pLocal.y
                },
                Orientation = new RosQuaternion
                {
                    X = qLocal.z,
                    Y = -qLocal.x,
                    Z = qLocal.y,
                    W = qLocal.w
                }
            }
        };

        pub.Publish(msg);
    }
}


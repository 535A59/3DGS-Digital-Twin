using UnityEngine;
using ROS2;
using geometry_msgs.msg;          // PoseStamped
using builtin_interfaces.msg;     // Time

using UnityVector3    = UnityEngine.Vector3;
using UnityQuaternion = UnityEngine.Quaternion;
using UnityTransform  = UnityEngine.Transform;

[RequireComponent(typeof(ROS2UnityComponent))]
public class ObjectPositionSubscriber : MonoBehaviour
{
    [Header("参考坐标系 (通常是 fr3_link0)")]
    public UnityTransform world;

    [Tooltip("Topic name (默认 /cup_pose)")]
    public string topicName = "/cup_pose";

    [Tooltip("ROS Node name (留空默认 cup_pose_sub)")]
    public string nodeName  = "cup_pose_sub";

    private ROS2UnityComponent ros;
    private ROS2Node           node;
    private ISubscription<PoseStamped> sub;
    private PoseStamped latest;

    void Start()
    {
        if (world == null)
        {
            Debug.LogError("[CupPoseSubscriber] 请在 Inspector 指定参考坐标系 Transform！");
            enabled = false;
            return;
        }

        ros = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        /* -------- 懒加载 ROS Node 与订阅 -------- */
        if (ros.Ok() && node == null)
        {
            if (string.IsNullOrEmpty(nodeName))  nodeName = "cup_pose_sub";
            if (string.IsNullOrEmpty(topicName)) topicName = "/cup_pose";

            node = ros.CreateNode(nodeName);
            sub  = node.CreateSubscription<PoseStamped>(
                topicName,
                msg => { latest = msg; });
            Debug.Log($"[CupPoseSubscriber] 订阅 {topicName}");
        }

        /* -------- 有新消息就更新杯子 Transform -------- */
        if (latest != null)
        {
            var pRos = latest.Pose.Position;          // Point
            var qRos = latest.Pose.Orientation;       // Quaternion

            // ① ROS → Unity 轴映射  (x y z) → (Z, -X,  Y)
            UnityVector3 posLocal = new UnityVector3(
                (float)-pRos.Y,
                (float)pRos.Z,
                (float)pRos.X);

            // ② 再映射到 Unity 全局
            transform.position = world.TransformPoint(posLocal);
        }
    }
}

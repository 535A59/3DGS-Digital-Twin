using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;

public class JointStateSubscriber : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<JointState> jointStateSub;

    private Dictionary<string, ArticulationBody> jointMap = new Dictionary<string, ArticulationBody>();

    private Dictionary<string, string> jointNameToLinkName = new Dictionary<string, string>
    { 
        {"panda_joint1", "fr3_link1"},
        {"panda_joint2", "fr3_link2"},
        {"panda_joint3", "fr3_link3"},
        {"panda_joint4", "fr3_link4"},
        {"panda_joint5", "fr3_link5"},
        {"panda_joint6", "fr3_link6"},
        {"panda_joint7", "fr3_link7"},
        {"panda_finger_joint1", "fr3_rightfinger"},
        {"panda_finger_joint2", "fr3_leftfinger"}
    };

    private ConcurrentQueue<JointCommand> commandQueue = new ConcurrentQueue<JointCommand>();

    private struct JointCommand
    {
        public string linkName;
        public float angleDeg;
    }

    void Start()
    {
        Time.fixedDeltaTime = 0.01f;
        ros2Unity = GetComponent<ROS2UnityComponent>();

        foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>(true))
        {
            jointMap[body.name] = body;
            Debug.Log($"[Unity] 已注册关节: {body.name}");
        }

        if (ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("ros2_unity_joint_state_listener");

            jointStateSub = ros2Node.CreateSubscription<JointState>(
                "/joint_states",
                msg =>
                {
                    try
                    {
                        for (int i = 0; i < msg.Name.Length && i < msg.Position.Length; i++)
                        {
                            string rosJointName = msg.Name[i];
                            double position = msg.Position[i];

                            if (jointNameToLinkName.TryGetValue(rosJointName, out string unityLinkName))
                            {
                                float angleDeg = (float)(position * Mathf.Rad2Deg);
                                commandQueue.Enqueue(new JointCommand
                                {
                                    linkName = unityLinkName,
                                    angleDeg = angleDeg
                                });
                            }
                            else
                            {
                                Debug.LogWarning($"[ROS2] 未定义关节映射: {rosJointName}");
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"[ROS2] JointState 回调异常: {ex.Message}");
                    }
                });

            Debug.Log("[ROS2] JointStateSubscriber 已启动并订阅 /joint_states");
        }
        else
        {
            Debug.LogError("[ROS2] ros2Unity 尚未准备好，订阅失败！");
        }
    }

    void Update()
    {
        while (commandQueue.TryDequeue(out JointCommand cmd))
        {
            if (jointMap.TryGetValue(cmd.linkName, out ArticulationBody joint))
            {
                // Unity jointPosition[0] 是弧度，转成度
                float currentDeg = joint.jointPosition[0] * Mathf.Rad2Deg;
                // xDrive.target 本身就是度
                float targetDeg  = joint.xDrive.target;

                Debug.Log($"[TEST] 当前角度: {currentDeg:F2}° (目标: {cmd.angleDeg:F2}°)");

                // 把收到的 cmd.angleDeg（度）写回去
                var drive = joint.xDrive;
                drive.target = cmd.angleDeg;
                joint.xDrive = drive;
            }
            else
            {
                Debug.LogWarning($"[Unity] 未找到 ArticulationBody: {cmd.linkName}");
            }
        }
    }
}

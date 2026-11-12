using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;
using builtin_interfaces.msg;
using TimeMsg = builtin_interfaces.msg.Time;

[RequireComponent(typeof(ROS2UnityComponent))]
public class JointStatePublisher : MonoBehaviour
{
    [Tooltip("Drag all ArticulationBody components here (including fixed joints).")]
    public ArticulationBody[] joints;

    // Joint limits taken from the URDF <limit> tags (in radians or meters)
    private readonly Dictionary<string, Vector2> limits = new Dictionary<string, Vector2>()
    {
        { "fr3_joint1",        new Vector2(-2.7437f,  2.7437f) },
        { "fr3_joint2",        new Vector2(-1.7837f,  1.7837f) },
        { "fr3_joint3",        new Vector2(-2.9007f,  2.9007f) },
        { "fr3_joint4",        new Vector2(-3.0421f, -0.1518f) },
        { "fr3_joint5",        new Vector2(-2.8065f,  2.8065f) },
        { "fr3_joint6",        new Vector2( 0.5445f,  4.5169f) },
        { "fr3_joint7",        new Vector2(-3.0159f,  3.0159f) },
        { "fr3_finger_joint1", new Vector2( 0.0f,     0.04f  ) },
        { "fr3_finger_joint2", new Vector2( 0.0f,     0.04f  ) },
    };

    private ArticulationBody[] armJoints;
    private ArticulationBody[] leftFingers;
    private ArticulationBody[] rightFingers;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node           ros2Node;
    private IPublisher<JointState> publisher;

    void Awake()
    {
        if (joints == null || joints.Length == 0)
        {
            Debug.LogError("[JointStatePublisher] No joints assigned in the Inspector!");
            enabled = false;
        }
    }

    void Start()
    {
        // Filter out only the movable arm joints (non-fixed fr3_link*)
        armJoints    = joints.Where(j => j.jointType != ArticulationJointType.FixedJoint
                                       && j.name.StartsWith("fr3_link")).ToArray();
        leftFingers  = joints.Where(j => j.name == "fr3_leftfinger").ToArray();
        rightFingers = joints.Where(j => j.name == "fr3_rightfinger").ToArray();

        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("[JointStatePublisher] Missing ROS2UnityComponent!");
            enabled = false;
        }
    }

    void FixedUpdate()
    {
        if (ros2Unity == null || !ros2Unity.Ok()) 
            return;

        // Lazy init of the ROS 2 node and publisher
        if (ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("unity_joint_state_pub");
            publisher = ros2Node.CreatePublisher<JointState>("/joint_states");
            if (publisher == null)
            {
                Debug.LogError("[JointStatePublisher] Failed to create /joint_states publisher");
                enabled = false;
                return;
            }
        }

        // Build and populate the JointState message
        int jointCount = armJoints.Length + 2;  // plus one finger joint
        var msg = new JointState
        {
            Name     = new string[jointCount],
            Position = new double[jointCount],
            Velocity = new double[jointCount],
            Effort   = new double[jointCount],
            Header   = new std_msgs.msg.Header { Frame_id = "world" }
        };

        // 1) Arm joints
        for (int i = 0; i < armJoints.Length; i++)
        {
            var body = armJoints[i];
            string name = body.name.Replace("link", "joint");
            double pos  = body.jointPosition[0];
            if (limits.TryGetValue(name, out Vector2 lim))
            {
                pos = Mathf.Clamp((float)pos, lim.x, lim.y);
            }
            msg.Name[i]     = name;
            msg.Position[i] = pos;
        }

        // 2) Gripper finger joint (average of left/right)
        // int idx = armJoints.Length;
        // double sum = leftFingers.Concat(rightFingers).Sum(b => b.jointPosition[0]);
        // int n = leftFingers.Length + rightFingers.Length;
        // double fingerPos = (n > 0) ? sum / n : 0.0;
        // if (limits.TryGetValue("fr3_finger_joint1", out Vector2 flim))
        // {
        //     fingerPos = Mathf.Clamp((float)fingerPos, flim.x, flim.y);
        // }
        // msg.Name[idx]     = "fr3_finger_joint1";
        // msg.Position[idx] = fingerPos;
        int idxL = armJoints.Length;
        {
            string name = "fr3_finger_joint1";
            double pos = (leftFingers.Length > 0) ? leftFingers[0].jointPosition[0] : 0.0;
            if (limits.TryGetValue(name, out Vector2 lim))
                pos = Mathf.Clamp((float)pos, lim.x, lim.y);

            msg.Name[idxL]     = name;
            msg.Position[idxL] = pos;
        }

        // 3) 右指关节 fr3_finger_joint2
        int idxR = armJoints.Length + 1;
        {
            string name = "fr3_finger_joint2";
            double pos = (rightFingers.Length > 0) ? rightFingers[0].jointPosition[0] : 0.0;
            if (limits.TryGetValue(name, out Vector2 lim))
                pos = Mathf.Clamp((float)pos, lim.x, lim.y);

            msg.Name[idxR]     = name;
            msg.Position[idxR] = pos;
        }

        // 3) Stamp the message
        var stamp = new TimeMsg();
        ros2Node.clock.UpdateROSClockTime(stamp);
        msg.Header.Stamp = stamp;

        // 4) Publish
        publisher.Publish(msg);
    }

    void OnDisable()
    {
        // Clean up
        publisher = null;
        ros2Node = null;
    }
}

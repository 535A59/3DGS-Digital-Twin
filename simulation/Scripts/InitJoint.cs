using System.Collections;
using UnityEngine;

public class InitJoint : MonoBehaviour
{
    public float stiffness   = 10000f;
    public float damping     = 100f;
    public float forceLimit  = 100f;
    public float delaySeconds = 1f;

    void Start()
    {
        StartCoroutine(InitAfterDelay());
    }

    IEnumerator InitAfterDelay()
    {
        yield return new WaitForSeconds(delaySeconds);  // 等 URDF 加载完

        foreach (var joint in GetComponentsInChildren<ArticulationBody>(true))
        {
            if (joint.jointType != ArticulationJointType.RevoluteJoint &&
                joint.jointType != ArticulationJointType.PrismaticJoint)
                continue;

            var drive = joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            drive.target = joint.jointPosition[0];     // 设初值
            drive.targetVelocity = 0f;
            drive.driveType = ArticulationDriveType.Target;
            joint.xDrive = drive;
            joint.WakeUp(); // 保证立即起效

            Debug.Log($"[InitJoint] 设置关节 {joint.name} 成功");
        }
    }
}

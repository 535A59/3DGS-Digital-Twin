using UnityEngine;

public class Transformation : MonoBehaviour
{
    // === 公开变量 (可以在Unity编辑器的Inspector窗口中输入4x4矩阵) ===

    [Header("4x4 变换矩阵 (行主序)")]
    [Tooltip("矩阵的第一行 (X轴向量和位置X)")]
    public Vector4 matrixRow0 = new Vector4(1f, 0f, 0f, 0f);

    [Tooltip("矩阵的第二行 (Y轴向量和位置Y)")]
    public Vector4 matrixRow1 = new Vector4(0f, 1f, 0f, 0f);

    [Tooltip("矩阵的第三行 (Z轴向量和位置Z)")]
    public Vector4 matrixRow2 = new Vector4(0f, 0f, 1f, 0f);

    [Tooltip("矩阵的第四行 (通常是 0, 0, 0, 1)")]
    public Vector4 matrixRow3 = new Vector4(0f, 0f, 0f, 1f);


    /// <summary>
    /// Awake() 在所有Start()函数之前被调用，
    /// 是在“运行前”应用初始状态的最佳位置。
    /// </summary>
    void Awake()
    {
        ApplyTransformFromMatrix();
    }

    /// <summary>
    /// 从公开的Vector4变量构建一个4x4矩阵，并将其应用到物体的Transform上。
    /// </summary>
    public void ApplyTransformFromMatrix()
    {
        // 1. 从四个Vector4行向量构建一个Matrix4x4对象
        Matrix4x4 transformationMatrix = new Matrix4x4();
        transformationMatrix.SetRow(0, matrixRow0);
        transformationMatrix.SetRow(1, matrixRow1);
        transformationMatrix.SetRow(2, matrixRow2);
        transformationMatrix.SetRow(3, matrixRow3);

        // 2. 从矩阵中分解出 位置、旋转 和 缩放
        //    Unity的Matrix4x4结构体提供了方便的属性来直接获取这些值

        // 获取位置 (矩阵的最后一列)
        Vector3 position = transformationMatrix.GetColumn(3);

        // 获取旋转 (以四元数表示)
        Quaternion rotation = transformationMatrix.rotation;

        // 获取缩放 (矩阵的三个轴向量的长度)
        Vector3 scale = transformationMatrix.lossyScale;

        // 3. 将分解出的值应用到物体的Transform组件上
        transform.position = position;
        transform.rotation = rotation;
        transform.localScale = scale;
        
        Debug.Log("已成功从4x4矩阵应用变换！");
    }
}

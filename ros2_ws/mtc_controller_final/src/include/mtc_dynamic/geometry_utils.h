#pragma once // 防止头文件被重复包含

#include <string>
#include <vector>
#include <utility> // for std::pair
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// 将工具函数放在一个新的命名空间下，以避免命名冲突
namespace mtc_dynamic {
namespace utils {

/**
 * @brief 从PLY文件中计算圆柱体的高度和直径
 *
 * @param ply_filepath PLY文件的完整路径
 * @return std::pair<double, double> 包含高度(first)和直径(second)
 */
std::pair<double, double> calculateCylinderDimensions(const std::string& ply_filepath);

/**
 * @brief 创建一个圆柱体形状的碰撞体消息
 *
 * @param object_id 物体的ID
 * @param frame_id 物体所在的参考坐标系
 * @param pose 物体底部中心的位姿
 * @param height 圆柱体的高度
 * @param radius 圆柱体的半径
 * @return moveit_msgs::msg::CollisionObject 创建好的碰撞体
 */
moveit_msgs::msg::CollisionObject createCylinderObject(const std::string& object_id,
                                                       const std::string& frame_id,
                                                       const geometry_msgs::msg::Pose& pose,
                                                       double height,
                                                       double radius);

} // namespace utils
} // namespace mtc_dynamic
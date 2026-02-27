#pragma once
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mtc_dynamic/pick_place_demo_parameters.hpp>
#include <atomic>
#include <memory>
// MTC, Octomap, and PlanningSceneMonitor headers
#include <moveit/task_constructor/task.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// TF2 (Transform) related headers
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mtc_dynamic {

class PickPlaceTask {
public:
    PickPlaceTask(const std::string& task_name);
    bool init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& params);
    bool plan(const std::size_t max_solutions = 1);
    bool execute();

private:
    // 回调函数
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

    void object1PositionCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg);
    void target1PositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void object2PositionCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg);
    void target2PositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void object3PositionCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg);
    void target3PositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void publishAttachedObjectPose(); // 用于发布抓取后物体位姿的函数

    // 核心MTC对象和ROS节点
    std::string task_name_;
    moveit::task_constructor::TaskPtr task_;
    rclcpp::Node::SharedPtr node_;
    pick_place_task_demo::Params params_;

    // ROS通信接口
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr object1_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target1_position_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr object2_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target2_position_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr object3_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target3_position_sub_;
    rclcpp::TimerBase::SharedPtr pose_update_timer_;

    // 存储从话题接收的数据
    geometry_msgs::msg::Pose object1_position_;
    geometry_msgs::msg::Vector3 object1_scale_;
    geometry_msgs::msg::Pose target1_pose_;
    double object_height_ = 0.1;  // Default height, can be updated from marker

    geometry_msgs::msg::Pose object2_position_;
    geometry_msgs::msg::Vector3 object2_scale_;
    geometry_msgs::msg::Pose target2_pose_;

    geometry_msgs::msg::Pose object3_position_;
    geometry_msgs::msg::Vector3 object3_scale_;
    geometry_msgs::msg::Pose target3_pose_;

    // 线程安全的状态标志位
    std::atomic<bool> received_octomap_{false};
    std::atomic<bool> received_object1_position_{false};
    std::atomic<bool> received_target1_pose_{false};
    std::atomic<bool> received_object2_position_{false};
    std::atomic<bool> received_target2_pose_{false};
    std::atomic<bool> received_object3_position_{false};
    std::atomic<bool> received_target3_pose_{false};

    // 帮助对象
    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace mtc_dynamic
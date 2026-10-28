#pragma once
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mtc_dynamic/pick_place_demo_parameters.hpp>
#include <chrono>
#include <map>
#include <mutex>
#include <moveit/task_constructor/task.h>

namespace mtc_dynamic {

// Pick & place of K objects (params.object_names), one MTC task per object:
//   init() once, then for each object: plan(name) + execute().
class PickPlaceTask {
public:
    PickPlaceTask(const std::string& task_name);
    // Subscribe to all object pose/target topics, wait for the first data, spawn env + object meshes.
    bool init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& params);
    // Re-read the object's latest pose/target, update its collision object, build + plan its sub-task.
    bool plan(const std::string& object_name, const std::size_t max_solutions = 1);
    bool execute();

private:
    using Clock = std::chrono::steady_clock;
    struct ObjectData {
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub;
        geometry_msgs::msg::Pose pose;    // body pose in world (latest)
        geometry_msgs::msg::Pose target;  // target body pose in world (latest)
        Clock::time_point pose_time{}, target_time{};
    };
    // Block until every object has a pose and a target received after `since` (false on timeout/shutdown).
    bool waitForData(const std::vector<std::string>& names, Clock::time_point since, double timeout_s);
    void buildTask(const std::string& name, const geometry_msgs::msg::Pose& body,
                   const geometry_msgs::msg::Pose& target);

    std::string task_name_;
    moveit::task_constructor::TaskPtr task_;
    rclcpp::Node::SharedPtr node_;
    pick_place_task_demo::Params params_;

    std::mutex mutex_;  // guards objects_ data written by the subscription callbacks
    std::map<std::string, ObjectData> objects_;
};

} // namespace mtc_dynamic

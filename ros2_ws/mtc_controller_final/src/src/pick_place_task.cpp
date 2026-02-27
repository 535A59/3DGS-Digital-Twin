#include "mtc_dynamic/pick_place_task.h"
#include "mtc_dynamic/geometry_utils.h"

// C++ and ROS standard libraries
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap_msgs/msg/octomap.hpp>

// MoveIt and MTC libraries
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/introspection.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Headers required for loading mesh files
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
// 新增: 解决类型不匹配问题所需的头文件
#include <boost/variant.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dynamic");

namespace {
// Helper function to convert a vector of doubles to an Eigen transform
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
    return Eigen::Translation3d(values[0], values[1], values[2]) *
           Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}
moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& object_id,
                                                        const std::string& frame_id,
                                                        const geometry_msgs::msg::Pose& pose,
                                                        double height,
                                                        double radius) {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = frame_id;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CUBE;
    object.primitives[0].dimensions = { 0.025, 0.025 }; // 使用传入的动态尺寸
    geometry_msgs::msg::Pose object_pose = pose;
    object_pose.position.z += 0.5 * height; // 使用传入的动态高度调整Z轴
    object.primitive_poses.push_back(object_pose);
    return object;
}

// Helper function to create a mesh collision object
moveit_msgs::msg::CollisionObject createMeshCollisionObject(const std::string& object_id,
                                                             const std::string& frame_id,
                                                             const geometry_msgs::msg::Pose& pose,
                                                             const geometry_msgs::msg::Vector3& scale,
                                                             const std::string& mesh_path) {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = frame_id;
    shapes::Shape* mesh_shape = shapes::createMeshFromResource("file://" + mesh_path);
    if (!mesh_shape) {
        RCLCPP_ERROR(LOGGER, "Failed to load mesh from resource: %s", mesh_path.c_str());
        return object;
    }

    auto* mesh = dynamic_cast<shapes::Mesh*>(mesh_shape);
    if (mesh) {
        mesh->scale(scale.x*0.7,scale.y*0.7,scale.z*0.7);
        RCLCPP_INFO(LOGGER, "Applied fixed scale to mesh: %.2f", scale.x);
    }

    shapes::ShapeMsg shape_msg_variant;
    if (!shapes::constructMsgFromShape(mesh_shape, shape_msg_variant)) {
        RCLCPP_ERROR(LOGGER, "Failed to construct message from shape");
        delete mesh_shape;
        return object;
    }
    delete mesh_shape;

    try {
        shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg_variant);
        object.meshes.push_back(mesh_msg);
        object.mesh_poses.push_back(pose);
    } catch (const boost::bad_get& e) {
        RCLCPP_ERROR(LOGGER, "Shape is not a mesh as expected: %s", e.what());
        return object;
    }
    RCLCPP_INFO(LOGGER, "Spawning MESH object from '%s'", mesh_path.c_str());
    return object;
}

// Helper function to apply a collision object to the planning scene
void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::msg::CollisionObject& object) {
    if (!psi.applyCollisionObject(object))
        throw std::runtime_error("Failed to spawn object: " + object.id);
}
} // namespace



namespace mtc_dynamic {

using namespace moveit::task_constructor;

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& params) {
    RCLCPP_INFO(LOGGER, "Initializing task pipeline");
    node_ = node;
    params_ = params;

    moveit::planning_interface::PlanningSceneInterface psi;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    
    
    rclcpp::QoS subscriber_qos(10);
    //octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", rclcpp::QoS(1).reliable().transient_local(), std::bind(&PickPlaceTask::octomapCallback, this, std::placeholders::_1));
    object1_position_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>("/box_position", subscriber_qos, std::bind(&PickPlaceTask::object1PositionCallback, this, std::placeholders::_1));
    target1_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/box_target_position", subscriber_qos, std::bind(&PickPlaceTask::target1PositionCallback, this, std::placeholders::_1));

    object2_position_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>("/hammer_position", subscriber_qos, std::bind(&PickPlaceTask::object2PositionCallback, this, std::placeholders::_1));
    target2_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/hammer_target_position", subscriber_qos, std::bind(&PickPlaceTask::target2PositionCallback, this, std::placeholders::_1));

    object3_position_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>("/cube_position", subscriber_qos, std::bind(&PickPlaceTask::object3PositionCallback, this, std::placeholders::_1));
    target3_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/cube_target_position", subscriber_qos, std::bind(&PickPlaceTask::target3PositionCallback, this, std::placeholders::_1));
    
    scene_monitor_->startSceneMonitor();
    scene_monitor_->startStateMonitor();
    using namespace std::chrono_literals;
    pose_update_timer_ = node_->create_wall_timer(33ms, std::bind(&PickPlaceTask::publishAttachedObjectPose, this));

    RCLCPP_INFO(LOGGER, "Waiting for initial data...");
    while (rclcpp::ok() && ( 
        // !received_octomap_          ||
        !received_object1_position_ || 
        !received_target1_pose_     ||
        !received_object2_position_ || 
        !received_target2_pose_     ||
        !received_object3_position_ || 
        !received_target3_pose_     
    
    
    )) {
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok()) {
        RCLCPP_WARN(LOGGER, "Shutdown signal received, skipping task setup.");
        return false;
    }
    RCLCPP_INFO(LOGGER, "Received all required initial data.");

    if (params_.object1_mesh_path.empty()) {
        RCLCPP_ERROR(LOGGER, "Parameter 'object1_mesh_path' is not set.");
        return false;
    }
    spawnObject(psi, createMeshCollisionObject(params_.object1_name,
                                               params_.object_reference_frame,
                                               object1_position_,
                                               object1_scale_,
                                               params_.object1_mesh_path));

    spawnObject(psi, createMeshCollisionObject(params_.object2_name,
                                               params_.object_reference_frame,
                                               object2_position_,
                                               object2_scale_,
                                               params_.object2_mesh_path));


    task_.reset();
    task_ = std::make_shared<Task>();
    Task& t = *task_;
    t.stages()->setName(task_name_);
    t.loadRobotModel(node);

    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

    t.setProperty("group", params.arm_group_name);
    t.setProperty("eef", params.eef_name);
    t.setProperty("hand", params.hand_group_name);
    t.setProperty("hand_grasping_frame", params.hand_frame);
    t.setProperty("ik_frame", params.hand_frame);

    

    {
        auto cs = std::make_unique<stages::CurrentState>("current state");
        auto filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(cs)); 
        filter->setPredicate([object = params.object1_name](const SolutionBase& s, std::string& comment){
            if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                comment = "object '" + object + "' already attached";
                return false;
            }
            return true;
        });
        t.add(std::move(filter));
    }


    Stage* initial_state_ptr = nullptr;
    {
        auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->setGroup(params.hand_group_name);
        stage->setGoal(params.hand_open_pose);
        initial_state_ptr = stage.get();
        t.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::Connect>("move to pick", stages::Connect::GroupPlannerVector{{params.arm_group_name,sampling_planner}});
      stage->setTimeout(1.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    Stage* pick_stage_ptr = nullptr;
    {
        auto grasp = std::make_unique<SerialContainer>("pick object");
        t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });


        {
            auto stage = std::make_unique<stages::MoveRelative>("approach object", sampling_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::GeneratePose>("generate grasp pose");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setMonitoredStage(initial_state_ptr);

            // Set target pose in world frame, with a vertical orientation
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = params.world_frame;

            // 获取物体的原始位置
            const auto& object_pose = object1_position_;
            // 设置位置
            p.pose.position.x = object_pose.position.x;
            p.pose.position.y = object_pose.position.y;
            p.pose.position.z = object_pose.position.z+0.03;

            // 设置四元数，确保抓手垂直向下（朝向-Z）或垂直向上（朝向+Z）
            // 假设你的抓手在`hand_frame`的Z轴是其抓取方向
            tf2::Quaternion q;
            q.setRPY(0, 0, -M_PI/10); // 这里设置为垂直向下抓取，如果需要垂直向上抓取，可以改为 q.setRPY(0, 0, 0);


            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();

            stage->setPose(p);

            // 将生成的姿态传递给 ComputeIK
            auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);  // Increase from 8
            wrapper->setMinSolutionDistance(0.5); 
            wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });

            // Insert the wrapper into the container
            grasp->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(
                params.object1_name,
                t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
            stage->setGroup(params.hand_group_name);
            std::map<std::string, double> grasp_goal;

            grasp_goal["panda_finger_joint1"] = 0.0225; // 对应4.5cm宽度的一半
            grasp_goal["panda_finger_joint2"] = 0.0225; // 对应4.5cm宽度的一半

            stage->setGoal(grasp_goal);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
            stage->attachObject(params.object1_name, params.hand_frame);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveRelative>("lift object", sampling_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
            stage->setIKFrame(params.hand_frame);
            stage->properties().set("marker_ns", "lift_object");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }


        pick_stage_ptr = grasp.get();
        t.add(std::move(grasp));
    }
     {
      auto stage = std::make_unique<stages::Connect>("move to place", stages::Connect::GroupPlannerVector{{params.arm_group_name,sampling_planner}});
      stage->setTimeout(1.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }


    {
        auto place = std::make_unique<SerialContainer>("place object");
        t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
        place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

        {
            auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.03, .13);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(params.object1_name);
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = params.world_frame;
            p.pose = target1_pose_;
            p.pose.position.z += 0.5 * object_height_ + params.place_surface_offset;
            stage->setPose(p);
            stage->setMonitoredStage(pick_stage_ptr);
            auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(params.hand_open_pose);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions(params.object1_name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name), false);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
            stage->detachObject(params.object1_name, params.hand_frame);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.12, .25);
            stage->setIKFrame(params.hand_frame);
            stage->properties().set("marker_ns", "retreat");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.hand_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        t.add(std::move(place));
    }

    {
        auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setGoal(params.arm_home_pose);
        stage->restrictDirection(stages::MoveTo::FORWARD);
        t.add(std::move(stage));
    }
    

//----------------------------------------------------------------------------------------------------------



    Stage* second_open_hand_ptr = nullptr;
    {
        auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->setGroup(params.hand_group_name);
        stage->setGoal(params.hand_open_pose);
        second_open_hand_ptr = stage.get();
        t.add(std::move(stage));
    }


    {
      auto stage = std::make_unique<stages::Connect>("move to pick", stages::Connect::GroupPlannerVector{{params.arm_group_name,sampling_planner}});
      stage->setTimeout(1.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    Stage* second_pick_ptr = nullptr;
    {
        auto grasp = std::make_unique<SerialContainer>("pick object 2");
        t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });


        {
            auto stage = std::make_unique<stages::MoveRelative>("approach object 2", sampling_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::GeneratePose>("generate grasp pose 2");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setMonitoredStage(second_open_hand_ptr);

            // Set target pose in world frame, with a vertical orientation
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = params.world_frame;

            // 获取物体的原始位置
            const auto& object_pose = object2_position_;
            // 设置位
            p.pose.position.x = object_pose.position.x;
            p.pose.position.y = object_pose.position.y;
            p.pose.position.z = object_pose.position.z;

            // 设置四元数，确保抓手垂直向下（朝向-Z）或垂直向上（朝向+Z）
            // 假设你的抓手在`hand_frame`的Z轴是其抓取方向
            tf2::Quaternion q;
            q.setRPY(0, 0, M_PI/9); // 这里设置为垂直向下抓取，如果需要垂直向上抓取，可以改为 q.setRPY(0, 0, 0);


            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();

            stage->setPose(p);

            // 将生成的姿态传递给 ComputeIK
            auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK 2", std::move(stage));
            wrapper->setMaxIKSolutions(20);  // Increase from 8
            wrapper->setMinSolutionDistance(0.5); 
            wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });

            // Insert the wrapper into the container
            grasp->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(
                params.object2_name,
                t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveTo>("close hand 2", sampling_planner);
            stage->setGroup(params.hand_group_name);
            std::map<std::string, double> grasp_goal;

            grasp_goal["panda_finger_joint1"] = 0.01; // 对应4.5cm宽度的一半
            grasp_goal["panda_finger_joint2"] = 0.01; // 对应4.5cm宽度的一半

            stage->setGoal(grasp_goal);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object 2");
            stage->attachObject(params.object2_name, params.hand_frame);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveRelative>("lift object 2", sampling_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
            stage->setIKFrame(params.hand_frame);
            stage->properties().set("marker_ns", "lift_object");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }


        second_pick_ptr = grasp.get();
        t.add(std::move(grasp));
    }

    {
      auto stage = std::make_unique<stages::Connect>("move to place 2", stages::Connect::GroupPlannerVector{{params.arm_group_name,sampling_planner}});
      stage->setTimeout(1.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }


    {
        auto place = std::make_unique<SerialContainer>("place object 2");
        t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
        place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

        {
            auto stage = std::make_unique<stages::MoveRelative>("lower object 2", cartesian_planner);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.03, .13);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose 2");
            stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(params.object2_name);
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = params.world_frame;

            tf2::Quaternion q;
            q.setRPY(0, 0, M_PI/2);
            target2_pose_.orientation.x = q.x();
            target2_pose_.orientation.y = q.y();
            target2_pose_.orientation.z = q.z();
            target2_pose_.orientation.w = q.w();

            
            p.pose = target2_pose_;
            p.pose.position.z += 0.5 * object_height_ + params.place_surface_offset;
            stage->setPose(p);
            stage->setMonitoredStage(second_pick_ptr);
            auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<stages::MoveTo>("open hand 2", sampling_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(params.hand_open_pose);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions(params.object2_name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name), false);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
            stage->detachObject(params.object2_name, params.hand_frame);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.12, .25);
            stage->setIKFrame(params.hand_frame);
            stage->properties().set("marker_ns", "retreat");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.hand_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        t.add(std::move(place));
    }

    {
        auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setGoal(params.arm_home_pose);
        stage->restrictDirection(stages::MoveTo::FORWARD);
        t.add(std::move(stage));
    }

    try {
        t.init();
    } catch (InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
        return false;
    }

    return true;
}

bool PickPlaceTask::plan(const std::size_t max_solutions) {
    RCLCPP_INFO(LOGGER, "Start searching for task solutions");
    if (!task_) {
        RCLCPP_ERROR(LOGGER, "Task is not initialized. Cannot plan.");
        return false;
    }

    if (task_->plan(max_solutions)) {
        RCLCPP_INFO(LOGGER, "Planning successful!");
        return true;
    } else {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        task_->printState();
        return false;
    }
}

bool PickPlaceTask::execute() {
    RCLCPP_INFO(LOGGER, "Executing solution trajectory");
    if (!task_ || task_->solutions().empty()) {
        RCLCPP_ERROR(LOGGER, "No solution found to execute.");
        return false;
    }

    const moveit::task_constructor::SolutionBase* best_solution = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();

    for (const auto& solution : task_->solutions()) {
        if (solution->cost() < min_cost) {
            min_cost = solution->cost();
            best_solution = solution.get();
        }
    }

    if (best_solution) {
        RCLCPP_INFO(LOGGER, "Executing best solution with cost: %.4f", min_cost);
        moveit_msgs::msg::MoveItErrorCodes execute_result = task_->execute(*best_solution);
        if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
            return false;
        }
        return true;
    }
    return false;
}

void PickPlaceTask::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    if (!received_octomap_) {
        RCLCPP_INFO(LOGGER, "Received OctoMap, applying to planning scene...");
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.octomap.header = msg->header;
        planning_scene_msg.world.octomap.octomap = *msg;
        moveit::planning_interface::PlanningSceneInterface psi;
        if (!psi.applyPlanningScene(planning_scene_msg)) {
             RCLCPP_ERROR(LOGGER, "Failed to apply OctoMap to planning scene");
        } else {
             RCLCPP_INFO(LOGGER, "Successfully applied OctoMap.");
             received_octomap_ = true;
        }
    }
}

void PickPlaceTask::object1PositionCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (!received_object1_position_) {
        object1_position_ = msg->pose;
        object1_scale_ = msg->scale;
        received_object1_position_ = true;
        RCLCPP_INFO(LOGGER, "Received object position");
    }
}

void PickPlaceTask::target1PositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!received_target1_pose_) {
        target1_pose_ = msg->pose;
        received_target1_pose_ = true;
        RCLCPP_INFO(LOGGER, "Received target position");
    }
}
void PickPlaceTask::object2PositionCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (!received_object2_position_) {
        object2_position_ = msg->pose;
        object2_scale_ = msg->scale;
        received_object2_position_ = true;
        RCLCPP_INFO(LOGGER, "Received object position");
    }
}

void PickPlaceTask::target2PositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!received_target2_pose_) {
        target2_pose_ = msg->pose;
        received_target2_pose_ = true;
        RCLCPP_INFO(LOGGER, "Received target position");
    }
}
void PickPlaceTask::object3PositionCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (!received_object3_position_) {
        object3_position_ = msg->pose;
        object3_scale_ = msg->scale;
        received_object3_position_ = true;
        RCLCPP_INFO(LOGGER, "Received object position");
    }
}

void PickPlaceTask::target3PositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!received_target3_pose_) {
        target3_pose_ = msg->pose;
        received_target3_pose_ = true;
        RCLCPP_INFO(LOGGER, "Received target position");
    }
}

void PickPlaceTask::publishAttachedObjectPose() {
    if (!node_ || !scene_monitor_) return;
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(scene_monitor_);
    if (!locked_scene) return;
    
    if (locked_scene->getCurrentState().hasAttachedBody(params_.object_name)) {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(params_.world_frame, params_.hand_frame, tf2::TimePointZero);
            auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
            pose_msg->header.stamp = node_->get_clock()->now();
            pose_msg->header.frame_id = params_.world_frame;
            pose_msg->pose.position.x = t.transform.translation.x;
            pose_msg->pose.position.y = t.transform.translation.y;
            pose_msg->pose.position.z = t.transform.translation.z;
            pose_msg->pose.orientation = t.transform.rotation;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(LOGGER, "Could not get transform: %s", ex.what());
        }
    }
}

} // namespace mtc_dynamic



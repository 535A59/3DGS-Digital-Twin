#include "mtc_dynamic/pick_place_task.h"

#include <algorithm>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Headers required for loading mesh files
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
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

// Mesh collision object: object frame = `pose`, mesh vertices are given in that frame (mesh pose = identity).
moveit_msgs::msg::CollisionObject createMeshCollisionObject(const std::string& object_id,
                                                             const std::string& frame_id,
                                                             const geometry_msgs::msg::Pose& pose,
                                                             const std::string& mesh_path,
                                                             double scale) {
    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource("file://" + mesh_path));
    if (!mesh)
        throw std::runtime_error("Failed to load mesh from resource: " + mesh_path);
    mesh->scale(scale, scale, scale);
    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh.get(), shape_msg))
        throw std::runtime_error("Failed to construct message from mesh: " + mesh_path);

    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = frame_id;
    object.pose = pose;
    object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    object.mesh_poses.resize(1);
    object.mesh_poses[0].orientation.w = 1.0;
    RCLCPP_INFO(LOGGER, "Spawning MESH object '%s' from '%s' (%zu vertices, scale %.3f)", object_id.c_str(),
                mesh_path.c_str(), object.meshes[0].vertices.size(), scale);
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
    if (params_.env_names.size() != params_.env_mesh_paths.size()) {
        RCLCPP_ERROR(LOGGER, "env_names and env_mesh_paths differ in length");
        return false;
    }

    rclcpp::QoS subscriber_qos(10);  // RELIABLE, VOLATILE
    for (const auto& name : params_.object_names) {
        const auto& op = params_.objects.object_names_map.at(name);
        ObjectData& d = objects_[name];
        d.pose_sub = node_->create_subscription<visualization_msgs::msg::Marker>(
            op.pose_topic, subscriber_qos, [this, &d](const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                d.pose = msg->pose;
                d.pose_time = Clock::now();
            });
        d.target_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            op.target_topic, subscriber_qos, [this, &d](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                d.target = msg->pose;
                d.target_time = Clock::now();
            });
        RCLCPP_INFO(LOGGER, "Object '%s': pose %s, target %s", name.c_str(), op.pose_topic.c_str(),
                    op.target_topic.c_str());
    }

    RCLCPP_INFO(LOGGER, "Waiting for initial data...");
    if (!waitForData(params_.object_names, Clock::time_point{}, std::numeric_limits<double>::infinity()))
        return false;
    RCLCPP_INFO(LOGGER, "Received all required initial data.");

    // The ExecuteTaskSolution capability of move_group must be up before the first execute().
    auto ac = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
        node_, "execute_task_solution");
    RCLCPP_INFO(LOGGER, "Waiting for move_group (execute_task_solution)...");
    while (rclcpp::ok() && !ac->wait_for_action_server(std::chrono::seconds(1))) {}

    try {
        moveit::planning_interface::PlanningSceneInterface psi;
        for (std::size_t i = 0; i < params_.env_names.size(); ++i) {
            geometry_msgs::msg::Pose identity;
            identity.orientation.w = 1.0;
            spawnObject(psi, createMeshCollisionObject(params_.env_names[i], params_.world_frame, identity,
                                                       params_.env_mesh_paths[i], params_.mesh_scale_factor));
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& name : params_.object_names)
            spawnObject(psi, createMeshCollisionObject(name, params_.world_frame, objects_[name].pose,
                                                       params_.objects.object_names_map.at(name).mesh_path,
                                                       params_.mesh_scale_factor));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
        return false;
    }
    return rclcpp::ok();
}

bool PickPlaceTask::waitForData(const std::vector<std::string>& names, Clock::time_point since, double timeout_s) {
    const auto start = Clock::now();
    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            bool ready = true;
            for (const auto& n : names)
                ready = ready && objects_[n].pose_time > since && objects_[n].target_time > since;
            if (ready)
                return true;
        }
        if (std::chrono::duration<double>(Clock::now() - start).count() > timeout_s) {
            RCLCPP_ERROR(LOGGER, "Timed out waiting for object pose/target topics");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_WARN(LOGGER, "Shutdown signal received, skipping task setup.");
    return false;
}

bool PickPlaceTask::plan(const std::string& name, const std::size_t max_solutions) {
    RCLCPP_INFO(LOGGER, "Start searching for task solutions for '%s'", name.c_str());
    // Physics moved the objects since init (settling, earlier pick & places): use poses received from now on
    // and refresh every object's collision pose, not only this one.
    if (!waitForData(params_.object_names, Clock::now(), 5.0))
        return false;
    std::map<std::string, ObjectData> latest;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest = objects_;
    }
    try {
        moveit::planning_interface::PlanningSceneInterface psi;
        for (const auto& n : params_.object_names)
            spawnObject(psi, createMeshCollisionObject(n, params_.world_frame, latest[n].pose,
                                                       params_.objects.object_names_map.at(n).mesh_path,
                                                       params_.mesh_scale_factor));
        buildTask(name, latest[name].pose, latest[name].target);
        task_->init();
    } catch (const InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
        return false;
    }

    task_->plan(max_solutions);  // returns TIMED_OUT after task_timeout, possibly with solutions
    if (task_->solutions().empty()) {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        task_->printState();
        return false;
    }
    RCLCPP_INFO(LOGGER, "Planning successful! (%zu solutions)", task_->solutions().size());
    return true;
}

void PickPlaceTask::buildTask(const std::string& name, const geometry_msgs::msg::Pose& body,
                              const geometry_msgs::msg::Pose& target) {
    const auto& params = params_;
    const auto& op = params.objects.object_names_map.at(name);

    // TCP (= ik_frame) poses in world. Grasp: top-down, offset in world axes, yaw about world Z
    // (with grasp_frame_transform's flip, the ik_frame's z points up while link8's z points down).
    // Place: the same body->TCP transform applied to the target body pose.
    Eigen::Isometry3d T_w_body, T_w_target;
    tf2::fromMsg(body, T_w_body);
    tf2::fromMsg(target, T_w_target);
    const Eigen::Isometry3d T_w_grasp =
        Eigen::Translation3d(T_w_body.translation() + Eigen::Vector3d(op.grasp_offset_xyz[0], op.grasp_offset_xyz[1],
                                                                      op.grasp_offset_xyz[2])) *
        Eigen::AngleAxisd(op.grasp_yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Isometry3d T_w_place = T_w_target * T_w_body.inverse() * T_w_grasp;
    const Eigen::Isometry3d ik_frame = vectorToEigen(params.grasp_frame_transform);

    task_ = std::make_shared<Task>();
    Task& t = *task_;
    t.stages()->setName(task_name_ + " " + name);
    t.loadRobotModel(node_);
    t.setTimeout(params.task_timeout);

    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node_);
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    for (solvers::PlannerInterface* p : { static_cast<solvers::PlannerInterface*>(sampling_planner.get()),
                                          static_cast<solvers::PlannerInterface*>(cartesian_planner.get()) }) {
        p->setMaxVelocityScalingFactor(params.motion_scaling);
        p->setMaxAccelerationScalingFactor(params.motion_scaling);
    }

    t.setProperty("group", params.arm_group_name);
    t.setProperty("eef", params.eef_name);
    t.setProperty("hand", params.hand_group_name);
    t.setProperty("hand_grasping_frame", params.hand_frame);
    t.setProperty("ik_frame", params.hand_frame);

    {
        auto cs = std::make_unique<stages::CurrentState>("current state");
        auto filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(cs));
        filter->setPredicate([name](const SolutionBase& s, std::string& comment) {
            if (s.start()->scene()->getCurrentState().hasAttachedBody(name)) {
                comment = "object '" + name + "' already attached";
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
        auto stage = std::make_unique<stages::Connect>("move to pick", stages::Connect::GroupPlannerVector{{params.arm_group_name, sampling_planner}});
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        t.add(std::move(stage));
    }

    Stage* pick_stage_ptr = nullptr;
    {
        auto grasp = std::make_unique<SerialContainer>("pick " + name);
        t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

        {
            auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(op.approach_min_dist, op.approach_max_dist);
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
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = params.world_frame;
            p.pose = tf2::toMsg(T_w_grasp);
            stage->setPose(p);

            auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(0.5);
            wrapper->setIKFrame(ik_frame, params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(
                name, t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(std::map<std::string, double>{ { "panda_finger_joint1", op.finger_close },
                                                          { "panda_finger_joint2", op.finger_close } });
            grasp->insert(std::move(stage));
        }

        // The settled object touches its support (physics contact offset): allow that while lifting off it.
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
            stage->allowCollisions({ name }, params.env_names, true);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
            stage->attachObject(name, params.hand_frame);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(op.lift_min_dist, op.lift_max_dist);
            stage->setIKFrame(params.hand_frame);
            stage->properties().set("marker_ns", "lift_object");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,support)");
            stage->allowCollisions({ name }, params.env_names, false);
            grasp->insert(std::move(stage));
        }

        pick_stage_ptr = grasp.get();
        t.add(std::move(grasp));
    }

    {
        auto stage = std::make_unique<stages::Connect>("move to place", stages::Connect::GroupPlannerVector{{params.arm_group_name, sampling_planner}});
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        t.add(std::move(stage));
    }

    {
        auto place = std::make_unique<SerialContainer>("place " + name);
        t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
        place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

        {
            auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", params.hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(op.lift_min_dist, op.lift_max_dist);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = params.world_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        {
            // TCP target computed above, so the attached body ends exactly at its target pose.
            auto stage = std::make_unique<stages::GeneratePose>("generate place pose");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setMonitoredStage(pick_stage_ptr);
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = params.world_frame;
            p.pose = tf2::toMsg(T_w_place);
            stage->setPose(p);

            auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(0.5);
            wrapper->setIKFrame(ik_frame, params.hand_frame);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
            stage->setGroup(params.hand_group_name);
            stage->setGoal(std::map<std::string, double>{ { "panda_finger_joint1", params.release_open },
                                                          { "panda_finger_joint2", params.release_open } });
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions(name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name), false);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
            stage->detachObject(name, params.hand_frame);
            place->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(op.approach_min_dist, op.approach_max_dist);
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
}

bool PickPlaceTask::execute() {
    RCLCPP_INFO(LOGGER, "Executing solution trajectory");
    if (!task_ || task_->solutions().empty()) {
        RCLCPP_ERROR(LOGGER, "No solution found to execute.");
        return false;
    }
    // solutions() is ordered by cost
    const auto& best_solution = *task_->solutions().front();
    RCLCPP_INFO(LOGGER, "Executing best solution with cost: %.4f", best_solution.cost());

    // Task::execute(), minus the leading sub-trajectories without motion (the current-state generator, no scene
    // change). MoveIt 2.12.4 PlanExecution::executeAndMonitor() starts the execution thread before it allocates
    // plan_components_attached_objects_; an instantly finished first part then makes isRemainingPathValid() index
    // the still unallocated vector and move_group segfaults (seen on the first execution of a move_group process).
    using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
    ExecuteTaskSolution::Goal goal;
    best_solution.toMsg(goal.solution, &task_->introspection());
    auto& subs = goal.solution.sub_trajectory;
    subs.erase(subs.begin(), std::find_if(subs.begin(), subs.end(), [](const auto& s) {
                   return !s.trajectory.joint_trajectory.points.empty();
               }));
    // A reversed Cartesian segment (approach) can start with two points at t = 0, and the JTC rejects the whole goal
    // ("Time between points 0 and 1 is not strictly increasing"): drop points that are not later than the previous.
    for (auto& s : subs) {
        auto& pts = s.trajectory.joint_trajectory.points;
        for (std::size_t i = 1; i < pts.size();) {
            if (rclcpp::Duration(pts[i].time_from_start) > rclcpp::Duration(pts[i - 1].time_from_start))
                ++i;
            else
                pts.erase(pts.begin() + i);
        }
    }
    auto ac = rclcpp_action::create_client<ExecuteTaskSolution>(node_, "execute_task_solution");
    if (!ac->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(LOGGER, "execute_task_solution action server not available");
        return false;
    }
    const auto goal_handle = ac->async_send_goal(goal).get();  // node_ is spun by the caller's thread
    if (!goal_handle) {
        RCLCPP_ERROR(LOGGER, "Task execution goal rejected");
        return false;
    }
    const auto result = ac->async_get_result(goal_handle).get();
    if (!result.result || result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: "
                                        << (result.result ? result.result->error_code.val : 0));
        return false;
    }
    return true;
}

} // namespace mtc_dynamic

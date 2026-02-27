// #include <rclcpp/rclcpp.hpp>
// #include <moveit/task_constructor/task.h>
// #include <moveit/task_constructor/container.h> 
// #include <moveit/task_constructor/solvers/cartesian_path.h>
// #include <moveit/task_constructor/solvers/pipeline_planner.h>
// #include <moveit/task_constructor/stages/current_state.h>
// #include <moveit/task_constructor/stages/move_to.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_eigen/tf2_eigen.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <moveit/task_constructor/stages/move_along.h>

// #include <mtc_dynamic/collectEnvDemo_parameters.hpp>


// #include <moveit/robot_trajectory/robot_trajectory.h>

// namespace mtc_dynamic
// {
// using namespace moveit::task_constructor;

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("collect_env_task");

// class CollectEnvTask
// {
// public:
//     CollectEnvTask(const std::string& task_name) : task_name_(task_name) {}
//     bool init(const rclcpp::Node::SharedPtr& node, const collectEnvDemo::Params& params);
//     bool plan();
//     bool execute();

// private:
//     rclcpp::Node::SharedPtr node_;
//     collectEnvDemo::Params params_;
//     std::shared_ptr<moveit::task_constructor::Task> task_;
//     std::string task_name_;
// };

// bool CollectEnvTask::init(const rclcpp::Node::SharedPtr& node, const collectEnvDemo::Params& params)
// {
//     RCLCPP_INFO(LOGGER, "Initializing environment collection task pipeline...");
//     node_ = node;
//     params_ = params;

//     task_ = std::make_shared<Task>();
//     Task& t = *task_; 
//     t.stages()->setName(task_name_);
//     t.loadRobotModel(node);

//     t.setProperty("group", params_.arm_group_name);
//     t.setProperty("eef", params_.eef_name);
//     t.setProperty("ik_frame", params_.hand_frame);

//     auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
//     auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

//     // 将最大速度和加速度都缩放到其允许值的 20% (您可以根据需要调整这个值)
//     double scaling_factor = 0.2; 
//     sampling_planner->setProperty("max_velocity_scaling_factor", scaling_factor);
//     sampling_planner->setProperty("max_acceleration_scaling_factor", scaling_factor);
//     cartesian_planner->setProperty("max_velocity_scaling_factor", scaling_factor);
//     cartesian_planner->setProperty("max_acceleration_scaling_factor", scaling_factor);

//     // 阶段 1: 从当前状态开始
//     t.add(std::make_unique<stages::CurrentState>("current state"));

//     // 阶段 2: 移动到一个安全的起始/预备位置
//     {
//         auto stage = std::make_unique<stages::MoveTo>("move to ready pose", sampling_planner);
//         stage->setGroup(params_.arm_group_name);
//         stage->setGoal(params_.arm_home_pose);
//         t.add(std::move(stage));
//     }

//     // 阶段 3: 执行环绕扫描动作
//     // {
//     //     RCLCPP_INFO(LOGGER, "Adding 'circle the table' container");
        
//     //     auto circle_container = std::make_unique<SerialContainer>("Circling Motion");

//     //     const double circle_center_x = 0.5;
//     //     const double circle_center_y = 0.0;
//     //     const double circle_radius   = 0.25;
//     //     const double circle_height   = 0.4;
//     //     const int    num_points      = 16;
        
//     //     tf2::Quaternion q;
//     //     q.setRPY(M_PI, 0, 0); 
//     //     geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

//     //     // for (int i = 0; i <= num_points; ++i) {
//     //     //     // // ▼▼▼ 关键修正：第一步使用灵活的采样规划器，后续步骤使用笛卡尔规划器 ▼▼▼
//     //     //     // std::unique_ptr<stages::MoveTo> stage;
//     //     //     // if (i == 0) {
//     //     //     //     // 第一步（大范围移动）使用采样规划器
//     //     //     //     stage = std::make_unique<stages::MoveTo>("move to waypoint " + std::to_string(i), sampling_planner);
//     //     //     // } else {
//     //     //     //     // 后续小步移动使用笛卡尔（直线）规划器
//     //     //     //     stage = std::make_unique<stages::MoveTo>("move to waypoint " + std::to_string(i), cartesian_planner);
//     //     //     // }
            
//     //     //     // stage->setGroup(params_.arm_group_name);
            
//     //     //     // geometry_msgs::msg::PoseStamped p;
//     //     //     // p.header.frame_id = params_.world_frame;
            
//     //     //     // double angle = 2.0 * M_PI * i / num_points;
//     //     //     // p.pose.position.x = circle_center_x + circle_radius * cos(angle);
//     //     //     // p.pose.position.y = circle_center_y + circle_radius * sin(angle);
//     //     //     // p.pose.position.z = circle_height;
            
//     //     //     // // 动态计算朝向
//     //     //     // double delta_x = circle_center_x - p.pose.position.x;
//     //     //     // double delta_y = circle_center_y - p.pose.position.y;
//     //     //     // double yaw = atan2(delta_y, delta_x);
//     //     //     // tf2::Quaternion q_pose;
//     //     //     // q_pose.setRPY(M_PI, 0, yaw);
//     //     //     // p.pose.orientation = tf2::toMsg(q_pose);

//     //     //     // stage->setGoal(p);
//     //     //     // circle_container->insert(std::move(stage));
//     //     // }
//     //     for (int i = 0; i <= num_points; ++i) {
//     //         // ▼▼▼ 关键修正：在这里重新加入对每个路径点姿态 p 的计算 ▼▼▼
//     //         geometry_msgs::msg::PoseStamped p;
//     //         p.header.frame_id = params_.world_frame;
            
//     //         double angle = 2.0 * M_PI * i / num_points;
//     //         p.pose.position.x = circle_center_x + circle_radius * cos(angle);
//     //         p.pose.position.y = circle_center_y + circle_radius * sin(angle);
//     //         p.pose.position.z = circle_height;
            
//     //         // 动态计算朝向
//     //         double delta_x = circle_center_x - p.pose.position.x;
//     //         double delta_y = circle_center_y - p.pose.position.y;
//     //         double yaw = atan2(delta_y, delta_x);
//     //         tf2::Quaternion q_pose;
//     //         q_pose.setRPY(M_PI, 0, yaw);
//     //         p.pose.orientation = tf2::toMsg(q_pose);
//     //         // ▲▲▲ 计算 p 的代码结束 ▲▲▲


//     //         // 创建一个 Alternatives 容器来存放备选规划方案
//     //         auto alternatives = std::make_unique<Alternatives>("try cartesian then ompl for waypoint " + std::to_string(i));

//     //         // **备选方案1 (首选): 笛卡尔路径规划**
//     //         auto cartesian_stage = std::make_unique<stages::MoveTo>("move cartesian", cartesian_planner);
//     //         cartesian_stage->setGroup(params_.arm_group_name);
//     //         cartesian_stage->setGoal(p); // 现在可以找到 p 了
//     //         alternatives->add(std::move(cartesian_stage));

//     //         // **备选方案2 (备用): OMPL 规划**
//     //         auto sampling_stage = std::make_unique<stages::MoveTo>("move ompl", sampling_planner);
//     //         sampling_stage->setGroup(params_.arm_group_name);
//     //         sampling_stage->setGoal(p); // 这里也可以找到 p
//     //         alternatives->add(std::move(sampling_stage));

//     //         // 将 Alternatives 容器插入到主任务流程中
//     //         circle_container->insert(std::move(alternatives));
//     //     }
//     //     t.add(std::move(circle_container));
//     // }

//       {
//         RCLCPP_INFO(LOGGER, "Adding '360-degree elliptical scan as a single trajectory' stage");
        
//         // 创建一个 SerialContainer 来容纳扫描动作
//         auto elliptical_scan_container = std::make_unique<SerialContainer>("Elliptical Scanning Motion");

//         // 我们将优先使用笛卡尔规划器来规划一条完整的轨迹
//         auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
//         cartesian_planner->setProperty("max_velocity_scaling_factor", 0.2);
//         cartesian_planner->setProperty("max_acceleration_scaling_factor", 0.2);


//         // 创建一个 MoveAlong 阶段，该阶段专门用于规划通过一系列航点的连续路径
//         auto stage = std::make_unique<stages::MoveAlong>("move along ellipse", cartesian_planner);
//         stage->setGroup(params_.arm_group_name);
        
//         // =================================================================
//         // ===            360度椭圆扫描路径参数 (可在此微调)             ===
//         // =================================================================
//         const double ellipse_center_x = 0.5;  // 椭圆中心的X坐标
//         const double ellipse_center_y = 0.0;  // 椭圆中心的Y坐标
//         const double radius_x         = 0.22; // X方向的半径 (长轴)
//         const double radius_y         = 0.45; // Y方向的半径 (短轴)
//         const double scan_height      = 0.4;  // 扫描时的高度
//         const int    num_points       = 32;   // 路径点的数量 (数量越多，路径越平滑)
//         // =================================================================

//         // 首先，生成所有路径点并将它们存储在一个 vector 中
//         std::vector<geometry_msgs::msg::PoseStamped> waypoints;
//         for (int i = 0; i <= num_points; ++i) {
//             geometry_msgs::msg::PoseStamped p;
//             p.header.frame_id = params_.world_frame;
            
//             // 计算完整的360度扫描角度 (0 to 2*PI)
//             double current_angle = 2.0 * M_PI * i / num_points;

//             // 根据椭圆参数方程计算路径点的 (x, y, z) 坐标
//             p.pose.position.x = ellipse_center_x + radius_x * cos(current_angle);
//             p.pose.position.y = ellipse_center_y + radius_y * sin(current_angle);
//             p.pose.position.z = scan_height;
            
//             // 动态计算姿态，使其始终朝向椭圆中心点
//             double delta_x = ellipse_center_x - p.pose.position.x;
//             double delta_y = ellipse_center_y - p.pose.position.y;
//             double yaw = atan2(delta_y, delta_x); // 计算偏航角
            
//             tf2::Quaternion q_pose;
//             // 设置翻滚角为PI(180度)，使工具末端大致朝下；俯仰角为0；偏航角为动态计算的值
//             q_pose.setRPY(M_PI, 0, yaw);
//             p.pose.orientation = tf2::toMsg(q_pose);

//             // 将计算好的航点添加到向量中
//             waypoints.push_back(p);
//         }

//         // 将整个 waypoints 向量设置为 MoveAlong 阶段的目标
//         stage->setWaypoints(waypoints);
        
//         // 将配置好的 MoveAlong 阶段添加到任务容器中
//         elliptical_scan_container->insert(std::move(stage));
        
//         // 将整个容器添加到主任务流程中
//         t.add(std::move(elliptical_scan_container));
//     }
    
//     // 阶段 4: 返回初始姿态
//     {
//         auto stage = std::make_unique<stages::MoveTo>("return home", sampling_planner);
//         stage->setGroup(params_.arm_group_name);
//         stage->setGoal(params_.arm_home_pose);
//         t.add(std::move(stage));
//     }

//     try {
//         t.init();
//     } catch (const InitStageException& e) {
//         RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
//         return false;
//     }

//     RCLCPP_INFO(LOGGER, "Task pipeline initialized successfully.");
//     return true;
// }

// // CollectEnvTask::plan
// bool CollectEnvTask::plan() {
//     RCLCPP_INFO(LOGGER, "Start searching for task solutions");
//     // 尝试寻找最多5个解决方案
//     if (!task_ || !task_->plan(5)) {
//         RCLCPP_ERROR(LOGGER, "Planning failed!");
//         task_->printState();
//         return false;
//     }
//     RCLCPP_INFO(LOGGER, "Planning successful! Found %zu solutions.", task_->solutions().size());
//     return true;
// }

// // CollectEnvTask::execute
// bool CollectEnvTask::execute() {
//     RCLCPP_INFO(LOGGER, "Executing solution trajectory");
//     if (!task_ || task_->solutions().empty()) {
//         RCLCPP_ERROR(LOGGER, "No solution found to execute.");
//         return false;
//     }

//     // 遍历所有解，找到成本最低的那个
//     const moveit::task_constructor::SolutionBase* best_solution = nullptr;
//     double min_cost = std::numeric_limits<double>::infinity();


//     for (const auto& sol : task_->solutions()) {
//         if (sol->cost() < min_cost) {
//             min_cost = sol->cost();
//             best_solution = sol.get();
//         }
//     }

//     auto trajectory_publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//         "/planned_trajectory_for_ros1", 10);


//     if (best_solution) {
//         // 1. 使用 dynamic_cast 尝试将 SolutionBase 指针向下转型为 SubTrajectory 指针。
//         //    如果 best_solution 指向的确实是一个 SubTrajectory 对象，转型会成功，
//         //    sub_trajectory 将是一个有效指针。否则，它将是 nullptr。
//         auto sub_trajectory = dynamic_cast<const moveit::task_constructor::SubTrajectory*>(best_solution);

//         // 2. 检查转型是否成功
//         if (sub_trajectory) {
//             // 转型成功，现在可以安全地访问 SubTrajectory 的成员了
            
//             // 3. 调用 trajectory() 方法获取 robot_trajectory::RobotTrajectoryConstPtr
//             //    这是MoveIt内部表示轨迹的核心对象。
//             auto robot_trajectory_ptr = sub_trajectory->trajectory();

//             if (robot_trajectory_ptr) {
//                 // 4. 将 RobotTrajectory 对象转换为标准的ROS消息格式
//                 moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
//                 robot_trajectory_ptr->getRobotTrajectoryMsg(robot_trajectory_msg);
                
//                 // 5. 从消息中提取出我们最终需要的 joint_trajectory
//                 trajectory_msgs::msg::JointTrajectory joint_trajectory = robot_trajectory_msg.joint_trajectory;

//                 RCLCPP_INFO(LOGGER, "成功提取轨迹，包含 %zu 个路点。", joint_trajectory.points.size());
                
//                 // 在这里，您就可以发布或使用 joint_trajectory 了
//             }
//         } else {
//             // 转型失败，意味着这个解决方案不是一个运动轨迹。
//             // 它可能是由 attach/detach object 等非运动Stage生成的。
//             RCLCPP_WARN(LOGGER, "该解决方案不是 SubTrajectory 类型，无法提取运动规划。");
//         }
//     }

//     if (!best_solution) {
//         RCLCPP_ERROR(LOGGER, "Could not identify the best solution.");
//         return false;
//     }
    
//     RCLCPP_INFO(LOGGER, "Executing solution with the lowest cost: %f", min_cost);
//     moveit_msgs::msg::MoveItErrorCodes execute_result = task_->execute(*best_solution);
    
//     if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
//         RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << execute_result.val);
//         return false;
//     }
//     RCLCPP_INFO(LOGGER, "Execution successful!");
//     return true;
// }

 //} // namespace mtc_dynamic

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/task_constructor/task.h>
// #include <moveit/task_constructor/container.h> 
// #include <moveit/task_constructor/solvers/cartesian_path.h>
// #include <moveit/task_constructor/solvers/pipeline_planner.h>
// #include <moveit/task_constructor/stages/current_state.h>
// #include <moveit/task_constructor/stages/move_to.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_eigen/tf2_eigen.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <mtc_dynamic/collectEnvDemo_parameters.hpp>


// #include <moveit/robot_trajectory/robot_trajectory.h>

// namespace mtc_dynamic
// {
// using namespace moveit::task_constructor;

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("collect_env_task");

// class CollectEnvTask
// {
// public:
//     CollectEnvTask(const std::string& task_name) : task_name_(task_name) {}
//     bool init(const rclcpp::Node::SharedPtr& node, const collectEnvDemo::Params& params);
//     bool plan();
//     bool execute();

// private:
//     rclcpp::Node::SharedPtr node_;
//     collectEnvDemo::Params params_;
//     std::shared_ptr<moveit::task_constructor::Task> task_;
//     std::string task_name_;
// };

// bool CollectEnvTask::init(const rclcpp::Node::SharedPtr& node, const collectEnvDemo::Params& params)
// {
//     RCLCPP_INFO(LOGGER, "Initializing environment collection task pipeline...");
//     node_ = node;
//     params_ = params;

//     task_ = std::make_shared<Task>();
//     Task& t = *task_; 
//     t.stages()->setName(task_name_);
//     t.loadRobotModel(node);

//     t.setProperty("group", params_.arm_group_name);
//     t.setProperty("eef", params_.eef_name);
//     t.setProperty("ik_frame", params_.hand_frame);

//     auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
//     auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

//     double scaling_factor = 0.1; 
//     sampling_planner->setProperty("max_velocity_scaling_factor", scaling_factor);
//     sampling_planner->setProperty("max_acceleration_scaling_factor", scaling_factor);
//     cartesian_planner->setProperty("max_velocity_scaling_factor", scaling_factor);
//     cartesian_planner->setProperty("max_acceleration_scaling_factor", scaling_factor);

//     // 阶段 1: 从当前状态开始
//     t.add(std::make_unique<stages::CurrentState>("current state"));

//     // 阶段 2: 移动到一个安全的起始/预备位置
//     {
//         auto stage = std::make_unique<stages::MoveTo>("move to ready pose", sampling_planner);
//         stage->setGroup(params_.arm_group_name);
//         stage->setGoal(params_.arm_home_pose);
//         t.add(std::move(stage));
//     }

//     // 阶段 3: 执行为3D重建优化的360度椭圆扫描
//     {
//         RCLCPP_INFO(LOGGER, "Adding '360-degree elliptical scan with forward safety distance' container");
        
//         auto elliptical_scan_container = std::make_unique<SerialContainer>("Elliptical Scanning Motion");

//         // =================================================================
//         // ===            360度椭圆扫描路径参数 (可在此微调)             ===
//         // =================================================================
//         // 坐标系以机器人基座 panda_link0 为原点
//         // X轴向前 (纵向), Y轴向左 (横向)

//         // --- 用户定义的约束条件 ---
//         const double MIN_FORWARD_DISTANCE = 0.20; // 最近的前向距离 (米), 设为16厘米以留出安全裕度
//         const double MAX_FORWARD_DISTANCE = 0.67; // 最远的前向距离 (米)
//         const double LATERAL_COVERAGE = 0.90;     // 横向覆盖总宽度 (米)

//         // --- 根据约束自动计算椭圆参数 ---
//         // 横向半径 (Y方向)
//         const double radius_y = LATERAL_COVERAGE / 2.0; // 0.90 / 2 = 0.45米
//         // 纵向半径 (X方向)
//         const double radius_x = (MAX_FORWARD_DISTANCE - MIN_FORWARD_DISTANCE) / 2.0; // (0.60 - 0.16) / 2 = 0.22米
//         // 椭圆中心点 (X方向)
//         const double ellipse_center_x = MIN_FORWARD_DISTANCE + radius_x; // 0.16 + 0.22 = 0.38米
//         const double ellipse_center_y = 0.0;
        
//         // 末端执行器（摄像头）在扫描时的高度
//         const double scan_height = 0.4;  // 米 (基座上方40厘米)

//         // 路径点的数量 (数量越多，路径越平滑)
//         const int num_points = 15;
//         // =================================================================

//         for (int i = 0; i <= num_points; ++i) {
//             geometry_msgs::msg::PoseStamped p;
//             p.header.frame_id = params_.world_frame;
            
//             // 计算完整的360度扫描角度 (0 to 2*PI)
//             double current_angle = 2.0 * M_PI * i / num_points;

//             // 根据椭圆参数方程计算路径点的 (x, y, z) 坐标
//             p.pose.position.x = ellipse_center_x + radius_x * cos(current_angle);
//             p.pose.position.y = ellipse_center_y + radius_y * sin(current_angle);
//             p.pose.position.z = scan_height;
            
//             // 动态计算姿态，使其始终朝向椭圆中心点
//             double delta_x = ellipse_center_x - p.pose.position.x;
//             double delta_y = ellipse_center_y - p.pose.position.y;
//             double yaw = atan2(delta_y, delta_x); // 计算偏航角
            
//             tf2::Quaternion q_pose;
//             // 设置翻滚角为PI，使手爪朝下；俯仰角为0；偏航角为计算出的值
//             q_pose.setRPY(M_PI, 0, yaw);
//             p.pose.orientation = tf2::toMsg(q_pose);

//             // 优先尝试直线路径，如果失败则回退到采样规划
//             auto alternatives = std::make_unique<Alternatives>("waypoint " + std::to_string(i));

//             // 方案一 (首选): 笛卡尔路径 (直线)
//             auto cartesian_stage = std::make_unique<stages::MoveTo>("move cartesian", cartesian_planner);
//             cartesian_stage->setGroup(params_.arm_group_name);
//             cartesian_stage->setGoal(p);
//             alternatives->add(std::move(cartesian_stage));

//             // 方案二 (备用): OMPL 采样规划
//             auto sampling_stage = std::make_unique<stages::MoveTo>("move ompl", sampling_planner);
//             sampling_stage->setGroup(params_.arm_group_name);
//             sampling_stage->setGoal(p);
//             alternatives->add(std::move(sampling_stage));

//             elliptical_scan_container->insert(std::move(alternatives));
//         }
//         t.add(std::move(elliptical_scan_container));
//     }
    
//     // 阶段 4: 返回初始姿态
//     {
//         auto stage = std::make_unique<stages::MoveTo>("return home", sampling_planner);
//         stage->setGroup(params_.arm_group_name);
//         stage->setGoal(params_.arm_home_pose);
//         t.add(std::move(stage));
//     }

//     try {
//         t.init();
//     } catch (const InitStageException& e) {
//         RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
//         return false;
//     }

//     RCLCPP_INFO(LOGGER, "Task pipeline initialized successfully.");
//     return true;
// }

// // CollectEnvTask::plan
// bool CollectEnvTask::plan() {
//     RCLCPP_INFO(LOGGER, "Start searching for task solutions");
//     if (!task_ || !task_->plan(5)) {
//         RCLCPP_ERROR(LOGGER, "Planning failed!");
//         task_->printState();
//         return false;
//     }
//     RCLCPP_INFO(LOGGER, "Planning successful! Found %zu solutions.", task_->solutions().size());
//     return true;
// }

// // CollectEnvTask::execute
// bool CollectEnvTask::execute() {
//     RCLCPP_INFO(LOGGER, "Executing solution trajectory");
//     if (!task_ || task_->solutions().empty()) {
//         RCLCPP_ERROR(LOGGER, "No solution found to execute.");
//         return false;
//     }

//     const moveit::task_constructor::SolutionBase* best_solution = nullptr;
//     double min_cost = std::numeric_limits<double>::infinity();

//     for (const auto& sol : task_->solutions()) {
//         if (sol->cost() < min_cost) {
//             min_cost = sol->cost();
//             best_solution = sol.get();
//         }
//     }

//     if (!best_solution) {
//         RCLCPP_ERROR(LOGGER, "Could not identify the best solution.");
//         return false;
//     }
    
//     RCLCPP_INFO(LOGGER, "Executing solution with the lowest cost: %f", min_cost);
//     moveit_msgs::msg::MoveItErrorCodes execute_result = task_->execute(*best_solution);
    
//     if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
//         RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << execute_result.val);
//         return false;
//     }
//     RCLCPP_INFO(LOGGER, "Execution successful!");
//     return true;
// }

// } // namespace mtc_dynamic

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h> 
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mtc_dynamic/collectEnvDemo_parameters.hpp>

namespace mtc_dynamic
{
using namespace moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("collect_env_task");

class CollectEnvTask
{
public:
    CollectEnvTask(const std::string& task_name) : task_name_(task_name) {}
    bool init(const rclcpp::Node::SharedPtr& node, const collectEnvDemo::Params& params);
    bool plan();
    bool execute();

private:
    // 执行连续扫描的辅助函数
    bool runContinuousEllipticalScan();

    rclcpp::Node::SharedPtr node_;
    collectEnvDemo::Params params_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    std::string task_name_;
};

// ==========================================================================================
// 辅助函数：实现连续的椭圆扫描
// ==========================================================================================
bool CollectEnvTask::runContinuousEllipticalScan() {
    RCLCPP_INFO(LOGGER, "Starting continuous elliptical scan...");
    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface mgi(node_, params_.arm_group_name);
    mgi.setPoseReferenceFrame(params_.world_frame);
    mgi.setEndEffectorLink(params_.hand_frame);
    
    const double max_vel_scale = 0.2;
    const double max_acc_scale = 0.2;
    mgi.setMaxVelocityScalingFactor(max_vel_scale);
    mgi.setMaxAccelerationScalingFactor(max_acc_scale);

    const double ellipse_center_x = 0.45;
    const double ellipse_center_y = 0.0;
    const double radius_x         = 0.27;
    const double radius_y         = 0.55;
    const double scan_height      = 0.55;
    const int    num_points       = 64; 

    // ▼▼▼▼▼ 核心修改：定义扇形的起始和结束角度 ▼▼▼▼▼
    // 以机器人前方为0度，逆时针为正
    const double start_angle_deg = -120.0;  // 从右前方45度开始
    const double end_angle_deg   = 120.0;   // 到左前方45度结束
    // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    // 将角度转换为弧度
    const double start_angle_rad = start_angle_deg * M_PI / 180.0;
    const double end_angle_rad   = end_angle_deg * M_PI / 180.0;
    const double angle_range     = end_angle_rad - start_angle_rad;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(num_points + 1);

    for (int i = 0; i <= num_points; ++i) {
        // ▼▼▼▼▼ 核心修改：重新计算当前角度 ▼▼▼▼▼
        // 不再是完整的2*PI，而是在[start_angle, end_angle]之间插值
        double current_angle = start_angle_rad + (static_cast<double>(i) / num_points) * angle_range;
        // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

        geometry_msgs::msg::Pose p;
        p.position.x = ellipse_center_x + radius_x * cos(current_angle);
        p.position.y = ellipse_center_y + radius_y * sin(current_angle);
        p.position.z = scan_height;
        
        // ...姿态计算部分保持不变...
        double delta_x = ellipse_center_x - p.position.x;
        double delta_y = ellipse_center_y - p.position.y;
        double yaw = atan2(delta_y, delta_x);
        
        // (这里的偏移量计算逻辑也保持不变)
        const double initial_yaw = M_PI;
        const double desired_start_angle = -0.7850222987219692;
        const double angle_offset = initial_yaw - desired_start_angle;
        double final_gripper_angle = yaw - angle_offset;

        double pitch_tilt = -M_PI/2;
        
        tf2::Quaternion q_pose;
        q_pose.setRPY(M_PI, pitch_tilt, final_gripper_angle);
        
        p.orientation = tf2::toMsg(q_pose);
        waypoints.push_back(p);
    }
    // 使用 MoveGroupInterface 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    const double eef_step = 0.005; // 5mm的插值步长
    const double jump_threshold = 0.0; // 禁用跳跃检测

    double fraction = mgi.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);

    if (fraction < 0.99) {
        RCLCPP_ERROR(LOGGER, "Continuous scan path planning failed. Only %.1f%% of the path was computed.", fraction * 100.0);
        return false;
    }

    RCLCPP_INFO(LOGGER, "Continuous scan path computed successfully (%.1f%%). Executing...", fraction * 100.0);
    
    // 执行轨迹
    moveit::core::MoveItErrorCode result = mgi.execute(trajectory_msg);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to execute continuous scan. Error code: " << result.val);
        return false;
    }

    RCLCPP_INFO(LOGGER, "Continuous elliptical scan completed successfully.");
    return true;
}


// ==========================================================================================
// MTC 初始化函数 (已简化)
// ==========================================================================================
bool CollectEnvTask::init(const rclcpp::Node::SharedPtr& node, const collectEnvDemo::Params& params)
{
    RCLCPP_INFO(LOGGER, "Initializing task pipeline for pre- and post-scan movements...");
    node_ = node;
    params_ = params;

    task_ = std::make_shared<Task>();
    Task& t = *task_; 
    t.stages()->setName(task_name_);
    t.loadRobotModel(node);

    t.setProperty("group", params_.arm_group_name);
    t.setProperty("eef", params_.eef_name);
    t.setProperty("ik_frame", params_.hand_frame);

    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
    sampling_planner->setProperty("max_velocity_scaling_factor", 0.2);
    sampling_planner->setProperty("max_acceleration_scaling_factor", 0.2);

    // 阶段 1: 从当前状态开始
    t.add(std::make_unique<stages::CurrentState>("current state"));

    // 阶段 2: 移动到一个安全的起始/预备位置
    {
        auto stage = std::make_unique<stages::MoveTo>("move to ready pose", sampling_planner);
        stage->setGroup(params_.arm_group_name);
        stage->setGoal(params_.arm_home_pose);
        t.add(std::move(stage));
    }
    
    try {
        t.init();
    } catch (const InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
        return false;
    }

    RCLCPP_INFO(LOGGER, "Task pipeline initialized successfully.");
    return true;
}


// ==========================================================================================
// MTC 规划函数 (不变)
// ==========================================================================================
bool CollectEnvTask::plan() {
    RCLCPP_INFO(LOGGER, "Start searching for task solutions for pre-scan movement.");
    if (!task_ || !task_->plan(10)) {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        task_->printState();
        return false;
    }
    RCLCPP_INFO(LOGGER, "Planning successful! Found %zu solutions.", task_->solutions().size());
    return true;
}

// ==========================================================================================
// 执行函数 (核心逻辑)
// ==========================================================================================
bool CollectEnvTask::execute() {
    if (!task_ || task_->solutions().empty()) {
        RCLCPP_ERROR(LOGGER, "No solution found for pre-scan movement.");
        return false;
    }
    
    // --- 步骤 1: 执行MTC任务，移动到预备位置 ---
    RCLCPP_INFO(LOGGER, "Executing pre-scan movement (move to ready pose)...");
    moveit_msgs::msg::MoveItErrorCodes execute_result = task_->execute(*task_->solutions().front());
    if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM( LOGGER, "Pre-scan movement failed with code: " << execute_result.val);
        return false;
    }
    RCLCPP_INFO(LOGGER, "Pre-scan movement successful.");

    // --- 步骤 2: 执行连续扫描 ---
    if (!runContinuousEllipticalScan()) {
        RCLCPP_ERROR(LOGGER, "Continuous scan execution failed.");
        // 即使扫描失败，也尝试返回home
    }

    // --- 步骤 3: 移动返回初始位置 ---
    RCLCPP_INFO(LOGGER, "Returning to home pose...");
    moveit::planning_interface::MoveGroupInterface mgi(node_, params_.arm_group_name);
    mgi.setNamedTarget(params_.arm_home_pose);
    if (mgi.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(LOGGER, "Failed to return to home pose after scanning.");
        return false; // 可以根据需求决定是否作为硬性失败
    }

    RCLCPP_INFO(LOGGER, "Task finished successfully!");
    return true;
}

} // namespace mtc_dynamic
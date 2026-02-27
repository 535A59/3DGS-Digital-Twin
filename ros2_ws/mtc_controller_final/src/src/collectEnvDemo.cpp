#include <rclcpp/rclcpp.hpp>
#include "collectEnvTask.cpp"

// 包含为 collectEnvDemo 生成的专属参数头文件
#include <mtc_dynamic/collectEnvDemo_parameters.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("collect_env_demo");

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("collect_env_node", node_options);
    std::thread spinning_thread([node] { rclcpp::spin(node); });

    // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
    // ▼▼▼ 核心修正：在这里添加一个延时，等待 move_group 节点完全启动 ▼▼▼
    // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
    RCLCPP_INFO(LOGGER, "Waiting for MoveIt to be ready...");
    // 通常5秒的延时足以让所有服务启动
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(LOGGER, "MoveIt should be ready. Starting task...");
    // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    // 从参数服务器加载参数
    const auto param_listener = std::make_shared<collectEnvDemo::ParamListener>(node);
    const auto params = param_listener->get_params();

    // 创建并运行环境扫描任务
    mtc_dynamic::CollectEnvTask collect_task("environment_scan");
    if (!collect_task.init(node, params)) {
        RCLCPP_ERROR(LOGGER, "Task initialization failed. Shutting down.");
        rclcpp::shutdown();
        spinning_thread.join();
        return 1;
    }

    if (collect_task.plan()) {
        RCLCPP_INFO(LOGGER, "Planning succeeded. Proceeding to execute.");
        collect_task.execute();
    } else {
        RCLCPP_ERROR(LOGGER, "Planning failed. Cannot execute task.");
    }

    RCLCPP_INFO(LOGGER, "Task finished. Shutting down.");
    spinning_thread.join();
    return 0;
}
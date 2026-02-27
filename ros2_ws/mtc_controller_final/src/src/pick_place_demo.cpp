#include <rclcpp/rclcpp.hpp>
#include <mtc_dynamic/pick_place_task.h> // ✅ 使用新的头文件路径和命名空间

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dynamic_demo");

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("mtc_dynamic_demo", node_options);
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	// ✅ 使用正确的参数命名空间
	const auto param_listener = std::make_shared<pick_place_task_demo::ParamListener>(node);
	const auto params = param_listener->get_params();
	

	// ✅ 使用新的命名空间
	mtc_dynamic::PickPlaceTask pick_place_task("pick_place_task");
	if (!pick_place_task.init(node, params)) {
		RCLCPP_INFO(LOGGER, "Initialization failed");
		return 1;
	}

	if (pick_place_task.plan(params.max_solutions)) {
		RCLCPP_INFO(LOGGER, "Planning succeded");
		pick_place_task.execute();
	} else {
		RCLCPP_INFO(LOGGER, "Planning failed");
	}

	// Keep introspection alive
	spinning_thread.join();
	return 0;
}
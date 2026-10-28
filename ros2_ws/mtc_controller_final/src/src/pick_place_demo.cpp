#include <rclcpp/rclcpp.hpp>
#include <mtc_dynamic/pick_place_task.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_dynamic_demo");

// Pick & place every object of `object_names` in order (one MTC task each, re-planned from the
// current robot/object state), then exit: 0 = all done, 1 = a task failed ("Execution complete" / "Task failed").
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("mtc_dynamic_demo", node_options);
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	const auto param_listener = std::make_shared<pick_place_task_demo::ParamListener>(node);
	const auto params = param_listener->get_params();

	mtc_dynamic::PickPlaceTask pick_place_task("pick_place_task");
	int rc = 0;
	if (!pick_place_task.init(node, params)) {
		RCLCPP_ERROR(LOGGER, "Initialization failed");
		rc = 1;
	}
	for (const auto& name : params.object_names) {
		if (rc)
			break;
		const auto t0 = node->now();
		if (!pick_place_task.plan(name, params.max_solutions)) {
			RCLCPP_ERROR(LOGGER, "Task failed: planning '%s'", name.c_str());
			rc = 1;
		} else if (!pick_place_task.execute()) {
			RCLCPP_ERROR(LOGGER, "Task failed: executing '%s'", name.c_str());
			rc = 1;
		} else {
			RCLCPP_INFO(LOGGER, "Object '%s' done in %.1f s (sim)", name.c_str(), (node->now() - t0).seconds());
		}
	}
	if (!rc)
		RCLCPP_INFO(LOGGER, "Execution complete: %zu objects", params.object_names.size());

	rclcpp::shutdown();
	spinning_thread.join();
	return rc;
}

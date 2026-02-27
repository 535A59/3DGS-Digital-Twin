import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess

def generate_launch_description():
    # moveit_config Builder
    # ###################################################################
    # This is the core of the MoveIt launch file.
    # It loads the robot description, semantic description, kinematics, etc.
    # It also loads the controller configuration that MoveIt will use to send commands.
    # We are keeping the trajectory_execution part because it tells MoveIt WHICH
    # controller action name to publish to (e.g., /panda_arm_controller/follow_joint_trajectory)
    # ###################################################################
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .planning_pipelines()
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Load ExecuteTaskSolutionCapability so we can execute found solutions
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    # ###################################################################
    # This is the main MoveIt node. It does all the planning.
    # ###################################################################
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    # ###################################################################
    # Starts RViz for visualization and interaction.
    # ###################################################################
    rviz_config_file = (
        get_package_share_directory("mtc_dynamic") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Robot State Publisher
    # ###################################################################
    # This node is still needed. It takes the /joint_states from your
    # real robot (via the bridge) and the robot_description (URDF)
    # and publishes the TF tree for RViz to display the robot model correctly.
    # ###################################################################
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # ###################################################################
    # REMOVED SECTIONS:
    # - static_tf_publisher: The TF from 'world' to 'panda_link0' should ideally be
    #   handled by your robot's driver setup or another dedicated node.
    # - ros2_control_node: This was the simulation backend (FakeSystem). Removed.
    # - load_controllers: These were for the simulation. Removed.
    # ###################################################################

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
        ]
    )
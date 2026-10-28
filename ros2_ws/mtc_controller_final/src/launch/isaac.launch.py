"""MoveIt 2 + MTC pick & place against the Isaac Sim digital twin (closed loop).

The ros2_control controller_manager runs inside Isaac Sim (isaacsim.ros2.control), so there is
no ros2_control_node here: the spawners load the 3 controllers into Isaac's controller_manager,
then pick_place_demo runs with the generated task parameters. Everything uses sim time (/clock
from Isaac). The launch shuts down when pick_place_demo exits.

Usage
-----
    ros2 launch mtc_dynamic isaac.launch.py task:=/abs/path/output/twin_res5/mtc_task.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory("mtc_dynamic")
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path=os.path.join(share, "config", "moveit_controllers_isaac.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    sim = {"use_sim_time": True}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), sim, {"capabilities": "move_group/ExecuteTaskSolutionCapability"}],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description, sim],
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        parameters=[sim],
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "panda_arm_controller", "panda_hand_controller",
                   "--controller-manager-timeout", "120"],
    )
    demo = Node(
        package="mtc_dynamic",
        executable="pick_place_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            os.path.join(share, "config", "panda_config.yaml"),
            LaunchConfiguration("task"),
            sim,
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("task", description="mtc_task.yaml from src/sim_export/export.py"),
        robot_state_publisher,
        static_tf,
        move_group,
        spawner,
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=[demo])),
        RegisterEventHandler(OnProcessExit(target_action=demo, on_exit=[EmitEvent(event=Shutdown())])),
    ])

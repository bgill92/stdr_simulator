"""Bring up stdr_server with maze1, the GUI, and spawn trin_bot at a configurable pose."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    stdr_resources = get_package_share_directory("stdr_resources")
    map_file = os.path.join(stdr_resources, "maps", "maze1.yaml")
    robot_yaml = os.path.join(stdr_resources, "resources", "robots", "trin_bot.yaml")

    x_arg = DeclareLaunchArgument(
        "x", default_value="1", description="Initial x position"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="2", description="Initial y position"
    )
    theta_arg = DeclareLaunchArgument(
        "theta", default_value="3.14", description="Initial heading in radians"
    )

    stdr_server = Node(
        package="stdr_server",
        executable="stdr_server_node",
        name="stdr_server",
        output="screen",
        parameters=[{"map_file": map_file}],
    )

    world_to_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world2map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )

    stdr_gui = Node(
        package="stdr_gui_ros",
        executable="stdr_gui_ros_node",
        name="stdr_gui",
        output="screen",
    )

    # robot_spawner waits up to 30 s for the action server.
    robot_spawner = Node(
        package="stdr_robot",
        executable="robot_spawner",
        name="robot_spawner",
        output="screen",
        arguments=[
            "--description",
            robot_yaml,
            "--x",
            LaunchConfiguration("x"),
            "--y",
            LaunchConfiguration("y"),
            "--theta",
            LaunchConfiguration("theta"),
        ],
    )

    return LaunchDescription(
        [
            x_arg,
            y_arg,
            theta_arg,
            stdr_server,
            world_to_map_tf,
            stdr_gui,
            robot_spawner,
        ]
    )

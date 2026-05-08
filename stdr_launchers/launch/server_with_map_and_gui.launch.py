"""Bring up stdr_server with sparse_obstacles.yaml, the GUI, and a static world→map transform."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    stdr_resources = get_package_share_directory("stdr_resources")
    map_file = os.path.join(stdr_resources, "maps", "sparse_obstacles.yaml")

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

    return LaunchDescription(
        [
            stdr_server,
            world_to_map_tf,
            stdr_gui,
        ]
    )

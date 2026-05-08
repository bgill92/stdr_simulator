"""Bring up stdr_server (no map) and a static world→map transform."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    stdr_server = Node(
        package="stdr_server",
        executable="stdr_server_node",
        name="stdr_server",
        output="screen",
    )

    world_to_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world2map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )

    return LaunchDescription(
        [
            stdr_server,
            world_to_map_tf,
        ]
    )

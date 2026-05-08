"""Launch rviz2 with the STDR simulator config."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("stdr_launchers"),
            "rviz",
            "config.rviz",
        ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz2,
        ]
    )

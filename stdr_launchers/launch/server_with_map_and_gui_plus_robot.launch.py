"""Bring up stdr_server, GUI, world→map TF, and spawn pandora_robot at (1, 2, 0)."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    stdr_resources = get_package_share_directory("stdr_resources")
    map_file = os.path.join(stdr_resources, "maps", "sparse_obstacles.yaml")
    robot_yaml = os.path.join(
        stdr_resources, "resources", "robots", "pandora_robot.yaml"
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

    # robot_spawner waits up to 30 s for the action server, so ordering
    # within the LaunchDescription is sufficient — no explicit delay needed.
    # name= is intentionally omitted: the process creates two nodes
    # (robot_spawner_client and StdrRobotNode) and a launch-side name= remap
    # would alias both to the same name, causing the second registration to fail.
    robot_spawner = Node(
        package="stdr_robot",
        executable="robot_spawner",
        output="screen",
        arguments=["--description", robot_yaml, "--x", "1", "--y", "2", "--theta", "0"],
    )

    return LaunchDescription(
        [
            stdr_server,
            world_to_map_tf,
            stdr_gui,
            robot_spawner,
        ]
    )

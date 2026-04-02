from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="ksu_storm",
                executable="robot_node",
                name="ksu_storm_robot",
                output="screen",
            ),
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("apriltag_amr"), "config", "mission_demo.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="apriltag_amr",
                executable="multi_camera_object_mission",
                name="multi_camera_object_mission",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="apriltag_amr",
                executable="demo_loop_runner",
                name="demo_loop_runner",
                parameters=[params_file],
                output="screen",
            ),
        ]
    )

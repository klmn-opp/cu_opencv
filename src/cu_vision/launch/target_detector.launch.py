from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("target_color", default_value="red"),
            DeclareLaunchArgument("min_area", default_value="150.0"),
            DeclareLaunchArgument("display", default_value="false"),
            DeclareLaunchArgument("preview_scale", default_value="1.0"),
            Node(
                package="cu_vision",
                executable="target_detector",
                name="target_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "target_color": LaunchConfiguration("target_color"),
                        "min_area": LaunchConfiguration("min_area"),
                        "display": LaunchConfiguration("display"),
                        "preview_scale": LaunchConfiguration("preview_scale"),
                    }
                ],
            ),
        ]
    )

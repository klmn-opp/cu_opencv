from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera_info"),
            DeclareLaunchArgument("target_color", default_value="red"),
            DeclareLaunchArgument("min_area", default_value="150.0"),
            DeclareLaunchArgument("display", default_value="false"),
            DeclareLaunchArgument("preview_scale", default_value="1.0"),
            DeclareLaunchArgument("shape_epsilon_ratio", default_value="0.03"),
            DeclareLaunchArgument("hsv_s_min", default_value="50"),#45
            DeclareLaunchArgument("hsv_v_min", default_value="40"),#35
            DeclareLaunchArgument("channel_delta", default_value="40"),#35
            DeclareLaunchArgument("close_kernel_size", default_value="7"),
            DeclareLaunchArgument("close_iterations", default_value="1"),
            DeclareLaunchArgument("open_kernel_size", default_value="3"),
            DeclareLaunchArgument("min_vertices", default_value="4"),
            DeclareLaunchArgument("max_vertices", default_value="6"),
            DeclareLaunchArgument("min_solidity", default_value="0.65"),
            DeclareLaunchArgument("filter_enable", default_value="true"),
            DeclareLaunchArgument("min_confirm_frames", default_value="3"),
            DeclareLaunchArgument("max_missed_frames", default_value="5"),
            DeclareLaunchArgument("filter_alpha", default_value="0.35"),
            DeclareLaunchArgument("max_center_jump_px", default_value="220.0"),
            DeclareLaunchArgument("enable_pnp", default_value="true"),
            DeclareLaunchArgument("target_side_m", default_value="1.0"),
            DeclareLaunchArgument("max_reprojection_error_px", default_value="8.0"),
            DeclareLaunchArgument("camera_fx", default_value="0.0"),
            DeclareLaunchArgument("camera_fy", default_value="0.0"),
            DeclareLaunchArgument("camera_cx", default_value="0.0"),
            DeclareLaunchArgument("camera_cy", default_value="0.0"),
            Node(
                package="cu_vision",
                executable="target_detector",
                name="target_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "target_color": LaunchConfiguration("target_color"),
                        "min_area": LaunchConfiguration("min_area"),
                        "display": LaunchConfiguration("display"),
                        "preview_scale": LaunchConfiguration("preview_scale"),
                        "shape_epsilon_ratio": LaunchConfiguration("shape_epsilon_ratio"),
                        "hsv_s_min": LaunchConfiguration("hsv_s_min"),
                        "hsv_v_min": LaunchConfiguration("hsv_v_min"),
                        "channel_delta": LaunchConfiguration("channel_delta"),
                        "close_kernel_size": LaunchConfiguration("close_kernel_size"),
                        "close_iterations": LaunchConfiguration("close_iterations"),
                        "open_kernel_size": LaunchConfiguration("open_kernel_size"),
                        "min_vertices": LaunchConfiguration("min_vertices"),
                        "max_vertices": LaunchConfiguration("max_vertices"),
                        "min_solidity": LaunchConfiguration("min_solidity"),
                        "filter_enable": LaunchConfiguration("filter_enable"),
                        "min_confirm_frames": LaunchConfiguration("min_confirm_frames"),
                        "max_missed_frames": LaunchConfiguration("max_missed_frames"),
                        "filter_alpha": LaunchConfiguration("filter_alpha"),
                        "max_center_jump_px": LaunchConfiguration("max_center_jump_px"),
                        "enable_pnp": LaunchConfiguration("enable_pnp"),
                        "target_side_m": LaunchConfiguration("target_side_m"),
                        "max_reprojection_error_px": LaunchConfiguration("max_reprojection_error_px"),
                        "camera_fx": LaunchConfiguration("camera_fx"),
                        "camera_fy": LaunchConfiguration("camera_fy"),
                        "camera_cx": LaunchConfiguration("camera_cx"),
                        "camera_cy": LaunchConfiguration("camera_cy"),
                    }
                ],
            ),
        ]
    )

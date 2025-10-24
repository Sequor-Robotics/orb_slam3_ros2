# Converted from zed_stereo_inertial.launch.yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch arguments (from YAML)
    vocab = LaunchConfiguration("vocab")
    config = LaunchConfiguration("config")
    rectify = LaunchConfiguration("rectify")
    equalize = LaunchConfiguration("equalize")
    camera_name = LaunchConfiguration("camera_name")
    downsample_method = LaunchConfiguration("downsample_method")

    declare_downsample_method = DeclareLaunchArgument(
        "downsample_method",
        default_value=TextSubstitution(text="area"),
        description="Downsampling method: auto | area | pyr (maps to ROS2 param 'downsample.method')",
    )

    declare_vocab = DeclareLaunchArgument(
        "vocab",
        default_value=PathJoinSubstitution([FindPackageShare("orbslam3"), "vocabulary", "ORBvoc.txt"]),
        description="Path to ORB-SLAM3 vocabulary file",
    )
    declare_config = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution([FindPackageShare("orbslam3"), "config", "stereo-inertial", "zed_ros1.yaml"]),
        description="Path to ORB-SLAM3 camera config",
    )
    declare_rectify = DeclareLaunchArgument(
        "rectify",
        default_value="false",
        description="Rectify input images (true/false)",
    )
    declare_equalize = DeclareLaunchArgument(
        "equalize",
        default_value="false",
        description="Histogram equalization (true/false)",
    )
    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="zedx",
        description="Base camera namespace (e.g., zed, zed2, zedx)",
    )

    slam_param_file = PathJoinSubstitution([
        FindPackageShare("orbslam3"),
        "config",
        "zed-stereo-inertial-slam.yaml",
    ])

    orbslam3_node = Node(
        package="orbslam3",
        executable="stereo-inertial",  # adjust if your binary name is different
        name="orbslam3",
        namespace=TextSubstitution(text="/orb_slam3"),
        # positional args expected by the C++ main: vocab, config, rectify, equalize
        arguments=[vocab, config, rectify, equalize],
        parameters=[
            slam_param_file,
            {"downsample.method": downsample_method},  # pass as ROS2 parameter
        ],
        remappings=[
            ("camera/left",
                [TextSubstitution(text="/"), camera_name,
                 TextSubstitution(text="/zed_node/left/image_rect_color")]),
            ("camera/right",
                [TextSubstitution(text="/"), camera_name,
                 TextSubstitution(text="/zed_node/right/image_rect_color")]),
            ("imu",
                [TextSubstitution(text="/"), camera_name,
                 TextSubstitution(text="/zed_node/imu/data")]),
        ],
        env=dict(os.environ),
    )

    return LaunchDescription([
        declare_downsample_method,
        declare_vocab,
        declare_config,
        declare_rectify,
        declare_equalize,
        declare_camera_name,
        orbslam3_node,
    ])

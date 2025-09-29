# Converted from zed_stereo_inertial.launch.yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import PathJoinSubstitution
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

    declare_vocab = DeclareLaunchArgument(
        "vocab",
        default_value=PathJoinSubstitution([FindPackageShare("orbslam3"), "vocabulary", "ORBvoc.txt"]),
        description="Path to ORB-SLAM3 vocabulary file",
    )
    declare_config = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution([FindPackageShare("orbslam3"), "config", "stereo-inertial", "zed.yaml"]),
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
        default_value="zed",
        description="Base camera namespace (e.g., zed, zed2, zedx)",
    )

    # Namespace like "/<camera_name>/zed_node"
    node_namespace = [camera_name, TextSubstitution(text="/zed_node")]

    # Parameter file from YAML
    slam_param_file = PathJoinSubstitution([
        FindPackageShare("orbslam3"),
        "config",
        "zed-stereo-inertial-slam.yaml",
    ])

    # NOTE:
    # The YAML did not explicitly list 'pkg' and 'exec' for the node.
    # We assume package="orbslam3" and executable="stereo_inertial".
    # If your executable name differs, update 'executable' accordingly.
    orbslam3_node = Node(
        package="orbslam3",
        executable="stereo-inertial",  # <-- adjust if your binary name is different
        prefix ="xterm -e gdb run --args",
        name="orbslam3",
        namespace=node_namespace,
        # Pass the same CLI args order as in YAML: vocab, config, rectify, equalize
        arguments=[vocab, config, rectify, equalize],
        parameters=[slam_param_file],
        # Remappings from YAML
        remappings=[
            ("camera/left",  "left/image_rect_color"),
            ("camera/right", "right/image_rect_color"),
            ("imu",          "imu/data"),
        ],
        env=dict(os.environ),
    )

    return LaunchDescription([
        declare_vocab,
        declare_config,
        declare_rectify,
        declare_equalize,
        declare_camera_name,
        orbslam3_node,
    ])

#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    # TODO: Add a launch args for camera name and namespace
    camera_name = "d455"
    camera_namespace = "d455" 

    robot_base_tf_base_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            f"{camera_name}_link",
        ],
    )
    # Realsense camera
    launch_file_realsense = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )
    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_realsense]),
        launch_arguments=[
            ("camera_name", camera_name),
            ("camera_namespace", camera_namespace),
            ("enable_infra1", "True"),
            ("enable_infra2", "True"),
            ("infra_width", "640"),
            ("infra_height", "480"),
            ("unite_imu_method", "2"),
            ("enable_color", "True"),
            ("enable_accel", "True"),
            ("enable_gyro", "True"),
        ],
    )

    # IMU filter
    imu_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        parameters=[
            {
                "use_mag": False,
                "publish_tf": False,
                "world_frame": "enu",
                "fixed_frame": f"{camera_name}_link",  # The parent frame to be used in publish_tf
            }
        ],
        remappings=[
            ("/imu/data_raw", f"{camera_namespace}/{camera_name}/imu"),
            ("/imu/data", "/rs_cam/imu"),
        ],
    )

    # Robot localization
    config_localization = PathJoinSubstitution(
        [FindPackageShare("orbslam3"), "config/ekf.yaml"]
    )
    filter_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[config_localization],
        remappings=[
            ("odometry/filtered", "platform/odom/filtered"),
            ("/diagnostics", "diagnostics"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    ld.add_action(launch_realsense)
    ld.add_action(imu_node)
    ld.add_action(robot_base_tf_base_camera)
    ld.add_action(filter_odom_node)

    return ld

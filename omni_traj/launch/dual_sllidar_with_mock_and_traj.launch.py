#!/usr/bin/env python3
# file: omni_traj/launch/dual_sllidar_with_mock_and_traj.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_mock_lidar = LaunchConfiguration("use_mock_lidar")
    use_rviz = LaunchConfiguration("use_rviz")

    channel_type = LaunchConfiguration("channel_type")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")

    lidar1_serial_port = LaunchConfiguration("lidar1_serial_port")
    lidar2_serial_port = LaunchConfiguration("lidar2_serial_port")
    lidar1_frame_id = LaunchConfiguration("lidar1_frame_id")
    lidar2_frame_id = LaunchConfiguration("lidar2_frame_id")

    rviz_config_path = os.path.join(
        get_package_share_directory("sllidar_ros2"),
        "rviz",
        "sllidar_ros2.rviz",
    )

    sllidar_common_params = {
        "channel_type": channel_type,
        "serial_baudrate": ParameterValue(serial_baudrate, value_type=int),
        "inverted": ParameterValue(inverted, value_type=bool),
        "angle_compensate": ParameterValue(angle_compensate, value_type=bool),
        "scan_mode": scan_mode,
    }

    # =========================
    # REAL LIDARS
    # =========================
    lidar1_real = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="lidar1",
        output="screen",
        parameters=[{**sllidar_common_params, "serial_port": lidar1_serial_port, "frame_id": lidar1_frame_id}],
        remappings=[("scan", "/lidar1/scan")],
        condition=UnlessCondition(use_mock_lidar),
    )

    lidar2_real_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="lidar2",
        output="screen",
        parameters=[{**sllidar_common_params, "serial_port": lidar2_serial_port, "frame_id": lidar2_frame_id}],
        remappings=[("scan", "/lidar2/scan")],
        condition=UnlessCondition(use_mock_lidar),
    )

    lidar2_real = TimerAction(
        period=0.5,
        actions=[lidar2_real_node],
        condition=UnlessCondition(use_mock_lidar),
    )

    # =========================
    # MOCK SCANS
    # =========================
    lidar1_mock = Node(
        package="omni_traj",
        executable="empty_scan_pub",
        name="lidar1_empty",
        output="screen",
        parameters=[{"topic": "/lidar1/scan", "frame_id": lidar1_frame_id, "rate_hz": 10.0, "num_readings": 360, "range_max": 10.0}],
        condition=IfCondition(use_mock_lidar),
    )

    lidar2_mock = Node(
        package="omni_traj",
        executable="empty_scan_pub",
        name="lidar2_empty",
        output="screen",
        parameters=[{"topic": "/lidar2/scan", "frame_id": lidar2_frame_id, "rate_hz": 10.0, "num_readings": 360, "range_max": 10.0}],
        condition=IfCondition(use_mock_lidar),
    )

    # =========================
    # Static TFs: world->odom and base_link -> lidars
    # LIDARs are at y=+0.10m and y=-0.10m, both on y-axis, facing forward on x
    # =========================
    
    # world -> odom (fixed frame for RViz visualization)
    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom",
        arguments=[
            "0.0", "0.0", "0.0",
            "0", "0", "0",
            "world", "odom",
        ],
        output="screen",
    )

    # odom -> base_link (published dynamically by the main node, but provide static fallback)
    # This is overridden by publish_odom_to_base_tf in the main node
    
    # base_link -> lidar1 (at y=+0.10m, no z offset)
    base_to_lidar1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar1",
        arguments=[
            "0.0", "0.10", "0.0",
            "0", "0", "0",
            "base_link", lidar1_frame_id,
        ],
        output="screen",
    )

    # base_link -> lidar2 (at y=-0.10m, no z offset)
    base_to_lidar2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar2",
        arguments=[
            "0.0", "-0.10", "0.0",
            "0", "0", "0",
            "base_link", lidar2_frame_id,
        ],
        output="screen",
    )

    # =========================
    # Main node
    # =========================
    traj = Node(
        package="omni_traj",
        executable="waypoint_traj",
        name="waypoint_traj",
        output="screen",
        parameters=[
            {
                # frames
                "map_frame": "odom",
                "base_frame": "base_link",

                # publish odom->base from /odom (set false if something else already publishes it)
                "publish_odom_to_base_tf": True,

                # robot footprint exclusion
                "robot_exclusion_enable": True,
                "robot_exclusion_radius_m": 0.22,

                # waypoint removal
                "waypoint_reached_tol_m": 0.10,

                # map/costmap sizing
                "global_map_res": 0.02,
                "global_map_width_m": 6.0,
                "global_map_height_m": 6.0,

                # inflation (NOTE: hard inflation now actually works)
                "hard_inflate_radius": 0.22,
                "soft_inflate_radius": 0.0,

                # scan
                "scan_max_age_s": 0.5,
                "scan_beam_stride": 1,

                # fused scan for RViz (display /scan_fused)
                "publish_fused_scan": True,
                "fused_angle_increment_deg": 1.0,
                "motion_compensate": False,  # set True if robot moves

                # start pose
                "start_pose": [0.0, 0.0, 0.0],

                # waypoints
                "wp_n": 0,
                "waypoints": [float("nan"), float("nan")],
                "add_wp": [float("nan"), float("nan")],

                # lidar topics
                "lidar1_topic": "/lidar1/scan",
                "lidar2_topic": "/lidar2/scan",
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_lidar", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("channel_type", default_value="serial"),
            DeclareLaunchArgument("serial_baudrate", default_value="460800"),
            DeclareLaunchArgument("inverted", default_value="false"),
            DeclareLaunchArgument("angle_compensate", default_value="true"),
            DeclareLaunchArgument("scan_mode", default_value="Standard"),
            DeclareLaunchArgument("lidar1_serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar2_serial_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("lidar1_frame_id", default_value="lidar1_link"),
            DeclareLaunchArgument("lidar2_frame_id", default_value="lidar2_link"),

            lidar1_real,
            lidar2_real,
            lidar1_mock,
            lidar2_mock,

            world_to_odom,
            base_to_lidar1,
            base_to_lidar2,

            traj,
            rviz,
        ]
    )

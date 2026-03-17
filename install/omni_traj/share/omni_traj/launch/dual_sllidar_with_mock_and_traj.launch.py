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
    rviz_config = LaunchConfiguration("rviz_config")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    publish_odom_to_base_tf = LaunchConfiguration("publish_odom_to_base_tf")
    publish_world_to_odom_tf = LaunchConfiguration("publish_world_to_odom_tf")
    rolling_map_enable = LaunchConfiguration("rolling_map_enable")
    rolling_map_margin_m = LaunchConfiguration("rolling_map_margin_m")
    persistent_obstacles_enable = LaunchConfiguration("persistent_obstacles_enable")
    enable_amcl_localization = LaunchConfiguration("enable_amcl_localization")
    amcl_params_file = LaunchConfiguration("amcl_params_file")

    channel_type = LaunchConfiguration("channel_type")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")

    lidar1_serial_port = LaunchConfiguration("lidar1_serial_port")
    lidar2_serial_port = LaunchConfiguration("lidar2_serial_port")
    lidar1_frame_id = LaunchConfiguration("lidar1_frame_id")
    lidar2_frame_id = LaunchConfiguration("lidar2_frame_id")

    traj_params_file = LaunchConfiguration("traj_params_file")

    traj_params_path = os.path.join(
        get_package_share_directory("omni_traj"),
        "config",
        "waypoint_traj.yaml",
    )

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
        namespace="lidar1",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{**sllidar_common_params, "serial_port": lidar1_serial_port, "frame_id": lidar1_frame_id}],
        remappings=[("scan", "/lidar1/scan")],
        condition=UnlessCondition(use_mock_lidar),
    )

    lidar2_real_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="lidar2",
        namespace="lidar2",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{**sllidar_common_params, "serial_port": lidar2_serial_port, "frame_id": lidar2_frame_id}],
        remappings=[("scan", "/lidar2/scan")],
        condition=UnlessCondition(use_mock_lidar),
    )

    lidar2_real = TimerAction(
        period=2.0,
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
        condition=IfCondition(publish_world_to_odom_tf),
    )

    # odom -> base_link (static fallback for prototyping without pose data)
    # This is overridden by publish_odom_to_base_tf when pose data is available
    odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base",
        arguments=[
            "0.0", "0.0", "0.0",
            "0", "0", "0",
            "odom", "base_link",
        ],
        output="screen",
        condition=UnlessCondition(publish_odom_to_base_tf),
    )
    
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
            traj_params_file,
            {
                # frames
                "map_frame": map_frame,
                "odom_frame": odom_frame,

                # publish odom->base from /odom (set false if something else already publishes it)
                "publish_odom_to_base_tf": ParameterValue(publish_odom_to_base_tf, value_type=bool),
                "rolling_map_enable": ParameterValue(rolling_map_enable, value_type=bool),
                "rolling_map_margin_m": ParameterValue(rolling_map_margin_m, value_type=float),
                "persistent_obstacles_enable": ParameterValue(persistent_obstacles_enable, value_type=bool),
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_params_file,
            {
                "use_map_topic": True,
                "scan_topic": "/scan_fused",
                "global_frame_id": map_frame,
                "odom_frame_id": odom_frame,
                "base_frame_id": "base_link",
                "tf_broadcast": True,
            },
        ],
        condition=IfCondition(enable_amcl_localization),
    )

    amcl_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["amcl"],
                "bond_timeout": 4.0,
            }
        ],
        condition=IfCondition(enable_amcl_localization),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_lidar", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config_path),
            DeclareLaunchArgument("traj_params_file", default_value=traj_params_path),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("publish_odom_to_base_tf", default_value="true"),
            DeclareLaunchArgument("publish_world_to_odom_tf", default_value="false"),
            DeclareLaunchArgument("rolling_map_enable", default_value="true"),
            DeclareLaunchArgument("rolling_map_margin_m", default_value="1.0"),
            DeclareLaunchArgument("persistent_obstacles_enable", default_value="false"),
            DeclareLaunchArgument("enable_amcl_localization", default_value="true"),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=os.path.join(get_package_share_directory("omni_traj"), "config", "amcl_localization.yaml"),
            ),
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
            odom_to_base,
            base_to_lidar1,
            base_to_lidar2,

            traj,
            amcl,
            amcl_lifecycle,
            rviz,
        ]
    )

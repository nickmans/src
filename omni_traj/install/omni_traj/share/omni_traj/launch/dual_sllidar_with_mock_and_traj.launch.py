#!/usr/bin/env python3
# file: omni_traj/launch/dual_sllidar_with_mock_and_traj.launch.py

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    enable_slam_toolbox = LaunchConfiguration("enable_slam_toolbox")
    enable_nav2_costmaps = LaunchConfiguration("enable_nav2_costmaps")
    nav2_costmaps_params_file = LaunchConfiguration("nav2_costmaps_params_file")
    scan_match_topic = LaunchConfiguration("scan_match_topic")
    enable_map_odom_startup_fallback = LaunchConfiguration("enable_map_odom_startup_fallback")
    map_odom_startup_fallback_grace_s = LaunchConfiguration("map_odom_startup_fallback_grace_s")
    map_odom_startup_fallback_backdate_s = LaunchConfiguration("map_odom_startup_fallback_backdate_s")
    slam_params_file = LaunchConfiguration("slam_params_file")
    enable_lidar_watchdog = LaunchConfiguration("enable_lidar_watchdog")
    lidar_watchdog_scan_timeout_s = LaunchConfiguration("lidar_watchdog_scan_timeout_s")
    lidar_watchdog_start_retry_s = LaunchConfiguration("lidar_watchdog_start_retry_s")
    lidar_watchdog_startup_grace_s = LaunchConfiguration("lidar_watchdog_startup_grace_s")

    channel_type = LaunchConfiguration("channel_type")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")

    lidar1_serial_port = LaunchConfiguration("lidar1_serial_port")
    lidar2_serial_port = LaunchConfiguration("lidar2_serial_port")
    lidar1_frame_id = LaunchConfiguration("lidar1_frame_id")
    lidar2_frame_id = LaunchConfiguration("lidar2_frame_id")
    lidar1_y_m = LaunchConfiguration("lidar1_y_m")
    lidar2_y_m = LaunchConfiguration("lidar2_y_m")
    lidar_yaw_rad = LaunchConfiguration("lidar_yaw_rad")

    traj_params_file = LaunchConfiguration("traj_params_file")

    pkg_share = get_package_share_directory("omni_traj")
    installed_traj_params_path = os.path.join(pkg_share, "config", "waypoint_traj.yaml")

    pkg_prefix = get_package_prefix("omni_traj")
    source_root_guess = os.path.dirname(os.path.dirname(pkg_prefix))
    source_traj_params_path = os.path.join(source_root_guess, "config", "waypoint_traj.yaml")
    traj_params_path = source_traj_params_path if os.path.exists(source_traj_params_path) else installed_traj_params_path

    installed_rviz_config_path = os.path.join(pkg_share, "rviz", "omni_nav.rviz")
    source_rviz_config_path = os.path.join(source_root_guess, "rviz", "omni_nav.rviz")
    rviz_config_path = source_rviz_config_path if os.path.exists(source_rviz_config_path) else installed_rviz_config_path

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
    # LIDARs are at y=+0.10m and y=-0.10m, both on y-axis.
    # STM32 yaw and LiDAR x-forward now use the same forward convention, so
    # the base_link -> lidar transforms must remain zero-yaw unless hardware
    # mounting changes again.
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

    # map -> odom fallback when no localization source publishes map->odom
    # Enabled only when both AMCL and SLAM are disabled.
    map_to_odom_fallback = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_fallback",
        arguments=[
            "0.0", "0.0", "0.0",
            "0", "0", "0",
            map_frame, odom_frame,
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                '"', enable_amcl_localization, '" == "false" and "',
                enable_slam_toolbox, '" == "false"'
            ])
        ),
    )

    map_odom_startup_fallback = Node(
        package="omni_traj",
        executable="map_odom_startup_fallback",
        name="map_odom_startup_fallback",
        output="screen",
        parameters=[
            {
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "publish_rate_hz": 12.0,
                "grace_period_s": ParameterValue(map_odom_startup_fallback_grace_s, value_type=float),
                "timestamp_backdate_s": ParameterValue(map_odom_startup_fallback_backdate_s, value_type=float),
            }
        ],
        condition=IfCondition(
            PythonExpression([
                '"', enable_map_odom_startup_fallback, '" == "true" and ((',
                '"', enable_amcl_localization, '" == "true" and "',
                enable_slam_toolbox, '" == "false") or ("',
                enable_amcl_localization, '" == "false" and "',
                enable_slam_toolbox, '" == "true"))'
            ])
        ),
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
    
    # base_link -> lidar1
    # Robot-forward convention: +x forward, +y left.
    # lidar1 is mounted on the physical left side and faces forward in base_link.
    base_to_lidar1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar1",
        arguments=[
            "--x", "0.0",
            "--y", lidar1_y_m,
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", lidar_yaw_rad,
            "--frame-id", "base_link",
            "--child-frame-id", lidar1_frame_id,
        ],
        output="screen",
    )

    # base_link -> lidar2
    # lidar2 is mounted on the physical right side and faces forward in base_link.
    base_to_lidar2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar2",
        arguments=[
            "--x", "0.0",
            "--y", lidar2_y_m,
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", lidar_yaw_rad,
            "--frame-id", "base_link",
            "--child-frame-id", lidar2_frame_id,
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
                "lidar1_frame_id": lidar1_frame_id,
                "lidar2_frame_id": lidar2_frame_id,
            }
        ],
    )

    lidar_watchdog = Node(
        package="omni_traj",
        executable="lidar_watchdog",
        name="lidar_watchdog",
        output="screen",
        parameters=[
            {
                "scan_topic_1": "/lidar1/scan",
                "scan_topic_2": "/lidar2/scan",
                "start_service_1": "/lidar1/start_motor",
                "start_service_2": "/lidar2/start_motor",
                "process_node_name_1": "lidar1",
                "process_node_name_2": "lidar2",
                "scan_timeout_s": ParameterValue(lidar_watchdog_scan_timeout_s, value_type=float),
                "start_motor_retry_s": ParameterValue(lidar_watchdog_start_retry_s, value_type=float),
                "startup_grace_s": ParameterValue(lidar_watchdog_startup_grace_s, value_type=float),
            }
        ],
        condition=IfCondition(
            PythonExpression([
                '"', use_mock_lidar, '" == "false" and "', enable_lidar_watchdog, '" == "true"'
            ])
        ),
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
                "scan_topic": scan_match_topic,
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

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file],
        remappings=[
            ("/scan", scan_match_topic),
            ("/map", "/slam_map"),
            ("/map_metadata", "/slam_map_metadata"),
        ],
        condition=IfCondition(enable_slam_toolbox),
    )

    slam_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["slam_toolbox"],
                "bond_timeout": 20.0,
            }
        ],
        condition=IfCondition(enable_slam_toolbox),
    )

    slam_lifecycle = TimerAction(
        period=4.0,
        actions=[slam_lifecycle_node],
        condition=IfCondition(enable_slam_toolbox),
    )

    nav2_local_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="local_costmap",
        namespace="local_costmap",
        output="screen",
        parameters=[nav2_costmaps_params_file],
        condition=IfCondition(enable_nav2_costmaps),
    )

    nav2_global_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="global_costmap",
        namespace="global_costmap",
        output="screen",
        parameters=[nav2_costmaps_params_file],
        condition=IfCondition(enable_nav2_costmaps),
    )

    nav2_costmaps_start = TimerAction(
        period=8.0,
        actions=[
            nav2_local_costmap,
            nav2_global_costmap,
        ],
        condition=IfCondition(enable_nav2_costmaps),
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
            DeclareLaunchArgument("enable_amcl_localization", default_value="false"),
            DeclareLaunchArgument("enable_slam_toolbox", default_value="true"),
            DeclareLaunchArgument("enable_nav2_costmaps", default_value="true"),
            DeclareLaunchArgument(
                "nav2_costmaps_params_file",
                default_value=os.path.join(get_package_share_directory("omni_traj"), "config", "nav2_dual_scan_costmaps.yaml"),
            ),
            DeclareLaunchArgument("scan_match_topic", default_value="/scan_match"),
            DeclareLaunchArgument("enable_map_odom_startup_fallback", default_value="true"),
            DeclareLaunchArgument("map_odom_startup_fallback_grace_s", default_value="25.0"),
            DeclareLaunchArgument("map_odom_startup_fallback_backdate_s", default_value="0.25"),
            DeclareLaunchArgument("enable_lidar_watchdog", default_value="true"),
            DeclareLaunchArgument("lidar_watchdog_scan_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("lidar_watchdog_start_retry_s", default_value="12.0"),
            DeclareLaunchArgument("lidar_watchdog_startup_grace_s", default_value="20.0"),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=os.path.join(get_package_share_directory("omni_traj"), "config", "amcl_localization.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(get_package_share_directory("omni_traj"), "config", "slam_toolbox_online_async.yaml"),
            ),
            DeclareLaunchArgument("channel_type", default_value="serial"),
            DeclareLaunchArgument("serial_baudrate", default_value="460800"),
            DeclareLaunchArgument("inverted", default_value="false"),
            DeclareLaunchArgument("angle_compensate", default_value="true"),
            DeclareLaunchArgument("scan_mode", default_value=""),
            DeclareLaunchArgument("lidar1_serial_port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("lidar2_serial_port", default_value="/dev/ttyAMA2"),
            DeclareLaunchArgument("lidar1_frame_id", default_value="lidar1_link"),
            DeclareLaunchArgument("lidar2_frame_id", default_value="lidar2_link"),
            DeclareLaunchArgument("lidar1_y_m", default_value="0.10"),
            DeclareLaunchArgument("lidar2_y_m", default_value="-0.10"),
            DeclareLaunchArgument("lidar_yaw_rad", default_value="3.141592653589793"),

            lidar1_real,
            lidar2_real,
            lidar1_mock,
            lidar2_mock,

            world_to_odom,
            map_to_odom_fallback,
            map_odom_startup_fallback,
            odom_to_base,
            base_to_lidar1,
            base_to_lidar2,

            traj,
            lidar_watchdog,
            amcl,
            amcl_lifecycle,
            slam_toolbox,
            slam_lifecycle,
            nav2_costmaps_start,
            rviz,
        ]
    )

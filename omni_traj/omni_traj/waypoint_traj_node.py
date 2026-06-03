#!/usr/bin/env python3
# file: omni_traj/waypoint_traj_node.py

from __future__ import annotations

import base64
import glob
import heapq
import json
import math
import os
import struct
import time
import zlib
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Float64MultiArray
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import yaml
except ImportError:
    yaml = None


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


SAVED_MAP_POSE_CONVENTION = "stm32_yaw_matches_lidar_x_forward_v1"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ScanSample:
    scan: LaserScan
    source_stamp_ns: int
    arrival_stamp_ns: int


@dataclass(frozen=True)
class GridSpec:
    res: float
    width: int
    height: int
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class GeofenceRect:
    cx: float
    cy: float
    yaw: float
    half_width: float
    half_height: float


class WaypointTrajNode(Node):
    """
    Dual-LiDAR fused costmap builder.

    Key fixes:
    - Proper inflation (hard inflation works even when soft=0)
    - Publish a fused LaserScan in base_link: /scan_fused
    - Publish exact fused points in base_link: /points_fused
    - Build the live obstacle costmap from exact transformed points
    - Clear robot footprint circle (0.22m radius) around base_link center
    - Pop reached waypoints within 0.10m
    """

    def __init__(self, parameter_overrides: Optional[List[Parameter]] = None) -> None:
        super().__init__("waypoint_traj", parameter_overrides=parameter_overrides or [])

        yaml_param_defaults = self._load_yaml_param_defaults()

        def cfg(name: str):
            if name in yaml_param_defaults:
                return yaml_param_defaults[name]
            raise RuntimeError(f"Missing required parameter '{name}' in waypoint_traj.yaml")

        # ===== Frames =====
        self.declare_parameter("map_frame", cfg("map_frame"))
        self.declare_parameter("odom_frame", cfg("odom_frame"))
        self.declare_parameter("base_frame", cfg("base_frame"))
        self.declare_parameter("prefer_tf_pose", cfg("prefer_tf_pose"))
        self.declare_parameter("pose_tf_timeout_s", cfg("pose_tf_timeout_s"))
        self.declare_parameter("odom_stale_timeout_s", cfg("odom_stale_timeout_s"))
        self.declare_parameter("odom_yaw_offset_rad", cfg("odom_yaw_offset_rad"))
        self.declare_parameter("tf_timestamp_backdate_s", cfg("tf_timestamp_backdate_s"))
        self.declare_parameter("main_loop_hz", cfg("main_loop_hz"))

        # If YOU already publish odom->base_link elsewhere, keep this false
        self.declare_parameter("publish_odom_to_base_tf", cfg("publish_odom_to_base_tf"))

        # ===== Robot exclusion =====
        self.declare_parameter("robot_exclusion_enable", cfg("robot_exclusion_enable"))
        self.declare_parameter("robot_exclusion_radius_m", cfg("robot_exclusion_radius_m"))  # diameter 0.44m

        # ===== Waypoint removal =====
        self.declare_parameter("waypoint_reached_tol_m", cfg("waypoint_reached_tol_m"))
        self.declare_parameter("remove_waypoint_radius_m", cfg("remove_waypoint_radius_m"))

        # ===== Global grid =====
        self.declare_parameter("global_map_res", cfg("global_map_res"))  # 5cm resolution
        self.declare_parameter("global_map_width_m", cfg("global_map_width_m"))
        self.declare_parameter("global_map_height_m", cfg("global_map_height_m"))
        self.declare_parameter("local_costmap_res", cfg("local_costmap_res"))
        self.declare_parameter("local_costmap_width_m", cfg("local_costmap_width_m"))
        self.declare_parameter("local_costmap_height_m", cfg("local_costmap_height_m"))
        self.declare_parameter("rolling_map_enable", cfg("rolling_map_enable"))
        self.declare_parameter("rolling_map_margin_m", cfg("rolling_map_margin_m"))
        self.declare_parameter("persistent_obstacles_enable", cfg("persistent_obstacles_enable"))
        self.declare_parameter("persistent_confirm_time_s", cfg("persistent_confirm_time_s"))
        self.declare_parameter("persistent_clear_time_s", cfg("persistent_clear_time_s"))
        self.declare_parameter("persistent_evidence_cap", cfg("persistent_evidence_cap"))
        self.declare_parameter("persistent_inf_clearing_enable", cfg("persistent_inf_clearing_enable"))
        self.declare_parameter("persistent_inf_clearing_ratio", cfg("persistent_inf_clearing_ratio"))
        self.declare_parameter("mapping_save_path", cfg("mapping_save_path"))
        self.declare_parameter("auto_load_saved_map", cfg("auto_load_saved_map"))
        self.declare_parameter("default_frozen_map_mode", cfg("default_frozen_map_mode"))
        self.declare_parameter("mapping_area_size_m", cfg("mapping_area_size_m"))
        self.declare_parameter("mapping_res_m", cfg("mapping_res_m"))
        self.declare_parameter("mapping_hit_logodd_inc", cfg("mapping_hit_logodd_inc"))
        self.declare_parameter("mapping_free_logodd_dec", cfg("mapping_free_logodd_dec"))
        self.declare_parameter("mapping_no_return_free_ratio", cfg("mapping_no_return_free_ratio"))
        self.declare_parameter("mapping_logodd_min", cfg("mapping_logodd_min"))
        self.declare_parameter("mapping_logodd_max", cfg("mapping_logodd_max"))
        self.declare_parameter("mapping_occ_threshold", cfg("mapping_occ_threshold"))
        self.declare_parameter("mapping_occ_set_threshold", cfg("mapping_occ_set_threshold"))
        self.declare_parameter("mapping_occ_clear_threshold", cfg("mapping_occ_clear_threshold"))
        self.declare_parameter("geofence_enable", cfg("geofence_enable"))
        self.declare_parameter("geofence_box_size_m", cfg("geofence_box_size_m"))
        self.declare_parameter("geofence_stop_margin_m", cfg("geofence_stop_margin_m"))
        self.declare_parameter("blank_geofence_enable", cfg("blank_geofence_enable"))
        self.declare_parameter("blank_geofence_width_m", cfg("blank_geofence_width_m"))
        self.declare_parameter("blank_geofence_height_m", cfg("blank_geofence_height_m"))
        self.declare_parameter("blank_geofence_align_to_start_yaw", cfg("blank_geofence_align_to_start_yaw"))

        # ===== Scans =====
        self.declare_parameter("lidar1_topic", cfg("lidar1_topic"))
        self.declare_parameter("lidar2_topic", cfg("lidar2_topic"))
        self.declare_parameter("lidar1_frame_id", cfg("lidar1_frame_id"))
        self.declare_parameter("lidar2_frame_id", cfg("lidar2_frame_id"))
        self.declare_parameter("scan_max_age_s", cfg("scan_max_age_s"))
        self.declare_parameter("use_arrival_stamp_for_scan_timing", cfg("use_arrival_stamp_for_scan_timing"))
        self.declare_parameter("max_scan_pair_skew_s", cfg("max_scan_pair_skew_s"))
        self.declare_parameter("scan_sync_queue_size", cfg("scan_sync_queue_size"))
        self.declare_parameter("scan_beam_stride", cfg("scan_beam_stride"))
        self.declare_parameter("scan_no_hit_eps_m", cfg("scan_no_hit_eps_m"))
        self.declare_parameter("max_lidar_range_m", cfg("max_lidar_range_m"))  # Max range for lidar processing (reduces CPU load)
        self.declare_parameter("scan_sensor_y_axis_right", cfg("scan_sensor_y_axis_right"))

        # ===== Fused scan output =====
        self.declare_parameter("publish_fused_scan", cfg("publish_fused_scan"))
        self.declare_parameter("publish_scan_match", cfg("publish_scan_match"))
        self.declare_parameter("scan_match_use_latest_tf_stamp", cfg("scan_match_use_latest_tf_stamp"))
        self.declare_parameter("publish_legacy_costmap", cfg("publish_legacy_costmap"))
        self.declare_parameter("fused_angle_min", cfg("fused_angle_min"))
        self.declare_parameter("fused_angle_max", cfg("fused_angle_max"))
        self.declare_parameter("fused_angle_increment_deg", cfg("fused_angle_increment_deg"))  # 0.5 degree bins
        self.declare_parameter("scan_match_angle_increment_deg", cfg("scan_match_angle_increment_deg"))
        self.declare_parameter("fused_voxel_merge_resolution_m", cfg("fused_voxel_merge_resolution_m"))
        self.declare_parameter("motion_compensate", cfg("motion_compensate"))         # set True if robot moves + you want de-warp

        # ===== Inflation =====
        self.declare_parameter("hard_inflate_radius", cfg("hard_inflate_radius"))  # typical = robot radius
        self.declare_parameter("soft_inflate_radius", cfg("soft_inflate_radius"))
        self.declare_parameter("planning_obstacle_hysteresis_enable", cfg("planning_obstacle_hysteresis_enable"))
        self.declare_parameter("planning_hard_obstacle_confirm_scans", cfg("planning_hard_obstacle_confirm_scans"))
        self.declare_parameter("planning_hard_obstacle_clear_scans", cfg("planning_hard_obstacle_clear_scans"))
        self.declare_parameter("cached_traj_reject_confirm_iters", cfg("cached_traj_reject_confirm_iters"))

        # ===== Robot kinematics & constraints =====
        self.declare_parameter("wheel_radius_m", cfg("wheel_radius_m"))           # Wheel radius in meters
        self.declare_parameter("wheelbase_m", cfg("wheelbase_m"))              # Distance from center to wheel (omni) or half-wheelbase
        self.declare_parameter("max_wheel_acceleration_ms2", cfg("max_wheel_acceleration_ms2")) # Max acceleration per wheel (m/s²)
        self.declare_parameter("max_wheel_jerk_rs3", cfg("max_wheel_jerk_rs3")) # Max wheel angular jerk (rad/s^3)
        self.declare_parameter("braking_decel_safety_factor", cfg("braking_decel_safety_factor")) # Derate braking to match closed-loop realizability
        self.declare_parameter("max_linear_velocity_ms", cfg("max_linear_velocity_ms"))    # Max linear velocity (m/s)
        self.declare_parameter("stm32_max_wheel_speed_rs", cfg("stm32_max_wheel_speed_rs"))
        self.declare_parameter("stm32_max_wheel_accel_rs2", cfg("stm32_max_wheel_accel_rs2"))
        self.declare_parameter("stm32_max_wheel_jerk_rs3", cfg("stm32_max_wheel_jerk_rs3"))
        self.declare_parameter("trajectory_limit_margin", cfg("trajectory_limit_margin"))
        self.declare_parameter("max_lateral_accel", cfg("max_lateral_accel"))        # Max lateral (centripetal) accel (m/s^2)
        self.declare_parameter("trajectory_replan_hz", cfg("trajectory_replan_hz"))     # Limit heavy trajectory re-generation frequency
        self.declare_parameter("trajectory_replan_min_move_m", cfg("trajectory_replan_min_move_m"))  # Replan sooner when robot moves this far
        self.declare_parameter("trajectory_replan_min_yaw_rad", cfg("trajectory_replan_min_yaw_rad"))  # Replan sooner when heading changes this much
        self.declare_parameter("trajectory_horizon_m", cfg("trajectory_horizon_m"))
        # Spline tuning knobs:
        # - For tighter turns / faster response: increase max_curvature, reduce blend_step, or reduce max_iters
        # - For gentler turns / less corner-cutting: decrease max_curvature, increase blend_step and/or max_iters
        self.declare_parameter("spline_sample_spacing_m", cfg("spline_sample_spacing_m"))  # [0.02-0.06] m; smaller = smoother/denser, larger = lighter CPU
        self.declare_parameter("spline_max_curvature", cfg("spline_max_curvature"))      # [~1.5-5.0] 1/m; lower = wider turns, <=0 disables curvature limiting
        self.declare_parameter("spline_curvature_blend_step", cfg("spline_curvature_blend_step"))  # [0.05-0.35]; higher = stronger pull toward polyline each iteration
        self.declare_parameter("spline_curvature_max_iters", cfg("spline_curvature_max_iters"))   # [2-12]; higher = more chances to satisfy curvature cap

        # ===== Odom history =====
        self.declare_parameter("odom_history_s", cfg("odom_history_s"))

        # ===== Waypoints =====
        self.declare_parameter(
            "waypoints",
            cfg("waypoints"),
            ParameterDescriptor(description="Flat list [x1,y1,(ignored), x2,y2,(ignored), ...]"),
        )
        self.declare_parameter(
            "add_wp",
            cfg("add_wp"),
            ParameterDescriptor(description="Set to [x,y] or [x,y,_] to append; node clears after consuming"),
        )
        self.declare_parameter("start_pose", cfg("start_pose"))
        self.declare_parameter("wp_n", cfg("wp_n"))

        # ===== TF =====
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._T_base_from_scan: Dict[str, object] = {}

        # ===== Odom =====
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 1)
        self.odom_pose_latest = Pose2D(0.0, 0.0, 0.0)
        self.odom_vel_body_latest: Tuple[float, float] = (0.0, 0.0)
        self.odom_vel_map_latest: Tuple[float, float] = (0.0, 0.0)
        self.odom_wz_latest: float = 0.0
        self.have_odom_pose = False
        self._last_odom_rx_ns: int = 0
        self._last_global_pose: Optional[Pose2D] = None
        self._mapping_pose_anchor_tf: Optional[object] = None
        self._odom_hist: Deque[Tuple[int, Pose2D]] = deque()

        # ===== LiDAR subs =====
        t1 = self.get_parameter("lidar1_topic").value
        t2 = self.get_parameter("lidar2_topic").value
        self.sub1 = self.create_subscription(LaserScan, t1, self.on_scan1, qos_profile_sensor_data)
        self.sub2 = self.create_subscription(LaserScan, t2, self.on_scan2, qos_profile_sensor_data)
        self.clicked_point_sub = self.create_subscription(PointStamped, "/clicked_point", self.on_clicked_point, 20)
        self.nav_goal_sub = self.create_subscription(PoseStamped, "/move_base_simple/goal", self.on_nav_goal, 20)
        self.nav2_goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.on_nav_goal, 20)
        self.remove_wp_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.on_remove_waypoint_pose,
            20,
        )
        self._scan1_buffer: Deque[ScanSample] = deque()
        self._scan2_buffer: Deque[ScanSample] = deque()

        # ===== Grid =====
        self.gs_map = self._make_global_grid_spec()
        # Use NumPy arrays for 50-80% faster grid operations
        self.static_occ = np.zeros((self.gs_map.height, self.gs_map.width), dtype=np.int8)
        self.persistent_evidence = np.zeros((self.gs_map.height, self.gs_map.width), dtype=np.int16)
        self._planning_hard_evidence = np.zeros((self.gs_map.height, self.gs_map.width), dtype=np.int16)
        self.mapping_active = False
        self.use_saved_map_only = False
        self.mapping_grid_spec: Optional[GridSpec] = None
        self.mapping_logodds: Optional[np.ndarray] = None
        self._mapping_occ_state: Optional[np.ndarray] = None
        self.saved_map_bounds: Optional[Tuple[float, float, float, float]] = None
        self.active_geofence_bounds: Optional[Tuple[float, float, float, float]] = None
        self.active_geofence_rect: Optional[GeofenceRect] = None
        self.geofence_anchor_corner: Optional[Tuple[float, float]] = None
        self.blank_mode_anchor_pose: Optional[Pose2D] = None

        self._init_mapping_grid(Pose2D(0.0, 0.0, 0.0))

        if bool(self.get_parameter("auto_load_saved_map").value):
            map_path = str(self.get_parameter("mapping_save_path").value)
            load_path, fallback_note = self._resolve_map_load_path(map_path)
            loaded, message = self._load_static_map(load_path)
            if fallback_note is not None:
                self.get_logger().warn(fallback_note)
            if loaded:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)

        self.use_saved_map_only = False
        if bool(self.get_parameter("default_frozen_map_mode").value):
            self.get_logger().warn(
                "default_frozen_map_mode is deprecated and ignored; live LiDAR costmap mode is always enabled."
            )

        # ===== Publishers =====
        # Use transient_local for /map so RViz2 can receive it even after subscribing
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.publish_legacy_costmap = bool(self.get_parameter("publish_legacy_costmap").value)
        self.costmap_pub = (
            self.create_publisher(OccupancyGrid, "/costmap", map_qos)
            if self.publish_legacy_costmap
            else None
        )
        self.scan_fused_pub = self.create_publisher(LaserScan, "/scan_fused", 10)
        self.scan_match_pub = self.create_publisher(LaserScan, "/scan_match", 10)
        self.points_fused_pub = self.create_publisher(PointCloud2, "/points_fused", 10)
        slam_map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid,
            "/slam_map",
            self.on_slam_map,
            slam_map_qos,
        )
        self._latest_slam_map: Optional[OccupancyGrid] = None
        self._latest_slam_map_ns: int = 0

        self.wp_marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 1)
        self.path_pub = self.create_publisher(Path, "/planned_path", 1)
        self.path_velocities_pub = self.create_publisher(Float64MultiArray, "/planned_path_velocities", 1)
        self.velocity_marker_pub = self.create_publisher(MarkerArray, "/path_velocity_markers", 1)
        self.robot_viz_pub = self.create_publisher(MarkerArray, "/robot_visualization", 1)
        self.geofence_marker_pub = self.create_publisher(MarkerArray, "/geofence_markers", 1)

        # ===== Services =====
        self.srv_clear_all_wp = self.create_service(Empty, "/clear_all_waypoints", self.handle_clear_all_waypoints)
        self.srv_pop_next_wp = self.create_service(Empty, "/pop_next_waypoint", self.handle_pop_next_waypoint)
        self.srv_mapping_start = self.create_service(Trigger, "/mapping/start", self.handle_mapping_start)
        self.srv_mapping_finish = self.create_service(Trigger, "/mapping/finish", self.handle_mapping_finish)
        self.srv_mapping_use_live = self.create_service(Trigger, "/mapping/use_live", self.handle_mapping_use_live)
        self.srv_mapping_use_frozen = self.create_service(Trigger, "/mapping/use_frozen", self.handle_mapping_use_frozen)
        self.srv_mapping_use_blank = self.create_service(Trigger, "/mapping/use_blank", self.handle_mapping_use_blank)
        self.srv_waypoint_test_pattern = self.create_service(
            Trigger,
            "/waypoints/generate_test_pattern",
            self.handle_generate_test_pattern_waypoints,
        )

        # Main loop rate is configurable; higher rates reduce visible pose lag.
        self._main_loop_hz = max(1.0, float(self.get_parameter("main_loop_hz").value))
        self.timer = self.create_timer(1.0 / self._main_loop_hz, self.on_timer)

        # Runtime load shedding to keep LiDAR processing responsive under CPU pressure
        self._effective_scan_stride = 1
        self._scan_stride_max = 4
        self._scan_beam_budget_per_scan = 1200
        self._timer_load_shed_ratio = 0.85
        self._timer_overrun_streak = 0
        self._timer_recover_streak = 0
        self._last_lidar_warn_ns = 0
        self._lidar_warn_period_ns = int(2.0 * 1e9)
        self._last_scan_rx_ns = 0

        self._last_tf_warn_ns = 0
        self._tf_warn_period_ns = int(1e9)

        # Queue RViz-clicked waypoints directly to avoid callback-time parameter write races
        self._pending_waypoints: Deque[Tuple[float, float]] = deque()
        self._auto_test_waypoint_pool: Deque[Tuple[float, float]] = deque()
        self._auto_test_waypoint_window: int = 5
        self._auto_test_waypoint_active: bool = False
        self._wp_test_toggle_armed: bool = False

        # Track consecutive iterations each waypoint sits in a hard cell
        self._wp_hard_cell_iters: Dict[Tuple[int, int], int] = {}

        # Trajectory planning cache (keeps publishers responsive under heavy planning load)
        self._cached_traj: Optional[Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]] = None
        self._cached_traj_collision_iters: int = 0
        self._last_plan_ns: int = 0
        self._last_plan_pose: Optional[Pose2D] = None
        self._last_plan_wps_sig: Optional[Tuple[Tuple[int, int], ...]] = None
        self._last_pose_for_waypoint_pop: Optional[Pose2D] = None
        self._last_published_map_crc: Optional[int] = None
        self._last_map_publish_ns: int = 0
        self._map_republish_period_ns: int = int(2.0 * 1e9)
        self._latest_fused_points_base: List[Tuple[float, float]] = []
        self._latest_scan_pairs: List[Tuple[LaserScan, int]] = []
        self._last_stable_pub_stamp_ns: int = 0
        self._warned_scan_match_latest_tf_stamp: bool = False

    def _load_yaml_param_defaults(self) -> Dict[str, object]:
        if yaml is None:
            raise RuntimeError("PyYAML is required to load waypoint_traj.yaml parameter defaults")

        try:
            pkg_share = get_package_share_directory("omni_traj")
            yaml_path = os.path.join(pkg_share, "config", "waypoint_traj.yaml")
            with open(yaml_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            ros_params = cfg.get("waypoint_traj", {}).get("ros__parameters", {})
            if not isinstance(ros_params, dict):
                raise RuntimeError("Invalid waypoint_traj.yaml format: ros__parameters must be a mapping")
            return ros_params
        except Exception as exc:
            raise RuntimeError(f"Failed to load waypoint_traj.yaml defaults: {exc}") from exc

    # =======================
    # Grid helpers
    # =======================
    def _make_global_grid_spec(self) -> GridSpec:
        res = float(self.get_parameter("global_map_res").value)
        w_m = float(self.get_parameter("global_map_width_m").value)
        h_m = float(self.get_parameter("global_map_height_m").value)
        width = int(round(w_m / res))
        height = int(round(h_m / res))
        origin_x = -0.5 * width * res
        origin_y = -0.5 * height * res
        return GridSpec(res=res, width=width, height=height, origin_x=origin_x, origin_y=origin_y)

    def _center_global_grid_on_pose(self, center_pose: Pose2D) -> None:
        """Re-center the fixed-size global grid window around a world pose."""
        width_m = self.gs_map.width * self.gs_map.res
        height_m = self.gs_map.height * self.gs_map.res
        res = self.gs_map.res

        origin_x = float(center_pose.x) - 0.5 * width_m
        origin_y = float(center_pose.y) - 0.5 * height_m

        # Keep origin aligned to whole cells so world<->grid indexing stays stable.
        origin_x = round(origin_x / res) * res
        origin_y = round(origin_y / res) * res

        self.gs_map = GridSpec(
            res=self.gs_map.res,
            width=self.gs_map.width,
            height=self.gs_map.height,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        # Force /map publish after origin updates even if occupancy bytes are unchanged.
        self._last_published_map_crc = None
        self._last_map_publish_ns = 0

    def _make_local_costmap_spec(self, base_pose_now: Pose2D) -> GridSpec:
        res = max(0.01, float(self.get_parameter("local_costmap_res").value))
        w_m = max(res, float(self.get_parameter("local_costmap_width_m").value))
        h_m = max(res, float(self.get_parameter("local_costmap_height_m").value))

        width = max(1, int(round(w_m / res)))
        height = max(1, int(round(h_m / res)))
        origin_x = float(base_pose_now.x) - 0.5 * width * res
        origin_y = float(base_pose_now.y) - 0.5 * height * res
        return GridSpec(res=res, width=width, height=height, origin_x=origin_x, origin_y=origin_y)

    @staticmethod
    def _shift_grid(grid: np.ndarray, width: int, height: int, shift_x: int, shift_y: int) -> np.ndarray:
        """
        Shift occupancy grid content into a new same-sized grid using NumPy slicing.
        Positive shift_x means world window moved +x, so old content appears at lower x indices.
        """
        if shift_x == 0 and shift_y == 0:
            return grid.copy()

        out = np.zeros((height, width), dtype=grid.dtype)
        
        # Compute valid source and destination regions for efficient array slicing
        y_src_start = max(0, shift_y)
        y_src_end = min(height, height + shift_y)
        y_dst_start = max(0, -shift_y)
        y_dst_end = min(height, height - shift_y)
        
        x_src_start = max(0, shift_x)
        x_src_end = min(width, width + shift_x)
        x_dst_start = max(0, -shift_x)
        x_dst_end = min(width, width - shift_x)
        
        # Single vectorized assignment instead of nested loops
        out[y_dst_start:y_dst_end, x_dst_start:x_dst_end] = \
            grid[y_src_start:y_src_end, x_src_start:x_src_end]
        
        return out

    def _maybe_roll_map(self, base_pose_now: Pose2D) -> None:
        if not bool(self.get_parameter("rolling_map_enable").value):
            return

        margin = max(0.0, float(self.get_parameter("rolling_map_margin_m").value))
        width_m = self.gs_map.width * self.gs_map.res
        height_m = self.gs_map.height * self.gs_map.res

        left = base_pose_now.x - self.gs_map.origin_x
        right = (self.gs_map.origin_x + width_m) - base_pose_now.x
        down = base_pose_now.y - self.gs_map.origin_y
        up = (self.gs_map.origin_y + height_m) - base_pose_now.y

        # Only re-center when robot gets close to map border
        if left > margin and right > margin and down > margin and up > margin:
            return

        new_origin_x = base_pose_now.x - 0.5 * width_m
        new_origin_y = base_pose_now.y - 0.5 * height_m

        # Snap to cell boundaries so grid-shift is integer cells and stable
        res = self.gs_map.res
        new_origin_x = round(new_origin_x / res) * res
        new_origin_y = round(new_origin_y / res) * res

        dx_cells = int(round((new_origin_x - self.gs_map.origin_x) / res))
        dy_cells = int(round((new_origin_y - self.gs_map.origin_y) / res))

        if dx_cells == 0 and dy_cells == 0:
            return

        self.static_occ = self._shift_grid(
            self.static_occ,
            self.gs_map.width,
            self.gs_map.height,
            dx_cells,
            dy_cells,
        )
        self.persistent_evidence = self._shift_grid(
            self.persistent_evidence,
            self.gs_map.width,
            self.gs_map.height,
            dx_cells,
            dy_cells,
        )
        self._planning_hard_evidence = self._shift_grid(
            self._planning_hard_evidence,
            self.gs_map.width,
            self.gs_map.height,
            dx_cells,
            dy_cells,
        )

        self.gs_map = GridSpec(
            res=self.gs_map.res,
            width=self.gs_map.width,
            height=self.gs_map.height,
            origin_x=float(new_origin_x),
            origin_y=float(new_origin_y),
        )

    def idx(self, gs: GridSpec, ix: int, iy: int) -> Tuple[int, int]:
        """Return (row, col) for NumPy 2D array indexing."""
        return (iy, ix)

    def in_bounds(self, gs: GridSpec, ix: int, iy: int) -> bool:
        return 0 <= ix < gs.width and 0 <= iy < gs.height

    def world_to_grid(self, gs: GridSpec, x: float, y: float) -> Tuple[int, int]:
        ix = int(math.floor((x - gs.origin_x) / gs.res))
        iy = int(math.floor((y - gs.origin_y) / gs.res))
        return ix, iy

    def grid_to_world(self, gs: GridSpec, ix: int, iy: int) -> Tuple[float, float]:
        x = gs.origin_x + (ix + 0.5) * gs.res
        y = gs.origin_y + (iy + 0.5) * gs.res
        return x, y

    def _bounds_from_grid_spec(self, gs: GridSpec) -> Tuple[float, float, float, float]:
        min_x = float(gs.origin_x)
        min_y = float(gs.origin_y)
        max_x = float(gs.origin_x + gs.width * gs.res)
        max_y = float(gs.origin_y + gs.height * gs.res)
        return (min_x, max_x, min_y, max_y)

    @staticmethod
    def _rect_from_bounds(bounds: Tuple[float, float, float, float]) -> GeofenceRect:
        gx_min, gx_max, gy_min, gy_max = bounds
        return GeofenceRect(
            cx=0.5 * (gx_min + gx_max),
            cy=0.5 * (gy_min + gy_max),
            yaw=0.0,
            half_width=max(1e-6, 0.5 * (gx_max - gx_min)),
            half_height=max(1e-6, 0.5 * (gy_max - gy_min)),
        )

    @staticmethod
    def _rect_corners(rect: GeofenceRect) -> List[Tuple[float, float]]:
        c = math.cos(rect.yaw)
        s = math.sin(rect.yaw)
        corners_local = [
            (-rect.half_width, -rect.half_height),
            ( rect.half_width, -rect.half_height),
            ( rect.half_width,  rect.half_height),
            (-rect.half_width,  rect.half_height),
        ]
        corners_world: List[Tuple[float, float]] = []
        for lx, ly in corners_local:
            wx = rect.cx + c * lx - s * ly
            wy = rect.cy + s * lx + c * ly
            corners_world.append((wx, wy))
        return corners_world

    @staticmethod
    def _rect_aabb(rect: GeofenceRect) -> Tuple[float, float, float, float]:
        corners = WaypointTrajNode._rect_corners(rect)
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        return (min(xs), max(xs), min(ys), max(ys))

    @staticmethod
    def _world_to_rect(rect: GeofenceRect, x: float, y: float) -> Tuple[float, float]:
        dx = float(x) - rect.cx
        dy = float(y) - rect.cy
        c = math.cos(rect.yaw)
        s = math.sin(rect.yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    @staticmethod
    def _rect_to_world(rect: GeofenceRect, x_local: float, y_local: float) -> Tuple[float, float]:
        c = math.cos(rect.yaw)
        s = math.sin(rect.yaw)
        return (
            rect.cx + c * float(x_local) - s * float(y_local),
            rect.cy + s * float(x_local) + c * float(y_local),
        )

    def _clear_geofence(self) -> None:
        self.active_geofence_bounds = None
        self.active_geofence_rect = None
        self.geofence_anchor_corner = None

    def _activate_geofence_rect(
        self,
        rect: GeofenceRect,
        *,
        anchor_corner: Optional[Tuple[float, float]] = None,
    ) -> None:
        self.active_geofence_rect = rect
        self.active_geofence_bounds = self._rect_aabb(rect)
        self.geofence_anchor_corner = anchor_corner

    def _compute_corner_geofence(self, base_pose_now: Pose2D) -> Optional[Tuple[float, float, float, float]]:
        if self.saved_map_bounds is None:
            return None

        min_x, max_x, min_y, max_y = self.saved_map_bounds
        box_size = max(0.1, float(self.get_parameter("geofence_box_size_m").value))

        corners = [
            (min_x, min_y),
            (min_x, max_y),
            (max_x, min_y),
            (max_x, max_y),
        ]
        anchor = min(corners, key=lambda c: math.hypot(base_pose_now.x - c[0], base_pose_now.y - c[1]))
        ax, ay = anchor

        if abs(ax - min_x) < 1e-9:
            gx_min = min_x
            gx_max = min(max_x, min_x + box_size)
        else:
            gx_max = max_x
            gx_min = max(min_x, max_x - box_size)

        if abs(ay - min_y) < 1e-9:
            gy_min = min_y
            gy_max = min(max_y, min_y + box_size)
        else:
            gy_max = max_y
            gy_min = max(min_y, max_y - box_size)

        self.geofence_anchor_corner = anchor
        return (gx_min, gx_max, gy_min, gy_max)

    def _compute_blank_geofence(self, anchor_pose: Pose2D) -> Optional[GeofenceRect]:
        if not bool(self.get_parameter("blank_geofence_enable").value):
            return None

        width_m = max(0.1, float(self.get_parameter("blank_geofence_width_m").value))
        height_m = max(0.1, float(self.get_parameter("blank_geofence_height_m").value))
        yaw = float(anchor_pose.yaw) if bool(self.get_parameter("blank_geofence_align_to_start_yaw").value) else 0.0
        return GeofenceRect(
            cx=float(anchor_pose.x),
            cy=float(anchor_pose.y),
            yaw=float(yaw),
            half_width=0.5 * width_m,
            half_height=0.5 * height_m,
        )

    def _ensure_geofence(self, base_pose_now: Pose2D) -> None:
        if self.active_geofence_rect is not None:
            return

        if bool(self.get_parameter("geofence_enable").value) and self.saved_map_bounds is not None:
            geofence = self._compute_corner_geofence(base_pose_now)
            if geofence is None:
                return
            self._activate_geofence_rect(
                self._rect_from_bounds(geofence),
                anchor_corner=self.geofence_anchor_corner,
            )
            gx_min, gx_max, gy_min, gy_max = geofence
            anchor = self.geofence_anchor_corner
            if anchor is not None:
                self.get_logger().info(
                    f"Geofence active from closest saved-map corner ({anchor[0]:.2f}, {anchor[1]:.2f}) -> "
                    f"x:[{gx_min:.2f},{gx_max:.2f}] y:[{gy_min:.2f},{gy_max:.2f}]"
                )
            return

        anchor_pose = self.blank_mode_anchor_pose or base_pose_now
        rect = self._compute_blank_geofence(anchor_pose)
        if rect is None:
            self._clear_geofence()
            return
        self._activate_geofence_rect(rect)
        self.get_logger().info(
            "Blank-map operating area locked to traj-local start pose "
            f"({rect.cx:.2f}, {rect.cy:.2f}, yaw={math.degrees(rect.yaw):.1f} deg) "
            f"size={2.0 * rect.half_width:.2f}x{2.0 * rect.half_height:.2f} m"
        )

    def _pose_inside_geofence(self, pose: Pose2D, margin_m: float = 0.0) -> bool:
        rect = self.active_geofence_rect
        if rect is None:
            return True
        local_x, local_y = self._world_to_rect(rect, pose.x, pose.y)
        return (
            abs(local_x) <= max(0.0, rect.half_width - margin_m)
            and abs(local_y) <= max(0.0, rect.half_height - margin_m)
        )

    def _clamp_waypoints_to_geofence(self, wps: List[Tuple[float, float]]) -> Tuple[List[Tuple[float, float]], bool]:
        rect = self.active_geofence_rect
        if rect is None or not wps:
            return wps, False

        out: List[Tuple[float, float]] = []
        changed = False
        for x, y in wps:
            local_x, local_y = self._world_to_rect(rect, float(x), float(y))
            clamped_x = min(rect.half_width, max(-rect.half_width, local_x))
            clamped_y = min(rect.half_height, max(-rect.half_height, local_y))
            if abs(clamped_x - local_x) > 1e-6 or abs(clamped_y - local_y) > 1e-6:
                changed = True
            out.append(self._rect_to_world(rect, clamped_x, clamped_y))
        return out, changed

    def _apply_geofence_to_costmap(self, costmap: np.ndarray, gs: GridSpec) -> None:
        rect = self.active_geofence_rect
        if rect is None:
            return

        xs = gs.origin_x + (np.arange(gs.width, dtype=np.float32) + 0.5) * gs.res
        ys = gs.origin_y + (np.arange(gs.height, dtype=np.float32) + 0.5) * gs.res
        xx, yy = np.meshgrid(xs, ys)
        c = math.cos(rect.yaw)
        s = math.sin(rect.yaw)
        local_x = c * (xx - rect.cx) + s * (yy - rect.cy)
        local_y = -s * (xx - rect.cx) + c * (yy - rect.cy)
        outside = (np.abs(local_x) > rect.half_width) | (np.abs(local_y) > rect.half_height)
        if np.any(outside):
            costmap[outside] = 100

    # =======================
    # Robot Visualization
    # =======================
    def _publish_robot_visualization(self, pose: Pose2D) -> None:
        """Publish robot footprint circle and orientation arrow."""
        frame = self.get_parameter("map_frame").value
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        now = self.get_clock().now().to_msg()

        markers = MarkerArray()

        # Circle marker (robot footprint)
        circle = Marker()
        circle.header.frame_id = frame
        circle.header.stamp = now
        circle.ns = "robot_footprint"
        circle.id = 0
        circle.type = Marker.CYLINDER
        circle.action = Marker.ADD
        circle.pose.position.x = pose.x
        circle.pose.position.y = pose.y
        circle.pose.position.z = 0.0
        circle.pose.orientation.w = 1.0
        circle.scale.x = wheelbase * 2.0  # diameter
        circle.scale.y = wheelbase * 2.0  # diameter
        circle.scale.z = 0.01  # thin disc
        circle.color.r = 0.0
        circle.color.g = 0.5
        circle.color.b = 1.0
        circle.color.a = 0.3  # semi-transparent
        markers.markers.append(circle)

        # Arrow marker (orientation)
        arrow = Marker()
        arrow.header.frame_id = frame
        arrow.header.stamp = now
        arrow.ns = "robot_orientation"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        
        # Arrow from center to edge of wheelbase circle
        start_point = Point()
        start_point.x = pose.x
        start_point.y = pose.y
        start_point.z = 0.05  # slightly above circle
        
        end_point = Point()
        end_point.x = pose.x + wheelbase * math.cos(pose.yaw)
        end_point.y = pose.y + wheelbase * math.sin(pose.yaw)
        end_point.z = 0.05
        
        arrow.points = [start_point, end_point]
        arrow.scale.x = 0.02  # shaft diameter
        arrow.scale.y = 0.04  # head diameter
        arrow.scale.z = 0.05  # head length
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        markers.markers.append(arrow)

        self._safe_publish(self.robot_viz_pub, markers, "robot_visualization")

    # =======================
    # TF / warnings
    # =======================
    def _warn_tf_throttled(self, msg: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_tf_warn_ns >= self._tf_warn_period_ns:
            self._last_tf_warn_ns = now_ns
            self.get_logger().warn(msg)

    def _warn_lidar_throttled(self, msg: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_lidar_warn_ns >= self._lidar_warn_period_ns:
            self._last_lidar_warn_ns = now_ns
            self.get_logger().warn(msg)

    def _finish_timer_cycle(self, timer_start_ns: int) -> None:
        elapsed_ns = self.get_clock().now().nanoseconds - timer_start_ns
        period_ns = int(1e9 / max(0.1, self._main_loop_hz))
        overload_ns = int(period_ns * self._timer_load_shed_ratio)

        if elapsed_ns > overload_ns:
            self._timer_overrun_streak += 1
            self._timer_recover_streak = 0
            if self._timer_overrun_streak >= 2 and self._effective_scan_stride < self._scan_stride_max:
                self._effective_scan_stride += 1
                self.get_logger().warn(
                    f"CPU overload detected ({elapsed_ns / 1e6:.1f} ms loop). "
                    f"Increasing LiDAR stride to {self._effective_scan_stride}."
                )
        else:
            self._timer_recover_streak += 1
            self._timer_overrun_streak = 0
            if self._timer_recover_streak >= 6 and self._effective_scan_stride > 1:
                self._effective_scan_stride -= 1
                self.get_logger().info(
                    f"CPU load recovered ({elapsed_ns / 1e6:.1f} ms loop). "
                    f"Reducing LiDAR stride to {self._effective_scan_stride}."
                )

    # =======================
    # Odom
    # =======================
    def on_odom(self, msg: Odometry) -> None:
        self._last_odom_rx_ns = self.get_clock().now().nanoseconds
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        yaw_offset = float(self.get_parameter("odom_yaw_offset_rad").value)
        yaw = wrap_to_pi(yaw + yaw_offset)

        pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.odom_pose_latest = pose

        # Odometry linear twist is expressed in the child/body frame; rotate to map frame.
        vx_body = float(msg.twist.twist.linear.x)
        vy_body = float(msg.twist.twist.linear.y)
        self.odom_vel_body_latest = (vx_body, vy_body)
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_map = c * vx_body - s * vy_body
        vy_map = s * vx_body + c * vy_body
        self.odom_vel_map_latest = (vx_map, vy_map)
        self.odom_wz_latest = float(msg.twist.twist.angular.z)

        self.have_odom_pose = True

        t_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if t_ns == 0:
            return

        self._odom_hist.append((t_ns, pose))

        hist_s = float(self.get_parameter("odom_history_s").value)
        keep_ns = int(max(0.2, hist_s) * 1e9)
        min_ns = t_ns - keep_ns
        while len(self._odom_hist) > 2 and self._odom_hist[0][0] < min_ns:
            self._odom_hist.popleft()

        # Publish odom->base_link immediately on fresh odometry so RViz/base pose
        # updates are not limited by the slower main-loop timer under CPU load.
        self._publish_odom_to_base_tf(stamp=self._stable_publish_stamp())

    def _odom_pose_at(self, t_ns: int) -> Optional[Pose2D]:
        if not self.have_odom_pose:
            return None

        if t_ns == 0 or len(self._odom_hist) < 2:
            return self.odom_pose_latest

        if t_ns <= self._odom_hist[0][0]:
            return self._odom_hist[0][1]
        if t_ns >= self._odom_hist[-1][0]:
            return self._odom_hist[-1][1]

        for i in range(len(self._odom_hist) - 1):
            t0, p0 = self._odom_hist[i]
            t1, p1 = self._odom_hist[i + 1]
            if t0 <= t_ns <= t1:
                dt = max(1, (t1 - t0))
                a = (t_ns - t0) / dt
                x = p0.x + a * (p1.x - p0.x)
                y = p0.y + a * (p1.y - p0.y)
                dyaw = wrap_to_pi(p1.yaw - p0.yaw)
                yaw = wrap_to_pi(p0.yaw + a * dyaw)
                return Pose2D(x=x, y=y, yaw=yaw)

        return self.odom_pose_latest

    def _predict_odom_pose_now(self, now_ns: int, max_dt_s: float = 0.25) -> Pose2D:
        if self._last_odom_rx_ns <= 0:
            return self.odom_pose_latest

        dt_s = max(0.0, (int(now_ns) - int(self._last_odom_rx_ns)) * 1e-9)
        dt_s = min(dt_s, max(0.0, float(max_dt_s)))
        if dt_s <= 0.0:
            return self.odom_pose_latest

        vx_map, vy_map = self.odom_vel_map_latest
        x = self.odom_pose_latest.x + vx_map * dt_s
        y = self.odom_pose_latest.y + vy_map * dt_s
        yaw = wrap_to_pi(self.odom_pose_latest.yaw + self.odom_wz_latest * dt_s)
        return Pose2D(x=x, y=y, yaw=yaw)

    def _base_pose_now(self) -> Pose2D:
        if self.mapping_active:
            if not self.have_odom_pose:
                if self._last_global_pose is not None:
                    self._warn_tf_throttled(
                        "Mapping active but STM odometry is unavailable; reusing last valid pose."
                    )
                    return self._last_global_pose
                self._warn_tf_throttled(
                    "Mapping active but STM odometry is unavailable; falling back to start_pose."
                )
                sp = self.get_parameter("start_pose").value
                return Pose2D(float(sp[0]), float(sp[1]), float(sp[2]))

            odom_stale_timeout_s = max(0.05, float(self.get_parameter("odom_stale_timeout_s").value))
            now_ns = self.get_clock().now().nanoseconds
            odom_is_fresh = (
                self._last_odom_rx_ns > 0
                and (now_ns - self._last_odom_rx_ns) <= int(odom_stale_timeout_s * 1e9)
            )

            if odom_is_fresh:
                odom_pose_for_mapping = self._predict_odom_pose_now(now_ns)
                pose_from_odom = self._odom_pose_from_stm_for_mapping(odom_pose_for_mapping)
                if pose_from_odom is not None:
                    self._last_global_pose = pose_from_odom
                    return pose_from_odom

            if self._last_global_pose is not None:
                self._warn_tf_throttled(
                    "Mapping active but STM odometry is stale; reusing last valid pose."
                )
                return self._last_global_pose

            self._warn_tf_throttled(
                "Mapping active but STM odometry is stale and no prior pose exists; using latest raw odom pose."
            )
            pose = self.odom_pose_latest
            self._last_global_pose = pose
            return pose

        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        base_frame = str(self.get_parameter("base_frame").value).lstrip("/")
        timeout_s = max(0.0, float(self.get_parameter("pose_tf_timeout_s").value))

        if bool(self.get_parameter("prefer_tf_pose").value):
            if map_frame and base_frame:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        map_frame,
                        base_frame,
                        Time(),
                        timeout=Duration(seconds=timeout_s),
                    )

                    tx = float(tf.transform.translation.x)
                    ty = float(tf.transform.translation.y)
                    qx = float(tf.transform.rotation.x)
                    qy = float(tf.transform.rotation.y)
                    qz = float(tf.transform.rotation.z)
                    qw = float(tf.transform.rotation.w)
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    pose = Pose2D(tx, ty, yaw)
                    self._last_global_pose = pose
                    return pose
                except Exception:
                    pass

        if self.have_odom_pose:
            pose_from_odom = self._odom_pose_in_map_frame(self.odom_pose_latest, timeout_s)
            if pose_from_odom is not None:
                self._last_global_pose = pose_from_odom
                return pose_from_odom

        if self._last_global_pose is not None:
            self._warn_tf_throttled(
                "Unable to resolve current pose in map frame; reusing last valid global pose."
            )
            return self._last_global_pose

        self._warn_tf_throttled(
            "Unable to resolve current pose in map frame; falling back to start_pose until TF is available."
        )
        sp = self.get_parameter("start_pose").value
        return Pose2D(float(sp[0]), float(sp[1]), float(sp[2]))

    def _odom_pose_in_map_frame(self, odom_pose: Pose2D, timeout_s: float) -> Optional[Pose2D]:
        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        odom_frame = str(self.get_parameter("odom_frame").value).lstrip("/")

        if not map_frame or not odom_frame:
            return None

        if map_frame == odom_frame:
            return odom_pose

        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                odom_frame,
                Time(),
                timeout=Duration(seconds=timeout_s),
            )
            return self._apply_transform_to_pose_2d(tf.transform, odom_pose)
        except Exception:
            return None

    def _odom_pose_at_in_map_frame(self, t_ns: int, timeout_s: float) -> Optional[Pose2D]:
        odom_pose = self._odom_pose_at(t_ns)
        if odom_pose is None:
            return None

        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        odom_frame = str(self.get_parameter("odom_frame").value).lstrip("/")

        if not map_frame or not odom_frame:
            return None
        if map_frame == odom_frame:
            return odom_pose

        try:
            stamp = Time(nanoseconds=int(t_ns)) if int(t_ns) > 0 else Time()
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                odom_frame,
                stamp,
                timeout=Duration(seconds=timeout_s),
            )
            return self._apply_transform_to_pose_2d(tf.transform, odom_pose)
        except Exception:
            # Fallback to latest transform if exact-time lookup is unavailable
            return self._odom_pose_in_map_frame(odom_pose, timeout_s)

    def _lookup_tf_pose_in_map_frame(self, t_ns: int, timeout_s: float) -> Optional[Pose2D]:
        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        base_frame = str(self.get_parameter("base_frame").value).lstrip("/")
        if not map_frame or not base_frame:
            return None

        try:
            stamp = Time(nanoseconds=int(t_ns)) if int(t_ns) > 0 else Time()
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                stamp,
                timeout=Duration(seconds=timeout_s),
            )
        except Exception:
            if int(t_ns) > 0:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        map_frame,
                        base_frame,
                        Time(),
                        timeout=Duration(seconds=timeout_s),
                    )
                except Exception:
                    return None
            else:
                return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        yaw = self._yaw_from_transform_quaternion(tf.transform)
        return Pose2D(tx, ty, yaw)

    @staticmethod
    def _yaw_from_transform_quaternion(transform: object) -> float:
        qx = float(transform.rotation.x)
        qy = float(transform.rotation.y)
        qz = float(transform.rotation.z)
        qw = float(transform.rotation.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _apply_transform_to_pose_2d(self, transform: object, pose: Pose2D) -> Pose2D:
        x, y = self._apply_transform_2d(transform, pose.x, pose.y)
        yaw_offset = self._yaw_from_transform_quaternion(transform)
        return Pose2D(x, y, wrap_to_pi(pose.yaw + yaw_offset))

    def _capture_mapping_pose_anchor(self, timeout_s: float = 0.0, warn_on_failure: bool = False) -> bool:
        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        odom_frame = str(self.get_parameter("odom_frame").value).lstrip("/")

        if not map_frame or not odom_frame or map_frame == odom_frame:
            self._mapping_pose_anchor_tf = None
            return True

        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                odom_frame,
                Time(),
                timeout=Duration(seconds=max(0.0, float(timeout_s))),
            )
            self._mapping_pose_anchor_tf = tf.transform
            self.get_logger().info("Captured fixed map<-odom pose anchor for mapping mode")
            return True
        except Exception as exc:
            if warn_on_failure:
                self._warn_tf_throttled(
                    f"Mapping mode could not capture map<-odom anchor yet; temporarily using raw odom pose. ({exc})"
                )
            return False

    def _odom_pose_from_stm_for_mapping(self, odom_pose: Pose2D) -> Optional[Pose2D]:
        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        odom_frame = str(self.get_parameter("odom_frame").value).lstrip("/")

        if not map_frame or not odom_frame:
            return odom_pose
        if map_frame == odom_frame:
            return odom_pose

        if self._mapping_pose_anchor_tf is None and not self._capture_mapping_pose_anchor(timeout_s=0.0):
            return None
        if self._mapping_pose_anchor_tf is None:
            return odom_pose

        return self._apply_transform_to_pose_2d(self._mapping_pose_anchor_tf, odom_pose)

    # =======================
    # TF publishing (odom->base_link)
    # =======================
    def _publish_odom_to_base_tf(self, stamp=None) -> None:
        if not bool(self.get_parameter("publish_odom_to_base_tf").value):
            return

        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value

        if self.have_odom_pose:
            pose_for_tf = self.odom_pose_latest
        else:
            sp = self.get_parameter("start_pose").value
            pose_for_tf = Pose2D(float(sp[0]), float(sp[1]), float(sp[2]))

        now = stamp if stamp is not None else self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = float(pose_for_tf.x)
        t.transform.translation.y = float(pose_for_tf.y)
        t.transform.translation.z = 0.0
        q = yaw_to_quat(pose_for_tf.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    # =======================
    # Scan capture
    # =======================
    @staticmethod
    def _scan_stamp_ns(scan: LaserScan) -> int:
        sec = int(scan.header.stamp.sec)
        nsec = int(scan.header.stamp.nanosec)
        return sec * int(1e9) + nsec

    @staticmethod
    def _sample_time_ns(sample: ScanSample, use_arrival_stamp: bool) -> int:
        if use_arrival_stamp:
            return sample.arrival_stamp_ns
        if sample.source_stamp_ns > 0:
            return sample.source_stamp_ns
        return sample.arrival_stamp_ns

    def _push_scan_sample(self, buffer: Deque[ScanSample], msg: LaserScan) -> None:
        now_ns = self.get_clock().now().nanoseconds
        sample = ScanSample(
            scan=msg,
            source_stamp_ns=self._scan_stamp_ns(msg),
            arrival_stamp_ns=now_ns,
        )
        buffer.append(sample)

        queue_size = max(2, int(self.get_parameter("scan_sync_queue_size").value))
        while len(buffer) > queue_size:
            buffer.popleft()

        self._last_scan_rx_ns = now_ns

    def _prune_stale_scan_samples(
        self,
        buffer: Deque[ScanSample],
        now_ns: int,
        max_age_ns: int,
        use_arrival_stamp: bool,
    ) -> None:
        if max_age_ns <= 0:
            return
        while buffer:
            t_ns = self._sample_time_ns(buffer[0], use_arrival_stamp)
            if now_ns - t_ns <= max_age_ns:
                break
            buffer.popleft()

    def _select_fusion_scans(self, now_t: Time) -> List[Tuple[LaserScan, int]]:
        max_age_s = max(0.0, float(self.get_parameter("scan_max_age_s").value))
        if self.mapping_active:
            max_age_s = max(max_age_s, 1.0)
        max_age_ns = int(max_age_s * 1e9)
        max_scan_pair_skew_s = max(0.0, float(self.get_parameter("max_scan_pair_skew_s").value))
        max_scan_pair_skew_ns = int(max_scan_pair_skew_s * 1e9)
        use_arrival_stamp = bool(self.get_parameter("use_arrival_stamp_for_scan_timing").value)

        now_ns = now_t.nanoseconds
        self._prune_stale_scan_samples(self._scan1_buffer, now_ns, max_age_ns, use_arrival_stamp)
        self._prune_stale_scan_samples(self._scan2_buffer, now_ns, max_age_ns, use_arrival_stamp)

        newest_1 = self._scan1_buffer[-1] if self._scan1_buffer else None
        newest_2 = self._scan2_buffer[-1] if self._scan2_buffer else None

        if newest_1 is None and newest_2 is None:
            return []
        if newest_1 is None:
            t2 = self._sample_time_ns(newest_2, use_arrival_stamp)
            return [(newest_2.scan, t2)]
        if newest_2 is None:
            t1 = self._sample_time_ns(newest_1, use_arrival_stamp)
            return [(newest_1.scan, t1)]

        best_pair: Optional[Tuple[ScanSample, int, ScanSample, int, int]] = None
        for sample_1 in self._scan1_buffer:
            t1 = self._sample_time_ns(sample_1, use_arrival_stamp)
            for sample_2 in self._scan2_buffer:
                t2 = self._sample_time_ns(sample_2, use_arrival_stamp)
                skew_ns = abs(t1 - t2)
                if best_pair is None or skew_ns < best_pair[4]:
                    best_pair = (sample_1, t1, sample_2, t2, skew_ns)

        if best_pair is None:
            t1 = self._sample_time_ns(newest_1, use_arrival_stamp)
            t2 = self._sample_time_ns(newest_2, use_arrival_stamp)
            return [(newest_1.scan, t1), (newest_2.scan, t2)]

        sample_1, t1, sample_2, t2, pair_skew_ns = best_pair
        pair_skew_s = pair_skew_ns * 1e-9
        if max_scan_pair_skew_ns > 0 and pair_skew_ns > max_scan_pair_skew_ns:
            self._warn_lidar_throttled(
                "LiDAR streams out of sync; using nearest-time dual-scan fusion "
                f"({pair_skew_s:.3f}s skew > {max_scan_pair_skew_s:.3f}s)"
            )

        return [(sample_1.scan, t1), (sample_2.scan, t2)]

    def _stable_publish_stamp(self) -> object:
        now_ns = self.get_clock().now().nanoseconds
        backdate_s = max(0.0, float(self.get_parameter("tf_timestamp_backdate_s").value))
        backdate_ns = int(backdate_s * 1e9)
        stamp_ns = max(0, now_ns - backdate_ns)

        if stamp_ns <= self._last_stable_pub_stamp_ns:
            stamp_ns = self._last_stable_pub_stamp_ns + 1
        self._last_stable_pub_stamp_ns = stamp_ns
        return Time(nanoseconds=int(stamp_ns)).to_msg()

    def on_scan1(self, msg: LaserScan) -> None:
        expected_frame = str(self.get_parameter("lidar1_frame_id").value).lstrip("/")
        frame = (msg.header.frame_id or "").lstrip("/")
        if not frame:
            self._warn_lidar_throttled("LiDAR1 scan received with empty frame_id; dropping sample")
            return
        if expected_frame and frame != expected_frame:
            self._warn_lidar_throttled(
                f"LiDAR1 frame_id mismatch: received '{frame}', expected '{expected_frame}'; dropping sample"
            )
            return
        self._push_scan_sample(self._scan1_buffer, msg)

    def on_scan2(self, msg: LaserScan) -> None:
        expected_frame = str(self.get_parameter("lidar2_frame_id").value).lstrip("/")
        frame = (msg.header.frame_id or "").lstrip("/")
        if not frame:
            self._warn_lidar_throttled("LiDAR2 scan received with empty frame_id; dropping sample")
            return
        if expected_frame and frame != expected_frame:
            self._warn_lidar_throttled(
                f"LiDAR2 frame_id mismatch: received '{frame}', expected '{expected_frame}'; dropping sample"
            )
            return
        self._push_scan_sample(self._scan2_buffer, msg)

    # =======================
    # Transform helpers
    # =======================
    def _lookup_base_from_scan(self, scan_frame: str) -> Optional[object]:
        base_frame = self.get_parameter("base_frame").value
        sf = (scan_frame or "").lstrip("/")

        if sf == "" or base_frame == "":
            return None

        if sf in self._T_base_from_scan:
            return self._T_base_from_scan[sf]

        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame,
                sf,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            self._T_base_from_scan[sf] = tf.transform
            self.get_logger().info(f"Cached TF {base_frame} <- {sf} : "
                                   f"t=({tf.transform.translation.x:.3f}, {tf.transform.translation.y:.3f})")
            return tf.transform
        except Exception as e:
            self._warn_tf_throttled(f"No TF {base_frame} <- {sf}. Check static transforms. ({e})")
            return None

    def _apply_transform_2d(self, tr: object, x: float, y: float) -> Tuple[float, float]:
        """
        Apply a 2D transform (rotation + translation) to a point.
        This extracts only the XY components from the 3D transform.
        """
        # Get translation - these should be the XY offset of the LIDAR from base_link
        tx = float(tr.translation.x)
        ty = float(tr.translation.y)
        tz = float(tr.translation.z)  # Extract but don't use in 2D
        
        qx = float(tr.rotation.x)
        qy = float(tr.rotation.y)
        qz = float(tr.rotation.z)
        qw = float(tr.rotation.w)

        # Extract yaw from quaternion (rotation around Z axis)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Rotate point by yaw, then translate by (tx, ty)
        c = math.cos(yaw)
        s = math.sin(yaw)
        xr = c * x - s * y
        yr = s * x + c * y
        return (xr + tx, yr + ty)

    @staticmethod
    def _se2_apply(p: Pose2D, x: float, y: float) -> Tuple[float, float]:
        c = math.cos(p.yaw)
        s = math.sin(p.yaw)
        return (p.x + c * x - s * y, p.y + s * x + c * y)

    @staticmethod
    def _se2_apply_inv(p: Pose2D, x: float, y: float) -> Tuple[float, float]:
        dx = x - p.x
        dy = y - p.y
        c = math.cos(p.yaw)
        s = math.sin(p.yaw)
        # rotate by -yaw
        xb = c * dx + s * dy
        yb = -s * dx + c * dy
        return (xb, yb)

    @staticmethod
    def _map_velocity_to_body(vx_map: float, vy_map: float, yaw: float) -> Tuple[float, float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_body = c * vx_map + s * vy_map
        vy_body = -s * vx_map + c * vy_map
        return vx_body, vy_body

    # =======================
    # Build fused scan in base_link
    # =======================
    def _points_from_scan_in_base(self, scan: LaserScan) -> List[Tuple[float, float]]:
        tr = self._lookup_base_from_scan(scan.header.frame_id)
        if tr is None:
            return []

        rmin = float(scan.range_min)
        rmax = float(scan.range_max)
        no_hit_eps = float(self.get_parameter("scan_no_hit_eps_m").value)
        base_stride = max(1, int(self.get_parameter("scan_beam_stride").value))
        stride = max(base_stride, self._effective_scan_stride)
        beam_budget = max(64, int(self._scan_beam_budget_per_scan))
        max_range = float(self.get_parameter("max_lidar_range_m").value)
        
        # Clamp rmax to max_range to reduce processing
        rmax = min(rmax, max_range)

        pts: List[Tuple[float, float]] = []
        ang0 = float(scan.angle_min)
        inc = float(scan.angle_increment)

        processed = 0
        ranges = scan.ranges
        sensor_y_axis_right = bool(self.get_parameter("scan_sensor_y_axis_right").value)
        for i in range(0, len(ranges), stride):
            if processed >= beam_budget:
                break
            processed += 1

            ang = ang0 + i * inc

            r = float(ranges[i])
            if (not math.isfinite(r)) or r < rmin or r >= (rmax - no_hit_eps) or r > max_range:
                continue

            # Convert scan frame handedness only when configured by hardware.
            # Most ROS scans are x-forward/y-left and do not need this flip.
            xs = r * math.cos(ang)
            ys = -r * math.sin(ang) if sensor_y_axis_right else r * math.sin(ang)
            xb, yb = self._apply_transform_2d(tr, xs, ys)
            pts.append((xb, yb))

        return pts

    @staticmethod
    def _voxel_merge_points_nearest(points: List[Tuple[float, float]], resolution_m: float) -> List[Tuple[float, float]]:
        if resolution_m <= 1e-6 or len(points) < 2:
            return points

        merged: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
        inv_res = 1.0 / resolution_m

        for x, y in points:
            key = (int(math.floor(x * inv_res)), int(math.floor(y * inv_res)))
            rr = x * x + y * y
            cur = merged.get(key)
            if cur is None or rr < cur[0]:
                merged[key] = (rr, x, y)

        return [(x, y) for (_rr, x, y) in merged.values()]

    def _build_scan_from_points(
        self,
        points_base: List[Tuple[float, float]],
        stamp_msg,
        frame_id: str,
        angle_increment_deg: float,
    ) -> Optional[LaserScan]:
        a_min = float(self.get_parameter("fused_angle_min").value)
        a_max = float(self.get_parameter("fused_angle_max").value)
        inc = float(angle_increment_deg) * math.pi / 180.0
        if inc <= 1e-6:
            inc = math.radians(1.0)

        n = int(math.floor((a_max - a_min) / inc))
        if n < 10:
            return None

        range_min = 0.0
        range_max = float(self.get_parameter("max_lidar_range_m").value)
        ranges = [math.inf] * n

        for x_final, y_final in points_base:
            rr_fused = math.hypot(x_final, y_final)
            if rr_fused < range_min or rr_fused > range_max:
                continue

            aa_fused = math.atan2(y_final, x_final)
            if aa_fused < a_min or aa_fused >= a_max:
                continue

            k = int((aa_fused - a_min) / inc)
            if 0 <= k < n and rr_fused < ranges[k]:
                ranges[k] = rr_fused

        msg = LaserScan()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = frame_id
        msg.angle_min = a_min
        msg.angle_max = a_max
        msg.angle_increment = inc
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = range_min
        msg.range_max = range_max
        msg.ranges = [r if math.isfinite(r) else math.inf for r in ranges]
        msg.intensities = []
        return msg

    def _build_scan_from_scan(self, source_scan: LaserScan, target_angle_increment_deg: float) -> Optional[LaserScan]:
        target_inc = float(target_angle_increment_deg) * math.pi / 180.0
        src_inc = float(source_scan.angle_increment)
        if target_inc <= 1e-6 or src_inc <= 1e-9:
            return None

        bin_width = max(1, int(round(target_inc / src_inc)))
        src_ranges = list(source_scan.ranges)
        if not src_ranges:
            return None

        n_target = max(1, len(src_ranges) // bin_width)
        ranges: List[float] = [math.inf] * n_target

        for i in range(n_target):
            start = i * bin_width
            end = min(len(src_ranges), start + bin_width)
            best = math.inf
            for r in src_ranges[start:end]:
                rr = float(r)
                if math.isfinite(rr) and rr < best:
                    best = rr
            ranges[i] = best

        msg = LaserScan()
        msg.header = source_scan.header
        msg.angle_min = float(source_scan.angle_min)
        msg.angle_increment = target_inc
        msg.angle_max = msg.angle_min + target_inc * float(max(0, n_target - 1))
        msg.time_increment = 0.0
        msg.scan_time = float(source_scan.scan_time)
        msg.range_min = float(source_scan.range_min)
        msg.range_max = float(source_scan.range_max)
        msg.ranges = [r if math.isfinite(r) else math.inf for r in ranges]
        msg.intensities = []
        return msg

    def _build_fused_scan(self, base_pose_now: Pose2D, stamp_msg) -> Optional[LaserScan]:
        self._latest_fused_points_base = []
        self._latest_scan_pairs = []
        if not bool(self.get_parameter("publish_fused_scan").value):
            return None

        now_t = self.get_clock().now()

        scan_pairs = self._select_fusion_scans(now_t)
        if not scan_pairs:
            return None
        self._latest_scan_pairs = list(scan_pairs)

        scans = [s for (s, _stamp_ns) in scan_pairs]

        use_excl = bool(self.get_parameter("robot_exclusion_enable").value)
        excl_r = float(self.get_parameter("robot_exclusion_radius_m").value)
        excl_r2 = excl_r * excl_r

        motion_comp = bool(self.get_parameter("motion_compensate").value)
        max_range = float(self.get_parameter("max_lidar_range_m").value)
        voxel_merge_res = max(0.0, float(self.get_parameter("fused_voxel_merge_resolution_m").value))

        # Motion compensation must use scan-time pose and current pose from the same frame.
        # Odom history (_odom_pose_at) is in odom frame, so de-warp in odom as well.
        if motion_comp and self.have_odom_pose:
            base_pose_now_mc = self.odom_pose_latest
        else:
            base_pose_now_mc = base_pose_now

        if motion_comp and not self.have_odom_pose:
            self._warn_tf_throttled(
                "motion_compensate enabled but no odom history available; using uncompensated fused scan"
            )
            motion_comp = False

        # Process each scan and fuse
        base_stride = max(1, int(self.get_parameter("scan_beam_stride").value))
        stride = max(base_stride, self._effective_scan_stride)
        no_hit_eps = float(self.get_parameter("scan_no_hit_eps_m").value)
        beam_budget = max(64, int(self._scan_beam_budget_per_scan))
        fused_points: List[Tuple[float, float]] = []

        for s, stamp_ns in scan_pairs:
            
            # Get robot pose at scan time for motion compensation
            if motion_comp:
                base_pose_scan = self._odom_pose_at(stamp_ns) or base_pose_now_mc
            else:
                base_pose_scan = base_pose_now

            # Get transform from sensor frame to base_link
            tr = self._lookup_base_from_scan(s.header.frame_id)
            if tr is None:
                continue

            # Process each beam in the scan
            rmin = float(s.range_min)
            rmax = float(s.range_max)
            beam_angle0 = float(s.angle_min)
            beam_inc = float(s.angle_increment)

            processed = 0
            sranges = s.ranges
            sensor_y_axis_right = bool(self.get_parameter("scan_sensor_y_axis_right").value)
            for i in range(0, len(sranges), stride):
                if processed >= beam_budget:
                    break
                processed += 1

                beam_angle = beam_angle0 + i * beam_inc

                r = float(sranges[i])
                if (not math.isfinite(r)) or r < rmin or r >= (rmax - no_hit_eps) or r > max_range:
                    continue

                # Convert scan frame handedness only when configured by hardware.
                xs = r * math.cos(beam_angle)
                ys = -r * math.sin(beam_angle) if sensor_y_axis_right else r * math.sin(beam_angle)
                
                # Transform from sensor frame to base_link frame (rotation + translation)
                x_origin, y_origin = self._apply_transform_2d(tr, xs, ys)

                # Apply motion compensation if enabled
                if motion_comp:
                    # Transform point to world frame using scan pose
                    xm, ym = self._se2_apply(base_pose_scan, x_origin, y_origin)
                    # Transform back using current pose (de-warp)
                    x_final, y_final = self._se2_apply_inv(base_pose_now_mc, xm, ym)
                else:
                    x_final = x_origin
                    y_final = y_origin

                # Check robot exclusion
                if use_excl and (x_final * x_final + y_final * y_final) < excl_r2:
                    continue

                fused_points.append((x_final, y_final))

        if fused_points:
            fused_points = self._voxel_merge_points_nearest(fused_points, voxel_merge_res)
        self._latest_fused_points_base = list(fused_points)
        return self._build_scan_from_points(
            fused_points,
            stamp_msg,
            str(self.get_parameter("base_frame").value),
            float(self.get_parameter("fused_angle_increment_deg").value),
        )

    def _build_points_cloud_msg(
        self,
        points_base: List[Tuple[float, float]],
        stamp_msg,
        frame_id: str,
    ) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points_base)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        payload = bytearray(msg.row_step)
        for idx, (x_pt, y_pt) in enumerate(points_base):
            struct.pack_into("<fff", payload, idx * msg.point_step, float(x_pt), float(y_pt), 0.0)
        msg.data = bytes(payload)
        return msg

    # =======================
    # Costmap building
    # =======================
    def _inflate(self, grid: np.ndarray, gs: GridSpec, hard_r: float, soft_r: float) -> None:
        """
        Inflate obstacles in grid using NumPy operations for performance.
        Now we iterate out to max(hard_r, soft_r), so hard inflation works even if soft=0.
        """
        hard_r = max(0.0, hard_r)
        soft_r = max(0.0, soft_r)

        rad = max(hard_r, soft_r)
        if rad <= 1e-6:
            return

        rad_cells = int(math.ceil(rad / gs.res))
        if rad_cells <= 0:
            return

        # Find all hard obstacle cells using NumPy (vectorized)
        hard_cells_mask = grid >= 100
        hard_ys, hard_xs = np.where(hard_cells_mask)
        
        if len(hard_xs) == 0:
            return

        # For each obstacle cell, inflate around it
        for hx, hy in zip(hard_xs, hard_ys):
            y_min = max(0, hy - rad_cells)
            y_max = min(gs.height, hy + rad_cells + 1)
            x_min = max(0, hx - rad_cells)
            x_max = min(gs.width, hx + rad_cells + 1)
            
            for iy in range(y_min, y_max):
                for ix in range(x_min, x_max):
                    dx = ix - hx
                    dy = iy - hy
                    d = math.hypot(dx, dy) * gs.res
                    
                    if hard_r > 0 and d <= hard_r:
                        grid[iy, ix] = 100
                    elif soft_r > 0 and d <= soft_r and grid[iy, ix] < 100:
                        grid[iy, ix] = max(grid[iy, ix], 50)

    def _clear_robot_circle_in_costmap(self, grid: np.ndarray, gs: GridSpec, base_pose_now: Pose2D) -> None:
        if not bool(self.get_parameter("robot_exclusion_enable").value):
            return
        r = float(self.get_parameter("robot_exclusion_radius_m").value)
        if r <= 1e-6:
            return

        rad_cells = int(math.ceil(r / gs.res))
        cx_i, cy_i = self.world_to_grid(gs, base_pose_now.x, base_pose_now.y)
        r2 = r * r

        y_min = max(0, cy_i - rad_cells)
        y_max = min(gs.height, cy_i + rad_cells + 1)
        x_min = max(0, cx_i - rad_cells)
        x_max = min(gs.width, cx_i + rad_cells + 1)

        for iy in range(y_min, y_max):
            for ix in range(x_min, x_max):
                wx, wy = self.grid_to_world(gs, ix, iy)
                if (wx - base_pose_now.x) ** 2 + (wy - base_pose_now.y) ** 2 <= r2:
                    grid[iy, ix] = 0

    def _build_costmap_from_fused_scan(self, fused: LaserScan, base_pose_now: Pose2D) -> np.ndarray:
        dynamic = np.zeros((self.gs_map.height, self.gs_map.width), dtype=np.int8)

        a = float(fused.angle_min)
        inc = float(fused.angle_increment)

        for r in fused.ranges:
            rr = float(r)
            if not math.isfinite(rr):
                a += inc
                continue

            xb = rr * math.cos(a)
            yb = rr * math.sin(a)

            xm, ym = self._se2_apply(base_pose_now, xb, yb)

            ix, iy = self.world_to_grid(self.gs_map, xm, ym)
            if self.in_bounds(self.gs_map, ix, iy):
                dynamic[iy, ix] = 100

            a += inc

        # Costmap is intentionally live-only (current fused LiDAR). Saved/static map
        # data is reserved for localization/geofence definition, not obstacle fusion.
        if self.mapping_active:
            self._update_mapping_logodds_from_fused_scan(fused, base_pose_now)

        combined = dynamic

        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        self._inflate(combined, self.gs_map, hard_r=hard_r, soft_r=soft_r)

        self._clear_robot_circle_in_costmap(combined, self.gs_map, base_pose_now)
        return combined

    def _build_costmap_from_static_map(self, base_pose_now: Pose2D) -> np.ndarray:
        combined = self.static_occ.copy()

        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        self._inflate(combined, self.gs_map, hard_r=hard_r, soft_r=soft_r)

        self._clear_robot_circle_in_costmap(combined, self.gs_map, base_pose_now)
        return combined

    def _build_local_costmap_from_fused_scan(
        self,
        fused: LaserScan,
        base_pose_now: Pose2D,
        local_gs: GridSpec,
    ) -> np.ndarray:
        local = np.zeros((local_gs.height, local_gs.width), dtype=np.int8)

        a = float(fused.angle_min)
        inc = float(fused.angle_increment)

        for r in fused.ranges:
            rr = float(r)
            if not math.isfinite(rr):
                a += inc
                continue

            xb = rr * math.cos(a)
            yb = rr * math.sin(a)
            xm, ym = self._se2_apply(base_pose_now, xb, yb)

            ix, iy = self.world_to_grid(local_gs, xm, ym)
            if self.in_bounds(local_gs, ix, iy):
                local[iy, ix] = 100

            a += inc

        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        self._inflate(local, local_gs, hard_r=hard_r, soft_r=soft_r)
        self._clear_robot_circle_in_costmap(local, local_gs, base_pose_now)
        return local

    def _build_local_costmap_from_points(
        self,
        points_base: List[Tuple[float, float]],
        base_pose_now: Pose2D,
        local_gs: GridSpec,
    ) -> np.ndarray:
        local = np.zeros((local_gs.height, local_gs.width), dtype=np.int8)

        for xb, yb in points_base:
            xm, ym = self._se2_apply(base_pose_now, xb, yb)
            ix, iy = self.world_to_grid(local_gs, xm, ym)
            if self.in_bounds(local_gs, ix, iy):
                local[iy, ix] = 100

        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        self._inflate(local, local_gs, hard_r=hard_r, soft_r=soft_r)
        self._clear_robot_circle_in_costmap(local, local_gs, base_pose_now)
        return local

    def _build_planning_costmap_from_local(
        self,
        local_costmap: np.ndarray,
        local_gs: GridSpec,
    ) -> np.ndarray:
        """
        Build a global-size planning grid where dynamic obstacles are only stamped
        from the local window. This keeps long-range planning area while limiting
        live obstacle designation/inflation to the local vicinity.
        """
        planning = self.static_occ.copy()

        use_hysteresis = bool(self.get_parameter("planning_obstacle_hysteresis_enable").value)
        if not use_hysteresis:
            occ_ys, occ_xs = np.where(local_costmap > 0)
            if len(occ_xs) == 0:
                return planning

            for ix_l, iy_l in zip(occ_xs, occ_ys):
                wx, wy = self.grid_to_world(local_gs, int(ix_l), int(iy_l))
                ix_g, iy_g = self.world_to_grid(self.gs_map, wx, wy)
                if self.in_bounds(self.gs_map, ix_g, iy_g):
                    planning[iy_g, ix_g] = max(planning[iy_g, ix_g], local_costmap[iy_l, ix_l])
            return planning

        confirm_scans = max(1, int(self.get_parameter("planning_hard_obstacle_confirm_scans").value))
        clear_scans = max(1, int(self.get_parameter("planning_hard_obstacle_clear_scans").value))
        evidence_cap = max(confirm_scans, clear_scans) + 4

        for iy_l in range(local_gs.height):
            for ix_l in range(local_gs.width):
                wx, wy = self.grid_to_world(local_gs, ix_l, iy_l)
                ix_g, iy_g = self.world_to_grid(self.gs_map, wx, wy)
                if not self.in_bounds(self.gs_map, ix_g, iy_g):
                    continue

                cell_val = int(local_costmap[iy_l, ix_l])
                evidence = int(self._planning_hard_evidence[iy_g, ix_g])
                if cell_val >= 100:
                    evidence = min(evidence_cap, evidence + 1)
                else:
                    evidence = max(-evidence_cap, evidence - 1)
                self._planning_hard_evidence[iy_g, ix_g] = evidence

                if 0 < cell_val < 100 and planning[iy_g, ix_g] < 100:
                    planning[iy_g, ix_g] = max(planning[iy_g, ix_g], cell_val)

                if evidence >= confirm_scans:
                    planning[iy_g, ix_g] = 100
                elif cell_val >= 100 and planning[iy_g, ix_g] < 100:
                    # Treat new hard hits as soft hints until they persist.
                    planning[iy_g, ix_g] = max(planning[iy_g, ix_g], 50)

        return planning

    def _reset_mapping_buffers(self) -> None:
        self.static_occ.fill(0)
        self.persistent_evidence.fill(0)
        self._planning_hard_evidence.fill(0)
        if self.mapping_logodds is not None:
            self.mapping_logodds.fill(0)
        if self._mapping_occ_state is not None:
            self._mapping_occ_state.fill(0)

    def _init_mapping_grid(self, center_pose: Pose2D) -> None:
        area_m = max(1.0, float(self.get_parameter("mapping_area_size_m").value))
        res_m = max(0.01, float(self.get_parameter("mapping_res_m").value))
        width = int(round(area_m / res_m))
        height = int(round(area_m / res_m))

        origin_x = float(center_pose.x) - 0.5 * width * res_m
        origin_y = float(center_pose.y) - 0.5 * height * res_m

        self.mapping_grid_spec = GridSpec(
            res=res_m,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        self.mapping_logodds = np.zeros((height, width), dtype=np.int16)
        self._mapping_occ_state = np.zeros((height, width), dtype=np.uint8)

    def _mapping_world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        if self.mapping_grid_spec is None:
            return (-1, -1)
        gs = self.mapping_grid_spec
        ix = int((x - gs.origin_x) / gs.res)
        iy = int((y - gs.origin_y) / gs.res)
        return ix, iy

    def _mapping_in_bounds(self, ix: int, iy: int) -> bool:
        if self.mapping_grid_spec is None:
            return False
        gs = self.mapping_grid_spec
        return (0 <= ix < gs.width) and (0 <= iy < gs.height)

    def _mapping_update_cell(self, iy: int, ix: int, delta: int, lo_min: int, lo_max: int) -> None:
        if self.mapping_logodds is None:
            return
        cur = int(self.mapping_logodds[iy, ix])
        cur = max(lo_min, min(lo_max, cur + delta))
        self.mapping_logodds[iy, ix] = cur

    def _update_mapping_logodds_from_fused_scan(self, fused: LaserScan, base_pose_now: Pose2D) -> None:
        if self.mapping_grid_spec is None or self.mapping_logodds is None:
            return

        lo_hit = max(1, int(self.get_parameter("mapping_hit_logodd_inc").value))
        lo_free = max(1, int(self.get_parameter("mapping_free_logodd_dec").value))
        lo_min = int(self.get_parameter("mapping_logodd_min").value)
        lo_max = int(self.get_parameter("mapping_logodd_max").value)
        no_return_free_ratio = min(1.0, max(0.0, float(self.get_parameter("mapping_no_return_free_ratio").value)))
        no_hit_eps = float(self.get_parameter("scan_no_hit_eps_m").value)
        max_range = float(self.get_parameter("max_lidar_range_m").value)
        rmin = float(fused.range_min)
        use_excl = bool(self.get_parameter("robot_exclusion_enable").value)
        excl_r = float(self.get_parameter("robot_exclusion_radius_m").value)
        excl_r2 = excl_r * excl_r
        range_max = min(float(fused.range_max), max_range)
        step_m = max(self.mapping_grid_spec.res * 0.8, 0.02)

        a = float(fused.angle_min)
        inc = float(fused.angle_increment)

        for r in fused.ranges:
            rr = float(r)
            finite_hit = (
                math.isfinite(rr)
                and rr >= rmin
                and rr < (float(fused.range_max) - no_hit_eps)
                and rr <= max_range
            )
            if finite_hit:
                free_to = max(0.0, rr - 0.5 * self.mapping_grid_spec.res)
            else:
                free_to = max(0.0, range_max * no_return_free_ratio)

            if free_to > 1e-6:
                n_steps = max(1, int(math.floor(free_to / step_m)))
                for j in range(1, n_steps + 1):
                    d = min(free_to, j * step_m)
                    xb = d * math.cos(a)
                    yb = d * math.sin(a)
                    if use_excl and (xb * xb + yb * yb) <= excl_r2:
                        continue
                    xm, ym = self._se2_apply(base_pose_now, xb, yb)
                    ix, iy = self._mapping_world_to_grid(xm, ym)
                    if not self._mapping_in_bounds(ix, iy):
                        break
                    self._mapping_update_cell(iy, ix, -lo_free, lo_min, lo_max)

            if finite_hit:
                xb = rr * math.cos(a)
                yb = rr * math.sin(a)
                if use_excl and (xb * xb + yb * yb) <= excl_r2:
                    a += inc
                    continue
                xm, ym = self._se2_apply(base_pose_now, xb, yb)
                ix, iy = self._mapping_world_to_grid(xm, ym)
                if self._mapping_in_bounds(ix, iy):
                    self._mapping_update_cell(iy, ix, lo_hit, lo_min, lo_max)

            a += inc

    def _update_mapping_logodds_from_scan_pairs(self, scan_pairs: List[Tuple[LaserScan, int]], base_pose_now: Pose2D) -> None:
        if self.mapping_grid_spec is None or self.mapping_logodds is None:
            return
        if not scan_pairs:
            return

        lo_hit = max(1, int(self.get_parameter("mapping_hit_logodd_inc").value))
        lo_free = max(1, int(self.get_parameter("mapping_free_logodd_dec").value))
        lo_min = int(self.get_parameter("mapping_logodd_min").value)
        lo_max = int(self.get_parameter("mapping_logodd_max").value)
        no_return_free_ratio = min(1.0, max(0.0, float(self.get_parameter("mapping_no_return_free_ratio").value)))
        use_excl = bool(self.get_parameter("robot_exclusion_enable").value)
        excl_r = float(self.get_parameter("robot_exclusion_radius_m").value)
        excl_r2 = excl_r * excl_r
        step_m = max(self.mapping_grid_spec.res * 0.8, 0.02)
        max_range = float(self.get_parameter("max_lidar_range_m").value)
        no_hit_eps = float(self.get_parameter("scan_no_hit_eps_m").value)
        timeout_s = max(0.0, float(self.get_parameter("pose_tf_timeout_s").value))
        odom_stale_timeout_s = max(0.05, float(self.get_parameter("odom_stale_timeout_s").value))
        map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        odom_frame = str(self.get_parameter("odom_frame").value).lstrip("/")
        base_stride = max(1, int(self.get_parameter("scan_beam_stride").value))
        stride = max(base_stride, int(self._effective_scan_stride))
        beam_budget = max(64, int(self._scan_beam_budget_per_scan))

        for scan_msg, stamp_ns in scan_pairs:
            tr = self._lookup_base_from_scan(scan_msg.header.frame_id)
            if tr is None:
                continue

            # Mapping always integrates each scan at its own odom-timestamped pose
            # (encoder-driven STM32 state estimator), so obstacles persist globally
            # as the robot moves.
            odom_is_fresh = (
                self._last_odom_rx_ns > 0
                and (self.get_clock().now().nanoseconds - self._last_odom_rx_ns) <= int(odom_stale_timeout_s * 1e9)
            )

            base_pose_scan: Optional[Pose2D] = None
            if odom_is_fresh:
                odom_pose_scan = self._odom_pose_at(stamp_ns)
                if odom_pose_scan is not None:
                    base_pose_scan = self._odom_pose_from_stm_for_mapping(odom_pose_scan)

                if base_pose_scan is None:
                    base_pose_scan = self._odom_pose_at_in_map_frame(stamp_ns, timeout_s)

            if base_pose_scan is None:
                base_pose_scan = self._lookup_tf_pose_in_map_frame(stamp_ns, timeout_s)

            if base_pose_scan is None:
                if map_frame == odom_frame:
                    base_pose_scan = self._odom_pose_at(stamp_ns) or base_pose_now
                else:
                    self._warn_tf_throttled(
                        "Skipping mapping scan integration: cannot resolve scan-time pose into map frame."
                    )
                    continue

            a0 = float(scan_msg.angle_min)
            inc = float(scan_msg.angle_increment)
            rmin = float(scan_msg.range_min)
            rmax = min(float(scan_msg.range_max), max_range)

            processed = 0
            ranges = scan_msg.ranges
            for i in range(0, len(ranges), stride):
                if processed >= beam_budget:
                    break
                processed += 1

                a = a0 + i * inc
                rr = float(ranges[i])
                finite_hit = math.isfinite(rr) and rr >= rmin and rr < (float(scan_msg.range_max) - no_hit_eps) and rr <= max_range
                if finite_hit:
                    free_to = max(0.0, rr - 0.5 * self.mapping_grid_spec.res)
                else:
                    free_to = max(0.0, rmax * no_return_free_ratio)

                if free_to > 1e-6:
                    n_steps = max(1, int(math.floor(free_to / step_m)))
                    for j in range(1, n_steps + 1):
                        d = min(free_to, j * step_m)
                        xb, yb = self._apply_transform_2d(tr, d * math.cos(a), d * math.sin(a))
                        if use_excl and (xb * xb + yb * yb) <= excl_r2:
                            continue
                        xm, ym = self._se2_apply(base_pose_scan, xb, yb)
                        ix, iy = self._mapping_world_to_grid(xm, ym)
                        if not self._mapping_in_bounds(ix, iy):
                            break
                        self._mapping_update_cell(iy, ix, -lo_free, lo_min, lo_max)

                if finite_hit:
                    xb, yb = self._apply_transform_2d(tr, rr * math.cos(a), rr * math.sin(a))
                    if use_excl and (xb * xb + yb * yb) <= excl_r2:
                        continue
                    xm, ym = self._se2_apply(base_pose_scan, xb, yb)
                    ix, iy = self._mapping_world_to_grid(xm, ym)
                    if self._mapping_in_bounds(ix, iy):
                        self._mapping_update_cell(iy, ix, lo_hit, lo_min, lo_max)

    def _mapping_occ_grid(self) -> np.ndarray:
        if self.mapping_logodds is None:
            return np.zeros((1, 1), dtype=np.int8)
        occ_set = int(self.get_parameter("mapping_occ_set_threshold").value)
        occ_clear = int(self.get_parameter("mapping_occ_clear_threshold").value)
        if occ_clear >= occ_set:
            occ_clear = occ_set - 1

        if self._mapping_occ_state is None or self._mapping_occ_state.shape != self.mapping_logodds.shape:
            self._mapping_occ_state = np.zeros(self.mapping_logodds.shape, dtype=np.uint8)

        self._mapping_occ_state[self.mapping_logodds >= occ_set] = 1
        self._mapping_occ_state[self.mapping_logodds <= occ_clear] = 0

        occ = np.zeros(self.mapping_logodds.shape, dtype=np.int8)
        occ[self._mapping_occ_state > 0] = 100
        return occ

    def _project_saved_map_to_static(self, saved_occ: np.ndarray, saved_meta: Dict[str, float]) -> None:
        out = np.zeros((self.gs_map.height, self.gs_map.width), dtype=np.int8)
        ys, xs = np.where(saved_occ >= 100)
        if len(xs) == 0:
            self.static_occ = out
            self.persistent_evidence.fill(0)
            self._planning_hard_evidence.fill(0)
            return

        s_res = float(saved_meta["resolution_m"])
        s_ox = float(saved_meta["origin_x"])
        s_oy = float(saved_meta["origin_y"])
        s_yaw = float(saved_meta.get("origin_yaw_rad", 0.0))

        lx = (xs.astype(np.float64) + 0.5) * s_res
        ly = (ys.astype(np.float64) + 0.5) * s_res
        if abs(s_yaw) > 1e-9:
            c = math.cos(s_yaw)
            s = math.sin(s_yaw)
            wx = s_ox + c * lx - s * ly
            wy = s_oy + s * lx + c * ly
        else:
            wx = s_ox + lx
            wy = s_oy + ly

        gx = ((wx - self.gs_map.origin_x) / self.gs_map.res).astype(np.int32)
        gy = ((wy - self.gs_map.origin_y) / self.gs_map.res).astype(np.int32)
        valid = (
            (gx >= 0)
            & (gx < self.gs_map.width)
            & (gy >= 0)
            & (gy < self.gs_map.height)
        )
        out[gy[valid], gx[valid]] = 100

        self.static_occ = out
        self.persistent_evidence.fill(0)
        self._planning_hard_evidence.fill(0)

    def _project_slam_map_to_static(self, slam_msg: OccupancyGrid) -> None:
        """Project a live SLAM map into runtime static layer; unknown cells do not overwrite prior state."""
        width = int(slam_msg.info.width)
        height = int(slam_msg.info.height)
        if width <= 0 or height <= 0:
            return

        raw = np.asarray(slam_msg.data, dtype=np.int16)
        if raw.size != width * height:
            return

        raw = raw.reshape((height, width))
        occ_mask = raw >= 50
        free_mask = (raw >= 0) & (raw < 50)

        src_gs = GridSpec(
            res=float(slam_msg.info.resolution),
            width=width,
            height=height,
            origin_x=float(slam_msg.info.origin.position.x),
            origin_y=float(slam_msg.info.origin.position.y),
        )
        oq = slam_msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (float(oq.w) * float(oq.z) + float(oq.x) * float(oq.y)),
            1.0 - 2.0 * (float(oq.y) * float(oq.y) + float(oq.z) * float(oq.z)),
        )
        c_yaw = math.cos(origin_yaw)
        s_yaw = math.sin(origin_yaw)

        out = self.static_occ.copy()

        if np.any(free_mask):
            ys, xs = np.where(free_mask)
            lx = (xs.astype(np.float64) + 0.5) * src_gs.res
            ly = (ys.astype(np.float64) + 0.5) * src_gs.res
            wx = src_gs.origin_x + c_yaw * lx - s_yaw * ly
            wy = src_gs.origin_y + s_yaw * lx + c_yaw * ly
            gx = ((wx - self.gs_map.origin_x) / self.gs_map.res).astype(np.int32)
            gy = ((wy - self.gs_map.origin_y) / self.gs_map.res).astype(np.int32)
            valid = (gx >= 0) & (gx < self.gs_map.width) & (gy >= 0) & (gy < self.gs_map.height)
            out[gy[valid], gx[valid]] = 0

        if np.any(occ_mask):
            ys, xs = np.where(occ_mask)
            lx = (xs.astype(np.float64) + 0.5) * src_gs.res
            ly = (ys.astype(np.float64) + 0.5) * src_gs.res
            wx = src_gs.origin_x + c_yaw * lx - s_yaw * ly
            wy = src_gs.origin_y + s_yaw * lx + c_yaw * ly
            gx = ((wx - self.gs_map.origin_x) / self.gs_map.res).astype(np.int32)
            gy = ((wy - self.gs_map.origin_y) / self.gs_map.res).astype(np.int32)
            valid = (gx >= 0) & (gx < self.gs_map.width) & (gy >= 0) & (gy < self.gs_map.height)
            out[gy[valid], gx[valid]] = 100

        self.static_occ = out

    def _save_static_map(self, output_path: str) -> None:
        mapping_occ: np.ndarray
        gs: GridSpec
        logodds_to_save: np.ndarray

        if self.mapping_active and self._latest_slam_map is not None:
            slam_msg = self._latest_slam_map
            width = int(slam_msg.info.width)
            height = int(slam_msg.info.height)
            if width <= 0 or height <= 0:
                raise RuntimeError("Latest SLAM map has invalid dimensions")

            raw = np.asarray(slam_msg.data, dtype=np.int16)
            if raw.size != width * height:
                raise RuntimeError("Latest SLAM map data size mismatch")

            raw = raw.reshape((height, width))
            mapping_occ = np.where(raw >= 50, 100, 0).astype(np.int8)
            gs = GridSpec(
                res=float(slam_msg.info.resolution),
                width=width,
                height=height,
                origin_x=float(slam_msg.info.origin.position.x),
                origin_y=float(slam_msg.info.origin.position.y),
            )
            oq = slam_msg.info.origin.orientation
            origin_yaw = math.atan2(
                2.0 * (float(oq.w) * float(oq.z) + float(oq.x) * float(oq.y)),
                1.0 - 2.0 * (float(oq.y) * float(oq.y) + float(oq.z) * float(oq.z)),
            )
            logodds_to_save = np.zeros((height, width), dtype=np.int16)
            self.mapping_grid_spec = gs
            self.mapping_logodds = logodds_to_save.copy()
        else:
            if self.mapping_grid_spec is None:
                raise RuntimeError("Mapping grid not initialized")
            mapping_occ = self._mapping_occ_grid()
            gs = self.mapping_grid_spec
            origin_yaw = 0.0
            logodds_to_save = (
                self.mapping_logodds.astype(np.int16)
                if self.mapping_logodds is not None
                else np.zeros((gs.height, gs.width), dtype=np.int16)
            )

        meta = {
            "frame": self.get_parameter("map_frame").value,
            "base_frame": self.get_parameter("base_frame").value,
            "pose_convention": SAVED_MAP_POSE_CONVENTION,
            "format": "occupancy_logodds_v1",
            "resolution_m": float(gs.res),
            "width": int(gs.width),
            "height": int(gs.height),
            "origin_x": float(gs.origin_x),
            "origin_y": float(gs.origin_y),
            "origin_yaw_rad": float(origin_yaw),
            "saved_unix_s": time.time(),
        }

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        np.savez_compressed(
            output_path,
            occ=mapping_occ.astype(np.int8),
            logodds=logodds_to_save,
            meta=json.dumps(meta),
        )

        self.saved_map_bounds = self._bounds_from_grid_spec(gs)
        self.active_geofence_bounds = None
        self.geofence_anchor_corner = None
        self._project_saved_map_to_static(mapping_occ, meta)

    def _load_static_map(self, input_path: str) -> Tuple[bool, str]:
        if not os.path.isfile(input_path):
            return False, f"Saved map not found: {input_path}"

        try:
            with np.load(input_path, allow_pickle=False) as data:
                occ = data["occ"]
                meta_raw = data["meta"] if "meta" in data else None
                logodds_raw = np.asarray(data["logodds"], dtype=np.int16) if "logodds" in data else None

            meta: Dict[str, float] = {}
            if meta_raw is not None:
                if hasattr(meta_raw, "item"):
                    meta_json = meta_raw.item()
                else:
                    meta_json = str(meta_raw)
                meta = json.loads(meta_json)

            if (
                "resolution_m" in meta
                and "origin_x" in meta
                and "origin_y" in meta
                and "width" in meta
                and "height" in meta
                and int(meta["width"]) == int(occ.shape[1])
                and int(meta["height"]) == int(occ.shape[0])
            ):
                expected_map_frame = str(self.get_parameter("map_frame").value)
                saved_frame = str(meta.get("frame", ""))
                if saved_frame and saved_frame != expected_map_frame:
                    return (
                        False,
                        f"Saved map frame mismatch: file uses '{saved_frame}', runtime expects '{expected_map_frame}'.",
                    )

                saved_pose_convention = str(meta.get("pose_convention", ""))
                if saved_pose_convention != SAVED_MAP_POSE_CONVENTION:
                    if saved_pose_convention:
                        return (
                            False,
                            "Saved map pose convention mismatch: "
                            f"file uses '{saved_pose_convention}', runtime expects '{SAVED_MAP_POSE_CONVENTION}'.",
                        )
                    return (
                        False,
                        "Saved map is missing pose convention metadata and may have been created before the STM32/LiDAR "
                        "forward-axis alignment update. Regenerate the map before loading it.",
                    )

                self._project_saved_map_to_static(np.asarray(occ, dtype=np.int8), meta)
                self.mapping_grid_spec = GridSpec(
                    res=float(meta["resolution_m"]),
                    width=int(meta["width"]),
                    height=int(meta["height"]),
                    origin_x=float(meta["origin_x"]),
                    origin_y=float(meta["origin_y"]),
                )
                self.saved_map_bounds = self._bounds_from_grid_spec(self.mapping_grid_spec)
                self.active_geofence_bounds = None
                self.geofence_anchor_corner = None
                if logodds_raw is not None:
                    if logodds_raw.shape == occ.shape:
                        self.mapping_logodds = logodds_raw.copy()
                    else:
                        self.mapping_logodds = np.zeros((self.mapping_grid_spec.height, self.mapping_grid_spec.width), dtype=np.int16)
                else:
                    self.mapping_logodds = np.zeros((self.mapping_grid_spec.height, self.mapping_grid_spec.width), dtype=np.int16)

                self._mapping_occ_state = np.zeros((self.mapping_grid_spec.height, self.mapping_grid_spec.width), dtype=np.uint8)
                self._mapping_occ_state[np.asarray(occ, dtype=np.int8) >= 100] = 1

                return True, f"Loaded saved map and projected to runtime grid: {input_path}"

            if occ.shape == (self.gs_map.height, self.gs_map.width):
                self.static_occ = np.asarray(occ, dtype=np.int8)
                self.persistent_evidence.fill(0)
                self._planning_hard_evidence.fill(0)
                self.saved_map_bounds = self._bounds_from_grid_spec(self.gs_map)
                self.active_geofence_bounds = None
                self.geofence_anchor_corner = None
                return True, f"Loaded legacy saved static map: {input_path}"

            return (
                False,
                f"Saved map shape mismatch: got {occ.shape}, expected {(self.gs_map.height, self.gs_map.width)} or metadata-backed map",
            )
        except Exception as e:
            return False, f"Failed loading saved map {input_path}: {e}"

    def _resolve_map_load_path(self, input_path: str) -> Tuple[str, Optional[str]]:
        if os.path.isfile(input_path):
            return input_path, None

        map_dir = os.path.dirname(input_path)
        if not map_dir or not os.path.isdir(map_dir):
            return input_path, None

        patterns = ["*costmap*.npz", "*map*.npz", "*.npz"]
        candidates: List[str] = []
        for pattern in patterns:
            matches = [p for p in glob.glob(os.path.join(map_dir, pattern)) if os.path.isfile(p)]
            if matches:
                candidates = sorted(matches, key=os.path.getmtime, reverse=True)
                break

        if not candidates:
            return input_path, None

        selected = candidates[0]
        note = (
            f"Configured saved map missing ({input_path}). "
            f"Falling back to newest map candidate: {selected}"
        )
        return selected, note

    def _persistent_threshold_scans(self, sec_param: str) -> int:
        t_s = max(0.0, float(self.get_parameter(sec_param).value))
        return max(2, int(round(t_s * max(self._main_loop_hz, 1e-6))))

    def _update_persistent_map_from_fused_scan(self, fused: LaserScan, base_pose_now: Pose2D) -> None:
        occ_thresh = self._persistent_threshold_scans("persistent_confirm_time_s")
        free_thresh = self._persistent_threshold_scans("persistent_clear_time_s")
        evidence_cap = max(2, int(self.get_parameter("persistent_evidence_cap").value))
        allow_inf_clear = bool(self.get_parameter("persistent_inf_clearing_enable").value)
        inf_ratio = float(self.get_parameter("persistent_inf_clearing_ratio").value)
        inf_ratio = min(1.0, max(0.0, inf_ratio))

        occ_cells: set[Tuple[int, int]] = set()
        free_cells: set[Tuple[int, int]] = set()

        use_excl = bool(self.get_parameter("robot_exclusion_enable").value)
        excl_r = float(self.get_parameter("robot_exclusion_radius_m").value)
        excl_r2 = excl_r * excl_r

        a = float(fused.angle_min)
        inc = float(fused.angle_increment)
        range_max = float(fused.range_max)

        step_m = max(self.gs_map.res * 0.5, 0.02)

        for r in fused.ranges:
            rr = float(r)
            finite_hit = math.isfinite(rr)

            if finite_hit:
                occ_range = max(0.0, rr)
                free_range = max(0.0, rr - 0.5 * self.gs_map.res)
            else:
                if not allow_inf_clear:
                    a += inc
                    continue
                occ_range = -1.0
                free_range = max(0.0, range_max * inf_ratio)

            if free_range > 1e-6:
                n_steps = max(1, int(math.floor(free_range / step_m)))
                for j in range(1, n_steps + 1):
                    d = min(free_range, j * step_m)
                    xb = d * math.cos(a)
                    yb = d * math.sin(a)
                    if use_excl and (xb * xb + yb * yb) <= excl_r2:
                        continue
                    xm, ym = self._se2_apply(base_pose_now, xb, yb)
                    ix, iy = self.world_to_grid(self.gs_map, xm, ym)
                    if not self.in_bounds(self.gs_map, ix, iy):
                        break
                    free_cells.add((iy, ix))

            if finite_hit and occ_range >= 0.0:
                xb = occ_range * math.cos(a)
                yb = occ_range * math.sin(a)
                if not (use_excl and (xb * xb + yb * yb) <= excl_r2):
                    xm, ym = self._se2_apply(base_pose_now, xb, yb)
                    ix, iy = self.world_to_grid(self.gs_map, xm, ym)
                    if self.in_bounds(self.gs_map, ix, iy):
                        occ_cells.add((iy, ix))

            a += inc

        touched = free_cells | occ_cells
        if not touched:
            return

        for k in touched:
            ev = int(self.persistent_evidence[k])
            if k in occ_cells:
                ev = min(evidence_cap, ev + 1)
            elif k in free_cells:
                ev = max(-evidence_cap, ev - 1)
            self.persistent_evidence[k] = ev

            if ev >= occ_thresh:
                self.static_occ[k] = 100
            elif ev <= -free_thresh:
                self.static_occ[k] = 0

    # =======================
    # Waypoints parsing + removal
    # =======================
    def _transform_point_to_map(
        self,
        x: float,
        y: float,
        src_frame: str,
        stamp: Optional[Time] = None,
    ) -> Optional[Tuple[float, float]]:
        map_frame = self.get_parameter("map_frame").value
        sf = (src_frame or "").lstrip("/")

        if sf == "":
            self._warn_tf_throttled("Received waypoint with empty frame_id; dropping waypoint.")
            return None

        if sf == map_frame:
            return (x, y)

        lookup_times: List[Time] = []
        if stamp is not None:
            lookup_times.append(stamp)
            if stamp.nanoseconds != 0:
                lookup_times.append(Time())
        else:
            lookup_times.append(Time())

        last_exc: Optional[Exception] = None
        for lookup_time in lookup_times:
            try:
                tf = self.tf_buffer.lookup_transform(
                    map_frame,
                    sf,
                    lookup_time,
                    timeout=Duration(seconds=0.2),
                )
                return self._apply_transform_2d(tf.transform, x, y)
            except Exception as exc:
                last_exc = exc

        if len(lookup_times) > 1:
            self._warn_tf_throttled(
                f"Failed to transform waypoint {sf} -> {map_frame} at stamped time and latest TF; dropping waypoint. ({last_exc})"
            )
        else:
            self._warn_tf_throttled(f"No TF {map_frame} <- {sf} for waypoint; dropping waypoint. ({last_exc})")
        return None

    def _queue_waypoint(self, x: float, y: float) -> None:
        self._pending_waypoints.append((float(x), float(y)))

    def _append_waypoint(self, x: float, y: float) -> None:
        wp_flat = list(self.get_parameter("waypoints").value)
        wp_n = int(self.get_parameter("wp_n").value)

        stride = 2
        if wp_n > 0 and len(wp_flat) == 3 * wp_n:
            stride = 3

        if wp_n == 0:
            wp_flat = [x, y] if stride == 2 else [x, y, float("nan")]
        else:
            if stride == 2:
                wp_flat.extend([x, y])
            else:
                wp_flat.extend([x, y, float("nan")])

        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, wp_flat),
                Parameter("wp_n", Parameter.Type.INTEGER, wp_n + 1),
            ]
        )
        self.get_logger().info(f"Added waypoint ({x:.2f}, {y:.2f}). Total: {wp_n + 1}")

    def _generate_test_waypoint_pattern(self, center_pose: Pose2D) -> List[Tuple[float, float]]:
        """Build waypoints as a bounded random walk centered on robot pose."""
        area_size_m = 2.0
        step_m = 0.75
        n_waypoints = 200
        half = 0.5 * area_size_m

        min_x = float(center_pose.x) - half
        max_x = float(center_pose.x) + half
        min_y = float(center_pose.y) - half
        max_y = float(center_pose.y) + half

        rng = np.random.default_rng()

        waypoints: List[Tuple[float, float]] = [(float(center_pose.x), float(center_pose.y))]
        while len(waypoints) < n_waypoints:
            last_x, last_y = waypoints[-1]

            # Pick a random heading that keeps the next point inside the fixed 2.0m box.
            next_pt: Optional[Tuple[float, float]] = None
            for _ in range(256):
                theta = float(rng.uniform(-math.pi, math.pi))
                nx = last_x + step_m * math.cos(theta)
                ny = last_y + step_m * math.sin(theta)
                if min_x <= nx <= max_x and min_y <= ny <= max_y:
                    next_pt = (nx, ny)
                    break

            if next_pt is None:
                # Deterministic fallback points inward to keep progress guaranteed.
                theta = math.atan2(center_pose.y - last_y, center_pose.x - last_x)
                nx = last_x + step_m * math.cos(theta)
                ny = last_y + step_m * math.sin(theta)
                next_pt = (nx, ny)

            waypoints.append((float(next_pt[0]), float(next_pt[1])))

        return waypoints

    def _refill_auto_test_waypoint_window(self) -> None:
        if not self._auto_test_waypoint_active:
            return

        wp_n = int(self.get_parameter("wp_n").value)
        while wp_n < self._auto_test_waypoint_window and self._auto_test_waypoint_pool:
            x, y = self._auto_test_waypoint_pool.popleft()
            self._append_waypoint(x, y)
            wp_n += 1

        if wp_n == 0 and not self._auto_test_waypoint_pool:
            self._auto_test_waypoint_active = False
            self._wp_test_toggle_armed = False
            self.get_logger().info("Auto test waypoint run complete.")

    def _remove_nearest_waypoint(self, x: float, y: float, radius: float) -> bool:
        if radius <= 0.0:
            return False

        wp_n = int(self.get_parameter("wp_n").value)
        if wp_n <= 0:
            return False

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            return False

        best_i = -1
        best_d = float("inf")
        for i in range(wp_n):
            wx = float(wp_flat[stride * i])
            wy = float(wp_flat[stride * i + 1])
            d = math.hypot(wx - x, wy - y)
            if d < best_d:
                best_d = d
                best_i = i

        if best_i < 0 or best_d > radius:
            return False

        start = stride * best_i
        del wp_flat[start:start + stride]
        wp_n -= 1

        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, wp_flat if wp_n > 0 else [float("nan"), float("nan")]),
                Parameter("wp_n", Parameter.Type.INTEGER, wp_n),
            ]
        )
        self.get_logger().info(f"Removed waypoint near ({x:.2f}, {y:.2f}); {wp_n} remaining.")
        return True

    def on_clicked_point(self, msg: PointStamped) -> None:
        pt = self._transform_point_to_map(
            float(msg.point.x),
            float(msg.point.y),
            msg.header.frame_id,
            Time.from_msg(msg.header.stamp),
        )
        if pt is None:
            return
        self._queue_waypoint(pt[0], pt[1])

    def on_nav_goal(self, msg: PoseStamped) -> None:
        pt = self._transform_point_to_map(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            msg.header.frame_id,
            Time.from_msg(msg.header.stamp),
        )
        if pt is None:
            return
        self._queue_waypoint(pt[0], pt[1])

    def on_remove_waypoint_pose(self, msg: PoseWithCovarianceStamped) -> None:
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        pt = self._transform_point_to_map(raw_x, raw_y, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        remove_radius = float(self.get_parameter("remove_waypoint_radius_m").value)

        candidates: List[Tuple[float, float, str]] = []
        if pt is not None:
            candidates.append((float(pt[0]), float(pt[1]), "transformed"))
        candidates.append((raw_x, raw_y, "raw"))

        tried: List[Tuple[float, float, str]] = []
        for cx, cy, source in candidates:
            duplicate = any(math.hypot(cx - tx, cy - ty) < 1e-6 for tx, ty, _ in tried)
            if duplicate:
                continue
            tried.append((cx, cy, source))
            if self._remove_nearest_waypoint(cx, cy, remove_radius):
                if source == "raw":
                    self.get_logger().warn(
                        "Waypoint removal succeeded using raw /initialpose coordinates; check RViz frame alignment."
                    )
                return

        if tried:
            t0x, t0y, _ = tried[0]
            self.get_logger().info(
                f"No waypoint within {remove_radius:.2f}m of clicked remove point ({t0x:.2f}, {t0y:.2f})."
            )

    def consume_add_wp(self) -> None:
        while self._pending_waypoints:
            x, y = self._pending_waypoints.popleft()
            self._append_waypoint(x, y)

        add_wp = self.get_parameter("add_wp").value
        if not add_wp or len(add_wp) not in (2, 3):
            return
        if not (math.isfinite(add_wp[0]) and math.isfinite(add_wp[1])):
            return

        self._append_waypoint(float(add_wp[0]), float(add_wp[1]))
        self.set_parameters(
            [
                Parameter("add_wp", Parameter.Type.DOUBLE_ARRAY, [float("nan"), float("nan"), float("nan")]),
            ]
        )

    def read_waypoints(self) -> Optional[List[Tuple[float, float]]]:
        wp_n = int(self.get_parameter("wp_n").value)
        if wp_n <= 0:
            return None

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            return None

        return [(float(wp_flat[stride * i]), float(wp_flat[stride * i + 1])) for i in range(wp_n)]

    @staticmethod
    def _waypoint_key(x: float, y: float) -> Tuple[int, int]:
        return (int(round(x * 1000.0)), int(round(y * 1000.0)))

    def _find_relocation_for_waypoint(
        self,
        costmap: np.ndarray,
        waypoint_xy: Tuple[float, float],
        base_pose_now: Pose2D,
        max_search_m: float,
    ) -> Optional[Tuple[float, float, str]]:
        wx, wy = waypoint_xy
        w_ix, w_iy = self.world_to_grid(self.gs_map, wx, wy)
        if not self.in_bounds(self.gs_map, w_ix, w_iy):
            return None

        max_cells = max(1, int(math.ceil(max_search_m / max(self.gs_map.res, 1e-6))))

        # 1) Prefer nearest free cell
        best_free: Optional[Tuple[int, int]] = None
        best_free_d2 = float("inf")
        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                ix = w_ix + dx
                iy = w_iy + dy
                if not self.in_bounds(self.gs_map, ix, iy):
                    continue
                cell_val = costmap[iy, ix]
                if cell_val >= 50:
                    continue
                d2 = dx * dx + dy * dy
                if d2 < best_free_d2:
                    best_free_d2 = d2
                    best_free = (ix, iy)

        if best_free is not None:
            rx, ry = self.grid_to_world(self.gs_map, best_free[0], best_free[1])
            return (rx, ry, "free")

        # 2) If no nearby free cell, use nearest soft cell in robot forward direction
        fx = math.cos(base_pose_now.yaw)
        fy = math.sin(base_pose_now.yaw)
        best_soft_xy: Optional[Tuple[float, float]] = None
        best_soft_d2 = float("inf")

        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                ix = w_ix + dx
                iy = w_iy + dy
                if not self.in_bounds(self.gs_map, ix, iy):
                    continue
                cell_val = costmap[iy, ix]
                if cell_val < 50 or cell_val >= 100:
                    continue

                cx, cy = self.grid_to_world(self.gs_map, ix, iy)
                rx = cx - base_pose_now.x
                ry = cy - base_pose_now.y
                if rx * fx + ry * fy <= 0.0:
                    continue

                d2_wp = (cx - wx) * (cx - wx) + (cy - wy) * (cy - wy)
                if d2_wp < best_soft_d2:
                    best_soft_d2 = d2_wp
                    best_soft_xy = (cx, cy)

        if best_soft_xy is not None:
            return (best_soft_xy[0], best_soft_xy[1], "soft-forward")

        # 3) Final fallback: nearest soft cell in any direction.
        # This prevents permanent planning starvation when a waypoint is trapped
        # in hard inflation and forward-only soft candidates are unavailable.
        best_soft_any_xy: Optional[Tuple[float, float]] = None
        best_soft_any_d2 = float("inf")

        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                ix = w_ix + dx
                iy = w_iy + dy
                if not self.in_bounds(self.gs_map, ix, iy):
                    continue
                cell_val = costmap[iy, ix]
                if cell_val < 50 or cell_val >= 100:
                    continue

                cx, cy = self.grid_to_world(self.gs_map, ix, iy)
                d2_wp = (cx - wx) * (cx - wx) + (cy - wy) * (cy - wy)
                if d2_wp < best_soft_any_d2:
                    best_soft_any_d2 = d2_wp
                    best_soft_any_xy = (cx, cy)

        if best_soft_any_xy is not None:
            return (best_soft_any_xy[0], best_soft_any_xy[1], "soft-any")

        return None

    def _relocate_stuck_waypoints(self, costmap: np.ndarray, base_pose_now: Pose2D) -> Optional[List[Tuple[float, float]]]:
        wp_n = int(self.get_parameter("wp_n").value)
        if wp_n <= 0:
            self._wp_hard_cell_iters.clear()
            return None

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            return None

        hard_iters_required = 3
        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        max_search_m = max(
            0.5,
            self.gs_map.res,
            2.0 * max(hard_r, soft_r, self.gs_map.res),
        )

        active_keys: set[Tuple[int, int]] = set()
        changed = False

        for i in range(wp_n):
            idx0 = stride * i
            wx = float(wp_flat[idx0])
            wy = float(wp_flat[idx0 + 1])
            key = self._waypoint_key(wx, wy)
            active_keys.add(key)

            ix, iy = self.world_to_grid(self.gs_map, wx, wy)
            in_hard = self.in_bounds(self.gs_map, ix, iy) and costmap[iy, ix] >= 100

            if not in_hard:
                self._wp_hard_cell_iters.pop(key, None)
                continue

            new_count = self._wp_hard_cell_iters.get(key, 0) + 1
            self._wp_hard_cell_iters[key] = new_count

            if new_count < hard_iters_required:
                continue

            relocated = self._find_relocation_for_waypoint(costmap, (wx, wy), base_pose_now, max_search_m)
            if relocated is None:
                continue

            new_x, new_y, mode = relocated
            wp_flat[idx0] = float(new_x)
            wp_flat[idx0 + 1] = float(new_y)
            changed = True

            self._wp_hard_cell_iters.pop(key, None)
            new_key = self._waypoint_key(new_x, new_y)
            self._wp_hard_cell_iters[new_key] = 0
            active_keys.add(new_key)

            self.get_logger().warn(
                f"Moved waypoint {i} from hard cell after {hard_iters_required} iters "
                f"to ({new_x:.2f}, {new_y:.2f}) via {mode}."
            )

        self._wp_hard_cell_iters = {
            k: v for (k, v) in self._wp_hard_cell_iters.items() if k in active_keys
        }

        if not changed:
            return None

        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, wp_flat),
            ]
        )

        return [(float(wp_flat[stride * i]), float(wp_flat[stride * i + 1])) for i in range(wp_n)]

    def _pop_reached_waypoints(self, base_pose_now: Pose2D) -> None:
        tol = float(self.get_parameter("waypoint_reached_tol_m").value)
        if tol <= 0.0:
            self._last_pose_for_waypoint_pop = base_pose_now
            return

        wp_n = int(self.get_parameter("wp_n").value)
        if wp_n <= 0:
            self._last_pose_for_waypoint_pop = base_pose_now
            return

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            self._last_pose_for_waypoint_pop = base_pose_now
            return

        prev_pose = self._last_pose_for_waypoint_pop
        removed = 0
        while wp_n > 0:
            wx = float(wp_flat[0])
            wy = float(wp_flat[1])

            reached = math.hypot(wx - base_pose_now.x, wy - base_pose_now.y) <= tol
            if (not reached) and (prev_pose is not None):
                reached = self._segment_passes_near_waypoint(prev_pose, base_pose_now, wx, wy, tol)

            if not reached:
                break

            wp_flat = wp_flat[stride:]
            wp_n -= 1
            removed += 1

        if removed > 0:
            self.set_parameters(
                [
                    Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY,
                              wp_flat if wp_n > 0 else [float("nan"), float("nan")]),
                    Parameter("wp_n", Parameter.Type.INTEGER, wp_n),
                ]
            )

        self._last_pose_for_waypoint_pop = base_pose_now

    @staticmethod
    def _segment_passes_near_waypoint(
        p0: Pose2D,
        p1: Pose2D,
        wx: float,
        wy: float,
        tol: float,
    ) -> bool:
        """Return true if motion segment p0->p1 passes within tol of waypoint."""
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        seg_len2 = dx * dx + dy * dy
        if seg_len2 <= 1e-12:
            return False

        t = ((wx - p0.x) * dx + (wy - p0.y) * dy) / seg_len2
        t = max(0.0, min(1.0, t))

        cx = p0.x + t * dx
        cy = p0.y + t * dy
        return math.hypot(wx - cx, wy - cy) <= tol

    def handle_clear_all_waypoints(self, request, response):
        """Clear all waypoints."""
        self._auto_test_waypoint_pool.clear()
        self._auto_test_waypoint_active = False
        self._wp_test_toggle_armed = False
        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, [float("nan"), float("nan")]),
                Parameter("wp_n", Parameter.Type.INTEGER, 0),
            ]
        )
        self.get_logger().info("All waypoints cleared.")
        return response

    def handle_pop_next_waypoint(self, request, response):
        """Remove the next (first) waypoint from the queue."""
        wp_n = int(self.get_parameter("wp_n").value)
        if wp_n <= 0:
            self.get_logger().warn("No waypoints to pop.")
            return response

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            self.get_logger().warn("Invalid waypoint format.")
            return response

        # Remove first waypoint
        wp_flat = wp_flat[stride:]
        wp_n -= 1

        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY,
                          wp_flat if wp_n > 0 else [float("nan"), float("nan")]),
                Parameter("wp_n", Parameter.Type.INTEGER, wp_n),
            ]
        )
        self.get_logger().info(f"Popped next waypoint. {wp_n} remaining.")
        self._refill_auto_test_waypoint_window()
        return response

    def handle_generate_test_pattern_waypoints(self, request, response):
        if self._wp_test_toggle_armed:
            # Deterministic toggle behavior: every second wp t clears.
            self._auto_test_waypoint_pool.clear()
            self._auto_test_waypoint_active = False
            self._wp_test_toggle_armed = False
            self.set_parameters(
                [
                    Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, [float("nan"), float("nan")]),
                    Parameter("wp_n", Parameter.Type.INTEGER, 0),
                ]
            )
            response.success = True
            response.message = "Cleared all waypoints (wp t toggle)."
            self.get_logger().info(response.message)
            return response

        center_pose = self._base_pose_now()
        generated = self._generate_test_waypoint_pattern(center_pose)

        self._auto_test_waypoint_pool = deque(generated)
        self._auto_test_waypoint_active = True
        self._wp_test_toggle_armed = True
        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, [float("nan"), float("nan")]),
                Parameter("wp_n", Parameter.Type.INTEGER, 0),
            ]
        )
        self._refill_auto_test_waypoint_window()
        queued_now = int(self.get_parameter("wp_n").value)

        response.success = True
        response.message = (
            f"Generated {len(generated)} waypoints in a 1.5m x 1.5m area centered at "
            f"({center_pose.x:.2f}, {center_pose.y:.2f}); queued {queued_now} now with rolling window "
            f"{self._auto_test_waypoint_window}."
        )
        self.get_logger().info(response.message)
        return response

    def handle_mapping_start(self, request, response):
        self.mapping_active = True
        self.use_saved_map_only = False
        self._mapping_pose_anchor_tf = None
        self._capture_mapping_pose_anchor(timeout_s=0.2, warn_on_failure=True)

        base_pose_now = self._base_pose_now()
        self._init_mapping_grid(base_pose_now)
        self._reset_mapping_buffers()
        self._cached_traj = None
        self._cached_traj_collision_iters = 0
        self._last_plan_wps_sig = None
        response.success = True
        response.message = "Mapping started: accumulating robust 10x10m occupancy (log-odds) from fused LiDAR."
        self.get_logger().info(response.message)
        return response

    def handle_mapping_finish(self, request, response):
        if not self.mapping_active:
            response.success = False
            response.message = "Mapping is not active."
            self.get_logger().warn(response.message)
            return response

        output_path = str(self.get_parameter("mapping_save_path").value)
        try:
            self._save_static_map(output_path)
            self.mapping_active = False
            self.use_saved_map_only = False
            self._mapping_pose_anchor_tf = None
            self._cached_traj = None
            self._cached_traj_collision_iters = 0
            self._last_plan_wps_sig = None
            self._clear_geofence()
            self.blank_mode_anchor_pose = None
            response.success = True
            response.message = (
                f"Mapping finished and saved: {output_path}. "
                "Live LiDAR costmap remains active; frozen-map mode is disabled."
            )
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to save map: {e}"
            self.get_logger().error(response.message)
            return response

    def handle_mapping_use_live(self, request, response):
        self.mapping_active = False
        self.use_saved_map_only = False
        self._mapping_pose_anchor_tf = None
        self._cached_traj = None
        self._cached_traj_collision_iters = 0
        self._last_plan_wps_sig = None
        response.success = True
        response.message = "Switched to live LiDAR map mode."
        self.get_logger().info(response.message)
        return response

    def handle_mapping_use_frozen(self, request, response):
        map_path = str(self.get_parameter("mapping_save_path").value)
        load_path, fallback_note = self._resolve_map_load_path(map_path)
        loaded, message = self._load_static_map(load_path)
        if fallback_note is not None:
            self.get_logger().warn(fallback_note)
        if not loaded:
            response.success = False
            response.message = message
            self.get_logger().warn(response.message)
            return response

        self.mapping_active = False
        self.use_saved_map_only = False
        self._mapping_pose_anchor_tf = None
        self._cached_traj = None
        self._cached_traj_collision_iters = 0
        self._last_plan_wps_sig = None
        self._clear_geofence()
        self.blank_mode_anchor_pose = None
        response.success = True
        response.message = "Loaded saved map; keeping live LiDAR costmap mode active (frozen mode deprecated)."
        self.get_logger().info(response.message)
        return response

    def handle_mapping_use_blank(self, request, response):
        self.mapping_active = False
        self.use_saved_map_only = False
        self._mapping_pose_anchor_tf = None
        self._cached_traj = None
        self._cached_traj_collision_iters = 0
        self._last_plan_wps_sig = None

        start_pose = self._base_pose_now()

        # Clear any prior static/global map so planning uses blank global space.
        self.static_occ.fill(0)
        self.persistent_evidence.fill(0)
        self._planning_hard_evidence.fill(0)
        self._center_global_grid_on_pose(start_pose)
        self.saved_map_bounds = None
        self._clear_geofence()
        self.blank_mode_anchor_pose = start_pose
        self._ensure_geofence(start_pose)
        self._latest_slam_map = None
        self._latest_slam_map_ns = 0

        response.success = True
        response.message = (
            "Switched to blank global map mode; local LiDAR costmap avoidance remains active "
            "with the global map centered at the traj-local start pose."
        )
        self.get_logger().info(response.message)
        return response

    # =======================
    # Planning (A*) — unchanged / minimal
    # =======================
    def line_collision_free(self, grid: np.ndarray, a_xy: Tuple[float, float], b_xy: Tuple[float, float]) -> bool:
        ax, ay = a_xy
        bx, by = b_xy
        dist = math.hypot(bx - ax, by - ay)
        if dist < 1e-9:
            return True

        step_m = max(self.gs_map.res * 0.5, 0.01)
        n = int(math.ceil(dist / step_m))
        for i in range(n + 1):
            t = i / max(n, 1)
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            ix, iy = self.world_to_grid(self.gs_map, x, y)
            if not self.in_bounds(self.gs_map, ix, iy):
                return False
            if grid[iy, ix] >= 100:  # Only block hard obstacles
                return False
        return True

    def _point_hits_hard_obstacle(self, grid: np.ndarray, x: float, y: float) -> bool:
        """Return True when the sampled trajectory point lies in a hard cell."""
        ix, iy = self.world_to_grid(self.gs_map, x, y)
        if not self.in_bounds(self.gs_map, ix, iy):
            return True
        return bool(grid[iy, ix] >= 100)

    def _trajectory_swept_collision_free(self, grid: np.ndarray, xs: List[float], ys: List[float]) -> bool:
        """Continuous hard-obstacle collision check along trajectory centerline samples."""
        n = min(len(xs), len(ys))
        if n <= 0:
            return False

        sample_step_m = max(0.01, 0.5 * self.gs_map.res)

        for i in range(n):
            x1 = float(xs[i])
            y1 = float(ys[i])
            if self._point_hits_hard_obstacle(grid, x1, y1):
                return False

            if i == 0:
                continue

            x0 = float(xs[i - 1])
            y0 = float(ys[i - 1])
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len <= sample_step_m:
                continue

            n_steps = int(math.ceil(seg_len / sample_step_m))
            for j in range(1, n_steps):
                t = j / max(1, n_steps)
                sx = x0 + t * (x1 - x0)
                sy = y0 + t * (y1 - y0)
                if self._point_hits_hard_obstacle(grid, sx, sy):
                    return False

        return True

    def _max_safe_prefix_index(self, grid: np.ndarray, xs: List[float], ys: List[float]) -> int:
        """Return last index whose swept prefix remains hard-collision free; -1 if start is unsafe."""
        n = min(len(xs), len(ys))
        if n <= 0:
            return -1

        sample_step_m = max(0.01, 0.5 * self.gs_map.res)

        x0 = float(xs[0])
        y0 = float(ys[0])
        if self._point_hits_hard_obstacle(grid, x0, y0):
            return -1

        last_safe = 0
        for i in range(1, n):
            x1 = float(xs[i])
            y1 = float(ys[i])
            seg_len = math.hypot(x1 - x0, y1 - y0)

            if seg_len <= sample_step_m:
                if self._point_hits_hard_obstacle(grid, x1, y1):
                    return last_safe
                last_safe = i
                x0, y0 = x1, y1
                continue

            n_steps = int(math.ceil(seg_len / sample_step_m))
            for j in range(1, n_steps + 1):
                t = j / max(1, n_steps)
                sx = x0 + t * (x1 - x0)
                sy = y0 + t * (y1 - y0)
                if self._point_hits_hard_obstacle(grid, sx, sy):
                    return last_safe

            last_safe = i
            x0, y0 = x1, y1

        return last_safe

    def _publish_trimmed_safe_prefix(
        self,
        frame: str,
        base_pose_now: Pose2D,
        planning_costmap: np.ndarray,
        traj: Tuple[List[float], List[float], List[float], List[float], List[float], List[float]],
        source_label: str,
    ) -> bool:
        xs, ys, yaws, velocities, vxs, vys = traj
        last_safe = self._max_safe_prefix_index(planning_costmap, xs, ys)
        if last_safe < 1:
            return False

        keep_n = last_safe + 1
        xs_t = list(xs[:keep_n])
        ys_t = list(ys[:keep_n])
        yaws_t = list(yaws[:keep_n])
        vel_t = list(velocities[:keep_n])
        vxs_t = list(vxs[:keep_n])
        vys_t = list(vys[:keep_n])

        travel = math.hypot(xs_t[-1] - xs_t[0], ys_t[-1] - ys_t[0])
        if travel < max(0.05, 2.0 * self.gs_map.res):
            return False

        # Enforce a controlled stop at the trimmed end.
        vel_t[-1] = 0.0
        vxs_t[-1] = 0.0
        vys_t[-1] = 0.0
        if len(vel_t) >= 2:
            vel_t[-2] = min(vel_t[-2], 0.0)
            vxs_t[-2] = 0.0
            vys_t[-2] = 0.0

        self._cached_traj = (xs_t, ys_t, yaws_t, vel_t, vxs_t, vys_t)
        self._cached_traj_collision_iters = 0
        self.get_logger().warn(
            f"{source_label} trajectory trimmed to safe prefix ({keep_n}/{len(xs)} pts) after swept check."
        )
        self._publish_trajectory(frame, xs_t, ys_t, yaws_t, vel_t, vxs_t, vys_t)
        return True

    def _publish_cached_if_valid(
        self,
        frame: str,
        base_pose_now: Pose2D,
        planning_costmap: np.ndarray,
    ) -> bool:
        if self._cached_traj is None:
            self._cached_traj_collision_iters = 0
            return False

        xs_c, ys_c, yaws_c, velocities_c, vxs_c, vys_c = self._cached_traj
        if self._trajectory_swept_collision_free(planning_costmap, xs_c, ys_c):
            self._cached_traj_collision_iters = 0
            self._publish_trajectory(frame, xs_c, ys_c, yaws_c, velocities_c, vxs_c, vys_c)
            return True

        self._cached_traj_collision_iters += 1
        reject_confirm_iters = max(1, int(self.get_parameter("cached_traj_reject_confirm_iters").value))
        if self._cached_traj_collision_iters < reject_confirm_iters:
            self.get_logger().warn(
                "Cached trajectory collision check failed on this cycle; "
                f"waiting for {reject_confirm_iters} consecutive failures before rejection "
                f"({self._cached_traj_collision_iters}/{reject_confirm_iters})."
            )
            self._publish_trajectory(frame, xs_c, ys_c, yaws_c, velocities_c, vxs_c, vys_c)
            return True

        if self._publish_trimmed_safe_prefix(
            frame,
            base_pose_now,
            planning_costmap,
            (xs_c, ys_c, yaws_c, velocities_c, vxs_c, vys_c),
            "Cached",
        ):
            self._cached_traj_collision_iters = 0
            return True

        self.get_logger().warn("Cached trajectory rejected by swept-footprint hard-collision check; holding.")
        self._cached_traj = None
        self._cached_traj_collision_iters = 0
        self._last_plan_wps_sig = None
        self._publish_hold_current_trajectory(frame, base_pose_now)
        return True

    def get_wheel_velocities(self, vx_body: float, vy_body: float, omega: float) -> List[float]:
        """
        Compute required wheel velocities for 3-wheel omnidirectional robot.
        Uses the same body->wheel mapping as STM32 controller.c.
        
        Args:
            vx_body: Linear velocity in x (body frame, m/s)
            vy_body: Linear velocity in y (body frame, m/s)
            omega: Angular velocity (rad/s)
        
        Returns:
            List of 3 wheel velocities [w1, w2, w3] in rad/s
        """
        r = float(self.get_parameter("wheel_radius_m").value)
        L = float(self.get_parameter("wheelbase_m").value)
        
        if r < 1e-6:
            return [0.0, 0.0, 0.0]
        
        # Match STM32 Controller_Step exactly:
        # body_desired_to_kin: vx_kin = vy_body, vy_kin = -vx_body
        # inverse_kinematics(kin):
        #   w1 = (vy_kin + L*omega)/r
        #   w2 = (-0.5*vy_kin + (sqrt(3)/2)*vx_kin + L*omega)/r
        #   w3 = (-0.5*vy_kin - (sqrt(3)/2)*vx_kin + L*omega)/r
        sqrt3_2 = math.sqrt(3.0) / 2.0

        w1 = (-vx_body + L * omega) / r
        w2 = (0.5 * vx_body + sqrt3_2 * vy_body + L * omega) / r
        w3 = (0.5 * vx_body - sqrt3_2 * vy_body + L * omega) / r
        
        return [w1, w2, w3]

    def body_velocity_from_wheel_velocity(self, w1: float, w2: float, w3: float) -> Tuple[float, float, float]:
        """
        Compute body velocity from wheel velocities.
        Inverse of get_wheel_velocities(), matched to STM32 controller mapping.
        
        Args:
            w1, w2, w3: Wheel velocities (rad/s)
        
        Returns:
            Tuple of (vx_body, vy_body, omega)
        """
        r = float(self.get_parameter("wheel_radius_m").value)
        L = float(self.get_parameter("wheelbase_m").value)
        
        if r < 1e-6 or L < 1e-6:
            return 0.0, 0.0, 0.0
        
        # Forward kinematics for STM32-matched mapping:
        # vx_body = r * (-2*w1 + w2 + w3) / 3
        # vy_body = r * (w2 - w3) / sqrt(3)
        sqrt3 = math.sqrt(3.0)

        vx_body = r * ((-2.0 * w1 + w2 + w3) / 3.0)
        vy_body = r * ((w2 - w3) / sqrt3)
        omega = r * (w1 + w2 + w3) / (3.0 * L)
        
        return vx_body, vy_body, omega

    def _effective_motion_limits(self) -> Tuple[float, float, float]:
        """Return trajectory limits clamped below STM32 controller limits."""
        wheel_radius = max(1e-6, float(self.get_parameter("wheel_radius_m").value))
        margin = float(self.get_parameter("trajectory_limit_margin").value)
        margin = min(0.999, max(0.5, margin))

        user_max_linear = max(1e-6, float(self.get_parameter("max_linear_velocity_ms").value))
        user_max_accel_lin = max(1e-6, float(self.get_parameter("max_wheel_acceleration_ms2").value))
        user_max_jerk_rs3 = max(1e-6, float(self.get_parameter("max_wheel_jerk_rs3").value))

        stm32_wheel_speed = max(1e-6, float(self.get_parameter("stm32_max_wheel_speed_rs").value))
        stm32_wheel_accel = max(1e-6, float(self.get_parameter("stm32_max_wheel_accel_rs2").value))
        stm32_wheel_jerk = max(1e-6, float(self.get_parameter("stm32_max_wheel_jerk_rs3").value))

        # Conservative omni bound: for omega=0, |wheel_speed| <= |v_body|/r.
        max_linear_from_stm = margin * wheel_radius * stm32_wheel_speed
        max_accel_from_stm = margin * wheel_radius * stm32_wheel_accel
        max_jerk_from_stm = margin * stm32_wheel_jerk

        max_linear = min(user_max_linear, max_linear_from_stm)
        max_accel_lin = min(user_max_accel_lin, max_accel_from_stm)
        max_jerk_rs3 = min(user_max_jerk_rs3, max_jerk_from_stm)

        return max_linear, max_accel_lin, max_jerk_rs3

    def max_velocity_for_acceleration(self, v_current: float, distance: float, max_accel: float) -> float:
        """
        Compute maximum velocity achievable given current velocity, 
        distance to next segment, and acceleration limit.
        
        Uses: v_next² = v_current² + 2*a*distance
        """
        if distance < 1e-6:
            return v_current
        
        v_next_sq = v_current * v_current + 2.0 * max_accel * distance
        return math.sqrt(max(0.0, v_next_sq))

    @staticmethod
    def _max_speed_for_jerk_limited_stop(distance: float, max_decel: float, max_jerk: float) -> float:
        """Maximum admissible speed that can stop within distance under S-curve limits.

        Uses a conservative binary search inversion of jerk-limited stopping distance
        with symmetric jerk profile (matching CM7 wheel jerk/accel limiting intent).
        """
        d = max(0.0, float(distance))
        a = max(1e-6, float(max_decel))
        j = max(1e-6, float(max_jerk))

        if d <= 1e-9:
            return 0.0

        def stop_distance_for_v(v0: float) -> float:
            v = max(0.0, float(v0))
            v_switch = (a * a) / j
            if v >= v_switch:
                t_j = a / j
                t_f = (v - v_switch) / a
                x1 = v * t_j - (j * (t_j ** 3)) / 6.0
                v1 = v - 0.5 * j * (t_j ** 2)
                x2 = v1 * t_f - 0.5 * a * (t_f ** 2)
                v2 = v1 - a * t_f
                x3 = v2 * t_j - 0.5 * a * (t_j ** 2) + (j * (t_j ** 3)) / 6.0
                return x1 + x2 + x3

            t_j = math.sqrt(v / j)
            x1 = v * t_j - (j * (t_j ** 3)) / 6.0
            v1 = v - 0.5 * j * (t_j ** 2)
            x2 = v1 * t_j - 0.5 * (j * t_j) * (t_j ** 2) + (j * (t_j ** 3)) / 6.0
            return x1 + x2

        v_hi = max(0.1, math.sqrt(2.0 * a * d) * 2.0)
        while stop_distance_for_v(v_hi) < d:
            v_hi *= 2.0
            if v_hi > 10.0:
                break

        v_lo = 0.0
        for _ in range(28):
            v_mid = 0.5 * (v_lo + v_hi)
            if stop_distance_for_v(v_mid) <= d:
                v_lo = v_mid
            else:
                v_hi = v_mid

        return max(0.0, v_lo)

    def _enforce_wheel_accel_limits_on_velocity_series(
        self,
        vxs: List[float],
        vys: List[float],
        dt: float,
        yaws: Optional[List[float]] = None,
        yaw_fallback: Optional[float] = None,
    ) -> Tuple[List[float], List[float], List[float]]:
        """Project velocity sequence onto per-wheel acceleration limits.

        Matches STM32 controller behavior: for each timestep, compute desired
        wheel-speed delta and scale all wheel deltas by inf-norm so
        max(|du_i|) <= aumax * dt.
        """
        n = min(len(vxs), len(vys))
        if n == 0:
            return [], [], []
        if n == 1 or dt <= 1e-6:
            v0 = [float(vxs[0])]
            v1 = [float(vys[0])]
            return v0, v1, [math.hypot(v0[0], v1[0])]

        max_linear_vel, max_wheel_accel, max_wheel_jerk = self._effective_motion_limits()
        wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        wheel_radius = max(1e-6, wheel_radius)
        max_wheel_accel_rs2 = max_wheel_accel / wheel_radius
        max_du_allowed = max(1e-9, max_wheel_accel_rs2 * dt)
        # Match CM7 controller.c jerk limiting exactly:
        # ddu_max = jerkmax * dt * dt for ddu = (du_k - du_{k-1})
        max_ddu_allowed = max(1e-9, max_wheel_jerk * dt * dt)

        out_vx = [float(v) for v in vxs[:n]]
        out_vy = [float(v) for v in vys[:n]]

        def clamp_linear_xy(vx: float, vy: float) -> Tuple[float, float]:
            vm = math.hypot(vx, vy)
            if vm > max_linear_vel and vm > 1e-9:
                s = max_linear_vel / vm
                return vx * s, vy * s
            return vx, vy

        for i in range(n):
            out_vx[i], out_vy[i] = clamp_linear_xy(out_vx[i], out_vy[i])

        if yaw_fallback is None:
            yaw_fallback = self.odom_pose_latest.yaw if self.have_odom_pose else 0.0

        def yaw_at(idx: int, yaws_local: Optional[List[float]]) -> float:
            if yaws_local is not None and idx < len(yaws_local):
                return float(yaws_local[idx])
            return float(yaw_fallback)

        def project_once(
            vx_in: List[float],
            vy_in: List[float],
            yaws_in: Optional[List[float]],
        ) -> Tuple[List[float], List[float]]:
            if not vx_in:
                return [], []

            px = [float(v) for v in vx_in]
            py = [float(v) for v in vy_in]
            for i in range(len(px)):
                px[i], py[i] = clamp_linear_xy(px[i], py[i])

            yaw0 = yaw_at(0, yaws_in)
            vx0_body, vy0_body = self._map_velocity_to_body(px[0], py[0], yaw0)
            w_prev = self.get_wheel_velocities(vx0_body, vy0_body, 0.0)
            du_prev = [0.0, 0.0, 0.0]

            for i in range(1, len(px)):
                yawi = yaw_at(i, yaws_in)
                vx_des_body, vy_des_body = self._map_velocity_to_body(px[i], py[i], yawi)
                w_des = self.get_wheel_velocities(vx_des_body, vy_des_body, 0.0)

                du0 = w_des[0] - w_prev[0]
                du1 = w_des[1] - w_prev[1]
                du2 = w_des[2] - w_prev[2]
                du_inf = max(abs(du0), abs(du1), abs(du2))

                du_cmd = [du0, du1, du2]
                if du_inf > max_du_allowed:
                    s = max_du_allowed / max(du_inf, 1e-9)
                    du_cmd[0] *= s
                    du_cmd[1] *= s
                    du_cmd[2] *= s

                ddu0 = du_cmd[0] - du_prev[0]
                ddu1 = du_cmd[1] - du_prev[1]
                ddu2 = du_cmd[2] - du_prev[2]
                ddu_inf = max(abs(ddu0), abs(ddu1), abs(ddu2))
                if ddu_inf > max_ddu_allowed:
                    s = max_ddu_allowed / max(ddu_inf, 1e-9)
                    ddu0 *= s
                    ddu1 *= s
                    ddu2 *= s

                du_cmd[0] = du_prev[0] + ddu0
                du_cmd[1] = du_prev[1] + ddu1
                du_cmd[2] = du_prev[2] + ddu2

                w_cmd = [
                    w_prev[0] + du_cmd[0],
                    w_prev[1] + du_cmd[1],
                    w_prev[2] + du_cmd[2],
                ]

                vx_body_cmd, vy_body_cmd, _ = self.body_velocity_from_wheel_velocity(
                    w_cmd[0], w_cmd[1], w_cmd[2]
                )

                c = math.cos(yawi)
                s = math.sin(yawi)
                px[i] = c * vx_body_cmd - s * vy_body_cmd
                py[i] = s * vx_body_cmd + c * vy_body_cmd
                px[i], py[i] = clamp_linear_xy(px[i], py[i])

                vx_body_post, vy_body_post = self._map_velocity_to_body(px[i], py[i], yawi)
                w_prev = self.get_wheel_velocities(vx_body_post, vy_body_post, 0.0)
                du_prev = [du_cmd[0], du_cmd[1], du_cmd[2]]

            return px, py

        # Bidirectional projection enforces feasibility for both acceleration
        # and deceleration/reorientation segments.
        out_vx, out_vy = project_once(out_vx, out_vy, yaws)

        rev_vx = list(reversed(out_vx))
        rev_vy = list(reversed(out_vy))
        rev_yaws = list(reversed(yaws[:n])) if (yaws is not None and len(yaws) >= n) else None
        rev_vx, rev_vy = project_once(rev_vx, rev_vy, rev_yaws)
        out_vx = list(reversed(rev_vx))
        out_vy = list(reversed(rev_vy))

        # Final forward pass to re-anchor start knot against current measured motion.
        out_vx, out_vy = project_once(out_vx, out_vy, yaws)

        out_v = [math.hypot(out_vx[i], out_vy[i]) for i in range(n)]
        return out_vx, out_vy, out_v

    def constrain_velocity_by_kinematics(self, vx: float, vy: float) -> Tuple[float, float]:
        """
        Constrain desired velocity (vx, vy) to respect max wheel acceleration.
        Assumes omega=0 (no rotation for now).
        Returns constrained (vx, vy).
        """
        max_linear_vel, max_wheel_accel, _ = self._effective_motion_limits()
        
        # Magnitude of desired velocity
        v_mag = math.hypot(vx, vy)
        
        # Cap by max linear velocity
        if v_mag > max_linear_vel:
            scale = max_linear_vel / max(v_mag, 1e-6)
            vx *= scale
            vy *= scale
            v_mag = max_linear_vel
        
        # For omega=0, wheel velocities are just vx/r scaled
        # Since we're not rotating, constraint simplifies to just max linear velocity
        # (Individual wheel accel is handled during trajectory smoothing)
        
        return vx, vy

    def astar(self, grid: np.ndarray, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        sx, sy = start_xy
        gx, gy = goal_xy
        sxi, syi = self.world_to_grid(self.gs_map, sx, sy)
        gxi, gyi = self.world_to_grid(self.gs_map, gx, gy)

        if not self.in_bounds(self.gs_map, sxi, syi) or not self.in_bounds(self.gs_map, gxi, gyi):
            return []
        if grid[gyi, gxi] >= 100:
            return []

        def h(ix: int, iy: int) -> float:
            return math.hypot(ix - gxi, iy - gyi)

        neigh = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        ]

        openq: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(openq, (h(sxi, syi), 0.0, (sxi, syi)))
        came: Dict[Tuple[int, int], Tuple[int, int]] = {}
        gscore = {(sxi, syi): 0.0}

        while openq:
            _, gcur, (ix, iy) = heapq.heappop(openq)
            if (ix, iy) == (gxi, gyi):
                return self._reconstruct(came, (gxi, gyi))

            for dx, dy, w in neigh:
                nx, ny = ix + dx, iy + dy
                if not self.in_bounds(self.gs_map, nx, ny):
                    continue
                if grid[ny, nx] >= 100:  # Hard obstacle blocks
                    continue
                
                # Add cell cost to movement cost: soft cells (50) add 0.5, hard cells add 1.0
                cell_cost = grid[ny, nx] / 100.0
                
                # Add direction change penalty to encourage smooth diagonal paths
                direction_penalty = 0.0
                if (ix, iy) in came:
                    px, py = came[(ix, iy)]
                    prev_dx = ix - px
                    prev_dy = iy - py
                    # Penalize direction changes (0.05 cost per 45-degree turn)
                    if (prev_dx, prev_dy) != (0, 0) and (prev_dx, prev_dy) != (dx, dy):
                        direction_penalty = 0.05
                
                ng = gcur + w + cell_cost + direction_penalty
                if (nx, ny) not in gscore or ng < gscore[(nx, ny)]:
                    gscore[(nx, ny)] = ng
                    came[(nx, ny)] = (ix, iy)
                    heapq.heappush(openq, (ng + h(nx, ny), ng, (nx, ny)))
        return []

    def _reconstruct(self, came: Dict[Tuple[int, int], Tuple[int, int]], goal_ij: Tuple[int, int]) -> List[Tuple[float, float]]:
        path_ij = [goal_ij]
        cur = goal_ij
        while cur in came:
            cur = came[cur]
            path_ij.append(cur)
        path_ij.reverse()
        return [self.grid_to_world(self.gs_map, ix, iy) for (ix, iy) in path_ij]

    def plan_segment_path(self, grid: np.ndarray, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)

        # If goal lands in a hard-inflated cell, nudge to nearest reachable cell
        # so planning can still proceed in nearby free space.
        gxi, gyi = self.world_to_grid(self.gs_map, goal_xy[0], goal_xy[1])
        if self.in_bounds(self.gs_map, gxi, gyi) and grid[gyi, gxi] >= 100:
            max_search_m = max(0.5, 2.0 * max(hard_r, soft_r, self.gs_map.res))
            relocated_goal = self._find_relocation_for_waypoint(
                grid,
                goal_xy,
                self._base_pose_now(),
                max_search_m,
            )
            if relocated_goal is not None:
                goal_xy = (relocated_goal[0], relocated_goal[1])

        if self.line_collision_free(grid, start_xy, goal_xy):
            return [start_xy, goal_xy]

        path = self.astar(grid, start_xy, goal_xy)
        if not path and soft_r > 1e-6:
            # Soft-layer fallback: keep only hard obstacles when A* fails.
            # This guards against cases where soft inflation over-constrains
            # search without an actual hard collision corridor.
            hard_only_grid = np.where(grid >= 100, 100, 0).astype(np.int8)
            if self.line_collision_free(hard_only_grid, start_xy, goal_xy):
                return [start_xy, goal_xy]
            path = self.astar(hard_only_grid, start_xy, goal_xy)

        if not path:
            return []

        path[0] = start_xy
        path[-1] = goal_xy
        
        # Simplify path to remove unnecessary waypoints (shortcutting)
        path = self.simplify_path(grid, path)
        
        return path

    def simplify_path(self, grid: np.ndarray, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Simplify path by removing unnecessary intermediate waypoints.
        Uses line-of-sight checks to shortcut straight segments.
        This dramatically reduces zig-zag patterns from grid-based A*.
        
        Args:
            grid: Costmap grid
            path: Original path from A*
            
        Returns:
            Simplified path with fewer waypoints
        """
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Try to find the farthest point we can reach directly
            farthest_idx = current_idx + 1
            
            for test_idx in range(len(path) - 1, current_idx, -1):
                if self.line_collision_free(grid, path[current_idx], path[test_idx]):
                    farthest_idx = test_idx
                    break
            
            # Add the farthest reachable point
            if farthest_idx < len(path):
                simplified.append(path[farthest_idx])
            current_idx = farthest_idx
        
        return simplified

    def _path_collision_free(self, grid: np.ndarray, path: List[Tuple[float, float]]) -> bool:
        """Check that all path points and segments stay out of hard obstacle cells."""
        if len(path) < 2:
            return True

        for i, (x, y) in enumerate(path):
            ix, iy = self.world_to_grid(self.gs_map, x, y)
            if not self.in_bounds(self.gs_map, ix, iy):
                return False
            if grid[iy, ix] >= 100:
                return False

            if i > 0 and not self.line_collision_free(grid, path[i - 1], path[i]):
                return False

        return True

    @staticmethod
    def _normalize_vec2(x: float, y: float, fallback: Tuple[float, float] = (1.0, 0.0)) -> Tuple[float, float]:
        mag = math.hypot(x, y)
        if mag <= 1e-9:
            return fallback
        return (x / mag, y / mag)

    @staticmethod
    def _estimate_path_curvature(path: List[Tuple[float, float]]) -> List[float]:
        """Estimate absolute curvature (1/m) using central differences."""
        n = len(path)
        if n < 3:
            return [0.0] * n

        kappas = [0.0] * n
        for i in range(1, n - 1):
            x_prev, y_prev = path[i - 1]
            x_curr, y_curr = path[i]
            x_next, y_next = path[i + 1]

            ds1 = max(1e-6, math.hypot(x_curr - x_prev, y_curr - y_prev))
            ds2 = max(1e-6, math.hypot(x_next - x_curr, y_next - y_curr))
            ds = 0.5 * (ds1 + ds2)

            dx_dt = (x_next - x_prev) / (2.0 * ds)
            dy_dt = (y_next - y_prev) / (2.0 * ds)
            ddx_dt = (x_next - 2.0 * x_curr + x_prev) / (ds * ds)
            ddy_dt = (y_next - 2.0 * y_curr + y_prev) / (ds * ds)

            denom = (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5
            if denom > 1e-9:
                kappas[i] = abs((dx_dt * ddy_dt - dy_dt * ddx_dt) / denom)

        kappas[0] = kappas[1]
        kappas[-1] = kappas[-2]
        return kappas

    @staticmethod
    def _resample_polyline(path: List[Tuple[float, float]], n_samples: int) -> List[Tuple[float, float]]:
        """Resample a polyline to n_samples uniformly in arc length."""
        if not path:
            return []
        if len(path) == 1 or n_samples <= 1:
            return [path[0]]

        cum = [0.0]
        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))

        total = cum[-1]
        if total <= 1e-9:
            return [path[0]] * n_samples

        targets = [total * i / max(1, n_samples - 1) for i in range(n_samples)]
        out: List[Tuple[float, float]] = []
        seg = 0
        for s_target in targets:
            while seg < len(cum) - 2 and cum[seg + 1] < s_target:
                seg += 1

            s0 = cum[seg]
            s1 = cum[seg + 1]
            x0, y0 = path[seg]
            x1, y1 = path[seg + 1]

            if s1 - s0 <= 1e-9:
                out.append((x0, y0))
            else:
                u = (s_target - s0) / (s1 - s0)
                out.append((x0 + u * (x1 - x0), y0 + u * (y1 - y0)))

        return out

    @staticmethod
    def _polyline_length(path: List[Tuple[float, float]]) -> float:
        if len(path) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            total += math.hypot(x1 - x0, y1 - y0)
        return total

    @staticmethod
    def _truncate_polyline_to_length(path: List[Tuple[float, float]], max_len_m: float) -> List[Tuple[float, float]]:
        """Return the path prefix up to max_len_m arc length."""
        if not path:
            return []
        if len(path) == 1:
            return [path[0]]

        remaining = max(0.0, float(max_len_m))
        if remaining <= 1e-9:
            return [path[0]]

        out: List[Tuple[float, float]] = [path[0]]

        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            seg_len = math.hypot(x1 - x0, y1 - y0)

            if seg_len <= 1e-9:
                out.append((x1, y1))
                continue

            if seg_len <= remaining + 1e-9:
                out.append((x1, y1))
                remaining -= seg_len
                continue

            t = remaining / seg_len
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            break

        return out

    def smooth_path_cubic_spline(self, grid: np.ndarray, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth path using cubic spline interpolation with clamped endpoint tangents.
        Produces smooth, natural-looking curves through waypoints.
        
        Args:
            grid: Costmap grid for collision validation
            path: A* waypoints (already simplified)
        
        Returns:
            Densely sampled smooth path, or original path if smoothing violates hard obstacles
        """
        if len(path) < 3 or not HAS_SCIPY:
            # Not enough points for spline or scipy unavailable - return original path
            return path
        
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        
        # Parameter along path (arc length approximation)
        t = [0.0]
        for i in range(1, len(path)):
            dx = xs[i] - xs[i-1]
            dy = ys[i] - ys[i-1]
            dist = math.hypot(dx, dy)
            t.append(t[-1] + dist)
        
        # Industry-standard clamped boundary spline:
        # constrain endpoint derivatives in arc-length parameter space so the
        # start tangent respects current velocity direction and the end tangent
        # matches the terminal path segment.
        first_seg_tan = self._normalize_vec2(xs[1] - xs[0], ys[1] - ys[0])
        last_seg_tan = self._normalize_vec2(xs[-1] - xs[-2], ys[-1] - ys[-2], fallback=first_seg_tan)

        vx_map_now, vy_map_now = self.odom_vel_map_latest
        speed_now = math.hypot(vx_map_now, vy_map_now)
        if speed_now > 0.05:
            vdir = self._normalize_vec2(vx_map_now, vy_map_now, fallback=first_seg_tan)
            alignment = vdir[0] * first_seg_tan[0] + vdir[1] * first_seg_tan[1]
            # Blend velocity direction with geometric path tangent while
            # preventing a full reverse tangent if the two are opposite.
            max_linear_vel, _, _ = self._effective_motion_limits()
            w = min(0.85, speed_now / max(0.1, max_linear_vel))
            if alignment < -0.25:
                w = min(w, 0.35)
            start_tan = self._normalize_vec2(
                (1.0 - w) * first_seg_tan[0] + w * vdir[0],
                (1.0 - w) * first_seg_tan[1] + w * vdir[1],
                fallback=first_seg_tan,
            )
        else:
            start_tan = first_seg_tan

        bc_x = ((1, start_tan[0]), (1, last_seg_tan[0]))
        bc_y = ((1, start_tan[1]), (1, last_seg_tan[1]))

        # Create cubic splines for x(t) and y(t)
        try:
            spline_x = CubicSpline(t, xs, bc_type=bc_x)
            spline_y = CubicSpline(t, ys, bc_type=bc_y)
        except Exception as e:
            self.get_logger().warn(f"Spline interpolation failed: {e}. Using original path.")
            return path
        
        # Resample smooth path at regular intervals
        sample_spacing = max(0.01, float(self.get_parameter("spline_sample_spacing_m").value))
        n_samples = max(100, int(t[-1] / sample_spacing))
        t_smooth = [t[0] + (t[-1] - t[0]) * i / max(1, n_samples - 1) for i in range(n_samples)]
        
        smooth_path = []
        for t_val in t_smooth:
            x_smooth = float(spline_x(t_val))
            y_smooth = float(spline_y(t_val))
            smooth_path.append((x_smooth, y_smooth))

        max_curvature = float(self.get_parameter("spline_max_curvature").value)
        if max_curvature > 0.0 and len(smooth_path) >= 3:
            blend_step = float(self.get_parameter("spline_curvature_blend_step").value)
            blend_step = min(0.8, max(0.01, blend_step))
            max_iters = int(self.get_parameter("spline_curvature_max_iters").value)
            max_iters = max(1, max_iters)
            reference = self._resample_polyline(path, len(smooth_path))

            tuned_path = list(smooth_path)
            for _ in range(max_iters):
                kappas = self._estimate_path_curvature(tuned_path)
                if max(kappas) <= max_curvature:
                    break
                next_path = [tuned_path[0]]
                for i in range(1, len(tuned_path) - 1):
                    x_cur, y_cur = tuned_path[i]
                    x_ref, y_ref = reference[i]
                    x_new = (1.0 - blend_step) * x_cur + blend_step * x_ref
                    y_new = (1.0 - blend_step) * y_cur + blend_step * y_ref
                    next_path.append((x_new, y_new))
                next_path.append(tuned_path[-1])
                tuned_path = next_path

            smooth_path = tuned_path

        if not self._path_collision_free(grid, smooth_path):
            self.get_logger().warn("Spline smoothing crossed hard obstacle cells; using unsmoothed path.")
            return path
        
        return smooth_path


    def build_velocity_constrained_trajectory(self, path: List[Tuple[float, float]]) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
        """Generate a velocity profile along the path respecting wheel acceleration
        constraints.

        Uses forward-backward passes and curvature-based limits to produce a
        velocity for each point along the provided path.

        Args:
            path: List of (x, y) waypoints from A*

        Returns:
            Tuple of (xs, ys, yaws, velocities, vxs, vys). Empty lists on failure.
        """
        if len(path) < 2:
            return [], [], [], [], [], []

        max_linear_vel, max_wheel_accel, max_wheel_jerk_rs3 = self._effective_motion_limits()
        braking_decel_safety_factor = float(self.get_parameter("braking_decel_safety_factor").value)
        braking_decel_safety_factor = max(0.2, min(1.0, braking_decel_safety_factor))
        max_lateral_accel = float(self.get_parameter("max_lateral_accel").value)
        wheel_radius = max(1e-6, float(self.get_parameter("wheel_radius_m").value))

        profile_decel = max(1e-6, max_wheel_accel * braking_decel_safety_factor)
        profile_jerk = max(1e-6, max_wheel_jerk_rs3 * wheel_radius)

        n = len(path)
        yaw_ref = self.odom_pose_latest.yaw if self.have_odom_pose else 0.0

        # If path is only two points (start and single waypoint), densify
        # the segment so the velocity profiler has intermediate samples to
        # accelerate over. This fixes the case where a 2-point path would
        # otherwise yield velocities=[0,0] (start and end only).
        if n == 2:
            x0, y0 = path[0]
            x1, y1 = path[1]
            total_dist = math.hypot(x1 - x0, y1 - y0)
            # target spacing ~5cm
            spacing = 0.05
            n_samples = max(3, int(math.ceil(total_dist / spacing)) + 1)
            dense = []
            for i in range(n_samples):
                t = i / max(1, n_samples - 1)
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            path = dense
            n = len(path)
        # segment distances and tangents
        dists: List[float] = []
        tangents: List[Tuple[float, float]] = []
        for i in range(n - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            dist = math.hypot(dx, dy)
            dists.append(dist)
            if dist < 1e-6:
                tangents.append((1.0, 0.0))
            else:
                tangents.append((dx / dist, dy / dist))

        # Numerical curvature estimate (central differences) on sampled points
        kappas = [0.0] * n
        if n >= 3:
            for i in range(1, n - 1):
                x_prev, y_prev = path[i - 1]
                x_curr, y_curr = path[i]
                x_next, y_next = path[i + 1]

                ds1 = max(1e-6, math.hypot(x_curr - x_prev, y_curr - y_prev))
                ds2 = max(1e-6, math.hypot(x_next - x_curr, y_next - y_curr))
                ds = 0.5 * (ds1 + ds2)

                dx_dt = (x_next - x_prev) / (2.0 * ds)
                dy_dt = (y_next - y_prev) / (2.0 * ds)
                ddx_dt = (x_next - 2.0 * x_curr + x_prev) / (ds * ds)
                ddy_dt = (y_next - 2.0 * y_curr + y_prev) / (ds * ds)

                denom = (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5
                if denom <= 1e-9:
                    kappas[i] = 0.0
                else:
                    kappas[i] = abs((dx_dt * ddy_dt - dy_dt * ddx_dt) / denom)
            kappas[0] = kappas[1]
            kappas[-1] = kappas[-2]

        # Improved initial velocity profile using cumulative distance
        # This ensures that when the path is a single long segment (one waypoint),
        # the profile will ramp up toward `max_linear_vel` using v = sqrt(2*a*s).
        s = [0.0] * n
        for i in range(1, n):
            s[i] = s[i - 1] + max(1e-9, dists[i - 1])

        total_s = s[-1]

        # Seed with current velocity component along initial path tangent.
        # This avoids over-estimating path-speed when the robot is currently
        # moving in a different direction.
        vx_map_now, vy_map_now = self.odom_vel_map_latest
        if tangents:
            tx0, ty0 = tangents[0]
            v_initial = max(0.0, vx_map_now * tx0 + vy_map_now * ty0)
        else:
            v_initial = math.hypot(vx_map_now, vy_map_now)
        v_initial = min(v_initial, max_linear_vel)

        v_forward = [0.0] * n
        for i in range(n):
            # from current speed: v = sqrt(v0^2 + 2 * a * distance)
            v_f = math.sqrt(max(0.0, v_initial * v_initial + 2.0 * max_wheel_accel * s[i]))
            v_forward[i] = min(v_f, max_linear_vel)

        v_backward = [0.0] * n
        for i in range(n - 1, -1, -1):
            dist_to_end = max(0.0, total_s - s[i])
            v_b = self._max_speed_for_jerk_limited_stop(dist_to_end, profile_decel, profile_jerk)
            v_backward[i] = min(v_b, max_linear_vel)

        velocities = [min(v_forward[i], v_backward[i]) for i in range(n)]

        # Apply curvature-based speed limit
        for i in range(n):
            k = kappas[i]
            if k > 1e-9:
                v_curv = math.sqrt(max(1e-6, max_lateral_accel / k))
                velocities[i] = min(velocities[i], v_curv)

        # Helper: wheel linear coefficients for given tangent (tx,ty)
        def wheel_coeffs_for_tangent(tx: float, ty: float) -> List[float]:
            tx_body, ty_body = self._map_velocity_to_body(tx, ty, yaw_ref)
            # v_w1 = vy
            # v_w2 = -0.5*vy + sqrt(3)/2*vx
            # v_w3 = -0.5*vy - sqrt(3)/2*vx
            s1 = ty_body
            s2 = (math.sqrt(3) / 2.0) * tx_body - 0.5 * ty_body
            s3 = -(math.sqrt(3) / 2.0) * tx_body - 0.5 * ty_body
            return [s1, s2, s3]

        # Iteratively enforce per-wheel acceleration limits along path
        # Perform a few passes of forward/backward limiting
        for _pass in range(3):
            # Forward (acceleration) pass
            for i in range(n - 1):
                v_i = velocities[i]
                v_j = velocities[i + 1]
                dist = max(1e-6, dists[i])
                # tangent for segment i
                tx, ty = tangents[i]
                coeffs = wheel_coeffs_for_tangent(tx, ty)
                max_coeff = max(abs(c) for c in coeffs) if coeffs else 0.0
                if max_coeff < 1e-9:
                    continue
                # estimate dt using average speed
                avg_speed = max(1e-3, 0.5 * (v_i + v_j))
                dt = dist / avg_speed
                allowed_delta = max_wheel_accel * dt / max_coeff
                if v_j > v_i + allowed_delta:
                    velocities[i + 1] = v_i + allowed_delta

            # Backward (deceleration) pass
            for i in range(n - 2, -1, -1):
                v_i = velocities[i]
                v_j = velocities[i + 1]
                dist = max(1e-6, dists[i])
                tx, ty = tangents[i]
                coeffs = wheel_coeffs_for_tangent(tx, ty)
                max_coeff = max(abs(c) for c in coeffs) if coeffs else 0.0
                if max_coeff < 1e-9:
                    continue
                avg_speed = max(1e-3, 0.5 * (v_i + v_j))
                dt = dist / avg_speed
                allowed_delta = profile_decel * dt / max_coeff
                if v_i > v_j + allowed_delta:
                    velocities[i] = v_j + allowed_delta

        # Final clamp to max linear velocity and non-negative
        for i in range(n):
            velocities[i] = max(0.0, min(velocities[i], max_linear_vel))

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        yaws = [yaw_ref] * n
        
        # Resample trajectory at 0.01s timesteps (100Hz) to match controller dt
        # Matching dt prevents aliasing and acceleration discontinuities
        xs_resampled, ys_resampled, yaws_resampled, vels_resampled, vxs_resampled, vys_resampled = self._resample_trajectory_at_dt(
            xs, ys, yaws, velocities, dists, dt=0.01
        )

        # Yaw stays tied to BNO odometry heading for this robot revision.
        yaws_resampled = [yaw_ref] * len(xs_resampled)

        # Preserve STM32-reported body velocity at the first knot, converted to map frame.
        if xs_resampled:
            vx_body_start, vy_body_start = self.odom_vel_body_latest
            speed_start = math.hypot(vx_body_start, vy_body_start)
            if speed_start > max_linear_vel and speed_start > 1e-9:
                scale = max_linear_vel / speed_start
                vx_body_start *= scale
                vy_body_start *= scale

            yaw_start = self.odom_pose_latest.yaw if self.have_odom_pose else 0.0
            c0 = math.cos(yaw_start)
            s0 = math.sin(yaw_start)
            vxs_resampled[0] = c0 * vx_body_start - s0 * vy_body_start
            vys_resampled[0] = s0 * vx_body_start + c0 * vy_body_start
            vels_resampled[0] = math.hypot(vx_body_start, vy_body_start)

        # First, enforce tangential/normal acceleration limits on the velocity
        # vector sequence so direction changes stay dynamically feasible.
        vxs_resampled, vys_resampled, vels_resampled = self._enforce_tangent_normal_limits_on_velocity_series(
            vxs_resampled,
            vys_resampled,
            0.01,
        )

        # Then re-apply per-wheel accel constraints after spline/resampling and
        # first-knot anchoring to guarantee feasible knot-to-knot wheel updates.
        vxs_resampled, vys_resampled, vels_resampled = self._enforce_wheel_accel_limits_on_velocity_series(
            vxs_resampled,
            vys_resampled,
            0.01,
            yaws=yaws_resampled,
            yaw_fallback=yaw_ref,
        )
        
        return xs_resampled, ys_resampled, yaws_resampled, vels_resampled, vxs_resampled, vys_resampled

    def _enforce_tangent_normal_limits_on_velocity_series(
        self,
        vxs: List[float],
        vys: List[float],
        dt: float,
    ) -> Tuple[List[float], List[float], List[float]]:
        """Limit per-step velocity-vector change using tangential/normal accel caps.

        This follows a robust, standard retiming pattern in mobile robotics:
        decompose acceleration into tangential and normal components and clamp
        each independently to keep turn transitions trackable.
        """
        n = min(len(vxs), len(vys))
        if n == 0:
            return [], [], []

        dt_eff = max(1e-4, dt)
        max_linear_vel, max_tan_accel, _ = self._effective_motion_limits()
        max_tan_accel = max(1e-6, max_tan_accel)
        max_norm_accel = max(1e-6, float(self.get_parameter("max_lateral_accel").value))
        max_linear_vel = max(1e-6, max_linear_vel)

        out_vx = list(vxs)
        out_vy = list(vys)

        def clamp_speed(i: int) -> None:
            spd = math.hypot(out_vx[i], out_vy[i])
            if spd > max_linear_vel and spd > 1e-9:
                scale = max_linear_vel / spd
                out_vx[i] *= scale
                out_vy[i] *= scale

        clamp_speed(0)

        for i in range(1, n):
            prev_vx = out_vx[i - 1]
            prev_vy = out_vy[i - 1]
            des_vx = out_vx[i]
            des_vy = out_vy[i]

            prev_speed = math.hypot(prev_vx, prev_vy)
            if prev_speed <= 1e-4:
                dvx = des_vx - prev_vx
                dvy = des_vy - prev_vy
                dv_mag = math.hypot(dvx, dvy)
                max_dv = max_tan_accel * dt_eff
                if dv_mag > max_dv and dv_mag > 1e-9:
                    scale = max_dv / dv_mag
                    dvx *= scale
                    dvy *= scale
                out_vx[i] = prev_vx + dvx
                out_vy[i] = prev_vy + dvy
            else:
                ux = prev_vx / prev_speed
                uy = prev_vy / prev_speed
                nx = -uy
                ny = ux

                a_req_x = (des_vx - prev_vx) / dt_eff
                a_req_y = (des_vy - prev_vy) / dt_eff

                a_t = a_req_x * ux + a_req_y * uy
                a_n = a_req_x * nx + a_req_y * ny

                a_t = max(-max_tan_accel, min(max_tan_accel, a_t))
                a_n = max(-max_norm_accel, min(max_norm_accel, a_n))

                out_vx[i] = prev_vx + dt_eff * (a_t * ux + a_n * nx)
                out_vy[i] = prev_vy + dt_eff * (a_t * uy + a_n * ny)

            clamp_speed(i)

        out_vels = [math.hypot(out_vx[i], out_vy[i]) for i in range(n)]
        return out_vx, out_vy, out_vels

    def _resample_trajectory_at_dt(
        self, 
        xs: List[float], 
        ys: List[float], 
        yaws: List[float], 
        velocities: List[float],
        dists: List[float],
        dt: float = 0.01
    ) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
        """
        Resample trajectory at fixed time intervals (dt) for controller tracking.
        
        Uses cubic spline interpolation for smooth velocity profiles.
        
        Args:
            xs, ys: Position coordinates of path points
            yaws: Yaw angles at path points
            velocities: Velocity at each path point
            dists: Distances between consecutive path points
            dt: Time step for resampling (default 0.01s = 100Hz)
            
        Returns:
            Tuple of (xs, ys, yaws, velocities, vxs, vys) resampled at dt intervals
        """
        if len(xs) < 2:
            return xs, ys, yaws, velocities, [0.0]*len(xs), [0.0]*len(xs)
            
        n = len(xs)
        
        # Build cumulative distance array
        s = [0.0]
        for i in range(len(dists)):
            s.append(s[-1] + max(1e-9, dists[i]))
        
        # Build cumulative time array by integrating inverse velocity
        t = [0.0]
        for i in range(n - 1):
            v_avg = max(1e-3, 0.5 * (velocities[i] + velocities[i + 1]))
            dist = s[i + 1] - s[i]
            dt_segment = dist / v_avg
            t.append(t[-1] + dt_segment)
        
        total_time = t[-1]
        
        # Use cubic spline interpolation if scipy available, otherwise linear
        if HAS_SCIPY and n >= 4:
            try:
                # Create cubic splines for smooth interpolation
                spline_x = CubicSpline(t, xs, bc_type='not-a-knot')
                spline_y = CubicSpline(t, ys, bc_type='not-a-knot')
                spline_v = CubicSpline(t, velocities, bc_type='not-a-knot')
                use_spline = True
            except:
                use_spline = False
        else:
            use_spline = False
        
        # Generate resampled trajectory at fixed dt intervals
        xs_new = []
        ys_new = []
        yaws_new = []
        vels_new = []
        vxs_new = []
        vys_new = []
        
        current_time = 0.0
        
        while current_time <= total_time:
            if use_spline:
                # Cubic spline interpolation (smooth derivatives)
                x_interp = float(spline_x(current_time))
                y_interp = float(spline_y(current_time))
                v_interp = float(spline_v(current_time))
                
                # Compute velocity direction from spline derivatives
                dx_dt = float(spline_x.derivative()(current_time))
                dy_dt = float(spline_y.derivative()(current_time))
                speed = math.hypot(dx_dt, dy_dt)
                
                if speed > 1e-6:
                    vx_interp = dx_dt * (v_interp / speed)
                    vy_interp = dy_dt * (v_interp / speed)
                else:
                    vx_interp = 0.0
                    vy_interp = 0.0
                    
                yaw_interp = yaws[0]  # Keep constant for omnidirectional
                
            else:
                # Linear interpolation fallback
                segment_idx = 0
                while segment_idx < n - 1 and current_time > t[segment_idx + 1]:
                    segment_idx += 1
                
                if segment_idx >= n - 1:
                    break
                
                t0 = t[segment_idx]
                t1 = t[segment_idx + 1]
                alpha = (current_time - t0) / max(1e-9, t1 - t0)
                alpha = max(0.0, min(1.0, alpha))
                
                x_interp = xs[segment_idx] + alpha * (xs[segment_idx + 1] - xs[segment_idx])
                y_interp = ys[segment_idx] + alpha * (ys[segment_idx + 1] - ys[segment_idx])
                v_interp = velocities[segment_idx] + alpha * (velocities[segment_idx + 1] - velocities[segment_idx])
                yaw_interp = yaws[segment_idx] + alpha * wrap_to_pi(yaws[segment_idx + 1] - yaws[segment_idx])
                
                dx = xs[segment_idx + 1] - xs[segment_idx]
                dy = ys[segment_idx + 1] - ys[segment_idx]
                path_dist = math.hypot(dx, dy)
                if path_dist > 1e-9:
                    vx_interp = v_interp * (dx / path_dist)
                    vy_interp = v_interp * (dy / path_dist)
                else:
                    vx_interp = 0.0
                    vy_interp = 0.0
            
            xs_new.append(x_interp)
            ys_new.append(y_interp)
            yaws_new.append(yaw_interp)
            vels_new.append(v_interp)
            vxs_new.append(vx_interp)
            vys_new.append(vy_interp)
            
            current_time += dt
        
        # Always include the final point
        if len(xs_new) == 0 or (xs_new[-1] != xs[-1] or ys_new[-1] != ys[-1]):
            xs_new.append(xs[-1])
            ys_new.append(ys[-1])
            yaws_new.append(yaws[-1] if yaws else 0.0)
            vels_new.append(0.0)
            vxs_new.append(0.0)
            vys_new.append(0.0)
        
        # Apply gentle smoothing only to remove high-frequency noise
        # 3-point moving average (lighter than before since spline is already smooth)
        if len(vxs_new) >= 3:
            vxs_smoothed = [vxs_new[0]]
            vys_smoothed = [vys_new[0]]
            vels_smoothed = [vels_new[0]]
            for i in range(1, len(vxs_new) - 1):
                vxs_smoothed.append((vxs_new[i-1] + vxs_new[i] + vxs_new[i+1]) / 3.0)
                vys_smoothed.append((vys_new[i-1] + vys_new[i] + vys_new[i+1]) / 3.0)
                vels_smoothed.append((vels_new[i-1] + vels_new[i] + vels_new[i+1]) / 3.0)
            vxs_smoothed.append(vxs_new[-1])
            vys_smoothed.append(vys_new[-1])
            vels_smoothed.append(vels_new[-1])
            vxs_new = vxs_smoothed
            vys_new = vys_smoothed
            vels_new = vels_smoothed
        
        self.get_logger().info(
            f"Resampled trajectory: {len(xs)} path points -> {len(xs_new)} time points "
            f"at dt={dt}s (total time: {total_time:.2f}s)"
        )
        
        return xs_new, ys_new, yaws_new, vels_new, vxs_new, vys_new

    # =======================
    # Publishing
    # =======================
    def _safe_publish(self, pub, msg, channel: str) -> None:
        try:
            pub.publish(msg)
        except Exception as exc:
            text = str(exc).lower()
            if "context is invalid" in text or "publisher's context is invalid" in text:
                self.get_logger().warn(f"Skipping publish on {channel}: ROS context is shutting down")
                return
            self.get_logger().error(f"Publish failed on {channel}: {exc}")

    def _publish_grid(self, pub, frame_id: str, grid: np.ndarray, gs: Optional[GridSpec] = None) -> None:
        if gs is None:
            gs = self.gs_map

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.info.resolution = gs.res
        msg.info.width = gs.width
        msg.info.height = gs.height
        msg.info.origin.position.x = gs.origin_x
        msg.info.origin.position.y = gs.origin_y
        msg.info.origin.position.z = 0.0
        q = yaw_to_quat(0.0)
        msg.info.origin.orientation.x = q[0]
        msg.info.origin.orientation.y = q[1]
        msg.info.origin.orientation.z = q[2]
        msg.info.origin.orientation.w = q[3]
        # Flatten NumPy array to list for ROS message (row-major order)
        msg.data = grid.flatten().astype(np.int8).tolist()
        self._safe_publish(pub, msg, "grid")

    def _publish_map_if_needed(self, frame_id: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._latest_slam_map is not None and (now_ns - self._latest_slam_map_ns) < int(3.0 * 1e9):
            msg = self._latest_slam_map
            map_crc = zlib.crc32(np.asarray(msg.data, dtype=np.int8).tobytes())
            unchanged = (self._last_published_map_crc == map_crc)
            if unchanged and (now_ns - self._last_map_publish_ns) < self._map_republish_period_ns:
                return
            self._safe_publish(self.map_pub, msg, "map")
            self._last_published_map_crc = map_crc
            self._last_map_publish_ns = now_ns
            return

        map_grid = self.static_occ
        map_gs = self.gs_map

        if self.mapping_active and self.mapping_grid_spec is not None and self.mapping_logodds is not None:
            map_grid = self._mapping_occ_grid()
            map_gs = self.mapping_grid_spec

        map_crc = zlib.crc32(map_grid.tobytes())
        unchanged = (self._last_published_map_crc == map_crc)
        if unchanged and (now_ns - self._last_map_publish_ns) < self._map_republish_period_ns:
            return

        self._publish_grid(self.map_pub, frame_id, map_grid, gs=map_gs)
        self._last_published_map_crc = map_crc
        self._last_map_publish_ns = now_ns

    def _has_fresh_slam_map(self, now_ns: Optional[int] = None, max_age_s: float = 3.0) -> bool:
        if self._latest_slam_map is None:
            return False
        if now_ns is None:
            now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self._latest_slam_map_ns) < int(max_age_s * 1e9)

    def on_slam_map(self, msg: OccupancyGrid) -> None:
        self._latest_slam_map = msg
        self._latest_slam_map_ns = self.get_clock().now().nanoseconds
        if self.mapping_active:
            self._project_slam_map_to_static(msg)

    def publish_waypoints(self, wps: List[Tuple[float, float]]) -> None:
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.get_parameter("map_frame").value
        m.ns = "wps"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.10
        m.scale.y = 0.10
        m.scale.z = 0.10
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        for (x, y) in wps:
            m.points.append(Point(x=float(x), y=float(y), z=0.05))
        ma.markers.append(m)
        self._safe_publish(self.wp_marker_pub, ma, "waypoint_markers")

    def publish_velocity_markers(self, xs: List[float], ys: List[float], velocities: List[float]) -> None:
        """Publish velocity as colored line strip along the path."""
        ma = MarkerArray()
        
        if len(xs) < 2 or len(velocities) != len(xs):
            self._safe_publish(self.velocity_marker_pub, ma, "velocity_markers")
            return
        
        # Create LINE_STRIP marker with thicker width
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.get_parameter("map_frame").value
        m.ns = "path_velocity"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.1  # Thicker line
        
        # Find max velocity for normalization
        vmax = max(velocities) if velocities else 1e-6
        vmax = max(vmax, 1e-6)
        
        # Add all points with gradient coloring based on actual velocity
        for i in range(len(xs)):
            x, y = xs[i], ys[i]
            m.points.append(Point(x=float(x), y=float(y), z=0.02))
            
            # Use actual velocity from trajectory
            v = velocities[i]
            
            # Normalize velocity to [0, 1]
            t = float(v) / float(vmax)
            t = max(0.0, min(1.0, t))
            
            # Color gradient: blue (slow/stopped) to red (fast)
            c = ColorRGBA(r=t, g=0.0, b=1.0 - t, a=1.0)
            m.colors.append(c)
        
        ma.markers.append(m)
        self._safe_publish(self.velocity_marker_pub, ma, "velocity_markers")

    def _publish_trajectory(
        self,
        frame: str,
        xs: List[float],
        ys: List[float],
        yaws: List[float],
        velocities: List[float],
        vxs: Optional[List[float]] = None,
        vys: Optional[List[float]] = None,
    ) -> None:
        """Publish a trajectory path and corresponding velocity markers."""
        if not xs:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame
        for i in range(len(xs)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(xs[i])
            ps.pose.position.y = float(ys[i])
            ps.pose.position.z = 0.0
            q = yaw_to_quat(yaws[i])
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            path.poses.append(ps)
        self._safe_publish(self.path_pub, path, "planned_path")

        # Publish velocities for UDP trajectory server
        if vxs is not None and vys is not None:
            vel_msg = Float64MultiArray()
            # Interleave vx and vy: [vx0, vy0, vx1, vy1, ...]
            vel_msg.data = []
            for i in range(min(len(vxs), len(vys))):
                vel_msg.data.append(float(vxs[i]))
                vel_msg.data.append(float(vys[i]))
            self._safe_publish(self.path_velocities_pub, vel_msg, "planned_path_velocities")

        self.publish_velocity_markers(xs, ys, velocities)

    def _publish_empty_trajectory(self, frame: str) -> None:
        """Explicitly clear path and velocity topics so downstream bridges stop output."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame
        self._safe_publish(self.path_pub, path, "planned_path")

        vel_msg = Float64MultiArray()
        vel_msg.data = []
        self._safe_publish(self.path_velocities_pub, vel_msg, "planned_path_velocities")

        # Clear RViz velocity visualization too.
        self._safe_publish(self.velocity_marker_pub, MarkerArray(), "velocity_markers")

    def _publish_hold_current_trajectory(self, frame: str, pose: Pose2D) -> None:
        """Publish a single-point hold trajectory at current pose with zero velocity."""
        self._publish_trajectory(
            frame,
            [float(pose.x)],
            [float(pose.y)],
            [float(pose.yaw)],
            [0.0],
            [0.0],
            [0.0],
        )

    def _publish_geofence_visualization(self) -> None:
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("map_frame").value

        outline = Marker()
        outline.header.stamp = stamp
        outline.header.frame_id = frame
        outline.ns = "geofence"
        outline.id = 0

        anchor = Marker()
        anchor.header.stamp = stamp
        anchor.header.frame_id = frame
        anchor.ns = "geofence"
        anchor.id = 1

        rect = self.active_geofence_rect
        if rect is None:
            outline.action = Marker.DELETE
            anchor.action = Marker.DELETE
            ma.markers.append(outline)
            ma.markers.append(anchor)
            self._safe_publish(self.geofence_marker_pub, ma, "geofence_markers")
            return

        corners = self._rect_corners(rect)

        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.scale.x = 0.04
        outline.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.95)
        outline.points = [Point(x=float(x), y=float(y), z=0.03) for (x, y) in (corners + [corners[0]])]
        ma.markers.append(outline)

        if self.geofence_anchor_corner is not None:
            ax, ay = self.geofence_anchor_corner
            anchor.type = Marker.SPHERE
            anchor.action = Marker.ADD
            anchor.scale.x = 0.12
            anchor.scale.y = 0.12
            anchor.scale.z = 0.12
            anchor.pose.position.x = float(ax)
            anchor.pose.position.y = float(ay)
            anchor.pose.position.z = 0.06
            anchor.pose.orientation.w = 1.0
            anchor.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.95)
        else:
            anchor.action = Marker.DELETE
        ma.markers.append(anchor)

        self._safe_publish(self.geofence_marker_pub, ma, "geofence_markers")

    def _waypoint_signature(self, wps: List[Tuple[float, float]]) -> Tuple[Tuple[int, int], ...]:
        """Grid-quantized signature of waypoints for replan-change detection."""
        res = max(self.gs_map.res, 1e-3)
        return tuple((int(round(x / res)), int(round(y / res))) for (x, y) in wps)

    def _should_replan_trajectory(self, base_pose_now: Pose2D, wps: List[Tuple[float, float]]) -> bool:
        """Decide whether heavy planning should run this cycle."""
        if not wps:
            return False

        if self._cached_traj is None or self._last_plan_wps_sig is None:
            return True

        wps_sig = self._waypoint_signature(wps)
        if wps_sig != self._last_plan_wps_sig:
            return True

        if self._last_plan_pose is not None:
            dx = base_pose_now.x - self._last_plan_pose.x
            dy = base_pose_now.y - self._last_plan_pose.y
            moved = math.hypot(dx, dy)
            yaw_delta = abs(wrap_to_pi(base_pose_now.yaw - self._last_plan_pose.yaw))
            if moved >= float(self.get_parameter("trajectory_replan_min_move_m").value):
                return True
            if yaw_delta >= float(self.get_parameter("trajectory_replan_min_yaw_rad").value):
                return True

        replan_hz = max(0.05, float(self.get_parameter("trajectory_replan_hz").value))
        period_ns = int(1e9 / replan_hz)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_plan_ns >= period_ns:
            return True

        return False
    
    def save_trajectory_json(self, xs: List[float], ys: List[float], yaws: List[float], velocities: List[float], vxs: List[float], vys: List[float]) -> None:
        """Save the trajectory to a JSON file."""
        try:
            output_path = "/home/nickolas/Desktop/last_trajectory.json"
            
            trajectory_data = {
                "timestamp": self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
                "dt": 0.01,  # Timestep in seconds (100Hz) - matches controller
                "points": []
            }
            
            for i in range(len(xs)):
                point = {
                    "x": float(xs[i]),
                    "y": float(ys[i]),
                    "yaw": float(yaws[i]),
                    "velocity": float(velocities[i]),
                    "vx": float(vxs[i]),
                    "vy": float(vys[i])
                }
                trajectory_data["points"].append(point)
            
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with open(output_path, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            
            self.get_logger().info(f"Trajectory saved to {output_path} ({len(xs)} points at dt=0.01s)")
        except Exception as e:
            self.get_logger().error(f"Failed to save trajectory: {e}")
    
    
    # =======================
    # Main timer
    # =======================
    def on_timer(self) -> None:
        timer_start_ns = self.get_clock().now().nanoseconds
        now_ns = timer_start_ns
        self.consume_add_wp()

        publish_stamp_msg = self._stable_publish_stamp()

        # base pose now (in odom/map_frame)
        base_pose_now = self._base_pose_now()
        self._ensure_geofence(base_pose_now)

        # Keep a rolling bounded map centered around robot while preserving in-window memory
        # Disabled whenever a saved map is loaded so AMCL scan-matching sees a stable /map.
        if self.saved_map_bounds is None:
            self._maybe_roll_map(base_pose_now)

        # remove reached waypoints
        self._pop_reached_waypoints(base_pose_now)
        self._refill_auto_test_waypoint_window()

        # Always publish odom->base_link at timer cadence with a stable stamp so
        # consumers (e.g., slam_toolbox filters) always have a transform at/near
        # outgoing scan timestamps even when odom input cadence jitters.
        self._publish_odom_to_base_tf(stamp=publish_stamp_msg)

        # publish robot visualization (footprint circle + orientation arrow)
        self._publish_robot_visualization(base_pose_now)
        self._publish_geofence_visualization()

        # build + publish fused scan (for RViz) and costmap from it
        fused = self._build_fused_scan(base_pose_now, publish_stamp_msg)
        if fused is not None:
            self._safe_publish(self.scan_fused_pub, fused, "scan_fused")
            if bool(self.get_parameter("publish_scan_match").value):
                scan_match = self._build_scan_from_scan(
                    fused,
                    float(self.get_parameter("scan_match_angle_increment_deg").value),
                )
                if scan_match is not None:
                    if bool(self.get_parameter("scan_match_use_latest_tf_stamp").value):
                        if not self._warned_scan_match_latest_tf_stamp:
                            self._warned_scan_match_latest_tf_stamp = True
                            self.get_logger().warn(
                                "scan_match_use_latest_tf_stamp is deprecated and ignored; "
                                "scan_match uses synchronized stable publish stamps."
                            )
                    self._safe_publish(self.scan_match_pub, scan_match, "scan_match")
            if self._latest_fused_points_base and self.points_fused_pub.get_subscription_count() > 0:
                points_msg = self._build_points_cloud_msg(
                    self._latest_fused_points_base,
                    publish_stamp_msg,
                    fused.header.frame_id,
                )
                self._safe_publish(self.points_fused_pub, points_msg, "points_fused")
        elif self._last_scan_rx_ns > 0:
            age_s = (self.get_clock().now().nanoseconds - self._last_scan_rx_ns) * 1e-9
            if age_s > max(1.0, float(self.get_parameter("scan_max_age_s").value) * 2.0):
                self._warn_lidar_throttled(
                    f"No fresh LiDAR messages for {age_s:.1f}s; publishing empty local costmap until scans recover."
                )

        frame = self.get_parameter("map_frame").value

        # /map is transient-local; only republish when contents change so AMCL
        # does not rebuild on every main-loop tick.
        self._publish_map_if_needed(frame)

        # publish waypoints even if no scan/costmap yet
        wps = self.read_waypoints()
        if wps:
            wps, wps_clamped = self._clamp_waypoints_to_geofence(wps)
            if wps_clamped:
                stride = 2
                wp_flat: List[float] = []
                for wx, wy in wps:
                    wp_flat.extend([float(wx), float(wy)])
                self.set_parameters(
                    [
                        Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, wp_flat),
                        Parameter("wp_n", Parameter.Type.INTEGER, len(wps)),
                    ]
                )
            self.publish_waypoints(wps)

        if fused is None:
            # No fresh live scans: publish empty live costmap (plus geofence),
            # never fall back to saved/static occupancy for /costmap.
            local_gs = self._make_local_costmap_spec(base_pose_now)
            empty_costmap = np.zeros((local_gs.height, local_gs.width), dtype=np.int8)
            self._apply_geofence_to_costmap(empty_costmap, local_gs)
            if self.costmap_pub is not None:
                self._publish_grid(self.costmap_pub, frame, empty_costmap, gs=local_gs)
            self._finish_timer_cycle(timer_start_ns)
            return

        local_gs = self._make_local_costmap_spec(base_pose_now)
        local_costmap = self._build_local_costmap_from_points(self._latest_fused_points_base, base_pose_now, local_gs)
        self._apply_geofence_to_costmap(local_costmap, local_gs)
        if self.costmap_pub is not None:
            self._publish_grid(self.costmap_pub, frame, local_costmap, gs=local_gs)

        if not self._pose_inside_geofence(
            base_pose_now,
            margin_m=max(0.0, float(self.get_parameter("geofence_stop_margin_m").value)),
        ):
            self.get_logger().warn(
                f"Robot pose ({base_pose_now.x:.2f}, {base_pose_now.y:.2f}) outside geofence. Holding trajectory."
            )
            self._cached_traj = None
            self._last_plan_wps_sig = None
            self._publish_hold_current_trajectory(frame, base_pose_now)
            self._finish_timer_cycle(timer_start_ns)
            return

        # plan if waypoints exist
        if not wps:
            if self.mapping_active and not self._has_fresh_slam_map(now_ns):
                self._update_mapping_logodds_from_scan_pairs(self._latest_scan_pairs, base_pose_now)
            self._cached_traj = None
            self._last_plan_wps_sig = None
            self._publish_hold_current_trajectory(frame, base_pose_now)
            self._finish_timer_cycle(timer_start_ns)
            return

        if self.mapping_active and not self._has_fresh_slam_map(now_ns):
            self._update_mapping_logodds_from_scan_pairs(self._latest_scan_pairs, base_pose_now)

        planning_costmap = self._build_planning_costmap_from_local(local_costmap, local_gs)

        # Relocate stuck waypoints for all modes, including wp t auto-test points,
        # so generated goals inside hard obstacles are shifted to traversable cells.
        moved_wps = self._relocate_stuck_waypoints(planning_costmap, base_pose_now)
        if moved_wps is not None:
            wps = moved_wps
            self.publish_waypoints(wps)

        if self._effective_scan_stride > 1 and self._cached_traj is not None:
            # Under load-shed, reuse validated cached trajectory.
            # If there is no cache yet, continue below and plan once to seed it.
            self._publish_cached_if_valid(frame, base_pose_now, planning_costmap)
            self._finish_timer_cycle(timer_start_ns)
            return
        if self._effective_scan_stride > 1 and self._cached_traj is None:
            self._warn_lidar_throttled(
                "Load-shed active with no cached trajectory; forcing replan seed this cycle."
            )

        should_replan = self._should_replan_trajectory(base_pose_now, wps)
        if not should_replan and self._cached_traj is not None:
            self._publish_cached_if_valid(frame, base_pose_now, planning_costmap)
            self._finish_timer_cycle(timer_start_ns)
            return

        traj_horizon_m = max(0.20, float(self.get_parameter("trajectory_horizon_m").value))

        stitched: List[Tuple[float, float]] = []
        start_xy = (base_pose_now.x, base_pose_now.y)

        for (gx, gy) in wps:
            seg = self.plan_segment_path(planning_costmap, start_xy, (gx, gy))
            if not seg:
                self.get_logger().warn(f"Planning failed start={start_xy} goal={(gx, gy)}")
                if not self._publish_cached_if_valid(frame, base_pose_now, planning_costmap):
                    self._publish_hold_current_trajectory(frame, base_pose_now)
                self._finish_timer_cycle(timer_start_ns)
                return
            if not stitched:
                stitched.extend(seg)
            else:
                stitched.extend(seg[1:])

            if self._polyline_length(stitched) >= traj_horizon_m:
                stitched = self._truncate_polyline_to_length(stitched, traj_horizon_m)
                break

            start_xy = (gx, gy)

        if len(stitched) < 2:
            if not self._publish_cached_if_valid(frame, base_pose_now, planning_costmap):
                self._publish_hold_current_trajectory(frame, base_pose_now)
            self._finish_timer_cycle(timer_start_ns)
            return

        # Smooth path using cubic spline interpolation (industry standard)
        stitched_smooth = self.smooth_path_cubic_spline(planning_costmap, stitched)
        stitched_smooth = self._truncate_polyline_to_length(stitched_smooth, traj_horizon_m)

        # Build velocity-constrained trajectory on smoothed path
        xs, ys, yaws, velocities, vxs, vys = self.build_velocity_constrained_trajectory(stitched_smooth)
        
        if not xs:
            if not self._publish_cached_if_valid(frame, base_pose_now, planning_costmap):
                self._publish_hold_current_trajectory(frame, base_pose_now)
            self._finish_timer_cycle(timer_start_ns)
            return

        if not self._trajectory_swept_collision_free(planning_costmap, xs, ys):
            self.get_logger().warn("New smoothed trajectory failed swept check; retrying unsmoothed path.")

            xs_u, ys_u, yaws_u, vel_u, vxs_u, vys_u = self.build_velocity_constrained_trajectory(stitched)
            if xs_u and self._trajectory_swept_collision_free(planning_costmap, xs_u, ys_u):
                xs, ys, yaws, velocities, vxs, vys = xs_u, ys_u, yaws_u, vel_u, vxs_u, vys_u
            elif xs_u and self._publish_trimmed_safe_prefix(
                frame,
                base_pose_now,
                planning_costmap,
                (xs_u, ys_u, yaws_u, vel_u, vxs_u, vys_u),
                "New",
            ):
                self._last_plan_ns = self.get_clock().now().nanoseconds
                self._last_plan_pose = Pose2D(base_pose_now.x, base_pose_now.y, base_pose_now.yaw)
                self._last_plan_wps_sig = self._waypoint_signature(wps)
                self._finish_timer_cycle(timer_start_ns)
                return
            else:
                if not self._publish_cached_if_valid(frame, base_pose_now, planning_costmap):
                    self._cached_traj = None
                    self._cached_traj_collision_iters = 0
                    self._last_plan_wps_sig = None
                    self._publish_hold_current_trajectory(frame, base_pose_now)
                self._finish_timer_cycle(timer_start_ns)
                return

        self._cached_traj = (xs, ys, yaws, velocities, vxs, vys)
        self._cached_traj_collision_iters = 0
        self._last_plan_ns = self.get_clock().now().nanoseconds
        self._last_plan_pose = Pose2D(base_pose_now.x, base_pose_now.y, base_pose_now.yaw)
        self._last_plan_wps_sig = self._waypoint_signature(wps)
        
        # Save trajectory to JSON file
        # self.save_trajectory_json(xs, ys, yaws, velocities, vxs, vys)

        self._publish_trajectory(frame, xs, ys, yaws, velocities, vxs, vys)
        self._finish_timer_cycle(timer_start_ns)


def main() -> None:
    rclpy.init()
    node = WaypointTrajNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

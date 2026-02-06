#!/usr/bin/env python3
# file: omni_traj/waypoint_traj_node.py

from __future__ import annotations

import base64
import heapq
import json
import math
import os
import time
import zlib
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridSpec:
    res: float
    width: int
    height: int
    origin_x: float
    origin_y: float


class WaypointTrajNode(Node):
    """
    Dual-LiDAR fused costmap builder.

    Key fixes:
    - Proper inflation (hard inflation works even when soft=0)
    - Publish a fused LaserScan in base_link: /scan_fused
    - Build costmap from fused scan so it doesn't look "double"
    - Clear robot footprint circle (0.22m radius) around base_link center
    - Pop reached waypoints within 0.10m
    """

    def __init__(self) -> None:
        super().__init__("waypoint_traj")

        # ===== Frames =====
        self.declare_parameter("map_frame", "odom")      # publish costmap in odom so RViz fixed frame "odom" works
        self.declare_parameter("base_frame", "base_link")

        # If YOU already publish odom->base_link elsewhere, keep this false
        self.declare_parameter("publish_odom_to_base_tf", True)

        # ===== Robot exclusion =====
        self.declare_parameter("robot_exclusion_enable", True)
        self.declare_parameter("robot_exclusion_radius_m", 0.22)  # diameter 0.44m

        # ===== Waypoint removal =====
        self.declare_parameter("waypoint_reached_tol_m", 0.10)

        # ===== Global grid =====
        self.declare_parameter("global_map_res", 0.5)  # 1cm resolution
        self.declare_parameter("global_map_width_m", 10.0)
        self.declare_parameter("global_map_height_m", 10.0)

        # ===== Scans =====
        self.declare_parameter("lidar1_topic", "/lidar1/scan")
        self.declare_parameter("lidar2_topic", "/lidar2/scan")
        self.declare_parameter("scan_max_age_s", 0.5)
        self.declare_parameter("scan_beam_stride", 1)
        self.declare_parameter("scan_no_hit_eps_m", 1e-3)

        # ===== Fused scan output =====
        self.declare_parameter("publish_fused_scan", True)
        self.declare_parameter("fused_angle_min", -math.pi)
        self.declare_parameter("fused_angle_max", math.pi)
        self.declare_parameter("fused_angle_increment_deg", 0.25)  # 1 degree bins
        self.declare_parameter("motion_compensate", False)         # set True if robot moves + you want de-warp

        # ===== Inflation =====
        self.declare_parameter("hard_inflate_radius", 0.22)  # typical = robot radius
        self.declare_parameter("soft_inflate_radius", 0.44)

        # ===== Robot kinematics & constraints =====
        self.declare_parameter("wheel_radius_m", 0.09)           # Wheel radius in meters
        self.declare_parameter("wheelbase_m", 0.22)              # Distance from center to wheel (omni) or half-wheelbase
        self.declare_parameter("max_wheel_acceleration_ms2", 1.0) # Max acceleration per wheel (m/s²)
        self.declare_parameter("max_linear_velocity_ms", 0.5)    # Max linear velocity (m/s)
        self.declare_parameter("max_lateral_accel", 1.0)        # Max lateral (centripetal) accel (m/s^2)

        # ===== Odom history =====
        self.declare_parameter("odom_history_s", 2.0)

        # ===== Waypoints =====
        self.declare_parameter(
            "waypoints",
            [float("nan"), float("nan")],
            ParameterDescriptor(description="Flat list [x1,y1,(ignored), x2,y2,(ignored), ...]"),
        )
        self.declare_parameter(
            "add_wp",
            [float("nan"), float("nan"), float("nan")],
            ParameterDescriptor(description="Set to [x,y] or [x,y,_] to append; node clears after consuming"),
        )
        self.declare_parameter("start_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("wp_n", 0)

        # ===== TF =====
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._T_base_from_scan: Dict[str, object] = {}

        # ===== Odom =====
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.odom_pose_latest = Pose2D(0.0, 0.0, 0.0)
        self.have_odom_pose = False
        self._odom_hist: Deque[Tuple[int, Pose2D]] = deque()

        # ===== LiDAR subs =====
        t1 = self.get_parameter("lidar1_topic").value
        t2 = self.get_parameter("lidar2_topic").value
        self.sub1 = self.create_subscription(LaserScan, t1, self.on_scan1, 20)
        self.sub2 = self.create_subscription(LaserScan, t2, self.on_scan2, 20)
        self.last_scan1: Optional[LaserScan] = None
        self.last_scan2: Optional[LaserScan] = None

        # ===== Grid =====
        self.gs_map = self._make_global_grid_spec()
        self.static_occ: List[int] = [0] * (self.gs_map.width * self.gs_map.height)

        # ===== Publishers =====
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/costmap", 1)
        self.scan_fused_pub = self.create_publisher(LaserScan, "/scan_fused", 10)

        self.wp_marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 1)
        self.path_pub = self.create_publisher(Path, "/planned_path", 1)
        self.velocity_marker_pub = self.create_publisher(MarkerArray, "/path_velocity_markers", 1)

        # ===== Services =====
        self.srv_clear_all_wp = self.create_service(Empty, "/clear_all_waypoints", self.handle_clear_all_waypoints)
        self.srv_pop_next_wp = self.create_service(Empty, "/pop_next_waypoint", self.handle_pop_next_waypoint)

        # Main loop at 5 Hz
        self.timer = self.create_timer(1.0 / 5.0, self.on_timer)

        self._last_tf_warn_ns = 0
        self._tf_warn_period_ns = int(1e9)

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

    def idx(self, gs: GridSpec, ix: int, iy: int) -> int:
        return iy * gs.width + ix

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

    # =======================
    # TF / warnings
    # =======================
    def _warn_tf_throttled(self, msg: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_tf_warn_ns >= self._tf_warn_period_ns:
            self._last_tf_warn_ns = now_ns
            self.get_logger().warn(msg)

    # =======================
    # Odom
    # =======================
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.odom_pose_latest = pose
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

    def _base_pose_now(self) -> Pose2D:
        if self.have_odom_pose:
            return self.odom_pose_latest
        sp = self.get_parameter("start_pose").value
        return Pose2D(float(sp[0]), float(sp[1]), float(sp[2]))

    # =======================
    # TF publishing (odom->base_link)
    # =======================
    def _publish_odom_to_base_tf(self) -> None:
        if not bool(self.get_parameter("publish_odom_to_base_tf").value):
            return
        if not self.have_odom_pose:
            return

        odom_frame = self.get_parameter("map_frame").value  # we treat map_frame as odom frame here
        base_frame = self.get_parameter("base_frame").value

        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = float(self.odom_pose_latest.x)
        t.transform.translation.y = float(self.odom_pose_latest.y)
        t.transform.translation.z = 0.0
        q = yaw_to_quat(self.odom_pose_latest.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    # =======================
    # Scan capture
    # =======================
    def on_scan1(self, msg: LaserScan) -> None:
        self.last_scan1 = msg

    def on_scan2(self, msg: LaserScan) -> None:
        self.last_scan2 = msg

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
        stride = max(1, int(self.get_parameter("scan_beam_stride").value))

        pts: List[Tuple[float, float]] = []
        ang = float(scan.angle_min)
        inc = float(scan.angle_increment)

        for i, rr in enumerate(scan.ranges):
            if (i % stride) != 0:
                ang += inc
                continue

            r = float(rr)
            if (not math.isfinite(r)) or r < rmin or r >= (rmax - no_hit_eps):
                ang += inc
                continue

            xs = r * math.cos(ang)
            ys = r * math.sin(ang)
            xb, yb = self._apply_transform_2d(tr, xs, ys)
            pts.append((xb, yb))

            ang += inc

        return pts

    def _build_fused_scan(self, base_pose_now: Pose2D) -> Optional[LaserScan]:
        if not bool(self.get_parameter("publish_fused_scan").value):
            return None

        max_age_s = float(self.get_parameter("scan_max_age_s").value)
        now_t = self.get_clock().now()

        scans: List[LaserScan] = []
        for s in (self.last_scan1, self.last_scan2):
            if s is None:
                continue
            stamp = Time.from_msg(s.header.stamp)
            age_s = (now_t - stamp).nanoseconds * 1e-9
            if max_age_s > 0.0 and age_s > max_age_s:
                continue
            scans.append(s)

        if not scans:
            return None

        a_min = float(self.get_parameter("fused_angle_min").value)
        a_max = float(self.get_parameter("fused_angle_max").value)
        inc = float(self.get_parameter("fused_angle_increment_deg").value) * math.pi / 180.0
        if inc <= 1e-6:
            inc = math.radians(1.0)

        n = int(math.floor((a_max - a_min) / inc))
        if n < 10:
            return None

        ranges = [math.inf] * n

        use_excl = bool(self.get_parameter("robot_exclusion_enable").value)
        excl_r = float(self.get_parameter("robot_exclusion_radius_m").value)
        excl_r2 = excl_r * excl_r

        motion_comp = bool(self.get_parameter("motion_compensate").value)

        # Use conservative range_max across sensors
        range_max = min(float(s.range_max) for s in scans)
        range_min = max(0.0, min(float(s.range_min) for s in scans))

        # Process each scan and fuse
        for s in scans:
            stamp_ns = Time.from_msg(s.header.stamp).nanoseconds
            
            # Get robot pose at scan time for motion compensation
            if motion_comp:
                base_pose_scan = self._odom_pose_at(stamp_ns) or base_pose_now
            else:
                base_pose_scan = base_pose_now

            # Get transform from sensor frame to base_link
            tr = self._lookup_base_from_scan(s.header.frame_id)
            if tr is None:
                continue

            # Process each beam in the scan
            rmin = float(s.range_min)
            rmax = float(s.range_max)
            no_hit_eps = float(self.get_parameter("scan_no_hit_eps_m").value)
            stride = max(1, int(self.get_parameter("scan_beam_stride").value))
            
            beam_angle = float(s.angle_min)
            beam_inc = float(s.angle_increment)
            
            for i, rr in enumerate(s.ranges):
                if (i % stride) != 0:
                    beam_angle += beam_inc
                    continue

                r = float(rr)
                if (not math.isfinite(r)) or r < rmin or r >= (rmax - no_hit_eps):
                    beam_angle += beam_inc
                    continue

                # Convert beam to Cartesian in sensor frame
                xs = r * math.cos(beam_angle)
                ys = r * math.sin(beam_angle)
                
                # Transform from sensor frame to base_link frame (rotation + translation)
                x_origin, y_origin = self._apply_transform_2d(tr, xs, ys)

                # Apply motion compensation if enabled
                if motion_comp:
                    # Transform point to world frame using scan pose
                    xm, ym = self._se2_apply(base_pose_scan, x_origin, y_origin)
                    # Transform back using current pose (de-warp)
                    x_final, y_final = self._se2_apply_inv(base_pose_now, xm, ym)
                else:
                    x_final = x_origin
                    y_final = y_origin

                # Check robot exclusion
                if use_excl and (x_final * x_final + y_final * y_final) < excl_r2:
                    beam_angle += beam_inc
                    continue

                # Compute range and angle from origin
                rr_fused = math.hypot(x_final, y_final)
                if rr_fused < range_min or rr_fused > range_max:
                    beam_angle += beam_inc
                    continue

                aa_fused = math.atan2(y_final, x_final)
                if aa_fused < a_min or aa_fused >= a_max:
                    beam_angle += beam_inc
                    continue

                # Bin the range (take minimum per angle bin)
                k = int((aa_fused - a_min) / inc)
                if 0 <= k < n and rr_fused < ranges[k]:
                    ranges[k] = rr_fused

                beam_angle += beam_inc

        msg = LaserScan()
        msg.header.stamp = now_t.to_msg()
        msg.header.frame_id = self.get_parameter("base_frame").value
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

    # =======================
    # Costmap building
    # =======================
    def _inflate(self, grid: List[int], hard_r: float, soft_r: float) -> None:
        """
        FIXED: previously you only iterated out to soft_r.
        Now we iterate out to max(hard_r, soft_r), so hard inflation works even if soft=0.
        """
        hard_r = max(0.0, hard_r)
        soft_r = max(0.0, soft_r)

        rad = max(hard_r, soft_r)
        if rad <= 1e-6:
            return

        rad_cells = int(math.ceil(rad / self.gs_map.res))
        if rad_cells <= 0:
            return

        hard_cells: List[Tuple[int, int]] = []
        for iy in range(self.gs_map.height):
            row = iy * self.gs_map.width
            for ix in range(self.gs_map.width):
                if grid[row + ix] >= 100:
                    hard_cells.append((ix, iy))

        if not hard_cells:
            return

        for (hx, hy) in hard_cells:
            for dy in range(-rad_cells, rad_cells + 1):
                for dx in range(-rad_cells, rad_cells + 1):
                    ix = hx + dx
                    iy = hy + dy
                    if not self.in_bounds(self.gs_map, ix, iy):
                        continue
                    d = math.hypot(dx, dy) * self.gs_map.res
                    k = self.idx(self.gs_map, ix, iy)
                    if hard_r > 0 and d <= hard_r:
                        grid[k] = 100
                    elif soft_r > 0 and d <= soft_r and grid[k] < 100:
                        grid[k] = max(grid[k], 50)

    def _clear_robot_circle_in_costmap(self, grid: List[int], base_pose_now: Pose2D) -> None:
        if not bool(self.get_parameter("robot_exclusion_enable").value):
            return
        r = float(self.get_parameter("robot_exclusion_radius_m").value)
        if r <= 1e-6:
            return

        rad_cells = int(math.ceil(r / self.gs_map.res))
        cx_i, cy_i = self.world_to_grid(self.gs_map, base_pose_now.x, base_pose_now.y)
        r2 = r * r

        for dy in range(-rad_cells, rad_cells + 1):
            for dx in range(-rad_cells, rad_cells + 1):
                ix = cx_i + dx
                iy = cy_i + dy
                if not self.in_bounds(self.gs_map, ix, iy):
                    continue
                wx, wy = self.grid_to_world(self.gs_map, ix, iy)
                if (wx - base_pose_now.x) ** 2 + (wy - base_pose_now.y) ** 2 <= r2:
                    grid[self.idx(self.gs_map, ix, iy)] = 0

    def _build_costmap_from_fused_scan(self, fused: LaserScan, base_pose_now: Pose2D) -> List[int]:
        dynamic = [0] * (self.gs_map.width * self.gs_map.height)

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
                dynamic[self.idx(self.gs_map, ix, iy)] = 100

            a += inc

        combined = [max(int(self.static_occ[i]), int(dynamic[i])) for i in range(len(dynamic))]

        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        self._inflate(combined, hard_r=hard_r, soft_r=soft_r)

        self._clear_robot_circle_in_costmap(combined, base_pose_now)
        return combined

    # =======================
    # Waypoints parsing + removal
    # =======================
    def consume_add_wp(self) -> None:
        add_wp = self.get_parameter("add_wp").value
        if not add_wp or len(add_wp) not in (2, 3):
            return
        if not (math.isfinite(add_wp[0]) and math.isfinite(add_wp[1])):
            return

        vals = [float(add_wp[0]), float(add_wp[1])]
        wp_flat = list(self.get_parameter("waypoints").value)
        wp_n = int(self.get_parameter("wp_n").value)

        stride = 2
        if wp_n > 0 and len(wp_flat) == 3 * wp_n:
            stride = 3

        if wp_n == 0:
            wp_flat = vals if stride == 2 else [vals[0], vals[1], float("nan")]
        else:
            if stride == 2:
                wp_flat.extend(vals)
            else:
                wp_flat.extend([vals[0], vals[1], float("nan")])

        self.set_parameters(
            [
                Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, wp_flat),
                Parameter("wp_n", Parameter.Type.INTEGER, wp_n + 1),
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

    def _pop_reached_waypoints(self, base_pose_now: Pose2D) -> None:
        tol = float(self.get_parameter("waypoint_reached_tol_m").value)
        if tol <= 0.0:
            return

        wp_n = int(self.get_parameter("wp_n").value)
        if wp_n <= 0:
            return

        wp_flat = list(self.get_parameter("waypoints").value)
        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            return

        removed = 0
        while wp_n > 0:
            wx = float(wp_flat[0])
            wy = float(wp_flat[1])
            if math.hypot(wx - base_pose_now.x, wy - base_pose_now.y) > tol:
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

    def handle_clear_all_waypoints(self, request, response):
        """Clear all waypoints."""
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
        return response

    # =======================
    # Planning (A*) — unchanged / minimal
    # =======================
    def line_collision_free(self, grid: List[int], a_xy: Tuple[float, float], b_xy: Tuple[float, float]) -> bool:
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
            if grid[self.idx(self.gs_map, ix, iy)] >= 100:  # Only block hard obstacles
                return False
        return True

    def get_wheel_velocities(self, vx_body: float, vy_body: float, omega: float) -> List[float]:
        """
        Compute required wheel velocities for 3-wheel omnidirectional robot.
        Uses standard 120° wheel configuration (equilateral triangle).
        
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
        
        # 3-wheel omni kinematics (120° wheel spacing)
        # Inverse kinematics: body velocity to wheel velocity
        sqrt3_2 = math.sqrt(3) / 2.0
        
        w1 = (2.0 / 3.0 * vy_body + L * omega) / r
        w2 = (-0.5 * vy_body + sqrt3_2 * vx_body + L * omega) / r
        w3 = (-0.5 * vy_body - sqrt3_2 * vx_body + L * omega) / r
        
        return [w1, w2, w3]

    def body_velocity_from_wheel_velocity(self, w1: float, w2: float, w3: float) -> Tuple[float, float, float]:
        """
        Compute body velocity from wheel velocities.
        Used for sensor feedback / odometry.
        
        Args:
            w1, w2, w3: Wheel velocities (rad/s)
        
        Returns:
            Tuple of (vx_body, vy_body, omega)
        """
        r = float(self.get_parameter("wheel_radius_m").value)
        L = float(self.get_parameter("wheelbase_m").value)
        
        if r < 1e-6 or L < 1e-6:
            return 0.0, 0.0, 0.0
        
        # Forward kinematics: wheel velocity to body velocity
        sqrt3_3 = math.sqrt(3) / 3.0
        
        vx_body = r * sqrt3_3 * (w2 - w3)
        vy_body = r * (2.0 / 3.0 * w1 - 1.0 / 3.0 * (w2 + w3))
        omega = r * (w1 + w2 + w3) / (3.0 * L)
        
        return vx_body, vy_body, omega

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

    def constrain_velocity_by_kinematics(self, vx: float, vy: float) -> Tuple[float, float]:
        """
        Constrain desired velocity (vx, vy) to respect max wheel acceleration.
        Assumes omega=0 (no rotation for now).
        Returns constrained (vx, vy).
        """
        max_wheel_accel = float(self.get_parameter("max_wheel_acceleration_ms2").value)
        max_linear_vel = float(self.get_parameter("max_linear_velocity_ms").value)
        
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

    def astar(self, grid: List[int], start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        sx, sy = start_xy
        gx, gy = goal_xy
        sxi, syi = self.world_to_grid(self.gs_map, sx, sy)
        gxi, gyi = self.world_to_grid(self.gs_map, gx, gy)

        if not self.in_bounds(self.gs_map, sxi, syi) or not self.in_bounds(self.gs_map, gxi, gyi):
            return []
        if grid[self.idx(self.gs_map, gxi, gyi)] >= 100:
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
                if grid[self.idx(self.gs_map, nx, ny)] >= 100:  # Hard obstacle blocks
                    continue
                
                # Add cell cost to movement cost: soft cells (50) add 0.5, hard cells add 1.0
                cell_cost = grid[self.idx(self.gs_map, nx, ny)] / 100.0
                
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

    def plan_segment_path(self, grid: List[int], start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        if self.line_collision_free(grid, start_xy, goal_xy):
            return [start_xy, goal_xy]
        path = self.astar(grid, start_xy, goal_xy)
        if not path:
            return []
        path[0] = start_xy
        path[-1] = goal_xy
        
        # Simplify path to remove unnecessary waypoints (shortcutting)
        path = self.simplify_path(grid, path)
        
        return path

    def simplify_path(self, grid: List[int], path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
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

    def smooth_path_cubic_spline(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth path using cubic spline interpolation (industry standard).
        Produces smooth, natural-looking curves through waypoints.
        
        Args:
            path: A* waypoints (already simplified)
        
        Returns:
            Densely sampled smooth path
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
        
        # Create cubic splines for x(t) and y(t)
        # Using 'not-a-knot' boundary condition for smoother paths around obstacles
        try:
            spline_x = CubicSpline(t, xs, bc_type='not-a-knot')
            spline_y = CubicSpline(t, ys, bc_type='not-a-knot')
        except Exception as e:
            self.get_logger().warn(f"Spline interpolation failed: {e}. Using original path.")
            return path
        
        # Resample smooth path at regular intervals
        # Use finer spacing (3cm) for better smoothness around obstacles
        n_samples = max(100, int(t[-1] / 0.03))
        t_smooth = [t[0] + (t[-1] - t[0]) * i / max(1, n_samples - 1) for i in range(n_samples)]
        
        smooth_path = []
        for t_val in t_smooth:
            x_smooth = float(spline_x(t_val))
            y_smooth = float(spline_y(t_val))
            smooth_path.append((x_smooth, y_smooth))
        
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

        max_wheel_accel = float(self.get_parameter("max_wheel_acceleration_ms2").value)
        max_linear_vel = float(self.get_parameter("max_linear_velocity_ms").value)
        max_lateral_accel = float(self.get_parameter("max_lateral_accel").value)

        n = len(path)

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

        v_forward = [0.0] * n
        for i in range(n):
            # from rest: v = sqrt(2 * a * distance)
            v_f = math.sqrt(max(0.0, 2.0 * max_wheel_accel * s[i]))
            v_forward[i] = min(v_f, max_linear_vel)

        v_backward = [0.0] * n
        for i in range(n - 1, -1, -1):
            dist_to_end = max(0.0, total_s - s[i])
            v_b = math.sqrt(max(0.0, 2.0 * max_wheel_accel * dist_to_end))
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
            # v_w1 = 2/3*vy
            # v_w2 = -0.5*vy + sqrt(3)/2*vx
            # v_w3 = -0.5*vy - sqrt(3)/2*vx
            s1 = (2.0 / 3.0) * ty
            s2 = (math.sqrt(3) / 2.0) * tx - 0.5 * ty
            s3 = -(math.sqrt(3) / 2.0) * tx - 0.5 * ty
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
                allowed_delta = max_wheel_accel * dt / max_coeff
                if v_i > v_j + allowed_delta:
                    velocities[i] = v_j + allowed_delta

        # Final clamp to max linear velocity and non-negative
        for i in range(n):
            velocities[i] = max(0.0, min(velocities[i], max_linear_vel))

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        yaws = [0.0] * n
        
        # Resample trajectory at 0.01s timesteps (100Hz) to match controller dt
        # Matching dt prevents aliasing and acceleration discontinuities
        xs_resampled, ys_resampled, yaws_resampled, vels_resampled, vxs_resampled, vys_resampled = self._resample_trajectory_at_dt(
            xs, ys, yaws, velocities, dists, dt=0.01
        )
        
        return xs_resampled, ys_resampled, yaws_resampled, vels_resampled, vxs_resampled, vys_resampled

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
    def _publish_grid(self, pub, frame_id: str, grid: List[int]) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.info.resolution = self.gs_map.res
        msg.info.width = self.gs_map.width
        msg.info.height = self.gs_map.height
        msg.info.origin.position.x = self.gs_map.origin_x
        msg.info.origin.position.y = self.gs_map.origin_y
        msg.info.origin.position.z = 0.0
        q = yaw_to_quat(0.0)
        msg.info.origin.orientation.x = q[0]
        msg.info.origin.orientation.y = q[1]
        msg.info.origin.orientation.z = q[2]
        msg.info.origin.orientation.w = q[3]
        msg.data = [int(v) for v in grid]
        pub.publish(msg)

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
        self.wp_marker_pub.publish(ma)

    def publish_velocity_markers(self, xs: List[float], ys: List[float], velocities: List[float]) -> None:
        """Publish velocity as colored line strip along the path."""
        ma = MarkerArray()
        
        if len(xs) < 2 or len(velocities) != len(xs):
            self.velocity_marker_pub.publish(ma)
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
        self.velocity_marker_pub.publish(ma)
    
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
        self.consume_add_wp()

        # base pose now (in odom/map_frame)
        base_pose_now = self._base_pose_now()

        # remove reached waypoints
        self._pop_reached_waypoints(base_pose_now)

        # publish odom->base tf if requested
        self._publish_odom_to_base_tf()

        # build + publish fused scan (for RViz) and costmap from it
        fused = self._build_fused_scan(base_pose_now)
        if fused is not None:
            self.scan_fused_pub.publish(fused)

        frame = self.get_parameter("map_frame").value

        # publish static map (empty) so RViz "Map" display can be used if you want
        self._publish_grid(self.map_pub, frame, self.static_occ)

        if fused is None:
            # no scans yet => empty costmap
            self._publish_grid(self.costmap_pub, frame, self.static_occ)
            return

        costmap = self._build_costmap_from_fused_scan(fused, base_pose_now)
        self._publish_grid(self.costmap_pub, frame, costmap)

        # plan if waypoints exist
        wps = self.read_waypoints()
        if not wps:
            return

        self.publish_waypoints(wps)

        stitched: List[Tuple[float, float]] = []
        start_xy = (base_pose_now.x, base_pose_now.y)

        for (gx, gy) in wps:
            seg = self.plan_segment_path(costmap, start_xy, (gx, gy))
            if not seg:
                self.get_logger().warn(f"Planning failed start={start_xy} goal={(gx, gy)}")
                return
            if not stitched:
                stitched.extend(seg)
            else:
                stitched.extend(seg[1:])
            start_xy = (gx, gy)

        # Smooth path using cubic spline interpolation (industry standard)
        stitched_smooth = self.smooth_path_cubic_spline(stitched)

        # Build velocity-constrained trajectory on smoothed path
        xs, ys, yaws, velocities, vxs, vys = self.build_velocity_constrained_trajectory(stitched_smooth)
        
        if not xs:
            return
        
        # Save trajectory to JSON file
        # self.save_trajectory_json(xs, ys, yaws, velocities, vxs, vys)
        
        # Publish path with orientation from yaw
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
        self.path_pub.publish(path)
        
        # Publish velocity markers using actual trajectory velocities
        self.publish_velocity_markers(xs, ys, velocities)


def main() -> None:
    rclpy.init()
    node = WaypointTrajNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

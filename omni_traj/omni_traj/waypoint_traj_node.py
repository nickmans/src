#!/usr/bin/env python3
"""
waypoint_traj.py

Yaw-decoupled translation trajectory planner for a 3-omni kiwi drive.

Robust multi-waypoint behavior:
- Removes forward-looking max-curvature "future cap" behavior that causes dip->rise->dip.
- Computes theta_v from smoothed derivatives of (x,y), eliminating curvature impulses.
- Builds a smooth feasible speed envelope v(s) via backward/forward passes:
    v(s) respects v_max, direction-rate cap, wheel-speed cap, and translational accel bound.
- Optionally rounds corners (C1 blend) WHILE still passing EXACTLY through each intermediate waypoint.

Exact waypoint visitation:
- Waypoints are positional constraints only (no forced stop).
- dt integrator guarantees a sample exactly at each waypoint arc-length when feasible,
  else approaches without crossing and hits exactly later.

Obstacle avoidance:
- Local costmap from fused LiDAR scans (+ inflation).
- Each segment uses straight line if collision-free, else A* (soft penalty).

Limits enforced (translation-only):
- v_max (live-tunable)
- wheel speed cap (direction-dependent kiwi translation model)
- wheel accel cap (exact per-wheel discrete constraint)
- velocity-direction rate cap via curvature of theta_v(s)
- smooth braking/accel envelope in s-space (prevents pre-corner bouncing)

Waypoint param format (backward-compatible):
- waypoints: flat [x1,y1,(ignored), x2,y2,(ignored), ...]
- add_wp: [x,y] or [x,y,_] appended; third value ignored
"""

import math
import heapq
import json
import bisect
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple, Dict, Set
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


PHI1 = math.pi / 2.0
PHI2 = -math.pi / 6.0
PHI3 = -5.0 * math.pi / 6.0


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


@dataclass
class GridSpec:
    res: float
    width: int
    height: int
    origin_x: float
    origin_y: float


class WaypointTrajNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_traj")

        # --- Params (live-tunable)
        self.declare_parameter("dt", 0.01)
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("ds_geom", 0.05)

        # direction-rate limiting (theta_v curvature -> v cap)
        self.declare_parameter("omega_dir_max", 2.0)

        # IMPORTANT: this is now a SMOOTHING length (meters) for curvature/heading,
        # not a forward-looking max window. Default 0 removes the dip->rise artifact.
        self.declare_parameter("omega_dir_lookahead_m", 0.0)

        # Additional theta_v smoothing (unit-vector moving average window in samples)
        self.declare_parameter("theta_smooth_window", 1)  # odd recommended: 5,7,9

        # Wheel feasibility
        self.declare_parameter("wheel_radius", 0.09)
        self.declare_parameter("max_wheel_speed", 12.0)   # rad/s
        self.declare_parameter("max_wheel_accel", 6.0)    # rad/s^2
        self.declare_parameter("use_yaw_for_wheel_limits", True)

        # --- SPEED PROFILE ANTI-BOUNCE FIX ---
        # a_trans used for v(s) envelope + dt accel clamp. Scale down to avoid accel/brake "pulses".
        self.declare_parameter("a_trans_scale", 0.65)     # 0.5..1.0 (lower = smoother)
        # Smooth v_prof(s) then re-project through envelope to kill small local maxima between dips.
        self.declare_parameter("profile_envelope_iters", 2)   # how many backward+forward projections
        self.declare_parameter("profile_smooth_window", 11)   # odd, in samples along s
        self.declare_parameter("profile_smooth_iters", 2)     # smooth+reproject cycles

        # Local costmap window
        self.declare_parameter("map_res", 0.01)
        self.declare_parameter("map_width_m", 3.0)
        self.declare_parameter("map_height_m", 3.0)
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # Inflation
        self.declare_parameter("hard_inflate_radius", 0.2)
        self.declare_parameter("soft_inflate_radius", 0.2)

        # A* config
        self.declare_parameter("astar_soft_penalty", 6.0)
        self.declare_parameter("astar_nearest_free_search_m", 0.25)

        # Corner rounding (robust fix for multi-waypoint jagged slowdown)
        self.declare_parameter("corner_enable", True)
        self.declare_parameter("corner_blend_m", 0.12)
        self.declare_parameter("corner_min_angle_deg", 12.0)
        self.declare_parameter("corner_blend_samples", 10)
        self.declare_parameter("corner_check_soft", False)

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

        self.declare_parameter("lidar1_topic", "/lidar1/scan")
        self.declare_parameter("lidar2_topic", "/lidar2/scan")

        # --- TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Odometry
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.current_speed = 0.0

        # --- LiDAR
        t1 = self.get_parameter("lidar1_topic").value
        t2 = self.get_parameter("lidar2_topic").value
        self.sub1 = self.create_subscription(LaserScan, t1, self.on_scan1, 10)
        self.sub2 = self.create_subscription(LaserScan, t2, self.on_scan2, 10)
        self.last_scan1: Optional[LaserScan] = None
        self.last_scan2: Optional[LaserScan] = None

        # --- Publishers
        self.wp_marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 1)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/costmap", 1)
        self.path_pub = self.create_publisher(Path, "/planned_path", 1)
        self.traj_path_pub = self.create_publisher(Path, "/trajectory_path", 1)
        self.traj_v_pub = self.create_publisher(Float32MultiArray, "/trajectory_v", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/trajectory_markers", 1)

        self.timer = self.create_timer(0.5, self.on_timer)
        self.marker_counter = 0

    # ---------- Callbacks ----------
    def on_scan1(self, msg: LaserScan) -> None:
        self.last_scan1 = msg

    def on_scan2(self, msg: LaserScan) -> None:
        self.last_scan2 = msg

    def on_odom(self, msg: Odometry) -> None:
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self.current_speed = float(math.hypot(vx, vy))

    # ---------- TF pose ----------
    def get_current_pose(self) -> Tuple[float, float, float]:
        frame_map = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(frame_map, base_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return float(t.x), float(t.y), float(yaw)
        except Exception:
            sp = self.get_parameter("start_pose").value
            return float(sp[0]), float(sp[1]), float(sp[2])

    # ---------- Grid utils ----------
    def make_grid_spec(self, center_xy: Tuple[float, float]) -> GridSpec:
        res = float(self.get_parameter("map_res").value)
        w_m = float(self.get_parameter("map_width_m").value)
        h_m = float(self.get_parameter("map_height_m").value)
        width = int(round(w_m / res))
        height = int(round(h_m / res))
        cx, cy = float(center_xy[0]), float(center_xy[1])
        origin_x = cx - 0.5 * width * res
        origin_y = cy - 0.5 * height * res
        return GridSpec(res=res, width=width, height=height, origin_x=origin_x, origin_y=origin_y)

    def world_to_grid(self, gs: GridSpec, x: float, y: float) -> Tuple[int, int]:
        ix = int(math.floor((x - gs.origin_x) / gs.res))
        iy = int(math.floor((y - gs.origin_y) / gs.res))
        return ix, iy

    def grid_to_world(self, gs: GridSpec, ix: int, iy: int) -> Tuple[float, float]:
        x = gs.origin_x + (ix + 0.5) * gs.res
        y = gs.origin_y + (iy + 0.5) * gs.res
        return x, y

    def in_bounds(self, gs: GridSpec, ix: int, iy: int) -> bool:
        return 0 <= ix < gs.width and 0 <= iy < gs.height

    def idx(self, gs: GridSpec, ix: int, iy: int) -> int:
        return iy * gs.width + ix

    def occ_at_world(self, gs: GridSpec, grid: List[int], x: float, y: float) -> Optional[int]:
        ix, iy = self.world_to_grid(gs, x, y)
        if not self.in_bounds(gs, ix, iy):
            return None
        return int(grid[self.idx(gs, ix, iy)])

    # ---------- Costmap ----------
    def build_costmap(self, center_xy: Tuple[float, float]) -> Tuple[GridSpec, List[int]]:
        gs = self.make_grid_spec(center_xy=center_xy)
        grid = [0] * (gs.width * gs.height)

        pts: List[Tuple[float, float]] = []
        if self.last_scan1 is not None:
            pts.extend(self.scan_to_points(self.last_scan1))
        if self.last_scan2 is not None:
            pts.extend(self.scan_to_points(self.last_scan2))

        for (x, y) in pts:
            ix, iy = self.world_to_grid(gs, x, y)
            if self.in_bounds(gs, ix, iy):
                grid[self.idx(gs, ix, iy)] = 100

        hard_r = float(self.get_parameter("hard_inflate_radius").value)
        soft_r = float(self.get_parameter("soft_inflate_radius").value)
        self.inflate(grid, gs, hard_r=hard_r, soft_r=soft_r)
        return gs, grid

    def scan_to_points(self, scan: LaserScan) -> List[Tuple[float, float]]:
        frame_map = self.get_parameter("map_frame").value
        scan_frame = scan.header.frame_id

        T = None
        try:
            tf = self.tf_buffer.lookup_transform(frame_map, scan_frame, Time())
            T = tf.transform
        except Exception:
            T = None

        pts: List[Tuple[float, float]] = []
        angle = float(scan.angle_min)
        for r in scan.ranges:
            rr = float(r)
            if math.isfinite(rr) and (scan.range_min <= rr <= scan.range_max):
                xs = rr * math.cos(angle)
                ys = rr * math.sin(angle)
                if T is None:
                    xm, ym = xs, ys
                else:
                    xm, ym = self.apply_transform_2d(T, xs, ys)
                pts.append((xm, ym))
            angle += float(scan.angle_increment)
        return pts

    def apply_transform_2d(self, tr, x: float, y: float) -> Tuple[float, float]:
        tx = float(tr.translation.x)
        ty = float(tr.translation.y)
        qx = float(tr.rotation.x)
        qy = float(tr.rotation.y)
        qz = float(tr.rotation.z)
        qw = float(tr.rotation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        xr = math.cos(yaw) * x - math.sin(yaw) * y
        yr = math.sin(yaw) * x + math.cos(yaw) * y
        return (xr + tx, yr + ty)

    def inflate(self, grid: List[int], gs: GridSpec, hard_r: float, soft_r: float) -> None:
        if soft_r <= 1e-6 and hard_r <= 1e-6:
            return

        hard_cells: List[Tuple[int, int]] = []
        for iy in range(gs.height):
            for ix in range(gs.width):
                if grid[self.idx(gs, ix, iy)] >= 100:
                    hard_cells.append((ix, iy))

        if not hard_cells:
            return

        soft_rad = int(math.ceil(soft_r / gs.res)) if soft_r > 0 else 0
        for (hx, hy) in hard_cells:
            for dy in range(-soft_rad, soft_rad + 1):
                for dx in range(-soft_rad, soft_rad + 1):
                    ix = hx + dx
                    iy = hy + dy
                    if not self.in_bounds(gs, ix, iy):
                        continue
                    d = math.hypot(dx, dy) * gs.res
                    if hard_r > 0 and d <= hard_r:
                        grid[self.idx(gs, ix, iy)] = 100
                    elif soft_r > 0 and d <= soft_r:
                        if grid[self.idx(gs, ix, iy)] < 100:
                            grid[self.idx(gs, ix, iy)] = max(grid[self.idx(gs, ix, iy)], 50)

    # ---------- A* helpers ----------
    def nearest_free_cell(
        self,
        gs: GridSpec,
        grid: List[int],
        ij: Tuple[int, int],
        max_radius_cells: int,
    ) -> Optional[Tuple[int, int]]:
        gx, gy = ij
        if self.in_bounds(gs, gx, gy) and grid[self.idx(gs, gx, gy)] < 100:
            return ij

        visited = {ij}
        q: Deque[Tuple[int, int]] = deque([ij])

        while q:
            x, y = q.popleft()
            if not self.in_bounds(gs, x, y):
                continue
            if abs(x - gx) > max_radius_cells or abs(y - gy) > max_radius_cells:
                continue
            if grid[self.idx(gs, x, y)] < 100:
                return (x, y)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    q.append((nx, ny))
        return None

    def line_collision_free(
        self,
        gs: GridSpec,
        grid: List[int],
        a_xy: Tuple[float, float],
        b_xy: Tuple[float, float],
        step_m: Optional[float] = None,
    ) -> bool:
        if step_m is None:
            step_m = max(gs.res * 0.5, 0.01)
        ax, ay = a_xy
        bx, by = b_xy
        dist = math.hypot(bx - ax, by - ay)
        if dist < 1e-9:
            return True
        n = int(math.ceil(dist / step_m))
        for i in range(n + 1):
            t = i / max(n, 1)
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            ix, iy = self.world_to_grid(gs, x, y)
            if not self.in_bounds(gs, ix, iy):
                return False
            if grid[self.idx(gs, ix, iy)] >= 100:
                return False
        return True

    def astar(self, gs: GridSpec, grid: List[int], start_xy, goal_xy) -> List[Tuple[float, float]]:
        sx, sy = start_xy
        gx, gy = goal_xy
        sxi, syi = self.world_to_grid(gs, sx, sy)
        gxi, gyi = self.world_to_grid(gs, gx, gy)

        if not self.in_bounds(gs, sxi, syi) or not self.in_bounds(gs, gxi, gyi):
            return []
        if grid[self.idx(gs, gxi, gyi)] >= 100:
            return []

        soft_pen = float(self.get_parameter("astar_soft_penalty").value)

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
                return self.reconstruct(gs, came, (gxi, gyi))

            for dx, dy, w in neigh:
                nx, ny = ix + dx, iy + dy
                if not self.in_bounds(gs, nx, ny):
                    continue
                occ = grid[self.idx(gs, nx, ny)]
                if occ >= 100:
                    continue
                penalty = soft_pen if occ >= 50 else 0.0
                ng = gcur + w + penalty
                if (nx, ny) not in gscore or ng < gscore[(nx, ny)]:
                    gscore[(nx, ny)] = ng
                    came[(nx, ny)] = (ix, iy)
                    heapq.heappush(openq, (ng + h(nx, ny), ng, (nx, ny)))
        return []

    def reconstruct(self, gs: GridSpec, came: dict, goal_ij: Tuple[int, int]) -> List[Tuple[float, float]]:
        path_ij = [goal_ij]
        cur = goal_ij
        while cur in came:
            cur = came[cur]
            path_ij.append(cur)
        path_ij.reverse()
        return [self.grid_to_world(gs, ix, iy) for (ix, iy) in path_ij]

    def plan_segment_path(
        self,
        gs: GridSpec,
        grid: List[int],
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        gxi, gyi = self.world_to_grid(gs, goal_xy[0], goal_xy[1])
        if not self.in_bounds(gs, gxi, gyi):
            self.get_logger().warn("Waypoint outside local costmap window. Increase map_width_m/map_height_m.")
            return []
        if grid[self.idx(gs, gxi, gyi)] >= 100:
            self.get_logger().warn("Waypoint lies in inflated obstacle; cannot satisfy exact visitation.")
            return []

        if self.line_collision_free(gs, grid, start_xy, goal_xy):
            return [start_xy, goal_xy]

        sxi, syi = self.world_to_grid(gs, start_xy[0], start_xy[1])
        if not self.in_bounds(gs, sxi, syi):
            return []
        if grid[self.idx(gs, sxi, syi)] >= 100:
            max_search_m = float(self.get_parameter("astar_nearest_free_search_m").value)
            max_r = int(math.ceil(max_search_m / gs.res))
            s_free = self.nearest_free_cell(gs, grid, (sxi, syi), max_r)
            if s_free is None:
                return []
            start_xy = self.grid_to_world(gs, s_free[0], s_free[1])

        path = self.astar(gs, grid, start_xy, goal_xy)
        if not path:
            return []
        path[0] = start_xy
        path[-1] = goal_xy
        return path

    # ---------- Waypoint parsing ----------
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

        self.set_parameters([
            Parameter("waypoints", Parameter.Type.DOUBLE_ARRAY, wp_flat),
            Parameter("wp_n", Parameter.Type.INTEGER, wp_n + 1),
            Parameter("add_wp", Parameter.Type.DOUBLE_ARRAY, [float("nan"), float("nan"), float("nan")]),
        ])
        self.get_logger().info(f"Added waypoint {vals}, wp_n now {wp_n + 1}")

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
            self.get_logger().warn(
                f"Waypoints length {len(wp_flat)} doesn't match wp_n={wp_n} (expected 2*wp_n or 3*wp_n)"
            )
            return None

        out: List[Tuple[float, float]] = []
        for i in range(wp_n):
            x = float(wp_flat[stride * i])
            y = float(wp_flat[stride * i + 1])
            out.append((x, y))
        return out

    # ---------- Geometry: simplify + resample ----------
    def simplify_polyline_keep_indices(
        self,
        pts: List[Tuple[float, float]],
        keep_vertex_indices: List[int],
        eps_angle: float = 1e-6,
        eps_dist: float = 1e-9,
    ) -> Tuple[List[Tuple[float, float]], List[int], List[int]]:
        if len(pts) < 3:
            return pts, keep_vertex_indices, list(range(len(pts)))

        keep_set = set(keep_vertex_indices)
        new_pts: List[Tuple[float, float]] = [pts[0]]
        old_to_new = [-1] * len(pts)
        old_to_new[0] = 0

        def is_collinear(a, b, c) -> bool:
            ax, ay = a
            bx, by = b
            cx, cy = c
            abx, aby = bx - ax, by - ay
            bcx, bcy = cx - bx, cy - by
            lab = math.hypot(abx, aby)
            lbc = math.hypot(bcx, bcy)
            if lab < eps_dist or lbc < eps_dist:
                return True
            th1 = math.atan2(aby, abx)
            th2 = math.atan2(bcy, bcx)
            return abs(wrap_to_pi(th2 - th1)) <= eps_angle

        for i in range(1, len(pts) - 1):
            if i in keep_set:
                new_pts.append(pts[i])
                old_to_new[i] = len(new_pts) - 1
                continue
            a = new_pts[-1]
            b = pts[i]
            c = pts[i + 1]
            if is_collinear(a, b, c):
                continue
            new_pts.append(b)
            old_to_new[i] = len(new_pts) - 1

        new_pts.append(pts[-1])
        old_to_new[len(pts) - 1] = len(new_pts) - 1

        new_keep: List[int] = []
        for idx in keep_vertex_indices:
            mapped = old_to_new[idx]
            if mapped >= 0:
                new_keep.append(mapped)
        return new_pts, sorted(set(new_keep)), old_to_new

    def resample_vertex_preserving_with_map(
        self,
        poly_xy: List[Tuple[float, float]],
        ds: float,
    ) -> Tuple[List[float], List[float], List[int]]:
        if not poly_xy:
            return [], [], []
        if len(poly_xy) == 1:
            x, y = poly_xy[0]
            return [float(x)], [float(y)], [0]

        ds = max(float(ds), 1e-6)

        xs: List[float] = [float(poly_xy[0][0])]
        ys: List[float] = [float(poly_xy[0][1])]
        vertex_out: List[int] = [0]

        for i in range(len(poly_xy) - 1):
            x0, y0 = float(poly_xy[i][0]), float(poly_xy[i][1])
            x1, y1 = float(poly_xy[i + 1][0]), float(poly_xy[i + 1][1])
            dx, dy = x1 - x0, y1 - y0
            seg_len = math.hypot(dx, dy)

            if seg_len < 1e-12:
                xs.append(x1)
                ys.append(y1)
                vertex_out.append(len(xs) - 1)
                continue

            n_inner = int(math.floor(seg_len / ds))
            for k in range(1, n_inner + 1):
                d = k * ds
                if d >= seg_len:
                    break
                a = d / seg_len
                xs.append(x0 + a * dx)
                ys.append(y0 + a * dy)

            xs.append(x1)
            ys.append(y1)
            vertex_out.append(len(xs) - 1)

        return xs, ys, vertex_out

    def compute_theta_from_xy(self, xs: List[float], ys: List[float]) -> List[float]:
        """Compute a smooth theta_v from central differences + unit-vector moving average."""
        n = len(xs)
        if n < 2:
            return [0.0] * n

        dx = [0.0] * n
        dy = [0.0] * n
        dx[0] = xs[1] - xs[0]
        dy[0] = ys[1] - ys[0]
        dx[-1] = xs[-1] - xs[-2]
        dy[-1] = ys[-1] - ys[-2]
        for i in range(1, n - 1):
            dx[i] = xs[i + 1] - xs[i - 1]
            dy[i] = ys[i + 1] - ys[i - 1]

        th = [float(math.atan2(dy[i], dx[i])) for i in range(n)]

        for i in range(1, n):
            th[i] = th[i - 1] + wrap_to_pi(th[i] - th[i - 1])

        w = int(self.get_parameter("theta_smooth_window").value)
        if w < 1:
            w = 1
        if w % 2 == 0:
            w += 1
        half = w // 2

        if w > 1:
            c = [math.cos(t) for t in th]
            s = [math.sin(t) for t in th]
            cs = [0.0] * n
            ss = [0.0] * n
            for i in range(n):
                j0 = max(0, i - half)
                j1 = min(n - 1, i + half)
                acc_c = 0.0
                acc_s = 0.0
                cnt = 0
                for j in range(j0, j1 + 1):
                    acc_c += c[j]
                    acc_s += s[j]
                    cnt += 1
                cs[i] = acc_c / max(1, cnt)
                ss[i] = acc_s / max(1, cnt)
            th = [math.atan2(ss[i], cs[i]) for i in range(n)]
            for i in range(1, n):
                th[i] = th[i - 1] + wrap_to_pi(th[i] - th[i - 1])

        return [wrap_to_pi(t) for t in th]

    # ---------- Corner blending (C1) ----------
    def hermite(self, p0, p1, m0, m1, t: float) -> Tuple[float, float]:
        t2 = t * t
        t3 = t2 * t
        h00 = 2.0 * t3 - 3.0 * t2 + 1.0
        h10 = t3 - 2.0 * t2 + t
        h01 = -2.0 * t3 + 3.0 * t2
        h11 = t3 - t2
        x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
        y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
        return (float(x), float(y))

    def try_blend_corner_through_waypoint(
        self,
        gs: GridSpec,
        grid: List[int],
        p_prev: Tuple[float, float],
        p_wp: Tuple[float, float],
        p_next: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        L = float(self.get_parameter("corner_blend_m").value)
        n = int(self.get_parameter("corner_blend_samples").value)
        ang_min = float(self.get_parameter("corner_min_angle_deg").value) * math.pi / 180.0
        check_soft = bool(self.get_parameter("corner_check_soft").value)

        if L <= 1e-4 or n < 3:
            return None

        vin = (p_wp[0] - p_prev[0], p_wp[1] - p_prev[1])
        vout = (p_next[0] - p_wp[0], p_next[1] - p_wp[1])
        lin = math.hypot(vin[0], vin[1])
        lout = math.hypot(vout[0], vout[1])
        if lin < 1e-6 or lout < 1e-6:
            return None

        uin = (vin[0] / lin, vin[1] / lin)
        uout = (vout[0] / lout, vout[1] / lout)

        dot = max(-1.0, min(1.0, uin[0] * uout[0] + uin[1] * uout[1]))
        ang = math.acos(dot)
        if ang < ang_min:
            return None

        Lin = min(L, 0.45 * lin)
        Lout = min(L, 0.45 * lout)
        if Lin < 1e-4 or Lout < 1e-4:
            return None

        A = (p_wp[0] - uin[0] * Lin, p_wp[1] - uin[1] * Lin)
        B = (p_wp[0] + uout[0] * Lout, p_wp[1] + uout[1] * Lout)

        t = (uin[0] + uout[0], uin[1] + uout[1])
        lt = math.hypot(t[0], t[1])
        if lt < 1e-6:
            return None
        t = (t[0] / lt, t[1] / lt)

        s_wp = 0.65 * min(Lin, Lout)
        mA = (uin[0] * Lin, uin[1] * Lin)
        mWP = (t[0] * s_wp, t[1] * s_wp)
        mB = (uout[0] * Lout, uout[1] * Lout)

        pts: List[Tuple[float, float]] = []

        for i in range(n):
            tt = i / float(n)
            pts.append(self.hermite(A, p_wp, mA, mWP, tt))
        pts.append((float(p_wp[0]), float(p_wp[1])))

        for i in range(1, n + 1):
            tt = i / float(n)
            pts.append(self.hermite(p_wp, B, mWP, mB, tt))

        reject_thresh = 50 if check_soft else 100
        for (x, y) in pts:
            occ = self.occ_at_world(gs, grid, x, y)
            if occ is None or occ >= reject_thresh:
                return None

        return pts

    def blend_waypoint_corners(
        self,
        gs: GridSpec,
        grid: List[int],
        poly: List[Tuple[float, float]],
        waypoint_indices: List[int],
    ) -> Tuple[List[Tuple[float, float]], List[int]]:
        if len(poly) < 3:
            return poly, waypoint_indices
        if not bool(self.get_parameter("corner_enable").value):
            return poly, waypoint_indices

        wp_set: Set[int] = set(waypoint_indices)
        last_wp = waypoint_indices[-1] if waypoint_indices else -1

        new_poly: List[Tuple[float, float]] = []
        new_wp_indices: List[int] = []

        for i in range(len(poly)):
            is_wp = i in wp_set
            if i == 0:
                new_poly.append(poly[i])
                if is_wp:
                    new_wp_indices.append(len(new_poly) - 1)
                continue

            if is_wp and 0 < i < len(poly) - 1 and i != last_wp:
                p_prev = poly[i - 1]
                p_wp = poly[i]
                p_next = poly[i + 1]
                blended = self.try_blend_corner_through_waypoint(gs, grid, p_prev, p_wp, p_next)
                if blended is not None:
                    for p in blended:
                        if math.hypot(p[0] - new_poly[-1][0], p[1] - new_poly[-1][1]) < 1e-9:
                            continue
                        new_poly.append(p)
                        if math.hypot(p[0] - p_wp[0], p[1] - p_wp[1]) < 1e-12:
                            new_wp_indices.append(len(new_poly) - 1)
                    continue

            if math.hypot(poly[i][0] - new_poly[-1][0], poly[i][1] - new_poly[-1][1]) >= 1e-9:
                new_poly.append(poly[i])
            if is_wp:
                new_wp_indices.append(len(new_poly) - 1)

        new_wp_indices = sorted(set(new_wp_indices))
        return new_poly, new_wp_indices

    # ---------- Caps ----------
    def build_v_dir_caps(
		self,
		s: List[float],
		xs: List[float],
		ys: List[float],
		v_user_max: float,
		omega_dir_max: float,
		smooth_m: float,
	) -> List[float]:
		n = len(s)
		if n < 3 or omega_dir_max <= 1e-6:
			return [v_user_max] * n

		# Geometric curvature from 3 points:
		# kappa = 4*Area/(a*b*c) = 2*|cross|/(a*b*c)
		kappa = [0.0] * n
		eps = 1e-12

		for i in range(1, n - 1):
			x0, y0 = float(xs[i - 1]), float(ys[i - 1])
			x1, y1 = float(xs[i]), float(ys[i])
			x2, y2 = float(xs[i + 1]), float(ys[i + 1])

			a = math.hypot(x1 - x0, y1 - y0)
			b = math.hypot(x2 - x1, y2 - y1)
			c = math.hypot(x2 - x0, y2 - y0)
			denom = a * b * c
			if denom < eps:
				kappa[i] = 0.0
				continue

			cross = abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))
			kappa[i] = (2.0 * cross) / denom

		kappa[0] = kappa[1]
		kappa[-1] = kappa[-2]

		# Optional smoothing in s-domain (moving average)
		if smooth_m > 1e-6:
			ds_list = [max(s[i + 1] - s[i], 1e-9) for i in range(n - 1)]
			ds_mean = sum(ds_list) / max(1, len(ds_list))
			win = int(max(1, round(smooth_m / max(ds_mean, 1e-9))))
			if win % 2 == 0:
				win += 1
			half = win // 2

			k2 = [0.0] * n
			for i in range(n):
				j0 = max(0, i - half)
				j1 = min(n - 1, i + half)
				k2[i] = sum(kappa[j0:j1 + 1]) / float(j1 - j0 + 1)
			kappa = k2

		out = [v_user_max] * n
		for i in range(n):
			if kappa[i] > 1e-9:
				out[i] = min(v_user_max, omega_dir_max / kappa[i])
		return out

    def _envelope_project_once(self, s: List[float], v: List[float], a: float) -> None:
        n = len(s)
        # backward (braking)
        for i in range(n - 2, -1, -1):
            ds = max(s[i + 1] - s[i], 1e-12)
            v_brake = math.sqrt(max(0.0, v[i + 1] * v[i + 1] + 2.0 * a * ds))
            if v[i] > v_brake:
                v[i] = v_brake
        # forward (accel)
        for i in range(n - 1):
            ds = max(s[i + 1] - s[i], 1e-12)
            v_acc = math.sqrt(max(0.0, v[i] * v[i] + 2.0 * a * ds))
            if v[i + 1] > v_acc:
                v[i + 1] = v_acc

    def _smooth_moving_avg(self, v: List[float], win: int) -> List[float]:
        n = len(v)
        if n == 0:
            return []
        win = int(win)
        if win < 3:
            return v[:]
        if win % 2 == 0:
            win += 1
        half = win // 2
        out = v[:]
        for i in range(n):
            j0 = max(0, i - half)
            j1 = min(n - 1, i + half)
            out[i] = sum(v[j0:j1 + 1]) / float(j1 - j0 + 1)
        return out

    def build_speed_profile_envelope(
        self,
        s: List[float],
        v_lim: List[float],
        a_trans: float,
    ) -> List[float]:
        n = len(s)
        if n == 0:
            return []
        v = [max(0.0, float(x)) for x in v_lim]
        v[-1] = 0.0

        a = max(1e-6, float(a_trans))

        # --- SPEED PROFILE ANTI-BOUNCE FIX ---
        env_iters = int(self.get_parameter("profile_envelope_iters").value)
        smooth_win = int(self.get_parameter("profile_smooth_window").value)
        smooth_iters = int(self.get_parameter("profile_smooth_iters").value)

        # initial projection (repeat to converge a bit)
        for _ in range(max(1, env_iters)):
            self._envelope_project_once(s, v, a)

        # smooth + clamp + re-project cycles (kills accel "bump" between close dips)
        for _ in range(max(0, smooth_iters)):
            v = self._smooth_moving_avg(v, smooth_win)
            for i in range(n):
                if v[i] > v_lim[i]:
                    v[i] = float(v_lim[i])
                if v[i] < 0.0:
                    v[i] = 0.0
            v[-1] = 0.0
            for __ in range(max(1, env_iters)):
                self._envelope_project_once(s, v, a)

        return v

    # ---------- Wheel feasibility ----------
    def k_from_theta_body(self, theta_body: float, r: float) -> Tuple[float, float, float]:
        inv_r = 1.0 / max(r, 1e-12)
        return (
            inv_r * math.cos(theta_body - PHI1),
            inv_r * math.cos(theta_body - PHI2),
            inv_r * math.cos(theta_body - PHI3),
        )

    def wheel_speed_cap_from_k(self, k: Tuple[float, float, float], w_max: float) -> float:
        eps = 1e-12
        caps = []
        for ki in k:
            aki = abs(ki)
            if aki > eps:
                caps.append(w_max / aki)
        return float(min(caps)) if caps else 1e9

    def feasible_v_interval_from_wheel_accel(
        self,
        k_next: Tuple[float, float, float],
        w_prev: Tuple[float, float, float],
        a_wheel_max: float,
        dt: float,
        v_cap: float,
    ) -> Tuple[float, float, bool]:
        a = a_wheel_max * dt
        lo = -1e18
        hi = 1e18
        eps = 1e-12

        for i, ki in enumerate(k_next):
            wi = w_prev[i]
            if abs(ki) < eps:
                if abs(wi) > a + 1e-9:
                    return 0.0, 0.0, False
                continue

            a_i = (wi - a) / ki
            b_i = (wi + a) / ki
            if a_i > b_i:
                a_i, b_i = b_i, a_i
            lo = max(lo, a_i)
            hi = min(hi, b_i)

        lo = max(lo, 0.0)
        hi = min(hi, v_cap)
        return float(lo), float(hi), bool(hi + 1e-12 >= lo)

    # ---------- Interp ----------
    def interp_lin(self, s_arr: List[float], arr: List[float], st: float) -> float:
        if not s_arr:
            return 0.0
        if st <= s_arr[0]:
            return float(arr[0])
        if st >= s_arr[-1]:
            return float(arr[-1])
        j = bisect.bisect_right(s_arr, st) - 1
        j = max(0, min(j, len(s_arr) - 2))
        s0, s1 = s_arr[j], s_arr[j + 1]
        a = (st - s0) / max(s1 - s0, 1e-12)
        return float(arr[j] + a * (arr[j + 1] - arr[j]))

    def interp_ang(self, s_arr: List[float], arr: List[float], st: float) -> float:
        if not s_arr:
            return 0.0
        if st <= s_arr[0]:
            return float(arr[0])
        if st >= s_arr[-1]:
            return float(arr[-1])
        j = bisect.bisect_right(s_arr, st) - 1
        j = max(0, min(j, len(s_arr) - 2))
        s0, s1 = s_arr[j], s_arr[j + 1]
        a = (st - s0) / max(s1 - s0, 1e-12)
        d = wrap_to_pi(arr[j + 1] - arr[j])
        return float(wrap_to_pi(arr[j] + a * d))

    # ---------- Trajectory generation ----------
    def build_dt_trajectory(
        self,
        xs: List[float],
        ys: List[float],
        theta_map: List[float],
        waypoint_s: List[float],
        current_speed: float,
        yaw_now: float,
    ) -> Tuple[List[float], List[float], List[float], List[float]]:
        if len(xs) < 2:
            return [], [], [], []

        dt = float(self.get_parameter("dt").value)
        v_user_max = float(self.get_parameter("v_max").value)
        omega_dir_max = float(self.get_parameter("omega_dir_max").value)
        smooth_m = float(self.get_parameter("omega_dir_lookahead_m").value)

        r = float(self.get_parameter("wheel_radius").value)
        w_max = float(self.get_parameter("max_wheel_speed").value)
        a_wheel_max = float(self.get_parameter("max_wheel_accel").value)
        use_yaw = bool(self.get_parameter("use_yaw_for_wheel_limits").value)

        # Arc-length
        s = [0.0]
        for i in range(1, len(xs)):
            s.append(s[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
        total = s[-1]
        if total < 1e-9:
            return [], [], [], []

		v_dir_cap = self.build_v_dir_caps(s, xs, ys, v_user_max, omega_dir_max, smooth_m)

        # Conservative translation accel envelope (scaled) + dt accel clamp
        a_trans_scale = float(self.get_parameter("a_trans_scale").value)
        a_trans = max(1e-6, (a_wheel_max * r) * max(0.05, a_trans_scale))
        dv_dt_max = a_trans  # m/s^2

        # Per-sample speed limits and smooth envelope v_prof(s)
        v_lim: List[float] = [v_user_max] * len(s)
        for i in range(len(s)):
            th_map = theta_map[i]
            th_body = wrap_to_pi(th_map - yaw_now) if use_yaw else th_map
            kk = self.k_from_theta_body(th_body, r)
            v_lim[i] = min(v_user_max, v_dir_cap[i], self.wheel_speed_cap_from_k(kk, w_max))
        v_prof = self.build_speed_profile_envelope(s, v_lim, a_trans)

        def state_at(st: float) -> Tuple[float, float, float, float, Tuple[float, float, float]]:
            x = self.interp_lin(s, xs, st)
            y = self.interp_lin(s, ys, st)
            th_map = self.interp_ang(s, theta_map, st)
            venv = self.interp_lin(s, v_prof, st)
            th_body = wrap_to_pi(th_map - yaw_now) if use_yaw else th_map
            kk = self.k_from_theta_body(th_body, r)
            return x, y, th_map, venv, kk

        wp_s = sorted([sv for sv in waypoint_s if 0.0 <= sv <= total])
        wp_ptr = 0

        st = 0.0
        x0, y0, th0, venv0, k0 = state_at(st)
        v0 = max(0.0, float(current_speed))
        v0 = min(v0, venv0)
        w_prev = (v0 * k0[0], v0 * k0[1], v0 * k0[2])

        out_x = [x0]
        out_y = [y0]
        out_th = [th0]
        out_v = [v0]

        wp_eps_s = 1e-6
        max_iters = int(max(50.0, (total / max(1e-3, v_user_max)) / max(dt, 1e-4)) * 10.0) + 8000

        for _ in range(max_iters):
            while wp_ptr < len(wp_s) and wp_s[wp_ptr] <= st + wp_eps_s:
                wp_ptr += 1

            remaining = total - st
            if remaining <= 1e-6:
                break

            v_prev = float(out_v[-1])
            next_wp = wp_s[wp_ptr] if wp_ptr < len(wp_s) else None

            # Try to hit the next waypoint exactly in this dt
            if next_wp is not None and next_wp > st + wp_eps_s:
                dist_to_wp = next_wp - st
                v_exact = max(0.0, 2.0 * dist_to_wp / max(dt, 1e-12) - v_prev)

                _, _, _, venv_e, k_e = state_at(next_wp)
                rem_e = total - next_wp

                v_cap_e = min(
                    venv_e,
                    rem_e / max(dt, 1e-12),
                    self.wheel_speed_cap_from_k(k_e, w_max),
                )

                lo_e, hi_e, ok_e = self.feasible_v_interval_from_wheel_accel(
                    k_next=k_e,
                    w_prev=w_prev,
                    a_wheel_max=a_wheel_max,
                    dt=dt,
                    v_cap=v_cap_e,
                )

                # --- dt accel clamp (smoothness) ---
                dv_max = dv_dt_max * dt
                lo_e = max(lo_e, v_prev - dv_max)
                hi_e = min(hi_e, v_prev + dv_max)

                if ok_e and lo_e - 1e-9 <= v_exact <= hi_e + 1e-9 and v_exact <= v_cap_e + 1e-9:
                    st_next = next_wp
                    v_next = float(min(max(v_exact, 0.0), v_cap_e))
                    x, y, th_map, _, k_now = state_at(st_next)
                    w_prev = (v_next * k_now[0], v_next * k_now[1], v_next * k_now[2])

                    out_x.append(x)
                    out_y.append(y)
                    out_th.append(th_map)
                    out_v.append(v_next)
                    st = st_next
                    continue

            # Otherwise: choose a step that does NOT cross the next waypoint.
            v_next = v_prev
            for __ in range(30):
                if next_wp is not None and next_wp > st + wp_eps_s:
                    dist = max(0.0, (next_wp - st) - wp_eps_s)
                    v_cross_cap = max(0.0, 2.0 * dist / max(dt, 1e-12) - v_prev)
                    v_next = min(v_next, v_cross_cap)

                v_next = max(0.0, v_next)

                st_cand = st + 0.5 * (v_prev + v_next) * dt
                st_cand = min(st_cand, total)

                _, _, _, venv_c, k_c = state_at(st_cand)
                rem_c = total - st_cand

                v_cap = min(
                    venv_c,
                    rem_c / max(dt, 1e-12),
                    self.wheel_speed_cap_from_k(k_c, w_max),
                )

                lo, hi, ok = self.feasible_v_interval_from_wheel_accel(
                    k_next=k_c,
                    w_prev=w_prev,
                    a_wheel_max=a_wheel_max,
                    dt=dt,
                    v_cap=v_cap,
                )
                if not ok:
                    v_next *= 0.5
                    continue

                # --- dt accel clamp (smoothness) ---
                dv_max = dv_dt_max * dt
                lo = max(lo, v_prev - dv_max)
                hi = min(hi, v_prev + dv_max)

                if hi + 1e-12 < lo:
                    v_next *= 0.5
                    continue

                v_new = min(v_cap, hi)
                if v_new < lo:
                    v_new = lo

                if abs(v_new - v_next) < 1e-4:
                    v_next = float(v_new)
                    break
                v_next = float(v_new)

            st_next = st + 0.5 * (v_prev + v_next) * dt
            st_next = min(st_next, total)

            if next_wp is not None and st_next > next_wp + wp_eps_s:
                st_next = next_wp
                v_next = max(0.0, 2.0 * (st_next - st) / max(dt, 1e-12) - v_prev)

                # also respect dt accel clamp on this forced truncation
                dv_max = dv_dt_max * dt
                v_next = min(v_next, v_prev + dv_max)
                v_next = max(v_next, max(0.0, v_prev - dv_max))

            if st_next <= st + 1e-12:
                self.get_logger().warn("No progress in dt integrator; truncating trajectory.")
                break

            x, y, th_map, _, k_now = state_at(st_next)
            w_prev = (v_next * k_now[0], v_next * k_now[1], v_next * k_now[2])

            out_x.append(x)
            out_y.append(y)
            out_th.append(th_map)
            out_v.append(float(v_next))
            st = st_next

        # Ensure final sample at goal, then brake tail to 0 (wheel-accel-consistent)
        xT, yT, thT, _, kT = state_at(total)
        if math.hypot(out_x[-1] - xT, out_y[-1] - yT) > 1e-6:
            out_x.append(xT)
            out_y.append(yT)
            out_th.append(thT)
            out_v.append(out_v[-1])

        w_prev = (out_v[-1] * kT[0], out_v[-1] * kT[1], out_v[-1] * kT[2])
        for _ in range(12000):
            if max(abs(w_prev[0]), abs(w_prev[1]), abs(w_prev[2])) <= 1e-3:
                break
            v_cap = min(v_user_max, self.wheel_speed_cap_from_k(kT, w_max))
            lo, hi, ok = self.feasible_v_interval_from_wheel_accel(
                k_next=kT,
                w_prev=w_prev,
                a_wheel_max=a_wheel_max,
                dt=dt,
                v_cap=v_cap,
            )
            if not ok:
                break

            dv_max = dv_dt_max * dt
            lo = max(lo, out_v[-1] - dv_max)  # keep braking smooth too

            v_next = max(0.0, lo)
            w_prev = (v_next * kT[0], v_next * kT[1], v_next * kT[2])

            out_x.append(xT)
            out_y.append(yT)
            out_th.append(thT)
            out_v.append(float(v_next))

        if out_v:
            out_v[-1] = 0.0
        return out_x, out_y, out_th, out_v

    # ---------- Visualization ----------
    def publish_waypoints(self, wps: List[Tuple[float, float]]) -> None:
        ma = MarkerArray()

        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.get_parameter("map_frame").value
        m.ns = "wps"
        m.id = self.marker_counter
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.12
        m.scale.y = 0.12
        m.scale.z = 0.12
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        for (x, y) in wps:
            m.points.append(Point(x=float(x), y=float(y), z=0.05))
        ma.markers.append(m)

        for i, (x, y) in enumerate(wps):
            t = Marker()
            t.header = m.header
            t.ns = "wp_text"
            t.id = 100 + self.marker_counter + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(x)
            t.pose.position.y = float(y)
            t.pose.position.z = 0.18
            t.scale.z = 0.18
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = str(i)
            ma.markers.append(t)

        self.wp_marker_pub.publish(ma)
        self.marker_counter += 1

    # ---------- Main loop ----------
    def on_timer(self) -> None:
        self.consume_add_wp()
        wps = self.read_waypoints()
        if not wps:
            return

        self.publish_waypoints(wps)

        cur_x, cur_y, cur_yaw = self.get_current_pose()
        gs, grid = self.build_costmap(center_xy=(cur_x, cur_y))
        self.publish_costmap(gs, grid)

        stitched: List[Tuple[float, float]] = []
        waypoint_vertex_indices: List[int] = []
        start_xy = (cur_x, cur_y)

        for (gx, gy) in wps:
            goal_xy = (gx, gy)
            seg = self.plan_segment_path(gs, grid, start_xy, goal_xy)
            if not seg:
                self.get_logger().warn(f"Planning failed start={start_xy} goal={goal_xy}")
                return
            if not stitched:
                stitched.extend(seg)
            else:
                stitched.extend(seg[1:])
            waypoint_vertex_indices.append(len(stitched) - 1)
            start_xy = goal_xy

        stitched, waypoint_vertex_indices, _ = self.simplify_polyline_keep_indices(
            stitched, waypoint_vertex_indices
        )

        stitched, waypoint_vertex_indices = self.blend_waypoint_corners(
            gs, grid, stitched, waypoint_vertex_indices
        )

        ds = float(self.get_parameter("ds_geom").value)
        xs, ys, vertex_out = self.resample_vertex_preserving_with_map(stitched, ds)
        if len(xs) < 2:
            return

        ths = self.compute_theta_from_xy(xs, ys)

        s = [0.0]
        for i in range(1, len(xs)):
            s.append(s[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))

        waypoint_s: List[float] = []
        for v_idx in waypoint_vertex_indices:
            if 0 <= v_idx < len(vertex_out):
                out_i = vertex_out[v_idx]
                if 0 <= out_i < len(s):
                    waypoint_s.append(float(s[out_i]))

        self.publish_path(xs, ys, ths)

        tx, ty, tth, tv = self.build_dt_trajectory(
            xs=xs,
            ys=ys,
            theta_map=ths,
            waypoint_s=waypoint_s,
            current_speed=float(self.current_speed),
            yaw_now=float(cur_yaw),
        )
        if len(tx) < 2:
            return

        self.publish_trajectory(tx, ty, tth, tv)

        data = {"x": tx, "y": ty, "theta_v": tth, "v": tv, "yaw": [0.0] * len(tx)}
        with open("/tmp/last_trajectory.json", "w") as f:
            json.dump(data, f)

        self.get_logger().info("Published trajectory (v_prof smoothed + dt accel clamp; no bounce).")

    # ---------- Publishing ----------
    def publish_costmap(self, gs: GridSpec, grid: List[int]) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("map_frame").value
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

        msg.data = [int(v) for v in grid]
        self.costmap_pub.publish(msg)

    def publish_path(self, xs: List[float], ys: List[float], theta_v: List[float]) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter("map_frame").value

        for i in range(len(xs)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(xs[i])
            ps.pose.position.y = float(ys[i])
            ps.pose.position.z = 0.0

            th = float(theta_v[i]) if i < len(theta_v) else 0.0
            q = yaw_to_quat(th)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            path.poses.append(ps)

        self.path_pub.publish(path)

    def publish_trajectory(self, x: List[float], y: List[float], theta_v: List[float], v: List[float]) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter("map_frame").value

        for i in range(len(x)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x[i])
            ps.pose.position.y = float(y[i])
            ps.pose.position.z = 0.0

            th = float(theta_v[i]) if i < len(theta_v) else 0.0
            q = yaw_to_quat(th)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            path.poses.append(ps)

        self.traj_path_pub.publish(path)

        arr = Float32MultiArray()
        arr.data = [float(val) for val in v]
        self.traj_v_pub.publish(arr)

        ma = MarkerArray()
        m = Marker()
        m.header = path.header
        m.ns = "traj_speed"
        m.id = self.marker_counter
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04
        m.color.a = 1.0

        vmax = max(max(v), 1e-6) if v else 1e-6
        for i in range(len(x)):
            p = Point(x=float(x[i]), y=float(y[i]), z=0.02)
            m.points.append(p)

            t = float(v[i]) / float(vmax)
            t = max(0.0, min(1.0, t))
            c = ColorRGBA(r=t, g=0.0, b=1.0 - t, a=1.0)
            m.colors.append(c)

        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.marker_counter += 1


def main() -> None:
    rclpy.init()
    node = WaypointTrajNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

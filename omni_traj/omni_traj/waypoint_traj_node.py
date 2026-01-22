#!/usr/bin/env python3
import math
import heapq
from dataclasses import dataclass
from typing import List, Tuple
import json

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def yaw_to_quat(yaw: float):
    # planar quaternion
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

@dataclass
class GridSpec:
    res: float
    width: int
    height: int
    origin_x: float
    origin_y: float

class WaypointTrajNode(Node):
    def __init__(self):
        super().__init__('waypoint_traj')

        # --- Params (live-tunable)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('v_max', 0.3)

        # Direction-rate limit:
        # omega_dir_max = v_at_radius_ref / turn_radius_ref
        self.declare_parameter('turn_radius_ref', 0.4)
        self.declare_parameter('v_at_radius_ref', 0.3)
        self.declare_parameter('omega_dir_max', -1.0)  # if >0, overrides computed

        self.declare_parameter('ds_geom', 0.03)  # curvature estimation spacing

        # Kiwi drive kinematics
        self.declare_parameter('wheel_radius', 0.09)  # r
        self.declare_parameter('wheel_base', 0.2)     # L
        self.declare_parameter('max_wheel_speed', 12.0)  # rad/s
        self.declare_parameter('max_wheel_accel', 6.0)  # rad/s^2
         # Reserve wheel authority for simultaneous yaw control *only when a waypoint requests a yaw change*
        self.declare_parameter('omega_reserve', 1.0)          # rad/s
        self.declare_parameter('alpha_reserve', 1.0)          # rad/s^2

        self.declare_parameter('yaw_change_threshold', 0.05)  # rad (~3 deg)        
        self.declare_parameter('wheel_speed_margin', 1.0)     # 0..1
        self.declare_parameter('wheel_accel_margin', 1.0)     # 0..1


        self.declare_parameter('map_res', 0.01)
        self.declare_parameter('map_width_m', 3.0)
        self.declare_parameter('map_height_m', 3.0)
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Inflation
        self.declare_parameter('hard_inflate_radius', 0.2)   # optional
        self.declare_parameter('soft_inflate_radius', 0.2)  # soft padding

        # Waypoints: flat list [x1,y1,(yaw1), x2,y2,(yaw2), ...]
        # Accepts either 2*wp_n elements (x,y pairs) or 3*wp_n elements (x,y,yaw triples).
        self.declare_parameter(
            'waypoints',
            [float('nan'), float('nan')],
            ParameterDescriptor(description='Flat list [x1,y1,(yaw1), x2,y2,(yaw2), ...]')
        )

        # Add waypoint command: set to [x,y] or [x,y,yaw] to append; node clears it after consuming
        self.declare_parameter(
            'add_wp',
            [float('nan'), float('nan'), float('nan')],
            ParameterDescriptor(description='Set to [x,y] or [x,y,yaw] to append; node clears it after consuming')
        )

        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('wp_n', 0)

        # LiDAR topics
        self.declare_parameter('lidar1_topic', '/lidar1/scan')
        self.declare_parameter('lidar2_topic', '/lidar2/scan')

        # --- TF (optional; for empty scans it's irrelevant, but later it matters)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Odometry for current velocity
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.current_v = 0.0

        # --- Subscriptions
        t1 = self.get_parameter('lidar1_topic').value
        t2 = self.get_parameter('lidar2_topic').value
        self.sub1 = self.create_subscription(LaserScan, t1, self.on_scan1, 10)
        self.sub2 = self.create_subscription(LaserScan, t2, self.on_scan2, 10)

        self.last_scan1 = None
        self.last_scan2 = None

        # --- Publishers
        self.wp_marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 1)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 1)
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)
        self.traj_path_pub = self.create_publisher(Path, '/trajectory_path', 1)
        self.traj_v_pub = self.create_publisher(Float32MultiArray, '/trajectory_v', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 1)

        # Recompute at 2 Hz for now
        self.timer = self.create_timer(0.5, self.on_timer)

        # Counter to force RViz marker updates on parameter changes
        self.marker_counter = 0

    # ---------- LiDAR callbacks ----------
    def on_scan1(self, msg: LaserScan):
        self.last_scan1 = msg

    def on_scan2(self, msg: LaserScan):
        self.last_scan2 = msg

    def on_odom(self, msg: Odometry):
        # Assume velocity along the robot's forward direction
        self.current_v = msg.twist.twist.linear.x

    # ---------- Utility: world<->grid ----------
    def make_grid_spec(self) -> GridSpec:
        res = float(self.get_parameter('map_res').value)
        w_m = float(self.get_parameter('map_width_m').value)
        h_m = float(self.get_parameter('map_height_m').value)

        width = int(round(w_m / res))
        height = int(round(h_m / res))

        # Fixed map centered around start pose in map_frame
        sp = self.get_parameter('start_pose').value
        cx, cy = float(sp[0]), float(sp[1])
        origin_x = cx - 0.5 * width * res
        origin_y = cy - 0.5 * height * res
        return GridSpec(res=res, width=width, height=height, origin_x=origin_x, origin_y=origin_y)

    def world_to_grid(self, gs: GridSpec, x: float, y: float) -> Tuple[int,int]:
        ix = int(math.floor((x - gs.origin_x) / gs.res))
        iy = int(math.floor((y - gs.origin_y) / gs.res))
        return ix, iy

    def grid_to_world(self, gs: GridSpec, ix: int, iy: int) -> Tuple[float,float]:
        x = gs.origin_x + (ix + 0.5) * gs.res
        y = gs.origin_y + (iy + 0.5) * gs.res
        return x, y

    def in_bounds(self, gs: GridSpec, ix: int, iy: int) -> bool:
        return 0 <= ix < gs.width and 0 <= iy < gs.height

    def idx(self, gs: GridSpec, ix: int, iy: int) -> int:
        return iy * gs.width + ix

    # ---------- Build fused costmap (hard + soft) ----------
    def build_costmap(self) -> Tuple[GridSpec, List[int]]:
        gs = self.make_grid_spec()
        grid = [0] * (gs.width * gs.height)  # 0 free, 50 soft, 100 hard

        # Convert scans to obstacle points in map_frame
        points = []
        if self.last_scan1 is not None:
            points += self.scan_to_points(self.last_scan1, gs)
        if self.last_scan2 is not None:
            points += self.scan_to_points(self.last_scan2, gs)

        # Mark hard obstacles
        for (x, y) in points:
            ix, iy = self.world_to_grid(gs, x, y)
            if self.in_bounds(gs, ix, iy):
                grid[self.idx(gs, ix, iy)] = 100

        # Inflate to soft ring
        soft_r = float(self.get_parameter('soft_inflate_radius').value)
        hard_r = float(self.get_parameter('hard_inflate_radius').value)
        self.inflate(grid, gs, hard_r=hard_r, soft_r=soft_r)

        return gs, grid

    def scan_to_points(self, scan: LaserScan, gs: GridSpec) -> List[Tuple[float,float]]:
        # Empty scan => no points
        # For real scans: transform each hit into map_frame (if TF available)
        pts = []

        frame_map = self.get_parameter('map_frame').value
        scan_frame = scan.header.frame_id

        # Lookup transform map <- scan
        T = None
        try:
            tf = self.tf_buffer.lookup_transform(frame_map, scan_frame, rclpy.time.Time())
            T = tf.transform
        except Exception:
            T = None  # fallback below

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and (scan.range_min <= r <= scan.range_max):
                xs = r * math.cos(angle)
                ys = r * math.sin(angle)
                # z ignored

                if T is None:
                    # fallback: treat scan frame as map frame
                    xm, ym = xs, ys
                else:
                    xm, ym = self.apply_transform_2d(T, xs, ys)

                pts.append((xm, ym))
            angle += scan.angle_increment
        return pts

    def apply_transform_2d(self, tr, x, y):
        # Apply 2D rotation+translation using quaternion
        tx = tr.translation.x
        ty = tr.translation.y
        qx = tr.rotation.x
        qy = tr.rotation.y
        qz = tr.rotation.z
        qw = tr.rotation.w

        # rotation matrix for yaw-only is simplest; but allow general quat -> yaw
        # yaw from quaternion:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        xr = math.cos(yaw) * x - math.sin(yaw) * y
        yr = math.sin(yaw) * x + math.cos(yaw) * y
        return (xr + tx, yr + ty)

    def inflate(self, grid: List[int], gs: GridSpec, hard_r: float, soft_r: float):
        if soft_r <= 1e-6 and hard_r <= 1e-6:
            return
        hard_cells = []
        for iy in range(gs.height):
            for ix in range(gs.width):
                if grid[self.idx(gs, ix, iy)] >= 100:
                    hard_cells.append((ix, iy))

        if not hard_cells:
            return

        hard_rad = int(math.ceil(hard_r / gs.res)) if hard_r > 0 else 0
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

    # ---------- A* ----------
    def astar(self, gs: GridSpec, grid: List[int], start_xy, goal_xy) -> List[Tuple[float,float]]:
        sx, sy = start_xy
        gx, gy = goal_xy
        sxi, syi = self.world_to_grid(gs, sx, sy)
        gxi, gyi = self.world_to_grid(gs, gx, gy)

        if not self.in_bounds(gs, sxi, syi) or not self.in_bounds(gs, gxi, gyi):
            return []
        if grid[self.idx(gs, gxi, gyi)] >= 100:
            return []

        def h(ix, iy):
            return math.hypot(ix - gxi, iy - gyi)

        # 8-neighborhood
        neigh = [(-1,0,1.0), (1,0,1.0), (0,-1,1.0), (0,1,1.0),
                 (-1,-1,math.sqrt(2)), (-1,1,math.sqrt(2)), (1,-1,math.sqrt(2)), (1,1,math.sqrt(2))]

        openq = []
        heapq.heappush(openq, (h(sxi, syi), 0.0, (sxi, syi)))
        came = {}
        gscore = { (sxi, syi): 0.0 }

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

                # Soft cells add penalty (tunable via weight)
                penalty = 0.0
                if occ >= 50:
                    penalty = 3.0

                ng = gcur + w + penalty
                if (nx, ny) not in gscore or ng < gscore[(nx, ny)]:
                    gscore[(nx, ny)] = ng
                    came[(nx, ny)] = (ix, iy)
                    f = ng + h(nx, ny)
                    heapq.heappush(openq, (f, ng, (nx, ny)))

        return []

    def reconstruct(self, gs: GridSpec, came, goal_ij):
        path_ij = [goal_ij]
        cur = goal_ij
        while cur in came:
            cur = came[cur]
            path_ij.append(cur)
        path_ij.reverse()
        return [self.grid_to_world(gs, ix, iy) for (ix, iy) in path_ij]

    def max_dv_accel(self, v1, theta, ds, a_max, r, L):
        if v1 < 1e-6:
            return a_max * ds  # arbitrary large
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        dw_dv = [
            sin_t / r,
            (-0.5 * sin_t + (math.sqrt(3)/2) * cos_t) / r,
            (-0.5 * sin_t - (math.sqrt(3)/2) * cos_t) / r
        ]
        k = max(abs(x) for x in dw_dv)
        if k < 1e-6:
            return 1e9  # no limit
        t = ds / v1
        dv_max = a_max * t / k
        return dv_max

    # ---------- Trajectory generation (dt fixed) ----------
    def build_dt_trajectory(self, path_xy: List[Tuple[float,float]], yaw_by_segment: List[float], reserve_by_segment: List[float], current_v: float):
        if len(path_xy) < 1:
            return [], [], [], []

        dt = float(self.get_parameter('dt').value)
        v_max = float(self.get_parameter('v_max').value)
        ds_geom = float(self.get_parameter('ds_geom').value)

        omega_override = float(self.get_parameter('omega_dir_max').value)
        if omega_override > 0.0:
            omega_dir_max = omega_override
        else:
            Rref = float(self.get_parameter('turn_radius_ref').value)
            vref = float(self.get_parameter('v_at_radius_ref').value)
            omega_dir_max = vref / max(Rref, 1e-6)

        # 1) resample geometry by ds_geom
        xs, ys, yaws, reserves = self.resample_with_yaw(path_xy, yaw_by_segment, reserve_by_segment, ds_geom)

        # 2) compute curvature-based vmax
        # alpha: tangent heading
        alpha = []
        for i in range(len(xs)-1):
            alpha.append(math.atan2(ys[i+1]-ys[i], xs[i+1]-xs[i]))
        if len(alpha) < 2:
            # almost a point / straight
            # output dt samples linearly
            return self.walk_dt(xs, ys, yaws, [v_max]*len(xs), ds_geom, dt)

        thetas = []
        for i in range(len(xs)-1):
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
            thetas.append(math.atan2(dy, dx))
        if thetas:
            thetas.append(thetas[-1])  # for last point

        kappa = [0.0]*len(xs)
        for i in range(1, len(alpha)):
            da = abs(wrap_to_pi(alpha[i] - alpha[i-1]))
            kappa[i] = da / max(ds_geom, 1e-6)

        v_geom = []
        for i in range(len(xs)):
            if kappa[i] < 1e-6:
                v_curve = 1e9
            else:
                v_curve = omega_dir_max / kappa[i]
            v_geom.append(min(v_max, v_curve))

        # Start from current velocity
        v_geom[0] = current_v

        # Adjust for kiwi drive wheel speed limits
        r = float(self.get_parameter('wheel_radius').value)
        L = float(self.get_parameter('wheel_base').value)
        w_max = float(self.get_parameter('max_wheel_speed').value)

        speed_margin = float(self.get_parameter('wheel_speed_margin').value)
        omega_reserve = abs(float(self.get_parameter('omega_reserve').value))
        w_max_nom = w_max * speed_margin
        omega_term = (L * omega_reserve) / max(r, 1e-9)
        for i in range(len(xs)):
            if i < len(xs) - 1:
                dx = xs[i+1] - xs[i]
                dy = ys[i+1] - ys[i]
                dist = math.hypot(dx, dy)
                if dist > 1e-6:
                    theta = math.atan2(dy, dx)
                    vx = v_geom[i] * math.cos(theta)
                    vy = v_geom[i] * math.sin(theta)
                    omega = 0.0  # translation-only; yaw reserve handled via w_max_eff below
                    w1 = (vy + L * omega) / r
                    w2 = (-0.5 * vy + (math.sqrt(3)/2) * vx + L * omega) / r
                    w3 = (-0.5 * vy - (math.sqrt(3)/2) * vx + L * omega) / r
                    max_w = max(abs(w1), abs(w2), abs(w3))
                    # Reserve wheel headroom only on segments where yaw is changing.
                    w_max_eff = max(0.0, w_max_nom - float(reserves[i]) * omega_term)
                    if w_max_eff < 1e-9:
                        v_geom[i] = 0.0
                        continue
                    if max_w > w_max_eff:
                        v_geom[i] *= w_max_eff / max_w

        # Apply acceleration limits with forward and backward pass
        a_max = float(self.get_parameter('max_wheel_accel').value)
        accel_margin = float(self.get_parameter('wheel_accel_margin').value)
        alpha_reserve = abs(float(self.get_parameter('alpha_reserve').value))

        a_max_nom = a_max * accel_margin
        alpha_term = (L * alpha_reserve) / max(r, 1e-9)
        ds = ds_geom

        # Set final velocity to 0
        v_geom[-1] = 0.0

        # Backward pass for deceleration to 0
        for i in range(len(v_geom)-2, -1, -1):
            theta = thetas[i]
            seg_reserve = max(float(reserves[i]), float(reserves[i+1]))
            a_eff = max(0.0, a_max_nom - seg_reserve * alpha_term)
            dv_max_decel = self.max_dv_accel(v_geom[i+1], theta, ds, a_eff, r, L)
            v_geom[i] = min(v_geom[i], v_geom[i+1] + dv_max_decel)

        # Forward pass for acceleration from start
        for i in range(1, len(v_geom)):
            theta = thetas[i - 1]
            seg_reserve = max(float(reserves[i - 1]), float(reserves[i]))
            a_eff = max(0.0, a_max_nom - seg_reserve * alpha_term)
            dv_max_accel = self.max_dv_accel(v_geom[i - 1], theta, ds, a_eff, r, L)
            v_geom[i] = min(v_geom[i], v_geom[i - 1] + dv_max_accel)

        # 3) walk in fixed dt: s += v(s)*dt
        return self.walk_dt(xs, ys, yaws, v_geom, ds_geom, dt)

    def resample_with_yaw(self, path_xy, yaw_by_segment, reserve_by_segment, ds):
        self.get_logger().info(f"resample called with path_xy len {len(path_xy)}, yaw_by_segment len {len(yaw_by_segment)}, reserve_by_segment len {len(reserve_by_segment)}")
        if not yaw_by_segment:
            self.get_logger().error("yaw_by_segment is empty!")
            return [path_xy[0][0]] if path_xy else [], [path_xy[0][1]] if path_xy else [], [0.0], [0.0]        # path_xy is a stitched list; yaw_by_segment is per-point desired yaw already aligned
        # resample by distance using linear interpolation
        xs = [path_xy[0][0]]
        ys = [path_xy[0][1]]
        yaws = [yaw_by_segment[0]]
        reserves = [float(reserve_by_segment[0]) if reserve_by_segment else 0.0]

        # cumulative along original
        s = [0.0]
        for i in range(1, len(path_xy)):
            dx = path_xy[i][0] - path_xy[i-1][0]
            dy = path_xy[i][1] - path_xy[i-1][1]
            s.append(s[-1] + math.hypot(dx, dy))

        total = s[-1]
        if total < 1e-6:
            return xs, ys, yaws, reserves

        # resample targets
        n = int(math.floor(total / ds))
        targets = [k*ds for k in range(1, n+1)]
        j = 0
        for st in targets:
            while j < len(s)-2 and s[j+1] < st:
                j += 1
            s0, s1 = s[j], s[j+1]
            t = (st - s0) / max(s1 - s0, 1e-9)
            x = path_xy[j][0] + t*(path_xy[j+1][0] - path_xy[j][0])
            y = path_xy[j][1] + t*(path_xy[j+1][1] - path_xy[j][1])
            # yaw desired: just interpolate linearly (or hold) — yaw is decoupled anyway
            yaw = yaw_by_segment[j] + t*wrap_to_pi(yaw_by_segment[j+1] - yaw_by_segment[j])
            rsv = 0.0
            if reserve_by_segment:
                rsv = max(float(reserve_by_segment[j]), float(reserve_by_segment[j+1]))
            xs.append(x); ys.append(y); yaws.append(yaw); reserves.append(rsv)

        # ensure final point
        xs.append(path_xy[-1][0]); ys.append(path_xy[-1][1]); yaws.append(yaw_by_segment[-1])
        reserves.append(float(reserve_by_segment[-1]) if reserve_by_segment else 0.0)
        return xs, ys, yaws, reserves

    def walk_dt(self, xs, ys, yaws, v_geom, ds_geom, dt):
        # v_geom defined at geom samples spaced ~ds_geom. We'll step along them.
        # Build cumulative s for geom samples
        s = [0.0]
        for i in range(1, len(xs)):
            s.append(s[-1] + math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]))
        total = s[-1]
        if total < 1e-6:
            return [], [], [], []

        # Helper: interpolate by s
        def interp(arr, st):
            # find segment
            j = 0
            # linear scan is OK for small; optimize later if needed
            while j < len(s)-2 and s[j+1] < st:
                j += 1
            s0, s1 = s[j], s[j+1]
            t = (st - s0) / max(s1 - s0, 1e-9)
            return arr[j] + t*(arr[j+1]-arr[j])

        # For yaw, wrap-safe interpolation
        def interp_yaw(st):
            j = 0
            while j < len(s)-2 and s[j+1] < st:
                j += 1
            s0, s1 = s[j], s[j+1]
            t = (st - s0) / max(s1 - s0, 1e-9)
            dy = wrap_to_pi(yaws[j+1] - yaws[j])
            return wrap_to_pi(yaws[j] + t*dy)

        # dt samples
        out_x = []
        out_y = []
        out_yaw = []
        out_v = []

        st = 0.0
        # initial
        out_x.append(xs[0]); out_y.append(ys[0]); out_yaw.append(yaws[0]); out_v.append(interp(v_geom, 0.0))

        while st < total - 1e-6:
            v = max(interp(v_geom, st), 0.01)  # avoid zero step
            st_next = min(st + v*dt, total)
            x = interp(xs, st_next)
            y = interp(ys, st_next)
            yaw = interp_yaw(st_next)
            v_cmd = interp(v_geom, st_next)

            out_x.append(x); out_y.append(y); out_yaw.append(yaw); out_v.append(v_cmd)
            st = st_next

        # final point speed = 0 (optional)
        out_v[-1] = 0.0
        return out_x, out_y, out_yaw, out_v

    def publish_waypoints(self, wps_xy):
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.get_parameter('map_frame').value
        m.ns = "wps"
        m.id = self.marker_counter
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.12
        m.scale.y = 0.12
        m.scale.z = 0.12
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        for wp in wps_xy:
            x, y = wp[0], wp[1]   # <-- FIX: works for (x,y) or (x,y,yaw)
            m.points.append(Point(x=float(x), y=float(y), z=0.05))

        ma.markers.append(m)

        for i, wp in enumerate(wps_xy):
            x, y = wp[0], wp[1]   # <-- FIX
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
    def on_timer(self):
        gs, grid = self.build_costmap()
        self.publish_costmap(gs, grid)

        # Handle add_waypoint command
        add_wp = self.get_parameter('add_wp').value
        if add_wp and len(add_wp) in (2, 3) and math.isfinite(add_wp[0]) and math.isfinite(add_wp[1]) and (len(add_wp) == 2 or math.isfinite(add_wp[2])):
            vals = [float(add_wp[0]), float(add_wp[1])] if len(add_wp) == 2 else [float(add_wp[0]), float(add_wp[1]), float(add_wp[2])]
            wp_flat = list(self.get_parameter('waypoints').value)
            wp_n = int(self.get_parameter('wp_n').value)

            # Infer current stride from wp_flat/wp_n; if unknown, default to vals stride.
            stride = None
            if wp_n > 0:
                if len(wp_flat) == 2 * wp_n:
                    stride = 2
                elif len(wp_flat) == 3 * wp_n:
                    stride = 3
            if stride is None:
                stride = len(vals)

            # Upgrade existing (x,y) list to (x,y,yaw) if user now adds yaw.
            if stride == 2 and len(vals) == 3 and wp_n > 0:
                upgraded = []
                for i in range(wp_n):
                    upgraded.extend([float(wp_flat[2*i]), float(wp_flat[2*i+1]), 0.0])
                wp_flat = upgraded
                stride = 3

            # Keep representation consistent.
            if stride == 3 and len(vals) == 2:
                vals = [vals[0], vals[1], 0.0]

            if wp_n == 0:
                wp_flat = vals
            else:
                wp_flat.extend(vals)

            new_wp_n = wp_n + 1
            self.set_parameters([
                Parameter('waypoints', Parameter.Type.DOUBLE_ARRAY, wp_flat),
                Parameter('wp_n', Parameter.Type.INTEGER, new_wp_n),
                Parameter('add_wp', Parameter.Type.DOUBLE_ARRAY, [float('nan'), float('nan'), float('nan')])
            ])
            self.get_logger().info(f"Added waypoint {vals}, wp_n now {new_wp_n}")

        # Build waypoint list
        wp_n = int(self.get_parameter('wp_n').value)
        if wp_n == 0:
            return  # no trajectory
        wp_flat = list(self.get_parameter('waypoints').value)

        if len(wp_flat) == 2 * wp_n:
            stride = 2
        elif len(wp_flat) == 3 * wp_n:
            stride = 3
        else:
            self.get_logger().warn(
                f"Waypoints parameter length {len(wp_flat)} doesn't match wp_n={wp_n} "
                f"(expected 2*wp_n or 3*wp_n). Skipping planning."
            )
            return

        wps = []
        for i in range(wp_n):
            x = float(wp_flat[stride * i])
            y = float(wp_flat[stride * i + 1])
            yaw = float(wp_flat[stride * i + 2]) if stride == 3 else 0.0
            wps.append((x, y, yaw))

        self.get_logger().info(f"Planning to waypoints: {wps}")
        self.publish_waypoints(wps)

        # Try to obtain current robot pose in map frame via TF. Fallback to
        # the `start_pose` parameter if TF is unavailable.
        cur = None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('map_frame').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time()
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            cur = (float(t.x), float(t.y), float(yaw))
        except Exception:
            sp = self.get_parameter('start_pose').value
            cur = (float(sp[0]), float(sp[1]), float(sp[2]))

        self.get_logger().info(f"Current pose: {cur}")

        stitched_xy = []
        yaw_by_point = []
        reserve_by_point = []

        # Reserve yaw authority only on segments whose target waypoint requests a yaw change.
        yaw_thresh = float(self.get_parameter('yaw_change_threshold').value)
        yaw_prev = float(cur[2])
        yaw_change_flags: List[bool] = []
        for (_, _, gyaw) in wps:
            yaw_change_flags.append(abs(wrap_to_pi(float(gyaw) - yaw_prev)) > yaw_thresh)
            yaw_prev = float(gyaw)

        # Plan sequentially to each waypoint
        start_xy = (cur[0], cur[1])
        for seg_idx, (gx, gy, gyaw) in enumerate(wps):
            # For omni robot, use direct line to waypoint (assuming no obstacles)
            start_x, start_y = start_xy
            if (start_x, start_y) == (gx, gy):
                seg = [(start_x, start_y)]
            else:
                seg = [(start_x, start_y), (gx, gy)]
            if not seg:
                self.get_logger().warn(f"No path to ({gx:.2f},{gy:.2f})")
                return

            pts = seg if not stitched_xy else seg[1:]  # avoid duplicating first point
            for p in pts:
                stitched_xy.append(p)
                yaw_by_point.append(float(gyaw))  # viz only; controller uses waypoint yaw directly
                reserve_by_point.append(1.0 if yaw_change_flags[seg_idx] else 0.0)
            start_xy = (gx, gy)

        self.publish_path('/planned_path', stitched_xy, yaw_by_point)

        # Build dt trajectory
        tx, ty, tyaw, tv = self.build_dt_trajectory(stitched_xy, yaw_by_point, reserve_by_point, self.current_v)
        if len(tx) < 2:
            return

        self.publish_trajectory(tx, ty, tyaw, tv)
        self.get_logger().info("Published trajectory")

    # ---------- Publishing ----------
    def publish_costmap(self, gs: GridSpec, grid: List[int]):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.info.resolution = gs.res
        msg.info.width = gs.width
        msg.info.height = gs.height
        msg.info.origin.position.x = gs.origin_x
        msg.info.origin.position.y = gs.origin_y
        msg.info.origin.position.z = 0.0

        q = yaw_to_quat(0.0)  # <-- FIX: don't use undefined yaw
        msg.info.origin.orientation.x = q[0]
        msg.info.origin.orientation.y = q[1]
        msg.info.origin.orientation.z = q[2]
        msg.info.origin.orientation.w = q[3]

        msg.data = [int(v) for v in grid]
        self.costmap_pub.publish(msg)


    def publish_path(self, topic_name: str, xy: List[Tuple[float, float]], yaw_list: List[float]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter('map_frame').value

        for i, (x, y) in enumerate(xy):  # <-- FIX: iterate xy + define i
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0

            yaw = yaw_list[min(i, len(yaw_list) - 1)]
            q = yaw_to_quat(yaw)  # <-- FIX: show yaw in RViz (optional but correct)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]

            path.poses.append(ps)

        self.path_pub.publish(path)

    def publish_trajectory(self, x, y, yaw, v):
        # Trajectory as Path + parallel speed array
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter('map_frame').value

        for i in range(len(x)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x[i])
            ps.pose.position.y = float(y[i])
            ps.pose.position.z = 0.0
            q = yaw_to_quat(yaw[i])
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            path.poses.append(ps)

        self.traj_path_pub.publish(path)

        arr = Float32MultiArray()
        arr.data = [float(val) for val in v]
        self.traj_v_pub.publish(arr)

        # Markers for RViz
        ma = MarkerArray()
        m = Marker()
        m.header = path.header
        m.ns = "traj_speed"
        m.id = self.marker_counter
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04

        # IMPORTANT: still set alpha somewhere (RViz uses either m.color or per-point colors)
        m.color.a = 1.0

        vmin = 0.0
        vmax = max(max(v), 1e-6)

        for i in range(len(x)):
            p = Point()
            p.x = float(x[i])
            p.y = float(y[i])
            p.z = 0.02
            m.points.append(p)

            # normalize speed 0..1
            t = float(v[i] - vmin) / float(vmax - vmin + 1e-9)
            t = max(0.0, min(1.0, t))

            # simple blue->red gradient:
            c = ColorRGBA()
            c.r = t
            c.g = 0.0
            c.b = 1.0 - t
            c.a = 1.0
            m.colors.append(c)

        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.marker_counter += 1

        # Save trajectory to file for debugging
        data = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'v': v
        }
        with open('/tmp/last_trajectory.json', 'w') as f:
            json.dump(data, f)

def main():
    rclpy.init()
    node = WaypointTrajNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

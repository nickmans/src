"""
UDP Server for OMNI robot communication with STM32 NUCLEO H755.

Receives POSE and CMD messages over UDP and sends TRAJ messages
back to the last-seen client address. Uses the same binary protocol
helpers in `protocol.py` so wire format is unchanged.
"""

import asyncio
import logging
import math
import socket
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable, List, Tuple

from protocol import (
    CommandID,
    MessageType,
    Header,
    Pose,
    Command,
    Trajectory,
    make_message,
    HEADER_SIZE,
    StreamParser,
)
from ros2_manager import ROS2Manager
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros2_pose_node import PosePublisherNode
from nav_msgs.msg import Path

logger = logging.getLogger(__name__)


@dataclass
class PoseData:
    pose_t_ms: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    received_t_ms: int


class OMNIUDPServer:
    """Simple UDP server that speaks the OMNI framed protocol over datagrams.

    - Binds to host:port and listens for incoming datagrams.
    - Remembers the last client address seen and will send TRAJ messages
      to that address when `set_trajectory_active(True)`.
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9000,
        on_pose_callback: Optional[Callable[[PoseData], None]] = None,
        on_cmd_callback: Optional[Callable[[int], None]] = None,
        get_trajectory_callback: Optional[Callable[[], Optional[Trajectory]]] = None,
    ):
        self.host = host
        self.port = port
        self.on_pose_callback = on_pose_callback
        self.on_cmd_callback = on_cmd_callback
        self.get_trajectory_callback = get_trajectory_callback

        self.sock: Optional[socket.socket] = None
        self.running = False
        self.recv_thread: Optional[threading.Thread] = None
        self.send_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ros2_retry_thread: Optional[threading.Thread] = None
        self._ros2_retry_interval_s = 5.0
        self._ros2_retry_enabled = False
        self._ros2_retry_lock = threading.Lock()

        self.client_addr: Optional[tuple] = None
        self.pose_lock = threading.Lock()
        self.latest_pose: Optional[PoseData] = None
        self._last_pose_seq: int = 0

        self.traj_lock = threading.Lock()
        self.trajectory_active = False
        self.traj_seq = 0
        self._shutdown_requested = False

        # Reuse stream parser for robustness even though UDP preserves datagram
        self.parser = StreamParser()

        # ROS2 bridge (pose publisher + trajectory generator) runs inside this process
        self.ros2_bridge = ROS2Bridge()

        # Default trajectory callback uses ROS2 bridge if none provided
        self.get_trajectory_callback = get_trajectory_callback or self._default_get_trajectory

        # ROS2 manager runs on a dedicated asyncio loop
        self.ros2_mgr = ROS2Manager(launch_file="dual_sllidar_with_mock_and_traj.launch.py", package="omni_traj")
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        logger.info(f"OMNIUDPServer initialized: {self.host}:{self.port}")

    def start(self):
        if self.running:
            logger.warning("Server already running")
            return

        self.running = True
        self._stop_event.clear()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(1.0)

        logger.info(f"UDP server listening on {self.host}:{self.port}")

        self._start_ros2_retry_worker()

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

    def stop(self):
        logger.info("Stopping UDP server...")
        self.running = False
        self._stop_event.set()
        self._set_ros2_retry_enabled(False, reason="server stopping")

        # Stop ROS2 stack first while manager loop is still alive.
        _ = self._run_ros2(self.ros2_mgr.stop)

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=1.0)

        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        if self._ros2_retry_thread and self._ros2_retry_thread.is_alive():
            self._ros2_retry_thread.join(timeout=1.0)

        # Stop ROS2 bridge
        self.ros2_bridge.shutdown()

        logger.info("UDP server stopped")

    def set_trajectory_active(self, active: bool):
        with self.traj_lock:
            self.trajectory_active = active
        logger.info(f"Trajectory sending: {'active' if active else 'inactive'}")

    def get_latest_pose(self) -> Optional[PoseData]:
        with self.pose_lock:
            return self.latest_pose

    def _recv_loop(self):
        while self.running:
            try:
                if not self.sock:
                    break
                data, addr = self.sock.recvfrom(4096)
                # Remember client address
                self.client_addr = addr

                # Feed parser (preserves compatibility with framed wire format)
                self.parser.feed(data)

                # Parse all complete messages from this datagram
                while True:
                    result = self.parser.parse_message()
                    if result is None:
                        break
                    header, payload = result
                    self._handle_message(header, payload, addr)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"Error in recv loop: {e}")
                break

    def _send_loop(self):
        send_interval = 0.2  # 5 Hz
        idle_interval = 5.0

        while self.running:
            try:
                should_send = False
                with self.traj_lock:
                    should_send = self.trajectory_active

                if should_send and self.get_trajectory_callback and self.client_addr:
                    traj = self.get_trajectory_callback()
                    if traj:
                        self._send_trajectory(traj, self.client_addr)
                    self._stop_event.wait(send_interval)
                else:
                    self._stop_event.wait(idle_interval)

            except Exception as e:
                if self.running:
                    logger.error(f"Error in send loop: {e}")
                break

    def _handle_message(self, header: Header, payload: bytes, addr: tuple):
        try:
            if header.msg_type == MessageType.POSE:
                self._handle_pose(header, payload, addr)
            elif header.msg_type == MessageType.CMD:
                self._handle_cmd(header, payload, addr)
            else:
                logger.warning(f"Unknown message type: {header.msg_type} from {addr}. Expected one of: {[e.value for e in MessageType]}")
        except Exception as e:
            logger.error(f"Error handling message: {e}")

    def _handle_pose(self, header: Header, payload: bytes, addr: tuple):
        pose = Pose.unpack(payload)
        now_ms = int(time.time() * 1000)
        pose_data = PoseData(
            pose_t_ms=pose.pose_t_ms if hasattr(pose, 'pose_t_ms') else 0,
            x=pose.x,
            y=pose.y,
            yaw=pose.yaw,
            vx=getattr(pose, 'vx', 0.0),
            vy=getattr(pose, 'vy', 0.0),
            wz=getattr(pose, 'wz', 0.0),
            received_t_ms=now_ms
        )
        if header.seq % 25 == 0:
            logger.debug(
                f"POSE seq={header.seq} from {addr}: x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.3f}"
            )

        with self.pose_lock:
            self.latest_pose = pose_data
            self._last_pose_seq = header.seq

        if self.on_pose_callback:
            self.on_pose_callback(pose_data)

        # Publish pose into ROS2 so downstream nodes see STM32 odom
        self.ros2_bridge.publish_pose(pose_data)

    def _send_cmd_ack(self, seq: int, addr: tuple, cmd_id: int) -> None:
        """Send a CMD-frame ack back to the STM32.

        STM32-side expects:
        - msg_type = CMD
        - seq matches the original command seq
        - payload contains at least cmd_id (uint16) + arg_len (uint16)
        """
        try:
            payload = Command(cmd_id=cmd_id, arg=b"").pack()
            message = make_message(MessageType.CMD, seq, payload, crc_payload=False)
            if self.sock:
                self.sock.sendto(message, addr)
        except Exception as exc:
            logger.error(f"Failed to send CMD ack: {exc}")

    def _handle_cmd(self, header: Header, payload: bytes, addr: tuple):
        cmd = Command.unpack(payload)
        logger.info(f"CMD received: command={cmd.cmd_id} (seq={header.seq}) from {addr}")
        if self.on_cmd_callback:
            self.on_cmd_callback(cmd.cmd_id)

        # STM32 semantics:
        #   traj 1 -> enable trajectory generation/sending
        #   traj 0 -> disable trajectory generation/sending
        #   traj2 2 -> start/restart ROS2 stack
        if cmd.cmd_id == CommandID.START_TRAJ:
            logger.info("START_TRAJ received; enabling trajectory output")
            self.set_trajectory_active(True)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.STOP_TRAJ:
            logger.info("STOP_TRAJ received; disabling trajectory output")
            self.set_trajectory_active(False)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.START_RESTART_ROS2:
            logger.info("START_RESTART_ROS2 received; restarting ROS2 stack")
            # Keep current trajectory_active state, but pause sending while restart happens.
            with self.traj_lock:
                resume_traj = bool(self.trajectory_active)
            self.set_trajectory_active(False)

            stopped = self._run_ros2(self.ros2_mgr.stop)
            time.sleep(0.5)
            started = self._run_ros2(self.ros2_mgr.start)

            if started:
                self._set_ros2_retry_enabled(False, reason="ROS2 stack started")
            else:
                self._set_ros2_retry_enabled(True, reason="ROS2 start failed; enabling periodic retry")

            if started and resume_traj:
                self.set_trajectory_active(True)

            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.STOP_ROS2:
            logger.info("STOP_ROS2 received; stopping ROS2 stack")
            self.set_trajectory_active(False)
            self._set_ros2_retry_enabled(False, reason="STOP_ROS2 command received")
            _ = self._run_ros2(self.ros2_mgr.stop)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.SHUTDOWN_PI5:
            logger.warning("SHUTDOWN_PI5 received; issuing sudo poweroff")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self._trigger_poweroff_command()
            return

        logger.warning(f"Unhandled CMD id={cmd.cmd_id}; acking")
        self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))

    def _trigger_poweroff_command(self) -> None:
        if self._shutdown_requested:
            logger.warning("Shutdown already requested; ignoring duplicate SHUTDOWN_PI5")
            return

        self._shutdown_requested = True
        shutdown_thread = threading.Thread(target=self._execute_poweroff_command, daemon=True)
        shutdown_thread.start()

    def _execute_poweroff_command(self) -> None:
        try:
            time.sleep(0.2)
            subprocess.Popen(
                ["bash", "-lc", "sudo poweroff"],
                start_new_session=True,
            )
            logger.warning("Poweroff command launched: sudo poweroff")
        except Exception as exc:
            logger.error(f"Failed to launch poweroff command: {exc}")
            self._shutdown_requested = False

    def _send_trajectory(self, traj: Trajectory, addr: tuple):
        try:
            payload = traj.pack()
            self.traj_seq += 1
            message = make_message(MessageType.TRAJ, self.traj_seq, payload, crc_payload=False)
            if self.sock:
                self.sock.sendto(message, addr)

            if self.traj_seq % 25 == 0:
                logger.debug(
                    f"TRAJ sent seq={self.traj_seq} to {addr}: knots={len(traj.knots)}"
                )
        except Exception as e:
            logger.error(f"Error sending TRAJ: {e}")

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_ros2(self, coro_factory) -> bool:
        try:
            future = asyncio.run_coroutine_threadsafe(coro_factory(), self._loop)
            # Stop may take longer (SIGINT -> SIGTERM escalation), so allow ample time.
            return future.result(timeout=15.0)
        except Exception as e:
            logger.error(f"ROS2 command failed: {e}")
            return False

    def _start_ros2_retry_worker(self) -> None:
        if self._ros2_retry_thread and self._ros2_retry_thread.is_alive():
            return

        self._ros2_retry_thread = threading.Thread(
            target=self._ros2_retry_loop,
            name="ros2_retry_worker",
            daemon=True,
        )
        self._ros2_retry_thread.start()

    def _set_ros2_retry_enabled(self, enabled: bool, reason: str = "") -> None:
        with self._ros2_retry_lock:
            if self._ros2_retry_enabled == enabled:
                return
            self._ros2_retry_enabled = enabled

        if enabled:
            logger.warning("ROS2 retry enabled: %s", reason or "start failures")
        else:
            logger.info("ROS2 retry disabled: %s", reason or "")

    def _is_ros2_retry_enabled(self) -> bool:
        with self._ros2_retry_lock:
            return self._ros2_retry_enabled

    def _ros2_retry_loop(self) -> None:
        while not self._stop_event.wait(self._ros2_retry_interval_s):
            if not self.running:
                continue
            if not self._is_ros2_retry_enabled():
                continue

            is_running = self._run_ros2(self.ros2_mgr.is_running)
            if is_running:
                self._set_ros2_retry_enabled(False, reason="stack is running")
                continue

            logger.warning("Retrying ROS2 stack startup...")
            started = self._run_ros2(self.ros2_mgr.start)
            if started:
                self._set_ros2_retry_enabled(False, reason="retry startup succeeded")

    def _default_get_trajectory(self) -> Optional[Trajectory]:
        """Convert the latest ROS2 planned path (/planned_path) into UDP trajectory knots."""
        path_points = self.ros2_bridge.get_planned_path_points(max_points=64)
        path_velocities = self.ros2_bridge.get_planned_path_velocities(max_points=64)
        
        if not path_points:
            return None

        dt = 0.2
        knots: List[Tuple[float, float, float, float, float]] = []

        prev_x = None
        prev_y = None
        prev_yaw = 0.0
        
        for idx, (x, y, yaw_opt) in enumerate(path_points):
            if yaw_opt is None:
                if prev_x is not None and prev_y is not None:
                    dx = x - prev_x
                    dy = y - prev_y
                    if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                        yaw = math.atan2(dy, dx)
                    else:
                        yaw = prev_yaw
                else:
                    yaw = prev_yaw
            else:
                yaw = float(yaw_opt)

            # Use velocities from trajectory if available, otherwise calculate from position
            if idx < len(path_velocities):
                vx, vy = path_velocities[idx]
            else:
                # Fallback: calculate from position difference
                if prev_x is None:
                    vx, vy = 0.0, 0.0
                else:
                    dx = x - prev_x
                    dy = y - prev_y
                    dist = math.hypot(dx, dy)
                    velocity = min(0.6, dist / dt)
                    if dist > 1e-9:
                        vx = velocity * (dx / dist)
                        vy = velocity * (dy / dist)
                    else:
                        vx, vy = 0.0, 0.0

            knots.append((float(x), float(y), float(yaw), float(vx), float(vy)))
            prev_x, prev_y, prev_yaw = x, y, yaw

        return Trajectory(
            reply_to_pose_seq=self._last_pose_seq,
            traj_t0_ms=int(time.time() * 1000),
            dt=dt,
            knots=knots,
            flags=0,
        )


class ROS2Bridge:
    """Lightweight ROS2 bridge.

    - Publishes STM32 pose into ROS2 (/robot/pose and /odom via PosePublisherNode)
    - Subscribes to /planned_path (from omni_traj waypoint_traj) so we can send
      knots back to STM32 when trajectory output is enabled.
    """

    def __init__(self) -> None:
        rclpy.init(args=None)
        self.pose_node = PosePublisherNode()

        self.path_lock = threading.Lock()
        self.latest_path: Optional[Path] = None
        self.latest_velocities: Optional[List[Tuple[float, float]]] = None

        self.path_node = rclpy.create_node("planned_path_sub")
        self.path_sub = self.path_node.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.vel_sub = self.path_node.create_subscription(
            Float64MultiArray, "/planned_path_velocities", self._on_velocities, 10
        )

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.pose_node)
        self.executor.add_node(self.path_node)
        self.running = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def publish_pose(self, pose: PoseData) -> None:
        try:
            self.pose_node.publish_pose(pose)
        except Exception as exc:
            logger.error(f"Failed to publish pose to ROS2: {exc}")

    def _on_path(self, msg: Path) -> None:
        with self.path_lock:
            self.latest_path = msg

    def _on_velocities(self, msg: Float64MultiArray) -> None:
        """Receive velocities as [vx0, vy0, vx1, vy1, ...]"""
        with self.path_lock:
            vels = []
            for i in range(0, len(msg.data), 2):
                if i + 1 < len(msg.data):
                    vels.append((float(msg.data[i]), float(msg.data[i + 1])))
            self.latest_velocities = vels if vels else None

    def get_planned_path_points(self, max_points: int = 64) -> List[Tuple[float, float, Optional[float]]]:
        """Return [(x,y,yaw|None), ...] from the latest /planned_path."""
        with self.path_lock:
            path = self.latest_path

        if path is None or not path.poses:
            return []

        pts: List[Tuple[float, float, Optional[float]]] = []
        for ps in path.poses[:max_points]:
            x = ps.pose.position.x
            y = ps.pose.position.y
            q = ps.pose.orientation
            # Some publishers leave orientation all-zeros; treat that as unknown.
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                yaw = None
            else:
                # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
            pts.append((float(x), float(y), yaw))
        return pts

    def get_planned_path_velocities(self, max_points: int = 64) -> List[Tuple[float, float]]:
        """Return [(vx, vy), ...] from the latest /planned_path_velocities."""
        with self.path_lock:
            vels = self.latest_velocities

        if vels is None:
            return []

        return vels[:max_points]

    def shutdown(self) -> None:
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.pose_node.destroy_node()
            self.path_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    def on_pose(pose: PoseData):
        pass

    def on_cmd(cmd_id: int):
        logger.info(f"Command callback: {cmd_id}")

    def get_traj() -> Optional[Trajectory]:
        # simple hold position for testing
        knots = [(0.0, 0.0, 0.0, 0.0, 0.0)]
        return Trajectory(reply_to_pose_seq=0, traj_t0_ms=0, dt=0.1, knots=knots)

    server = OMNIUDPServer(host="0.0.0.0", port=9000, on_pose_callback=on_pose, on_cmd_callback=on_cmd, get_trajectory_callback=get_traj)

    try:
        server.start()
        logger.info("UDP server running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        server.stop()


if __name__ == "__main__":
    main()

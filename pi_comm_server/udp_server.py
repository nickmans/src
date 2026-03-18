"""
UDP Server for OMNI robot communication with STM32 NUCLEO H755.

Receives POSE and CMD messages over UDP and sends TRAJ messages
back to the last-seen client address. Uses the same binary protocol
helpers in `protocol.py` so wire format is unchanged.
"""

import asyncio
import fcntl
import logging
import math
import os
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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

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
        self._send_wakeup = threading.Event()
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
        self._active_traj_signature: Optional[Tuple[int, float, float, float, float]] = None
        self._active_traj_t0_ms: Optional[int] = None
        self._hold_until_ms: int = 0
        self._hold_traj_t0_ms: Optional[int] = None
        self._hold_knot: Optional[Tuple[float, float, float, float, float]] = None
        self._last_valid_traj: Optional[Trajectory] = None

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
        self._active_stack_mode = "standby"
        self._stack_transition_lock = threading.Lock()
        self._stack_lock_path = "/tmp/omni_ros2_stack_controller.lock"
        self._stack_lock_fd: Optional[int] = None
        self._stack_control_enabled = True
        self._server_lock_path = "/tmp/omni_udp_server.lock"
        self._server_lock_fd: Optional[int] = None

        logger.info(f"OMNIUDPServer initialized: {self.host}:{self.port}")

    def _switch_stack_mode(self, stack_mode: str) -> bool:
        if not self._stack_control_enabled:
            logger.warning(f"Stack lifecycle control disabled; cannot switch to mode={stack_mode}")
            return False

        target_mode = (stack_mode or "").strip().lower()
        if target_mode not in {"standby", "mapping", "localization"}:
            logger.error(f"Invalid stack mode request: {stack_mode}")
            return False

        with self._stack_transition_lock:
            self._set_ros2_retry_enabled(False, reason=f"mode transition to {target_mode}")

            try:
                changed = self.ros2_mgr.set_stack_mode(target_mode)
            except Exception as exc:
                logger.error(f"Failed to set ROS2 stack mode '{target_mode}': {exc}")
                return False

            is_running = self._run_ros2(self.ros2_mgr.is_running)
            needs_restart = True

            if is_running and not changed:
                async def _healthy_check() -> bool:
                    return await self.ros2_mgr.is_mode_healthy(target_mode)

                if self._run_ros2(_healthy_check):
                    logger.info(f"ROS2 stack already healthy in {target_mode} mode")
                    self._active_stack_mode = target_mode
                    return True
                logger.warning(f"ROS2 stack running but unhealthy in {target_mode} mode; forcing restart")

            if is_running and needs_restart:
                logger.info(f"Switching ROS2 stack from {self._active_stack_mode} to {target_mode}")
                stopped = self._run_ros2(self.ros2_mgr.stop)
                if not stopped:
                    logger.error("Failed to stop ROS2 stack during mode switch")
                    self._set_ros2_retry_enabled(True, reason="stop failed during transition")
                    return False
                time.sleep(0.5)

            started = self._run_ros2(self.ros2_mgr.start)
            if not started:
                logger.error(f"Failed to start ROS2 stack in {target_mode} mode")
                self._set_ros2_retry_enabled(True, reason="start failed during transition")
                return False

            self._active_stack_mode = target_mode
            self._set_ros2_retry_enabled(False, reason="mode transition complete")
            return True

    def start(self):
        if self.running:
            logger.warning("Server already running")
            return

        if not self._acquire_server_instance_lock():
            raise RuntimeError("UDP server instance lock is already held")

        if not self._acquire_stack_controller_lock():
            self._stack_control_enabled = False
            logger.warning(
                "ROS2 stack controller lock is already held; starting UDP server in bridge-only mode "
                "(pose/commands active, stack start/stop/mode-switch disabled)"
            )
        else:
            self._stack_control_enabled = True

        self.running = True
        self._stop_event.clear()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.host, self.port))
        except OSError as exc:
            self.running = False
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self._release_stack_controller_lock()
            self._release_server_instance_lock()
            raise RuntimeError(f"Failed to bind UDP socket {self.host}:{self.port}: {exc}") from exc
        self.sock.settimeout(2.0)  # Increased from 1.0 to reduce CPU wake-ups

        logger.info(f"UDP server listening on {self.host}:{self.port}")

        self._start_ros2_retry_worker()

        # Required default flow: startup in local-costmap-only standby mode.
        if self._stack_control_enabled:
            ok_stack = self._switch_stack_mode("standby")
            if not ok_stack:
                logger.warning("Startup standby mode switch failed; enabling retry worker")
                self._set_ros2_retry_enabled(True, reason="startup standby failed")

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

    def stop(self, stop_ros2_stack: bool = True):
        logger.info("Stopping UDP server...")
        self.running = False
        self._stop_event.set()
        self._send_wakeup.set()
        self._set_ros2_retry_enabled(False, reason="server stopping")

        # Stop ROS2 stack first while manager loop is still alive.
        if stop_ros2_stack:
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
        self._release_stack_controller_lock()
        self._release_server_instance_lock()

        logger.info("UDP server stopped")

    def _acquire_server_instance_lock(self) -> bool:
        if self._server_lock_fd is not None:
            return True

        fd: Optional[int] = None
        try:
            fd = os.open(self._server_lock_path, os.O_RDWR | os.O_CREAT, 0o644)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.ftruncate(fd, 0)
            owner = f"pid={os.getpid()} controller=udp_server_singleton\n"
            os.write(fd, owner.encode("utf-8"))
            os.fsync(fd)
            self._server_lock_fd = fd
            logger.info(f"Acquired UDP server singleton lock: {self._server_lock_path}")
            return True
        except BlockingIOError:
            logger.error(f"UDP server singleton lock busy: {self._server_lock_path}")
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
            return False
        except Exception as exc:
            logger.error(f"Failed acquiring UDP server singleton lock: {exc}")
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
            return False

    def _release_server_instance_lock(self) -> None:
        if self._server_lock_fd is None:
            return

        fd = self._server_lock_fd
        self._server_lock_fd = None

        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        except Exception:
            pass

        try:
            os.close(fd)
        except Exception:
            pass

    def _acquire_stack_controller_lock(self) -> bool:
        if self._stack_lock_fd is not None:
            return True

        fd: Optional[int] = None
        try:
            fd = os.open(self._stack_lock_path, os.O_RDWR | os.O_CREAT, 0o644)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.ftruncate(fd, 0)
            owner = f"pid={os.getpid()} controller=udp_server\n"
            os.write(fd, owner.encode("utf-8"))
            os.fsync(fd)
            self._stack_lock_fd = fd
            logger.info(f"Acquired ROS2 stack controller lock: {self._stack_lock_path}")
            return True
        except BlockingIOError:
            logger.error(f"ROS2 stack controller lock busy: {self._stack_lock_path}")
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
            return False
        except Exception as exc:
            logger.error(f"Failed acquiring ROS2 stack controller lock: {exc}")
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
            return False

    def _release_stack_controller_lock(self) -> None:
        if self._stack_lock_fd is None:
            return

        fd = self._stack_lock_fd
        self._stack_lock_fd = None

        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        except Exception:
            pass

        try:
            os.close(fd)
        except Exception:
            pass

    def set_trajectory_active(self, active: bool):
        with self.traj_lock:
            self.trajectory_active = active
        # Wake sender immediately so mode changes are reflected without idle delay.
        self._send_wakeup.set()
        logger.info(f"Trajectory sending: {'active' if active else 'inactive'}")

    @staticmethod
    def _now_ms() -> int:
        """Monotonic milliseconds for internal timeouts/intervals."""
        return int(time.monotonic() * 1000)

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
                self._send_wakeup.set()

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
                    self._send_wakeup.wait(send_interval)
                    self._send_wakeup.clear()
                else:
                    self._send_wakeup.wait(idle_interval)
                    self._send_wakeup.clear()

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
        now_ms = self._now_ms()
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

        # Keep a continuously refreshed hold anchor from raw STM32 pose.
        # This ensures START_TRAJ can immediately hold at the robot's latest
        # observed pose, even after spending time in traj 0 mode.
        with self.traj_lock:
            self._hold_knot = (
                float(pose_data.x),
                float(pose_data.y),
                float(pose_data.yaw),
                0.0,
                0.0,
            )
            self._hold_traj_t0_ms = int(pose_data.pose_t_ms)
            self._hold_until_ms = now_ms + 500

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
        #   traj 1 -> enter global LiDAR mapping mode (map 1)
        #   traj 0 -> disable trajectory generation/sending
        #   traj2 2 -> start/restart ROS2 stack
        if cmd.cmd_id == CommandID.START_TRAJ:
            logger.info("START_TRAJ received; entering mapping mode (equivalent to map 1)")
            # Requested behavior:
            # - traj 1 immediately switches to mapping mode
            # - trajectory streaming is paused while mapping
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode("mapping")
            if not ok_stack:
                logger.warning("START_TRAJ failed to switch ROS2 stack to mapping mode")
            ok = self.ros2_bridge.start_mapping_mode()
            if not ok:
                logger.warning("START_TRAJ->START_MAPPING failed: mapping service unavailable or rejected")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.STOP_TRAJ:
            logger.info("STOP_TRAJ received; disabling trajectory output")
            now_ms = self._now_ms()
            with self.pose_lock:
                latest_pose = self.latest_pose
            with self.traj_lock:
                if latest_pose is not None:
                    self._hold_knot = (
                        float(latest_pose.x),
                        float(latest_pose.y),
                        float(latest_pose.yaw),
                        0.0,
                        0.0,
                    )
                    self._hold_traj_t0_ms = int(latest_pose.pose_t_ms)
                    self._hold_until_ms = now_ms + 500
                self._active_traj_signature = None
                self._active_traj_t0_ms = None
                self._last_valid_traj = None
            self.set_trajectory_active(False)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.START_RESTART_ROS2:
            logger.info("START_RESTART_ROS2 received; restarting ROS2 stack in standby mode")
            # Keep current trajectory_active state, but pause sending while restart happens.
            with self.traj_lock:
                resume_traj = bool(self.trajectory_active)
            self.set_trajectory_active(False)

            started = self._switch_stack_mode("standby")

            if started and resume_traj:
                self.set_trajectory_active(True)

            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.STOP_ROS2:
            logger.info("STOP_ROS2 received; stopping ROS2 stack")
            self.set_trajectory_active(False)
            with self._stack_transition_lock:
                self._set_ros2_retry_enabled(False, reason="STOP_ROS2 command received")
                _ = self._run_ros2(self.ros2_mgr.stop)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.SHUTDOWN_PI5:
            logger.warning("SHUTDOWN_PI5 received; issuing sudo poweroff")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self._trigger_poweroff_command()
            return

        if cmd.cmd_id == CommandID.START_MAPPING:
            logger.info("START_MAPPING received; enabling mapping mode")
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode("mapping")
            if not ok_stack:
                logger.warning("START_MAPPING failed to switch ROS2 stack to mapping mode")
            ok = self.ros2_bridge.start_mapping_mode()
            if not ok:
                logger.warning("START_MAPPING failed: mapping service unavailable or rejected")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.FINISH_MAPPING:
            logger.info("FINISH_MAPPING received; finalizing mapping and switching to AMCL localization mode")
            # Requested behavior for map 0:
            # 1) finish map save
            # 2) restart stack into localization mode (AMCL only)
            # 3) load saved/frozen map
            # 4) resume trajectory following output
            self.set_trajectory_active(False)
            ok_finish = self.ros2_bridge.finish_mapping_mode()
            if not ok_finish:
                logger.warning("FINISH_MAPPING failed: mapping service unavailable or rejected")

            ok_stack = self._switch_stack_mode("localization")
            if not ok_stack:
                logger.warning("FINISH_MAPPING failed to switch ROS2 stack to localization mode")

            ok_frozen = self.ros2_bridge.use_frozen_map_mode()
            if not ok_frozen:
                logger.warning("FINISH_MAPPING->USE_FROZEN_MAP failed: service unavailable or rejected")

            self.set_trajectory_active(True)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.USE_LIVE_MAP:
            logger.info("USE_LIVE_MAP received; switching to live LiDAR mode")
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode("mapping")
            if not ok_stack:
                logger.warning("USE_LIVE_MAP failed to switch ROS2 stack to mapping mode")
            ok = self.ros2_bridge.use_live_map_mode()
            if not ok:
                logger.warning("USE_LIVE_MAP failed: service unavailable or rejected")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            return

        if cmd.cmd_id == CommandID.USE_FROZEN_MAP:
            logger.info("USE_FROZEN_MAP received; switching to AMCL localization on saved map")
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode("localization")
            if not ok_stack:
                logger.warning("USE_FROZEN_MAP failed to switch ROS2 stack to localization mode")
            ok = self.ros2_bridge.use_frozen_map_mode()
            if not ok:
                logger.warning("USE_FROZEN_MAP failed: service unavailable or rejected")
            self.set_trajectory_active(True)
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
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
            if not self._stack_control_enabled:
                continue

            target_mode = self._active_stack_mode

            async def _healthy_check() -> bool:
                return await self.ros2_mgr.is_mode_healthy(target_mode)

            healthy = self._run_ros2(_healthy_check)
            if healthy:
                self._set_ros2_retry_enabled(False, reason="stack is healthy")
                continue

            logger.warning(f"Retrying ROS2 stack recovery in mode={target_mode}...")
            recovered = self._switch_stack_mode(target_mode)
            if recovered:
                self._set_ros2_retry_enabled(False, reason="retry recovery succeeded")

    def _default_get_trajectory(self) -> Optional[Trajectory]:
        """Forward the latest ROS2 trajectory knots without modifying values."""
        planner_data_fresh = self.ros2_bridge.planner_data_is_fresh(max_age_ms=800)
        path_points = self.ros2_bridge.get_planned_path_points(max_points=64) if planner_data_fresh else []
        path_velocities = self.ros2_bridge.get_planned_path_velocities(max_points=64) if planner_data_fresh else []
        now_ms = self._now_ms()

        if not path_points or not path_velocities:
            if planner_data_fresh and self._last_valid_traj is not None:
                return self._last_valid_traj

            # Keep publishing a short-horizon HOLD knot whenever no valid path
            # exists, using the most recently observed STM32 pose.
            with self.pose_lock:
                latest_pose = self.latest_pose

            if latest_pose is not None:
                self._hold_knot = (
                    float(latest_pose.x),
                    float(latest_pose.y),
                    float(latest_pose.yaw),
                    0.0,
                    0.0,
                )
                self._hold_traj_t0_ms = int(latest_pose.pose_t_ms)
            elif self._hold_knot is None:
                self._hold_knot = (0.0, 0.0, 0.0, 0.0, 0.0)
                self._hold_traj_t0_ms = 0

            self._hold_until_ms = now_ms + 500

            self._active_traj_signature = None
            self._active_traj_t0_ms = None

            if self._hold_knot is not None and now_ms <= self._hold_until_ms:
                hold_t0_ms = int(self._hold_traj_t0_ms) if self._hold_traj_t0_ms is not None else 0
                return Trajectory(
                    reply_to_pose_seq=self._last_pose_seq,
                    traj_t0_ms=hold_t0_ms,
                    dt=0.01,
                    knots=[self._hold_knot],
                    flags=1,
                )

            self._hold_knot = None
            self._hold_traj_t0_ms = None
            self._hold_until_ms = 0
            return None

        if len(path_points) != len(path_velocities):
            logger.warning(
                "Skipping TRAJ publish: path/velocity length mismatch (path=%d, vel=%d)",
                len(path_points),
                len(path_velocities),
            )
            if self._last_valid_traj is not None:
                return self._last_valid_traj
            return None

        # Any valid path cancels pending hold streaming.
        self._hold_knot = None
        self._hold_traj_t0_ms = None
        self._hold_until_ms = 0

        # Planner publishes knots at 100 Hz; keep TRAJ knot spacing metadata aligned
        # even though the full TRAJ packet is refreshed at 5 Hz.
        dt = 0.01
        knots: List[Tuple[float, float, float, float, float]] = []

        for idx in range(len(path_points)):
            x, y, yaw_opt = path_points[idx]
            if yaw_opt is None:
                logger.warning("Skipping TRAJ publish: missing yaw at knot %d", idx)
                return None

            vx, vy = path_velocities[idx]
            knots.append((float(x), float(y), float(yaw_opt), float(vx), float(vy)))

        # Keep a stable trajectory start timestamp until knots materially change.
        # Resetting t0 every send causes STM32 to keep replaying early knots.
        first_x, first_y, _ = path_points[0]
        last_x, last_y, _ = path_points[-1]
        traj_signature = (
            len(path_points),
            round(float(first_x), 4),
            round(float(first_y), 4),
            round(float(last_x), 4),
            round(float(last_y), 4),
        )

        with self.pose_lock:
            latest_pose = self.latest_pose

        if self._active_traj_signature != traj_signature:
            self._active_traj_signature = traj_signature
            self._active_traj_t0_ms = int(latest_pose.pose_t_ms) if latest_pose is not None else 0

        traj_t0_ms = int(self._active_traj_t0_ms) if self._active_traj_t0_ms is not None else 0

        traj = Trajectory(
            reply_to_pose_seq=self._last_pose_seq,
            traj_t0_ms=traj_t0_ms,
            dt=dt,
            knots=knots,
            flags=0,
        )
        self._last_valid_traj = traj
        return traj


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
        self.latest_path_rx_ms: int = 0
        self.latest_vel_rx_ms: int = 0

        self.path_node = rclpy.create_node("planned_path_sub")
        self.path_sub = self.path_node.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.vel_sub = self.path_node.create_subscription(
            Float64MultiArray, "/planned_path_velocities", self._on_velocities, 10
        )
        self.mapping_start_client = self.path_node.create_client(Trigger, "/mapping/start")
        self.mapping_finish_client = self.path_node.create_client(Trigger, "/mapping/finish")
        self.mapping_use_live_client = self.path_node.create_client(Trigger, "/mapping/use_live")
        self.mapping_use_frozen_client = self.path_node.create_client(Trigger, "/mapping/use_frozen")

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
            self.latest_path_rx_ms = int(time.monotonic() * 1000)

    def _on_velocities(self, msg: Float64MultiArray) -> None:
        """Receive velocities as [vx0, vy0, vx1, vy1, ...]"""
        with self.path_lock:
            vels = []
            for i in range(0, len(msg.data), 2):
                if i + 1 < len(msg.data):
                    vels.append((float(msg.data[i]), float(msg.data[i + 1])))
            self.latest_velocities = vels if vels else None
            self.latest_vel_rx_ms = int(time.monotonic() * 1000)

    def planner_data_is_fresh(self, max_age_ms: int = 800) -> bool:
        """Return true if path + velocities were updated recently."""
        now_ms = int(time.monotonic() * 1000)
        with self.path_lock:
            if self.latest_path is None or self.latest_velocities is None:
                return False
            path_age_ms = now_ms - int(self.latest_path_rx_ms)
            vel_age_ms = now_ms - int(self.latest_vel_rx_ms)

        return (path_age_ms <= max_age_ms) and (vel_age_ms <= max_age_ms)

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

    def _call_trigger(self, client, service_name: str, timeout_sec: float = 1.5) -> bool:
        if not client.wait_for_service(timeout_sec=0.3):
            logger.warning(f"Service not available: {service_name}")
            return False

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.01)

        if not future.done():
            logger.warning(f"Service timeout: {service_name}")
            return False

        try:
            result = future.result()
            if result is None:
                logger.warning(f"Service returned no result: {service_name}")
                return False
            if not result.success:
                logger.warning(f"Service rejected request: {service_name} ({result.message})")
            return bool(result.success)
        except Exception as exc:
            logger.error(f"Service call failed: {service_name}: {exc}")
            return False

    def start_mapping_mode(self) -> bool:
        return self._call_trigger(self.mapping_start_client, "/mapping/start")

    def finish_mapping_mode(self) -> bool:
        return self._call_trigger(self.mapping_finish_client, "/mapping/finish")

    def use_live_map_mode(self) -> bool:
        return self._call_trigger(self.mapping_use_live_client, "/mapping/use_live")

    def use_frozen_map_mode(self) -> bool:
        return self._call_trigger(self.mapping_use_frozen_client, "/mapping/use_frozen")

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

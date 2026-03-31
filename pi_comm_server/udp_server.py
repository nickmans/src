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
import pty
import queue
import signal
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
        self.cmd_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._send_wakeup = threading.Event()
        self._cmd_queue: "queue.Queue[Tuple[Header, bytes, tuple]]" = queue.Queue(maxsize=128)
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
        self._last_mode_switch_monotonic = 0.0
        self._last_mode_request_monotonic = 0.0
        self._last_mode_request_target: Optional[str] = None
        self._last_mode_request_result = False
        self._mode_request_debounce_s = 1.0
        self._mode_health_cache_ttl_s = 1.5
        self._last_mode_health_check_monotonic = 0.0
        self._last_mode_health_check_target: Optional[str] = None
        self._last_mode_health_check_result = False
        self._warmup_guard_s = 20.0
        self._throttled_log_last_ts: dict[str, float] = {}
        self._stack_transition_lock = threading.Lock()
        self._stack_lock_path = "/tmp/omni_ros2_stack_controller.lock"
        self._stack_lock_fd: Optional[int] = None
        self._stack_control_enabled = True
        self._server_lock_path = "/tmp/omni_udp_server.lock"
        self._server_lock_fd: Optional[int] = None
        self._sock_send_lock = threading.Lock()
        self._terminal_lock = threading.Lock()
        self._terminal_master_fd: Optional[int] = None
        self._terminal_slave_fd: Optional[int] = None
        self._terminal_proc: Optional[subprocess.Popen] = None
        self._terminal_reader_thread: Optional[threading.Thread] = None
        self._terminal_client_addr: Optional[tuple] = None
        self._terminal_seq = 0
        self._terminal_active = False

        # Trajectory send cadence (default 10 Hz) can be overridden with
        # environment variable OMNI_TRAJ_SEND_HZ.
        send_hz_raw = os.getenv("OMNI_TRAJ_SEND_HZ", "10")
        try:
            send_hz = float(send_hz_raw)
            if send_hz <= 0.0:
                raise ValueError("non-positive")
        except Exception:
            logger.warning(
                "Invalid OMNI_TRAJ_SEND_HZ='%s'; falling back to 10 Hz",
                send_hz_raw,
            )
            send_hz = 10.0
        self._traj_send_hz = send_hz
        self._traj_send_interval_s = 1.0 / self._traj_send_hz
        self._traj_idle_interval_s = 5.0
        self._traj_tx_count = 0
        self._traj_tx_window_start = time.monotonic()

        logger.info(
            "OMNIUDPServer initialized: %s:%s (traj_send_hz=%.2f)",
            self.host,
            self.port,
            self._traj_send_hz,
        )

    def _switch_stack_mode(self, stack_mode: str) -> bool:
        if not self._stack_control_enabled:
            logger.warning(f"Stack lifecycle control disabled; cannot switch to mode={stack_mode}")
            return False

        target_mode = (stack_mode or "").strip().lower()
        if target_mode not in {"standby", "mapping", "localization"}:
            logger.error(f"Invalid stack mode request: {stack_mode}")
            return False

        with self._stack_transition_lock:
            now = time.monotonic()
            if (
                self._last_mode_request_target == target_mode
                and (now - float(self._last_mode_request_monotonic)) < float(self._mode_request_debounce_s)
            ):
                return bool(self._last_mode_request_result)

            self._last_mode_request_target = target_mode
            self._last_mode_request_monotonic = now

            self._set_ros2_retry_enabled(False, reason=f"mode transition to {target_mode}")

            try:
                changed = self.ros2_mgr.set_stack_mode(target_mode)
            except Exception as exc:
                logger.error(f"Failed to set ROS2 stack mode '{target_mode}': {exc}")
                self._last_mode_request_result = False
                return False

            is_running = self._run_ros2(self.ros2_mgr.is_running)
            needs_restart = True

            if is_running and not changed:
                since_last_switch_s = now - float(self._last_mode_switch_monotonic)
                if self._active_stack_mode == target_mode and since_last_switch_s < float(self._warmup_guard_s):
                    self._log_throttled(
                        f"mode-warmup-{target_mode}",
                        logging.WARNING,
                        (
                            f"ROS2 stack in {target_mode} is still warming up "
                            f"({since_last_switch_s:.1f}s); skipping forced restart"
                        ),
                        interval_s=2.0,
                    )
                    self._last_mode_request_result = True
                    return True

                healthy = self._is_mode_healthy_cached(target_mode, now)
                if healthy:
                    logger.info(f"ROS2 stack already healthy in {target_mode} mode")
                    self._active_stack_mode = target_mode
                    self._last_mode_request_result = True
                    return True

                self._log_throttled(
                    f"mode-unhealthy-{target_mode}",
                    logging.WARNING,
                    f"ROS2 stack running but unhealthy in {target_mode} mode; forcing restart",
                    interval_s=2.0,
                )

            if is_running and needs_restart:
                logger.info(f"Switching ROS2 stack from {self._active_stack_mode} to {target_mode}")
                stopped = self._run_ros2(self.ros2_mgr.stop)
                if not stopped:
                    logger.error("Failed to stop ROS2 stack during mode switch")
                    self._set_ros2_retry_enabled(True, reason="stop failed during transition")
                    self._last_mode_request_result = False
                    return False
                # Allow serial devices/process groups to settle before relaunch,
                # reducing immediate start/stop oscillation on LiDAR drivers.
                    time.sleep(0.3)

            started = self._run_ros2(self.ros2_mgr.start)
            if not started:
                logger.error(f"Failed to start ROS2 stack in {target_mode} mode")
                self._set_ros2_retry_enabled(True, reason="start failed during transition")
                self._last_mode_request_result = False
                return False

            self._active_stack_mode = target_mode
            self._last_mode_switch_monotonic = time.monotonic()
            self._last_mode_health_check_target = target_mode
            self._last_mode_health_check_monotonic = self._last_mode_switch_monotonic
            self._last_mode_health_check_result = True
            self._set_ros2_retry_enabled(False, reason="mode transition complete")
            self._last_mode_request_result = True
            return True

    def _switch_stack_mode_with_retries(
        self,
        stack_mode: str,
        attempts: int = 2,
        retry_delay_s: float = 2.0,
    ) -> bool:
        attempts = max(1, int(attempts))
        for attempt in range(1, attempts + 1):
            if self._switch_stack_mode(stack_mode):
                return True

            if attempt >= attempts:
                break

            logger.warning(
                f"ROS2 stack switch to {stack_mode} failed on attempt {attempt}/{attempts}; "
                f"retrying in {retry_delay_s:.1f}s"
            )
            time.sleep(max(0.0, float(retry_delay_s)))

        return False

    def _is_mode_healthy_cached(self, target_mode: str, now_monotonic: Optional[float] = None) -> bool:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        if (
            self._last_mode_health_check_target == target_mode
            and (now - float(self._last_mode_health_check_monotonic)) < float(self._mode_health_cache_ttl_s)
        ):
            return bool(self._last_mode_health_check_result)

        async def _healthy_check() -> bool:
            return await self.ros2_mgr.is_mode_healthy(target_mode)

        healthy = bool(self._run_ros2(_healthy_check))
        self._last_mode_health_check_target = target_mode
        self._last_mode_health_check_monotonic = now
        self._last_mode_health_check_result = healthy
        return healthy

    def _log_throttled(self, key: str, level: int, message: str, interval_s: float = 1.0) -> None:
        now = time.monotonic()
        last_ts = float(self._throttled_log_last_ts.get(key, 0.0))
        if (now - last_ts) < float(interval_s):
            return
        self._throttled_log_last_ts[key] = now
        logger.log(level, message)

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

        self.cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
        self.cmd_thread.start()

    def stop(self, stop_ros2_stack: bool = True):
        logger.info("Stopping UDP server...")
        self.running = False
        self._stop_event.set()
        self._send_wakeup.set()
        self._set_ros2_retry_enabled(False, reason="server stopping")
        self._stop_terminal_passthrough(notify_client=False, reason="server stopping")

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
        if self.cmd_thread and self.cmd_thread.is_alive():
            self.cmd_thread.join(timeout=1.0)
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
            changed = (self.trajectory_active != active)
            self.trajectory_active = active
        # Wake sender immediately so mode changes are reflected without idle delay.
        self._send_wakeup.set()
        if changed:
            logger.info(f"Trajectory sending: {'active' if active else 'inactive'}")

    @staticmethod
    def _now_ms() -> int:
        """Monotonic milliseconds for internal timeouts/intervals."""
        return int(time.monotonic() * 1000)

    def _safe_sendto(self, message: bytes, addr: tuple) -> None:
        if not addr:
            return

        with self._sock_send_lock:
            if self.sock:
                self.sock.sendto(message, addr)

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
        send_interval = self._traj_send_interval_s
        idle_interval = self._traj_idle_interval_s

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
                try:
                    self._cmd_queue.put_nowait((header, payload, addr))
                except queue.Full:
                    logger.error(
                        "CMD queue full; dropping command seq=%d from %s",
                        header.seq,
                        addr,
                    )
            else:
                logger.warning(f"Unknown message type: {header.msg_type} from {addr}. Expected one of: {[e.value for e in MessageType]}")
        except Exception as e:
            logger.error(f"Error handling message: {e}")

    def _cmd_loop(self):
        while self.running and not self._stop_event.is_set():
            try:
                header, payload, addr = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self._handle_cmd(header, payload, addr)
            except Exception as exc:
                logger.error(f"Error handling queued CMD seq={header.seq} from {addr}: {exc}")
            finally:
                self._cmd_queue.task_done()

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
            self._safe_sendto(message, addr)
        except Exception as exc:
            logger.error(f"Failed to send CMD ack: {exc}")

    def _next_terminal_seq(self) -> int:
        with self._terminal_lock:
            self._terminal_seq += 1
            return self._terminal_seq

    def _send_terminal_control(self, cmd_id: int, addr: Optional[tuple] = None) -> None:
        target_addr = addr or self._terminal_client_addr
        if not target_addr:
            return

        try:
            payload = Command(cmd_id=cmd_id, arg=b"").pack()
            message = make_message(MessageType.CMD, self._next_terminal_seq(), payload, crc_payload=False)
            self._safe_sendto(message, target_addr)
        except Exception as exc:
            logger.error("Failed to send terminal control cmd=%s: %s", cmd_id, exc)

    def _send_terminal_data(self, data: bytes, addr: Optional[tuple] = None) -> None:
        if not data:
            return

        target_addr = addr or self._terminal_client_addr
        if not target_addr:
            return

        try:
            payload = Command(cmd_id=CommandID.TERMINAL_PASSTHROUGH_DATA, arg=data).pack()
            message = make_message(MessageType.CMD, self._next_terminal_seq(), payload, crc_payload=False)
            self._safe_sendto(message, target_addr)
        except Exception as exc:
            logger.error("Failed to send terminal data: %s", exc)

    def _send_terminal_notice(self, text: str, addr: Optional[tuple] = None) -> None:
        self._send_terminal_data(text.encode("utf-8", errors="replace"), addr)

    def _terminal_reader_loop(self, master_fd: int) -> None:
        try:
            while self.running:
                try:
                    data = os.read(master_fd, 256)
                except OSError:
                    break

                if not data:
                    break

                self._send_terminal_data(data)
        finally:
            self._stop_terminal_passthrough(notify_client=True, reason="shell exited")

    def _start_terminal_passthrough(self, addr: tuple) -> bool:
        self._stop_terminal_passthrough(notify_client=False, reason="restart terminal session")

        shell_path = os.environ.get("SHELL") or "/bin/bash"
        home_dir = os.path.expanduser("~")
        master_fd: Optional[int] = None
        slave_fd: Optional[int] = None

        try:
            master_fd, slave_fd = pty.openpty()
            env = os.environ.copy()
            env.setdefault("TERM", "xterm-256color")
            env.setdefault("HOME", home_dir)
            proc = subprocess.Popen(
                [shell_path, "-i"],
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=slave_fd,
                cwd=home_dir,
                env=env,
                start_new_session=True,
                close_fds=True,
            )
            os.close(slave_fd)
        except Exception as exc:
            logger.error("Failed to start terminal passthrough shell: %s", exc)
            try:
                os.close(master_fd)
            except Exception:
                pass
            try:
                os.close(slave_fd)
            except Exception:
                pass
            self._send_terminal_notice("\r\n[failed to start pi terminal]\r\n", addr)
            return False

        reader_thread = threading.Thread(
            target=self._terminal_reader_loop,
            args=(master_fd,),
            name="pi_terminal_reader",
            daemon=True,
        )

        with self._terminal_lock:
            self._terminal_master_fd = master_fd
            self._terminal_slave_fd = None
            self._terminal_proc = proc
            self._terminal_reader_thread = reader_thread
            self._terminal_client_addr = addr
            self._terminal_active = True

        reader_thread.start()
        self._send_terminal_notice("\r\n[pi terminal passthrough active; send * to exit]\r\n", addr)
        logger.info("Terminal passthrough shell started for %s", addr)
        return True

    def _handle_terminal_input(self, data: bytes, addr: tuple) -> None:
        with self._terminal_lock:
            master_fd = self._terminal_master_fd
            active = self._terminal_active
            if active:
                self._terminal_client_addr = addr

        if not active or master_fd is None:
            self._send_terminal_notice("\r\n[no active pi terminal session]\r\n", addr)
            return

        try:
            os.write(master_fd, data)
        except OSError as exc:
            logger.error("Failed to write terminal input: %s", exc)
            self._stop_terminal_passthrough(notify_client=True, reason="terminal write failed")

    def _stop_terminal_passthrough(self, notify_client: bool, reason: str = "") -> None:
        with self._terminal_lock:
            was_active = self._terminal_active
            proc = self._terminal_proc
            master_fd = self._terminal_master_fd
            slave_fd = self._terminal_slave_fd
            reader_thread = self._terminal_reader_thread
            client_addr = self._terminal_client_addr

            self._terminal_active = False
            self._terminal_proc = None
            self._terminal_master_fd = None
            self._terminal_slave_fd = None
            self._terminal_reader_thread = None
            self._terminal_client_addr = None

        if not was_active and proc is None and master_fd is None:
            return

        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGHUP)
            except Exception:
                pass
            try:
                proc.terminate()
                proc.wait(timeout=0.5)
            except Exception:
                try:
                    proc.kill()
                    proc.wait(timeout=0.5)
                except Exception:
                    pass

        for fd in (master_fd, slave_fd):
            if fd is None:
                continue
            try:
                os.close(fd)
            except Exception:
                pass

        if (
            reader_thread is not None
            and reader_thread.is_alive()
            and reader_thread is not threading.current_thread()
        ):
            reader_thread.join(timeout=0.5)

        if notify_client and client_addr:
            self._send_terminal_control(CommandID.STOP_TERMINAL_PASSTHROUGH, client_addr)

        if reason:
            logger.info("Terminal passthrough stopped: %s", reason)

    def _call_mapping_service_with_retries(
        self,
        service_label: str,
        call_fn: Callable[[], bool],
        attempts: int = 4,
        retry_delay_s: float = 0.7,
        pre_wait_s: float = 0.0,
    ) -> bool:
        attempts = max(1, int(attempts))
        wait_s = max(0.0, float(pre_wait_s))
        if wait_s > 0.0:
            time.sleep(wait_s)

        for attempt in range(1, attempts + 1):
            ok = False
            try:
                ok = bool(call_fn())
            except Exception as exc:
                logger.warning(f"{service_label} attempt {attempt}/{attempts} raised exception: {exc}")

            if ok:
                if attempt > 1:
                    logger.info(f"{service_label} succeeded on retry {attempt}/{attempts}")
                return True

            if attempt < attempts:
                logger.warning(
                    f"{service_label} failed on attempt {attempt}/{attempts}; retrying in {retry_delay_s:.1f}s"
                )
                time.sleep(max(0.0, float(retry_delay_s)))

        logger.warning(f"{service_label} failed after {attempts} attempts")
        return False

    def _handle_cmd(self, header: Header, payload: bytes, addr: tuple):
        cmd = Command.unpack(payload)
        self._log_throttled(
            key=f"cmd-rx-{int(cmd.cmd_id)}",
            level=logging.INFO,
            message=f"CMD received: command={cmd.cmd_id} (seq={header.seq}) from {addr}",
            interval_s=0.5,
        )
        if self.on_cmd_callback:
            self.on_cmd_callback(cmd.cmd_id)

        # STM32 semantics:
        #   traj 1 -> autonomous trajectory following with localization on saved map
        #   traj 0 -> disable trajectory generation/sending (idle/manual standby)
        #   traj2 2 -> manual driving on STM32 with AMCL localization active on Pi
        #   traj 3 -> autonomous trajectory following with blank global map
        #             and local costmap obstacle avoidance
        #   map 1 -> dedicated mapping mode
        if cmd.cmd_id == CommandID.START_TRAJ:
            logger.info("START_TRAJ received; enabling autonomous localization mode using saved map")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            # Requested behavior:
            # - traj 1 switches to localization mode
            # - saved map is loaded/reloaded
            # - trajectory streaming is enabled for autonomous following
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode_with_retries("localization", attempts=3, retry_delay_s=3.0)
            if not ok_stack:
                logger.warning("START_TRAJ failed to switch ROS2 stack to localization mode")
            ok = False
            if ok_stack:
                ok = self._call_mapping_service_with_retries(
                    service_label="START_TRAJ->/mapping/use_frozen",
                    call_fn=self.ros2_bridge.use_frozen_map_mode,
                    attempts=4,
                    retry_delay_s=0.7,
                    pre_wait_s=0.4,
                )
            if not ok:
                logger.warning(
                    "START_TRAJ could not confirm saved-map reload; continuing in localization mode"
                )
            if ok_stack:
                self.set_trajectory_active(True)
            else:
                logger.warning(
                    "START_TRAJ left trajectory output disabled because localization is not confirmed healthy"
                )
            return

        if cmd.cmd_id == CommandID.STOP_TRAJ:
            logger.info("STOP_TRAJ received; disabling trajectory output and returning to standby mode")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
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
            ok_stack = self._switch_stack_mode("standby")
            if not ok_stack:
                logger.warning("STOP_TRAJ failed to switch ROS2 stack to standby mode")
            return

        if cmd.cmd_id == CommandID.START_RESTART_ROS2:
            logger.info(
                "START_RESTART_ROS2 received; switching to manual localization mode "
                "(trajectory output disabled)"
            )
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))

            self.set_trajectory_active(False)
            with self.traj_lock:
                self._active_traj_signature = None
                self._active_traj_t0_ms = None
                self._last_valid_traj = None

            started = self._switch_stack_mode_with_retries("localization", attempts=3, retry_delay_s=3.0)
            if not started:
                logger.warning(
                    "START_RESTART_ROS2 failed to switch ROS2 stack to localization mode; "
                    "trajectory output remains disabled for manual safety"
                )
            else:
                _ = self._call_mapping_service_with_retries(
                    service_label="START_RESTART_ROS2->/mapping/use_frozen",
                    call_fn=self.ros2_bridge.use_frozen_map_mode,
                    attempts=3,
                    retry_delay_s=0.5,
                    pre_wait_s=0.3,
                )
                logger.info("Localization mode active; STM32 remains in manual mode (no trajectory streaming)")
            return

        if cmd.cmd_id == CommandID.START_TRAJ_LOCAL:
            logger.info(
                "START_TRAJ_LOCAL received; enabling autonomous mode with blank global map "
                "and local costmap avoidance"
            )
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))

            self.set_trajectory_active(False)
            with self.traj_lock:
                self._active_traj_signature = None
                self._active_traj_t0_ms = None
                self._last_valid_traj = None

            ok_stack = self._switch_stack_mode_with_retries("standby", attempts=2, retry_delay_s=2.0)
            if not ok_stack:
                logger.warning("START_TRAJ_LOCAL failed to switch ROS2 stack to standby mode")

            ok_blank = False
            if ok_stack:
                ok_blank = self._call_mapping_service_with_retries(
                    service_label="START_TRAJ_LOCAL->/mapping/use_blank",
                    call_fn=self.ros2_bridge.use_blank_map_mode,
                    attempts=4,
                    retry_delay_s=0.6,
                    pre_wait_s=0.3,
                )

            if not ok_blank:
                logger.warning(
                    "START_TRAJ_LOCAL could not confirm blank global map mode; "
                    "trajectory output remains disabled"
                )
                return

            self.set_trajectory_active(True)
            logger.info("Blank global map mode active; trajectory streaming enabled")
            return

        if cmd.cmd_id == CommandID.START_TERMINAL_PASSTHROUGH:
            logger.info("START_TERMINAL_PASSTHROUGH received; opening Pi shell passthrough")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self._start_terminal_passthrough(addr)
            return

        if cmd.cmd_id == CommandID.TERMINAL_PASSTHROUGH_DATA:
            self._handle_terminal_input(cmd.arg, addr)
            return

        if cmd.cmd_id == CommandID.STOP_TERMINAL_PASSTHROUGH:
            logger.info("STOP_TERMINAL_PASSTHROUGH received; closing Pi shell passthrough")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self._stop_terminal_passthrough(notify_client=False, reason="STM32 requested terminal exit")
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
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode_with_retries("mapping", attempts=2, retry_delay_s=2.0)
            if not ok_stack:
                logger.warning("START_MAPPING failed to switch ROS2 stack to mapping mode")
            ok = self._call_mapping_service_with_retries(
                service_label="START_MAPPING->/mapping/start",
                call_fn=self.ros2_bridge.start_mapping_mode,
                attempts=5,
                retry_delay_s=0.8,
                pre_wait_s=0.8,
            )
            if not ok:
                logger.warning("START_MAPPING failed: mapping service unavailable or rejected")
            return

        if cmd.cmd_id == CommandID.FINISH_MAPPING:
            logger.info("FINISH_MAPPING received; finalizing mapping and switching to AMCL localization mode")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            # Requested behavior for map 0:
            # 1) finish map save
            # 2) restart stack into localization mode (AMCL only)
            # 3) resume trajectory following output once localization stack is healthy
            self.set_trajectory_active(False)
            ok_finish = self._call_mapping_service_with_retries(
                service_label="FINISH_MAPPING->/mapping/finish",
                call_fn=self.ros2_bridge.finish_mapping_mode,
                attempts=6,
                retry_delay_s=1.0,
                pre_wait_s=0.8,
            )
            if not ok_finish:
                logger.warning(
                    "FINISH_MAPPING could not confirm mapping finalization after retries; "
                    "continuing localization transition for availability"
                )

            ok_stack = self._switch_stack_mode_with_retries("localization", attempts=3, retry_delay_s=3.0)
            if not ok_stack:
                logger.warning("FINISH_MAPPING failed to switch ROS2 stack to localization mode")

            if ok_stack:
                logger.info(
                    "FINISH_MAPPING completed with localization mode; deprecated frozen-map service is skipped"
                )

            if ok_stack:
                self.set_trajectory_active(True)
            else:
                logger.warning("FINISH_MAPPING left trajectory output disabled because localization is not confirmed healthy")
            return

        if cmd.cmd_id == CommandID.USE_LIVE_MAP:
            logger.info("USE_LIVE_MAP received; switching to live LiDAR mode")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode_with_retries("mapping", attempts=2, retry_delay_s=2.0)
            if not ok_stack:
                logger.warning("USE_LIVE_MAP failed to switch ROS2 stack to mapping mode")
            ok = False
            if ok_stack:
                ok = self._call_mapping_service_with_retries(
                    service_label="USE_LIVE_MAP->/mapping/use_live",
                    call_fn=self.ros2_bridge.use_live_map_mode,
                    attempts=4,
                    retry_delay_s=0.7,
                    pre_wait_s=0.4,
                )
            if not ok:
                logger.warning("USE_LIVE_MAP failed: service unavailable or rejected")
            return

        if cmd.cmd_id == CommandID.USE_FROZEN_MAP:
            logger.info("USE_FROZEN_MAP received; switching to AMCL localization mode")
            self._send_cmd_ack(header.seq, addr, int(cmd.cmd_id))
            self.set_trajectory_active(False)
            ok_stack = self._switch_stack_mode_with_retries("localization", attempts=3, retry_delay_s=3.0)
            if not ok_stack:
                logger.warning("USE_FROZEN_MAP failed to switch ROS2 stack to localization mode")
            if ok_stack:
                logger.info("USE_FROZEN_MAP now aliases to localization mode; deprecated frozen-map service is skipped")
                self.set_trajectory_active(True)
            else:
                logger.warning("USE_FROZEN_MAP left trajectory output disabled because localization is not confirmed healthy")
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
            self._safe_sendto(message, addr)

            self._traj_tx_count += 1
            now_mono = time.monotonic()
            elapsed = now_mono - self._traj_tx_window_start
            if elapsed >= 5.0:
                effective_hz = float(self._traj_tx_count) / elapsed
                logger.info(
                    "TRAJ TX effective_hz=%.2f seq=%d knots=%d",
                    effective_hz,
                    self.traj_seq,
                    len(traj.knots),
                )
                self._traj_tx_count = 0
                self._traj_tx_window_start = now_mono

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
            self._log_throttled(
                key="ros2-retry-enabled",
                level=logging.WARNING,
                message=f"ROS2 retry enabled: {reason or 'start failures'}",
                interval_s=1.0,
            )
        else:
            self._log_throttled(
                key="ros2-retry-disabled",
                level=logging.INFO,
                message=f"ROS2 retry disabled: {reason or ''}",
                interval_s=1.0,
            )

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

            self._log_throttled(
                key=f"ros2-retry-recover-{target_mode}",
                level=logging.WARNING,
                message=f"Retrying ROS2 stack recovery in mode={target_mode}...",
                interval_s=2.0,
            )
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
            x_stm, y_stm, yaw_stm, vx_stm, vy_stm, _ = self.ros2_bridge.pose_node.transform_ros_traj_to_stm(
                float(x),
                float(y),
                float(yaw_opt),
                float(vx),
                float(vy),
                0.0,
            )
            knots.append((x_stm, y_stm, yaw_stm, vx_stm, vy_stm))

        # Keep a stable trajectory start timestamp until knots materially change.
        # Resetting t0 every send causes STM32 to keep replaying early knots.
        first_x, first_y = knots[0][0], knots[0][1]
        last_x, last_y = knots[-1][0], knots[-1][1]
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
        self._last_plan_tf_warn_ms: int = 0

        self.path_node = rclpy.create_node("planned_path_sub")
        self.path_sub = self.path_node.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.vel_sub = self.path_node.create_subscription(
            Float64MultiArray, "/planned_path_velocities", self._on_velocities, 10
        )
        self.mapping_start_client = self.path_node.create_client(Trigger, "/mapping/start")
        self.mapping_finish_client = self.path_node.create_client(Trigger, "/mapping/finish")
        self.mapping_use_live_client = self.path_node.create_client(Trigger, "/mapping/use_live")
        self.mapping_use_frozen_client = self.path_node.create_client(Trigger, "/mapping/use_frozen")
        self.mapping_use_blank_client = self.path_node.create_client(Trigger, "/mapping/use_blank")

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

    @staticmethod
    def _normalize_frame_id(frame_id: Optional[str]) -> str:
        frame = (frame_id or "").strip().lstrip("/")
        return frame if frame else "odom"

    @staticmethod
    def _yaw_from_quaternion(quat: object) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _lookup_plan_to_odom_tf(self, source_frame: str) -> Optional[Tuple[float, float, float]]:
        src = self._normalize_frame_id(source_frame)
        if src == "odom":
            return (0.0, 0.0, 0.0)

        try:
            tr = self.pose_node.tf_buffer.lookup_transform("odom", src, rclpy.time.Time())
            tx = float(tr.transform.translation.x)
            ty = float(tr.transform.translation.y)
            yaw = self._yaw_from_quaternion(tr.transform.rotation)
            return (tx, ty, yaw)
        except Exception as exc:
            now_ms = int(time.monotonic() * 1000)
            if (now_ms - self._last_plan_tf_warn_ms) > 2000:
                logger.warning(
                    "Unable to transform planned path frame '%s' -> 'odom'; holding trajectory until TF is available (%s)",
                    src,
                    exc,
                )
                self._last_plan_tf_warn_ms = now_ms
            return None

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

        source_frame = self._normalize_frame_id(path.header.frame_id)
        tf_plan_to_odom = self._lookup_plan_to_odom_tf(source_frame)
        if tf_plan_to_odom is None:
            return []
        tx, ty, yaw_tf = tf_plan_to_odom
        c = math.cos(yaw_tf)
        s = math.sin(yaw_tf)

        pts: List[Tuple[float, float, Optional[float]]] = []
        for ps in path.poses[:max_points]:
            x_src = float(ps.pose.position.x)
            y_src = float(ps.pose.position.y)
            x_odom = c * x_src - s * y_src + tx
            y_odom = s * x_src + c * y_src + ty
            q = ps.pose.orientation
            # Some publishers leave orientation all-zeros; treat that as unknown.
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                yaw = None
            else:
                # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw_src = math.atan2(siny_cosp, cosy_cosp)
                yaw = PosePublisherNode._wrap_to_pi(yaw_src + yaw_tf)
            pts.append((x_odom, y_odom, yaw))
        return pts

    def get_planned_path_velocities(self, max_points: int = 64) -> List[Tuple[float, float]]:
        """Return [(vx, vy), ...] from the latest /planned_path_velocities."""
        with self.path_lock:
            vels = self.latest_velocities
            path = self.latest_path

        if vels is None or path is None:
            return []

        source_frame = self._normalize_frame_id(path.header.frame_id)
        tf_plan_to_odom = self._lookup_plan_to_odom_tf(source_frame)
        if tf_plan_to_odom is None:
            return []
        _, _, yaw_tf = tf_plan_to_odom
        c = math.cos(yaw_tf)
        s = math.sin(yaw_tf)

        out: List[Tuple[float, float]] = []
        for vx_src, vy_src in vels[:max_points]:
            vx_odom = c * float(vx_src) - s * float(vy_src)
            vy_odom = s * float(vx_src) + c * float(vy_src)
            out.append((vx_odom, vy_odom))
        return out

    def _call_trigger(
        self,
        client,
        service_name: str,
        timeout_sec: float = 2.5,
        attempts: int = 1,
        availability_wait_sec: float = 1.0,
        retry_delay_sec: float = 0.25,
    ) -> bool:
        attempts = max(1, int(attempts))
        availability_wait_sec = max(0.1, float(availability_wait_sec))
        timeout_sec = max(0.2, float(timeout_sec))
        retry_delay_sec = max(0.0, float(retry_delay_sec))

        for attempt in range(1, attempts + 1):
            if not client.wait_for_service(timeout_sec=availability_wait_sec):
                logger.warning(
                    f"Service not available: {service_name} "
                    f"(attempt {attempt}/{attempts})"
                )
                if attempt < attempts and retry_delay_sec > 0.0:
                    time.sleep(retry_delay_sec)
                continue

            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                if future.done():
                    break
                time.sleep(0.01)

            if not future.done():
                logger.warning(
                    f"Service timeout: {service_name} "
                    f"(attempt {attempt}/{attempts})"
                )
                if attempt < attempts and retry_delay_sec > 0.0:
                    time.sleep(retry_delay_sec)
                continue

            try:
                result = future.result()
                if result is None:
                    logger.warning(
                        f"Service returned no result: {service_name} "
                        f"(attempt {attempt}/{attempts})"
                    )
                    if attempt < attempts and retry_delay_sec > 0.0:
                        time.sleep(retry_delay_sec)
                    continue

                if not result.success:
                    logger.warning(
                        f"Service rejected request: {service_name} ({result.message}) "
                        f"(attempt {attempt}/{attempts})"
                    )
                    if attempt < attempts and retry_delay_sec > 0.0:
                        time.sleep(retry_delay_sec)
                    continue

                return True
            except Exception as exc:
                logger.error(
                    f"Service call failed: {service_name}: {exc} "
                    f"(attempt {attempt}/{attempts})"
                )
                if attempt < attempts and retry_delay_sec > 0.0:
                    time.sleep(retry_delay_sec)

        return False

    def start_mapping_mode(self) -> bool:
        return self._call_trigger(
            self.mapping_start_client,
            "/mapping/start",
            timeout_sec=2.0,
            attempts=3,
            availability_wait_sec=1.0,
            retry_delay_sec=0.3,
        )

    def finish_mapping_mode(self) -> bool:
        return self._call_trigger(
            self.mapping_finish_client,
            "/mapping/finish",
            timeout_sec=3.0,
            attempts=5,
            availability_wait_sec=1.0,
            retry_delay_sec=0.4,
        )

    def use_live_map_mode(self) -> bool:
        return self._call_trigger(
            self.mapping_use_live_client,
            "/mapping/use_live",
            timeout_sec=2.0,
            attempts=3,
            availability_wait_sec=1.0,
            retry_delay_sec=0.3,
        )

    def use_frozen_map_mode(self) -> bool:
        return self._call_trigger(
            self.mapping_use_frozen_client,
            "/mapping/use_frozen",
            timeout_sec=2.0,
            attempts=3,
            availability_wait_sec=1.0,
            retry_delay_sec=0.3,
        )

    def use_blank_map_mode(self) -> bool:
        return self._call_trigger(
            self.mapping_use_blank_client,
            "/mapping/use_blank",
            timeout_sec=2.0,
            attempts=3,
            availability_wait_sec=1.0,
            retry_delay_sec=0.3,
        )

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

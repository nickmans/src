"""
ROS2 stack manager for launching ROS2 launch files directly.

Provides safe start/stop/status checks with subprocess management.
"""

import asyncio
import logging
import os
import shlex
import signal
import subprocess
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)


class ROS2Manager:
    """Manage ROS2 stack by launching ROS2 launch files directly."""

    def __init__(self, launch_file: str = "dual_sllidar_with_mock_and_traj.launch.py", package: str = "omni_traj"):
        """
        Initialize ROS2Manager.
        
        Args:
            launch_file: Name of the launch file to execute
            package: ROS2 package containing the launch file
        """
        self.launch_file = launch_file
        self.package = package
        self.stack_mode = "standby"
        self.launch_args = self._build_launch_args(self.stack_mode)
        self.launch_match = f"ros2 launch {self.package} {self.launch_file}"
        self.process: Optional[asyncio.subprocess.Process] = None
        self.timeout_sec = 2.5
        self.start_verify_timeout_sec = 8.0
        self.start_stability_window_sec = 1.0
        self.lidar_boot_settle_delay_sec = 1.5
        self.lidar_scan_probe_timeout_sec = 3.0
        self.mode_health_cache_ttl_sec = 1.5
        self._mode_health_cache: dict[str, tuple[float, bool]] = {}

        self._residual_process_patterns = [
            r"/lib/nav2_amcl/amcl(\s|$)",
            r"/lib/slam_toolbox/async_slam_toolbox_node(\s|$)",
            r"__node:=lifecycle_manager_localization",
            r"__node:=lifecycle_manager_slam",
            r"/lib/sllidar_ros2/sllidar_node .*__node:=lidar1",
            r"/lib/sllidar_ros2/sllidar_node .*__node:=lidar2",
            r"__node:=base_to_lidar1",
            r"__node:=base_to_lidar2",
            r"/omni_traj/waypoint_traj .*__node:=waypoint_traj",
        ]

        self._workspace_setup_candidates = [
            "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/setup.bash",
            "/home/nickolas/ros2_ws/src/omni_src/install/setup.bash",
            "/home/nickolas/ros2_ws/install/setup.bash",
            os.path.expanduser("~/ros2_ws/install/setup.bash"),
        ]

    def _pick_workspace_setup(self) -> Optional[str]:
        for candidate in self._workspace_setup_candidates:
            if os.path.isfile(candidate):
                return candidate
        return None

    def _build_launch_args(self, stack_mode: str) -> list[str]:
        args = [
            "use_mock_lidar:=false",
            "use_rviz:=false",
            "map_frame:=map",
            "publish_odom_to_base_tf:=true",
            "publish_world_to_odom_tf:=false",
            "rolling_map_enable:=true",
            "rolling_map_margin_m:=1.0",
            "persistent_obstacles_enable:=true",
            "lidar1_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2608b4e7586eef118367e9c2c169b110-if00-port0",
            "lidar2_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_420b6b8a586eef11a134e0c2c169b110-if00-port0",
        ]

        if stack_mode == "localization":
            args.extend([
                "enable_amcl_localization:=true",
                "enable_slam_toolbox:=false",
            ])
        elif stack_mode == "mapping":
            args.extend([
                "enable_amcl_localization:=false",
                "enable_slam_toolbox:=true",
            ])
        else:
            # standby: local costmap + fused scans only, no global scan matching
            args.extend([
                "enable_amcl_localization:=false",
                "enable_slam_toolbox:=false",
            ])

        return args

    def set_stack_mode(self, stack_mode: str) -> bool:
        normalized = (stack_mode or "").strip().lower()
        if normalized not in {"standby", "mapping", "localization"}:
            raise ValueError(f"Unsupported stack mode: {stack_mode}")

        if normalized == self.stack_mode:
            return False

        self.stack_mode = normalized
        self.launch_args = self._build_launch_args(self.stack_mode)
        self._mode_health_cache.clear()
        logger.info(f"ROS2 stack mode set to: {self.stack_mode}")
        return True

    async def start(self) -> bool:
        """
        Start the ROS2 stack by launching the specified launch file.

        Returns True if already running or successfully started, False otherwise.
        """
        # Check if already running and healthy for target mode
        if await self.is_running():
            if await self.is_mode_healthy(self.stack_mode):
                logger.info(f"ROS2 stack already running in healthy {self.stack_mode} mode")
                return True
            logger.warning("ROS2 stack running but unhealthy/mismatched; restarting before start")
            if not await self.stop():
                return False

        try:
            # Best-effort cleanup of stale detached processes from previous runs.
            self._cleanup_residual_processes(signal.SIGTERM)
            await asyncio.sleep(0.4)

            workspace_setup = self._pick_workspace_setup()
            if not workspace_setup:
                logger.error("No workspace setup.bash found for ROS2 stack startup")
                return False

            workspace_setup_quoted = shlex.quote(workspace_setup)
            stty_cmd = (
                "stty -F /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2608b4e7586eef118367e9c2c169b110-if00-port0 "
                "460800 raw -echo -crtscts -ixon -ixoff && "
                "stty -F /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_420b6b8a586eef11a134e0c2c169b110-if00-port0 "
                "460800 raw -echo -crtscts -ixon -ixoff"
            )
            launch_cmd = " ".join(["ros2", "launch", self.package, self.launch_file, *self.launch_args])

            cmd = [
                "bash",
                "-c",
                f"source /opt/ros/jazzy/setup.bash && source {workspace_setup_quoted} && {stty_cmd} && exec {launch_cmd}"
            ]
            
            logger.info(f"Starting ROS2 stack ({self.stack_mode}) using {workspace_setup}: {launch_cmd}")
            
            # Start the process in the background
            self.process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for easier cleanup
            )
            
            # Give it a moment to start
            await asyncio.sleep(1.0)
            
            # Check if process is still running (didn't immediately crash)
            if self.process.returncode is not None:
                logger.error(f"ROS2 stack process exited immediately with code {self.process.returncode}")
                stdout, stderr = await self.process.communicate()
                if stderr:
                    logger.error(f"Stderr: {stderr.decode()}")
                self.process = None
                return False
            
            logger.info(f"Started ROS2 stack (PID: {self.process.pid})")
            
            # Start background task to log output
            asyncio.create_task(self._log_output())

            # Keep LiDAR motor/scan recovery, but run it in the background so
            # stack mode transitions are not blocked by long serial bringup.
            self._schedule_lidar_recovery(workspace_setup)

            # Verify expected mode processes are present (and incompatible ones absent).
            ready = await self._wait_mode_healthy(self.stack_mode, timeout_sec=self.start_verify_timeout_sec)
            if not ready:
                logger.error(f"ROS2 stack failed mode health check after startup (mode={self.stack_mode})")
                await self.stop()
                return False

            # Require a short stability window so transient startups do not pass as healthy.
            await asyncio.sleep(self.start_stability_window_sec)
            if not await self.is_mode_healthy(self.stack_mode):
                logger.error(f"ROS2 stack failed post-start stability check (mode={self.stack_mode})")
                await self.stop()
                return False
            
            return True
            
        except Exception as e:
            logger.error(f"Error starting ROS2 stack: {e}")
            self.process = None
            return False

    def _schedule_lidar_recovery(self, workspace_setup: str) -> None:
        def _worker() -> None:
            try:
                time.sleep(max(0.0, float(self.lidar_boot_settle_delay_sec)))
                self._ensure_lidar_scans(workspace_setup)
            except Exception as exc:
                logger.warning(f"LiDAR recovery worker failed: {exc}")

        threading.Thread(target=_worker, name="lidar_recovery_worker", daemon=True).start()

    def _run_ros_shell_cmd(self, workspace_setup: str, command: str, timeout_sec: float = 10.0) -> subprocess.CompletedProcess:
        workspace_setup_quoted = shlex.quote(workspace_setup)
        shell_cmd = f"source /opt/ros/jazzy/setup.bash && source {workspace_setup_quoted} && {command}"
        return subprocess.run(
            ["bash", "-lc", shell_cmd],
            text=True,
            capture_output=True,
            timeout=timeout_sec,
            check=False,
        )

    def _scan_available(self, workspace_setup: str, topic_name: str) -> bool:
        probe_timeout = max(1.0, float(self.lidar_scan_probe_timeout_sec))
        cmd = (
            f"timeout {probe_timeout:g} ros2 topic echo --qos-profile sensor_data --once --field header.frame_id "
            f"{shlex.quote(topic_name)} >/dev/null 2>&1"
        )
        result = self._run_ros_shell_cmd(workspace_setup, cmd, timeout_sec=probe_timeout + 3.0)
        return result.returncode == 0

    def _ensure_lidar_scans(self, workspace_setup: str) -> None:
        topics = ["/lidar1/scan", "/lidar2/scan"]
        for topic_name in topics:
            if self._scan_available(workspace_setup, topic_name):
                continue
            logger.warning(
                f"No startup scan on {topic_name}; ROS2Manager will not call start_motor and expects lidar_watchdog to recover"
            )

    async def stop(self) -> bool:
        """
        Stop the ROS2 stack process.

        Returns True if already stopped or successfully stopped, False otherwise.
        """
        launch_pids = self._get_launch_pids()
        if self.process is None and not launch_pids:
            logger.info(f"ROS2 stack already stopped")
            return True

        try:
            signaled = self._signal_stack(signal.SIGINT)
            if signaled:
                logger.info("Sent SIGINT (Ctrl+C) to ROS2 stack")

            stopped = await self._wait_stopped(timeout_sec=self.timeout_sec)
            if not stopped:
                logger.warning("ROS2 stack did not stop on SIGINT, escalating to SIGTERM")
                self._signal_stack(signal.SIGTERM)
                stopped = await self._wait_stopped(timeout_sec=self.timeout_sec)

            if not stopped:
                logger.warning("ROS2 stack did not stop on SIGTERM, forcing SIGKILL")
                self._signal_stack(signal.SIGKILL)
                stopped = await self._wait_stopped(timeout_sec=self.timeout_sec)

            # Cleanup detached/orphaned processes that can survive launch termination.
            if self._list_residual_pids():
                logger.warning("Detected residual ROS2 stack processes after stop; cleaning up")
                self._cleanup_residual_processes(signal.SIGTERM)
                await asyncio.sleep(0.4)
                if self._list_residual_pids():
                    self._cleanup_residual_processes(signal.SIGKILL)
                    await asyncio.sleep(0.2)

            self.process = None
            if stopped:
                self._mode_health_cache.clear()
                logger.info("Stopped ROS2 stack")
                return True
            logger.error("ROS2 stack still appears to be running")
            return False
        except Exception as e:
            logger.error(f"Error stopping ROS2 stack: {e}")
            self.process = None
            return False

    async def is_running(self) -> bool:
        """Check if the ROS2 stack process is running."""
        if self.process is not None and self.process.returncode is not None:
            logger.debug(f"ROS2 process exited with code {self.process.returncode}")
            self.process = None

        if self.process is not None:
            return True

        return len(self._get_launch_pids()) > 0

    async def is_mode_healthy(self, expected_mode: Optional[str] = None) -> bool:
        mode = (expected_mode or self.stack_mode).strip().lower()
        if mode not in {"standby", "mapping", "localization"}:
            return False

        now = time.monotonic()
        cached = self._mode_health_cache.get(mode)
        if cached is not None:
            cached_t, cached_ok = cached
            if (now - float(cached_t)) <= float(self.mode_health_cache_ttl_sec):
                return bool(cached_ok)

        if not await self.is_running():
            self._mode_health_cache[mode] = (now, False)
            return False

        healthy = self._mode_process_health(mode, require_amcl_active=True)
        self._mode_health_cache[mode] = (now, healthy)
        return healthy

    def _get_launch_pids(self) -> list[int]:
        """Get PIDs for stack launch parent process(es)."""
        try:
            output = subprocess.check_output(
                ["pgrep", "-f", self.launch_match],
                text=True,
            )
            return [int(pid.strip()) for pid in output.splitlines() if pid.strip()]
        except subprocess.CalledProcessError:
            return []
        except Exception as exc:
            logger.debug(f"Failed to list ROS2 launch PIDs: {exc}")
            return []

    def _mode_process_health(self, mode: str, require_amcl_active: bool = False) -> bool:
        has_amcl = self._is_process_present(r"/lib/nav2_amcl/amcl(\s|$)")
        has_slam = self._is_process_present(r"/lib/slam_toolbox/async_slam_toolbox_node(\s|$)")
        has_lm_loc = self._is_process_present(r"__node:=lifecycle_manager_localization")
        has_lm_slam = self._is_process_present(r"__node:=lifecycle_manager_slam")

        if mode == "localization":
            if not (has_amcl and has_lm_loc and (not has_slam)):
                return False
            if require_amcl_active:
                return self._is_amcl_lifecycle_active()
            return True

        if mode == "mapping":
            return has_slam and has_lm_slam and (not has_amcl)

        # standby
        return (not has_slam) and (not has_amcl)

    async def _wait_mode_healthy(self, mode: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._mode_process_health(mode, require_amcl_active=True):
                return True
            await asyncio.sleep(0.25)
        return self._mode_process_health(mode, require_amcl_active=True)

    def _is_amcl_lifecycle_active(self) -> bool:
        """Return True when AMCL lifecycle state is active."""
        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/nickolas/ros2_ws/src/omni_src/install/setup.bash && "
            "timeout 4 ros2 service call /amcl/get_state lifecycle_msgs/srv/GetState '{}'"
        )
        try:
            output = subprocess.check_output(["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError:
            return False
        except Exception as exc:
            logger.debug(f"Failed AMCL lifecycle check: {exc}")
            return False

        return "label='active'" in output

    def _is_process_present(self, pattern: str) -> bool:
        try:
            subprocess.check_output(["pgrep", "-f", pattern], text=True)
            return True
        except subprocess.CalledProcessError:
            return False
        except Exception as exc:
            logger.debug(f"Failed process presence check for '{pattern}': {exc}")
            return False

    def _list_residual_pids(self) -> set[int]:
        pids: set[int] = set()
        for pattern in self._residual_process_patterns:
            try:
                output = subprocess.check_output(["pgrep", "-f", pattern], text=True)
            except subprocess.CalledProcessError:
                continue
            except Exception as exc:
                logger.debug(f"Failed residual PID lookup for '{pattern}': {exc}")
                continue

            for line in output.splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    pid = int(line)
                except ValueError:
                    continue
                if pid == os.getpid():
                    continue
                pids.add(pid)
        return pids

    def _cleanup_residual_processes(self, sig: signal.Signals) -> None:
        pids = self._list_residual_pids()
        if not pids:
            return

        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
            except Exception as exc:
                logger.debug(f"Failed to signal residual PID {pid} with {sig}: {exc}")

    def _signal_stack(self, sig: signal.Signals) -> bool:
        """Signal ROS2 stack process groups (tracked and externally started)."""
        pids = set(self._get_launch_pids())
        if self.process and self.process.returncode is None:
            pids.add(self.process.pid)

        if not pids:
            return False

        signaled = False
        pgids = set()
        for pid in pids:
            try:
                pgids.add(os.getpgid(pid))
            except ProcessLookupError:
                continue

        for pgid in pgids:
            try:
                os.killpg(pgid, sig)
                signaled = True
            except ProcessLookupError:
                continue
            except Exception as exc:
                logger.debug(f"Failed signaling pgid {pgid} with {sig}: {exc}")

        return signaled

    async def _wait_stopped(self, timeout_sec: float) -> bool:
        """Wait until stack launch process is gone."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self._get_launch_pids():
                if self.process and self.process.returncode is not None:
                    self.process = None
                return True
            await asyncio.sleep(0.2)
        return len(self._get_launch_pids()) == 0

    async def _log_output(self) -> None:
        """Background task to log process output."""
        if not self.process or not self.process.stdout or not self.process.stderr:
            return
        
        try:
            # Log stdout
            async def log_stream(stream, prefix):
                try:
                    while True:
                        line = await stream.readline()
                        if not line:
                            break
                        logger.debug(f"{prefix}: {line.decode().rstrip()}")
                except Exception as e:
                    logger.debug(f"Error reading {prefix}: {e}")
            
            # Run both in parallel
            await asyncio.gather(
                log_stream(self.process.stdout, "ROS2-OUT"),
                log_stream(self.process.stderr, "ROS2-ERR"),
                return_exceptions=True
            )
        except Exception as e:
            logger.debug(f"Error in log output task: {e}")

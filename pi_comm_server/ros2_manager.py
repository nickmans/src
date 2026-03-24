"""
ROS2 stack manager for launching ROS2 launch files directly.

Provides safe start/stop/status checks with subprocess management.
"""

import asyncio
import glob
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
        self.launch_match = f"ros2 launch {self.package} {self.launch_file}"
        self.process: Optional[asyncio.subprocess.Process] = None
        self.timeout_sec = 2.5
        self.start_verify_timeout_sec = 8.0
        self.localization_start_verify_timeout_sec = 20.0
        self.start_stability_window_sec = 1.0
        self.localization_stability_window_sec = 2.0
        self.lidar_boot_settle_delay_sec = 1.5
        self.serial_preflight_settle_delay_sec = 1.0
        self.lidar_scan_probe_timeout_sec = 3.0
        self.mode_health_cache_ttl_sec = 1.5
        self._mode_health_cache: dict[str, tuple[float, bool]] = {}
        self._config_search_roots = [
            "/home/nickolas/ros2_ws/src/omni_src/omni_traj/config",
            "/home/nickolas/ros2_ws/src/omni_src/install/omni_traj/share/omni_traj/config",
            "/home/nickolas/ros2_ws/install/omni_traj/share/omni_traj/config",
            "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/omni_traj/share/omni_traj/config",
        ]

        self.traj_params_file = self._pick_config_file("waypoint_traj.yaml")
        self.amcl_params_file = self._pick_config_file("amcl_localization.yaml")
        self.slam_params_file = self._pick_config_file("slam_toolbox_online_async.yaml")
        self.launch_args = self._build_launch_args(self.stack_mode)

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
            "/home/nickolas/ros2_ws/src/omni_src/install/local_setup.bash",
            "/home/nickolas/ros2_ws/src/omni_src/install/setup.bash",
            "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/local_setup.bash",
            "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/setup.bash",
            "/home/nickolas/ros2_ws/install/setup.bash",
            os.path.expanduser("~/ros2_ws/install/setup.bash"),
        ]

    def _pick_config_file(self, file_name: str) -> Optional[str]:
        for root in self._config_search_roots:
            candidate = os.path.join(root, file_name)
            if os.path.isfile(candidate):
                return candidate
        return None

    def _find_waypoint_traj_impl(self, install_prefix: str) -> Optional[str]:
        matches = glob.glob(
            os.path.join(
                install_prefix,
                "omni_traj",
                "lib",
                "python*",
                "site-packages",
                "omni_traj",
                "waypoint_traj_node.py",
            )
        )
        if matches:
            return matches[0]

        egg_links = glob.glob(
            os.path.join(
                install_prefix,
                "omni_traj",
                "lib",
                "python*",
                "site-packages",
                "*.egg-link",
            )
        )
        for egg_link in egg_links:
            try:
                with open(egg_link, "r", encoding="utf-8") as handle:
                    source_dir = handle.readline().strip()
            except OSError:
                continue

            impl_path = os.path.join(source_dir, "omni_traj", "waypoint_traj_node.py")
            if os.path.isfile(impl_path):
                return impl_path

        return None

    def _workspace_has_synced_fusion(self, setup_path: str) -> bool:
        install_prefix = os.path.dirname(setup_path)
        impl_path = self._find_waypoint_traj_impl(install_prefix)
        if not impl_path or not os.path.isfile(impl_path):
            return False

        try:
            with open(impl_path, "r", encoding="utf-8") as handle:
                return "_select_fusion_scans" in handle.read()
        except OSError:
            return False

    def _pick_workspace_setup(self) -> Optional[str]:
        for candidate in self._workspace_setup_candidates:
            if os.path.isfile(candidate) and self._workspace_has_synced_fusion(candidate):
                return candidate

        for candidate in self._workspace_setup_candidates:
            if os.path.isfile(candidate):
                logger.warning(
                    "Falling back to workspace setup without synchronized fusion marker: %s",
                    candidate,
                )
                return candidate
        return None

    def _console_devices(self) -> set[str]:
        console_devices: set[str] = set()
        try:
            with open("/proc/cmdline", "r", encoding="utf-8") as handle:
                cmdline = handle.read().strip()
        except OSError:
            return console_devices

        for token in cmdline.split():
            if not token.startswith("console="):
                continue
            tty_name = token.split("=", 1)[1].split(",", 1)[0]
            if tty_name:
                console_devices.add(f"/dev/{tty_name}")
        return console_devices

    def _pick_lidar_ports(self) -> tuple[str, str]:
        env_lidar1 = os.getenv("LIDAR1_SERIAL_PORT")
        env_lidar2 = os.getenv("LIDAR2_SERIAL_PORT")
        if env_lidar1 and env_lidar2:
            return env_lidar1, env_lidar2

        console_devices = self._console_devices()
        ttyama_ports = sorted(glob.glob("/dev/ttyAMA*"), key=lambda value: int(value.rsplit("ttyAMA", 1)[1]))
        ttyama_ports = [path for path in ttyama_ports if path not in console_devices]
        if len(ttyama_ports) >= 2:
            return ttyama_ports[0], ttyama_ports[1]

        default_lidar1 = "/dev/ttyAMA0" if os.path.exists("/dev/ttyAMA0") else "/dev/serial0"
        default_lidar2 = "/dev/ttyAMA2" if os.path.exists("/dev/ttyAMA2") else "/dev/serial1"

        if default_lidar1 in console_devices:
            default_lidar1 = "/dev/serial0"
        if default_lidar2 in console_devices:
            default_lidar2 = "/dev/serial1"

        lidar1_port = env_lidar1 or default_lidar1
        lidar2_port = env_lidar2 or default_lidar2
        return lidar1_port, lidar2_port

    def _append_launch_arg(self, args: list[str], name: str, value: object) -> None:
        value_str = "" if value is None else str(value).strip()
        if not value_str:
            return
        args.append(f"{name}:={value_str}")

    def _sanitize_launch_args(self, args: list[str]) -> list[str]:
        sanitized: list[str] = []
        for arg in args:
            if ":=" not in arg:
                logger.warning("Skipping malformed launch argument without ':=': %s", arg)
                continue

            name, value = arg.split(":=", 1)
            name = name.strip()
            value = value.strip()

            if not name:
                logger.warning("Skipping malformed launch argument with empty name: %s", arg)
                continue

            if not value:
                logger.warning("Skipping empty launch argument %s to avoid ROS2 launch failure", name)
                continue

            sanitized.append(f"{name}:={value}")

        return sanitized

    def _build_launch_args(self, stack_mode: str) -> list[str]:
        lidar1_port, lidar2_port = self._pick_lidar_ports()
        scan_mode = os.getenv("OMNI_LIDAR_SCAN_MODE", "").strip()

        args = [
            "use_mock_lidar:=false",
            "use_rviz:=false",
            "map_frame:=map",
            "publish_odom_to_base_tf:=true",
            "publish_world_to_odom_tf:=false",
            "rolling_map_enable:=true",
            "rolling_map_margin_m:=1.0",
            "persistent_obstacles_enable:=true",
            "serial_baudrate:=460800",
        ]

        self._append_launch_arg(args, "scan_mode", scan_mode)
        self._append_launch_arg(args, "lidar1_serial_port", lidar1_port)
        self._append_launch_arg(args, "lidar2_serial_port", lidar2_port)

        if self.traj_params_file:
            self._append_launch_arg(args, "traj_params_file", self.traj_params_file)
        if self.amcl_params_file:
            self._append_launch_arg(args, "amcl_params_file", self.amcl_params_file)
        if self.slam_params_file:
            self._append_launch_arg(args, "slam_params_file", self.slam_params_file)

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
            resolved_impl = self._find_waypoint_traj_impl(os.path.dirname(workspace_setup))
            resolved_impl_msg = shlex.quote(resolved_impl or "<unknown>")
            settle_s = max(0.0, float(self.serial_preflight_settle_delay_sec))
            lidar1_port, lidar2_port = self._pick_lidar_ports()
            lidar1_port_quoted = shlex.quote(lidar1_port)
            lidar2_port_quoted = shlex.quote(lidar2_port)
            stty_cmd = (
                f"sleep {settle_s:g}; "
                f"stty -F {lidar1_port_quoted} "
                "460800 raw -echo -crtscts -ixon -ixoff >/dev/null 2>&1 || "
                "echo '[ros2_manager] WARN: lidar1 serial preflight stty failed'; "
                f"stty -F {lidar2_port_quoted} "
                "460800 raw -echo -crtscts -ixon -ixoff >/dev/null 2>&1 || "
                "echo '[ros2_manager] WARN: lidar2 serial preflight stty failed'"
            )
            sanitized_launch_args = self._sanitize_launch_args(self.launch_args)
            launch_cmd = shlex.join(["ros2", "launch", self.package, self.launch_file, *sanitized_launch_args])
            trace_cmd = (
                f"echo '[ros2_manager] Using workspace setup: {workspace_setup_quoted}'; "
                f"echo '[ros2_manager] Selected waypoint_traj implementation: {resolved_impl_msg}'; "
                "python3 -c \"import omni_traj.waypoint_traj_node as module; print('[ros2_manager] Python resolved waypoint_traj module:', module.__file__)\""
            )
            root_ws_setup = "/home/nickolas/ros2_ws/install/local_setup.bash"
            root_ws_setup_cmd = ""
            if os.path.isfile(root_ws_setup):
                root_ws_setup_cmd = f"source {shlex.quote(root_ws_setup)} && "
            env_reset_cmd = (
                "unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH "
                "PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH; "
            )

            cmd = [
                "bash",
                "-c",
                f"{env_reset_cmd}source /opt/ros/jazzy/setup.bash && {root_ws_setup_cmd}source {workspace_setup_quoted} && {stty_cmd} && {trace_cmd} && exec {launch_cmd}"
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
                if stdout:
                    logger.error(f"Stdout: {stdout.decode()}")
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
            verify_timeout_sec = self.start_verify_timeout_sec
            stability_window_sec = self.start_stability_window_sec
            if self.stack_mode == "localization":
                verify_timeout_sec = self.localization_start_verify_timeout_sec
                stability_window_sec = self.localization_stability_window_sec

            require_active = self.stack_mode != "localization"
            ready = await self._wait_mode_healthy(
                self.stack_mode,
                timeout_sec=verify_timeout_sec,
                require_amcl_active=require_active,
            )
            if not ready:
                logger.error(f"ROS2 stack failed mode health check after startup (mode={self.stack_mode})")
                await self.stop()
                return False

            # Require a short stability window so transient startups do not pass as healthy.
            await asyncio.sleep(stability_window_sec)
            if not self._mode_process_health(self.stack_mode, require_amcl_active=require_active):
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

    async def _wait_mode_healthy(
        self,
        mode: str,
        timeout_sec: float,
        require_amcl_active: bool = True,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._mode_process_health(mode, require_amcl_active=require_amcl_active):
                return True
            await asyncio.sleep(0.25)
        return self._mode_process_health(mode, require_amcl_active=require_amcl_active)

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
            async def log_stream(stream, prefix, log_fn):
                try:
                    while True:
                        line = await stream.readline()
                        if not line:
                            break
                        log_fn(f"{prefix}: {line.decode(errors='replace').rstrip()}")
                except Exception as e:
                    logger.debug(f"Error reading {prefix}: {e}")
            
            # Run both in parallel
            await asyncio.gather(
                log_stream(self.process.stdout, "ROS2-OUT", logger.info),
                log_stream(self.process.stderr, "ROS2-ERR", logger.warning),
                return_exceptions=True
            )
        except Exception as e:
            logger.debug(f"Error in log output task: {e}")

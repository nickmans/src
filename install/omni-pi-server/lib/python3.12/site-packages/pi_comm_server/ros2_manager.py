"""
ROS2 stack manager for launching ROS2 launch files directly.

Provides safe start/stop/status checks with subprocess management.
"""

import asyncio
import logging
import os
import signal
import subprocess
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
        self.launch_args = [
            "use_mock_lidar:=false",
            "use_rviz:=false",
            "map_frame:=odom",
            "publish_odom_to_base_tf:=true",
            "publish_world_to_odom_tf:=false",
            "rolling_map_enable:=true",
            "rolling_map_margin_m:=1.0",
            "persistent_obstacles_enable:=true",
            "lidar1_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2608b4e7586eef118367e9c2c169b110-if00-port0",
            "lidar2_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_420b6b8a586eef11a134e0c2c169b110-if00-port0",
        ]
        self.launch_match = f"ros2 launch {self.package} {self.launch_file}"
        self.process: Optional[asyncio.subprocess.Process] = None
        self.timeout_sec = 5.0

    async def start(self) -> bool:
        """
        Start the ROS2 stack by launching the specified launch file.

        Returns True if already running or successfully started, False otherwise.
        """
        # Check if already running
        if await self.is_running():
            logger.info(f"ROS2 stack already running")
            return True

        try:
            workspace_setup = os.path.expanduser("~/ros2_ws/install/setup.bash")
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
                f"source /opt/ros/jazzy/setup.bash && source {workspace_setup} && {stty_cmd} && exec {launch_cmd}"
            ]
            
            logger.info(f"Starting ROS2 stack: {launch_cmd}")
            
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
            
            return True
            
        except Exception as e:
            logger.error(f"Error starting ROS2 stack: {e}")
            self.process = None
            return False

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

            self.process = None
            if stopped:
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

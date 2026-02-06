"""
ROS2 stack manager for launching ROS2 launch files directly.

Provides safe start/stop/status checks with subprocess management.
"""

import asyncio
import logging
import os
import signal
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
            # Source ROS2 and launch the file
            # We need to source the workspace setup and then run ros2 launch
            workspace_setup = os.path.expanduser("~/ros2_ws/install/setup.bash")
            
            # Build the command to source setup and launch
            cmd = [
                "bash",
                "-c",
                f"source /opt/ros/jazzy/setup.bash && source {workspace_setup} && ros2 launch {self.package} {self.launch_file}"
            ]
            
            logger.info(f"Starting ROS2 stack: ros2 launch {self.package} {self.launch_file}")
            
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
        # Check if already stopped
        if not await self.is_running():
            logger.info(f"ROS2 stack already stopped")
            return True

        try:
            if self.process:
                logger.info(f"Stopping ROS2 stack (PID: {self.process.pid})")
                
                # Try graceful shutdown first (SIGTERM to process group)
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    logger.debug("Process already terminated")
                    self.process = None
                    return True
                
                # Wait for graceful shutdown
                try:
                    await asyncio.wait_for(self.process.wait(), timeout=self.timeout_sec)
                    logger.info("ROS2 stack stopped gracefully")
                except asyncio.TimeoutError:
                    # Force kill if not stopped within timeout
                    logger.warning("ROS2 stack did not stop gracefully, forcing kill")
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                        await self.process.wait()
                    except ProcessLookupError:
                        pass
                
                self.process = None
                logger.info("Stopped ROS2 stack")
                return True
                
        except Exception as e:
            logger.error(f"Error stopping ROS2 stack: {e}")
            self.process = None
            return False

    async def is_running(self) -> bool:
        """Check if the ROS2 stack process is running."""
        if self.process is None:
            return False
        
        # Check if process is still alive
        if self.process.returncode is not None:
            logger.debug(f"ROS2 process exited with code {self.process.returncode}")
            self.process = None
            return False
        
        return True

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

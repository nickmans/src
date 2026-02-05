"""
ROS2 stack manager using systemd --user services.

Provides safe start/stop/status checks with timeout handling.
"""

import asyncio
import logging
import subprocess
from typing import Optional

logger = logging.getLogger(__name__)


class ROS2Manager:
    """Manage ROS2 stack via systemd --user service."""

    def __init__(self, service_name: str = "omni_ros2_stack.service"):
        self.service_name = service_name
        self.timeout_sec = 5.0

    async def start(self) -> bool:
        """
        Start the ROS2 stack service.

        Returns True if already running or successfully started, False otherwise.
        """
        # Check if already running
        if await self.is_running():
            logger.info(f"ROS2 stack already running ({self.service_name})")
            return True

        try:
            result = await asyncio.wait_for(
                self._run_command(["systemctl", "--user", "start", self.service_name]),
                timeout=self.timeout_sec,
            )
            if result:
                logger.info(f"Started ROS2 stack ({self.service_name})")
                await asyncio.sleep(0.5)  # Give it time to start
                return True
            else:
                logger.error(f"Failed to start ROS2 stack ({self.service_name})")
                return False
        except asyncio.TimeoutError:
            logger.error(f"Timeout starting ROS2 stack ({self.service_name})")
            return False
        except Exception as e:
            logger.error(f"Error starting ROS2 stack: {e}")
            return False

    async def stop(self) -> bool:
        """
        Stop the ROS2 stack service.

        Returns True if already stopped or successfully stopped, False otherwise.
        """
        # Check if already stopped
        if not await self.is_running():
            logger.info(f"ROS2 stack already stopped ({self.service_name})")
            return True

        try:
            result = await asyncio.wait_for(
                self._run_command(["systemctl", "--user", "stop", self.service_name]),
                timeout=self.timeout_sec,
            )
            if result:
                logger.info(f"Stopped ROS2 stack ({self.service_name})")
                return True
            else:
                logger.error(f"Failed to stop ROS2 stack ({self.service_name})")
                return False
        except asyncio.TimeoutError:
            logger.error(f"Timeout stopping ROS2 stack ({self.service_name})")
            return False
        except Exception as e:
            logger.error(f"Error stopping ROS2 stack: {e}")
            return False

    async def is_running(self) -> bool:
        """Check if the ROS2 stack service is running."""
        try:
            result = await asyncio.wait_for(
                self._run_command(["systemctl", "--user", "is-active", self.service_name]),
                timeout=self.timeout_sec,
            )
            return result
        except Exception as e:
            logger.debug(f"Error checking ROS2 stack status: {e}")
            return False

    async def _run_command(self, cmd: list) -> bool:
        """
        Run a shell command asynchronously.

        Returns True on success (return code 0), False otherwise.
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            stdout, stderr = await proc.communicate()
            return proc.returncode == 0
        except Exception as e:
            logger.error(f"Command failed: {' '.join(cmd)}, error: {e}")
            return False

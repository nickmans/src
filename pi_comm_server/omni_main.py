"""
Main integration script for OMNI TCP server with ROS2.

Combines TCP server, ROS2 pose publishing, and trajectory generation.
Handles CMD messages to start/stop trajectory generation.
"""

import logging
import sys
import time
import threading
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor

from tcp_server import OMNITCPServer, PoseData
from ros2_pose_node import PosePublisherNode
from ros2_trajectory_node import TrajectoryGeneratorNode
from protocol import Trajectory, CommandID

logger = logging.getLogger(__name__)


class OMNISystem:
    """
    Complete OMNI system integrating TCP server and ROS2.
    """

    def __init__(self, host: str = "192.168.1.100", port: int = 9000):
        """
        Initialize OMNI system.

        Args:
            host: IP address for TCP server
            port: Port for TCP server
        """
        self.host = host
        self.port = port
        
        # Initialize ROS2
        rclpy.init()
        
        # ROS2 nodes
        self.pose_node = PosePublisherNode()
        self.traj_node: Optional[TrajectoryGeneratorNode] = None
        
        # ROS2 executor (runs in separate thread)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.pose_node)
        
        self.executor_thread: Optional[threading.Thread] = None
        self.executor_running = False
        
        # TCP Server
        self.tcp_server = OMNITCPServer(
            host=self.host,
            port=self.port,
            on_pose_callback=self.handle_pose,
            on_cmd_callback=self.handle_command,
            get_trajectory_callback=self.get_trajectory,
        )
        
        # Trajectory generation state
        self.traj_lock = threading.Lock()
        self.trajectory_active = False
        self.traj_process: Optional[subprocess.Popen] = None
        
        logger.info("OMNI System initialized")

    def start(self):
        """Start the complete system."""
        logger.info("Starting OMNI system...")
        
        # Start ROS2 executor in separate thread
        self.executor_running = True
        self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
        self.executor_thread.start()
        
        # Start TCP server
        self.tcp_server.start()
        
        logger.info("OMNI system started")

    def stop(self):
        """Stop the complete system."""
        logger.info("Stopping OMNI system...")
        
        # Stop trajectory generation if active
        self.stop_trajectory_generation()
        
        # Stop TCP server
        self.tcp_server.stop()
        
        # Stop ROS2 executor
        self.executor_running = False
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)
        
        # Shutdown ROS2
        if self.traj_node:
            self.executor.remove_node(self.traj_node)
            self.traj_node.destroy_node()
        
        self.pose_node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        
        logger.info("OMNI system stopped")

    def _run_executor(self):
        """Run ROS2 executor (runs in separate thread)."""
        try:
            while self.executor_running:
                # Spin at 5Hz to reduce CPU usage (was 10Hz)
                self.executor.spin_once(timeout_sec=0.2)
        except Exception as e:
            logger.error(f"Error in ROS2 executor: {e}")

    def handle_pose(self, pose: PoseData):
        """
        Handle incoming POSE message from TCP server.
        
        Args:
            pose: PoseData from STM32
        """
        # Publish to ROS2
        self.pose_node.publish_pose(pose)

    def handle_command(self, command: int):
        """
        Handle incoming CMD message from TCP server.
        
        Args:
            command: Command ID from STM32
        """
        logger.info(f"Received command: {command}")
        
        if command == CommandID.START_TRAJ:
            logger.info("START_TRAJ command received")
            self.start_trajectory_generation()
        
        elif command == CommandID.STOP_TRAJ:
            logger.info("STOP_TRAJ command received")
            self.stop_trajectory_generation()
        
        else:
            logger.warning(f"Unknown command: {command}")

    def get_trajectory(self) -> Optional[Trajectory]:
        """
        Get current trajectory setpoint for sending to STM32.
        
        Returns:
            Trajectory object or None
        """
        with self.traj_lock:
            if not self.trajectory_active or self.traj_node is None:
                return None
        
        # Get setpoint from trajectory node
        setpoint = self.traj_node.get_trajectory_setpoint()
        if setpoint is None:
            return None
        
        # Create Trajectory message
        traj = Trajectory(
            x_des=setpoint['x_des'],
            y_des=setpoint['y_des'],
            yaw_des=setpoint['yaw_des'],
            vx_world=setpoint['vx_world'],
            vy_world=setpoint['vy_world'],
        )
        
        return traj

    def start_trajectory_generation(self):
        """Start trajectory generation node."""
        with self.traj_lock:
            if self.trajectory_active:
                logger.warning("Trajectory generation already active")
                return
            
            logger.info("Starting trajectory generation...")
            
            try:
                # Create trajectory node if not exists
                if self.traj_node is None:
                    self.traj_node = TrajectoryGeneratorNode()
                    self.executor.add_node(self.traj_node)
                
                self.trajectory_active = True
                
                # Enable trajectory sending in TCP server
                self.tcp_server.set_trajectory_active(True)
                
                logger.info("Trajectory generation started")
                
            except Exception as e:
                logger.error(f"Error starting trajectory generation: {e}")
                self.trajectory_active = False

    def stop_trajectory_generation(self):
        """Stop trajectory generation node."""
        with self.traj_lock:
            if not self.trajectory_active:
                logger.warning("Trajectory generation already stopped")
                return
            
            logger.info("Stopping trajectory generation...")
            
            try:
                # Disable trajectory sending in TCP server
                self.tcp_server.set_trajectory_active(False)
                
                # Remove and destroy trajectory node
                if self.traj_node:
                    self.executor.remove_node(self.traj_node)
                    self.traj_node.destroy_node()
                    self.traj_node = None
                
                self.trajectory_active = False
                
                logger.info("Trajectory generation stopped")
                
            except Exception as e:
                logger.error(f"Error stopping trajectory generation: {e}")


def main():
    """Main entry point."""
    # Set process to lower priority to avoid interfering with SSH
    import os
    try:
        os.nice(10)  # Lower priority (higher nice value)
        logger.info("Process priority lowered to nice +10")
    except Exception as e:
        logger.warning(f"Could not set process priority: {e}")
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler("/tmp/omni_system.log"),
        ],
    )
    
    logger.info("=" * 60)
    logger.info("OMNI Robot Communication System")
    logger.info("=" * 60)
    
    # Create system - bind to all interfaces (0.0.0.0) to ensure STM32 can connect
    system = OMNISystem(host="0.0.0.0", port=9000)
    
    # Setup signal handler for graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Shutdown signal received")
        system.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start system
    try:
        system.start()
        
        logger.info("System running. Press Ctrl+C to stop.")
        logger.info("Waiting for STM32 connection from 192.168.1.10 on port 9000...")
        
        # Keep main thread alive
        while True:
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error in main: {e}", exc_info=True)
    finally:
        system.stop()


if __name__ == "__main__":
    main()

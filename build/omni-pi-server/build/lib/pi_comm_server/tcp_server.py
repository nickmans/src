"""
TCP Server for OMNI robot communication with STM32.

Receives POSE messages at 5 Hz, sends TRAJ messages when active.
Handles CMD messages to start/stop ROS2 trajectory generation.
"""

import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

from protocol import (
    MessageType,
    CommandID,
    Header,
    Pose,
    Command,
    Trajectory,
    StreamParser,
    make_message,
    HEADER_SIZE,
)

logger = logging.getLogger(__name__)


@dataclass
class PoseData:
    """Current robot pose and velocity."""
    pose_t_ms: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    received_t_ms: int  # When we received it


class OMNITCPServer:
    """
    TCP Server for OMNI robot communication.
    
    Listens on specified host:port for STM32 connection.
    Receives POSE and CMD messages, sends TRAJ messages.
    """

    def __init__(
        self,
        host: str = "192.168.1.100",
        port: int = 9000,
        on_pose_callback: Optional[Callable[[PoseData], None]] = None,
        on_cmd_callback: Optional[Callable[[int], None]] = None,
        get_trajectory_callback: Optional[Callable[[], Optional[Trajectory]]] = None,
    ):
        """
        Initialize TCP server.

        Args:
            host: IP address to bind to
            port: Port to listen on
            on_pose_callback: Callback when POSE message received
            on_cmd_callback: Callback when CMD message received
            get_trajectory_callback: Callback to get trajectory setpoint for TRAJ message
        """
        self.host = host
        self.port = port
        self.on_pose_callback = on_pose_callback
        self.on_cmd_callback = on_cmd_callback
        self.get_trajectory_callback = get_trajectory_callback

        # Connection state
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.client_address: Optional[tuple] = None
        self.connected = False
        self.running = False

        # Threading
        self.accept_thread: Optional[threading.Thread] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.send_thread: Optional[threading.Thread] = None

        # Shared state (protected by locks)
        self.pose_lock = threading.Lock()
        self.latest_pose: Optional[PoseData] = None
        
        self.traj_lock = threading.Lock()
        self.trajectory_active = False  # Set to True when trajectory should be sent

        # Message sequencing
        self.traj_seq = 0
        self.pose_seq = 0

        # Parser for incoming messages
        self.parser = StreamParser()
        
        # Connection backoff
        self.reconnect_delay = 1.0  # Start with 1 second
        self.max_reconnect_delay = 10.0

        logger.info(f"OMNITCPServer initialized: {self.host}:{self.port}")

    def start(self):
        """Start the TCP server and all threads."""
        if self.running:
            logger.warning("Server already running")
            return

        self.running = True

        # Create server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(2.0)  # Reduce wake frequency to save CPU

        bind_msg = f"Server listening on {self.host}:{self.port}"
        if self.host == "0.0.0.0":
            bind_msg += " (all interfaces - accessible from 192.168.1.10)"
        logger.info(bind_msg)
        logger.info(f"Waiting for STM32 to connect from 192.168.1.10...")

        # Start accept thread
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()

        logger.info("Server started")

    def stop(self):
        """Stop the server and all threads."""
        logger.info("Stopping server...")
        self.running = False

        # Close client connection
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception as e:
                logger.error(f"Error closing client socket: {e}")
            self.client_socket = None

        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception as e:
                logger.error(f"Error closing server socket: {e}")
            self.server_socket = None

        # Wait for threads to finish
        if self.accept_thread and self.accept_thread.is_alive():
            self.accept_thread.join(timeout=2.0)
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=2.0)

        logger.info("Server stopped")

    def set_trajectory_active(self, active: bool):
        """Enable or disable trajectory sending."""
        with self.traj_lock:
            self.trajectory_active = active
        logger.info(f"Trajectory sending: {'active' if active else 'inactive'}")

    def get_latest_pose(self) -> Optional[PoseData]:
        """Get the latest pose data (thread-safe)."""
        with self.pose_lock:
            return self.latest_pose

    def _accept_loop(self):
        """Accept incoming connections (runs in separate thread)."""
        while self.running:
            try:
                if not self.server_socket:
                    break

                # Accept connection (with timeout to allow periodic checks)
                try:
                    client_socket, client_address = self.server_socket.accept()
                except socket.timeout:
                    continue  # Check running flag and try again

                logger.info(f"✓ Client connected from {client_address}")

                # Close previous client if any
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except:
                        pass

                # Set socket timeout to prevent blocking forever
                client_socket.settimeout(1.0)

                self.client_socket = client_socket
                self.client_address = client_address
                self.connected = True
                self.parser.clear()
                self.reconnect_delay = 1.0  # Reset backoff on successful connection

                # Start receive and send threads
                self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
                self.receive_thread.start()

                self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
                self.send_thread.start()

            except Exception as e:
                if self.running:
                    logger.error(f"Error in accept loop: {e}")
                    logger.info(f"Retrying in {self.reconnect_delay:.1f}s...")
                    time.sleep(self.reconnect_delay)
                    # Exponential backoff
                    self.reconnect_delay = min(self.reconnect_delay * 1.5, self.max_reconnect_delay)

    def _receive_loop(self):
        """Receive and parse messages from client (runs in separate thread)."""
        while self.running and self.connected:
            try:
                if not self.client_socket:
                    break

                # Receive data
                data = self.client_socket.recv(4096)
                if not data:
                    logger.warning("Client disconnected (no data)")
                    self.connected = False
                    break

                # Feed to parser
                self.parser.feed(data)

                # Parse all complete messages
                while True:
                    result = self.parser.parse_message()
                    if result is None:
                        break

                    header, payload = result
                    self._handle_message(header, payload)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"Error in receive loop: {e}")
                self.connected = False
                break

        logger.info("Receive loop ended")

    def _send_loop(self):
        """Send TRAJ messages at 5 Hz when trajectory is active (runs in separate thread)."""
        send_interval = 0.2  # 5 Hz = 200ms
        idle_interval = 5.0  # Check every 5s when inactive to save CPU

        while self.running and self.connected:
            try:
                # Check if trajectory should be sent
                should_send = False
                with self.traj_lock:
                    should_send = self.trajectory_active

                if should_send and self.get_trajectory_callback:
                    # Get trajectory setpoint
                    traj = self.get_trajectory_callback()
                    if traj:
                        # Send TRAJ message
                        self._send_trajectory(traj)
                    # Sleep for active rate
                    time.sleep(send_interval)
                else:
                    # Sleep longer when inactive to reduce CPU usage
                    time.sleep(idle_interval)

            except Exception as e:
                if self.running:
                    logger.error(f"Error in send loop: {e}")
                break

        logger.info("Send loop ended")

    def _handle_message(self, header: Header, payload: bytes):
        """Handle received message based on type."""
        try:
            if header.msg_type == MessageType.POSE:
                self._handle_pose(header, payload)
            elif header.msg_type == MessageType.CMD:
                self._handle_cmd(header, payload)
            else:
                logger.warning(f"Unknown message type: {header.msg_type}")

        except Exception as e:
            logger.error(f"Error handling message type {header.msg_type}: {e}")

    def _handle_pose(self, header: Header, payload: bytes):
        """Handle POSE message."""
        try:
            pose = Pose.unpack(payload)
            
            # Store latest pose
            pose_data = PoseData(
                pose_t_ms=pose.pose_t_ms,
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw,
                vx=pose.vx,
                vy=pose.vy,
                wz=pose.wz,
                received_t_ms=int(time.time() * 1000),
            )

            with self.pose_lock:
                self.latest_pose = pose_data
                self.pose_seq = header.seq

            # Callback
            if self.on_pose_callback:
                self.on_pose_callback(pose_data)

            # Log at reduced rate
            if header.seq % 25 == 0:  # Every 5 seconds at 5Hz
                logger.debug(
                    f"POSE seq={header.seq}: x={pose.x:.3f}, y={pose.y:.3f}, "
                    f"yaw={pose.yaw:.3f}, vx={pose.vx:.3f}, vy={pose.vy:.3f}"
                )

        except Exception as e:
            logger.error(f"Error parsing POSE message: {e}")

    def _handle_cmd(self, header: Header, payload: bytes):
        """Handle CMD message."""
        try:
            cmd = Command.unpack(payload)
            logger.info(f"CMD received: command={cmd.cmd_id} (seq={header.seq})")

            # Callback
            if self.on_cmd_callback:
                self.on_cmd_callback(cmd.cmd_id)

        except Exception as e:
            logger.error(f"Error parsing CMD message: {e}")

    def _send_trajectory(self, traj: Trajectory):
        """Send TRAJ message to client."""
        try:
            if not self.client_socket or not self.connected:
                return

            # Pack payload
            payload = traj.pack()

            # Create message
            self.traj_seq += 1
            message = make_message(MessageType.TRAJ, self.traj_seq, payload, crc_payload=False)

            # Send
            self.client_socket.sendall(message)

            # Log at reduced rate
            if self.traj_seq % 25 == 0:  # Every 5 seconds at 5Hz
                logger.debug(
                    f"TRAJ sent seq={self.traj_seq}: x_des={traj.x_des:.3f}, "
                    f"y_des={traj.y_des:.3f}, yaw_des={traj.yaw_des:.3f}"
                )

        except Exception as e:
            logger.error(f"Error sending TRAJ message: {e}")
            self.connected = False


def main():
    """Test main function."""
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    def on_pose(pose: PoseData):
        """Pose callback."""
        pass  # Handled by logger

    def on_cmd(command: int):
        """Command callback."""
        logger.info(f"Command received: {command}")

    def get_traj() -> Optional[Trajectory]:
        """Get trajectory setpoint."""
        # Return a simple hold position for testing
        return Trajectory(x_des=0.0, y_des=0.0, yaw_des=0.0, vx_world=0.0, vy_world=0.0)

    server = OMNITCPServer(
        host="0.0.0.0",
        port=9000,
        on_pose_callback=on_pose,
        on_cmd_callback=on_cmd,
        get_trajectory_callback=get_traj,
    )

    try:
        server.start()
        logger.info("Server running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        server.stop()


if __name__ == "__main__":
    main()

"""
Test client simulating STM32 Nucleo H755ZI.

Sends POSE messages at 5 Hz and can send CMD messages.
Receives and displays TRAJ messages from server.
"""

import socket
import struct
import time
import threading
import logging
import argparse
import math

from protocol import (
    MessageType,
    CommandID,
    Header,
    Pose,
    Trajectory,
    StreamParser,
    make_message,
    HEADER_SIZE,
)

logger = logging.getLogger(__name__)


class STM32Simulator:
    """
    Simulates STM32 behavior for testing.
    
    Sends POSE messages at 5 Hz with simulated robot motion.
    Can send CMD messages to start/stop trajectory.
    Receives and logs TRAJ messages.
    """

    def __init__(self, server_host: str = "192.168.1.100", server_port: int = 9000):
        """
        Initialize simulator.

        Args:
            server_host: Server IP address
            server_port: Server port
        """
        self.server_host = server_host
        self.server_port = server_port
        
        # Socket
        self.socket: socket.socket = None
        self.connected = False
        
        # Threads
        self.running = False
        self.send_thread: threading.Thread = None
        self.receive_thread: threading.Thread = None
        
        # Simulation state
        self.sim_time = 0.0
        self.pose_seq = 0
        self.cmd_seq = 0
        
        # Simulated robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # Motion mode
        self.motion_mode = 'stationary'  # 'stationary', 'forward', 'circle'
        
        # Parser for incoming messages
        self.parser = StreamParser()
        
        logger.info(f"STM32 Simulator initialized: target {server_host}:{server_port}")

    def connect(self):
        """Connect to server."""
        try:
            logger.info(f"Connecting to {self.server_host}:{self.server_port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_host, self.server_port))
            self.connected = True
            logger.info("Connected to server")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from server."""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        logger.info("Disconnected from server")

    def start(self):
        """Start simulator threads."""
        if self.running:
            logger.warning("Simulator already running")
            return
        
        if not self.connected:
            logger.error("Not connected to server")
            return
        
        self.running = True
        
        # Start send thread (POSE at 5 Hz)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        
        # Start receive thread (TRAJ messages)
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        
        logger.info("Simulator started")

    def stop(self):
        """Stop simulator."""
        logger.info("Stopping simulator...")
        self.running = False
        
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=2.0)
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
        
        self.disconnect()
        logger.info("Simulator stopped")

    def send_command(self, command: int):
        """
        Send CMD message.

        Args:
            command: Command ID (1=START_TRAJ, 2=STOP_TRAJ)
        """
        try:
            if not self.connected:
                logger.error("Not connected")
                return
            
            # Create command payload
            payload = struct.pack("<I", command)
            
            # Create message
            self.cmd_seq += 1
            message = make_message(MessageType.CMD, self.cmd_seq, payload, crc_payload=False)
            
            # Send
            self.socket.sendall(message)
            
            cmd_name = "START_TRAJ" if command == 1 else "STOP_TRAJ" if command == 2 else "UNKNOWN"
            logger.info(f"Sent CMD: {cmd_name} (command={command}, seq={self.cmd_seq})")
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            self.connected = False

    def set_motion_mode(self, mode: str):
        """
        Set motion simulation mode.

        Args:
            mode: 'stationary', 'forward', 'circle'
        """
        if mode in ['stationary', 'forward', 'circle']:
            self.motion_mode = mode
            logger.info(f"Motion mode set to: {mode}")
        else:
            logger.warning(f"Unknown motion mode: {mode}")

    def _send_loop(self):
        """Send POSE messages at 5 Hz."""
        send_interval = 0.2  # 5 Hz = 200ms
        
        while self.running and self.connected:
            try:
                # Update simulation
                self._update_simulation(send_interval)
                
                # Create POSE message
                self._send_pose()
                
                # Sleep
                time.sleep(send_interval)
                
            except Exception as e:
                if self.running:
                    logger.error(f"Error in send loop: {e}")
                break
        
        logger.info("Send loop ended")

    def _receive_loop(self):
        """Receive and parse TRAJ messages."""
        while self.running and self.connected:
            try:
                # Receive data
                data = self.socket.recv(4096)
                if not data:
                    logger.warning("Server disconnected (no data)")
                    self.connected = False
                    break
                
                # Feed to parser
                self.parser.feed(data)
                
                # Parse messages
                while True:
                    result = self.parser.parse_message()
                    if result is None:
                        break
                    
                    header, payload = result
                    self._handle_message(header, payload)
                
            except Exception as e:
                if self.running:
                    logger.error(f"Error in receive loop: {e}")
                self.connected = False
                break
        
        logger.info("Receive loop ended")

    def _update_simulation(self, dt: float):
        """Update simulated robot state."""
        self.sim_time += dt
        
        # Update state based on motion mode
        if self.motion_mode == 'stationary':
            # No motion
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
        
        elif self.motion_mode == 'forward':
            # Move forward at 0.2 m/s
            speed = 0.2
            self.vx = speed * math.cos(self.yaw)
            self.vy = speed * math.sin(self.yaw)
            self.wz = 0.0
        
        elif self.motion_mode == 'circle':
            # Circular motion: radius=1m, speed=0.2m/s
            radius = 1.0
            speed = 0.2
            omega = speed / radius
            
            theta = omega * self.sim_time
            self.x = radius * math.cos(theta)
            self.y = radius * math.sin(theta)
            self.yaw = theta + math.pi / 2
            
            self.vx = -radius * omega * math.sin(theta)
            self.vy = radius * omega * math.cos(theta)
            self.wz = omega
            
            return  # Skip integration below
        
        # Integrate velocities
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw += self.wz * dt
        
        # Wrap yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

    def _send_pose(self):
        """Send POSE message."""
        try:
            # Create pose
            pose_t_ms = int(self.sim_time * 1000)
            pose = Pose(
                pose_t_ms=pose_t_ms,
                x=self.x,
                y=self.y,
                yaw=self.yaw,
                vx=self.vx,
                vy=self.vy,
                wz=self.wz,
            )
            
            # Pack payload
            payload = pose.pack()
            
            # Create message
            self.pose_seq += 1
            message = make_message(MessageType.POSE, self.pose_seq, payload, crc_payload=False)
            
            # Send
            self.socket.sendall(message)
            
            # Log at reduced rate
            if self.pose_seq % 25 == 0:  # Every 5 seconds
                logger.info(
                    f"Sent POSE seq={self.pose_seq}: x={self.x:.3f}, y={self.y:.3f}, "
                    f"yaw={self.yaw:.3f}, vx={self.vx:.3f}, vy={self.vy:.3f}"
                )
            
        except Exception as e:
            logger.error(f"Error sending POSE: {e}")
            self.connected = False

    def _handle_message(self, header: Header, payload: bytes):
        """Handle received message."""
        try:
            if header.msg_type == MessageType.TRAJ:
                self._handle_traj(header, payload)
            else:
                logger.warning(f"Received unexpected message type: {header.msg_type}")
        except Exception as e:
            logger.error(f"Error handling message: {e}")

    def _handle_traj(self, header: Header, payload: bytes):
        """Handle TRAJ message."""
        try:
            traj = Trajectory.unpack(payload)
            
            # Log at reduced rate
            if header.seq % 25 == 0:  # Every 5 seconds
                logger.info(
                    f"Received TRAJ seq={header.seq}: x_des={traj.x_des:.3f}, "
                    f"y_des={traj.y_des:.3f}, yaw_des={traj.yaw_des:.3f}, "
                    f"vx={traj.vx_world:.3f}, vy={traj.vy_world:.3f}"
                )
        except Exception as e:
            logger.error(f"Error parsing TRAJ message: {e}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="STM32 Simulator for OMNI system")
    parser.add_argument("--host", default="192.168.1.100", help="Server host")
    parser.add_argument("--port", type=int, default=9000, help="Server port")
    parser.add_argument("--motion", default="stationary", 
                       choices=['stationary', 'forward', 'circle'],
                       help="Motion simulation mode")
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    
    logger.info("=" * 60)
    logger.info("STM32 Simulator")
    logger.info("=" * 60)
    
    # Create simulator
    sim = STM32Simulator(server_host=args.host, server_port=args.port)
    sim.set_motion_mode(args.motion)
    
    # Connect
    if not sim.connect():
        logger.error("Failed to connect to server")
        return
    
    # Start
    sim.start()
    
    # Interactive commands
    logger.info("\nCommands:")
    logger.info("  1 or start - Send START_TRAJ command")
    logger.info("  2 or stop  - Send STOP_TRAJ command")
    logger.info("  m <mode>   - Set motion mode (stationary/forward/circle)")
    logger.info("  q or quit  - Quit")
    logger.info("")
    
    try:
        while True:
            try:
                cmd = input("> ").strip().lower()
                
                if cmd in ['q', 'quit', 'exit']:
                    break
                elif cmd in ['1', 'start']:
                    sim.send_command(CommandID.START_TRAJ)
                elif cmd in ['2', 'stop']:
                    sim.send_command(CommandID.STOP_TRAJ)
                elif cmd.startswith('m '):
                    mode = cmd.split()[1]
                    sim.set_motion_mode(mode)
                else:
                    logger.info("Unknown command")
            except EOFError:
                break
            except KeyboardInterrupt:
                break
    finally:
        sim.stop()
        logger.info("Simulator exited")


if __name__ == "__main__":
    main()

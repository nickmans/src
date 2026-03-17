"""
Main TCP server for OMNI robot communication.

- Listens for one TCP client (STM32)
- Parses POSE messages at 5 Hz
- Generates trajectories asynchronously (latest-wins strategy)
- Handles commands (SET_IDLE, START/STOP ROS2, GET_STATUS)
- Maintains state and timing info
"""

import asyncio
import logging
import sys
import time
from dataclasses import dataclass
from typing import Optional

from protocol import (
    CommandID,
    Command,
    Header,
    MessageType,
    Pose,
    Status,
    StreamParser,
    Trajectory,
    make_message,
)
from planner_stub import PoseState, make_traj_from_pose
from ros2_manager import ROS2Manager

logger = logging.getLogger(__name__)


@dataclass
class ServerState:
    """Server state and metrics."""
    idle_mode: bool = False
    ros2_running: bool = False
    last_pose_seq: int = 0
    last_pose_t_ms: int = 0
    last_pose: Optional[PoseState] = None
    last_pose_latency_ms: float = 0.0
    last_traj_seq: int = 0
    last_pose_received_t_ms: int = 0
    
    # Yaw alignment for saved map mode
    use_saved_map: bool = False
    yaw_offset: float = 0.0  # Offset from robot frame to map frame (radians)
    yaw_calibrated: bool = False
    calibration_yaw_samples: list = None
    calibration_pose_count: int = 0
    
    def __post_init__(self):
        if self.calibration_yaw_samples is None:
            self.calibration_yaw_samples = []


class OMNIServer:
    """Main TCP server for OMNI communication."""

    def __init__(self, host: str = "0.0.0.0", port: int = 9000, enable_ros2_cmds: bool = False, use_saved_map: bool = False, map_yaw_offset: float = 0.0):
        self.host = host
        self.port = port
        self.enable_ros2_cmds = enable_ros2_cmds
        self.state = ServerState(use_saved_map=use_saved_map)
        self.ros2_mgr = ROS2Manager()
        
        # Saved map configuration
        self.calibration_pose_threshold = 5  # Number of stationary poses to average for calibration
        self.calibration_velocity_threshold = 0.05  # Max velocity (m/s) to consider stationary
        self.map_yaw_offset = map_yaw_offset  # Known offset if provided at launch

        # Network
        self.server: Optional[asyncio.Server] = None
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.parser = StreamParser()

        # Output queue: list of (msg_type, seq, payload)
        self.tx_queue: asyncio.Queue = asyncio.Queue(maxsize=100)

        # Trajectory planning: track pending job
        self.pending_traj_seq: Optional[int] = None
        self.traj_planning_task: Optional[asyncio.Task] = None

        # Watchdog: timeout if no POSE for >1s
        self.watchdog_timeout_sec = 1.0
        self.last_watchdog_check_t_ms = int(time.time() * 1000)

        logger.info(f"OMNIServer initialized: {self.host}:{self.port}")

    async def start(self) -> None:
        """Start the server."""
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        addr = self.server.sockets[0].getsockname()
        logger.info(f"Server listening on {addr}")
        async with self.server:
            await self.server.serve_forever()

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        """Handle a single client connection."""
        addr = writer.get_extra_info("peername")
        logger.info(f"Client connected from {addr}")

        # Drop any existing client
        if self.writer is not None:
            try:
                self.writer.close()
                await self.writer.wait_closed()
                logger.info("Dropped previous client")
            except Exception as e:
                logger.warning(f"Error closing previous client: {e}")

        self.reader = reader
        self.writer = writer
        self.parser.clear()
        self.state.last_pose_received_t_ms = int(time.time() * 1000)

        try:
            # Start background tasks
            rx_task = asyncio.create_task(self._rx_loop())
            tx_task = asyncio.create_task(self._tx_loop())
            watchdog_task = asyncio.create_task(self._watchdog_loop())

            await asyncio.gather(rx_task, tx_task, watchdog_task)
        except asyncio.CancelledError:
            logger.info("Client handler cancelled")
        except Exception as e:
            logger.error(f"Client handler error: {e}")
        finally:
            # Cleanup
            if self.traj_planning_task and not self.traj_planning_task.done():
                self.traj_planning_task.cancel()
                try:
                    await self.traj_planning_task
                except asyncio.CancelledError:
                    pass

            try:
                self.writer.close()
                await self.writer.wait_closed()
            except Exception as e:
                logger.warning(f"Error closing writer: {e}")

            self.reader = None
            self.writer = None
            logger.info(f"Client disconnected: {addr}")

    async def _rx_loop(self) -> None:
        """Receive and parse messages from client."""
        while self.reader is not None:
            try:
                # Read data with timeout
                data = await asyncio.wait_for(self.reader.read(4096), timeout=2.0)
                if not data:
                    logger.info("Client closed connection")
                    break

                self.parser.feed(data)

                # Parse all available messages
                while True:
                    result = self.parser.parse_message()
                    if result is None:
                        break
                    header, payload = result
                    await self._handle_message(header, payload)

            except asyncio.TimeoutError:
                # Timeout is OK; just wait for more data
                pass
            except Exception as e:
                logger.error(f"RX error: {e}")
                break

    async def _tx_loop(self) -> None:
        """Send queued messages to client."""
        while self.writer is not None:
            try:
                # Get message from queue with timeout
                msg_type, seq, payload = await asyncio.wait_for(self.tx_queue.get(), timeout=1.0)

                # Pack and send
                frame = make_message(msg_type, seq, payload)
                self.writer.write(frame)
                await self.writer.drain()

            except asyncio.TimeoutError:
                # Queue empty; that's OK
                pass
            except Exception as e:
                logger.error(f"TX error: {e}")
                break

    async def _watchdog_loop(self) -> None:
        """Watchdog: detect client timeout and set idle."""
        while self.writer is not None:
            try:
                await asyncio.sleep(0.2)
                now_ms = int(time.time() * 1000)
                elapsed_ms = now_ms - self.state.last_pose_received_t_ms

                if elapsed_ms > self.watchdog_timeout_sec * 1000:
                    if not self.state.idle_mode:
                        logger.warning(f"Watchdog: no POSE for {elapsed_ms}ms, setting idle")
                        self.state.idle_mode = True

            except Exception as e:
                logger.error(f"Watchdog error: {e}")

    async def _handle_message(self, header: Header, payload: bytes) -> None:
        """Handle incoming message."""
        try:
            logger.debug(f"Received message: type={header.msg_type}, seq={header.seq}, len={len(payload)}")
            if header.msg_type == MessageType.POSE:
                await self._handle_pose(header, payload)
            elif header.msg_type == MessageType.CMD:
                await self._handle_cmd(header, payload)
            elif header.msg_type == MessageType.EVENT:
                logger.debug(f"Received EVENT seq={header.seq}")
            else:
                logger.warning(f"Unknown message type: {header.msg_type}")
        except Exception as e:
            logger.error(f"Error handling message: {e}")

    async def _handle_pose(self, header: Header, payload: bytes) -> None:
        """Handle POSE message from STM32."""
        try:
            pose_msg = Pose.unpack(payload)
            now_ms = int(time.time() * 1000)
            latency_ms = now_ms - pose_msg.pose_t_ms

            # Update state
            self.state.last_pose_seq = header.seq
            self.state.last_pose_t_ms = pose_msg.pose_t_ms
            self.state.last_pose_latency_ms = latency_ms
            self.state.last_pose_received_t_ms = now_ms

            # Raw pose from STM32
            raw_pose = PoseState(
                x=pose_msg.x,
                y=pose_msg.y,
                yaw=pose_msg.yaw,
                vx=pose_msg.vx,
                vy=pose_msg.vy,
                wz=pose_msg.wz,
                t_ms=pose_msg.pose_t_ms,
            )
            
            # Handle yaw calibration for saved map mode
            if self.state.use_saved_map and not self.state.yaw_calibrated:
                await self._calibrate_yaw_offset(raw_pose)
            
            # Transform pose to map frame if using saved map
            pose_state = self._transform_pose_to_map_frame(raw_pose)
            self.state.last_pose = pose_state

            logger.debug(
                f"POSE seq={header.seq} x={pose_state.x:.2f} y={pose_state.y:.2f} yaw={pose_state.yaw:.2f} "
                f"latency={latency_ms:.1f}ms calibrated={self.state.yaw_calibrated}"
            )

            # Trigger trajectory planning (latest-wins)
            if self.traj_planning_task and not self.traj_planning_task.done():
                self.traj_planning_task.cancel()
                logger.debug(f"Cancelled previous traj planning for seq {self.pending_traj_seq}")

            self.pending_traj_seq = header.seq
            self.traj_planning_task = asyncio.create_task(self._plan_and_send_traj(pose_state, header.seq))

        except Exception as e:
            logger.error(f"Error handling POSE: {e}")

    async def _calibrate_yaw_offset(self, pose: PoseState) -> None:
        """Calibrate yaw offset using initial stationary poses."""
        # Check if robot is stationary (low velocity)
        velocity = (pose.vx**2 + pose.vy**2)**0.5
        
        if velocity < self.calibration_velocity_threshold and abs(pose.wz) < 0.1:
            # Collect stationary pose yaw samples
            self.state.calibration_yaw_samples.append(pose.yaw)
            self.state.calibration_pose_count += 1
            
            logger.info(
                f"Calibration sample {self.state.calibration_pose_count}/{self.calibration_pose_threshold}: "
                f"yaw={pose.yaw:.3f} rad ({pose.yaw*180/3.14159:.1f}°)"
            )
            
            # Once we have enough samples, compute the average offset
            if self.state.calibration_pose_count >= self.calibration_pose_threshold:
                import math
                
                # Use circular mean for angles
                sin_sum = sum(math.sin(yaw) for yaw in self.state.calibration_yaw_samples)
                cos_sum = sum(math.cos(yaw) for yaw in self.state.calibration_yaw_samples)
                mean_yaw = math.atan2(sin_sum, cos_sum)
                
                # Calculate offset (if map_yaw_offset provided, use it; otherwise assume map is at 0)
                self.state.yaw_offset = self.map_yaw_offset - mean_yaw
                self.state.yaw_calibrated = True
                
                logger.info(
                    f"Yaw calibration complete! Robot initial yaw: {mean_yaw:.3f} rad ({mean_yaw*180/3.14159:.1f}°), "
                    f"Map offset: {self.map_yaw_offset:.3f} rad, "
                    f"Computed offset: {self.state.yaw_offset:.3f} rad ({self.state.yaw_offset*180/3.14159:.1f}°)"
                )
                
                # Clear samples to free memory
                self.state.calibration_yaw_samples.clear()
    
    def _transform_pose_to_map_frame(self, pose: PoseState) -> PoseState:
        """Transform pose from robot frame to map frame using yaw offset."""
        if not self.state.use_saved_map or not self.state.yaw_calibrated:
            # No transformation needed
            return pose
        
        import math
        from planner_stub import wrap_yaw
        
        # Apply yaw offset
        new_yaw = wrap_yaw(pose.yaw + self.state.yaw_offset)
        
        # Transform velocities to map frame
        cos_offset = math.cos(self.state.yaw_offset)
        sin_offset = math.sin(self.state.yaw_offset)
        
        new_vx = pose.vx * cos_offset - pose.vy * sin_offset
        new_vy = pose.vx * sin_offset + pose.vy * cos_offset
        
        return PoseState(
            x=pose.x,
            y=pose.y,
            yaw=new_yaw,
            vx=new_vx,
            vy=new_vy,
            wz=pose.wz,
            t_ms=pose.t_ms,
        )
    
    async def _plan_and_send_traj(self, pose: PoseState, reply_to_seq: int) -> None:
        """Plan trajectory asynchronously and send via TX queue."""
        try:
            # Simulate planning delay (in real system, call actual planner)
            await asyncio.sleep(0.01)

            # Check if we got a newer POSE while planning
            if reply_to_seq != self.pending_traj_seq:
                logger.debug(f"Dropping traj for seq {reply_to_seq}, newer seq available")
                return

            # Generate trajectory (pose is already in map frame if calibrated)
            knots = make_traj_from_pose(pose, idle=self.state.idle_mode, dt=0.05, horizon=1.2)

            # Create TRAJ message
            traj = Trajectory(
                reply_to_pose_seq=reply_to_seq,
                traj_t0_ms=pose.t_ms,
                dt=0.05,
                knots=knots,
                flags=1 if self.state.idle_mode else 2,  # bit0=idle, bit1=has_vel
            )

            payload = traj.pack()
            self.state.last_traj_seq = reply_to_seq

            # Enqueue for transmission
            await self.tx_queue.put((MessageType.TRAJ, reply_to_seq, payload))
            logger.debug(f"Queued TRAJ for POSE seq={reply_to_seq}, {len(knots)} knots")

        except asyncio.CancelledError:
            logger.debug(f"Traj planning cancelled for seq {reply_to_seq}")
        except Exception as e:
            logger.error(f"Error in trajectory planning: {e}")

    async def _handle_cmd(self, header: Header, payload: bytes) -> None:
        """Handle CMD message from STM32."""
        try:
            cmd = Command.unpack(payload)
            arg_str = cmd.arg.decode("utf-8", errors="ignore") if cmd.arg else ""

            logger.info(f"CMD seq={header.seq} cmd_id={cmd.cmd_id} arg={arg_str}")

            if cmd.cmd_id == CommandID.START_ROS2:
                success = await self.ros2_mgr.start()
                self.state.ros2_running = success
                if success:
                    await self.tx_queue.put((MessageType.ACK, header.seq, b""))
                else:
                    await self.tx_queue.put((MessageType.NACK, header.seq, b"Failed to start ROS2"))

            elif cmd.cmd_id == CommandID.STOP_ROS2:
                success = await self.ros2_mgr.stop()
                self.state.ros2_running = False
                if success:
                    await self.tx_queue.put((MessageType.ACK, header.seq, b""))
                else:
                    await self.tx_queue.put((MessageType.NACK, header.seq, b"Failed to stop ROS2"))

            elif cmd.cmd_id == CommandID.STOP_TRAJ:
                logger.info("STOP_TRAJ command received - stopping ROS2 and relaunching")
                success = await self.ros2_mgr.stop()
                self.state.ros2_running = False
                if success:
                    await self.tx_queue.put((MessageType.ACK, header.seq, b""))
                    # Give the OS/ROS middleware a brief moment to release resources before relaunch.
                    await asyncio.sleep(0.5)

                    # Relaunch ROS2 stack immediately after stopping
                    relaunch_success = await self.ros2_mgr.start()
                    self.state.ros2_running = relaunch_success
                    if relaunch_success:
                        logger.info("ROS2 stack relaunched successfully after STOP_TRAJ")
                    else:
                        logger.error("Failed to relaunch ROS2 stack after STOP_TRAJ")
                else:
                    await self.tx_queue.put((MessageType.NACK, header.seq, b"Failed to stop ROS2"))

            else:
                logger.warning(f"Unknown command: {cmd.cmd_id}")
                await self.tx_queue.put((MessageType.NACK, header.seq, b"Unknown command"))

        except Exception as e:
            logger.error(f"Error handling CMD: {e}")


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="OMNI TCP Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=9000, help="Bind port (default: 9000)")
    parser.add_argument(
        "--enable-ros2-cmds",
        action="store_true",
        help="Enable ROS2 stack control commands (default: disabled)",
    )
    parser.add_argument(
        "--use-saved-map",
        action="store_true",
        help="Enable saved map mode with yaw frame alignment (default: disabled)",
    )
    parser.add_argument(
        "--map-yaw-offset",
        type=float,
        default=0.0,
        help="Known yaw offset of map frame in radians (default: 0.0, assumes map is at 0°)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )

    args = parser.parse_args()

    # Setup logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    server = OMNIServer(
        host=args.host, 
        port=args.port, 
        enable_ros2_cmds=args.enable_ros2_cmds,
        use_saved_map=args.use_saved_map,
        map_yaw_offset=args.map_yaw_offset,
    )
    
    if args.use_saved_map:
        logger.info(f"Saved map mode enabled. Map yaw offset: {args.map_yaw_offset:.3f} rad ({args.map_yaw_offset*180/3.14159:.1f}°)")
        logger.info("Waiting for initial stationary poses to calibrate robot yaw...")
    
    try:
        await server.start()
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

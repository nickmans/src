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


class OMNIServer:
    """Main TCP server for OMNI communication."""

    def __init__(self, host: str = "0.0.0.0", port: int = 9000, enable_ros2_cmds: bool = False):
        self.host = host
        self.port = port
        self.enable_ros2_cmds = enable_ros2_cmds
        self.state = ServerState()
        self.ros2_mgr = ROS2Manager()

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

            pose_state = PoseState(
                x=pose_msg.x,
                y=pose_msg.y,
                yaw=pose_msg.yaw,
                vx=pose_msg.vx,
                vy=pose_msg.vy,
                wz=pose_msg.wz,
                t_ms=pose_msg.pose_t_ms,
            )
            self.state.last_pose = pose_state

            logger.debug(
                f"POSE seq={header.seq} x={pose_msg.x:.2f} y={pose_msg.y:.2f} "
                f"latency={latency_ms:.1f}ms"
            )

            # Trigger trajectory planning (latest-wins)
            if self.traj_planning_task and not self.traj_planning_task.done():
                self.traj_planning_task.cancel()
                logger.debug(f"Cancelled previous traj planning for seq {self.pending_traj_seq}")

            self.pending_traj_seq = header.seq
            self.traj_planning_task = asyncio.create_task(self._plan_and_send_traj(pose_state, header.seq))

        except Exception as e:
            logger.error(f"Error handling POSE: {e}")

    async def _plan_and_send_traj(self, pose: PoseState, reply_to_seq: int) -> None:
        """Plan trajectory asynchronously and send via TX queue."""
        try:
            # Simulate planning delay (in real system, call actual planner)
            await asyncio.sleep(0.01)

            # Check if we got a newer POSE while planning
            if reply_to_seq != self.pending_traj_seq:
                logger.debug(f"Dropping traj for seq {reply_to_seq}, newer seq available")
                return

            # Generate trajectory
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

            if cmd.cmd_id == CommandID.SET_IDLE:
                # Toggle idle mode
                if arg_str.lower() == "true":
                    self.state.idle_mode = True
                elif arg_str.lower() == "false":
                    self.state.idle_mode = False
                else:
                    self.state.idle_mode = not self.state.idle_mode

                logger.info(f"Set idle_mode = {self.state.idle_mode}")
                await self.tx_queue.put((MessageType.ACK, header.seq, b""))

            elif cmd.cmd_id == CommandID.START_ROS2:
                if not self.enable_ros2_cmds:
                    logger.warning("ROS2 commands disabled; NACK")
                    await self.tx_queue.put((MessageType.NACK, header.seq, b"ROS2 commands disabled"))
                else:
                    success = await self.ros2_mgr.start()
                    self.state.ros2_running = success
                    if success:
                        await self.tx_queue.put((MessageType.ACK, header.seq, b""))
                    else:
                        await self.tx_queue.put((MessageType.NACK, header.seq, b"Failed to start ROS2"))

            elif cmd.cmd_id == CommandID.STOP_ROS2:
                if not self.enable_ros2_cmds:
                    logger.warning("ROS2 commands disabled; NACK")
                    await self.tx_queue.put((MessageType.NACK, header.seq, b"ROS2 commands disabled"))
                else:
                    success = await self.ros2_mgr.stop()
                    self.state.ros2_running = False
                    if success:
                        await self.tx_queue.put((MessageType.ACK, header.seq, b""))
                    else:
                        await self.tx_queue.put((MessageType.NACK, header.seq, b"Failed to stop ROS2"))

            elif cmd.cmd_id == CommandID.GET_STATUS:
                status = Status(
                    status_t_ms=int(time.time() * 1000),
                    idle=self.state.idle_mode,
                    ros2_running=self.state.ros2_running,
                    last_pose_seq=self.state.last_pose_seq,
                    last_traj_seq=self.state.last_traj_seq,
                    last_pose_latency_ms=self.state.last_pose_latency_ms,
                )
                payload = status.pack()
                await self.tx_queue.put((MessageType.STATUS, header.seq, payload))

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

    server = OMNIServer(host=args.host, port=args.port, enable_ros2_cmds=args.enable_ros2_cmds)
    try:
        await server.start()
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

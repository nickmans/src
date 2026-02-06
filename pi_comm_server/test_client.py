"""
Test client simulating STM32 NUCLEO H755.

Connects to server, sends POSE at 5 Hz, receives TRAJ, and allows interactive commands.
"""

import asyncio
import logging
import struct
import sys
import time
from dataclasses import dataclass

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

logger = logging.getLogger(__name__)


@dataclass
class SimulatedPose:
    """Simulated robot pose that moves in a circle."""
    radius: float = 1.0
    speed: float = 0.5  # m/s
    start_time: float = 0.0

    def get_pose_at(self, t_sec: float) -> tuple:
        """Get pose at time t (in seconds)."""
        theta = self.speed * t_sec / self.radius  # rad
        x = self.radius * (1.0 - __import__("math").cos(theta))
        y = self.radius * __import__("math").sin(theta)
        yaw = theta
        vx = self.speed * __import__("math").sin(theta)
        vy = self.speed * __import__("math").cos(theta)
        wz = self.speed / self.radius
        return x, y, yaw, vx, vy, wz


class TestClient:
    """Client that simulates STM32 behavior."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9000):
        self.host = host
        self.port = port
        self.reader: asyncio.StreamReader | None = None
        self.writer: asyncio.StreamWriter | None = None
        self.parser = StreamParser()
        self.seq_tx = 0
        self.seq_rx = 0
        self.pose_sim = SimulatedPose()
        self.running = True

    async def connect(self) -> bool:
        """Connect to server."""
        try:
            self.reader, self.writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port), timeout=5.0
            )
            logger.info(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    async def run(self) -> None:
        """Main run loop."""
        if not await self.connect():
            return

        try:
            # Start background tasks
            rx_task = asyncio.create_task(self._rx_loop())
            tx_task = asyncio.create_task(self._tx_loop())
            cmd_task = asyncio.create_task(self._cmd_loop())

            await asyncio.gather(rx_task, tx_task, cmd_task)
        except Exception as e:
            logger.error(f"Run error: {e}")
        finally:
            self.running = False
            if self.writer:
                self.writer.close()
                await self.writer.wait_closed()

    async def _rx_loop(self) -> None:
        """Receive messages from server."""
        while self.running and self.reader:
            try:
                data = await asyncio.wait_for(self.reader.read(4096), timeout=2.0)
                if not data:
                    logger.info("Server closed connection")
                    self.running = False
                    break

                self.parser.feed(data)

                while True:
                    result = self.parser.parse_message()
                    if result is None:
                        break
                    header, payload = result
                    await self._handle_message(header, payload)

            except asyncio.TimeoutError:
                pass
            except Exception as e:
                logger.error(f"RX error: {e}")
                self.running = False
                break

    async def _tx_loop(self) -> None:
        """Send POSE at 5 Hz."""
        start_t = time.time()
        pose_interval = 0.2  # 5 Hz

        while self.running and self.writer:
            try:
                elapsed = time.time() - start_t
                x, y, yaw, vx, vy, wz = self.pose_sim.get_pose_at(elapsed)

                pose_msg = Pose(
                    pose_t_ms=int(elapsed * 1000),
                    x=x,
                    y=y,
                    yaw=yaw,
                    vx=vx,
                    vy=vy,
                    wz=wz,
                )

                payload = pose_msg.pack()
                frame = make_message(MessageType.POSE, self.seq_tx, payload)

                self.writer.write(frame)
                await self.writer.drain()

                logger.info(
                    f"TX POSE seq={self.seq_tx} x={x:.3f} y={y:.3f} yaw={yaw:.3f} "
                    f"vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}"
                )
                self.seq_tx += 1

                await asyncio.sleep(pose_interval)

            except Exception as e:
                logger.error(f"TX error: {e}")
                self.running = False
                break

    async def _cmd_loop(self) -> None:
        """Interactive command input loop."""
        loop = asyncio.get_event_loop()
        while self.running:
            try:
                # Read command from stdin in executor to avoid blocking
                cmd_line = await loop.run_in_executor(None, input, "\n> ")
                await self._send_command(cmd_line.strip())
            except EOFError:
                self.running = False
                break
            except Exception as e:
                logger.error(f"Cmd error: {e}")

    async def _send_command(self, cmd_str: str) -> None:
        """Parse and send a command."""
        if not self.writer or not cmd_str:
            return

        parts = cmd_str.split(maxsplit=1)
        cmd_name = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else ""

        try:
            if cmd_name == "start" or cmd_name == "start_ros2" or cmd_name == "1":
                cmd_msg = Command(cmd_id=CommandID.START_ROS2, arg=b"")
                payload = cmd_msg.pack()
                frame = make_message(MessageType.CMD, self.seq_tx, payload)
                self.writer.write(frame)
                await self.writer.drain()
                self.seq_tx += 1
                logger.info(f"Sent START_ROS2 (cmd=1)")

            elif cmd_name == "stop" or cmd_name == "stop_ros2" or cmd_name == "0":
                cmd_msg = Command(cmd_id=CommandID.STOP_ROS2, arg=b"")
                payload = cmd_msg.pack()
                frame = make_message(MessageType.CMD, self.seq_tx, payload)
                self.writer.write(frame)
                await self.writer.drain()
                self.seq_tx += 1
                logger.info(f"Sent STOP_ROS2 (cmd=0)")

            elif cmd_name == "help":
                print(
                    "\nAvailable commands:\n"
                    "  1 or start          - Start ROS2 stack (launch dual_sllidar)\n"
                    "  0 or stop           - Stop ROS2 stack\n"
                    "  quit                - Disconnect\n"
                )

            elif cmd_name == "quit":
                self.running = False

            else:
                print("Unknown command. Type 'help' for available commands.")

        except Exception as e:
            logger.error(f"Command error: {e}")

    async def _handle_message(self, header: Header, payload: bytes) -> None:
        """Handle incoming message from server."""
        try:
            if header.msg_type == MessageType.TRAJ:
                traj = Trajectory.unpack(payload)
                logger.info(
                    f"RX TRAJ seq={header.seq} reply_to={traj.reply_to_pose_seq} "
                    f"knots={len(traj.knots)} dt={traj.dt:.3f}s"
                )
                # Print first and last knot
                if traj.knots:
                    x0, y0, y0_ang, vx0, vy0, wz0 = traj.knots[0]
                    xf, yf, yf_ang, vxf, vyf, wzf = traj.knots[-1]
                    logger.info(
                        f"  Start: x={x0:.3f} y={y0:.3f} yaw={y0_ang:.3f} "
                        f"vx={vx0:.3f} vy={vy0:.3f} wz={wz0:.3f}"
                    )
                    logger.info(
                        f"  End:   x={xf:.3f} y={yf:.3f} yaw={yf_ang:.3f} "
                        f"vx={vxf:.3f} vy={vyf:.3f} wz={wzf:.3f}"
                    )

            elif header.msg_type == MessageType.ACK:
                logger.info(f"RX ACK seq={header.seq}")

            elif header.msg_type == MessageType.NACK:
                msg = payload.decode("utf-8", errors="ignore")
                logger.warning(f"RX NACK seq={header.seq} msg={msg}")

            elif header.msg_type == MessageType.STATUS:
                status = Status.unpack(payload)
                logger.info(
                    f"RX STATUS: idle={status.idle} ros2_running={status.ros2_running} "
                    f"last_pose_seq={status.last_pose_seq} last_traj_seq={status.last_traj_seq} "
                    f"latency={status.last_pose_latency_ms:.1f}ms"
                )

            else:
                logger.debug(f"RX message type={header.msg_type} seq={header.seq}")

        except Exception as e:
            logger.error(f"Error handling message: {e}")


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="OMNI Test Client (STM32 Simulator)")
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9000, help="Server port (default: 9000)")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    client = TestClient(host=args.host, port=args.port)
    try:
        await client.run()
    except KeyboardInterrupt:
        logger.info("Client interrupted by user")
    except Exception as e:
        logger.error(f"Client error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

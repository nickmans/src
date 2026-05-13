"""
Binary protocol for OMNI UDP communication.

Framed messages with header and optional CRC validation.
All integers are little-endian.
"""

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

# Constants
MAGIC = 0x4F4D4E49  # 'OMNI'
VERSION = 1
HEADER_SIZE = 24  # bytes: magic(4) + version(2) + msg_type(2) + seq(4) + t_ms(4) + payload_len(4) + crc32(4)
MAX_PAYLOAD_SIZE = 65535


class MessageType(IntEnum):
    """Message type IDs."""
    POSE = 1
    EVENT = 3
    TRAJ = 10
    CORR = 11
    ACK = 12
    NACK = 13
    STATUS = 15
    CMD = 20


class CommandID(IntEnum):
    """Command IDs for CMD messages."""
    STOP_ROS2 = 0
    # STM32 runtime semantics:
    #   1 = traj 1 (autonomous localization on saved map)
    #   2 = traj 0 (standby/manual)
    #   3 = traj 2 (manual + localization)
    #   9 = traj 3 (autonomous with blank global map + local costmap avoidance)
    # Historically, id=1 was named START_ROS2. Keep alias for backward
    # compatibility with legacy tooling.
    START_ROS2 = 1
    START_TRAJ = 1
    STOP_TRAJ = 2
    START_RESTART_ROS2 = 3
    SHUTDOWN_PI5 = 4
    START_MAPPING = 5
    FINISH_MAPPING = 6
    USE_LIVE_MAP = 7
    USE_FROZEN_MAP = 8
    START_TRAJ_LOCAL = 9
    START_TERMINAL_PASSTHROUGH = 10
    TERMINAL_PASSTHROUGH_DATA = 11
    STOP_TERMINAL_PASSTHROUGH = 12
    WP_TEST_PATTERN = 13
    TOGGLE_JOYSTICK = 14


@dataclass
class Header:
    """Message header."""
    magic: int
    version: int
    msg_type: int
    seq: int
    t_ms: int
    payload_len: int
    crc32: int

    def pack(self) -> bytes:
        """Pack header to little-endian bytes."""
        return struct.pack(
            "<IHHIIII",
            self.magic,
            self.version,
            self.msg_type,
            self.seq,
            self.t_ms,
            self.payload_len,
            self.crc32,
        )

    @staticmethod
    def unpack(data: bytes) -> "Header":
        """Unpack header from little-endian bytes."""
        if len(data) < HEADER_SIZE:
            raise ValueError(f"Header too short: {len(data)} < {HEADER_SIZE}")
        magic, version, msg_type, seq, t_ms, payload_len, crc32 = struct.unpack(
            "<IHHIIII", data[:HEADER_SIZE]
        )
        return Header(magic, version, msg_type, seq, t_ms, payload_len, crc32)


@dataclass
class Pose:
    """POSE message payload."""
    pose_t_ms: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float

    def pack(self) -> bytes:
        return struct.pack("<Iffffff", self.pose_t_ms, self.x, self.y, self.yaw, self.vx, self.vy, self.wz)

    @staticmethod
    def unpack(data: bytes) -> "Pose":
        if len(data) < 28:  # 4 + 6*4
            raise ValueError(f"Pose payload too short: {len(data)}")
        pose_t_ms, x, y, yaw, vx, vy, wz = struct.unpack("<Iffffff", data[:28])
        return Pose(pose_t_ms, x, y, yaw, vx, vy, wz)


@dataclass
class Command:
    """CMD message payload."""
    cmd_id: int
    arg: bytes

    def pack(self) -> bytes:
        arg_len = len(self.arg)
        return struct.pack("<HH", self.cmd_id, arg_len) + self.arg

    @staticmethod
    def unpack(data: bytes) -> "Command":
        if len(data) < 4:
            raise ValueError(f"Command payload too short: {len(data)}")
        
        # Support old STM32 protocol: if payload is exactly 4 bytes, treat as uint32 command
        if len(data) == 4:
            cmd_id = struct.unpack("<I", data[:4])[0]
            return Command(cmd_id, b"")
        
        # New protocol: cmd_id (uint16) + arg_len (uint16) + arg
        cmd_id, arg_len = struct.unpack("<HH", data[:4])
        if len(data) < 4 + arg_len:
            raise ValueError(f"Command arg too short: expected {arg_len}, got {len(data) - 4}")
        arg = data[4 : 4 + arg_len]
        return Command(cmd_id, arg)


@dataclass
class Trajectory:
    """TRAJ message payload."""
    reply_to_pose_seq: int
    traj_t0_ms: int
    dt: float
    knots: list  # list of (x, y, yaw, vx, vy)
    flags: int = 0  # bit0=idle_traj, bit1=has_vel

    def pack(self) -> bytes:
        n = len(self.knots)
        payload = struct.pack("<IIHHI", self.reply_to_pose_seq, self.traj_t0_ms, n, self.flags, 0)
        payload += struct.pack("<f", self.dt)
        for x, y, yaw, vx, vy in self.knots:
            payload += struct.pack("<fffff", x, y, yaw, vx, vy)
        return payload

    @staticmethod
    def unpack(data: bytes) -> "Trajectory":
        if len(data) < 14:  # reply_to_pose_seq(4) + traj_t0_ms(4) + n(2) + flags(2) + dt(4)
            raise ValueError(f"Trajectory payload too short: {len(data)}")
        reply_to_pose_seq, traj_t0_ms, n, flags = struct.unpack("<IIHH", data[:12])
        dt = struct.unpack("<f", data[12:16])[0]
        knots = []
        offset = 16
        for _ in range(n):
            if offset + 20 > len(data):
                raise ValueError("Trajectory knot data incomplete")
            x, y, yaw, vx, vy = struct.unpack("<fffff", data[offset : offset + 20])
            knots.append((x, y, yaw, vx, vy))
            offset += 20
        return Trajectory(reply_to_pose_seq, traj_t0_ms, dt, knots, flags)


@dataclass
class Status:
    """STATUS message payload."""
    status_t_ms: int
    idle: bool
    ros2_running: bool
    last_pose_seq: int
    last_traj_seq: int
    last_pose_latency_ms: float

    def pack(self) -> bytes:
        return struct.pack(
            "<IBBHIIf",
            self.status_t_ms,
            1 if self.idle else 0,
            1 if self.ros2_running else 0,
            0,  # reserved
            self.last_pose_seq,
            self.last_traj_seq,
            self.last_pose_latency_ms,
        )

    @staticmethod
    def unpack(data: bytes) -> "Status":
        if len(data) < 19:  # 4 + 1 + 1 + 2 + 4 + 4 + 4
            raise ValueError(f"Status payload too short: {len(data)}")
        status_t_ms, idle, ros2_running, reserved, last_pose_seq, last_traj_seq, last_pose_latency_ms = struct.unpack(
            "<IBBHIIf", data[:19]
        )
        return Status(status_t_ms, bool(idle), bool(ros2_running), last_pose_seq, last_traj_seq, last_pose_latency_ms)


def compute_crc32(data: bytes) -> int:
    """Compute CRC32 of payload using zlib."""
    return zlib.crc32(data) & 0xFFFFFFFF


def validate_crc32(data: bytes, crc_expected: int) -> bool:
    """Validate CRC32 of payload."""
    if crc_expected == 0:
        return True  # Skip validation if crc32 is 0
    crc_computed = compute_crc32(data)
    return crc_computed == crc_expected


class StreamParser:
    """
    Robust parser for binary framed messages.
    Handles partial reads, resynchronization on bad magic, and validation.
    """

    def __init__(self):
        self.buffer = b""
        self.last_seq = -1

    def feed(self, data: bytes) -> None:
        """Add data to buffer."""
        self.buffer += data

    def parse_message(self) -> Optional[Tuple[Header, bytes]]:
        """
        Try to parse one complete message from buffer.
        Returns (header, payload) on success, None if incomplete or invalid.
        On success, consumes message from buffer.
        On parse error, attempts resync to next magic number.
        """
        while len(self.buffer) >= HEADER_SIZE:
            # Check for magic at current position
            potential_magic = struct.unpack("<I", self.buffer[:4])[0]
            if potential_magic != MAGIC:
                # Resync: search for next magic number
                pos = self._find_magic(4)
                if pos == -1:
                    # No magic found; keep only last 3 bytes in case magic spans chunks
                    self.buffer = self.buffer[-(HEADER_SIZE - 1) :] if len(self.buffer) >= HEADER_SIZE else self.buffer
                    return None
                # Found magic at pos; discard everything before it
                self.buffer = self.buffer[pos:]
                continue

            # Try to parse header
            try:
                header = Header.unpack(self.buffer[:HEADER_SIZE])
            except ValueError:
                # Bad header; skip one byte and resync
                self.buffer = self.buffer[1:]
                continue

            # Validate header
            if header.version != VERSION:
                self.buffer = self.buffer[1:]
                continue
            if header.payload_len > MAX_PAYLOAD_SIZE:
                self.buffer = self.buffer[1:]
                continue

            # Check if we have the full payload
            total_size = HEADER_SIZE + header.payload_len
            if len(self.buffer) < total_size:
                return None  # Wait for more data

            # Extract payload
            payload = self.buffer[HEADER_SIZE : total_size]

            # Validate CRC if present
            if header.crc32 != 0:
                if not validate_crc32(payload, header.crc32):
                    self.buffer = self.buffer[1:]
                    continue

            # Success: consume message from buffer and return
            self.buffer = self.buffer[total_size:]
            return (header, payload)

        return None

    def _find_magic(self, start: int) -> int:
        """Search for MAGIC in buffer starting at offset start. Returns -1 if not found."""
        magic_bytes = struct.pack("<I", MAGIC)
        pos = self.buffer.find(magic_bytes, start)
        return pos

    def clear(self) -> None:
        """Clear buffer (use on disconnect/resync)."""
        self.buffer = b""


def make_message(msg_type: int, seq: int, payload: bytes, crc_payload: bool = False) -> bytes:
    """
    Create a complete framed message.

    Args:
        msg_type: MessageType enum value
        seq: sequence number
        payload: payload bytes
        crc_payload: if True, compute and include CRC32; else set crc32=0

    Returns:
        Full framed message (header + payload)
    """
    import time

    t_ms = int(time.time() * 1000) % (2**32)
    payload_len = len(payload)

    crc32_val = 0
    if crc_payload:
        crc32_val = compute_crc32(payload)

    header = Header(
        magic=MAGIC,
        version=VERSION,
        msg_type=msg_type,
        seq=seq,
        t_ms=t_ms,
        payload_len=payload_len,
        crc32=crc32_val,
    )

    return header.pack() + payload

import logging
import socket
import struct
import time


MAGIC = 0x4F4D4E49
VERSION = 1
MSG_TYPE_CMD = 20

CMD_JOY_MODE_ON = 100
CMD_JOY_MODE_OFF = 101
CMD_JOY_VECTOR = 102


class EthernetUdpLink:
    def __init__(self, target_ip: str, target_port: int, logger: logging.Logger) -> None:
        self._target = (target_ip, target_port)
        self._logger = logger
        self._sock: socket.socket | None = None
        self._seq = 1
        self._connected = False
        self._last_error_log_monotonic = 0.0
        self._last_tx_log_label = ""
        self._last_tx_log_monotonic = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        if self._sock is not None:
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._connected = True
        self._logger.info("Ethernet UDP link ready -> %s:%d", self._target[0], self._target[1])

    def stop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._connected = False

    def _build_cmd_packet(self, cmd_id: int, arg: bytes) -> bytes:
        payload = struct.pack("<HH", cmd_id, len(arg)) + arg

        header = struct.pack(
            "<IHHIIII",
            MAGIC,
            VERSION,
            MSG_TYPE_CMD,
            self._seq,
            int(time.time() * 1000) & 0xFFFFFFFF,
            len(payload),
            0,
        )
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        if self._seq == 0:
            self._seq = 1

        return header + payload

    def _send(self, cmd_id: int, arg: bytes, log_label: str) -> bool:
        if self._sock is None:
            return False

        packet = self._build_cmd_packet(cmd_id, arg)
        try:
            self._sock.sendto(packet, self._target)
            self._connected = True
            now = time.monotonic()
            if log_label != self._last_tx_log_label or (now - self._last_tx_log_monotonic) > 1.0:
                self._logger.info("TX ETH -> STM32: %s", log_label)
                self._last_tx_log_label = log_label
                self._last_tx_log_monotonic = now
            return True
        except OSError as exc:
            self._connected = False
            now = time.monotonic()
            if now - self._last_error_log_monotonic > 2.0:
                self._logger.warning("Ethernet send failed: %s", exc)
                self._last_error_log_monotonic = now
            return False

    def send_joy_enable(self, enable: bool) -> bool:
        cmd_id = CMD_JOY_MODE_ON if enable else CMD_JOY_MODE_OFF
        return self._send(cmd_id, b"", "joy on" if enable else "joy off")

    def send_vector(self, angle: int, speed: int) -> bool:
        angle_u16 = int(angle) & 0xFFFF
        speed_u8 = max(0, min(100, int(speed)))
        arg = struct.pack("<HB", angle_u16, speed_u8)
        return self._send(CMD_JOY_VECTOR, arg, f"vector a={angle_u16} s={speed_u8}")
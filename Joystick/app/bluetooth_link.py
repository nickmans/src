import logging
import threading
import time

import serial


class BluetoothSerialLink:
    def __init__(
        self,
        device: str,
        baudrate: int,
        reconnect_interval_s: float,
        logger: logging.Logger,
    ) -> None:
        self._device = device
        self._baudrate = baudrate
        self._reconnect_interval_s = reconnect_interval_s
        self._logger = logger

        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._last_tx_log_line = ""
        self._last_tx_log_monotonic = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._close_serial()

    def _worker(self) -> None:
        while not self._stop_event.is_set():
            if not self._connected:
                self._attempt_connect()
                if not self._connected:
                    time.sleep(self._reconnect_interval_s)
                    continue

            with self._serial_lock:
                serial_obj = self._serial

            if serial_obj is None or not serial_obj.is_open:
                self._logger.warning("Bluetooth serial became unavailable; reconnecting")
                self._close_serial()
            time.sleep(0.2)

    def _attempt_connect(self) -> None:
        try:
            self._logger.info(
                "Connecting Bluetooth serial on %s at %d baud",
                self._device,
                self._baudrate,
            )
            serial_obj = serial.Serial(
                port=self._device,
                baudrate=self._baudrate,
                timeout=0.2,
                write_timeout=0.2,
            )
            with self._serial_lock:
                self._serial = serial_obj
            self._connected = True
            self._logger.info("Bluetooth serial connected")
        except (serial.SerialException, OSError) as exc:
            self._connected = False
            self._logger.warning("Bluetooth connect failed: %s", exc)

    def _close_serial(self) -> None:
        with self._serial_lock:
            serial_obj = self._serial
            self._serial = None

        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:
                pass

        if self._connected:
            self._logger.warning("Bluetooth serial disconnected")
        self._connected = False

    def send_line(self, line: str) -> bool:
        payload = line if line.endswith("\n") else f"{line}\n"

        with self._serial_lock:
            serial_obj = self._serial

        if serial_obj is None or not self._connected:
            return False

        try:
            serial_obj.write(payload.encode("ascii", errors="ignore"))
            serial_obj.flush()
            now = time.monotonic()
            line = payload.strip()
            if line != self._last_tx_log_line or (now - self._last_tx_log_monotonic) > 1.0:
                self._logger.info("TX -> STM32: %s", line)
                self._last_tx_log_line = line
                self._last_tx_log_monotonic = now
            return True
        except (serial.SerialException, OSError) as exc:
            self._logger.warning("Bluetooth send failed: %s", exc)
            self._close_serial()
            return False
import asyncio
import json
import logging
import time
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .bluetooth_link import BluetoothSerialLink
from .ethernet_link import EthernetUdpLink
from .command_mapper import JoystickCommand, format_command_lines, map_xy_to_command
from .config import get_config


class JoystickState:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.last_input_monotonic = time.monotonic()
        self.clients = 0
        self.joy_pending = False
        self.joy_enabled = False

    async def set_input(self, x: float, y: float) -> None:
        async with self._lock:
            self.raw_x = x
            self.raw_y = y
            self.last_input_monotonic = time.monotonic()

    async def trigger_joy_mode(self) -> None:
        async with self._lock:
            self.joy_pending = True
            self.joy_enabled = True

    async def add_client(self) -> int:
        async with self._lock:
            self.clients += 1
            return self.clients

    async def remove_client(self) -> int:
        async with self._lock:
            self.clients = max(0, self.clients - 1)
            if self.clients == 0:
                self.raw_x = 0.0
                self.raw_y = 0.0
                self.last_input_monotonic = time.monotonic()
                self.joy_pending = False
                self.joy_enabled = False
            return self.clients

    async def snapshot(self) -> tuple[float, float, float, int, bool, bool]:
        async with self._lock:
            age = time.monotonic() - self.last_input_monotonic
            return (
                self.raw_x,
                self.raw_y,
                age,
                self.clients,
                self.joy_pending,
                self.joy_enabled,
            )

    async def clear_joy_pending(self) -> None:
        async with self._lock:
            self.joy_pending = False

    async def force_zero(self) -> None:
        async with self._lock:
            self.raw_x = 0.0
            self.raw_y = 0.0
            self.last_input_monotonic = time.monotonic()


class WebSocketHub:
    def __init__(self, logger: logging.Logger) -> None:
        self._lock = asyncio.Lock()
        self._clients: set[WebSocket] = set()
        self._logger = logger

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._clients.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(websocket)

    async def broadcast(self, payload: dict) -> None:
        text = json.dumps(payload)
        async with self._lock:
            clients = list(self._clients)

        if not clients:
            return

        stale_clients: list[WebSocket] = []
        for ws in clients:
            try:
                await ws.send_text(text)
            except Exception:
                stale_clients.append(ws)

        if stale_clients:
            async with self._lock:
                for ws in stale_clients:
                    self._clients.discard(ws)
            self._logger.warning("Removed %d stale websocket clients", len(stale_clients))


def setup_logging(level: str) -> logging.Logger:
    logging.basicConfig(
        level=getattr(logging, level, logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    return logging.getLogger("robot_joystick")


cfg = get_config()
logger = setup_logging(cfg.log_level)
state = JoystickState()
hub = WebSocketHub(logger)
bt_link: BluetoothSerialLink | None = None
eth_link: EthernetUdpLink | None = None

if cfg.transport == "bluetooth":
    bt_link = BluetoothSerialLink(
        device=cfg.bt_device,
        baudrate=cfg.bt_baudrate,
        reconnect_interval_s=cfg.bt_reconnect_interval_s,
        logger=logger,
    )
else:
    eth_link = EthernetUdpLink(
        target_ip=cfg.eth_target_ip,
        target_port=cfg.eth_target_port,
        logger=logger,
    )


def link_connected() -> bool:
    if bt_link is not None:
        return bt_link.connected
    if eth_link is not None:
        return eth_link.connected
    return False


def link_start() -> None:
    if bt_link is not None:
        bt_link.start()
    if eth_link is not None:
        eth_link.start()


def link_stop() -> None:
    if bt_link is not None:
        bt_link.stop()
    if eth_link is not None:
        eth_link.stop()


def send_joy_enable() -> bool:
    if bt_link is not None:
        return bt_link.send_line(cfg.cmd_joy)
    if eth_link is not None:
        return eth_link.send_joy_enable(True)
    return False


def send_command_to_stm32(cmd: JoystickCommand) -> None:
    if bt_link is not None:
        for line in format_command_lines(cmd, cfg):
            bt_link.send_line(line)
        return

    if eth_link is not None:
        eth_link.send_vector(cmd.angle, cmd.speed)


def send_zero_immediately(reason: str) -> None:
    logger.warning("Safety zero output triggered: %s", reason)
    send_command_to_stm32(JoystickCommand(angle=0, speed=0))


async def command_stream_loop() -> None:
    interval_s = 1.0 / cfg.update_hz
    timeout_active = False

    while True:
        raw_x, raw_y, age, client_count, joy_pending, joy_enabled = await state.snapshot()

        if joy_pending:
            if send_joy_enable():
                logger.info("Joystick mode command sent: %s", cfg.cmd_joy)
                await state.clear_joy_pending()

        if age > cfg.input_timeout_s:
            raw_x, raw_y = 0.0, 0.0
            if not timeout_active:
                timeout_active = True
                logger.warning(
                    "Input timeout exceeded (%.3fs > %.3fs), forcing zero",
                    age,
                    cfg.input_timeout_s,
                )
        else:
            timeout_active = False

        if not joy_enabled:
            raw_x, raw_y = 0.0, 0.0

        cmd = map_xy_to_command(raw_x, raw_y, cfg)
        send_command_to_stm32(cmd)

        await hub.broadcast(
            {
                "type": "telemetry",
                "angle": cmd.angle,
                "speed": cmd.speed,
                "ws_clients": client_count,
                "transport": cfg.transport,
                "link_connected": link_connected(),
                "joy_enabled": joy_enabled,
            }
        )
        await asyncio.sleep(interval_s)


@asynccontextmanager
async def lifespan(_: FastAPI):
    logger.info("Starting robot joystick backend")
    link_start()
    stream_task = asyncio.create_task(command_stream_loop())
    try:
        yield
    finally:
        stream_task.cancel()
        await asyncio.gather(stream_task, return_exceptions=True)
        link_stop()
        logger.info("Robot joystick backend stopped")


app = FastAPI(title="Robot Joystick Bridge", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=str(cfg.static_dir)), name="static")


@app.get("/")
async def root() -> FileResponse:
    return FileResponse(cfg.static_dir / "index.html")


@app.get("/health")
async def health() -> dict:
    _, _, age, clients, _, joy_enabled = await state.snapshot()
    return {
        "ok": True,
        "ws_clients": clients,
        "transport": cfg.transport,
        "link_connected": link_connected(),
        "last_input_age_s": round(age, 3),
        "joy_enabled": joy_enabled,
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await hub.connect(websocket)
    clients = await state.add_client()
    logger.info("Client connected (active clients=%d)", clients)

    try:
        while True:
            raw_msg = await websocket.receive_text()
            try:
                msg = json.loads(raw_msg)
            except json.JSONDecodeError:
                logger.warning("Invalid websocket JSON payload ignored")
                continue

            msg_type = msg.get("type")

            if msg_type == "joystick":
                try:
                    x = float(msg.get("x", 0.0))
                    y = float(msg.get("y", 0.0))
                except (TypeError, ValueError):
                    logger.warning("Invalid joystick payload ignored")
                    continue
                await state.set_input(x, y)
            elif msg_type == "enable_joy":
                await state.trigger_joy_mode()
                logger.info("Joystick mode requested by client")
            elif msg_type == "estop":
                await state.force_zero()
                send_zero_immediately("E-stop from UI")
            elif msg_type == "ping":
                await websocket.send_text(json.dumps({"type": "pong"}))
            else:
                logger.warning("Unknown websocket message type: %s", msg_type)

    except WebSocketDisconnect:
        logger.warning("Websocket disconnected")
    finally:
        await hub.disconnect(websocket)
        clients = await state.remove_client()
        logger.info("Client disconnected (active clients=%d)", clients)
        if clients == 0:
            send_zero_immediately("Last websocket client disconnected")


def run() -> None:
    uvicorn.run(
        "app.main:app",
        host=cfg.host,
        port=cfg.port,
        reload=False,
        log_level=cfg.log_level.lower(),
    )


if __name__ == "__main__":
    run()
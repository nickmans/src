import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from dotenv import load_dotenv


def _to_bool(value: str, default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _to_int(value: str, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(value: str, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    log_level: str
    transport: str

    update_hz: float
    deadzone: float
    max_speed: int
    input_timeout_s: float
    input_hold_grace_s: float

    bt_device: str
    bt_baudrate: int
    bt_reconnect_interval_s: float

    cmd_joy: str
    cmd_focus: str
    cmd_rotate_template: str
    cmd_dir_template: str
    cmd_speed_template: str
    cmd_use_single_line: bool
    cmd_single_line_template: str

    eth_target_ip: str
    eth_target_port: int

    project_root: Path
    static_dir: Path


@lru_cache(maxsize=1)
def get_config() -> AppConfig:
    project_root = Path(__file__).resolve().parent.parent
    env_file = project_root / ".env"
    load_dotenv(dotenv_path=env_file, override=False)

    host = os.getenv("HOST", "0.0.0.0")
    port = _to_int(os.getenv("PORT"), 8000)
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()
    transport = os.getenv("TRANSPORT", "ethernet").strip().lower()
    if transport not in {"bluetooth", "ethernet"}:
        transport = "ethernet"

    update_hz = max(1.0, _to_float(os.getenv("UPDATE_HZ"), 20.0))
    deadzone = min(0.95, max(0.0, _to_float(os.getenv("DEADZONE"), 0.08)))
    max_speed = max(1, _to_int(os.getenv("MAX_SPEED"), 100))
    input_timeout_s = max(0.05, _to_float(os.getenv("INPUT_TIMEOUT_S"), 0.35))
    input_hold_grace_s = max(0.0, _to_float(os.getenv("INPUT_HOLD_GRACE_S"), 0.9))

    bt_device = os.getenv("BT_DEVICE", "/dev/rfcomm0")
    bt_baudrate = max(1200, _to_int(os.getenv("BT_BAUDRATE"), 115200))
    bt_reconnect_interval_s = max(
        0.2, _to_float(os.getenv("BT_RECONNECT_INTERVAL_S"), 2.0)
    )

    cmd_joy = os.getenv("CMD_JOY", "joy")
    cmd_focus = os.getenv("CMD_FOCUS", "focus")
    cmd_rotate_template = os.getenv("CMD_ROTATE_TEMPLATE", "w {value}")
    cmd_dir_template = os.getenv("CMD_DIR_TEMPLATE", "dir {angle}")
    cmd_speed_template = os.getenv("CMD_SPEED_TEMPLATE", "speed {speed}")
    cmd_use_single_line = _to_bool(os.getenv("CMD_USE_SINGLE_LINE"), True)
    cmd_single_line_template = os.getenv(
        "CMD_SINGLE_LINE_TEMPLATE", "dir {angle} speed {speed}"
    )

    eth_target_ip = os.getenv("ETH_TARGET_IP", "192.168.1.10")
    eth_target_port = max(1, min(65535, _to_int(os.getenv("ETH_TARGET_PORT"), 9001)))

    static_dir = project_root / "static"

    return AppConfig(
        host=host,
        port=port,
        log_level=log_level,
        transport=transport,
        update_hz=update_hz,
        deadzone=deadzone,
        max_speed=max_speed,
        input_timeout_s=input_timeout_s,
        input_hold_grace_s=input_hold_grace_s,
        bt_device=bt_device,
        bt_baudrate=bt_baudrate,
        bt_reconnect_interval_s=bt_reconnect_interval_s,
        cmd_joy=cmd_joy,
        cmd_focus=cmd_focus,
        cmd_rotate_template=cmd_rotate_template,
        cmd_dir_template=cmd_dir_template,
        cmd_speed_template=cmd_speed_template,
        cmd_use_single_line=cmd_use_single_line,
        cmd_single_line_template=cmd_single_line_template,
        eth_target_ip=eth_target_ip,
        eth_target_port=eth_target_port,
        project_root=project_root,
        static_dir=static_dir,
    )
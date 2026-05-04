import math
from dataclasses import dataclass

from .config import AppConfig


@dataclass(frozen=True)
class JoystickCommand:
    angle: int
    speed: int


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def sanitize_xy(raw_x: float, raw_y: float) -> tuple[float, float]:
    x = _clamp(raw_x, -1.0, 1.0)
    y = _clamp(raw_y, -1.0, 1.0)

    magnitude = math.hypot(x, y)
    if magnitude > 1.0 and magnitude > 0.0:
        x /= magnitude
        y /= magnitude

    return x, y


def map_xy_to_command(raw_x: float, raw_y: float, cfg: AppConfig) -> JoystickCommand:
    x, y = sanitize_xy(raw_x, raw_y)
    magnitude = math.hypot(x, y)

    if magnitude <= cfg.deadzone:
        return JoystickCommand(angle=0, speed=0)

    scaled = (magnitude - cfg.deadzone) / (1.0 - cfg.deadzone)
    scaled = _clamp(scaled, 0.0, 1.0)

    angle = int(round(math.degrees(math.atan2(x, y)))) % 360
    speed = int(round(scaled * cfg.max_speed))

    return JoystickCommand(angle=angle, speed=speed)


def format_command_lines(cmd: JoystickCommand, cfg: AppConfig) -> list[str]:
    if cfg.cmd_use_single_line:
        line = cfg.cmd_single_line_template.format(angle=cmd.angle, speed=cmd.speed)
        return [line]

    dir_line = cfg.cmd_dir_template.format(angle=cmd.angle, speed=cmd.speed)
    speed_line = cfg.cmd_speed_template.format(angle=cmd.angle, speed=cmd.speed)
    return [dir_line, speed_line]
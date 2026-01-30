"""
Trajectory planner stub.

Generates trajectory knots from pose, idle state, and parameters.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class PoseState:
    """Current robot pose and velocity."""
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    t_ms: int


def wrap_yaw(yaw: float) -> float:
    """Wrap yaw to [-pi, pi]."""
    while yaw > math.pi:
        yaw -= 2 * math.pi
    while yaw < -math.pi:
        yaw += 2 * math.pi
    return yaw


def make_traj_from_pose(
    pose: PoseState,
    idle: bool,
    dt: float = 0.05,
    horizon: float = 1.2,
) -> List[Tuple[float, float, float, float]]:
    """
    Generate trajectory knots from pose.

    If idle: hold position (velocity = 0)
    Else: constant-velocity rollout using pose velocities

    Args:
        pose: Current pose state
        idle: If True, generate hold-position traj
        dt: Time step between knots (seconds)
        horizon: Total trajectory duration (seconds)

    Returns:
        List of knots: [(x, y, yaw, velocity), ...]
    """
    n_knots = max(2, int(round(horizon / dt)))

    knots: List[Tuple[float, float, float, float]] = []

    if idle:
        # Hold position: constant pose, zero velocity
        for _ in range(n_knots):
            knots.append((pose.x, pose.y, pose.yaw, 0.0))
    else:
        # Constant-velocity rollout
        for i in range(n_knots):
            t = i * dt
            x = pose.x + pose.vx * t
            y = pose.y + pose.vy * t
            yaw = pose.yaw + pose.wz * t
            yaw = wrap_yaw(yaw)
            # Compute linear velocity magnitude from (vx, vy)
            velocity = math.sqrt(pose.vx * pose.vx + pose.vy * pose.vy)
            knots.append((x, y, yaw, velocity))

    return knots

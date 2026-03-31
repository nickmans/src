#!/usr/bin/env python3
"""
Virtual STM32 for OMNI UDP stack.

This process emulates the CM7 control/estimation loop so ROS2 can be run
without physical STM32 hardware:

- Receives TRAJ packets from `udp_server.py`
- Runs CM7-like controller + wheel/encoder simulation at 100 Hz
- Runs CM7-like state estimator (yaw from simulated IMU)
- Sends POSE packets back to the UDP server at 5 Hz
- Sends CMD packets (START_RESTART_ROS2 and START_TRAJ) on startup
"""

import argparse
import logging
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from protocol import Command, CommandID, MessageType, Pose, StreamParser, make_message

logger = logging.getLogger(__name__)


def wrap_pi(angle: float) -> float:
    two_pi = 2.0 * math.pi
    while angle > math.pi:
        angle -= two_pi
    while angle <= -math.pi:
        angle += two_pi
    return angle


def clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


@dataclass
class TrajectoryFrame:
    seq: int
    traj_t0_ms: int
    dt: float
    knots: List[Tuple[float, float, float, float, float]]


class CM7Controller:
    """Python port of CM7 `Controller_Step` behavior."""

    def __init__(self, rw: float = 0.09, wheel_base_l: float = 0.2) -> None:
        self.rw = rw
        self.wheel_base_l = wheel_base_l

        self.k_p = 0.5
        self.umax = 11.0
        self.aumax = 1.0 / self.rw
        self.jerkmax = 200.0
        self.wmax = 2.0
        self.dwmax = 1.0

        self.du_prev = [0.0, 0.0, 0.0]
        self.u_prev = [0.0, 0.0, 0.0]
        self.u_trans_prev = [0.0, 0.0, 0.0]
        self.yawrate_prev = 0.0

    def _inverse_kinematics(self, vx_body: float, vy_body: float, omega: float) -> List[float]:
        r_inv = 1.0 / self.rw
        s3_2 = 0.86602540378443864676
        l_val = self.wheel_base_l
        return [
            r_inv * (vy_body + l_val * omega),
            r_inv * (-0.5 * vy_body + s3_2 * vx_body + l_val * omega),
            r_inv * (-0.5 * vy_body - s3_2 * vx_body + l_val * omega),
        ]

    @staticmethod
    def _norm2(vx: float, vy: float) -> float:
        return math.sqrt(vx * vx + vy * vy)

    @staticmethod
    def _maxabs3(values: List[float]) -> float:
        return max(abs(values[0]), abs(values[1]), abs(values[2]))

    def step(self, x: List[float], xd: List[float], vd: List[float], selector: int, dt: float) -> List[float]:
        pos_x, pos_y, yaw = x[0], x[1], x[2]
        pos_dx, pos_dy, yaw_d, vx_world, vy_world = xd

        if selector:
            c = math.cos(yaw)
            s = math.sin(yaw)

            v_ff_body_x = c * vx_world + s * vy_world
            v_ff_body_y = -s * vx_world + c * vy_world

            d_world_x = pos_dx - pos_x
            d_world_y = pos_dy - pos_y
            d_body_x = c * d_world_x + s * d_world_y
            d_body_y = -s * d_world_x + c * d_world_y

            v_corr_body_x = self.k_p * d_body_x
            v_corr_body_y = self.k_p * d_body_y

            v_corr_mag = self._norm2(v_corr_body_x, v_corr_body_y)
            if v_corr_mag > 0.1:
                scale = 0.1 / v_corr_mag
                v_corr_body_x *= scale
                v_corr_body_y *= scale

            vx_body = v_ff_body_x + v_corr_body_x
            vy_body = v_ff_body_y + v_corr_body_y

            e = wrap_pi(yaw_d - yaw)
            yaw_dead = math.radians(1.0)
            yaw_lin = math.radians(6.0)
            ae = abs(e)
            if ae < yaw_dead:
                omega = 0.0
            elif ae < yaw_lin:
                omega = (self.wmax / yaw_lin) * e
            else:
                omega = self.wmax * (1.0 if e > 0.0 else -1.0)
            omega = clamp(omega, -self.wmax, self.wmax)
        else:
            c = math.cos(yaw)
            s = math.sin(yaw)
            vx_body = c * vd[0] + s * vd[1]
            vy_body = -s * vd[0] + c * vd[1]
            omega = clamp(vd[2], -self.wmax, self.wmax)

        domega = omega - self.yawrate_prev
        domega_max = self.dwmax * dt
        domega = clamp(domega, -domega_max, domega_max)
        omega = self.yawrate_prev + domega
        self.yawrate_prev = omega

        u_rot = self._inverse_kinematics(0.0, 0.0, omega)
        u_trans = self._inverse_kinematics(vx_body, vy_body, 0.0)

        du_trans_max = self.aumax * dt
        for idx in range(3):
            du_i = u_trans[idx] - self.u_trans_prev[idx]
            du_i_limited = clamp(du_i, -du_trans_max, du_trans_max)
            u_trans[idx] = self.u_trans_prev[idx] + du_i_limited

        s_lo = 0.0
        s_hi = 1.0
        for idx in range(3):
            a = u_trans[idx]
            b = u_rot[idx]
            if abs(a) < 1e-12:
                if abs(b) > self.umax:
                    s_hi = -1.0
                continue
            s1 = (-self.umax - b) / a
            s2 = (self.umax - b) / a
            smin = min(s1, s2)
            smax = max(s1, s2)
            s_lo = max(s_lo, smin)
            s_hi = min(s_hi, smax)

        s_scale = 0.0 if s_hi < s_lo else clamp(s_hi, 0.0, 1.0)

        u_des = [
            u_rot[0] + s_scale * u_trans[0],
            u_rot[1] + s_scale * u_trans[1],
            u_rot[2] + s_scale * u_trans[2],
        ]

        du_des = [
            u_des[0] - self.u_prev[0],
            u_des[1] - self.u_prev[1],
            u_des[2] - self.u_prev[2],
        ]

        du_max = self.aumax * dt
        du_inf = self._maxabs3(du_des)
        if du_inf > du_max and du_inf > 0.0:
            scale = du_max / du_inf
            du_acc = [du_des[0] * scale, du_des[1] * scale, du_des[2] * scale]
        else:
            du_acc = du_des

        ddu = [
            du_acc[0] - self.du_prev[0],
            du_acc[1] - self.du_prev[1],
            du_acc[2] - self.du_prev[2],
        ]
        ddu_max = self.jerkmax * dt * dt
        ddu_inf = self._maxabs3(ddu)
        if ddu_inf > ddu_max and ddu_inf > 0.0:
            scale = ddu_max / ddu_inf
            ddu = [ddu[0] * scale, ddu[1] * scale, ddu[2] * scale]

        du = [
            self.du_prev[0] + ddu[0],
            self.du_prev[1] + ddu[1],
            self.du_prev[2] + ddu[2],
        ]
        u_cmd = [
            self.u_prev[0] + du[0],
            self.u_prev[1] + du[1],
            self.u_prev[2] + du[2],
        ]

        self.du_prev = du
        self.u_prev = u_cmd
        self.u_trans_prev = [u_cmd[0] - u_rot[0], u_cmd[1] - u_rot[1], u_cmd[2] - u_rot[2]]
        return u_cmd


class CM7StateEstimator:
    """Python port of CM7 `StateEstimator_Update` behavior."""

    def __init__(self, rw: float = 0.09) -> None:
        self.rw = rw
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.imu_yaw_zero = 0.0
        self.inited = False

    def reset(self, x0: float = 0.0, y0: float = 0.0, yaw0: float = 0.0) -> None:
        self.x = x0
        self.y = y0
        self.yaw = wrap_pi(yaw0)
        self.inited = True

    def zero_imu_yaw(self, imu_yaw_rad: float) -> None:
        self.imu_yaw_zero = imu_yaw_rad

    def update(self, wheel_rad_s: List[float], dt: float, imu_yaw_rad: float) -> List[float]:
        if not self.inited:
            self.reset(0.0, 0.0, 0.0)
        if dt <= 0.0:
            return [self.x, self.y, self.yaw]

        w1, w2, w3 = wheel_rad_s
        k_s3 = 0.57735026918962576451
        vx_body = self.rw * (k_s3 * (w2 - w3))
        vy_body = self.rw * ((2.0 / 3.0) * w1 - (1.0 / 3.0) * (w2 + w3))

        yaw = wrap_pi(imu_yaw_rad - self.imu_yaw_zero)
        self.yaw = yaw

        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_world = vx_body * c - vy_body * s
        vy_world = vx_body * s + vy_body * c

        self.x += dt * vx_world
        self.y += dt * vy_world
        return [self.x, self.y, self.yaw]


class WheelPlantAndEncoder:
    """Simple wheel + encoder plant to feed controller output back into estimator."""

    def __init__(self, rw: float, wheel_base_l: float, cpr: float, motor_tau: float) -> None:
        self.rw = rw
        self.wheel_base_l = wheel_base_l
        self.cpr = cpr
        self.motor_tau = motor_tau

        self.w_actual = [0.0, 0.0, 0.0]
        self.count_residual = [0.0, 0.0, 0.0]

        self.t_since_edge_s = [0.0, 0.0, 0.0]
        self.rpm_period = [0.0, 0.0, 0.0]
        self.rpm_filt = [0.0, 0.0, 0.0]

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_yaw = 0.0

    def update(self, wheel_cmd_rad_s: List[float], dt: float) -> Tuple[List[float], float, float, float, float, float]:
        alpha_motor = 1.0 - math.exp(-dt / max(1e-4, self.motor_tau))
        for idx in range(3):
            self.w_actual[idx] += alpha_motor * (wheel_cmd_rad_s[idx] - self.w_actual[idx])

        w1, w2, w3 = self.w_actual
        k_s3 = 0.57735026918962576451
        vx_body = self.rw * (k_s3 * (w2 - w3))
        vy_body = self.rw * ((2.0 / 3.0) * w1 - (1.0 / 3.0) * (w2 + w3))
        wz = self.rw * (w1 + w2 + w3) / (3.0 * self.wheel_base_l)

        self.true_yaw = wrap_pi(self.true_yaw + wz * dt)
        c = math.cos(self.true_yaw)
        s = math.sin(self.true_yaw)
        vx_world = vx_body * c - vy_body * s
        vy_world = vx_body * s + vy_body * c
        self.true_x += vx_world * dt
        self.true_y += vy_world * dt

        rpm_per_count = 60.0 / (self.cpr * dt)
        pulse_timeout_s = 0.25
        alpha = 0.35

        rpm_meas = [0.0, 0.0, 0.0]
        for idx in range(3):
            self.t_since_edge_s[idx] += dt

            counts_f = (self.w_actual[idx] * dt * self.cpr / (2.0 * math.pi)) + self.count_residual[idx]
            delta_counts = int(round(counts_f))
            self.count_residual[idx] = counts_f - float(delta_counts)

            rpm_window = float(delta_counts) * rpm_per_count
            if delta_counts != 0:
                mag = abs(delta_counts)
                if self.t_since_edge_s[idx] > 0.0:
                    self.rpm_period[idx] = (float(delta_counts) * 60.0) / (self.cpr * self.t_since_edge_s[idx])
                self.t_since_edge_s[idx] = 0.0

                if mag <= 1:
                    blend_period = 0.85
                else:
                    blend_period = 0.35
                rpm_est = blend_period * self.rpm_period[idx] + (1.0 - blend_period) * rpm_window
            else:
                if self.t_since_edge_s[idx] > pulse_timeout_s:
                    self.rpm_period[idx] = 0.0
                rpm_est = self.rpm_period[idx]

            self.rpm_filt[idx] += alpha * (rpm_est - self.rpm_filt[idx])
            rpm_meas[idx] = self.rpm_filt[idx]

        return rpm_meas, vx_world, vy_world, wz, self.true_yaw, vx_body


class VirtualSTM32UDP:
    def __init__(
        self,
        server_host: str,
        server_port: int,
        local_host: str,
        local_port: int,
        rw: float,
        wheel_base_l: float,
        cpr: float,
        control_hz: float,
        pose_hz: float,
        motor_tau: float,
    ) -> None:
        self.server_addr = (server_host, server_port)
        self.local_addr = (local_host, local_port)

        self.control_dt = 1.0 / control_hz
        self.pose_dt = 1.0 / pose_hz

        self.sock: Optional[socket.socket] = None
        self.parser = StreamParser()

        self.controller = CM7Controller(rw=rw, wheel_base_l=wheel_base_l)
        self.estimator = CM7StateEstimator(rw=rw)
        self.plant = WheelPlantAndEncoder(rw=rw, wheel_base_l=wheel_base_l, cpr=cpr, motor_tau=motor_tau)

        self.running = False
        self.recv_thread: Optional[threading.Thread] = None
        self.control_thread: Optional[threading.Thread] = None

        self.latest_traj: Optional[TrajectoryFrame] = None
        self.traj_lock = threading.Lock()
        self.traj_mode = False

        self.pose_seq = 0
        self.cmd_seq = 0

        self.pose_state = [0.0, 0.0, 0.0]
        self.prev_pose_state = [0.0, 0.0, 0.0]
        self.last_pose_send_t = 0.0
        self.hal_tick_ms = 0
        self._last_selector = 0
        self._last_traj_idx = -1

        control_tick_ms = int(round(self.control_dt * 1000.0))
        self.control_tick_ms = max(1, control_tick_ms)

    def start(self, auto_start_ros2: bool = True, auto_start_traj: bool = True) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.local_addr)
        self.sock.settimeout(0.5)  # Increased from 0.05 to reduce CPU wake-ups

        self.running = True
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.recv_thread.start()
        self.control_thread.start()

        logger.info("Virtual STM32 started, local=%s:%s -> server=%s:%s", self.local_addr[0], self.local_addr[1], self.server_addr[0], self.server_addr[1])

        if auto_start_ros2:
            self.send_cmd(CommandID.START_RESTART_ROS2)
        if auto_start_traj:
            self.send_cmd(CommandID.START_TRAJ)
            self.traj_mode = True

    def stop(self) -> None:
        self.running = False
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        logger.info("Virtual STM32 stopped")

    def send_cmd(self, cmd_id: int) -> None:
        if not self.sock:
            return
        self.cmd_seq += 1
        payload = Command(cmd_id=int(cmd_id), arg=b"").pack()
        msg = make_message(MessageType.CMD, self.cmd_seq, payload, crc_payload=False)
        self.sock.sendto(msg, self.server_addr)
        logger.info("CMD sent: id=%s seq=%s", int(cmd_id), self.cmd_seq)

    def _recv_loop(self) -> None:
        while self.running:
            try:
                if not self.sock:
                    return
                data, _addr = self.sock.recvfrom(4096)
                self.parser.feed(data)
                while True:
                    parsed = self.parser.parse_message()
                    if parsed is None:
                        break
                    header, payload = parsed
                    self._handle_message(header.msg_type, header.seq, payload)
            except socket.timeout:
                continue
            except Exception as exc:
                if self.running:
                    logger.error("Receive loop error: %s", exc)
                return

    def _handle_message(self, msg_type: int, seq: int, payload: bytes) -> None:
        if msg_type == MessageType.TRAJ:
            traj = self._parse_traj_payload(seq, payload)
            if traj is not None:
                with self.traj_lock:
                    self.latest_traj = traj
                if seq % 20 == 0:
                    logger.info("TRAJ rx: seq=%s knots=%s dt=%.3f", seq, len(traj.knots), traj.dt)
                    # Log first, middle, and last knots to verify trajectory content
                    if len(traj.knots) > 0:
                        logger.info("  knot[0]:  x=%.3f y=%.3f yaw=%.3f vx=%.3f vy=%.3f", *traj.knots[0])
                        if len(traj.knots) > 1:
                            mid = len(traj.knots) // 2
                            logger.info("  knot[%d]: x=%.3f y=%.3f yaw=%.3f vx=%.3f vy=%.3f", mid, *traj.knots[mid])
                            logger.info("  knot[%d]: x=%.3f y=%.3f yaw=%.3f vx=%.3f vy=%.3f", len(traj.knots)-1, *traj.knots[-1])
            return

        if msg_type == MessageType.CMD:
            try:
                cmd = Command.unpack(payload)
                logger.info("CMD rx (ack): cmd_id=%s seq=%s", cmd.cmd_id, seq)
            except Exception:
                logger.debug("CMD rx parse failed")
            return

    @staticmethod
    def _parse_traj_payload(seq: int, payload: bytes) -> Optional[TrajectoryFrame]:
        if len(payload) < 20:
            return None
        try:
            reply_to_pose_seq, traj_t0_ms, n_knots, _flags, _reserved, dt = struct.unpack("<IIHHIf", payload[:20])
            _ = reply_to_pose_seq
            knots: List[Tuple[float, float, float, float, float]] = []
            offset = 20
            for _idx in range(n_knots):
                if offset + 20 > len(payload):
                    break
                x, y, yaw, vx, vy = struct.unpack("<fffff", payload[offset : offset + 20])
                knots.append((float(x), float(y), float(yaw), float(vx), float(vy)))
                offset += 20
            if not knots:
                return None
            return TrajectoryFrame(seq=seq, traj_t0_ms=int(traj_t0_ms), dt=float(dt), knots=knots)
        except Exception:
            return None

    def _select_desired_state(self) -> Tuple[List[float], List[float], int]:
        xd = [0.0, 0.0, 0.0, 0.0, 0.0]
        vd = [0.0, 0.0, 0.0]

        if not self.traj_mode:
            return xd, vd, 0

        with self.traj_lock:
            traj = self.latest_traj
        if traj is None or traj.dt <= 0.0 or not traj.knots:
            return xd, vd, 1

        # Match CM7 logic exactly:
        #   now_ms = HAL_GetTick()
        #   elapsed_ms = (now_ms >= traj_t0_ms) ? (now_ms - traj_t0_ms) : 0
        now_ms = self.hal_tick_ms
        elapsed_ms = (now_ms - traj.traj_t0_ms) if now_ms >= traj.traj_t0_ms else 0
        dt_ms = max(1, int(traj.dt * 1000.0))
        idx = min(len(traj.knots) - 1, elapsed_ms // dt_ms)

        x, y, yaw, vx, vy = traj.knots[idx]
        xd[0] = x
        xd[1] = y
        xd[2] = yaw
        # Use feedforward velocities directly from trajectory
        xd[3] = vx
        xd[4] = vy

        if idx != self._last_traj_idx:
            logger.debug(
                "TRAJ consume: hal_tick_ms=%d t0=%d dt_ms=%d idx=%d/%d",
                now_ms,
                traj.traj_t0_ms,
                dt_ms,
                idx,
                len(traj.knots) - 1,
            )
            self._last_traj_idx = idx

        return xd, vd, 1

    def _send_pose(self, vx_world: float, vy_world: float, wz: float) -> None:
        if not self.sock:
            return
        self.pose_seq += 1
        pose_t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        p = Pose(
            pose_t_ms=pose_t_ms,
            x=float(self.pose_state[0]),
            y=float(self.pose_state[1]),
            yaw=float(self.pose_state[2]),
            vx=float(vx_world),
            vy=float(vy_world),
            wz=float(wz),
        )
        payload = p.pack()
        msg = make_message(MessageType.POSE, self.pose_seq, payload, crc_payload=False)
        self.sock.sendto(msg, self.server_addr)
        if self.pose_seq % 25 == 0:
            logger.info("POSE tx: seq=%s x=%.3f y=%.3f yaw=%.3f", self.pose_seq, self.pose_state[0], self.pose_state[1], self.pose_state[2])

    def _control_loop(self) -> None:
        last_t = time.monotonic()
        self.last_pose_send_t = last_t
        imu_zeroed = False
        next_control_t = last_t + self.control_dt

        while self.running:
            now_t = time.monotonic()
            # Sleep until next control cycle (more efficient than tight polling)
            sleep_time = next_control_t - now_t
            if sleep_time > 0.001:  # Only sleep if >1ms remains
                time.sleep(sleep_time * 0.95)  # Sleep 95% to avoid overshooting
                continue
            
            # Match CM7 fixed-step scheduler semantics.
            dt = self.control_dt
            last_t = now_t
            next_control_t += self.control_dt
            if (now_t - next_control_t) > 0.5:
                next_control_t = now_t + self.control_dt

            self.hal_tick_ms = (self.hal_tick_ms + self.control_tick_ms) & 0xFFFFFFFF

            xd, vd, selector = self._select_desired_state()

            if self._last_selector == 0 and selector == 1:
                # CM7 rising edge behavior when entering traj mode.
                self.estimator.reset(0.0, 0.0, 0.0)
                if imu_zeroed:
                    self.estimator.zero_imu_yaw(self.plant.true_yaw)
                xd = [0.0, 0.0, 0.0, 0.0, 0.0]
                self._last_traj_idx = -1
            self._last_selector = selector

            wheel_cmd = self.controller.step(self.pose_state, xd, vd, selector, dt)

            rpm_meas, vx_world_true, vy_world_true, wz_true, imu_yaw, _vx_body = self.plant.update(wheel_cmd, dt)
            wheel_rad_s = [rpm_meas[i] * (2.0 * math.pi / 60.0) for i in range(3)]

            if not imu_zeroed:
                self.estimator.reset(0.0, 0.0, 0.0)
                self.estimator.zero_imu_yaw(imu_yaw)
                imu_zeroed = True

            self.prev_pose_state = self.pose_state[:]
            self.pose_state = self.estimator.update(wheel_rad_s, dt, imu_yaw)

            if (now_t - self.last_pose_send_t) >= self.pose_dt:
                self._send_pose(vx_world_true, vy_world_true, wz_true)
                self.last_pose_send_t = now_t


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Virtual STM32 UDP simulator for OMNI")
    parser.add_argument("--server-host", default="127.0.0.1", help="Pi UDP server host")
    parser.add_argument("--server-port", type=int, default=9000, help="Pi UDP server port")
    parser.add_argument("--local-host", default="0.0.0.0", help="Local bind host")
    parser.add_argument("--local-port", type=int, default=0, help="Local bind port (0 = ephemeral)")
    parser.add_argument("--rw", type=float, default=0.09, help="Wheel radius (m)")
    parser.add_argument("--wheel-base-l", type=float, default=0.2, help="Center-to-wheel distance L (m)")
    parser.add_argument("--cpr", type=float, default=5303.0, help="Encoder counts per revolution")
    parser.add_argument("--control-hz", type=float, default=100.0, help="Control loop rate")
    parser.add_argument("--pose-hz", type=float, default=5.0, help="POSE publish rate")
    parser.add_argument("--motor-tau", type=float, default=0.08, help="Motor first-order lag time constant")
    parser.add_argument("--no-auto-start-ros2", action="store_true", help="Do not send START_RESTART_ROS2 on startup")
    parser.add_argument("--no-auto-start-traj", action="store_true", help="Do not send START_TRAJ on startup")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    sim = VirtualSTM32UDP(
        server_host=args.server_host,
        server_port=args.server_port,
        local_host=args.local_host,
        local_port=args.local_port,
        rw=args.rw,
        wheel_base_l=args.wheel_base_l,
        cpr=args.cpr,
        control_hz=args.control_hz,
        pose_hz=args.pose_hz,
        motor_tau=args.motor_tau,
    )

    try:
        sim.start(
            auto_start_ros2=not args.no_auto_start_ros2,
            auto_start_traj=not args.no_auto_start_traj,
        )
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Stopping virtual STM32...")
    finally:
        sim.stop()


if __name__ == "__main__":
    main()

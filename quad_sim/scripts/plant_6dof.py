#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np


def clamp(x, xmin, xmax):
    return max(xmin, min(xmax, x))


def rot_enu_from_flu(roll, pitch, yaw):
    """
    body(FLU) -> world(ENU)
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]], dtype=float)
    ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]], dtype=float)
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr,  cr]], dtype=float)
    return rz @ ry @ rx


class Quad6DOFPlant:
    """
    纯动力学模型，不依赖 ROS / MAVLink
    世界系: ENU
    机体系: FLU
    """

    def __init__(self, params=None):
        params = params or {}

        self.m = float(params.get("mass", 1.5))
        self.g = float(params.get("gravity", 9.81))

        self.Jx = float(params.get("Jx", 0.02))
        self.Jy = float(params.get("Jy", 0.02))
        self.Jz = float(params.get("Jz", 0.04))
        self.J = np.diag([self.Jx, self.Jy, self.Jz])
        self.J_inv = np.linalg.inv(self.J)

        # PX4 控制量 -> 等效推力/力矩
        self.k_thrust = float(params.get("k_thrust", 9.2))
        self.k_tau_roll = float(params.get("k_tau_roll", 0.03))
        self.k_tau_pitch = float(params.get("k_tau_pitch", 0.03))
        self.k_tau_yaw = float(params.get("k_tau_yaw", 0.01))
        self.motor_roll_moment = float(params.get("motor_roll_moment", 0.08))
        self.motor_pitch_moment = float(params.get("motor_pitch_moment", 0.08))
        self.motor_yaw_moment = float(params.get("motor_yaw_moment", 0.05))

        self.kv = float(params.get("linear_damping", 1.5))
        self.kw = float(params.get("angular_damping", 0.35))

        self.hover_throttle = float(params.get("hover_throttle", 0.40))
        self.throttle_scale = float(params.get("throttle_scale", 0.15))
        self.throttle_alpha = float(params.get("throttle_alpha", 0.95))
        self.idle_throttle_deadzone = float(params.get("idle_throttle_deadzone", 0.05))

        self.tau_scale_air = float(params.get("tau_scale_air", 0.15))
        self.tau_scale_ground = float(params.get("tau_scale_ground", 0.0))
        self.max_tilt_rad = float(params.get("max_tilt_rad", 0.8))
        self.ground_attitude_damping = float(params.get("ground_attitude_damping", 0.98))

        self.wind_force_enu = np.array([
            float(params.get("wind_force_x", 0.0)),
            float(params.get("wind_force_y", 0.0)),
            float(params.get("wind_force_z", 0.0)),
        ], dtype=float)
        self.wind_sine_amp = float(params.get("wind_sine_amp", 0.0))
        self.wind_sine_freq = float(params.get("wind_sine_freq", 0.5))

        self.reset()

    def reset(self):
        self.p_enu = np.zeros(3, dtype=float)
        self.v_enu = np.zeros(3, dtype=float)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.omega_flu = np.zeros(3, dtype=float)
        self.acc_enu = np.zeros(3, dtype=float)

        # [roll, pitch, yaw, throttle], used by the older MAVROS actuator topic bridge.
        self.u = np.zeros(4, dtype=float)
        self.motor_outputs = np.zeros(4, dtype=float)
        self.control_mode = "actuator_controls"

        self.throttle_filt = 0.0
        self.time_s = 0.0

    def freeze_at_origin(self):
        self.p_enu[:] = 0.0
        self.v_enu[:] = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.omega_flu[:] = 0.0
        self.acc_enu[:] = 0.0
        self.u[:] = 0.0
        self.motor_outputs[:] = 0.0
        self.control_mode = "actuator_controls"
        self.throttle_filt = 0.0

    def set_actuator_controls(self, controls):
        c = np.asarray(controls, dtype=float).reshape(-1)
        if c.size < 4:
            raise ValueError("controls must have at least 4 elements")
        self.u[:] = c[:4]
        self.control_mode = "actuator_controls"

    def set_motor_outputs(self, outputs):
        c = np.asarray(outputs, dtype=float).reshape(-1)
        if c.size < 4:
            raise ValueError("outputs must have at least 4 elements")
        self.motor_outputs[:] = [clamp(float(x), 0.0, 1.0) for x in c[:4]]
        self.control_mode = "motor_outputs"

    def _disturbance_force_enu(self):
        fx = self.wind_force_enu[0] + self.wind_sine_amp * math.sin(
            2.0 * math.pi * self.wind_sine_freq * self.time_s
        )
        fy = self.wind_force_enu[1]
        fz = self.wind_force_enu[2]
        return np.array([fx, fy, fz], dtype=float)

    def _motor_to_force_torque(self):
        if self.control_mode == "motor_outputs":
            return self._motor_outputs_to_force_torque()

        roll_ctrl = clamp(float(self.u[0]), -0.7, 0.7)
        pitch_ctrl = clamp(float(self.u[1]), -0.7, 0.7)
        yaw_ctrl = clamp(float(self.u[2]), -0.7, 0.7)
        px4_throttle = clamp(float(self.u[3]), 0.0, 1.0)

        # 低油门真零推力
        if px4_throttle <= self.idle_throttle_deadzone:
            self.throttle_filt = 0.0
            thrust = 0.0
        else:
            throttle_cmd = self.hover_throttle + self.throttle_scale * (
                px4_throttle - self.hover_throttle
            )
            throttle_cmd = clamp(throttle_cmd, 0.0, 1.0)

            self.throttle_filt = (
                self.throttle_alpha * self.throttle_filt
                + (1.0 - self.throttle_alpha) * throttle_cmd
            )
            thrust = self.k_thrust * 4.0 * self.throttle_filt

        tau_scale = self.tau_scale_ground if self.p_enu[2] <= 0.05 else self.tau_scale_air
        tau = np.array([
            tau_scale * self.k_tau_roll * roll_ctrl,
            tau_scale * self.k_tau_pitch * pitch_ctrl,
            tau_scale * self.k_tau_yaw * yaw_ctrl,
        ], dtype=float)

        return thrust, tau

    def _motor_outputs_to_force_torque(self):
        u = np.asarray([clamp(float(x), 0.0, 1.0) for x in self.motor_outputs], dtype=float)
        thrusts = self.k_thrust * u
        thrust = float(np.sum(thrusts))

        # Match PX4 v1.13 quad_w normalized mixer signs:
        # front_right, rear_left, front_left, rear_right.
        # Use only differential output for moments so collective throttle does not
        # create artificial torque from small geometry/model mismatches.
        du = u - float(np.mean(u))
        roll_scale = np.array([-0.495383, 0.495383, 0.495383, -0.495383], dtype=float)
        pitch_scale = np.array([-0.707107, 0.707107, -0.707107, 0.707107], dtype=float)
        yaw_scale = np.array([-0.765306, -1.0, 0.765306, 1.0], dtype=float)

        roll_tau = self.motor_roll_moment * float(np.dot(du, roll_scale))
        pitch_tau = self.motor_pitch_moment * float(np.dot(du, pitch_scale))
        yaw_tau = self.motor_yaw_moment * float(np.dot(du, yaw_scale))

        tau_scale = self.tau_scale_ground if self.p_enu[2] <= 0.05 else self.tau_scale_air
        tau = tau_scale * np.array([roll_tau, pitch_tau, yaw_tau], dtype=float)

        return thrust, tau

    def step(self, dt):
        dt = float(dt)
        if dt <= 0.0:
            return

        self.time_s += dt

        thrust, tau = self._motor_to_force_torque()
        R = rot_enu_from_flu(self.roll, self.pitch, self.yaw)

        # 平动
        f_thrust_enu = R @ np.array([0.0, 0.0, thrust], dtype=float)
        f_gravity_enu = np.array([0.0, 0.0, -self.m * self.g], dtype=float)
        f_damping_enu = -self.kv * self.v_enu
        f_disturb_enu = self._disturbance_force_enu()

        total_force_enu = f_thrust_enu + f_gravity_enu + f_damping_enu + f_disturb_enu
        self.acc_enu = total_force_enu / self.m

        self.v_enu = self.v_enu + self.acc_enu * dt
        self.p_enu = self.p_enu + self.v_enu * dt

        # 转动
        Jw = self.J @ self.omega_flu
        coriolis = np.cross(self.omega_flu, Jw)
        tau_damping = self.kw * self.omega_flu
        omega_dot = self.J_inv @ (tau - coriolis - tau_damping)
        self.omega_flu = self.omega_flu + omega_dot * dt

        self.omega_flu[0] = clamp(self.omega_flu[0], -5.0, 5.0)
        self.omega_flu[1] = clamp(self.omega_flu[1], -5.0, 5.0)
        self.omega_flu[2] = clamp(self.omega_flu[2], -3.0, 3.0)

        # 欧拉角运动学
        p, q, r = self.omega_flu
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        ct = math.cos(self.pitch)
        if abs(ct) < 1e-4:
            ct = 1e-4 if ct >= 0.0 else -1e-4
        tt = math.tan(self.pitch)

        self.roll += (p + sr * tt * q + cr * tt * r) * dt
        self.pitch += (cr * q - sr * r) * dt
        self.yaw += ((sr / ct) * q + (cr / ct) * r) * dt

        self.roll = clamp(self.roll, -self.max_tilt_rad, self.max_tilt_rad)
        self.pitch = clamp(self.pitch, -self.max_tilt_rad, self.max_tilt_rad)

        # 地面约束
        if self.p_enu[2] < 0.0:
            self.p_enu[2] = 0.0
            if self.v_enu[2] < 0.0:
                self.v_enu[2] = 0.0
            self.v_enu[0] *= 0.8
            self.v_enu[1] *= 0.8
            self.omega_flu *= 0.8
            self.roll *= self.ground_attitude_damping
            self.pitch *= self.ground_attitude_damping

    def get_specific_force_body_flu(self):
        R = rot_enu_from_flu(self.roll, self.pitch, self.yaw)
        g_enu = np.array([0.0, 0.0, -self.g], dtype=float)
        specific_force_enu = self.acc_enu - g_enu
        return R.T @ specific_force_enu

    def get_state(self):
        return {
            "p_enu": self.p_enu.copy(),
            "v_enu": self.v_enu.copy(),
            "roll": float(self.roll),
            "pitch": float(self.pitch),
            "yaw": float(self.yaw),
            "omega_flu": self.omega_flu.copy(),
            "acc_enu": self.acc_enu.copy(),
            "u": self.u.copy(),
            "motor_outputs": self.motor_outputs.copy(),
            "control_mode": self.control_mode,
        }

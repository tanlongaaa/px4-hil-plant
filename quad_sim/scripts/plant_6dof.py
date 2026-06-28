#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plant_6dof.py — 6-DOF Quadrotor Plant (v9 recovery build)

Architecture (post 2026-06-19):
  Active control: du-based mixer (old-plant design, PX4 PID verified).
  Passive physics: linear body drag + rotor lateral drag (Martin & Salaün 2010)
                   + blade flapping + rotor gyroscopic + dynamic inflow (Pitt-Peters)

Wind paths (single-entry, no triple-counting):
  set_wind_vel_enu  → rotor-level effects (lateral drag + blade flapping)
  set_ext_force_enu → body-level effects (quadratic drag, computed in backend)
  Linear damping    → v_body only, NO wind bias

Sources: iris.sdf, gazebo_motor_model.cpp, wind_dynamics_gap_analysis.md
Frames: World=ENU, Body=FLU
"""

import math
import numpy as np


def clamp(x, xmin, xmax):
    return max(xmin, min(xmax, x))


def rot_enu_from_flu(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


class Quad6DOFPlant:

    def __init__(self, params=None):
        p = params or {}

        # ── Rigid body — iris.sdf ──────────────────────
        self.m   = float(p.get("mass", 1.5))
        self.g   = float(p.get("gravity", 9.81))
        self.Jx  = float(p.get("Jx", 0.029125))
        self.Jy  = float(p.get("Jy", 0.029125))
        self.Jz  = float(p.get("Jz", 0.055225))
        self.J   = np.diag([self.Jx, self.Jy, self.Jz])
        self.J_inv = np.linalg.inv(self.J)

        # ── Active control — du-based mixer ────────────
        self.k_thrust        = float(p.get("k_thrust", 7.36))
        self.hover_throttle  = float(p.get("hover_throttle", 0.50))
        self.motor_roll_moment  = float(p.get("motor_roll_moment", 0.08))
        self.motor_pitch_moment = float(p.get("motor_pitch_moment", 0.08))
        self.motor_yaw_moment   = float(p.get("motor_yaw_moment", 0.05))
        self.tau_scale_air      = float(p.get("tau_scale_air", 1.0))
        self.tau_scale_ground   = float(p.get("tau_scale_ground", 0.0))
        self.k_tau_roll  = float(p.get("k_tau_roll", 0.02))
        self.k_tau_pitch = float(p.get("k_tau_pitch", 0.02))
        self.k_tau_yaw   = float(p.get("k_tau_yaw", 0.008))
        self.idle_throttle_deadzone = float(p.get("idle_throttle_deadzone", 0.05))

        # ── Linear body drag (v9) ──────────────────
        # base_drag = bulk isotropic from Iris calibration
        # body_drag_x/y = anisotropic corrections ON TOP of base (v9: lateral > frontal)
        self.base_drag   = float(p.get("base_linear_drag", 3.5))
        self.body_drag_x = float(p.get("body_drag_x", 0.10))
        self.body_drag_y = float(p.get("body_drag_y", 0.30))
        self.body_drag_z = float(p.get("body_drag_z", 0.1))

        # ── Rotor lateral drag (Martin & Salaün 2010) ──
        self.k_rotor_drag = float(p.get("rotor_drag_coefficient", 0.000175))
        self.omega_hover  = float(p.get("omega_hover", 800.0))
        self.omega_scale  = float(p.get("omega_scale", 0.5))

        # ── Blade flapping (v9) ────────────────────────
        # τ_flap = -Σ|ω_i| · k_flap · v_rel_horizontal  (wind-vane effect)
        # NOTE (2026-06-28): default lowered 5e-5 → 1e-5. Real blade flapping has
        # its own (rotor-revolution) time constant and CANNOT track 20Hz Dryden
        # turbulence. The old value turned random wind into high-frequency torque
        # noise (±40-55 rad/s² @ σ=4) that PX4's rate loop could not reject.
        self.flapping_coefficient = float(p.get("flapping_coefficient", 1e-05))

        # ── Wind low-pass (2026-06-28) ─────────────────
        # Rotor-level effects (lateral drag + blade flapping) see only a
        # quasi-steady wind. A first-order LPF strips the high-frequency
        # turbulence that the physical rotor cannot follow. tau≈0.15s.
        # Set wind_filter_tau<=0 to disable (raw wind, legacy behaviour).
        self.wind_filter_tau = float(p.get("wind_filter_tau", 0.15))
        self.wind_vel_filt_enu = np.zeros(3)

        # ── Rotor gyroscopic (v9) ──────────────────────
        # τ_gyro = I_rotor · Σ Ω_i · (e_z × ω_body)
        self.I_rotor = float(p.get("I_rotor", 4e-05))

        # ── Dynamic inflow — Pitt-Peters (v9) ──────────
        self.inflow_tau = float(p.get("inflow_tau", 0.05))
        self.v_inflow = 0.0

        # ── Body quadratic drag coefficient (v9) ───────
        # Used by backend_main via ext_force_enu
        self.body_CdA = float(p.get("body_CdA", 0.12))
        self.rho_air  = float(p.get("rho", 1.225))

        # ── Angular damping ────────────────────────────
        self.kw = float(p.get("angular_damping", 0.3))

        # ── Limits ─────────────────────────────────────
        self.max_tilt_rad            = float(p.get("max_tilt_rad", 0.8))
        self.ground_attitude_damping = float(p.get("ground_attitude_damping", 0.98))

        # ── Rotor positions — iris.sdf (FLU) ──────────
        self.rotor_positions = np.array([
            [ 0.13, -0.22, 0.023],
            [-0.13,  0.20, 0.023],
            [ 0.13,  0.22, 0.023],
            [-0.13, -0.20, 0.023],
        ])
        # Rotor spin directions (ε: CCW=+1, CW=-1)
        # Match PX4 quad_x mixer roll signs
        self.rotor_spin = np.array([-1.0, 1.0, 1.0, -1.0])

        # ── External interfaces ────────────────────────
        self.ext_force_enu = np.zeros(3)
        self.wind_vel_enu  = np.zeros(3)

        self.reset()

    def reset(self):
        self.p_enu    = np.zeros(3)
        self.v_enu    = np.zeros(3)
        self.roll = self.pitch = self.yaw = 0.0
        self.omega_flu = np.zeros(3)
        self.acc_enu   = np.zeros(3)
        self.u = np.zeros(4)
        self.motor_outputs = np.zeros(4)
        self.control_mode = "actuator_controls"
        self.ext_force_enu[:] = 0.0
        self.wind_vel_enu[:]  = 0.0
        self.wind_vel_filt_enu[:] = 0.0
        self.v_inflow = 0.0
        self.time_s = 0.0

    def freeze_at_origin(self):
        self.reset()

    def set_actuator_controls(self, c):
        c = np.asarray(c).ravel()
        if c.size < 4: raise ValueError("need 4 actuator controls")
        self.u[:] = c[:4]
        self.control_mode = "actuator_controls"

    def set_motor_outputs(self, c):
        c = np.asarray(c).ravel()
        if c.size < 4: raise ValueError("need 4 motor outputs")
        self.motor_outputs[:] = [clamp(float(x), 0, 1) for x in c[:4]]
        self.control_mode = "motor_outputs"

    def set_ext_force_enu(self, f):
        self.ext_force_enu[:] = np.asarray(f).ravel()[:3]

    def set_wind_vel_enu(self, w, dt=None):
        """Set commanded wind (ENU). The rotor-level physics use a low-pass
        filtered copy (wind_vel_filt_enu) so high-frequency turbulence is not
        converted into unphysical torque noise. Pass dt for time-accurate
        filtering; if omitted, a fixed alpha is used."""
        self.wind_vel_enu[:] = np.asarray(w).ravel()[:3]
        tau = self.wind_filter_tau
        if tau <= 0.0:
            self.wind_vel_filt_enu[:] = self.wind_vel_enu
            return
        if dt is not None and dt > 0.0:
            alpha = float(dt) / (tau + float(dt))
        else:
            alpha = 0.3
        self.wind_vel_filt_enu += alpha * (self.wind_vel_enu - self.wind_vel_filt_enu)

    # ══════════════════════════════════════════════════════
    #  Active control
    # ══════════════════════════════════════════════════════

    def _compute_rotor_omegas(self):
        """Angular velocity (rad/s) for each rotor from motor outputs."""
        ht = max(self.hover_throttle, 0.01)
        return self.omega_hover * (self.motor_outputs / ht) ** self.omega_scale

    def _motor_outputs_to_force_torque(self):
        u = np.array(self.motor_outputs)
        du = u - float(np.mean(u))
        thrust = self.k_thrust * float(np.sum(u))
        rs = np.array([-0.495383, 0.495383, 0.495383, -0.495383])
        ps = np.array([-0.707107, 0.707107, -0.707107, 0.707107])
        ys = np.array([-0.765306, -1.0, 0.765306, 1.0])
        rt = self.motor_roll_moment  * float(np.dot(du, rs))
        pt = self.motor_pitch_moment * float(np.dot(du, ps))
        yt = self.motor_yaw_moment   * float(np.dot(du, ys))
        ts = self.tau_scale_ground if self.p_enu[2] <= 0.05 else self.tau_scale_air
        return thrust, ts * np.array([rt, pt, yt])

    def _actuator_to_force_torque(self):
        r = clamp(float(self.u[0]), -0.7, 0.7)
        p = clamp(float(self.u[1]), -0.7, 0.7)
        y = clamp(float(self.u[2]), -0.7, 0.7)
        t = clamp(float(self.u[3]), 0, 1)
        if t <= self.idle_throttle_deadzone:
            thrust = 0.0
        else:
            thrust = self.m * self.g * (t / max(self.hover_throttle, 0.01))
        ts = self.tau_scale_ground if self.p_enu[2] <= 0.05 else self.tau_scale_air
        return thrust, ts * np.array([self.k_tau_roll * r,
                                       self.k_tau_pitch * p,
                                       self.k_tau_yaw   * y])

    # ══════════════════════════════════════════════════════
    #  Passive physics — rotor-level (wind enters here)
    # ══════════════════════════════════════════════════════

    def _v_rel_body_flu(self):
        """Relative velocity (body FLU): v_drone - v_wind.
        Uses the low-pass filtered wind so rotor-level effects (lateral drag,
        blade flapping) only respond to quasi-steady wind, not turbulence."""
        R = rot_enu_from_flu(self.roll, self.pitch, self.yaw)
        return R.T @ (self.v_enu - self.wind_vel_filt_enu)

    def _rotor_lateral_drag_flu(self, v_rel_body, omegas):
        """F_drag = -Σ|ω_i| · k_drag · v_perp  (Martin & Salaün 2010)"""
        drag = np.zeros(3)
        for i in range(4):
            vp = np.array([v_rel_body[0], v_rel_body[1], 0.0])
            drag -= abs(omegas[i]) * self.k_rotor_drag * vp
        return drag

    def _blade_flapping_torque_flu(self, v_rel_body, omegas):
        """
        Blade flapping restoring torque (wind-vane effect).
        τ_flap = -Σ|ω_i| · k_flap · v_rel_horizontal

        Horizontal flow → advancing/retreating blade asymmetry
        → rotor disc tilts away from flow → restoring moment.
        Key passive anti-wind mechanism for multirotors.
        """
        v_h = np.array([v_rel_body[0], v_rel_body[1], 0.0])
        torque = np.zeros(3)
        for i in range(4):
            torque -= abs(omegas[i]) * self.flapping_coefficient * v_h
        return torque

    def _gyroscopic_torque_flu(self, omegas):
        """
        Rotor gyroscopic torque.
        τ_gyro = I_rotor · Σ ε_i·ω_i · (e_z × ω_body)

        Spinning rotors resist body rotation via gyroscopic precession.
        ε_i = +1 (CCW), -1 (CW).
        """
        omega_sum = float(np.dot(self.rotor_spin, omegas))
        if abs(omega_sum) < 1e-6:
            return np.zeros(3)
        # e_z = [0, 0, 1] in FLU (thrust direction)
        e_z = np.array([0.0, 0.0, 1.0])
        return self.I_rotor * omega_sum * np.cross(e_z, self.omega_flu)

    def _dynamic_inflow_update(self, thrust, dt):
        """
        Pitt-Peters dynamic inflow model (first-order filter).
        τ · v̇_inflow + v_inflow = v_steady
        v_steady = sqrt(T / (2·ρ·A))
        """
        # Disc area estimate (0.255m radius for 10" prop, Iris)
        A = math.pi * 0.13 ** 2
        if thrust > 0 and self.rho_air > 0:
            v_steady = math.sqrt(thrust / (2.0 * self.rho_air * A))
        else:
            v_steady = 0.0
        alpha = dt / (self.inflow_tau + dt)
        self.v_inflow += alpha * (v_steady - self.v_inflow)
        return self.v_inflow

    # ══════════════════════════════════════════════════════
    #  Step — main integration
    # ══════════════════════════════════════════════════════

    def step(self, dt):
        dt = float(dt)
        if dt <= 0:
            return
        self.time_s += dt
        R = rot_enu_from_flu(self.roll, self.pitch, self.yaw)

        # ── Active control ──────────────────────────────
        if self.control_mode == "motor_outputs":
            thrust, torque_active = self._motor_outputs_to_force_torque()
            omegas = self._compute_rotor_omegas()
        else:
            thrust, torque_active = self._actuator_to_force_torque()
            # Estimate rotor speeds from thrust for passive physics
            motor_thrust_each = thrust / 4.0 if thrust > 0 else 0.0
            omega_est = math.sqrt(motor_thrust_each / max(self.k_thrust, 0.01)) if motor_thrust_each > 0 else 0.0
            omegas = np.full(4, omega_est)

        # ── Relative velocity (shared for rotor-level physics) ─
        v_rel = self._v_rel_body_flu()

        # ── Forces (ENU) ────────────────────────────────
        # Thrust
        f_thrust = R @ np.array([0.0, 0.0, thrust])

        # Gravity
        f_grav = np.array([0.0, 0.0, -self.m * self.g])

        # Rotor lateral drag (body FLU → ENU)
        f_rotor_drag = R @ self._rotor_lateral_drag_flu(v_rel, omegas)

        # Anisotropic linear body drag (body FLU → ENU, NO wind — v9 fix)
        # total = base (Iris bulk) + anisotropic correction (v9 tuning)
        v_body = R.T @ self.v_enu
        f_body_drag = R @ np.array([
            -(self.base_drag + self.body_drag_x) * v_body[0],
            -(self.base_drag + self.body_drag_y) * v_body[1],
            -(self.base_drag + self.body_drag_z) * v_body[2],
        ])

        # External force (body quadratic wind drag from backend)
        f_ext = self.ext_force_enu.copy()

        # Sum forces
        f_total = f_thrust + f_grav + f_rotor_drag + f_body_drag + f_ext
        self.acc_enu = f_total / self.m
        self.v_enu += self.acc_enu * dt
        self.p_enu += self.v_enu * dt

        # ── Torques (body FLU) ──────────────────────────
        # Blade flapping
        tau_flap = self._blade_flapping_torque_flu(v_rel, omegas)

        # Rotor gyroscopic
        tau_gyro = self._gyroscopic_torque_flu(omegas)

        # Dynamic inflow (affects thrust indirectly — momentum theory)
        self._dynamic_inflow_update(thrust, dt)

        # Sum torques
        tau_total = torque_active + tau_flap + tau_gyro

        # Angular dynamics
        Jw = self.J @ self.omega_flu
        coriolis = np.cross(self.omega_flu, Jw)
        tau_damp = self.kw * self.omega_flu
        w_dot = self.J_inv @ (tau_total - coriolis - tau_damp)
        self.omega_flu += w_dot * dt
        self.omega_flu[0] = clamp(self.omega_flu[0], -5, 5)
        self.omega_flu[1] = clamp(self.omega_flu[1], -5, 5)
        self.omega_flu[2] = clamp(self.omega_flu[2], -3, 3)

        # ── Euler kinematics ────────────────────────────
        p, q, r = self.omega_flu
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        ct = math.cos(self.pitch)
        if abs(ct) < 1e-4:
            ct = 1e-4 if ct >= 0 else -1e-4
        tt = math.tan(self.pitch)
        self.roll  += (p + sr * tt * q + cr * tt * r) * dt
        self.pitch += (cr * q - sr * r) * dt
        self.yaw   += ((sr / ct) * q + (cr / ct) * r) * dt
        self.roll  = clamp(self.roll,  -self.max_tilt_rad, self.max_tilt_rad)
        self.pitch = clamp(self.pitch, -self.max_tilt_rad, self.max_tilt_rad)

        # ── Ground constraint ───────────────────────────
        if self.p_enu[2] < 0:
            self.p_enu[2] = 0
            if self.v_enu[2] < 0:
                self.v_enu[2] = 0
            self.v_enu[:2] *= 0.8
            self.omega_flu *= 0.8
            self.roll  *= self.ground_attitude_damping
            self.pitch *= self.ground_attitude_damping

    def get_specific_force_body_flu(self):
        R = rot_enu_from_flu(self.roll, self.pitch, self.yaw)
        return R.T @ (self.acc_enu - np.array([0, 0, -self.g]))

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

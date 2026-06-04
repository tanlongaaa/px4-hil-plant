#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import numpy as np
from pymavlink import mavutil


class MavlinkHilBackend:
    """
    Minimal PX4 HIL simulator backend over MAVLink TCP.

    It listens on tcpin:0.0.0.0:4560, receives HIL_ACTUATOR_CONTROLS from PX4,
    and publishes HIL_SENSOR / HIL_GPS / HIL_STATE_QUATERNION messages.
    """

    def __init__(self, config=None):
        config = config or {}

        self.connection_url = config.get("connection_url", "tcpin:0.0.0.0:4560")
        self.source_system = int(config.get("source_system", 142))
        self.source_component = int(config.get("source_component", 1))
        self.target_system = int(config.get("target_system", 1))
        self.target_component = int(config.get("target_component", 1))

        self.actuator_axis_signs = np.array(
            config.get("actuator_axis_signs", [-1.0, -1.0, -1.0, 1.0]),
            dtype=float
        )
        self.actuator_input_type = str(config.get("actuator_input_type", "motor_outputs"))
        self.control_timeout_sec = float(config.get("control_timeout_sec", 0.5))
        self.log_interval_sec = float(config.get("log_interval_sec", 1.0))

        self.master = None
        self.last_actuator_controls = np.zeros(4, dtype=float)
        self.last_actuator_wall = None
        self.last_log_wall = 0.0
        self.actuator_count = 0

    def connect(self):
        print("Waiting for PX4 simulator connection on %s ..." % self.connection_url)
        self.master = mavutil.mavlink_connection(
            self.connection_url,
            source_system=self.source_system,
            source_component=self.source_component,
            autoreconnect=True,
            force_connected=False,
        )
        print("MAVLink backend ready.")

    def actuator_age(self):
        if self.last_actuator_wall is None:
            return None
        return time.monotonic() - self.last_actuator_wall

    def actuator_fresh(self):
        age = self.actuator_age()
        return age is not None and age < self.control_timeout_sec

    def get_actuator_controls(self):
        if self.actuator_fresh():
            return self.last_actuator_controls.copy()
        return np.zeros(4, dtype=float)

    def poll(self):
        if self.master is None:
            raise RuntimeError("connect() must be called before poll()")

        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

            if msg_type == "HIL_ACTUATOR_CONTROLS":
                self._handle_hil_actuator_controls(msg)
            elif msg_type == "TIMESYNC":
                self._handle_timesync(msg)
            elif msg_type == "PING":
                self._handle_ping(msg)

    def _handle_hil_actuator_controls(self, msg):
        controls = np.asarray(msg.controls[:4], dtype=float)
        mode = int(getattr(msg, "mode", 0))

        if self.actuator_input_type == "motor_outputs":
            mapped = np.array([max(0.0, min(1.0, float(x))) for x in controls], dtype=float)
        else:
            mapped = controls * self.actuator_axis_signs

        self.last_actuator_controls[:] = mapped
        self.last_actuator_wall = time.monotonic()
        self.actuator_count += 1

        now = time.monotonic()
        if now - self.last_log_wall > self.log_interval_sec:
            self.last_log_wall = now
            armed = bool(mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            hil = bool(mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED)
            print(
                "HIL_ACTUATOR_CONTROLS type=%s mode=0x%02x armed=%s hil=%s "
                "raw=[%.3f %.3f %.3f %.3f] mapped=[%.3f %.3f %.3f %.3f] mean/min/max=[%.3f %.3f %.3f]" %
                (
                    self.actuator_input_type,
                    mode,
                    armed,
                    hil,
                    controls[0], controls[1], controls[2], controls[3],
                    self.last_actuator_controls[0],
                    self.last_actuator_controls[1],
                    self.last_actuator_controls[2],
                    self.last_actuator_controls[3],
                    float(np.mean(self.last_actuator_controls)),
                    float(np.min(self.last_actuator_controls)),
                    float(np.max(self.last_actuator_controls)),
                )
            )

    def _handle_timesync(self, msg):
        now_ns = time.monotonic_ns()
        if getattr(msg, "tc1", 0) == 0:
            self.master.mav.timesync_send(int(msg.ts1), now_ns)

    def _handle_ping(self, msg):
        self.master.mav.ping_send(
            int(time.time() * 1.0e6),
            msg.seq,
            msg.get_srcSystem(),
            msg.get_srcComponent()
        )

    def send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def send_hil_sensor(self, msg):
        self.master.mav.send(msg)

    def send_hil_gps(self, msg):
        self.master.mav.send(msg)

    def send_hil_state_quaternion(self, msg):
        self.master.mav.send(msg)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time

import numpy as np
from tf.transformations import quaternion_from_euler

try:
    import yaml
except ImportError:
    yaml = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from mavlink_backend import MavlinkHilBackend
from plant_6dof import Quad6DOFPlant
from sensor_models import HilSensorModel


DEFAULT_CONFIG = {
    "loop_rate_hz": 250.0,
    "gps_rate_hz": 10.0,
    "state_rate_hz": 0.0,
    "enable_hil_state_quaternion": False,
    "heartbeat_rate_hz": 1.0,
    "ekf_warmup_sec": 3.0,
    "takeoff_throttle_threshold": 0.02,
    "ctrl_alpha": 0.85,
    "backend": {
        "connection_url": "tcpin:0.0.0.0:4560",
        "actuator_input_type": "motor_outputs",
        "actuator_axis_signs": [-1.0, -1.0, -1.0, 1.0],
        "control_timeout_sec": 0.5,
    },
    "plant": {},
    "sensors": {},
}


class RosDebugPublisher:
    def __init__(self):
        import rospy
        import tf
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from mavros_msgs.msg import ActuatorControl
        from geometry_msgs.msg import Vector3Stamped

        self.rospy = rospy
        self.Odometry = Odometry
        self.Imu = Imu
        self.ActuatorControl = ActuatorControl
        self.Vector3Stamped = Vector3Stamped
        self.tf_br = tf.TransformBroadcaster()

        rospy.init_node("quad_hil_backend", anonymous=False, disable_signals=True)
        self.odom_pub = rospy.Publisher("/sim/odom", Odometry, queue_size=20)
        self.imu_pub = rospy.Publisher("/sim/imu", Imu, queue_size=20)
        self.control_pub = rospy.Publisher("/sim/hil_actuator_controls", ActuatorControl, queue_size=20)

        # Wind field subscriber
        self._wind_vel = np.zeros(3)
        self._wind_stamp = 0.0
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)

    def _wind_cb(self, msg):
        self._wind_vel[0] = msg.vector.x
        self._wind_vel[1] = msg.vector.y
        self._wind_vel[2] = msg.vector.z
        self._wind_stamp = self.rospy.Time.now().to_sec()

    def wind_fresh(self):
        """True if wind data arrived within last 0.5s."""
        return (self.rospy.Time.now().to_sec() - self._wind_stamp) < 0.5

    def get_wind_drag_force(self, plant):
        """Body quadratic aerodynamic drag: F = 0.5·ρ·CdA·|Vrel|·Vrel.
        Uses the plant's low-pass filtered wind (single wind source) so the
        body-level drag and the rotor-level effects respond to the same
        quasi-steady wind. Call AFTER plant.set_wind_vel_enu()."""
        wind = plant.wind_vel_filt_enu
        Vrel = wind - plant.v_enu
        speed = float(np.linalg.norm(Vrel))
        if speed < 1e-6:
            return np.zeros(3)
        return 0.5 * plant.rho_air * plant.body_CdA * speed * Vrel

    def is_shutdown(self):
        return self.rospy.is_shutdown()

    def publish(self, plant):
        from geometry_msgs.msg import Quaternion

        st = plant.get_state()
        p = st["p_enu"]
        v = st["v_enu"]
        omega = st["omega_flu"]
        q = quaternion_from_euler(st["roll"], st["pitch"], st["yaw"])

        stamp = self.rospy.Time.now()

        odom = self.Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(p[0])
        odom.pose.pose.position.y = float(p[1])
        odom.pose.pose.position.z = float(p[2])
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x = float(v[0])
        odom.twist.twist.linear.y = float(v[1])
        odom.twist.twist.linear.z = float(v[2])
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])
        self.odom_pub.publish(odom)

        self.tf_br.sendTransform(
            (float(p[0]), float(p[1]), float(p[2])),
            q,
            stamp,
            "base_link",
            "map",
        )

        sf_body_flu = plant.get_specific_force_body_flu()
        imu = self.Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation = Quaternion(*q)
        imu.angular_velocity.x = float(omega[0])
        imu.angular_velocity.y = float(omega[1])
        imu.angular_velocity.z = float(omega[2])
        imu.linear_acceleration.x = float(sf_body_flu[0])
        imu.linear_acceleration.y = float(sf_body_flu[1])
        imu.linear_acceleration.z = float(sf_body_flu[2])
        self.imu_pub.publish(imu)

    def publish_controls(self, controls):
        msg = self.ActuatorControl()
        msg.header.stamp = self.rospy.Time.now()
        msg.group_mix = 0
        values = [float(x) for x in controls[:4]]
        msg.controls = values + [0.0] * (8 - len(values))
        self.control_pub.publish(msg)


def deep_update(base, updates):
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_update(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path):
    config = deep_update({}, DEFAULT_CONFIG.copy())
    config["backend"] = DEFAULT_CONFIG["backend"].copy()
    config["plant"] = DEFAULT_CONFIG["plant"].copy()
    config["sensors"] = DEFAULT_CONFIG["sensors"].copy()

    if not path:
        return config

    if yaml is None:
        raise RuntimeError("PyYAML is required to load config file: %s" % path)

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return deep_update(config, data)


def should_run(period, now, last):
    return now - last >= period


def main():
    parser = argparse.ArgumentParser(description="Minimal PX4 MAVLink HIL backend for Quad6DOFPlant")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "sim_default.yaml"),
        help="YAML config path"
    )
    parser.add_argument(
        "--ros-debug",
        action="store_true",
        help="publish /sim/odom and /sim/imu for RViz/flight_state_monitor"
    )
    args, _unknown_ros_args = parser.parse_known_args()

    config = load_config(args.config if os.path.exists(args.config) else None)
    ros_debug = args.ros_debug or bool(config.get("ros_debug", {}).get("enabled", False))

    loop_rate_hz = float(config.get("loop_rate_hz", 250.0))
    gps_rate_hz = float(config.get("gps_rate_hz", 10.0))
    state_rate_hz = float(config.get("state_rate_hz", 50.0))
    enable_hil_state_quaternion = bool(config.get("enable_hil_state_quaternion", False))
    heartbeat_rate_hz = float(config.get("heartbeat_rate_hz", 1.0))
    ekf_warmup_sec = float(config.get("ekf_warmup_sec", 3.0))
    takeoff_throttle_threshold = float(config.get("takeoff_throttle_threshold", 0.02))
    ctrl_alpha = float(config.get("ctrl_alpha", 0.85))

    plant = Quad6DOFPlant(config.get("plant", {}))
    plant.freeze_at_origin()

    sensors = HilSensorModel(config.get("sensors", {}))
    backend = MavlinkHilBackend(config.get("backend", {}))
    ros_pub = RosDebugPublisher() if ros_debug else None
    backend.connect()

    nominal_dt = 1.0 / loop_rate_hz
    gps_period = 1.0 / gps_rate_hz
    state_period = 1.0 / state_rate_hz if state_rate_hz > 0.0 else None
    heartbeat_period = 1.0 / heartbeat_rate_hz

    ctrl_filt = np.zeros(4, dtype=float)
    started_at = time.monotonic()
    last_loop = started_at
    last_gps = 0.0
    last_state = 0.0
    last_heartbeat = 0.0
    flight_started = False

    print(
        "HIL backend loop=%.1fHz gps=%.1fHz state=%.1fHz warmup=%.1fs" %
        (loop_rate_hz, gps_rate_hz, state_rate_hz, ekf_warmup_sec)
    )
    print(
        "HIL sensor mag_ned=[%.3f %.3f %.3f] mag_yaw_offset=%.3f rad" %
        (
            sensors.mag_gauss_ned[0],
            sensors.mag_gauss_ned[1],
            sensors.mag_gauss_ned[2],
            sensors.mag_yaw_offset_rad,
        )
    )

    while (ros_pub is None) or (not ros_pub.is_shutdown()):
        loop_start = time.monotonic()
        dt_wall = loop_start - last_loop
        last_loop = loop_start
        dt = min(max(dt_wall, 0.5 * nominal_dt), 1.5 * nominal_dt)

        backend.poll()

        phase = "EKF_WARMUP"
        elapsed = loop_start - started_at

        if elapsed < ekf_warmup_sec:
            plant.freeze_at_origin()
            ctrl_filt[:] = 0.0
        else:
            controls = backend.get_actuator_controls()
            motor_level = float(np.mean(controls))
            if not flight_started:
                if backend.actuator_fresh() and motor_level > takeoff_throttle_threshold:
                    flight_started = True
                    print("Throttle detected, entering dynamic plant phase.")
                else:
                    phase = "WAIT_TAKEOFF"
                    plant.freeze_at_origin()
                    ctrl_filt[:] = 0.0

            if flight_started:
                phase = "DYNAMIC"
                ctrl_filt = ctrl_alpha * ctrl_filt + (1.0 - ctrl_alpha) * controls
                if backend.actuator_input_type == "motor_outputs":
                    plant.set_motor_outputs(ctrl_filt)
                else:
                    plant.set_actuator_controls(ctrl_filt)

                # ── Wind injection (two-path: rotor-level + body-level) ──
                # set_wind_vel_enu runs the LPF (needs dt); get_wind_drag_force
                # then reads the filtered wind → single coherent wind source.
                wind_ok = ros_pub is not None and ros_pub.wind_fresh()
                if wind_ok:
                    plant.set_wind_vel_enu(ros_pub._wind_vel, dt)
                    plant.set_ext_force_enu(ros_pub.get_wind_drag_force(plant))
                else:
                    plant.set_wind_vel_enu(np.zeros(3), dt)
                    plant.set_ext_force_enu(np.zeros(3))

                plant.step(dt)

        backend.send_hil_sensor(sensors.hil_sensor(plant))
        if ros_pub is not None:
            ros_pub.publish(plant)
            ros_pub.publish_controls(ctrl_filt)

        if should_run(gps_period, loop_start, last_gps):
            last_gps = loop_start
            backend.send_hil_gps(sensors.hil_gps(plant))

        if enable_hil_state_quaternion and state_period is not None and should_run(state_period, loop_start, last_state):
            last_state = loop_start
            backend.send_hil_state_quaternion(sensors.hil_state_quaternion(plant))

        if should_run(heartbeat_period, loop_start, last_heartbeat):
            last_heartbeat = loop_start
            backend.send_heartbeat()
            st = plant.get_state()
            print(
                "phase=%s actuator_age=%s throttle=%.3f pos_enu=(%.2f %.2f %.2f)" %
                (
                    phase,
                    "%.3f" % backend.actuator_age() if backend.actuator_age() is not None else "none",
                    float(np.mean(ctrl_filt)),
                    st["p_enu"][0],
                    st["p_enu"][1],
                    st["p_enu"][2],
                )
            )

        sleep_s = nominal_dt - (time.monotonic() - loop_start)
        if sleep_s > 0.0:
            time.sleep(sleep_s)


if __name__ == "__main__":
    main()

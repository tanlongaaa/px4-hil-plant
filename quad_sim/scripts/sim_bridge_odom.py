#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import os
import sys

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import ActuatorControl

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from plant_6dof import Quad6DOFPlant, rot_enu_from_flu


def param_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class SimBridgeOdom:
    """
    估计级闭环桥：

    /mavros/target_actuator_control -> Plant -> /mavros/odometry/in
                                                  -> /sim/odom
                                                  -> /sim/imu
    """

    def __init__(self):
        rospy.init_node("sim_bridge_odom")
        rospy.loginfo("SIM_BRIDGE_ODOM VERSION = mavros_odom_loop_v2")

        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))
        self.ekf_warmup_sec = float(rospy.get_param("~ekf_warmup_sec", 3.0))
        self.takeoff_throttle_threshold = float(rospy.get_param("~takeoff_throttle_threshold", 0.02))
        self.control_timeout_sec = float(rospy.get_param("~control_timeout_sec", 0.5))
        self.odom_frame_id = str(rospy.get_param("~odom_frame_id", "map"))
        self.base_frame_id = str(rospy.get_param("~base_frame_id", "base_link"))
        self.mavros_odom_topic = str(rospy.get_param("~mavros_odom_topic", "/mavros/odometry/in"))
        self.enable_mavros_odom = param_bool(rospy.get_param("~enable_mavros_odom", False))

        self.publish_tf = bool(rospy.get_param("~publish_tf", False))

        # 控制输入低通滤波
        self.ctrl_alpha = float(rospy.get_param("~ctrl_alpha", 0.85))
        self.ctrl_filt = np.zeros(4, dtype=float)
        self.actuator_axis_signs = np.array(
            rospy.get_param("~actuator_axis_signs", [1.0, -1.0, -1.0, 1.0]),
            dtype=float
        )
        if self.actuator_axis_signs.size < 4:
            rospy.logwarn("~actuator_axis_signs 至少需要4个值，回退到 [1, -1, -1, 1]")
            self.actuator_axis_signs = np.array([1.0, -1.0, -1.0, 1.0], dtype=float)
        else:
            self.actuator_axis_signs = self.actuator_axis_signs[:4]

        plant_params = {
            "mass": rospy.get_param("~mass", 1.5),
            "gravity": rospy.get_param("~gravity", 9.81),

            "Jx": rospy.get_param("~Jx", 0.02),
            "Jy": rospy.get_param("~Jy", 0.02),
            "Jz": rospy.get_param("~Jz", 0.04),

            "k_thrust": rospy.get_param("~k_thrust", 9.1),
            "k_tau_roll": rospy.get_param("~k_tau_roll", 0.02),
            "k_tau_pitch": rospy.get_param("~k_tau_pitch", 0.02),
            "k_tau_yaw": rospy.get_param("~k_tau_yaw", 0.008),

            "linear_damping": rospy.get_param("~linear_damping", 4.0),
            "angular_damping": rospy.get_param("~angular_damping", 0.45),

            "hover_throttle": rospy.get_param("~hover_throttle", 0.42),
            "throttle_scale": rospy.get_param("~throttle_scale", 0.03),
            "throttle_alpha": rospy.get_param("~throttle_alpha", 0.97),
            "idle_throttle_deadzone": rospy.get_param("~idle_throttle_deadzone", 0.05),

            "tau_scale_air": rospy.get_param("~tau_scale_air", 0.08),
            "tau_scale_ground": rospy.get_param("~tau_scale_ground", 0.0),

            "wind_force_x": rospy.get_param("~wind_force_x", 0.0),
            "wind_force_y": rospy.get_param("~wind_force_y", 0.0),
            "wind_force_z": rospy.get_param("~wind_force_z", 0.0),
            "wind_sine_amp": rospy.get_param("~wind_sine_amp", 0.0),
            "wind_sine_freq": rospy.get_param("~wind_sine_freq", 0.5),
        }

        self.plant = Quad6DOFPlant(plant_params)
        self.plant.freeze_at_origin()

        self.mavros_odom_pub = (
            rospy.Publisher(self.mavros_odom_topic, Odometry, queue_size=20)
            if self.enable_mavros_odom else None
        )
        self.sim_odom_pub = rospy.Publisher("/sim/odom", Odometry, queue_size=20)
        self.sim_imu_pub = rospy.Publisher("/sim/imu", Imu, queue_size=20)
        self.tf_br = tf.TransformBroadcaster()

        rospy.loginfo(
            "mavros_odom_enabled=%s mavros_odom_topic=%s actuator_axis_signs=[%.1f, %.1f, %.1f, %.1f]",
            self.enable_mavros_odom,
            self.mavros_odom_topic,
            self.actuator_axis_signs[0],
            self.actuator_axis_signs[1],
            self.actuator_axis_signs[2],
            self.actuator_axis_signs[3],
        )

        rospy.Subscriber(
            "/mavros/target_actuator_control",
            ActuatorControl,
            self.actuator_cb,
            queue_size=1
        )

        self.last_controls = np.zeros(4, dtype=float)
        self.first_actuator_received = False
        self.last_actuator_msg_wall = None
        self.flight_started = False

        self.last_wall = time.monotonic()
        self.start_time = rospy.Time.now()

    def actuator_cb(self, msg: ActuatorControl):
        if len(msg.controls) >= 4:
            raw_u0 = float(msg.controls[0])   # roll
            raw_u1 = float(msg.controls[1])   # pitch
            raw_u2 = float(msg.controls[2])   # yaw
            raw_u3 = float(msg.controls[3])   # throttle

            raw = np.array([raw_u0, raw_u1, raw_u2, raw_u3], dtype=float)
            mapped = raw * self.actuator_axis_signs

            self.last_controls[:] = mapped
            self.first_actuator_received = True
            self.last_actuator_msg_wall = time.monotonic()

            rospy.loginfo_throttle(
                0.5,
                "target_actuator_control raw=[%.3f, %.3f, %.3f, %.3f] mapped=[%.3f, %.3f, %.3f, %.3f]" %
                (
                    raw_u0, raw_u1, raw_u2, raw_u3,
                    mapped[0], mapped[1], mapped[2], mapped[3]
                )
            )

    def actuator_age(self):
        if self.last_actuator_msg_wall is None:
            return -1.0
        return time.monotonic() - self.last_actuator_msg_wall

    def actuator_fresh(self):
        age = self.actuator_age()
        return (age >= 0.0) and (age < self.control_timeout_sec)

    def build_odom_msg(self):
        st = self.plant.get_state()
        p = st["p_enu"]

        q = quaternion_from_euler(st["roll"], st["pitch"], st["yaw"])

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()

        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = float(p[0])
        odom.pose.pose.position.y = float(p[1])
        odom.pose.pose.position.z = float(p[2])
        odom.pose.pose.orientation = Quaternion(*q)

        v = st["v_enu"]
        omega = st["omega_flu"]

        odom.twist.twist.linear.x = float(v[0])
        odom.twist.twist.linear.y = float(v[1])
        odom.twist.twist.linear.z = float(v[2])

        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])

        pose_cov = np.zeros((6, 6), dtype=float)
        twist_cov = np.zeros((6, 6), dtype=float)

        pose_cov[0, 0] = 0.02 ** 2
        pose_cov[1, 1] = 0.02 ** 2
        pose_cov[2, 2] = 0.03 ** 2
        pose_cov[3, 3] = math.radians(2.0) ** 2
        pose_cov[4, 4] = math.radians(2.0) ** 2
        pose_cov[5, 5] = math.radians(3.0) ** 2

        twist_cov[0, 0] = 0.05 ** 2
        twist_cov[1, 1] = 0.05 ** 2
        twist_cov[2, 2] = 0.08 ** 2
        twist_cov[3, 3] = math.radians(5.0) ** 2
        twist_cov[4, 4] = math.radians(5.0) ** 2
        twist_cov[5, 5] = math.radians(8.0) ** 2

        odom.pose.covariance = pose_cov.reshape(-1).tolist()
        odom.twist.covariance = twist_cov.reshape(-1).tolist()

        return odom, q

    def publish_debug(self, odom, q):
        self.sim_odom_pub.publish(odom)

        st = self.plant.get_state()
        sf_body_flu = self.plant.get_specific_force_body_flu()
        omega = st["omega_flu"]

        imu = Imu()
        imu.header.stamp = odom.header.stamp
        imu.header.frame_id = "base_link"
        imu.orientation = Quaternion(*q)

        imu.angular_velocity.x = float(omega[0])
        imu.angular_velocity.y = float(omega[1])
        imu.angular_velocity.z = float(omega[2])

        imu.linear_acceleration.x = float(sf_body_flu[0])
        imu.linear_acceleration.y = float(sf_body_flu[1])
        imu.linear_acceleration.z = float(sf_body_flu[2])

        self.sim_imu_pub.publish(imu)

        if self.publish_tf:
            self.tf_br.sendTransform(
                (
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z
                ),
                q,
                odom.header.stamp,
                self.base_frame_id,
                odom.header.frame_id
            )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        nominal_dt = 1.0 / self.rate_hz

        while not rospy.is_shutdown():
            now_wall = time.monotonic()
            dt_wall = now_wall - self.last_wall
            self.last_wall = now_wall
            dt = min(max(dt_wall, 0.5 * nominal_dt), 1.5 * nominal_dt)

            phase = "EKF_WARMUP"

            if (rospy.Time.now() - self.start_time).to_sec() < self.ekf_warmup_sec:
                self.plant.freeze_at_origin()
                self.ctrl_filt[:] = 0.0
            else:
                if not self.flight_started:
                    if self.actuator_fresh() and float(self.last_controls[3]) > self.takeoff_throttle_threshold:
                        self.flight_started = True
                        rospy.loginfo("检测到油门超过起飞门限，进入动态仿真阶段")
                    else:
                        phase = "WAIT_TAKEOFF"
                        self.plant.freeze_at_origin()
                        self.ctrl_filt[:] = 0.0

                if self.flight_started:
                    phase = "DYNAMIC"
                    if self.actuator_fresh():
                        controls_cmd = self.last_controls.copy()
                    else:
                        controls_cmd = np.zeros(4, dtype=float)
                        rospy.logwarn_throttle(1.0, "执行器控制超时，动态阶段回到零控制")

                    self.ctrl_filt = self.ctrl_alpha * self.ctrl_filt + (1.0 - self.ctrl_alpha) * controls_cmd

                    self.plant.set_actuator_controls(self.ctrl_filt)
                    self.plant.step(dt)

            odom, q = self.build_odom_msg()
            if self.mavros_odom_pub is not None:
                self.mavros_odom_pub.publish(odom)
            self.publish_debug(odom, q)

            rospy.loginfo_throttle(
                1.0,
                "phase=%s actuator_age=%.3f throttle_raw=%.3f throttle_filt=%.3f pos_enu=(%.2f, %.2f, %.2f)" %
                (
                    phase,
                    self.actuator_age(),
                    float(self.last_controls[3]),
                    float(self.ctrl_filt[3]),
                    self.plant.p_enu[0],
                    self.plant.p_enu[1],
                    self.plant.p_enu[2]
                )
            )

            rate.sleep()


if __name__ == "__main__":
    try:
        node = SimBridgeOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass

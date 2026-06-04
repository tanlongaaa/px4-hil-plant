#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import tf

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped
from mavros_msgs.msg import ActuatorControl
from tf.transformations import quaternion_from_euler


class PX46DOFNode:
    def __init__(self):
        rospy.init_node("px4_6dof_node")

        # =========================
        # 基本参数
        # =========================
        self.rate_hz = rospy.get_param("~rate_hz", 200.0)
        self.dt = 1.0 / self.rate_hz

        self.m = rospy.get_param("~mass", 1.5)
        self.g = rospy.get_param("~gravity", 9.81)

        self.Jx = rospy.get_param("~Jx", 0.02)
        self.Jy = rospy.get_param("~Jy", 0.02)
        self.Jz = rospy.get_param("~Jz", 0.04)
        self.J = np.diag([self.Jx, self.Jy, self.Jz])
        self.J_inv = np.linalg.inv(self.J)

        self.arm_length = rospy.get_param("~arm_length", 0.25)
        self.k_thrust = rospy.get_param("~k_thrust", 9.2)

        self.k_tau_roll = rospy.get_param("~k_tau_roll", 0.03)
        self.k_tau_pitch = rospy.get_param("~k_tau_pitch", 0.03)
        self.k_tau_yaw = rospy.get_param("~k_tau_yaw", 0.01)

        self.kv = rospy.get_param("~linear_damping", 1.5)
        self.kw = rospy.get_param("~angular_damping", 0.35)

        # PX4 推力解释
        self.use_px4_throttle = rospy.get_param("~use_px4_throttle", True)
        self.hover_throttle = rospy.get_param("~hover_throttle", 0.40)
        self.throttle_blend = rospy.get_param("~throttle_blend", 1.0)

        self.hover_throttle = rospy.get_param("~hover_throttle", 0.40)
        self.throttle_scale = rospy.get_param("~throttle_scale", 0.15)   # 先很小
        self.throttle_filt = self.hover_throttle

        self.tau_scale_air = rospy.get_param("~tau_scale_air", 0.15)
        self.tau_scale_ground = rospy.get_param("~tau_scale_ground", 0.0)
        # EKF 预热时间：前几秒冻结状态，只喂稳定视觉位置
        self.ekf_warmup_sec = rospy.get_param("~ekf_warmup_sec", 3.0)
        self.start_time = rospy.Time.now()

        # 扰动
        self.wind_force_x = rospy.get_param("~wind_force_x", 0.0)
        self.wind_force_y = rospy.get_param("~wind_force_y", 0.0)
        self.wind_force_z = rospy.get_param("~wind_force_z", 0.0)
        self.wind_sine_amp = rospy.get_param("~wind_sine_amp", 0.0)
        self.wind_sine_freq = rospy.get_param("~wind_sine_freq", 0.5)

        # =========================
        # 状态
        # =========================
        self.p = np.array([0.0, 0.0, 0.0], dtype=float)
        self.v = np.zeros(3, dtype=float)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.omega = np.zeros(3, dtype=float)
        self.acc_world = np.zeros(3, dtype=float)

        # PX4 actuator 输出 [roll, pitch, yaw, throttle]
        self.u = np.zeros(4, dtype=float)

        # =========================
        # 发布器
        # =========================
        self.imu_pub = rospy.Publisher("/mavros/imu/data_raw", Imu, queue_size=10)
        self.vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/sim/odom", Odometry, queue_size=10)
        self.mavros_odom_pub = rospy.Publisher("/mavros/odometry/in", Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

        # =========================
        # 订阅器
        # =========================
        rospy.Subscriber(
            "/mavros/target_actuator_control",
            ActuatorControl,
            self.actuator_callback,
            queue_size=1
        )

        rospy.loginfo("PX4 6DOF node started (PX4 PID mode with EKF warmup).")

    def publish_mavros_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.v[0]
        odom.twist.twist.linear.y = self.v[1]
        odom.twist.twist.linear.z = self.v[2]

        odom.twist.twist.angular.x = self.omega[0]
        odom.twist.twist.angular.y = self.omega[1]
        odom.twist.twist.angular.z = self.omega[2]

        self.mavros_odom_pub.publish(odom)

    @staticmethod
    def clamp(x, xmin, xmax):
        return max(xmin, min(xmax, x))

    def actuator_callback(self, msg: ActuatorControl):
        if len(msg.controls) >= 4:
            self.u[0] = float(msg.controls[0])
            self.u[1] = float(msg.controls[1])
            self.u[2] = float(msg.controls[2])
            self.u[3] = float(msg.controls[3])

        rospy.loginfo_throttle(
            0.5,
            "u=[%.3f, %.3f, %.3f, %.3f]" % (self.u[0], self.u[1], self.u[2], self.u[3])
        )

    def rotation_matrix(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        Rz = np.array([[cy, -sy, 0],
                       [sy,  cy, 0],
                       [0,    0, 1]], dtype=float)

        Ry = np.array([[cp, 0, sp],
                       [0,  1, 0],
                       [-sp, 0, cp]], dtype=float)

        Rx = np.array([[1, 0,  0],
                       [0, cr, -sr],
                       [0, sr,  cr]], dtype=float)

        return Rz @ Ry @ Rx

    def euler_kinematics(self, roll, pitch, omega):
        p, q, r = omega
        cr = math.cos(roll)
        sr = math.sin(roll)
        ct = math.cos(pitch)

        if abs(ct) < 1e-4:
            ct = 1e-4 if ct >= 0 else -1e-4

        tt = math.tan(pitch)

        roll_dot = p + sr * tt * q + cr * tt * r
        pitch_dot = cr * q - sr * r
        yaw_dot = (sr / ct) * q + (cr / ct) * r

        return np.array([roll_dot, pitch_dot, yaw_dot], dtype=float)

    def motor_to_force_torque(self):
        roll_ctrl = float(self.u[0])
        pitch_ctrl = float(self.u[1])
        yaw_ctrl = float(self.u[2])

        # PX4 actuator 第4维先按 [0,1] 解释
        px4_throttle = self.clamp(float(self.u[3]), 0.0, 1.0)

        # 关键：不要直接把 PX4 throttle 原样喂给 Plant
        # 只把它当成“围绕 hover_throttle 的修正量”
        throttle_cmd = self.hover_throttle + self.throttle_scale * (px4_throttle - self.hover_throttle)
        throttle_cmd = self.clamp(throttle_cmd, 0.0, 1.0)

        # 再做一阶滤波，避免一下子冲爆
        alpha_t = 0.95
        self.throttle_filt = alpha_t * self.throttle_filt + (1.0 - alpha_t) * throttle_cmd
        throttle = self.throttle_filt

        T = self.k_thrust * 4.0 * throttle

        # 控制量限幅
        roll_ctrl = self.clamp(roll_ctrl, -0.5, 0.5)
        pitch_ctrl = self.clamp(pitch_ctrl, -0.5, 0.5)
        yaw_ctrl = self.clamp(yaw_ctrl, -0.5, 0.5)

        # 地面阶段完全不给姿态力矩；离地后也只给很小一部分
        if self.p[2] <= 0.05:
            tau_scale = self.tau_scale_ground
        else:
            tau_scale = self.tau_scale_air

        tau_x = tau_scale * self.k_tau_roll * roll_ctrl
        tau_y = tau_scale * self.k_tau_pitch * pitch_ctrl
        tau_z = tau_scale * self.k_tau_yaw * yaw_ctrl

        tau = np.array([tau_x, tau_y, tau_z], dtype=float)

        rospy.loginfo_throttle(
            0.5,
            "thr_px4=%.3f thr_use=%.3f T=%.3f tau=[%.3f, %.3f, %.3f] z=%.3f vz=%.3f" %
            (px4_throttle, throttle, T, tau_x, tau_y, tau_z, self.p[2], self.v[2])
        )

        return T, tau

    def disturbance_force_world(self):
        t = rospy.get_time()
        fx = self.wind_force_x + self.wind_sine_amp * math.sin(2.0 * math.pi * self.wind_sine_freq * t)
        fy = self.wind_force_y
        fz = self.wind_force_z
        return np.array([fx, fy, fz], dtype=float)

    def step(self):
        # EKF 预热：前几秒冻结状态，只发稳定视觉位置
        warmup = (rospy.Time.now() - self.start_time).to_sec() < self.ekf_warmup_sec
        if warmup:
            self.p[:] = 0.0
            self.v[:] = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.omega[:] = 0.0
            self.acc_world[:] = 0.0

            rospy.loginfo_throttle(
                0.5,
                "EKF warmup... holding stable pose at origin"
            )
            return

        T, tau = self.motor_to_force_torque()

        R = self.rotation_matrix(self.roll, self.pitch, self.yaw)

        f_thrust_world = R @ np.array([0.0, 0.0, T], dtype=float)
        f_gravity_world = np.array([0.0, 0.0, -self.m * self.g], dtype=float)
        f_damping_world = -self.kv * self.v
        f_disturb_world = self.disturbance_force_world()

        total_force_world = f_thrust_world + f_gravity_world + f_damping_world + f_disturb_world
        self.acc_world = total_force_world / self.m

        self.v = self.v + self.acc_world * self.dt
        self.p = self.p + self.v * self.dt

        Jw = self.J @ self.omega
        coriolis = np.cross(self.omega, Jw)
        tau_damping = self.kw * self.omega
        omega_dot = self.J_inv @ (tau - coriolis - tau_damping)
        self.omega = self.omega + omega_dot * self.dt

        self.omega[0] = self.clamp(self.omega[0], -5.0, 5.0)
        self.omega[1] = self.clamp(self.omega[1], -5.0, 5.0)
        self.omega[2] = self.clamp(self.omega[2], -3.0, 3.0)

        euler_dot = self.euler_kinematics(self.roll, self.pitch, self.omega)
        self.roll += euler_dot[0] * self.dt
        self.pitch += euler_dot[1] * self.dt
        self.yaw += euler_dot[2] * self.dt

        # 防止姿态一下子翻掉
        self.roll = self.clamp(self.roll, -0.8, 0.8)
        self.pitch = self.clamp(self.pitch, -0.8, 0.8)

        # 地面接触：只做最小必要约束
        if self.p[2] < 0.0:
            self.p[2] = 0.0
            if self.v[2] < 0.0:
                self.v[2] = 0.0

            self.v[0] *= 0.8
            self.v[1] *= 0.8
            self.omega *= 0.8

        rospy.loginfo_throttle(
            0.5,
            "acc_z=%.3f z=%.3f vz=%.3f roll=%.3f pitch=%.3f" %
            (self.acc_world[2], self.p[2], self.v[2], self.roll, self.pitch)
        )

    def publish_vision_pose(self):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"

        ps.pose.position.x = self.p[0]
        ps.pose.position.y = self.p[1]
        ps.pose.position.z = self.p[2]

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        ps.pose.orientation = Quaternion(*q)

        self.vision_pose_pub.publish(ps)

    def publish_imu(self):
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "base_link"

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu.orientation = Quaternion(*q)

        imu.angular_velocity.x = self.omega[0]
        imu.angular_velocity.y = self.omega[1]
        imu.angular_velocity.z = self.omega[2]

        R = self.rotation_matrix(self.roll, self.pitch, self.yaw)
        g_world = np.array([0.0, 0.0, -self.g], dtype=float)
        specific_force_world = self.acc_world - g_world
        specific_force_body = R.T @ specific_force_world

        imu.linear_acceleration.x = specific_force_body[0]
        imu.linear_acceleration.y = specific_force_body[1]
        imu.linear_acceleration.z = specific_force_body[2]

        self.imu_pub.publish(imu)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.v[0]
        odom.twist.twist.linear.y = self.v[1]
        odom.twist.twist.linear.z = self.v[2]

        odom.twist.twist.angular.x = self.omega[0]
        odom.twist.twist.angular.y = self.omega[1]
        odom.twist.twist.angular.z = self.omega[2]

        self.odom_pub.publish(odom)

        self.tf_br.sendTransform(
            (self.p[0], self.p[1], self.p[2]),
            q,
            rospy.Time.now(),
            "base_link",
            "world"
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            self.publish_vision_pose()
            self.publish_imu()
            self.publish_odom()
            self.publish_mavros_odometry()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = PX46DOFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
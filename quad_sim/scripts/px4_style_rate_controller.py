#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion


class PX4StyleRateController:
    def __init__(self):
        rospy.init_node("px4_style_rate_controller")

        # =========================
        # 目标
        # =========================
        self.z_ref = rospy.get_param("~z_ref", 1.5)
        self.roll_ref = rospy.get_param("~roll_ref", 0.0)
        self.pitch_ref = rospy.get_param("~pitch_ref", 0.0)
        self.yaw_ref = rospy.get_param("~yaw_ref", 0.0)

        # =========================
        # 机体参数
        # =========================
        self.m = rospy.get_param("~mass", 1.0)
        self.g = rospy.get_param("~gravity", 9.81)

        # =========================
        # 高度外环参数
        # T = mg + kpz * ez + kdz * evz
        # =========================
        self.kpz = rospy.get_param("~kpz", 3.0)
        self.kdz = rospy.get_param("~kdz", 3.5)

        # =========================
        # 姿态外环（姿态角 -> 角速度期望）
        # p_sp = kphi * (phi_ref - phi)
        # q_sp = ktheta * (theta_ref - theta)
        # r_sp = kpsi * (psi_ref - psi)
        # =========================
        self.kphi = rospy.get_param("~kphi", 2.5)
        self.ktheta = rospy.get_param("~ktheta", 2.5)
        self.kpsi = rospy.get_param("~kpsi", 1.5)

        # 角速度期望限幅
        self.p_sp_max = rospy.get_param("~p_sp_max", 1.5)   # rad/s
        self.q_sp_max = rospy.get_param("~q_sp_max", 1.5)
        self.r_sp_max = rospy.get_param("~r_sp_max", 1.0)

        # =========================
        # 角速度 PID 内环
        # tau = Kp*e + Ki*int(e) + Kd*de/dt
        # =========================
        self.kp_p = rospy.get_param("~kp_p", 0.12)
        self.ki_p = rospy.get_param("~ki_p", 0.02)
        self.kd_p = rospy.get_param("~kd_p", 0.01)

        self.kp_q = rospy.get_param("~kp_q", 0.12)
        self.ki_q = rospy.get_param("~ki_q", 0.02)
        self.kd_q = rospy.get_param("~kd_q", 0.01)

        self.kp_r = rospy.get_param("~kp_r", 0.08)
        self.ki_r = rospy.get_param("~ki_r", 0.01)
        self.kd_r = rospy.get_param("~kd_r", 0.005)

        # 推力和力矩限幅
        self.T_min = rospy.get_param("~T_min", 0.0)
        self.T_max = rospy.get_param("~T_max", 18.0)

        self.tau_x_max = rospy.get_param("~tau_x_max", 0.15)
        self.tau_y_max = rospy.get_param("~tau_y_max", 0.15)
        self.tau_z_max = rospy.get_param("~tau_z_max", 0.08)

        # PID积分限幅
        self.int_p_max = rospy.get_param("~int_p_max", 0.3)
        self.int_q_max = rospy.get_param("~int_q_max", 0.3)
        self.int_r_max = rospy.get_param("~int_r_max", 0.2)

        # 状态缓存
        self.last_time = None
        self.int_p = 0.0
        self.int_q = 0.0
        self.int_r = 0.0

        self.last_e_p = 0.0
        self.last_e_q = 0.0
        self.last_e_r = 0.0

        # 发布/订阅
        self.cmd_pub = rospy.Publisher("/quad_cmd", Float32MultiArray, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        self.t0 = rospy.Time.now().to_sec()
        self.ready_to_land = False
        rospy.loginfo("PX4-style rate controller started.")

    @staticmethod
    def clamp(value, vmin, vmax):
        return max(vmin, min(value, vmax))

    @staticmethod
    def wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    def reference_generator(self, t):
        circle_center_x = 2.0
        circle_center_y = 0.0
        circle_r = 1.0
        circle_w = 0.5

        circle_duration = 2.0 * math.pi / circle_w

        t_takeoff_end = 5.0
        t_ab_end = 12.0
        t_circle_end = t_ab_end + circle_duration
        t_hover_end = t_circle_end + 4.0
        t_land_end = t_hover_end + 8.0

        if t < t_takeoff_end:
            x_ref = 0.0
            y_ref = 0.0
            z_ref = 1.5
            yaw_ref = 0.0
            phase = "takeoff"

        elif t < t_ab_end:
            s = (t - t_takeoff_end) / (t_ab_end - t_takeoff_end)
            s = min(max(s, 0.0), 1.0)

            x_ref = 2.0 * s
            y_ref = 0.0
            z_ref = 1.5
            yaw_ref = 0.0
            phase = "line"

        elif t < t_circle_end:
            tc = t - t_ab_end

            x_ref = circle_center_x + circle_r * math.cos(circle_w * tc)
            y_ref = circle_center_y + circle_r * math.sin(circle_w * tc)
            z_ref = 1.5
            yaw_ref = 0.0
            phase = "circle"

        elif t < t_hover_end:
            x_ref = circle_center_x + circle_r
            y_ref = circle_center_y
            z_ref = 1.5
            yaw_ref = 0.0
            phase = "hover_before_land"

        elif t < t_land_end:
            s = (t - t_hover_end) / (t_land_end - t_hover_end)

            x_ref = circle_center_x + circle_r
            y_ref = circle_center_y
            z_ref = 1.5 * (1.0 - s)
            yaw_ref = 0.0
            phase = "land"

        else:
            x_ref = circle_center_x + circle_r
            y_ref = circle_center_y
            z_ref = 0.0
            yaw_ref = 0.0
            phase = "landed"

        return x_ref, y_ref, z_ref, yaw_ref, phase
    def odom_callback(self, msg: Odometry):
        now = rospy.Time.now().to_sec()

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        if dt <= 1e-4:
            return
        self.last_time = now
        # =========================
        # 生成参考轨迹
        # =========================
        t = rospy.Time.now().to_sec() - self.t0
        x_ref, y_ref, z_ref, yaw_ref,phase = self.reference_generator(t)
        # =========================
        # 从 /odom 取状态
        # =========================
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        z = msg.pose.pose.position.z
        vz = msg.twist.twist.linear.z

        q_msg = msg.pose.pose.orientation
        quat = [q_msg.x, q_msg.y, q_msg.z, q_msg.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        # 机体系角速度
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        # =========================
        # 1) 高度外环 -> T
        # =========================
        # =========================
        # 判断是否准备降落
        # =========================
        if phase == "hover_before_land":
            ex = x_ref - x
            ey = y_ref - y

            if abs(ex) < 0.05 and abs(ey) < 0.05 and abs(vx) < 0.10 and abs(vy) < 0.10:
                self.ready_to_land = True
        # =========================
        # XY位置外环 → roll/pitch
        # =========================
        if phase == "landed":
            roll_ref = 0.0
            pitch_ref = 0.0

        elif phase == "land":
            if self.ready_to_land:
                # 已经在悬停点稳定住了，开始垂直下降
                roll_ref = 0.0
                pitch_ref = 0.0
            else:
                # 还没停稳，先主动刹车
                roll_ref = -0.3 * vy
                pitch_ref = 0.3 * vx

        else:
            ex = x_ref - x
            ey = y_ref - y

            evx = -1.5 * vx
            evy = -1.5 * vy

            kx = 0.8
            ky = 0.8
            kdx = 0.5
            kdy = 0.5

            pitch_ref = kx * ex + kdx * evx
            roll_ref = -(ky * ey + kdy * evy)

            roll_ref = self.clamp(roll_ref, -0.3, 0.3)
            pitch_ref = self.clamp(pitch_ref, -0.3, 0.3)
        ez = z_ref - z
        evz = -vz
        T = self.m * self.g + self.kpz * ez + self.kdz * evz
        T = self.clamp(T, self.T_min, self.T_max)

        # =========================
        # 2) 姿态外环 -> 角速度期望
        # =========================
        e_roll = self.wrap_angle(roll_ref - roll)
        e_pitch = self.wrap_angle(pitch_ref - pitch)
        e_yaw = self.wrap_angle(yaw_ref - yaw)

        p_sp = self.kphi * e_roll
        q_sp = self.ktheta * e_pitch
        r_sp = self.kpsi * e_yaw

        p_sp = self.clamp(p_sp, -self.p_sp_max, self.p_sp_max)
        q_sp = self.clamp(q_sp, -self.q_sp_max, self.q_sp_max)
        r_sp = self.clamp(r_sp, -self.r_sp_max, self.r_sp_max)

        # =========================
        # 3) 角速度 PID 内环
        # =========================
        e_p = p_sp - p
        e_q = q_sp - q
        e_r = r_sp - r

        # 积分
        self.int_p += e_p * dt
        self.int_q += e_q * dt
        self.int_r += e_r * dt

        self.int_p = self.clamp(self.int_p, -self.int_p_max, self.int_p_max)
        self.int_q = self.clamp(self.int_q, -self.int_q_max, self.int_q_max)
        self.int_r = self.clamp(self.int_r, -self.int_r_max, self.int_r_max)

        # 微分
        de_p = (e_p - self.last_e_p) / dt
        de_q = (e_q - self.last_e_q) / dt
        de_r = (e_r - self.last_e_r) / dt

        self.last_e_p = e_p
        self.last_e_q = e_q
        self.last_e_r = e_r

        tau_x = self.kp_p * e_p + self.ki_p * self.int_p + self.kd_p * de_p
        tau_y = self.kp_q * e_q + self.ki_q * self.int_q + self.kd_q * de_q
        tau_z = self.kp_r * e_r + self.ki_r * self.int_r + self.kd_r * de_r

        tau_x = self.clamp(tau_x, -self.tau_x_max, self.tau_x_max)
        tau_y = self.clamp(tau_y, -self.tau_y_max, self.tau_y_max)
        tau_z = self.clamp(tau_z, -self.tau_z_max, self.tau_z_max)
        # 简单落地停机逻辑
        if z < 0.05 and phase =="landed":
            T = 0.0
            tau_x = 0.0
            tau_y = 0.0
            tau_z = 0.0
        # 发布控制量
        cmd = Float32MultiArray()
        cmd.data = [T, tau_x, tau_y, tau_z]
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            1.0,
            f"z={z:.2f}, T={T:.2f}, p_sp={p_sp:.2f}, q_sp={q_sp:.2f}, r_sp={r_sp:.2f}, "
            f"tau=[{tau_x:.3f}, {tau_y:.3f}, {tau_z:.3f}]"
        )


if __name__ == "__main__":
    try:
        PX4StyleRateController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
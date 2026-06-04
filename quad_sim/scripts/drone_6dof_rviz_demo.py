#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最小可运行版：6DOF 四旋翼刚体模型 + ROS + RViz 可视化

功能：
1. 自己在 ROS 节点中实现四旋翼 6DOF 刚体动力学
2. 发布 /odom 和 /tf，直接在 RViz 中显示运动
3. 内置一个非常简单的“演示控制输入”模式：
   - 先悬停
   - 再做小幅滚转/俯仰/偏航变化
4. 不依赖 PX4，不依赖 AirSim，适合今晚汇报演示架构

适用环境：
- ROS1（建议 Noetic）
- Python 3
- Ubuntu

运行前请确认：
sudo apt install ros-noetic-tf ros-noetic-nav-msgs ros-noetic-geometry-msgs

运行：
1) roscore
2) chmod +x drone_6dof_rviz_demo.py
3) rosrun <你的包名> drone_6dof_rviz_demo.py
4) rosrun rviz rviz
5) RViz 中设置 Fixed Frame = world
6) 添加：Odometry（话题 /odom）、TF、Axes（可选）

"""

import math
import numpy as np
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class Quadrotor6DOFNode:
    def __init__(self):
        rospy.init_node("quadrotor_6dof_demo")

        # =========================
        # 参数区
        # =========================
        self.rate_hz = rospy.get_param("~rate_hz", 100.0)
        self.dt = 1.0 / self.rate_hz

        # 机体参数（一个典型小四旋翼量级）
        self.m = rospy.get_param("~mass", 1.0)              # kg
        self.g = rospy.get_param("~gravity", 9.81)          # m/s^2
        self.Jx = rospy.get_param("~Jx", 0.02)              # kg*m^2
        self.Jy = rospy.get_param("~Jy", 0.02)
        self.Jz = rospy.get_param("~Jz", 0.04)
        self.J = np.diag([self.Jx, self.Jy, self.Jz])
        self.J_inv = np.linalg.inv(self.J)

        # 阻尼项
        self.kv = rospy.get_param("~linear_damping", 0.8)
        self.kw = rospy.get_param("~angular_damping", 0.3)

        # 初始状态
        self.p = np.array([0.0, 0.0, 0.0], dtype=float)      # 世界系位置
        self.v = np.array([0.0, 0.0, 0.0], dtype=float)      # 世界系速度

        # 用四元数表示姿态： [x, y, z, w]
        self.q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        # 机体系角速度 [p, q, r]
        self.omega = np.array([0.0, 0.0, 0.0], dtype=float)

        # 发布器与 TF
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

                # 外部控制输入: [T, tau_x, tau_y, tau_z]
        self.cmd = np.array([self.m * self.g, 0.0, 0.0, 0.0], dtype=float)

        self.cmd_sub = rospy.Subscriber(
            "/quad_cmd",
            Float32MultiArray,
            self.cmd_callback,
            queue_size=1
        )
        # 时间记录
        self.t0 = rospy.Time.now().to_sec()
    def cmd_callback(self, msg):
        if len(msg.data) >= 4:
            self.cmd[0] = float(msg.data[0])
            self.cmd[1] = float(msg.data[1])
            self.cmd[2] = float(msg.data[2])
            self.cmd[3] = float(msg.data[3])

        rospy.loginfo("Quadrotor 6DOF demo node started.")

    # =========================
    # 数学工具函数
    # =========================
    @staticmethod
    def quat_normalize(q):
        norm = np.linalg.norm(q)
        if norm < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return q / norm

    @staticmethod
    def quat_to_rotmat(q):
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ], dtype=float)
        return R

    @staticmethod
    def omega_matrix(omega):
        p, q, r = omega
        return np.array([
            [0.0, -p,  -q,  -r],
            [p,   0.0,  r,  -q],
            [q,  -r,   0.0,  p],
            [r,   q,  -p,   0.0]
        ], dtype=float)

    @staticmethod
    def wrap_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # =========================
    # 演示用控制输入
    # 这里只是为了让无人机“动起来”
    # 后续可替换成 PX4 / 你自己的控制器输出
    # =========================
    def demo_input(self, t):
        T_hover = self.m * self.g

        if t < 3.0:
            T = T_hover
            tau = np.array([0.0, 0.0, 0.0], dtype=float)
        elif t < 6.0:
            T = T_hover + 0.8
            tau = np.array([0.0, 0.0, 0.0], dtype=float)
        elif t < 9.0:
            T = T_hover - 0.6
            tau = np.array([0.0, 0.0, 0.0], dtype=float)
        else:
            T = T_hover
            tau = np.array([0.0, 0.0, 0.0], dtype=float)

        return T, tau

    # =========================
    # 动力学更新
    # =========================
    def step_dynamics(self, T, tau):
        # 当前姿态矩阵：机体系 -> 世界系
        R = self.quat_to_rotmat(self.q)

        # 总推力在机体系中沿 z 轴正方向
        # 转到世界系：f_T^w = R * [0, 0, T]^T
        f_thrust_world = R.dot(np.array([0.0, 0.0, T], dtype=float))

        # 世界系重力
        f_gravity_world = np.array([0.0, 0.0, -self.m * self.g], dtype=float)

        # 简单线阻尼
        f_damping_world = -self.kv * self.v

        # 平动动力学
        # m * v_dot = f_gravity + f_thrust + f_damping
        a = (f_gravity_world + f_thrust_world + f_damping_world) / self.m
        self.v = self.v + a * self.dt
        self.p = self.p + self.v * self.dt

        # 转动动力学
        # J * omega_dot = tau - omega x (J omega) - kw * omega
        Jw = self.J.dot(self.omega)
        coriolis = np.cross(self.omega, Jw)
        tau_damping = self.kw * self.omega
        omega_dot = self.J_inv.dot(tau - coriolis - tau_damping)
        self.omega = self.omega + omega_dot * self.dt

        # 四元数更新：q_dot = 1/2 * Omega(omega) * q
        x, y, z, w = self.q
        q_wxyz = np.array([w, x, y, z], dtype=float)

        p_, q_, r_ = self.omega
        Omega = np.array([
            [0.0, -p_, -q_, -r_],
            [p_,  0.0,  r_, -q_],
            [q_, -r_,  0.0,  p_],
            [r_,  q_, -p_,  0.0]
        ], dtype=float)

        qdot_wxyz = 0.5 * Omega.dot(q_wxyz)
        q_wxyz = q_wxyz + qdot_wxyz * self.dt
        q_wxyz = q_wxyz / np.linalg.norm(q_wxyz)

        # 转回 ROS 常用 [x, y, z, w]
        self.q = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)
        self.q = self.quat_normalize(self.q)

        # 地面简单约束，防止一直掉下去
        if self.p[2] < 0.0:
            self.p[2] = 0.0
            if self.v[2] < 0.0:
                self.v[2] = 0.0

    # =========================
    # 发布 Odom 和 TF
    # =========================
    def publish_state(self):
        now = rospy.Time.now()

        # /tf: world -> base_link
        self.tf_br.sendTransform(
            (self.p[0], self.p[1], self.p[2]),
            (self.q[0], self.q[1], self.q[2], self.q[3]),
            now,
            "base_link",
            "world"
        )

        # /odom
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]

        odom.pose.pose.orientation.x = self.q[0]
        odom.pose.pose.orientation.y = self.q[1]
        odom.pose.pose.orientation.z = self.q[2]
        odom.pose.pose.orientation.w = self.q[3]

        odom.twist.twist.linear.x = self.v[0]
        odom.twist.twist.linear.y = self.v[1]
        odom.twist.twist.linear.z = self.v[2]

        odom.twist.twist.angular.x = self.omega[0]
        odom.twist.twist.angular.y = self.omega[1]
        odom.twist.twist.angular.z = self.omega[2]

        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            T = self.cmd[0]
            tau = self.cmd[1:4].copy()
            self.step_dynamics(T, tau)
            self.publish_state()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = Quadrotor6DOFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

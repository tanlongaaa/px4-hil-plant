#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion


class SimpleHoverController:
    def __init__(self):
        rospy.init_node("simple_hover_controller")

        # 目标
        self.z_ref = rospy.get_param("~z_ref", 1.5)
        self.yaw_ref = rospy.get_param("~yaw_ref", 0.0)

        # 参数
        self.m = rospy.get_param("~mass", 1.0)
        self.g = rospy.get_param("~gravity", 9.81)

        # 高度 PD
        self.kpz = rospy.get_param("~kpz", 3.0)
        self.kdz = rospy.get_param("~kdz", 3.5)

        # 姿态稳定 PD
        self.kp_roll = rospy.get_param("~kp_roll", 0.20)
        self.kd_roll = rospy.get_param("~kd_roll", 0.05)

        self.kp_pitch = rospy.get_param("~kp_pitch", 0.20)
        self.kd_pitch = rospy.get_param("~kd_pitch", 0.05)

        self.kp_yaw = rospy.get_param("~kp_yaw", 0.12)
        self.kd_yaw = rospy.get_param("~kd_yaw", 0.03)

        # 限幅
        self.T_min = rospy.get_param("~T_min", 0.0)
        self.T_max = rospy.get_param("~T_max", 18.0)

        self.tau_max_roll = rospy.get_param("~tau_max_roll", 0.12)
        self.tau_max_pitch = rospy.get_param("~tau_max_pitch", 0.12)
        self.tau_max_yaw = rospy.get_param("~tau_max_yaw", 0.08)

        self.cmd_pub = rospy.Publisher("/quad_cmd", Float32MultiArray, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

        rospy.loginfo("Simple hover controller started.")

    @staticmethod
    def clamp(value, vmin, vmax):
        return max(vmin, min(value, vmax))

    @staticmethod
    def wrap_angle(angle):
        while angle > 3.1415926:
            angle -= 2.0 * 3.1415926
        while angle < -3.1415926:
            angle += 2.0 * 3.1415926
        return angle

    def odom_callback(self, msg: Odometry):
        # 状态
        z = msg.pose.pose.position.z
        vz = msg.twist.twist.linear.z

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        p = msg.twist.twist.angular.x
        q_rate = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        # 1) 高度控制
        ez = self.z_ref - z
        evz = -vz
        T = self.m * self.g + self.kpz * ez + self.kdz * evz
        T = self.clamp(T, self.T_min, self.T_max)

        # 2) 姿态稳定（目标 roll/pitch=0, yaw=yaw_ref）
        yaw_err = self.wrap_angle(self.yaw_ref - yaw)

        tau_roll = -self.kp_roll * roll - self.kd_roll * p
        tau_pitch = -self.kp_pitch * pitch - self.kd_pitch * q_rate
        tau_yaw = self.kp_yaw * yaw_err - self.kd_yaw * r

        tau_roll = self.clamp(tau_roll, -self.tau_max_roll, self.tau_max_roll)
        tau_pitch = self.clamp(tau_pitch, -self.tau_max_pitch, self.tau_max_pitch)
        tau_yaw = self.clamp(tau_yaw, -self.tau_max_yaw, self.tau_max_yaw)

        cmd = Float32MultiArray()
        cmd.data = [T, tau_roll, tau_pitch, tau_yaw]
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            1.0,
            f"z={z:.2f}, vz={vz:.2f}, T={T:.2f}, tau=[{tau_roll:.3f}, {tau_pitch:.3f}, {tau_yaw:.3f}]"
        )


if __name__ == "__main__":
    try:
        SimpleHoverController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
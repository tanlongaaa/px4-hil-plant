#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import ActuatorControl


class PX4Bridge:
    def __init__(self):
        rospy.init_node("px4_bridge")

        # 发布给你的6DOF模型
        self.pub = rospy.Publisher("/quad_cmd", Float32MultiArray, queue_size=1)

        # 订阅PX4输出
        rospy.Subscriber("/mavros/actuator_control",
                         ActuatorControl,
                         self.callback,
                         queue_size=1)

        # 参数（可以调）
        self.kT = 10.0   # 推力系数
        self.kR = 1.0    # 滚转/俯仰
        self.kY = 0.5    # 偏航

        rospy.loginfo("PX4 bridge started")

    def callback(self, msg):
        u = msg.controls  # 长度8，但前4个是电机

        u1, u2, u3, u4 = u[0], u[1], u[2], u[3]
        print("PX4:", u1, u2, u3, u4)
        # 转换
        T = self.kT * (u1 + u2 + u3 + u4)
        tau_x = self.kR * (u2 - u4)
        tau_y = self.kR * (u3 - u1)
        tau_z = self.kY * (u1 - u2 + u3 - u4)

        cmd = Float32MultiArray()
        cmd.data = [T, tau_x, tau_y, tau_z]

        self.pub.publish(cmd)
#print("PX4:", u1, u2, u3, u4)

if __name__ == "__main__":
    PX4Bridge()
    rospy.spin()
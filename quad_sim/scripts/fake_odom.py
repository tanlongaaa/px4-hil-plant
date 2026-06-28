#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_odom.py — 模拟无人机航迹，发布 /sim/odom 供 wind_visualizer 测试

航迹: 水平 8 字 + 高度正弦振荡
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, Twist

if __name__ == '__main__':
    rospy.init_node('fake_odom_pub')
    pub = rospy.Publisher('/sim/odom', Odometry, queue_size=10)
    rate = rospy.Rate(50)

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        # 8字轨迹
        x = 5.0 * np.sin(t * 0.2)
        y = 3.0 * np.sin(t * 0.4)
        z = 2.5 + 0.5 * np.sin(t * 0.15)

        vx = 1.0 * np.cos(t * 0.2)
        vy = 1.2 * np.cos(t * 0.4)
        vz = 0.075 * np.cos(t * 0.15)

        msg.pose.pose.position = Point(x, y, z)
        msg.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        msg.twist.twist.linear = Vector3(vx, vy, vz)
        msg.twist.twist.angular = Vector3(0, 0, 0)

        pub.publish(msg)
        rate.sleep()

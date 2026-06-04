#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray

def main():
    rospy.init_node("test_cmd_pub")

    pub = rospy.Publisher("/quad_cmd", Float32MultiArray, queue_size=1)

    rospy.sleep(1.0)  # ⭐关键：等ROS建立连接

    rate = rospy.Rate(30)

    m = 1.0
    g = 9.81
    T_hover = m * g

    t0 = rospy.Time.now().to_sec()

    rospy.loginfo("test_cmd_pub running...")

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        msg = Float32MultiArray()

        if t < 3.0:
            msg.data = [T_hover, 0.0, 0.0, 0.0]
        elif t < 6.0:
            msg.data = [T_hover + 0.3, 0.0, 0.0, 0.0]
        elif t < 9.0:
            msg.data = [T_hover - 0.3, 0.0, 0.0, 0.0]
        else:
            msg.data = [T_hover, 0.0, 0.0, 0.0]

        pub.publish(msg)
        print("Publishing:", msg.data)  # ⭐强制打印
        rate.sleep()

if __name__ == "__main__":
    main()
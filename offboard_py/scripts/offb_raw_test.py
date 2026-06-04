#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("offb_raw_final")

    rospy.Subscriber("/mavros/state", State, state_cb)

    pub = rospy.Publisher(
        "/mavros/setpoint_raw/local",
        PositionTarget,
        queue_size=10
    )

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")

    arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    sp = PositionTarget()
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    # ⭐⭐⭐关键：只用位置控制⭐⭐⭐
    sp.type_mask = (
        PositionTarget.IGNORE_VX |
        PositionTarget.IGNORE_VY |
        PositionTarget.IGNORE_VZ |
        PositionTarget.IGNORE_AFX |
        PositionTarget.IGNORE_AFY |
        PositionTarget.IGNORE_AFZ |
        PositionTarget.IGNORE_YAW_RATE
    )

    sp.position.x = 0.0
    sp.position.y = 0.0
    # MAVROS 的 ROS topic 侧按 ENU 填，正 z 才是向上。
    sp.position.z = 1.0

    sp.yaw = 0.0

    rospy.loginfo("预发送 setpoint...")
    for _ in range(200):
        pub.publish(sp)
        rate.sleep()

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():

        pub.publish(sp)

        now = rospy.Time.now()

        if current_state.mode != "OFFBOARD" and (now - last_req) > rospy.Duration(1.0):
            res = mode(0, "OFFBOARD")
            rospy.loginfo(f"OFFBOARD: {res.mode_sent}")
            last_req = now

        if not current_state.armed and (now - last_req) > rospy.Duration(1.0):
            res = arm(True)
            rospy.loginfo(f"ARM: {res.success}")
            last_req = now

        rate.sleep()

if __name__ == "__main__":
    main()

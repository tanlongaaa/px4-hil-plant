#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("offb_hold")

    state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0):
            set_mode_client(custom_mode="OFFBOARD")
            last_req = rospy.Time.now()

        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(2.0):
                arming_client(True)
                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    main()
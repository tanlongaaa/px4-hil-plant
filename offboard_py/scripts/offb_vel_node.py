#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


def main():
    rospy.init_node("offboard_velocity_test")

    rospy.Subscriber("/mavros/state", State, state_cb)

    pub = rospy.Publisher(
        "/mavros/setpoint_raw/local",
        PositionTarget,
        queue_size=10
    )

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")

    arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while not current_state.connected:
        rate.sleep()

    sp = PositionTarget()
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    # 只用速度控制
    sp.type_mask = (
        PositionTarget.IGNORE_PX |
        PositionTarget.IGNORE_PY |
        PositionTarget.IGNORE_PZ |
        PositionTarget.IGNORE_AFX |
        PositionTarget.IGNORE_AFY |
        PositionTarget.IGNORE_AFZ |
        PositionTarget.IGNORE_YAW
    )

    # 向上飞（NED坐标，z负是向上）
    sp.velocity.z = -0.5

    # 预热
    for _ in range(100):
        sp.header.stamp = rospy.Time.now()
        pub.publish(sp)
        rate.sleep()

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        sp.header.stamp = now
        pub.publish(sp)

        if current_state.mode != "OFFBOARD":
            set_mode(0, "OFFBOARD")

        if not current_state.armed:
            arm(True)

        rospy.loginfo_throttle(
            1.0,
            f"mode={current_state.mode}, armed={current_state.armed}"
        )

        rate.sleep()


if __name__ == "__main__":
    main()
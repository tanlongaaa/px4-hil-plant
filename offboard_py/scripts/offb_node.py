#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("offboard_minimal_test")

    rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")

    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    rospy.loginfo("等待连接 PX4...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 2.0

    rospy.loginfo("预发送 setpoint...")
    for _ in range(100):
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    last_req_offb = rospy.Time.now()
    last_req_arm = rospy.Time.now()

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)

        now = rospy.Time.now()

        if current_state.mode != "OFFBOARD" and (now - last_req_offb) > rospy.Duration(2.0):
            try:
                resp = set_mode_client(0, "OFFBOARD")
                rospy.loginfo(f"OFFBOARD result: mode_sent={resp.mode_sent}, current_mode={current_state.mode}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"OFFBOARD failed: {e}")
            last_req_offb = now

        if not current_state.armed and (now - last_req_arm) > rospy.Duration(2.0):
            try:
                resp = arming_client(True)
                rospy.loginfo(f"ARM result: success={resp.success}, armed={current_state.armed}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"ARM failed: {e}")
            last_req_arm = now

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass



# import math
# import rospy
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, SetMode

# current_state = State()

# def state_cb(msg):
#     global current_state
#     current_state = msg


# def main():
#     rospy.init_node("offboard_mission")

#     rospy.Subscriber("/mavros/state", State, state_cb)
#     local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

#     rospy.wait_for_service("/mavros/cmd/arming")
#     rospy.wait_for_service("/mavros/set_mode")

#     arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
#     set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

#     rate = rospy.Rate(20)

#     rospy.loginfo("等待连接 PX4...")
#     while not rospy.is_shutdown() and not current_state.connected:
#         rate.sleep()

#     pose = PoseStamped()
#     pose.pose.position.x = 0
#     pose.pose.position.y = 0
#     pose.pose.position.z = 2.0   # 起飞高度

#     # ===============================
#     # 1️⃣ 预发送 setpoint（必须）
#     # ===============================
#     rospy.loginfo("预发送 setpoint...")
#     for _ in range(100):
#         pose.header.stamp = rospy.Time.now()
#         local_pos_pub.publish(pose)
#         rate.sleep()

#     last_req = rospy.Time.now()

#     rospy.loginfo("进入 Offboard 控制阶段")

#     start_time = rospy.Time.now()
#     phase = "TAKEOFF"

   
#     # ===============================
#     # 模式切换
#     # ===============================
#     last_req_offb = rospy.Time.now()
#     last_req_arm = rospy.Time.now()

#     while not rospy.is_shutdown():
#         pose.header.stamp = rospy.Time.now()
#         local_pos_pub.publish(pose)

#         now = rospy.Time.now()

#         if current_state.mode != "OFFBOARD" and (now - last_req_offb) > rospy.Duration(2.0):
#             try:
#                 offb_resp = set_mode_client(0, "OFFBOARD")
#                 rospy.loginfo(f"OFFBOARD result: mode_sent={offb_resp.mode_sent}, current_mode={current_state.mode}")
#             except rospy.ServiceException as e:
#                 rospy.logwarn(f"OFFBOARD 服务失败: {e}")
#             last_req_offb = now

#         if not current_state.armed and (now - last_req_arm) > rospy.Duration(2.0):
#             try:
#                 arm_resp = arming_client(True)
#                 rospy.loginfo(f"ARM result: success={arm_resp.success}, armed={current_state.armed}")
#             except rospy.ServiceException as e:
#                 rospy.logwarn(f"ARM 服务失败: {e}")
#             last_req_arm = now

#         rate.sleep()
#         # ===============================
#         # 状态机控制
#         # ===============================
#         elapsed = (rospy.Time.now() - start_time).to_sec()

#         # 起飞阶段
#         if phase == "TAKEOFF":
#             if elapsed > 5:
#                 rospy.loginfo("进入悬停阶段")
#                 phase = "HOVER"

#         elif phase == "HOVER":

#             # 圆轨迹参数
#             radius = 2.0       # 半径（米）
#             omega = 0.5        # 角速度（越大越快）

#             t = elapsed - 5    # 从起飞结束开始计时

#             pose.pose.position.x = radius * math.cos(omega * t)
#             pose.pose.position.y = radius * math.sin(omega * t)
#             pose.pose.position.z = 2.0

#             if elapsed > 25:
#                 rospy.loginfo("开始降落")
#                 phase = "LAND"

#         # 降落阶段
#         elif phase == "LAND":
#             set_mode_client(0, "AUTO.LAND")
#             rospy.loginfo("切换 AUTO.LAND")
#             break

#         rate.sleep()


# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
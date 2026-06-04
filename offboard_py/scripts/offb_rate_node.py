#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

current_state = State()

current_x = 0.0
current_y = 0.0
current_z = 0.0

current_vx = 0.0
current_vy = 0.0
current_vz = 0.0


def odom_cb(msg):
    global current_x, current_y, current_z
    global current_vx, current_vy, current_vz

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    current_z = msg.pose.pose.position.z

    current_vx = msg.twist.twist.linear.x
    current_vy = msg.twist.twist.linear.y
    current_vz = msg.twist.twist.linear.z


def state_cb(msg):
    global current_state
    current_state = msg


def main():
    rospy.init_node("offboard_attitude_test")

    rospy.Subscriber("/sim/odom", Odometry, odom_cb, queue_size=10)
    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)

    att_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=20)

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")

    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(50)

    rospy.loginfo("等待连接 PX4...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    cmd = AttitudeTarget()

    # 只使用 attitude + thrust，忽略 body rate
    cmd.type_mask = (
        AttitudeTarget.IGNORE_ROLL_RATE |
        AttitudeTarget.IGNORE_PITCH_RATE |
        AttitudeTarget.IGNORE_YAW_RATE
    )

    # 初始水平姿态
    q = quaternion_from_euler(0.0, 0.0, 0.0)
    cmd.orientation = Quaternion(*q)

    # 预发送时不给油
    cmd.thrust = 0.0

    rospy.loginfo("预发送 attitude + thrust setpoint...")
    for _ in range(150):
        cmd.header.stamp = rospy.Time.now()
        att_pub.publish(cmd)
        rate.sleep()

    last_req_offb = rospy.Time.now()
    last_req_arm = rospy.Time.now()

    rospy.loginfo("开始请求 OFFBOARD / ARM")

    # =========================
    # 目标
    # =========================
    x_des = 0.0
    y_des = 0.0
    z_des = 1.0
    yaw_des = 0.0

    # =========================
    # 高度 PD 参数
    # 1.0 = 悬停推力
    # =========================
    kp_z = 0.06
    kd_z = 0.10

    # =========================
    # 横向定点参数（先保守）
    # =========================
    kp_xy = 0.03
    kd_xy = 0.05

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # =========================
        # 横向定点控制
        # =========================
        e_x = x_des - current_x
        e_y = y_des - current_y
        e_vx = 0.0 - current_vx
        e_vy = 0.0 - current_vy

        # 小角度姿态命令
        # 如果发现越纠越偏，后面把符号整体反一下
        pitch_cmd = kp_xy * e_x + kd_xy * e_vx
        roll_cmd = -(kp_xy * e_y + kd_xy * e_vy)

        # 限幅，防止倾角过大
        roll_cmd = max(-0.12, min(0.12, roll_cmd))
        pitch_cmd = max(-0.12, min(0.12, pitch_cmd))

        q = quaternion_from_euler(roll_cmd, pitch_cmd, yaw_des)
        cmd.orientation = Quaternion(*q)

        # =========================
        # 高度控制
        # =========================
        if current_state.armed:
            e_z = z_des - current_z
            e_vz = 0.0 - current_vz

            cmd.thrust = 1.0 + kp_z * e_z + kd_z * e_vz

            # 限幅，避免过猛
            cmd.thrust = max(0.90, min(1.10, cmd.thrust))
        else:
            cmd.thrust = 0.0

        cmd.header.stamp = now
        att_pub.publish(cmd)

        # =========================
        # OFFBOARD 模式请求
        # =========================
        if current_state.mode != "OFFBOARD" and (now - last_req_offb) > rospy.Duration(2.0):
            try:
                resp = set_mode_client(0, "OFFBOARD")
                rospy.loginfo(f"OFFBOARD result: mode_sent={resp.mode_sent}, current_mode={current_state.mode}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"OFFBOARD failed: {e}")
            last_req_offb = now

        # =========================
        # ARM 请求
        # =========================
        if not current_state.armed and (now - last_req_arm) > rospy.Duration(2.0):
            try:
                resp = arming_client(True)
                rospy.loginfo(f"ARM result: success={resp.success}, armed={current_state.armed}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"ARM failed: {e}")
            last_req_arm = now

        rospy.loginfo_throttle(
            0.5,
            f"mode={current_state.mode}, armed={current_state.armed}, "
            f"x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}, "
            f"vx={current_vx:.2f}, vy={current_vy:.2f}, vz={current_vz:.2f}, "
            f"roll_cmd={roll_cmd:.3f}, pitch_cmd={pitch_cmd:.3f}, thrust={cmd.thrust:.3f}"
        )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
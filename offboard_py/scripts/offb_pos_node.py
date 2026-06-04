#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, StatusText, EstimatorStatus, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from tf.transformations import euler_from_quaternion, quaternion_from_euler

current_state = State()

px4_x = 0.0
px4_y = 0.0
px4_z = 0.0
px4_yaw = 0.0

got_px4_pose = False
px4_pose_count = 0
last_px4_pose_time = None

sim_x = 0.0
sim_y = 0.0
sim_z = 0.0
sim_yaw = 0.0
got_sim_odom = False

last_status_text = ""

estimator_status = EstimatorStatus()
got_estimator_status = False
last_estimator_status_time = None


def state_cb(msg):
    global current_state
    current_state = msg


def px4_pose_cb(msg):
    global px4_x, px4_y, px4_z, px4_yaw
    global got_px4_pose, px4_pose_count, last_px4_pose_time

    px4_x = msg.pose.position.x
    px4_y = msg.pose.position.y
    px4_z = msg.pose.position.z

    q = msg.pose.orientation
    _, _, px4_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    got_px4_pose = True
    px4_pose_count += 1
    last_px4_pose_time = rospy.Time.now()


def sim_odom_cb(msg):
    global sim_x, sim_y, sim_z, sim_yaw, got_sim_odom
    sim_x = msg.pose.pose.position.x
    sim_y = msg.pose.pose.position.y
    sim_z = msg.pose.pose.position.z
    q = msg.pose.pose.orientation
    _, _, sim_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    got_sim_odom = True


def status_text_cb(msg):
    global last_status_text
    text = msg.text.strip()
    if text and text != last_status_text:
        last_status_text = text
        rospy.logwarn("PX4 STATUSTEXT severity=%d: %s", msg.severity, text)


def estimator_status_cb(msg):
    global estimator_status, got_estimator_status, last_estimator_status_time
    estimator_status = msg
    got_estimator_status = True
    last_estimator_status_time = rospy.Time.now()


def finite_pose(x, y, z, yaw):
    return all(math.isfinite(v) for v in [x, y, z, yaw])


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def px4_pose_sane(max_abs_xy, min_z, max_z, max_truth_error, max_yaw_truth_error):
    if not got_px4_pose:
        return False, "no_px4_pose"
    if not finite_pose(px4_x, px4_y, px4_z, px4_yaw):
        return False, "non_finite_px4_pose"
    if abs(px4_x) > max_abs_xy or abs(px4_y) > max_abs_xy:
        return False, "px4_xy_out_of_range"
    if px4_z < min_z or px4_z > max_z:
        return False, "px4_z_out_of_range"
    if got_sim_odom:
        err = math.sqrt(
            (px4_x - sim_x) ** 2 +
            (px4_y - sim_y) ** 2 +
            (px4_z - sim_z) ** 2
        )
        if err > max_truth_error:
            return False, "px4_truth_mismatch"
        yaw_err = abs(angle_error(px4_yaw, sim_yaw))
        if yaw_err > max_yaw_truth_error:
            return False, "px4_truth_yaw_mismatch"
    return True, "ok"


def sim_pose_sane(max_abs_xy, min_z, max_z):
    if not got_sim_odom:
        return False, "no_sim_odom"
    if not all(math.isfinite(v) for v in [sim_x, sim_y, sim_z]):
        return False, "non_finite_sim_odom"
    if abs(sim_x) > max_abs_xy or abs(sim_y) > max_abs_xy:
        return False, "sim_xy_out_of_range"
    if sim_z < min_z or sim_z > max_z:
        return False, "sim_z_out_of_range"
    return True, "ok"


def px4_pose_ready(max_abs_xy, min_z, max_z, max_truth_error, max_yaw_truth_error, pose_timeout_sec):
    if last_px4_pose_time is None:
        return False, "no_px4_pose_time", 999.0

    age = (rospy.Time.now() - last_px4_pose_time).to_sec()
    if age > pose_timeout_sec:
        return False, "stale_px4_pose", age

    pose_valid, invalid_reason = px4_pose_sane(
        max_abs_xy,
        min_z,
        max_z,
        max_truth_error,
        max_yaw_truth_error
    )
    if not pose_valid:
        return False, invalid_reason, age

    return True, "ok", age


def estimator_status_text():
    if not got_estimator_status:
        return "none"

    return (
        "att=%d vel_h=%d vel_v=%d pos_rel=%d pos_abs=%d hgt_abs=%d hgt_agl=%d"
        % (
            int(estimator_status.attitude_status_flag),
            int(estimator_status.velocity_horiz_status_flag),
            int(estimator_status.velocity_vert_status_flag),
            int(estimator_status.pos_horiz_rel_status_flag),
            int(estimator_status.pos_horiz_abs_status_flag),
            int(estimator_status.pos_vert_abs_status_flag),
            int(estimator_status.pos_vert_agl_status_flag),
        )
    )


def px4_estimator_ready(timeout_sec):
    if last_estimator_status_time is None:
        return False, "no_estimator_status", 999.0

    age = (rospy.Time.now() - last_estimator_status_time).to_sec()
    if age > timeout_sec:
        return False, "stale_estimator_status", age

    horizontal_pos_ok = (
        estimator_status.pos_horiz_rel_status_flag or
        estimator_status.pos_horiz_abs_status_flag
    )
    vertical_pos_ok = (
        estimator_status.pos_vert_abs_status_flag or
        estimator_status.pos_vert_agl_status_flag
    )
    velocity_ok = (
        estimator_status.velocity_horiz_status_flag and
        estimator_status.velocity_vert_status_flag
    )

    if not estimator_status.attitude_status_flag:
        return False, "estimator_attitude_invalid", age
    if not horizontal_pos_ok:
        return False, "estimator_horizontal_position_invalid", age
    if not vertical_pos_ok:
        return False, "estimator_vertical_position_invalid", age
    if not velocity_ok:
        return False, "estimator_velocity_invalid", age

    return True, "ok", age


def is_estimator_reason(reason):
    return "estimator" in reason


def main():
    rospy.init_node("offboard_hold_takeoff")

    rate_hz = rospy.get_param("~rate_hz", 20.0)
    pre_send_count = rospy.get_param("~pre_send_count", 200)
    pose_stable_count = rospy.get_param("~pose_stable_count", 30)
    pose_timeout_sec = rospy.get_param("~pose_timeout_sec", 0.5)
    estimator_timeout_sec = rospy.get_param(
        "~estimator_timeout_sec",
        max(2.0, pose_timeout_sec * 5.0)
    )

    # 目标高度增量：默认 1m，用于悬停实验，避免贴地阶段干扰判断。
    takeoff_delta_z = rospy.get_param("~takeoff_delta_z", 1.0)

    # 缓升速度，避免一上来就给大台阶
    z_ramp_rate = rospy.get_param("~z_ramp_rate", 0.10)  # m/s

    # 默认锁定当前 yaw；如果你想强制指定，再传参数覆盖
    use_current_yaw = rospy.get_param("~use_current_yaw", True)
    target_yaw_param = rospy.get_param("~target_yaw", 0.0)

    # 安全限幅
    max_xy_error_for_start = rospy.get_param("~max_xy_error_for_start", 1.0)
    max_z_cmd = rospy.get_param("~max_z_cmd", 1.0)
    max_abs_px4_xy = rospy.get_param("~max_abs_px4_xy", 5.0)
    min_valid_px4_z = rospy.get_param("~min_valid_px4_z", -0.5)
    max_valid_px4_z = rospy.get_param("~max_valid_px4_z", 3.0)
    max_px4_truth_error = rospy.get_param("~max_px4_truth_error", 0.75)
    max_px4_truth_yaw_error = rospy.get_param("~max_px4_truth_yaw_error", 0.35)
    use_sim_truth_reference = rospy.get_param("~use_sim_truth_reference", False)
    require_px4_local_position = rospy.get_param("~require_px4_local_position", True)
    require_px4_estimator_status = rospy.get_param("~require_px4_estimator_status", True)
    require_disarmed_start = rospy.get_param("~require_disarmed_start", True)
    auto_disarm_on_invalid = rospy.get_param("~auto_disarm_on_invalid", True)
    safety_request_interval_sec = rospy.get_param("~safety_request_interval_sec", 1.0)
    ensure_hil_params = rospy.get_param("~ensure_hil_params", True)
    ekf2_abl_lim_min = rospy.get_param("~ekf2_abl_lim_min", 0.8)

    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, px4_pose_cb, queue_size=10)
    rospy.Subscriber("/sim/odom", Odometry, sim_odom_cb, queue_size=10)
    rospy.Subscriber("/mavros/statustext/recv", StatusText, status_text_cb, queue_size=20)
    rospy.Subscriber("/mavros/estimator_status", EstimatorStatus, estimator_status_cb, queue_size=20)

    setpoint_pub = rospy.Publisher(
        "/mavros/setpoint_position/local",
        PoseStamped,
        queue_size=20
    )

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    rospy.wait_for_service("/mavros/param/set")

    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    param_set_client = rospy.ServiceProxy("/mavros/param/set", ParamSet)

    rate = rospy.Rate(rate_hz)
    last_safety_req = rospy.Time(0)

    def safety_stop(reason):
        nonlocal last_safety_req
        if not auto_disarm_on_invalid:
            return

        now = rospy.Time.now()
        if (now - last_safety_req) < rospy.Duration(safety_request_interval_sec):
            return
        last_safety_req = now

        if current_state.mode == "OFFBOARD":
            try:
                set_mode_client(0, "MANUAL")
                rospy.logwarn("Safety stop: leaving OFFBOARD, reason=%s", reason)
            except rospy.ServiceException as e:
                rospy.logwarn("Safety stop MANUAL failed: %s", e)

        if current_state.armed:
            try:
                resp = arming_client(False)
                rospy.logwarn("Safety stop: disarm requested success=%s, reason=%s", resp.success, reason)
            except rospy.ServiceException as e:
                rospy.logwarn("Safety stop disarm failed: %s", e)

    rospy.loginfo("等待PX4连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    if ensure_hil_params:
        try:
            value = ParamValue()
            value.integer = 0
            value.real = float(ekf2_abl_lim_min)
            resp = param_set_client("EKF2_ABL_LIM", value)
            rospy.loginfo(
                "HIL param set EKF2_ABL_LIM=%.3f success=%s reported=%.3f",
                ekf2_abl_lim_min, resp.success, resp.value.real
            )
        except rospy.ServiceException as e:
            rospy.logwarn("HIL param set EKF2_ABL_LIM failed: %s", e)

    if require_disarmed_start and current_state.armed:
        rospy.logwarn("offboard_pos_node要求从未解锁状态启动；当前已armed，先触发安全停机。")
        while not rospy.is_shutdown() and current_state.armed:
            safety_stop("armed_before_offboard_node_start")
            rate.sleep()

    if use_sim_truth_reference:
        rospy.loginfo("PX4已连接，等待 /sim/odom 稳定，使用仿真真值作为offboard锁点...")
        if require_px4_local_position:
            rospy.loginfo("位置OFFBOARD仍要求 PX4 local_position 有效，等待 /mavros/local_position/pose...")
    else:
        rospy.loginfo("PX4已连接，等待 /mavros/local_position/pose 稳定，使用PX4 local pose作为offboard锁点...")

    valid_pose_count = 0
    while not rospy.is_shutdown():
        if use_sim_truth_reference:
            sim_valid, sim_reason = sim_pose_sane(
                max_abs_px4_xy,
                min_valid_px4_z,
                max_valid_px4_z
            )
            px4_ready, px4_reason, px4_age = px4_pose_ready(
                max_abs_px4_xy,
                min_valid_px4_z,
                max_valid_px4_z,
                max_px4_truth_error,
                max_px4_truth_yaw_error,
                pose_timeout_sec
            )
            estimator_ready, estimator_reason, estimator_age = px4_estimator_ready(estimator_timeout_sec)
            px4_required_ready = (
                (px4_ready or not require_px4_local_position) and
                (estimator_ready or not require_px4_estimator_status)
            )
            pose_valid = sim_valid and px4_required_ready
            if not sim_valid:
                invalid_reason = sim_reason
            elif require_px4_local_position and not px4_ready:
                invalid_reason = px4_reason
            elif require_px4_estimator_status and not estimator_ready:
                invalid_reason = estimator_reason
            else:
                invalid_reason = "ok"
        else:
            pose_valid, invalid_reason, px4_age = px4_pose_ready(
                max_abs_px4_xy,
                min_valid_px4_z,
                max_valid_px4_z,
                max_px4_truth_error,
                max_px4_truth_yaw_error,
                pose_timeout_sec
            )
            if pose_valid and require_px4_estimator_status:
                estimator_ready, estimator_reason, estimator_age = px4_estimator_ready(estimator_timeout_sec)
                if not estimator_ready:
                    pose_valid = False
                    invalid_reason = estimator_reason

        if pose_valid:
            valid_pose_count += 1
        else:
            valid_pose_count = 0
            if require_disarmed_start and current_state.armed:
                safety_stop(invalid_reason)

        if valid_pose_count >= pose_stable_count:
            break

        rospy.loginfo_throttle(
            1.0,
            "等待锁点稳定... valid_count=%d/%d px4_count=%d reason=%s px4=(%.2f, %.2f, %.2f, yaw=%.2f) sim=(%.2f, %.2f, %.2f, yaw=%.2f) estimator=(%s)",
            valid_pose_count, pose_stable_count, px4_pose_count, invalid_reason,
            px4_x, px4_y, px4_z, px4_yaw,
            sim_x, sim_y, sim_z, sim_yaw,
            estimator_status_text()
        )

        if require_px4_estimator_status and is_estimator_reason(invalid_reason):
            rospy.logwarn_throttle(
                5.0,
                "PX4 estimator未ready，OFFBOARD会被拒绝。若reason是stale_estimator_status，"
                "可调大 _estimator_timeout_sec；若flags为0，请检查 ekf2 status 和HIL传感器输入。 estimator=(%s)",
                estimator_status_text()
            )
        rate.sleep()

    # 锁定当前点作为悬停基准
    if use_sim_truth_reference:
        hold_x = sim_x
        hold_y = sim_y
        hold_z = sim_z
        hold_yaw = target_yaw_param
    else:
        hold_x = px4_x
        hold_y = px4_y
        hold_z = px4_z
        hold_yaw = px4_yaw if use_current_yaw else target_yaw_param

    xy_norm = math.hypot(hold_x, hold_y)
    if xy_norm > max_xy_error_for_start:
        rospy.logwarn(
            "起飞前当前位置偏差过大: hold=(%.2f, %.2f, %.2f), |xy|=%.2f > %.2f",
            hold_x, hold_y, hold_z, xy_norm, max_xy_error_for_start
        )

    target_x = hold_x
    target_y = hold_y

    # 位置setpoint写入PX4 local frame，因此高度目标必须相对PX4当前local z。
    # max_z_cmd在这里作为本次起飞允许的最大高度增量，避免PX4 local z非零时目标反而低于当前估计。
    climb_delta_z = min(takeoff_delta_z, max_z_cmd)
    final_target_z = hold_z + climb_delta_z

    # 预发送阶段保持当前高度；进入OFFBOARD并解锁后再按z_ramp_rate缓升。
    target_z_cmd = hold_z

    rospy.loginfo(
        "锁定当前点起飞: hold=(%.2f, %.2f, %.2f, yaw=%.2f) climb_delta=%.2f -> final_target=(%.2f, %.2f, %.2f, yaw=%.2f)",
        hold_x, hold_y, hold_z, hold_yaw,
        climb_delta_z,
        target_x, target_y, final_target_z, hold_yaw
    )

    target = PoseStamped()

    def fill_pose_target(msg, stamp, x, y, z, yaw):
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

    fill_pose_target(target, rospy.Time.now(), target_x, target_y, target_z_cmd, hold_yaw)

    rospy.loginfo("预发送 pose local setpoint...")
    for _ in range(pre_send_count):
        fill_pose_target(target, rospy.Time.now(), target_x, target_y, target_z_cmd, hold_yaw)
        setpoint_pub.publish(target)
        rate.sleep()

    last_req_offb = rospy.Time.now()
    last_req_arm = rospy.Time.now()
    last_loop_time = rospy.Time.now()

    rospy.loginfo("开始请求 OFFBOARD / ARM")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_loop_time).to_sec()
        if dt < 0.0:
            dt = 0.0
        last_loop_time = now

        if use_sim_truth_reference:
            sim_valid, sim_reason = sim_pose_sane(
                max_abs_px4_xy,
                min_valid_px4_z,
                max_valid_px4_z
            )
            px4_ready, px4_reason, pose_age = px4_pose_ready(
                max_abs_px4_xy,
                min_valid_px4_z,
                max_valid_px4_z,
                max_px4_truth_error,
                max_px4_truth_yaw_error,
                pose_timeout_sec
            )
            estimator_ready, estimator_reason, estimator_age = px4_estimator_ready(estimator_timeout_sec)
            px4_required_ready = (
                (px4_ready or not require_px4_local_position) and
                (estimator_ready or not require_px4_estimator_status)
            )
            pose_ok = sim_valid and px4_required_ready
            if not sim_valid:
                invalid_reason = sim_reason
            elif require_px4_local_position and not px4_ready:
                invalid_reason = px4_reason
            elif require_px4_estimator_status and not estimator_ready:
                invalid_reason = estimator_reason
            else:
                invalid_reason = "ok"
        else:
            pose_ok, invalid_reason, pose_age = px4_pose_ready(
                max_abs_px4_xy,
                min_valid_px4_z,
                max_valid_px4_z,
                max_px4_truth_error,
                max_px4_truth_yaw_error,
                pose_timeout_sec
            )
            if pose_ok and require_px4_estimator_status:
                estimator_ready, estimator_reason, estimator_age = px4_estimator_ready(estimator_timeout_sec)
                if not estimator_ready:
                    pose_ok = False
                    invalid_reason = estimator_reason

        if not pose_ok:
            fallback_x = px4_x if got_px4_pose else 0.0
            fallback_y = px4_y if got_px4_pose else 0.0
            fallback_z = px4_z if got_px4_pose else 0.0
            fallback_yaw = px4_yaw if got_px4_pose else target_yaw_param
            fill_pose_target(target, now, fallback_x, fallback_y, fallback_z, fallback_yaw)
            setpoint_pub.publish(target)
            safety_stop(invalid_reason)

            rospy.logwarn_throttle(
                1.0,
                "offboard锁点无效，禁止OFFBOARD/ARM: reason=%s pose_age=%.3f px4_count=%d px4=(%.2f, %.2f, %.2f, yaw=%.2f) sim=(%.2f, %.2f, %.2f, yaw=%.2f) estimator=(%s)",
                invalid_reason, pose_age, px4_pose_count,
                px4_x, px4_y, px4_z, px4_yaw,
                sim_x, sim_y, sim_z, sim_yaw,
                estimator_status_text()
            )

            if require_px4_estimator_status and is_estimator_reason(invalid_reason):
                rospy.logwarn_throttle(
                    5.0,
                    "PX4 estimator未ready，OFFBOARD会被拒绝。若reason是stale_estimator_status，"
                    "可调大 _estimator_timeout_sec；若flags为0，请检查 ekf2 status 和HIL传感器输入。 estimator=(%s)",
                    estimator_status_text()
                )
            rate.sleep()
            continue

        # 如果外部参数把目标高度改高，进入 OFFBOARD 后继续限速爬升。
        if current_state.mode == "OFFBOARD" and current_state.armed:
            target_z_cmd = min(target_z_cmd + z_ramp_rate * dt, final_target_z)

        fill_pose_target(target, now, target_x, target_y, target_z_cmd, hold_yaw)
        setpoint_pub.publish(target)

        if current_state.mode != "OFFBOARD" and (now - last_req_offb) > rospy.Duration(1.0):
            try:
                resp = set_mode_client(0, "OFFBOARD")
                rospy.loginfo(
                    "OFFBOARD result: mode_sent=%s, current_mode=%s",
                    resp.mode_sent, current_state.mode
                )
            except rospy.ServiceException as e:
                rospy.logwarn("OFFBOARD failed: %s", e)
            last_req_offb = now

        if current_state.mode == "OFFBOARD" and (not current_state.armed) and (now - last_req_arm) > rospy.Duration(1.0):
            try:
                resp = arming_client(True)
                rospy.loginfo(
                    "ARM result: success=%s result=%s armed=%s",
                    resp.success, resp.result, current_state.armed
                )
            except rospy.ServiceException as e:
                rospy.logwarn("ARM failed: %s", e)
            last_req_arm = now

        if got_sim_odom:
            rospy.loginfo_throttle(
                0.5,
                "mode=%s armed=%s target_enu=(%.2f, %.2f, %.2f, yaw=%.2f) "
                "px4_est=(%.2f, %.2f, %.2f, yaw=%.2f) sim_truth=(%.2f, %.2f, %.2f) pose_age=%.3f",
                current_state.mode, current_state.armed,
                target_x, target_y, target_z_cmd, hold_yaw,
                px4_x, px4_y, px4_z, px4_yaw,
                sim_x, sim_y, sim_z,
                pose_age
            )
        else:
            rospy.loginfo_throttle(
                0.5,
                "mode=%s armed=%s target_enu=(%.2f, %.2f, %.2f, yaw=%.2f) "
                "px4_est=(%.2f, %.2f, %.2f, yaw=%.2f) pose_age=%.3f",
                current_state.mode, current_state.armed,
                target_x, target_y, target_z_cmd, hold_yaw,
                px4_x, px4_y, px4_z, px4_yaw,
                pose_age
            )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

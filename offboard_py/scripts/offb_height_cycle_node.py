#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, StatusText, EstimatorStatus
from mavros_msgs.srv import CommandBool, SetMode
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
got_sim_odom = False

estimator_status = EstimatorStatus()
got_estimator_status = False
last_estimator_status_time = None

last_status_text = ""


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
    global sim_x, sim_y, sim_z, got_sim_odom

    sim_x = msg.pose.pose.position.x
    sim_y = msg.pose.pose.position.y
    sim_z = msg.pose.pose.position.z
    got_sim_odom = True


def estimator_status_cb(msg):
    global estimator_status, got_estimator_status, last_estimator_status_time

    estimator_status = msg
    got_estimator_status = True
    last_estimator_status_time = rospy.Time.now()


def status_text_cb(msg):
    global last_status_text

    text = msg.text.strip()
    if text and text != last_status_text:
        last_status_text = text
        rospy.logwarn("PX4 STATUSTEXT severity=%d: %s", msg.severity, text)


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


def px4_pose_ready(pose_timeout_sec, max_abs_xy, min_z, max_z):
    if not got_px4_pose or last_px4_pose_time is None:
        return False, "no_px4_pose"

    age = (rospy.Time.now() - last_px4_pose_time).to_sec()
    if age > pose_timeout_sec:
        return False, "stale_px4_pose"

    if not all(math.isfinite(v) for v in [px4_x, px4_y, px4_z, px4_yaw]):
        return False, "non_finite_px4_pose"

    if abs(px4_x) > max_abs_xy or abs(px4_y) > max_abs_xy:
        return False, "px4_xy_out_of_range"

    if px4_z < min_z or px4_z > max_z:
        return False, "px4_z_out_of_range"

    return True, "ok"


def estimator_ready(estimator_timeout_sec):
    if not got_estimator_status or last_estimator_status_time is None:
        return False, "no_estimator_status"

    age = (rospy.Time.now() - last_estimator_status_time).to_sec()
    if age > estimator_timeout_sec:
        return False, "stale_estimator_status"

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
        return False, "estimator_attitude_invalid"

    if not horizontal_pos_ok:
        return False, "estimator_horizontal_position_invalid"

    if not vertical_pos_ok:
        return False, "estimator_vertical_position_invalid"

    if not velocity_ok:
        return False, "estimator_velocity_invalid"

    return True, "ok"


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


def step_towards(current, target, rate, dt):
    if current < target:
        return min(current + rate * dt, target)
    else:
        return max(current - rate * dt, target)


def main():
    rospy.init_node("offboard_height_cycle")

    # ===== 基本频率 =====
    rate_hz = float(rospy.get_param("~rate_hz", 20.0))
    pre_send_count = int(rospy.get_param("~pre_send_count", 200))
    pose_stable_count = int(rospy.get_param("~pose_stable_count", 30))

    # ===== 高度循环任务参数 =====
    # height_step = float(rospy.get_param("~height_step", 0.2))
    # num_cycles = int(rospy.get_param("~num_cycles", 5))
    target_delta_z = float(rospy.get_param("~target_delta_z", 1.0))
    num_cycles = int(rospy.get_param("~num_cycles", 5))

    z_ramp_rate_up = float(rospy.get_param("~z_ramp_rate_up", 0.03))
    z_ramp_rate_down = float(rospy.get_param("~z_ramp_rate_down", 0.04))

    hover_top_sec = float(rospy.get_param("~hover_top_sec", 5.0))
    hover_base_sec = float(rospy.get_param("~hover_base_sec", 3.0))

    z_reach_tol = float(rospy.get_param("~z_reach_tol", 0.05))

    # ===== 起始点/原点稳定判据 =====
    # START_HOLD 用于过滤刚进入 OFFBOARD/ARM 时的异常上冲，不计入任务轮次。
    start_hold_sec = float(rospy.get_param("~start_hold_sec", 6.0))
    start_settle_tol = float(rospy.get_param("~start_settle_tol", 0.06))

    # 每轮下降到底部后，必须回到起始点附近并稳定，才允许下一轮开始。
    base_xy_tol = float(rospy.get_param("~base_xy_tol", 0.12))
    base_z_tol = float(rospy.get_param("~base_z_tol", 0.06))
    base_stable_sec = float(rospy.get_param("~base_stable_sec", 3.0))
    # 回原点时不要贴地飞，先保持一个小高度，再水平回到起始点。
    return_home_delta_z = float(rospy.get_param("~return_home_delta_z", 0.35))
    return_xy_tol = float(rospy.get_param("~return_xy_tol", 0.06))
    return_z_tol = float(rospy.get_param("~return_z_tol", 0.06))

    # 在返航高度回到原点附近后，连续稳定一段时间，才允许真正下降到地面
    return_home_stable_sec = float(rospy.get_param("~return_home_stable_sec", 2.0))

    # ===== 安全参数 =====
    pose_timeout_sec = float(rospy.get_param("~pose_timeout_sec", 0.5))
    estimator_timeout_sec = float(rospy.get_param("~estimator_timeout_sec", 5.0))
    max_abs_xy = float(rospy.get_param("~max_abs_xy", 5.0))
    min_valid_z = float(rospy.get_param("~min_valid_z", -0.5))
    max_valid_z = float(rospy.get_param("~max_valid_z", 5.0))

    # 是否完成所有循环后自动降落并 disarm
    auto_disarm_when_done = bool(rospy.get_param("~auto_disarm_when_done", False))

    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, px4_pose_cb, queue_size=10)
    rospy.Subscriber("/mavros/estimator_status", EstimatorStatus, estimator_status_cb, queue_size=20)
    rospy.Subscriber("/mavros/statustext/recv", StatusText, status_text_cb, queue_size=20)
    rospy.Subscriber("/sim/odom", Odometry, sim_odom_cb, queue_size=10)

    setpoint_pub = rospy.Publisher(
        "/mavros/setpoint_position/local",
        PoseStamped,
        queue_size=20
    )

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")

    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(rate_hz)

    rospy.loginfo("等待 PX4/MAVROS 连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    rospy.loginfo("PX4 已连接，等待 local_position 和 estimator 稳定...")

    valid_count = 0
    while not rospy.is_shutdown():
        pose_ok, pose_reason = px4_pose_ready(
            pose_timeout_sec,
            max_abs_xy,
            min_valid_z,
            max_valid_z
        )
        est_ok, est_reason = estimator_ready(estimator_timeout_sec)

        if pose_ok and est_ok:
            valid_count += 1
        else:
            valid_count = 0

        if valid_count >= pose_stable_count:
            break

        rospy.loginfo_throttle(
            1.0,
            "等待状态稳定... valid=%d/%d pose=%s est=%s px4=(%.2f, %.2f, %.2f) estimator=(%s)",
            valid_count,
            pose_stable_count,
            pose_reason,
            est_reason,
            px4_x,
            px4_y,
            px4_z,
            estimator_status_text()
        )

        rate.sleep()

    # ===== 锁定起始点 =====
    base_x = 0.0
    base_y = 0.0
    base_z = 0.0
    base_z_raw = px4_z
    base_yaw = px4_yaw

    return_home_z = base_z + return_home_delta_z

    target_z_cmd = base_z
    target = PoseStamped()

    rospy.loginfo(
        "锁定起始点: base=(%.2f, %.2f, %.2f, yaw=%.2f), raw_z=%.2f, return_home_z=%.2f, target_delta_z=%.2f, target_top_z=%.2f, num_cycles=%d",
        base_x,
        base_y,
        base_z,
        base_yaw,
        base_z_raw,
        return_home_z,
        target_delta_z,
        base_z + target_delta_z,
        num_cycles
    )

    # ===== 预发送 setpoint =====
    rospy.loginfo("预发送起始点 setpoint...")
    for _ in range(pre_send_count):
        fill_pose_target(target, rospy.Time.now(), base_x, base_y, base_z, base_yaw)
        setpoint_pub.publish(target)
        rate.sleep()

    # ===== 主状态机 =====
    cycle_idx = 1
    phase = "START_HOLD"
    phase_start_time = rospy.Time.now()

    start_stable_count = 0
    base_stable_count = 0
    return_home_stable_count = 0

    start_stable_count_required = max(1, int(start_hold_sec * rate_hz))
    base_stable_count_required = max(1, int(base_stable_sec * rate_hz))
    return_home_stable_count_required = max(1, int(return_home_stable_sec * rate_hz))
    last_req_offb = rospy.Time.now()
    last_req_arm = rospy.Time.now()
    last_loop_time = rospy.Time.now()

    rospy.loginfo("开始高度循环任务...")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last_loop_time).to_sec()
        if dt < 0.0:
            dt = 0.0
        last_loop_time = now

        pose_ok, pose_reason = px4_pose_ready(
            pose_timeout_sec,
            max_abs_xy,
            min_valid_z,
            max_valid_z
        )
        est_ok, est_reason = estimator_ready(estimator_timeout_sec)

        if not pose_ok or not est_ok:
            fill_pose_target(target, now, px4_x, px4_y, px4_z, px4_yaw)
            setpoint_pub.publish(target)

            rospy.logwarn_throttle(
                1.0,
                "状态异常，保持当前位置，不推进任务: pose=%s est=%s px4=(%.2f, %.2f, %.2f)",
                pose_reason,
                est_reason,
                px4_x,
                px4_y,
                px4_z
            )
            rate.sleep()
            continue

        # 先持续发布 setpoint，再请求模式和解锁
        if current_state.mode != "OFFBOARD" and (now - last_req_offb) > rospy.Duration(1.0):
            try:
                resp = set_mode_client(0, "OFFBOARD")
                rospy.loginfo("OFFBOARD result: mode_sent=%s current_mode=%s", resp.mode_sent, current_state.mode)
            except rospy.ServiceException as e:
                rospy.logwarn("OFFBOARD failed: %s", e)
            last_req_offb = now

        if current_state.mode == "OFFBOARD" and (not current_state.armed) and (now - last_req_arm) > rospy.Duration(1.0):
            try:
                resp = arming_client(True)
                rospy.loginfo("ARM result: success=%s armed=%s", resp.success, current_state.armed)
            except rospy.ServiceException as e:
                rospy.logwarn("ARM failed: %s", e)
            last_req_arm = now

        # ===== 只有 OFFBOARD + armed 后才推进高度任务 =====
        if current_state.mode == "OFFBOARD" and current_state.armed:
            current_top_z = base_z + target_delta_z  # 每轮固定同一个目标高度，不再叠加

            px4_base_reached = (
                abs(px4_x - base_x) <= base_xy_tol and
                abs(px4_y - base_y) <= base_xy_tol and
                abs(px4_z - base_z) <= base_z_tol
            )
            # 如果 /sim/odom 可用，则同时要求仿真真值也回到起始点附近。
            # 这样可以避免 PX4 估计已经接近原点，但真实模型还没回来的情况。
            if got_sim_odom:
                sim_base_reached = (
                    abs(sim_x - base_x) <= base_xy_tol and
                    abs(sim_y - base_y) <= base_xy_tol and
                    abs(sim_z - base_z) <= base_z_tol
                )
            else:
                sim_base_reached = True

            base_reached = px4_base_reached and sim_base_reached

            if phase == "START_HOLD":
                # 正式任务开始前，强制保持起始点。
                # 这一步用于过滤刚进入 OFFBOARD/ARM 时的异常上冲/回落过程，不计入 cycle。
                target_z_cmd = base_z

                start_reached = (
                    abs(px4_x - base_x) <= base_xy_tol and
                    abs(px4_y - base_y) <= base_xy_tol and
                    abs(px4_z - base_z) <= 0.12
                )

                if got_sim_odom:
                    sim_start_reached = (
                        abs(sim_x - base_x) <= base_xy_tol and
                        abs(sim_y - base_y) <= base_xy_tol and
                        abs(sim_z - base_z) <= 0.08
                    )
                else:
                    sim_start_reached = True

                if start_reached and sim_start_reached:
                    start_stable_count += 1
                else:
                    start_stable_count = 0

                if start_stable_count >= start_stable_count_required:
                    phase = "ASCEND"
                    phase_start_time = now
                    rospy.loginfo(
                        "起始点已稳定 %.1f s，正式开始第 1 轮，高度目标 %.2f m",
                        start_hold_sec,
                        current_top_z
                    )

            elif phase == "ASCEND":
                target_z_cmd = step_towards(
                    target_z_cmd,
                    current_top_z,
                    z_ramp_rate_up,
                    dt
                )

                if abs(px4_z - current_top_z) <= z_reach_tol:
                    phase = "HOVER_TOP"
                    phase_start_time = now
                    rospy.loginfo(
                        "第 %d 轮到达目标高度 %.2f m，开始顶部悬停 %.1f s",
                        cycle_idx,
                        current_top_z,
                        hover_top_sec
                    )

            elif phase == "HOVER_TOP":
                target_z_cmd = current_top_z

                if (now - phase_start_time).to_sec() >= hover_top_sec:
                    phase = "RETURN_HOME"
                    phase_start_time = now
                    base_stable_count = 0
                    rospy.loginfo(
                        "第 %d 轮顶部悬停结束，先下降/保持到返航高度 %.2f m，并回到起始点 x/y=(%.2f, %.2f)",
                        cycle_idx,
                        return_home_z,
                        base_x,
                        base_y
                    )
            elif phase == "RETURN_HOME":
                # 在返航高度回到起始 x/y，避免贴地后无法水平移动。
                target_z_cmd = step_towards(
                    target_z_cmd,
                    return_home_z,
                    z_ramp_rate_down,
                    dt
                )

                px4_return_reached = (
                    abs(px4_x - base_x) <= return_xy_tol and
                    abs(px4_y - base_y) <= return_xy_tol and
                    abs(px4_z - return_home_z) <= return_z_tol
                )

                if got_sim_odom:
                    sim_return_reached = (
                        abs(sim_x - base_x) <= return_xy_tol and
                        abs(sim_y - base_y) <= return_xy_tol and
                        abs(sim_z - return_home_z) <= return_z_tol
                    )
                else:
                    sim_return_reached = True

                return_reached = px4_return_reached and sim_return_reached

                if return_reached:
                    return_home_stable_count += 1
                else:
                    return_home_stable_count = 0

                if return_home_stable_count >= return_home_stable_count_required:
                    phase = "DESCEND"
                    phase_start_time = now
                    base_stable_count = 0
                    rospy.loginfo(
                        "第 %d 轮已在返航高度回到起始点并稳定 %.1f s，开始垂直下降到 base_z=%.2f",
                        cycle_idx,
                        return_home_stable_sec,
                        base_z
                    )

                rospy.loginfo_throttle(
                    1.0,
                    "RETURN_HOME: target_z=%.2f px4_err=(%.3f, %.3f, %.3f) sim_err=(%.3f, %.3f, %.3f) stable=%d/%d",
                    return_home_z,
                    px4_x - base_x,
                    px4_y - base_y,
                    px4_z - return_home_z,
                    sim_x - base_x,
                    sim_y - base_y,
                    sim_z - return_home_z,
                    return_home_stable_count,
                    return_home_stable_count_required
                )

            elif phase == "DESCEND":
                # 下降阶段如果发现 x/y 又偏离原点，立刻中止贴地下降，
                # 重新回到 RETURN_HOME，在空中把 x/y 拉回原点。
                px4_xy_ok = (
                    abs(px4_x - base_x) <= return_xy_tol and
                    abs(px4_y - base_y) <= return_xy_tol
                )

                if got_sim_odom:
                    sim_xy_ok = (
                        abs(sim_x - base_x) <= return_xy_tol and
                        abs(sim_y - base_y) <= return_xy_tol
                    )
                else:
                    sim_xy_ok = True

                xy_ok_for_descent = px4_xy_ok and sim_xy_ok

                if not xy_ok_for_descent:
                    phase = "RETURN_HOME"
                    phase_start_time = now
                    return_home_stable_count = 0
                    target_z_cmd = return_home_z

                    rospy.logwarn(
                        "DESCEND阶段发现x/y未回原点，重新进入RETURN_HOME: px4_err_xy=(%.3f, %.3f), sim_err_xy=(%.3f, %.3f)",
                        px4_x - base_x,
                        px4_y - base_y,
                        sim_x - base_x,
                        sim_y - base_y
                    )

                else:
                    target_z_cmd = step_towards(
                        target_z_cmd,
                        base_z,
                        z_ramp_rate_down,
                        dt
                    )

                    if base_reached:
                        phase = "HOVER_BASE"
                        phase_start_time = now
                        base_stable_count = 0
                        rospy.loginfo(
                            "第 %d 轮已回到起始点附近，开始底部稳定等待 %.1f s",
                            cycle_idx,
                            base_stable_sec
                        )

            elif phase == "HOVER_BASE":
                target_z_cmd = base_z

                # 如果已经贴近地面，但 x/y 仍然偏离，则不能继续死等；
                # 重新升到返航高度，把水平位置拉回原点。
                px4_xy_ok = (
                    abs(px4_x - base_x) <= base_xy_tol and
                    abs(px4_y - base_y) <= base_xy_tol
                )

                if got_sim_odom:
                    sim_xy_ok = (
                        abs(sim_x - base_x) <= base_xy_tol and
                        abs(sim_y - base_y) <= base_xy_tol
                    )
                else:
                    sim_xy_ok = True

                if not (px4_xy_ok and sim_xy_ok):
                    phase = "RETURN_HOME"
                    phase_start_time = now
                    return_home_stable_count = 0
                    target_z_cmd = return_home_z

                    rospy.logwarn(
                        "HOVER_BASE阶段x/y仍未回原点，重新进入RETURN_HOME: px4_err_xy=(%.3f, %.3f), sim_err_xy=(%.3f, %.3f)",
                        px4_x - base_x,
                        px4_y - base_y,
                        sim_x - base_x,
                        sim_y - base_y
                    )

                else:
                    if base_reached:
                        base_stable_count += 1
                    else:
                        base_stable_count = 0

                    if (
                        (now - phase_start_time).to_sec() >= hover_base_sec and
                        base_stable_count >= base_stable_count_required
                    ):
                        if cycle_idx >= num_cycles:
                            phase = "DONE"
                            phase_start_time = now
                            rospy.loginfo("所有 %d 轮高度循环完成。", num_cycles)
                        else:
                            cycle_idx += 1
                            phase = "ASCEND"
                            phase_start_time = now
                            rospy.loginfo(
                                "进入第 %d 轮，从起始点再次起飞，高度目标 %.2f m",
                                cycle_idx,
                                base_z + target_delta_z
                            )

        fill_pose_target(target, now, base_x, base_y, target_z_cmd, base_yaw)
        setpoint_pub.publish(target)

        rospy.loginfo_throttle(
            0.5,
            "cycle=%d/%d phase=%s mode=%s armed=%s target=(%.2f, %.2f, %.2f) "
            "px4_est=(%.2f, %.2f, %.2f) sim=(%.2f, %.2f, %.2f)",
            cycle_idx,
            num_cycles,
            phase,
            current_state.mode,
            current_state.armed,
            base_x,
            base_y,
            target_z_cmd,
            px4_x,
            px4_y,
            px4_z,
            sim_x,
            sim_y,
            sim_z
        )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
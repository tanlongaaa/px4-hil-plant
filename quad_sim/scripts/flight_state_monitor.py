#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import ActuatorControl, PositionTarget
from tf.transformations import euler_from_quaternion


class FlightStateMonitor:
    def __init__(self):
        rospy.init_node("flight_state_monitor")

        # ===== 参数 =====
        self.max_points = int(rospy.get_param("~max_points", 4000))
        self.sample_hz = float(rospy.get_param("~sample_hz", 20.0))
        self.display_window_sec = float(rospy.get_param("~display_window_sec", 20.0))

        self.t0 = rospy.Time.now()

        # ===== 最新值 =====
        self.truth_pos = [0.0, 0.0, 0.0]
        self.truth_att = [0.0, 0.0, 0.0]   # roll, pitch, yaw
        self.truth_vel = [0.0, 0.0, 0.0]
        self.truth_rate = [0.0, 0.0, 0.0]

        self.est_pos = [0.0, 0.0, 0.0]
        self.est_att = [0.0, 0.0, 0.0]
        self.est_vel = [0.0, 0.0, 0.0]
        self.est_rate = [0.0, 0.0, 0.0]

        self.ctrl = [0.0, 0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, 0.0]

        # ===== 历史缓存（普通 list，手动裁剪）=====
        self.t_hist = []

        self.truth_x = []
        self.truth_y = []
        self.truth_z = []

        self.truth_roll = []
        self.truth_pitch = []
        self.truth_yaw = []

        self.est_roll = []
        self.est_pitch = []
        self.est_yaw = []

        self.truth_vx = []
        self.truth_vy = []
        self.truth_vz = []

        self.truth_p = []
        self.truth_q = []
        self.truth_r = []

        self.ctrl_u0 = []
        self.ctrl_u1 = []
        self.ctrl_u2 = []
        self.ctrl_u3 = []

        self.err_x = []
        self.err_y = []
        self.err_z = []

        self.est_x = []
        self.est_y = []
        self.est_z = []

        # ===== 订阅 =====
        rospy.Subscriber("/sim/odom", Odometry, self.truth_odom_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.est_pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.est_odom_cb, queue_size=10)
        rospy.Subscriber("/mavros/target_actuator_control", ActuatorControl, self.ctrl_cb, queue_size=10)
        rospy.Subscriber("/sim/hil_actuator_controls", ActuatorControl, self.ctrl_cb, queue_size=10)
        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, self.target_raw_cb, queue_size=10)
        rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, self.target_pose_cb, queue_size=10)

        # ===== 定时采样 =====
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.sample_hz), self.sample_timer_cb)

        # ===== 画布 =====
        plt.ion()
        self.fig, self.axes = plt.subplots(3, 3, figsize=(18, 12))
        self.fig.suptitle("UAV Flight State Monitor", fontsize=16)
        self.fig.show()

    @staticmethod
    def quat_to_euler(x, y, z, w):
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        return [roll, pitch, yaw]

    def truth_odom_cb(self, msg: Odometry):
        self.truth_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

        q = msg.pose.pose.orientation
        self.truth_att = self.quat_to_euler(q.x, q.y, q.z, q.w)

        self.truth_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

        self.truth_rate = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]

    def est_pose_cb(self, msg: PoseStamped):
        self.est_pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

        q = msg.pose.orientation
        self.est_att = self.quat_to_euler(q.x, q.y, q.z, q.w)

    def est_odom_cb(self, msg: Odometry):
        self.est_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

        self.est_rate = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]

    def ctrl_cb(self, msg: ActuatorControl):
        if len(msg.controls) >= 4:
            self.ctrl = [
                float(msg.controls[0]),
                float(msg.controls[1]),
                float(msg.controls[2]),
                float(msg.controls[3]),
            ]

    def target_raw_cb(self, msg: PositionTarget):
        self.target = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
        ]

    def target_pose_cb(self, msg: PoseStamped):
        self.target = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def trim_history(self):
        if len(self.t_hist) <= self.max_points:
            return

        overflow = len(self.t_hist) - self.max_points

        del self.t_hist[:overflow]

        del self.truth_x[:overflow]
        del self.truth_y[:overflow]
        del self.truth_z[:overflow]

        del self.truth_roll[:overflow]
        del self.truth_pitch[:overflow]
        del self.truth_yaw[:overflow]

        del self.est_roll[:overflow]
        del self.est_pitch[:overflow]
        del self.est_yaw[:overflow]

        del self.truth_vx[:overflow]
        del self.truth_vy[:overflow]
        del self.truth_vz[:overflow]

        del self.truth_p[:overflow]
        del self.truth_q[:overflow]
        del self.truth_r[:overflow]

        del self.ctrl_u0[:overflow]
        del self.ctrl_u1[:overflow]
        del self.ctrl_u2[:overflow]
        del self.ctrl_u3[:overflow]

        del self.err_x[:overflow]
        del self.err_y[:overflow]
        del self.err_z[:overflow]

        del self.est_x[:overflow]
        del self.est_y[:overflow]
        del self.est_z[:overflow]

    def get_window_start_index(self):
        if len(self.t_hist) < 2:
            return 0

        t_end = self.t_hist[-1]
        t_start = max(0.0, t_end - self.display_window_sec)

        for i, tt in enumerate(self.t_hist):
            if tt >= t_start:
                return i
        return 0

    def sample_timer_cb(self, _event):
        t = (rospy.Time.now() - self.t0).to_sec()

        err_x = self.target[0] - self.est_pos[0]
        err_y = self.target[1] - self.est_pos[1]
        err_z = self.target[2] - self.est_pos[2]

        self.t_hist.append(t)

        self.truth_x.append(self.truth_pos[0])
        self.truth_y.append(self.truth_pos[1])
        self.truth_z.append(self.truth_pos[2])

        self.truth_roll.append(self.truth_att[0])
        self.truth_pitch.append(self.truth_att[1])
        self.truth_yaw.append(self.truth_att[2])

        self.est_roll.append(self.est_att[0])
        self.est_pitch.append(self.est_att[1])
        self.est_yaw.append(self.est_att[2])

        self.truth_vx.append(self.truth_vel[0])
        self.truth_vy.append(self.truth_vel[1])
        self.truth_vz.append(self.truth_vel[2])

        self.truth_p.append(self.truth_rate[0])
        self.truth_q.append(self.truth_rate[1])
        self.truth_r.append(self.truth_rate[2])

        self.ctrl_u0.append(self.ctrl[0])
        self.ctrl_u1.append(self.ctrl[1])
        self.ctrl_u2.append(self.ctrl[2])
        self.ctrl_u3.append(self.ctrl[3])

        self.est_x.append(self.est_pos[0])
        self.est_y.append(self.est_pos[1])
        self.est_z.append(self.est_pos[2])

        self.err_x.append(err_x)
        self.err_y.append(err_y)
        self.err_z.append(err_z)

        self.trim_history()

        rospy.loginfo_throttle(
            2.0,
            "monitor sample: t=%.2f truth=(%.2f, %.2f, %.2f yaw=%.2f) est=(%.2f, %.2f, %.2f yaw=%.2f) ctrl=(%.2f, %.2f, %.2f, %.2f)",
            t,
            self.truth_pos[0], self.truth_pos[1], self.truth_pos[2], self.truth_att[2],
            self.est_pos[0], self.est_pos[1], self.est_pos[2], self.est_att[2],
            self.ctrl[0], self.ctrl[1], self.ctrl[2], self.ctrl[3]
        )

    def update_plot(self):
        if len(self.t_hist) < 2:
            return

        idx0 = self.get_window_start_index()

        t = self.t_hist[idx0:]

        truth_x = self.truth_x[idx0:]
        truth_y = self.truth_y[idx0:]
        truth_z = self.truth_z[idx0:]

        truth_roll = self.truth_roll[idx0:]
        truth_pitch = self.truth_pitch[idx0:]
        truth_yaw = self.truth_yaw[idx0:]

        est_roll = self.est_roll[idx0:]
        est_pitch = self.est_pitch[idx0:]
        est_yaw = self.est_yaw[idx0:]

        truth_vx = self.truth_vx[idx0:]
        truth_vy = self.truth_vy[idx0:]
        truth_vz = self.truth_vz[idx0:]

        truth_p = self.truth_p[idx0:]
        truth_q = self.truth_q[idx0:]
        truth_r = self.truth_r[idx0:]

        ctrl_u0 = self.ctrl_u0[idx0:]
        ctrl_u1 = self.ctrl_u1[idx0:]
        ctrl_u2 = self.ctrl_u2[idx0:]
        ctrl_u3 = self.ctrl_u3[idx0:]

        err_x = self.err_x[idx0:]
        err_y = self.err_y[idx0:]
        err_z = self.err_z[idx0:]

        est_x = self.est_x[idx0:]
        est_y = self.est_y[idx0:]
        est_z = self.est_z[idx0:]

        ax = self.axes

        # 1. 位置
        ax[0, 0].cla()
        ax[0, 0].plot(t, truth_x, label="x")
        ax[0, 0].plot(t, truth_y, label="y")
        ax[0, 0].plot(t, truth_z, label="z")
        ax[0, 0].set_title("Truth Position")
        ax[0, 0].set_ylabel("m")
        ax[0, 0].grid(True)
        ax[0, 0].legend()

        # 2. 姿态
        ax[0, 1].cla()
        ax[0, 1].plot(t, truth_roll, label="roll_truth")
        ax[0, 1].plot(t, truth_pitch, label="pitch_truth")
        ax[0, 1].plot(t, truth_yaw, label="yaw_truth")
        ax[0, 1].plot(t, est_roll, "--", label="roll_est")
        ax[0, 1].plot(t, est_pitch, "--", label="pitch_est")
        ax[0, 1].plot(t, est_yaw, "--", label="yaw_est")
        ax[0, 1].set_title("Truth vs PX4 Attitude")
        ax[0, 1].set_ylabel("rad")
        ax[0, 1].grid(True)
        ax[0, 1].legend()

        # 3. 线速度
        ax[0, 2].cla()
        ax[0, 2].plot(t, truth_vx, label="vx")
        ax[0, 2].plot(t, truth_vy, label="vy")
        ax[0, 2].plot(t, truth_vz, label="vz")
        ax[0, 2].set_title("Truth Linear Velocity")
        ax[0, 2].set_ylabel("m/s")
        ax[0, 2].grid(True)
        ax[0, 2].legend()

        # 4. 角速度
        ax[1, 0].cla()
        ax[1, 0].plot(t, truth_p, label="p")
        ax[1, 0].plot(t, truth_q, label="q")
        ax[1, 0].plot(t, truth_r, label="r")
        ax[1, 0].set_title("Truth Angular Rate")
        ax[1, 0].set_ylabel("rad/s")
        ax[1, 0].grid(True)
        ax[1, 0].legend()

        # 5. 控制输入
        ax[1, 1].cla()
        ax[1, 1].plot(t, ctrl_u0, label="u0")
        ax[1, 1].plot(t, ctrl_u1, label="u1")
        ax[1, 1].plot(t, ctrl_u2, label="u2")
        ax[1, 1].plot(t, ctrl_u3, label="u3")
        ax[1, 1].set_title("Control Inputs")
        ax[1, 1].grid(True)
        ax[1, 1].legend()

        # 6. 误差
        ax[1, 2].cla()
        ax[1, 2].plot(t, err_x, label="ex")
        ax[1, 2].plot(t, err_y, label="ey")
        ax[1, 2].plot(t, err_z, label="ez")
        ax[1, 2].set_title("Tracking Error (target - px4_est)")
        ax[1, 2].set_ylabel("m")
        ax[1, 2].grid(True)
        ax[1, 2].legend()

        # 7. X 对比
        ax[2, 0].cla()
        ax[2, 0].plot(t, truth_x, label="x_truth")
        ax[2, 0].plot(t, est_x, label="x_est")
        ax[2, 0].set_title("Truth vs PX4 Estimate: X")
        ax[2, 0].set_xlabel("time [s]")
        ax[2, 0].set_ylabel("m")
        ax[2, 0].grid(True)
        ax[2, 0].legend()

        # 8. Y 对比
        ax[2, 1].cla()
        ax[2, 1].plot(t, truth_y, label="y_truth")
        ax[2, 1].plot(t, est_y, label="y_est")
        ax[2, 1].set_title("Truth vs PX4 Estimate: Y")
        ax[2, 1].set_xlabel("time [s]")
        ax[2, 1].set_ylabel("m")
        ax[2, 1].grid(True)
        ax[2, 1].legend()

        # 9. Z 对比
        ax[2, 2].cla()
        ax[2, 2].plot(t, truth_z, label="z_truth")
        ax[2, 2].plot(t, est_z, label="z_est")
        ax[2, 2].set_title("Truth vs PX4 Estimate: Z")
        ax[2, 2].set_xlabel("time [s]")
        ax[2, 2].set_ylabel("m")
        ax[2, 2].grid(True)
        ax[2, 2].legend()

        self.fig.tight_layout(rect=[0, 0, 1, 0.96])

    def run(self):
        rate = rospy.Rate(10)  # 刷图频率 10Hz 足够
        while not rospy.is_shutdown():
            self.update_plot()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = FlightStateMonitor()
        node.run()
    except rospy.ROSInterruptException:
        pass

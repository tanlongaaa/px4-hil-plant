#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_ab.py — 湍流风下 PX4 参数 A/B 鲁棒性验证

恒定风调出的参数, 必须用真实湍流风验证不震荡/不发散。
同一确定性湍流序列 (固定 seed) 喂给不同参数组, 公平对比。

指标:
  exy_rms/max  位置误差
  exy_p95      95分位 (抗峰值能力)
  vxy_rms      水平速度 RMS (震荡指标! 高 = 来回冲)
  tilt_rms/max 姿态
  jerk_xy      水平加速度变化率 RMS (震荡/抖动指标)
"""
import argparse
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from tf.transformations import euler_from_quaternion

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wind_field import WindField


class ABTest:
    def __init__(self, u_ref=5.0, sigma_scale=0.25, gust=2.0, rate=50.0, seed=42):
        rospy.init_node('wind_ab')
        self.rate_hz = rate
        self.dt = 1.0 / rate
        self.seed = seed
        self.u_ref = u_ref
        self.sigma_scale = sigma_scale
        self.gust = gust

        self.pos = np.zeros(3); self.vel = np.zeros(3); self.rpy = np.zeros(3)
        self._got = False
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)
        self.pub_wind = rospy.Publisher('/wind_field/velocity', Vector3Stamped, queue_size=10)
        rospy.wait_for_service('/mavros/param/set')
        self._set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        while not self._got and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo("AB 就绪")
        self.rate = rospy.Rate(rate)

    def _odom_cb(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        self.vel[0] = msg.twist.twist.linear.x
        self.vel[1] = msg.twist.twist.linear.y
        self.vel[2] = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self.rpy = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self._got = True

    def set_param(self, name, value):
        v = ParamValue(); v.real = float(value); v.integer = 0
        try:
            return self._set(param_id=name, value=v).success
        except rospy.ServiceException:
            return False

    def wind_off(self, secs):
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < secs and not rospy.is_shutdown():
            m = Vector3Stamped(); m.header.stamp = rospy.Time.now()
            self.pub_wind.publish(m); self.rate.sleep()

    def run_case(self, label, params, warmup=8.0, measure=40.0):
        for k, v in params.items():
            self.set_param(k, v)
        rospy.sleep(0.5)
        self.wind_off(6.0)  # 回稳态
        # 固定 seed 的湍流, 保证各组风序列一致
        np.random.seed(self.seed)
        wf = WindField(dt=self.dt, u_ref=self.u_ref,
                       sigma_scale=self.sigma_scale, gust_w_max=self.gust)
        target = np.array([0.0, 0.0, 2.5])
        # warmup (打风但不采)
        t0 = rospy.Time.now().to_sec()
        sim_t = 0.0
        while rospy.Time.now().to_sec() - t0 < warmup and not rospy.is_shutdown():
            w = wf.step(self.pos[2], sim_t); sim_t += self.dt
            m = Vector3Stamped(); m.header.stamp = rospy.Time.now()
            m.vector.x, m.vector.y, m.vector.z = w
            self.pub_wind.publish(m); self.rate.sleep()
        # measure
        exy, ez, vxy, tilt = [], [], [], []
        ax_prev = None; jerk = []
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < measure and not rospy.is_shutdown():
            w = wf.step(self.pos[2], sim_t); sim_t += self.dt
            m = Vector3Stamped(); m.header.stamp = rospy.Time.now()
            m.vector.x, m.vector.y, m.vector.z = w
            self.pub_wind.publish(m)
            e = self.pos - target
            exy.append(np.hypot(e[0], e[1])); ez.append(abs(e[2]))
            vh = np.hypot(self.vel[0], self.vel[1]); vxy.append(vh)
            tilt.append(np.degrees(np.hypot(self.rpy[0], self.rpy[1])))
            if ax_prev is not None:
                jerk.append(abs(vh - ax_prev) / self.dt)
            ax_prev = vh
            self.rate.sleep()
        exy=np.array(exy); ez=np.array(ez); vxy=np.array(vxy); tilt=np.array(tilt); jerk=np.array(jerk)
        res = {
            'exy_rms': np.sqrt((exy**2).mean()), 'exy_max': exy.max(),
            'exy_p95': np.percentile(exy, 95),
            'ez_rms': np.sqrt((ez**2).mean()),
            'vxy_rms': np.sqrt((vxy**2).mean()), 'vxy_max': vxy.max(),
            'tilt_rms': np.sqrt((tilt**2).mean()), 'tilt_max': tilt.max(),
            'jerk_rms': np.sqrt((jerk**2).mean()) if len(jerk) else 0.0,
        }
        rospy.loginfo("[%s] exy_rms=%.3f max=%.3f p95=%.3f | vxy_rms=%.3f max=%.3f | "
                      "tilt_rms=%.1f max=%.1f | jerk_rms=%.2f | ez_rms=%.3f",
                      label, res['exy_rms'], res['exy_max'], res['exy_p95'],
                      res['vxy_rms'], res['vxy_max'], res['tilt_rms'], res['tilt_max'],
                      res['jerk_rms'], res['ez_rms'])
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--u-ref', type=float, default=5.0)
    ap.add_argument('--sigma-scale', type=float, default=0.25)
    ap.add_argument('--gust', type=float, default=2.0)
    ap.add_argument('--measure', type=float, default=40.0)
    args, _ = ap.parse_known_args()

    ab = ABTest(u_ref=args.u_ref, sigma_scale=args.sigma_scale, gust=args.gust)

    common = {
        'MPC_XY_P': 0.95, 'MPC_XY_VEL_P_ACC': 1.8,
        'MPC_XY_VEL_D_ACC': 0.2, 'MPC_Z_VEL_P_ACC': 4.0, 'MPC_Z_VEL_I_ACC': 2.0,
    }
    cases = [
        ("I=0.4 (出厂)",  dict(common, MPC_XY_VEL_I_ACC=0.4)),
        ("I=1.2",         dict(common, MPC_XY_VEL_I_ACC=1.2)),
        ("I=1.6 (调参)",  dict(common, MPC_XY_VEL_I_ACC=1.6)),
    ]
    rospy.loginfo("=" * 70)
    rospy.loginfo("湍流 A/B: u_ref=%.1f sigma_scale=%.2f gust=%.1f measure=%.0fs",
                  args.u_ref, args.sigma_scale, args.gust, args.measure)
    rospy.loginfo("=" * 70)
    results = []
    for label, params in cases:
        r = ab.run_case(label, params, measure=args.measure)
        results.append((label, r))
    ab.wind_off(4.0)
    rospy.loginfo("\n" + "=" * 70)
    rospy.loginfo("湍流 A/B 汇总:")
    for label, r in results:
        rospy.loginfo("  %-14s exy_rms=%.3f p95=%.3f | vxy_rms=%.3f (震荡) | tilt_rms=%.1f | jerk=%.2f",
                      label, r['exy_rms'], r['exy_p95'], r['vxy_rms'], r['tilt_rms'], r['jerk_rms'])
    rospy.loginfo("=" * 70)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

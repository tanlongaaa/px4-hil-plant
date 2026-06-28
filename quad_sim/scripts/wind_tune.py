#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_tune.py — PX4 抗风增益自动调参 (坐标下降)

策略:
  - 不改 plant 物理 (wind_filter_tau / flapping 保持保真值)
  - 只热调 PX4 MPC_* 增益 (rosservice, 无需重启)
  - 确定性恒定阶跃风 → 公平可复现评测
  - 逐维坐标下降: 每维扫一组候选, 取综合代价最低者固定, 再扫下一维

评测指标 (稳态段, 风施加后丢弃前 settle 秒):
  cost = err_xy_rms + 0.5*err_z_rms + 0.02*tilt_rms + 0.3*err_xy_max

用法:
  rosrun quad_sim wind_tune.py --wind 5.0 --axis y
  (需 HIL 栈已悬停在 (0,0,2.5))
"""
import argparse
import sys
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.srv import ParamSet, ParamGet
from mavros_msgs.msg import ParamValue
from tf.transformations import euler_from_quaternion


class Tuner:
    def __init__(self, wind_speed=5.0, axis='y', settle=8.0, measure=8.0, rate=50.0):
        rospy.init_node('wind_tune')
        self.wind_speed = wind_speed
        self.axis = axis
        self.settle = settle
        self.measure = measure
        self.dt = 1.0 / rate

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.rpy = np.zeros(3)
        self._got = False

        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_cb)
        self.pub_wind = rospy.Publisher('/wind_field/velocity', Vector3Stamped, queue_size=10)

        rospy.loginfo("等待 MAVROS param 服务...")
        rospy.wait_for_service('/mavros/param/set')
        rospy.wait_for_service('/mavros/param/get')
        self._set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self._get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        rospy.loginfo("等待 odometry...")
        while not self._got and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo("Tuner 就绪")
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
        v = ParamValue()
        v.real = float(value)
        v.integer = 0
        try:
            res = self._set(param_id=name, value=v)
            return res.success
        except rospy.ServiceException as e:
            rospy.logwarn("set %s 失败: %s", name, e)
            return False

    def get_param(self, name):
        try:
            res = self._get(param_id=name)
            return res.value.real if res.success else None
        except rospy.ServiceException:
            return None

    def _wind_vec(self, mag):
        v = Vector3Stamped()
        v.header.stamp = rospy.Time.now()
        if self.axis == 'x':
            v.vector.x = mag
        elif self.axis == 'y':
            v.vector.y = mag
        else:  # diagonal
            v.vector.x = mag / np.sqrt(2)
            v.vector.y = mag / np.sqrt(2)
        return v

    def _publish_wind(self, mag):
        self.pub_wind.publish(self._wind_vec(mag))

    def evaluate(self, label=""):
        """施加恒定风, settle 后采样 measure 秒, 返回指标 dict"""
        target = np.array([0.0, 0.0, 2.5])
        # settle: 持续打风
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < self.settle and not rospy.is_shutdown():
            self._publish_wind(self.wind_speed)
            self.rate.sleep()
        # measure: 持续打风 + 采样
        exy, ez, tilt = [], [], []
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < self.measure and not rospy.is_shutdown():
            self._publish_wind(self.wind_speed)
            e = self.pos - target
            exy.append(np.hypot(e[0], e[1]))
            ez.append(abs(e[2]))
            tilt.append(np.degrees(np.hypot(self.rpy[0], self.rpy[1])))
            self.rate.sleep()
        exy = np.array(exy); ez = np.array(ez); tilt = np.array(tilt)
        m = {
            'exy_rms': float(np.sqrt((exy**2).mean())),
            'exy_max': float(exy.max()),
            'ez_rms': float(np.sqrt((ez**2).mean())),
            'tilt_rms': float(np.sqrt((tilt**2).mean())),
        }
        m['cost'] = (m['exy_rms'] + 0.5*m['ez_rms']
                     + 0.02*m['tilt_rms'] + 0.3*m['exy_max'])
        rospy.loginfo("  [%s] cost=%.3f | exy_rms=%.3f exy_max=%.3f ez_rms=%.3f tilt_rms=%.1f",
                      label, m['cost'], m['exy_rms'], m['exy_max'], m['ez_rms'], m['tilt_rms'])
        return m

    def wind_off(self, secs=6.0):
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < secs and not rospy.is_shutdown():
            self._publish_wind(0.0)
            self.rate.sleep()

    def sweep(self, param, candidates, fixed):
        """扫一个参数, 其余固定; 返回 (best_value, best_metrics, all_results)"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("扫描 %s, 候选 %s", param, candidates)
        for k, v in fixed.items():
            self.set_param(k, v)
        rospy.sleep(1.0)
        results = []
        best = None
        for c in candidates:
            self.set_param(param, c)
            rospy.sleep(0.5)
            self.wind_off(5.0)  # 回到无风稳态再测, 公平
            m = self.evaluate(label="%s=%.3f" % (param, c))
            results.append((c, m))
            if best is None or m['cost'] < best[1]['cost']:
                best = (c, m)
        rospy.loginfo(">>> %s 最优 = %.3f (cost=%.3f)", param, best[0], best[1]['cost'])
        return best[0], best[1], results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wind', type=float, default=5.0)
    ap.add_argument('--axis', default='y', choices=['x', 'y', 'diag'])
    ap.add_argument('--settle', type=float, default=8.0)
    ap.add_argument('--measure', type=float, default=8.0)
    args, _ = ap.parse_known_args()

    t = Tuner(wind_speed=args.wind, axis=args.axis,
              settle=args.settle, measure=args.measure)

    # 出厂默认基线 (MEMORY 验证过的值)
    base = {
        'MPC_XY_P': 0.95,
        'MPC_XY_VEL_P_ACC': 1.8,
        'MPC_XY_VEL_I_ACC': 0.4,
        'MPC_XY_VEL_D_ACC': 0.2,
        'MPC_Z_VEL_P_ACC': 4.0,
        'MPC_Z_VEL_I_ACC': 2.0,
    }
    rospy.loginfo("\n########## 基线评测 (出厂默认) ##########")
    for k, v in base.items():
        t.set_param(k, v)
    rospy.sleep(1.0)
    t.wind_off(5.0)
    base_m = t.evaluate(label="BASELINE")

    best = dict(base)

    # ── 维度1: 速度积分 (抗持续风核心) ──
    v, m, _ = t.sweep('MPC_XY_VEL_I_ACC', [0.4, 0.8, 1.2, 1.6, 2.0], best)
    best['MPC_XY_VEL_I_ACC'] = v

    # ── 维度2: 速度 P (抗扰响应) ──
    v, m, _ = t.sweep('MPC_XY_VEL_P_ACC', [1.8, 2.4, 3.0, 3.6], best)
    best['MPC_XY_VEL_P_ACC'] = v

    # ── 维度3: 位置 P ──
    v, m, _ = t.sweep('MPC_XY_P', [0.95, 1.2, 1.5, 1.8], best)
    best['MPC_XY_P'] = v

    # ── 复测最优组合 ──
    rospy.loginfo("\n########## 最优组合复测 ##########")
    for k, val in best.items():
        t.set_param(k, val)
    rospy.sleep(1.0)
    t.wind_off(5.0)
    final_m = t.evaluate(label="FINAL")
    t.wind_off(4.0)

    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("调参完成")
    rospy.loginfo("基线 cost=%.3f → 最优 cost=%.3f (降 %.0f%%)",
                  base_m['cost'], final_m['cost'],
                  (1 - final_m['cost']/base_m['cost'])*100)
    rospy.loginfo("最优参数:")
    for k, val in best.items():
        rospy.loginfo("  %s = %.3f", k, val)
    rospy.loginfo("=" * 60)
    # 打印成 rosservice 命令方便复用
    print("\n# === 最优参数 rosservice 命令 ===")
    for k, val in best.items():
        print("rosservice call /mavros/param/set \"{param_id: '%s', value: {real: %.3f}}\"" % (k, val))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

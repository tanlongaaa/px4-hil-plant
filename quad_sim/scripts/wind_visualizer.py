#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_visualizer.py — 风场 RViz 可视化节点

订阅:
  /wind_field/velocity  (Vector3Stamped) — 无人机位置处风矢量 ENU
  /sim/odom             (Odometry)        — 无人机位置

发布:
  /wind_viz/markers     (MarkerArray)     — RViz 风场可视化

可视化元素:
  1. 风矢量箭头 (Arrow)           — 彩色, 红=危险高风速, 绿=安全低风速
  2. 风速文字标签 (Text)          — "Wind: XX.X m/s"
  3. 水平面网格风场 (Arrow x 25)  — 5×5 网格展示空间风场分布
  4. 风场历史轨迹 (LineStrip)     — 最近 N 秒的风矢历史
"""

import argparse
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class WindVisualizer:
    def __init__(self, rate_hz=10, grid_size=5, grid_spacing=2.0,
                 u_ref=5.0, z_ref=10.0, alpha=0.35, wind_dir_deg=45.0,
                 gust_w_max=8.0, gust_H=15.0, gust_interval=20.0):
        rospy.init_node('wind_visualizer', anonymous=False)

        self.rate_hz = rate_hz
        self.grid_size = grid_size          # N×N grid
        self.grid_spacing = grid_spacing    # spacing between grid points [m]

        # ── Mean wind parameters (for grid recomputation) ──
        self.u_ref = u_ref
        self.z_ref = z_ref
        self.alpha = alpha
        self.wind_dir = np.deg2rad(wind_dir_deg)

        # ── Gust parameters (for grid) ──
        self.gust_w_max = gust_w_max
        self.gust_H = gust_H
        self.gust_prop_speed = u_ref
        self.gust_interval = gust_interval
        self._gust_active = False
        self._gust_t0 = 0.0
        self._next_gust = np.random.uniform(5.0, 15.0)

        # ── State ────────────────────────────────────────
        self._drone_pos = np.zeros(3)       # ENU
        self._wind_vel_enu = np.zeros(3)    # wind at drone [We,Wn,Wu]
        self._wind_mag = 0.0
        self._got_odom = False
        self._got_wind = False
        self._t0 = rospy.Time.now().to_sec()

        # ── Wind history (trail) ──
        self._history_max = 200             # max points in trail
        self._history_pos = []              # [(x,y,z), ...]
        self._history_wind = []             # [(we,wn,wu), ...]

        # ── Subscribers ──────────────────────────────────
        rospy.Subscriber('/wind_field/velocity', Vector3Stamped, self._wind_cb)
        rospy.Subscriber('/sim/odom', Odometry, self._odom_cb)

        # ── Publisher ────────────────────────────────────
        self._marker_pub = rospy.Publisher(
            '/wind_viz/markers', MarkerArray, queue_size=5)

        rospy.loginfo("🌪️  Wind Visualizer ready @ %d Hz  grid=%dx%d  spacing=%.1fm",
                      rate_hz, grid_size, grid_size, grid_spacing)

    def _wind_cb(self, msg):
        self._wind_vel_enu[0] = msg.vector.x
        self._wind_vel_enu[1] = msg.vector.y
        self._wind_vel_enu[2] = msg.vector.z
        self._wind_mag = float(np.linalg.norm(self._wind_vel_enu))
        self._got_wind = True

    def _odom_cb(self, msg):
        self._drone_pos[0] = msg.pose.pose.position.x
        self._drone_pos[1] = msg.pose.pose.position.y
        self._drone_pos[2] = msg.pose.pose.position.z
        self._got_odom = True

    # ══════════════════════════════════════════════════════
    #  Mean wind model (deterministic, for grid)
    # ══════════════════════════════════════════════════════

    def _mean_wind_enu(self, altitude):
        """Power-law mean wind at given altitude."""
        z = max(altitude, 0.5)
        u_mag = self.u_ref * (z / self.z_ref) ** self.alpha
        We = -u_mag * np.sin(self.wind_dir)
        Wn = -u_mag * np.cos(self.wind_dir)
        return np.array([We, Wn, 0.0])

    def _gust_enu(self, sim_time):
        """1-cos vertical gust (matching WindField logic)."""
        Wu_g = 0.0
        if not self._gust_active and sim_time >= self._next_gust:
            self._gust_active = True
            self._gust_t0 = sim_time
            rospy.loginfo("💨 [Viz] Gust triggered at t=%.1f", sim_time)

        if self._gust_active:
            dt_g = sim_time - self._gust_t0
            duration = 2.0 * self.gust_H / self.gust_prop_speed
            if dt_g <= duration:
                x = self.gust_prop_speed * (dt_g - duration / 2.0)
                Wu_g = self.gust_w_max * 0.5 * (1.0 - np.cos(np.pi * x / self.gust_H))
            else:
                self._gust_active = False
                self._next_gust = sim_time + np.random.uniform(
                    self.gust_interval * 0.7, self.gust_interval * 1.3)
                rospy.loginfo("💨 [Viz] Gust ended at t=%.1f", sim_time)
        return Wu_g

    def _compute_grid_wind(self, center_xy, altitude, sim_time):
        """Compute mean+gust wind at grid points around center_xy."""
        W_mean = self._mean_wind_enu(altitude)
        Wu_g = self._gust_enu(sim_time)
        W_grid = W_mean.copy()
        W_grid[2] += Wu_g
        return W_grid

    # ══════════════════════════════════════════════════════
    #  Color mapping: wind speed → color
    # ══════════════════════════════════════════════════════

    @staticmethod
    def _wind_color(speed, max_speed=15.0):
        """Green (0) → Yellow (7.5) → Red (15+) m/s"""
        t = min(speed / max_speed, 1.0)
        if t < 0.5:
            # green → yellow
            r = 2.0 * t
            g = 1.0
        else:
            # yellow → red
            r = 1.0
            g = 2.0 * (1.0 - t)
        return ColorRGBA(r=r, g=g, b=0.1, a=0.9)

    # ══════════════════════════════════════════════════════
    #  Marker builders
    # ══════════════════════════════════════════════════════

    def _make_arrow(self, idx, ns, pos, vector, color, scale=None, lifetime=None):
        """Create a single Arrow marker."""
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = idx
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position = Point(*pos)
        # Quaternion: default arrow points +X, we rotate to vector direction
        v = np.asarray(vector)
        mag = float(np.linalg.norm(v))
        if mag < 1e-6:
            v = np.array([1.0, 0.0, 0.0])
            mag = 1.0
        # Quaternion from +X to vector direction
        x_axis = np.array([1.0, 0.0, 0.0])
        axis = np.cross(x_axis, v / mag)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-6:
            q = (0.0, 0.0, 0.0, 1.0)  # parallel
        else:
            axis = axis / axis_norm
            angle = np.arccos(np.clip(np.dot(x_axis, v / mag), -1.0, 1.0))
            half = angle / 2.0
            q = (axis[0] * np.sin(half),
                 axis[1] * np.sin(half),
                 axis[2] * np.sin(half),
                 np.cos(half))
        m.pose.orientation = Quaternion(*q)

        if scale is None:
            scale = Vector3(0.15, 0.02, 0.02)  # shaft_w, head_w, head_l, scaled
            m.scale = Vector3(mag, 0.12, 0.18)  # length=mag, head_w, head_l
        else:
            m.scale = scale

        m.color = color
        if lifetime is not None:
            m.lifetime = rospy.Duration(lifetime)
        return m

    def _make_text(self, idx, ns, pos, text, color=None, scale=0.5, lifetime=None):
        """Create a Text marker."""
        if color is None:
            color = ColorRGBA(1.0, 1.0, 1.0, 0.9)
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(*pos)
        m.scale.z = scale
        m.text = text
        m.color = color
        if lifetime is not None:
            m.lifetime = rospy.Duration(lifetime)
        return m

    def _make_linestrip(self, idx, ns, points, color, scale=0.03, lifetime=None):
        """Create a LineStrip marker."""
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.points = [Point(*p) for p in points]
        m.scale.x = scale
        m.color = color
        if lifetime is not None:
            m.lifetime = rospy.Duration(lifetime)
        return m

    # ══════════════════════════════════════════════════════
    #  Main publish cycle
    # ══════════════════════════════════════════════════════

    def publish(self):
        if not self._got_wind or not self._got_odom:
            return

        t_now = rospy.Time.now().to_sec()
        sim_t = t_now - self._t0
        markers = MarkerArray()
        mid = 0

        p = self._drone_pos
        w = self._wind_vel_enu
        wmag = self._wind_mag

        # ── 1. Wind arrow at drone ──────────────────────
        markers.markers.append(self._make_arrow(
            mid, "wind_drone_arrow", p, w,
            self._wind_color(wmag, max_speed=15.0),
            scale=Vector3(wmag * 0.8, 0.15, 0.22),
            lifetime=1.0 / self.rate_hz * 2.5))
        mid += 1

        # ── 2. Wind speed text above drone ──────────────
        text_pos = p + np.array([0, 0, 1.2])
        markers.markers.append(self._make_text(
            mid, "wind_text", text_pos,
            "Wind: %.1f m/s | Gnd: %.1f" % (wmag, np.linalg.norm(w[:2])),
            self._wind_color(wmag, max_speed=15.0),
            scale=0.6,
            lifetime=1.0 / self.rate_hz * 2.5))
        mid += 1

        # ── 3. Wind components text ─────────────────────
        comp_pos = p + np.array([0, 0, 0.8])
        markers.markers.append(self._make_text(
            mid, "wind_components", comp_pos,
            "E:%.1f N:%.1f U:%.1f" % (w[0], w[1], w[2]),
            ColorRGBA(0.7, 0.8, 1.0, 0.8),
            scale=0.35,
            lifetime=1.0 / self.rate_hz * 2.5))
        mid += 1

        # ── 4. Grid wind field (5×5 horizontal plane) ──
        # Center grid around drone, at drone altitude
        grid_alt = p[2]
        grid_wind = self._compute_grid_wind(p[:2], grid_alt, sim_t)
        half = (self.grid_size - 1) * self.grid_spacing / 2.0
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                gx = p[0] + (i * self.grid_spacing - half)
                gy = p[1] + (j * self.grid_spacing - half)
                gpos = np.array([gx, gy, grid_alt])
                # Scale arrow to show wind magnitude
                gmag = float(np.linalg.norm(grid_wind))
                markers.markers.append(self._make_arrow(
                    mid, "wind_grid", gpos, grid_wind,
                    ColorRGBA(0.3, 0.6, 1.0, 0.5),
                    scale=Vector3(max(gmag * 0.6, 0.3), 0.06, 0.10),
                    lifetime=1.0 / self.rate_hz * 2.5))
                mid += 1

        # Grid text at first cell (top-left for speed read)
        grid_text_pos = np.array([
            p[0] - half, p[1] - half, grid_alt + 0.5
        ])
        markers.markers.append(self._make_text(
            mid, "wind_grid_info", grid_text_pos,
            "Mean: %.1f m/s @ %.1fm" % (
                float(np.linalg.norm(grid_wind)), grid_alt),
            ColorRGBA(0.4, 0.7, 1.0, 0.7),
            scale=0.35,
            lifetime=1.0 / self.rate_hz * 2.5))
        mid += 1

        # ── 5. Wind history trail (colored line strip) ──
        self._history_pos.append(p.copy())
        self._history_wind.append(w.copy())
        if len(self._history_pos) > self._history_max:
            self._history_pos.pop(0)
            self._history_wind.pop(0)

        # Draw trail as line strip, color by wind magnitude
        # We split into segments for color variation
        history = list(zip(self._history_pos, self._history_wind))
        n_hist = len(history)
        if n_hist >= 2 and n_hist % 2 == 0:
            # Single line strip for whole trail (simplest)
            markers.markers.append(self._make_linestrip(
                mid, "wind_trail", self._history_pos,
                ColorRGBA(0.2, 0.8, 1.0, 0.4),
                scale=0.05,
                lifetime=1.0 / self.rate_hz * 2.5))
            mid += 1

            # Colored dots along trail (by wind magnitude)
            step = max(1, n_hist // 30)
            for k in range(0, n_hist, step):
                pp, ww = history[k]
                ww_mag = float(np.linalg.norm(ww))
                dot = Marker()
                dot.header.frame_id = "map"
                dot.header.stamp = rospy.Time.now()
                dot.ns = "wind_trail_dots"
                dot.id = mid
                dot.type = Marker.SPHERE
                dot.action = Marker.ADD
                dot.pose.position = Point(*pp)
                dot.scale = Vector3(0.08, 0.08, 0.08)
                dot.color = self._wind_color(ww_mag, max_speed=15.0)
                dot.lifetime = rospy.Duration(1.0 / self.rate_hz * 2.5)
                markers.markers.append(dot)
                mid += 1

        # ── 6. Legend text (static, top-left corner in world) ──
        if sim_t < 0.5:  # only send once
            legend_y = 0.0
            for label, col in [
                ("🟢 <5 m/s  Calm", ColorRGBA(0.2, 1.0, 0.2, 0.9)),
                ("🟡 5-10 m/s Moderate", ColorRGBA(1.0, 1.0, 0.2, 0.9)),
                ("🔴 >10 m/s Extreme", ColorRGBA(1.0, 0.2, 0.2, 0.9)),
            ]:
                markers.markers.append(self._make_text(
                    mid, "wind_legend", np.array([0, legend_y, 5.0]),
                    label, col, scale=0.4, lifetime=999))
                mid += 1
                legend_y -= 0.5

        self._marker_pub.publish(markers)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


# ══════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='风场 RViz 可视化节点')
    parser.add_argument('--rate', type=int, default=10,
                        help='发布频率 Hz (默认 10)')
    parser.add_argument('--grid-size', type=int, default=5,
                        help='网格点数 N×N (默认 5)')
    parser.add_argument('--grid-spacing', type=float, default=2.0,
                        help='网格间距 m (默认 2.0)')
    parser.add_argument('--u-ref', type=float, default=5.0,
                        help='参考风速 m/s (默认 5)')
    parser.add_argument('--wind-dir', type=float, default=45.0,
                        help='风向 ° (默认 45, 东北风)')
    # parse_known_args 忽略 ROS 附加参数 (__name:=, __log:=)
    args, _unknown_ros = parser.parse_known_args()

    try:
        WindVisualizer(
            rate_hz=args.rate,
            grid_size=args.grid_size,
            grid_spacing=args.grid_spacing,
            u_ref=args.u_ref,
            wind_dir_deg=args.wind_dir,
        ).run()
    except rospy.ROSInterruptException:
        pass

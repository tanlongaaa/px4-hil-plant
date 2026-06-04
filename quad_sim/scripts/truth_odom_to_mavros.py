#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def param_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class TruthOdomToMavros:
    def __init__(self):
        rospy.init_node("truth_odom_to_mavros")

        self.input_topic = rospy.get_param("~input_topic", "/sim/odom")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/odometry/in")
        self.vision_pose_topic = rospy.get_param("~vision_pose_topic", "/mavros/vision_pose/pose")
        self.enable_mavros_injection = param_bool(rospy.get_param("~enable_mavros_injection", False))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        self.pos_std = float(rospy.get_param("~pos_std", 0.02))
        self.z_std = float(rospy.get_param("~z_std", 0.03))
        self.angle_std_deg = float(rospy.get_param("~angle_std_deg", 2.0))
        self.vel_std = float(rospy.get_param("~vel_std", 0.05))
        self.ang_vel_std_deg = float(rospy.get_param("~ang_vel_std_deg", 5.0))

        if not self.enable_mavros_injection:
            rospy.logwarn(
                "truth_odom_to_mavros is disabled. Set ~enable_mavros_injection:=true "
                "only when intentionally feeding external odometry/vision into PX4."
            )
            return

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.vision_pose_pub = rospy.Publisher(self.vision_pose_topic, PoseStamped, queue_size=20)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_cb, queue_size=20)

        rospy.loginfo("truth_odom_to_mavros: %s -> %s", self.input_topic, self.output_topic)
        rospy.loginfo("truth_odom_to_mavros vision pose: %s -> %s", self.input_topic, self.vision_pose_topic)

    def _pose_covariance(self):
        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = self.pos_std ** 2
        cov[1, 1] = self.pos_std ** 2
        cov[2, 2] = self.z_std ** 2
        cov[3, 3] = math.radians(self.angle_std_deg) ** 2
        cov[4, 4] = math.radians(self.angle_std_deg) ** 2
        cov[5, 5] = math.radians(self.angle_std_deg) ** 2
        return cov.reshape(-1).tolist()

    def _twist_covariance(self):
        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = self.vel_std ** 2
        cov[1, 1] = self.vel_std ** 2
        cov[2, 2] = self.vel_std ** 2
        cov[3, 3] = math.radians(self.ang_vel_std_deg) ** 2
        cov[4, 4] = math.radians(self.ang_vel_std_deg) ** 2
        cov[5, 5] = math.radians(self.ang_vel_std_deg) ** 2
        return cov.reshape(-1).tolist()

    def odom_cb(self, msg):
        out = Odometry()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        out.pose.covariance = self._pose_covariance()
        out.twist.covariance = self._twist_covariance()
        self.pub.publish(out)

        pose = PoseStamped()
        pose.header = out.header
        pose.pose = out.pose.pose
        self.vision_pose_pub.publish(pose)


if __name__ == "__main__":
    try:
        TruthOdomToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
"""Step wind test: 15s no wind → 30s step wind 5m/s → 45s recovery"""
import rospy
from geometry_msgs.msg import Vector3Stamped

rospy.init_node("step_wind")
pub = rospy.Publisher("/wind_field/velocity", Vector3Stamped, queue_size=10)
rate = rospy.Rate(10)

# 0-15s: no wind
for i in range(150):
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.vector.x = 0.0
    msg.vector.y = 0.0
    msg.vector.z = 0.0
    pub.publish(msg)
    rate.sleep()

rospy.loginfo("🌬️  Step wind ON: 5 m/s from south (positive Y)")
# 15-45s: step wind 5 m/s ENU +Y (from south)
for i in range(300):
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.vector.x = 0.0
    msg.vector.y = 5.0
    msg.vector.z = 0.0
    pub.publish(msg)
    rate.sleep()

rospy.loginfo("☀️  Wind OFF — observing recovery")
# 45-90s: wind off, recovery
for i in range(450):
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.vector.x = 0.0
    msg.vector.y = 0.0
    msg.vector.z = 0.0
    pub.publish(msg)
    rate.sleep()

rospy.loginfo("✅ Step wind test complete")

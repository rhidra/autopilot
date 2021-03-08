#!/usr/bin/env python

import rospy, numpy as np
from geometry_msgs.msg import TwistStamped

rospy.init_node('test', anonymous=True)
pub = rospy.Publisher('reference/setpoint', TwistStamped, queue_size=10)

rate = rospy.Rate(30)

f = .0004
f0 = .03
R = .8
R0 = 5
wR = 2 * np.pi * .5

w0 = 2 * np.pi * f0
dw = 2 * np.pi * f

start = rospy.Time.now()
while not rospy.is_shutdown():
    t = (rospy.Time.now() - start).to_sec()

    w = dw * t + w0

    r = R * np.cos(wR * t) + R0
    dr = - wR * R * np.sin(wR * t)

    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.twist.angular.x = r * np.cos(w * t)
    msg.twist.angular.y = r * np.sin(w * t)
    msg.twist.angular.z = 5.

    msg.twist.linear.x = dr * np.cos(w * t) - r * (dw * t + w) * np.sin(w * t)
    msg.twist.linear.y = dr * np.sin(w * t) + r * (dw * t + w) * np.cos(w * t)
    msg.twist.linear.z = 0.
    
    pub.publish(msg)
    rate.sleep()
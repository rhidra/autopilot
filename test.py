#!/usr/bin/env python

import rospy, numpy as np
from geometry_msgs.msg import TwistStamped
from controller_msgs.msg import FlatTarget
from std_msgs.msg import Float32

rospy.init_node('test', anonymous=True)
pub = rospy.Publisher('reference/flatsetpoint', FlatTarget, queue_size=10)
pub_yaw = rospy.Publisher('reference/yaw', Float32, queue_size=10)

rate = rospy.Rate(30)

f = .0004
f0 = .0001
R = .8
R0 = 5
wR = 2 * np.pi * .5

w0 = 2 * np.pi * f0

dw = 2 * np.pi * f
ddw = 0

start = rospy.Time.now()
while not rospy.is_shutdown():
    t = (rospy.Time.now() - start).to_sec()

    w = dw * t + w0

    r = R0
    dr = 0
    ddr = 0

    vx = dr * np.cos(w * t) - r * (dw * t + w) * np.sin(w * t)
    vy = dr * np.sin(w * t) + r * (dw * t + w) * np.cos(w * t)
    yaw = np.arctan2(vy, vx)

    msg = FlatTarget()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.type_mask = FlatTarget.IGNORE_SNAP_JERK
    msg.position.x = r * np.cos(w * t)
    msg.position.y = r * np.sin(w * t)
    msg.position.z = 5.

    msg.velocity.x = vx
    msg.velocity.y = vy
    msg.velocity.z = 0.

    msg.acceleration.x = ddr * np.cos(w*t) - dr * (dw*t+w) * np.sin(w*t) - dr * (dw*t+w) * np.sin(w*t) - r * (ddw*t+2*dw) * np.sin(w*t) - r * (dw*t+w)**2 * np.cos(w*t)
    msg.acceleration.y = ddr * np.cos(w*t) + dr * (dw*t+w) * np.cos(w*t) + dr * (dw*t+w) * np.cos(w*t) + r * (ddw*t+2*dw) * np.cos(w*t) - r * (dw*t+w)**2 * np.sin(w*t)
    msg.acceleration.z = 0.
    
    pub.publish(msg)

    msg = Float32()
    msg.data = yaw
    pub_yaw.publish(msg)
    
    rate.sleep()
#!/usr/bin/env python

import rospy, numpy as np
from sensor_msgs.msg import Imu

accs = []

def cb(msg):
    acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    accs.append(acc)

def shutdown():
    np.save('./acc_data.npy', np.array(accs))

rospy.init_node('plot_acc', anonymous=True)
sub = rospy.Subscriber('mavros/imu/data', Imu, cb)

rospy.on_shutdown(shutdown)
rospy.spin()
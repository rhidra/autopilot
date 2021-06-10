#!/usr/bin/env python

"""
traj_viz

Small utility to visualize the real trajectory of the UAV in a 3D Rviz environment.
"""

import rospy, numpy as np
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

rospy.init_node('traj_viz', anonymous=True)
pub = rospy.Publisher('/autopilot/traj_viz', Path, queue_size=10)
start_pos = np.array([0, 0, 0])
traj = Path()
traj.header.stamp = rospy.Time.now()
traj.header.frame_id = '/map'

i = 0
def cb(data):
    data.pose.position.x += start_pos[0]
    data.pose.position.y += start_pos[1]
    data.pose.position.z += start_pos[2]

    traj.poses.append(data)

    global i
    i += 1
    if i % 10 == 0:
        pub.publish(traj)

    if i % 200 == 0:
        s = 0
        for j in range(len(traj.poses) - 1):
            a, b = traj.poses[j].pose, traj.poses[j + 1].pose
            a = np.array([a.position.x, a.position.y, a.position.z])
            b = np.array([b.position.x, b.position.y, b.position.z])
            s += np.linalg.norm(b - a)
        print('Distance: {}'.format(s))


start_pos = np.array([float(rospy.get_param('/start/x', 0)), float(rospy.get_param('/start/y', 0)), float(rospy.get_param('/start/init_z', 0))])
sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, cb)
rospy.spin()

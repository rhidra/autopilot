#!/usr/bin/env python

"""
octomap_slicer

Small utility to slice the display of an octomap on Rviz.
Publish the new octomap on /octomap_sliced
"""

import rospy
from visualization_msgs.msg import MarkerArray

pub = rospy.Publisher('/octomap_sliced', MarkerArray, queue_size=10)

def cb(data):
    print('Receiving !')
    for m in data.markers:
        t = []
        c = []
        for pt, colors in zip(m.points, m.colors):
            if pt.z < 1.5:
                t.append(pt)
                c.append(colors)
        m.points = t
        m.colors = c
    print('Sending !')
    pub.publish(data)

rospy.init_node('offset_poses', anonymous=True)
sub = rospy.Subscriber('/occupied_cells_vis_array', MarkerArray, cb)
rospy.spin()
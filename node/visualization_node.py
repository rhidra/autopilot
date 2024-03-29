#!/usr/bin/env python
import rospy, math, numpy as np
from base_node import BaseNode
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class VisualizationNode(BaseNode):
    def setup(self):
        super(VisualizationNode, self).setup()
        self.viz_global_pub = rospy.Publisher('/autopilot/viz/global', MarkerArray, queue_size=10) # Custom topic used with Rviz
        self.viz_local_pub = rospy.Publisher('/autopilot/viz/local', MarkerArray, queue_size=10) # Custom topic used with Rviz
        self.rate.sleep()

    """
    visualize_global_path()
    Publish a local path to a ROS topic, in order to be visualize by the Rviz visualizer.
    @param path: [[x, y, z]] 3D path in local coordinates.
    """
    def visualize_global_path(self, path=[], path2=[], nodes=[], start=None, goal=None, point=None):
        if not hasattr(self, 'temp_marker'):
            self.temp_marker = []

        for marker in self.temp_marker:
            marker.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers.extend(self.temp_marker)
        self.temp_marker = []

        if len(path) > 0:
            marker_array.markers.append(viz_path(path))
            for i, pt in enumerate(path):
                m = viz_point(pt, color=(0, 1, 1), id=10 + i, size=.1)
                self.temp_marker.append(m)
                marker_array.markers.append(m)
        if len(path2) > 0:
            marker_array.markers.append(viz_path(path2, color=(1, 0, 0), id=1e5))
        if len(nodes) > 0:
            marker_array.markers.append(viz_nodes(nodes))
            for i, node in enumerate(nodes):
                m = viz_point(node.pos, color=(0, 1, .5), id=100 + i, size=.05)
                self.temp_marker.append(m)
                marker_array.markers.append(m)
        if start is not None:
            marker_array.markers.append(viz_point(start, color=(0, 1, 0), id=0, size=.3))
        if goal is not None:
            marker_array.markers.append(viz_point(goal, color=(0, 0, 1), id=1, size=.3))
        if point is not None:
            marker_array.markers.append(viz_point(point, color=(1, 0, 1), id=2, size=.3))

        self.viz_global_pub.publish(marker_array)


    """
    visualize_local_path()
    Publish a local path to a ROS topic, in order to be visualized by the Rviz visualizer.
    """
    def visualize_local_path(self, trajLibrary=[], trajSelected=None, trajHistory=[], tf=1):
        if not hasattr(self, 'temp_marker'):
            self.temp_marker = []

        for marker in self.temp_marker:
            marker.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers.extend(self.temp_marker)
        self.temp_marker = []

        t = np.linspace(0, tf, 10)
        maxCost = np.max([traj._cost for traj in trajLibrary] + [0])
        maxVel = np.max([np.linalg.norm(traj.get_velocity(traj._tf)) for traj in trajLibrary] + [0])
        for i, traj in enumerate(trajLibrary):
            pos = traj.get_position(t)
            vel = traj.get_velocity(t)
            maxCost = 1000
            d = traj._cost / maxCost if traj._cost < maxCost else 1
            # d = np.linalg.norm(traj.get_velocity(traj._tf)) / maxVel
            # if traj._cost >= maxCost:
            #     marker_array.markers.append(clean_marker(id=i, ns='path'))
            #     continue
            c = np.array([0, 1, 0]) * (1 - d) + np.array([1, 0, 0]) * d
            m = viz_path(pos, color=c if traj is not trajSelected else (0,1,1), id=i, width=.08 if traj is trajSelected else .03, alpha=1 if traj is trajSelected else .1)
            marker_array.markers.append(m)
            marker_array.markers.append(viz_point(pos[-1], color=c, id=i+1, size=.1, alpha=.1))
            if traj is trajSelected:
                for iv, (v, p) in enumerate(zip(vel, pos)):
                    marker_array.markers.append(viz_arrow(p, p + v*.4, color=(242/255., 198/255., 22/255.), id=100+iv, size=.03))
        marker_array.markers.append(viz_point(pos[0], color=(1, 1, 1), id=0, size=.2, alpha=.5))

        for i, traj in enumerate(trajHistory):
            pos = traj.get_position(t)
            m = viz_path(pos, color=(.9, .9, .9), id=i, ns='hist', width=.03)
            marker_array.markers.append(m)
            marker_array.markers.append(viz_point(traj.local_goal, size=.2, id=i, ns='hist'))
            marker_array.markers.append(viz_arrow(pos[-1], traj.local_goal, (0,0,0), size=.02, id=i, ns='hist'))

        self.viz_local_pub.publish(marker_array)


def viz_path(path, color=(0, 1, 0), id=0, width=.06, alpha=1, ns=''):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'path_' + ns
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.id = id
    marker.type = Marker.LINE_LIST

    marker.scale.x = width
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = alpha

    prev = path[0]
    for curr in path[1:]:
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = prev
        p2.x, p2.y, p2.z = curr

        marker.points.append(p1)
        marker.points.append(p2)
        prev = curr

    return marker


def viz_nodes(nodes):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'nodes'
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.id = 1
    marker.type = Marker.LINE_LIST

    marker.scale.x = 0.03
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1

    for node in nodes:
        if node.parent is None:
            continue

        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = node.parent.pos
        p2.x, p2.y, p2.z = node.pos
        marker.points.append(p1)
        marker.points.append(p2)

    return marker


def viz_point(point, color=(1., 1., 1.), id=0, size=.5, alpha=1, ns=''):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'points_' + ns
    marker.action = Marker.ADD
    marker.id = id
    marker.type = Marker.SPHERE

    marker.pose.position.x = float(point[0])
    marker.pose.position.y = float(point[1])
    marker.pose.position.z = float(point[2])
    marker.pose.orientation.w = 1.0

    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = alpha

    return marker


def viz_arrow(start, end, color=(1, 1, 1), id=0, size=.1, ns=''):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'arrows_' + ns
    marker.action = Marker.ADD
    marker.id = id
    marker.type = Marker.ARROW


    a, b = Point(), Point()
    a.x, a.y, a.z = start[0], start[1], start[2]
    b.x, b.y, b.z = end[0], end[1], end[2]
    marker.points.append(a)
    marker.points.append(b)

    marker.scale.x = size
    marker.scale.y = size * 1.7
    marker.scale.z = size * 1.7

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1

    return marker

def clean_marker(id=0, ns=''):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'path_' + ns
    marker.action = Marker.DELETE
    marker.id = id
    return marker

#!/usr/bin/env python
import rospy, math
from base_node import BaseNode
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class VisualizationNode(BaseNode):
    def setup(self):
        super(VisualizationNode, self).setup()
        self.path_viz_pub = rospy.Publisher('/path_viz', MarkerArray, queue_size=10) # Custom topic used with Rviz

    """
    visualize_local_path()
    Publish a local path to a ROS topic, in order to be visualize by the Rviz visualizer.
    @param path: [[x, y, z]] 3D path in local coordinates.
    """
    def visualize_path(self, path=[], nodes=[], start=None, goal=None, point=None):
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
                m = viz_point(pt, color=(0, 1, 1), id=10 + i, size=.5)
                self.temp_marker.append(m)
                marker_array.markers.append(m)
        if len(nodes) > 0:
            marker_array.markers.append(viz_nodes(nodes))
            for i, node in enumerate(nodes):
                m = viz_point(node.pos, color=(0, 1, .5), id=100 + i, size=.3)
                self.temp_marker.append(m)
                marker_array.markers.append(m)
        if start is not None:
            marker_array.markers.append(viz_point(start, color=(0, 1, 0), id=0))
        if goal is not None:
            marker_array.markers.append(viz_point(goal, color=(0, 0, 1), id=1))
        if point is not None:
            marker_array.markers.append(viz_point(point, color=(1, 0, 1), id=2))

        self.path_viz_pub.publish(marker_array)


def viz_path(path):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'path'
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.id = 0
    marker.type = Marker.LINE_LIST

    marker.scale.x = 0.1
    marker.color.g = 1
    marker.color.a = 1

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


def viz_point(point, color=(1., 1., 1.), id=0, size=1):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'points'
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
    marker.color.a = 1

    return marker
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy


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

def viz_point(point, color=(1., 1., 1.), id=0):
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

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1

    return marker

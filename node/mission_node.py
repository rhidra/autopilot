#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from base_node import BaseNode
from path_utils import local_to_global, build_waypoints
from visualization import viz_path, viz_nodes, viz_point
from visualization_msgs.msg import MarkerArray


class MissionNode(BaseNode):
    def __init__(self, *args, **kwargs):
        super(MissionNode, self).__init__(*args, **kwargs)
        self.waypoints = []

    def setup(self):
        super(MissionNode, self).setup()
        self.clear_wps()

    def load_local_path(self, path):
        # Visualize the path in Rviz
        self.visualize_path(path)
        self.path = path

        # Convert the path to global GPS coordinates
        path_global = local_to_global(path)

        # Convert the path as a MAVROS mission
        waypoints = build_waypoints(path_global)
        self.waypoints = waypoints

    """
    visualize_local_path()
    Publish a local path to a ROS topic, in order to be visualize by the Rviz visualizer.
    @param path: [[x, y, z]] 3D path in local coordinates.
    """
    def visualize_path(self, path=[], nodes=[], start=None, goal=None, point=None):
        marker_array = MarkerArray()
        if len(path) > 0:
            marker_array.markers.append(viz_path(path))
        if len(nodes) > 0:
            marker_array.markers.append(viz_nodes(nodes))
        if start is not None:
            marker_array.markers.append(viz_point(start, color=(0, 1, 0), id=0))
        if goal is not None:
            marker_array.markers.append(viz_point(goal, color=(0, 0, 1), id=1))
        if point is not None:
            marker_array.markers.append(viz_point(point, color=(1, 0, 1), id=2))

        self.path_viz_pub.publish(marker_array)


    def exec_mission(self):
        self.clear_wps()
        self.send_wps(self.waypoints)

        self.set_mode('AUTO.MISSION')
        self.set_arm(True)

        while not rospy.is_shutdown():
            self.visualize_path(path=self.path)
            self.rate.sleep()

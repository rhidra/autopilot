#!/usr/bin/env python
import rospy, math, octomap, numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from base_node import BaseNode


class MissionNode(BaseNode):
    def __init__(self, *args, **kwargs):
        super(MissionNode, self).__init__(*args, **kwargs)
        self.waypoints = []

    def setup(self):
        super(MissionNode, self).setup()
        self.clear_wps()

    def octomap_cb(self, *args, **kwargs):
        super(MissionNode, self).octomap_cb(*args, **kwargs)

        # Read the octomap binary data and load it in the octomap wrapper class
        data = np.array(self.octomap.data, dtype=np.int8).tostring()
        s = '# Octomap OcTree binary file\nid {}\n'.format(self.octomap.id)
        s += 'size 42\nres {}\ndata\n'.format(self.octomap.resolution)
        s += data

        # An error is triggered because a wrong tree size has been specified in the
        # header. We did not find a way to extract the tree size from the octomap msg
        tree = octomap.OcTree(self.octomap.resolution)
        tree.readBinary(s)
        self.octree = tree

    def check_point(self, point):
        node = self.octree.search(point)
        return self.octree.isNodeOccupied(node)

    """
    load_waypoints()
    Convert a list of 3D points to a list of MAVLink waypoint.
    Store the mission in self.mission

    @param points: [[lat, lon, alt]] 3D points in the global coordinate system
    """
    def load_waypoints(self, path):
        assert len(path) > 0, 'The path is empty !'

        waypoints = []
        for lat, lon, alt in path:
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = CommandCode.NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            waypoints.append(wp)

        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = CommandCode.NAV_LAND
        wp.is_current = False
        wp.autocontinue = True
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        waypoints.append(wp)
        waypoints[0].command = CommandCode.NAV_TAKEOFF
        waypoints[0].is_current = True

        self.waypoints = waypoints

    def exec_mission(self):
        self.clear_wps()
        self.send_wps(self.waypoints)

        self.set_mode('AUTO.MISSION')
        self.set_arm(True)

        while not rospy.is_shutdown():
            self.rate.sleep()

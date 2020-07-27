#!/usr/bin/env python
import rospy, math
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

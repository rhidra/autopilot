#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from node_base import MavrosNodeBase


class NodeBasic(MavrosNodeBase):
    def setup(self):
        super(NodeBasic, self).setup()
        self.clear_wps()

    def main(self):
        super(NodeBasic, self).main()
        count = 0

        wps = []

        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = CommandCode.NAV_TAKEOFF
        wp.is_current = False
        wp.autocontinue = True
        wp.x_lat = 47.397742
        wp.y_long = 8.5455934
        wp.z_alt = 25
        wps.append(wp)

        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = CommandCode.NAV_WAYPOINT
        wp.is_current = False
        wp.autocontinue = True
        wp.x_lat = 47.397743
        wp.y_long = 8.54559
        wp.z_alt = 30
        wps.append(wp)

        self.clear_wps()
        self.send_wps(wps)

        self.set_mode('AUTO.MISSION')
        self.set_arm(True)

        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    node = NodeBasic(node_name='basic_node')
    node.main()

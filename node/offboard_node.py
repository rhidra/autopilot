#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from base_node import BaseNode
from path_utils import local_to_global, build_waypoints, fix_path_orientation


class OffboardNode(BaseNode):
    def __init__(self, *args, **kwargs):
        super(OffboardNode, self).__init__(*args, **kwargs)

    def setup(self):
        super(OffboardNode, self).setup()

    def load_local_path(self, path):
        # Visualize the path in Rviz
        self.viz_path = path
        self.visualize_path(self.viz_path)
        self.path = path
        # self.path = fix_path_orientation(path)

    def exec_mission(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        for i in range(100):
            self.position_pub.publish(msg)
            self.rate.sleep()

        self.set_mode('OFFBOARD')
        self.set_arm(True)

        last = rospy.Time.now().secs
        currentNode = 0

        while not rospy.is_shutdown():
            rospy.loginfo('viz path:' + str(self.viz_path))
            rospy.loginfo('path:' + str(self.path))
            self.visualize_path(path=self.viz_path)

            now = rospy.Time.now().secs
            if last + 5 < now and currentNode + 1 < len(self.path):
                last = now
                currentNode += 1

            msg.header.stamp = rospy.Time.now()

            msg.pose.position.x = self.path[currentNode][0]
            msg.pose.position.y = self.path[currentNode][1]
            msg.pose.position.z = self.path[currentNode][2]
            
            self.position_pub.publish(msg)
            self.rate.sleep()

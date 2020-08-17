#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import local_to_global, build_waypoints, fix_path_orientation, remove_start_offset


class OffboardNode(OctomapNode):
    def load_local_path(self, path):
        # Visualize the path in Rviz
        self.viz_path = path
        self.visualize_path(self.viz_path)
        self.path = remove_start_offset(path)
        # self.path = fix_path_orientation(path)

    def dist_from(self, p, sqrt=False):
        sqr = (p[0] - self.local_position.pose.position.x)**2 + \
              (p[1] - self.local_position.pose.position.y)**2 + \
              (p[2] - self.local_position.pose.position.z)**2
        return math.sqrt(sqr) if sqrt else sqr
        

    def exec_mission(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 2
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        for i in range(100):
            msg.header.stamp = rospy.Time.now()
            self.position_pub.publish(msg)
            self.rate.sleep()

        last = rospy.Time.now().secs
        currentNode = 0
        done = False

        while not rospy.is_shutdown() and not done:
            self.try_set_mode('OFFBOARD')
            self.try_set_arm(True)

            self.visualize_path(path=self.viz_path)

            now = rospy.Time.now().secs
            d = self.dist_from(self.path[currentNode])
            rospy.loginfo('Distance from waypoint: ' + str(d))
            if d < 1. and currentNode + 1 < len(self.path):
                last = now
                currentNode += 1
            elif currentNode + 1 == len(self.path):
                done = True

            msg.header.stamp = rospy.Time.now()

            msg.pose.position.x = self.path[currentNode][0]
            msg.pose.position.y = self.path[currentNode][1]
            msg.pose.position.z = self.path[currentNode][2]
            
            self.position_pub.publish(msg)
            self.rate.sleep()
        
        rospy.loginfo('Mission completed successfully !')
        self.set_mode('AUTO.LAND')

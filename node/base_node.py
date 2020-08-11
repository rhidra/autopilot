#!/usr/bin/env python
import rospy, math, octomap, numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import MarkerArray
from visualization import viz_path, viz_nodes, viz_point


class BaseNode(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self.state = State()
        self.mission_wp = WaypointList()
        self.octree = octomap.OcTree(0.1)

    def setup(self):
        rospy.init_node(self.node_name, anonymous=True)

        try:
            rospy.wait_for_service('/mavros/cmd/arming')
            rospy.wait_for_service('/mavros/set_mode')
            rospy.wait_for_service('/mavros/mission/push')
            rospy.wait_for_service('/mavros/mission/clear')
            rospy.loginfo('All ROS services are up !')
        except rospy.ROSException:
            exit('Failed to connect to services')

        # Client / Service init
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

        # Subscribers
        self.alt_sub = rospy.Subscriber('/mavros/altitude', Altitude, self.altitude_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.home_pos_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_position_cb)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu, self.imu_data_cb)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_cb)
        self.mission_wp_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.mission_wp_cb)
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.octomap_cb)

        # Publishers
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) # Offboard control
        self.path_viz_pub = rospy.Publisher('/path_viz', MarkerArray, queue_size=10) # Custom topic used with Rviz

        # 20Hz loop rate
        self.rate = rospy.Rate(20)
        self.rate.sleep()


    """ Callback functions """
    def altitude_cb(self, data):
        self.altitude = data

    def global_position_cb(self, data):
        self.global_position = data

    def imu_data_cb(self, data):
        self.imu_data = data

    def home_position_cb(self, data):
        self.home_position = data

    def local_position_callback(self, data):
        self.local_position = data

    def mission_wp_cb(self, data):
        if self.mission_wp.current_seq != data.current_seq:
                rospy.loginfo("current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.mission_wp = data

    def state_cb(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(self.state.mode, data.mode))
        self.state = data

    def octomap_cb(self, data):
        rospy.loginfo('Octomap updated !')
        self.octomap = data

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

    def is_point_occupied(self, point):
        node = self.octree.search(point)
        try:
            res = self.octree.isNodeOccupied(node)
        except octomap.NullPointerException:
            # The point is unknown
            return False
        return res

    def cast_ray(self, origin, dest):
        origin = np.array(origin, dtype=np.double)
        dest = np.array(dest, dtype=np.double)
        direction = dest - origin
        end = np.array([0.0, 0.0, 0.0])

        hit = self.octree.castRay(origin, direction, end, ignoreUnknownCells=True, maxRange=np.linalg.norm(direction))
        return hit, end

    """ Helper methods """
    def set_mode(self, mode, timeout=5, loop_freq=1):
        """mode: PX4 mode string, timeout(int): seconds, loop_freq(int): seconds"""
        rospy.loginfo("Setting FCU mode: {0}".format(mode))
        rate = rospy.Rate(loop_freq)
        mode_set = False

        for i in range(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("Mode set successfully in {} sec".format(i / loop_freq))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not mode_set:
            exit('Timeout: failed to set the mode {} !'.format(mode))


    def set_arm(self, arm, timeout=5, loop_freq=1):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("Setting FCU arm: {0}".format(arm))
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in range(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("Arming set successfully in {} sec".format(i / loop_freq))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()
        if not arm_set:
            exit('Timeout: failed to arm !')


    def clear_wps(self, timeout=5, loop_freq=1):
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in range(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo("Cleared waypoints successfully in {} sec".format(i / loop_freq))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("Failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()

        if not wps_cleared:
            exit('Failed to clear waypoints !')


    def send_wps(self, waypoints, timeout=5, loop_freq=1):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo('Sending mission waypoints...')
        rospy.loginfo(waypoints)
        if self.mission_wp.waypoints:
            rospy.loginfo('FCU already has mission waypoints')

        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in range(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("Waypoints successfully transferred")
                    else:
                        rospy.logerr(res)
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                wps_verified = len(waypoints) == len(self.mission_wp.waypoints) or wps_verified

            if wps_sent and wps_verified:
                break
            rate.sleep()

        if not wps_sent and not wps_verified:
            exit('Mission could not be transferred and verified !')


    def wait_for_mav_type(self, timeout=5, loop_freq=1):
        """ Wait for MAV_TYPE parameter, timeout(int): seconds """
        rospy.loginfo("Waiting for MAV_TYPE")
        rate = rospy.Rate(loop_freq)
        type_obtained = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo('MAV_TYPE received ({0}) in {1} sec'.format(self.mav_type, i / loop_freq))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)
            rate.sleep()

        if res is None or not res.success:
            exit('Cannot get MAV_TYPE param !')


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
    

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")

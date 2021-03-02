#!/usr/bin/env python
import rospy, math, numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path


class BaseNode(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self.state = State()
        self.mission_wp = WaypointList()
        self.pos = np.array([0, 0, 0])
        self.vel = np.array([0, 0, 0])

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
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_cb)
        self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.local_velocity_cb)
        self.mission_wp_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.mission_wp_cb)
        self.local_goal_sub = rospy.Subscriber('/autopilot/local_goal', PoseStamped, self.local_goal_cb)

        # Publishers
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.position_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

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
        self.acc = np.array([self.imu_data.linear_acceleration.x, self.imu_data.linear_acceleration.y, self.imu_data.linear_acceleration.z])

    def home_position_cb(self, data):
        self.home_position = data

    def local_position_cb(self, data):
        self.local_position = data
        self.pos = np.array([self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z])

    def local_velocity_cb(self, data):
        self.local_velocity = data
        self.vel = np.array([self.local_velocity.twist.linear.x, self.local_velocity.twist.linear.y, self.local_velocity.twist.linear.z])

    def local_goal_cb(self, data):
        self.local_goal = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

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


    # Similar to set_mode() but it is supposed to be used in a main loop
    def try_set_mode(self, mode, freq=0.2):
        if self.state.mode == mode:
            return
        if not hasattr(self, 'last_mode_request'):
            self.last_mode_request = rospy.get_time()

        now = rospy.get_time()
        if self.last_mode_request + 1./freq > now:
            self.last_mode_request = now
            rospy.loginfo('Try to set mode {}...'.format(mode))
            try:
                res = self.set_mode_srv(0, mode)
                if not res.mode_sent:
                    rospy.logerr('Failed to send mode command')
            except rospy.ServiceException as e:
                rospy.logerr(e)
    
    def try_set_arm(self, arm, freq=0.2):
        if self.state.armed == arm:
            return
        if not hasattr(self, 'last_arm_request'):
            self.last_arm_request = rospy.get_time()
        
        now = rospy.get_time()
        if self.last_arm_request + 1./freq > now:
            self.last_arm_request = now
            rospy.loginfo('Try to set arming...')
            try:
                res = self.set_arming_srv(arm)
                if not res.success:
                    rospy.logerr('Failed to send arm command')
            except rospy.ServiceException as e:
                rospy.logerr(e)


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

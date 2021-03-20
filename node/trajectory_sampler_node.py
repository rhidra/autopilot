#!/usr/bin/env python
import rospy, math, numpy as np, time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import build_traj_tracker
from motion_primitive import MotionPrimitiveLibrary, TrajectoryError, buildMotionPrimitiveFromMsg
from autopilot.msg import MotionPrimitive
from controller_msgs.msg import FlatTarget

TOLERANCE_FROM_GOAL = 1
IDLE_DURATION = 1

NAV_PAUSE = 1
NAV_FOLLOW = 2

class TrajectorySamplerNode(OctomapNode):
    def setup(self):
        super(TrajectorySamplerNode, self).setup()
        self.rate = rospy.Rate(50)
        self.trajectory_sub = rospy.Subscriber('/autopilot/trajectory/response', MotionPrimitive, self.load_trajectory)
        self.trajectory_pub = rospy.Publisher('/autopilot/trajectory/request', Empty, queue_size=10)
        self.traj_tracking_pub = rospy.Publisher('/reference/flatsetpoint', FlatTarget, queue_size=10)
        self.yaw_tracking_pub = rospy.Publisher('/reference/yaw', Float32, queue_size=10)
        self.trajectory = None
        self.next_trajectory = None
        self.nav_state = NAV_PAUSE
        self.pause_start = None
        self.pause_pos = None
        self.traj_start = rospy.Time.now()
        self.has_requested = False
    

    def load_trajectory(self, msg):
        rospy.loginfo('Trajectory received !')
        self.next_trajectory = buildMotionPrimitiveFromMsg(msg)
        self.has_requested = False


    def request_trajectory(self):
        if self.has_requested:
            return
        self.has_requested = True
        rospy.loginfo('Requesting trajectory...')
        self.trajectory_pub.publish(Empty())


    def execute_trajectory(self):
        self.wait_local_goal()

        for _ in range(100):
            self.traj_tracking_pub.publish(build_traj_tracker())
            self.rate.sleep()

        self.request_trajectory()
        self.nav_state = NAV_PAUSE
        rospy.loginfo('Entering NAV_PAUSE mode')

        while not rospy.is_shutdown():
            self.try_set_mode('OFFBOARD')
            self.try_set_arm(True)

            if self.nav_state == NAV_PAUSE:
                self.pause_navigation()
                continue

            if self.dist_from(self.local_goal_point, vertical=False) < TOLERANCE_FROM_GOAL:
                rospy.loginfo('Goal attained !')
                rospy.loginfo('Mission done !')
                break

            if self.trajectory is None and self.next_trajectory is None:
                self.rate.sleep()
                continue
            
            t = (rospy.Time.now() - self.traj_start).to_sec()

            try:
                if self.next_trajectory is None:
                    self.request_trajectory()

                # If no trajectory or if we reached the end, we change trajectory
                if self.trajectory is None or t >= self.trajectory._tf:
                    self.trajectory = self.next_trajectory
                    self.next_trajectory = None
                    self.traj_start = rospy.Time.now()
                    t = 0

                if self.trajectory is None and self.next_trajectory is None:
                    self.rate.sleep()
                    continue
                
                pos = self.trajectory.get_position(t)
                vel = self.trajectory.get_velocity(t)
                acc = self.trajectory.get_acceleration(t)

                rospy.loginfo('Pos={} | Vel={} | Acc={}'.format(pos, vel, acc))

                msg = build_traj_tracker(pos, vel, acc)
                msg_yaw = Float32()
                msg_yaw.data = np.arctan2(vel[1], vel[0])
                self.traj_tracking_pub.publish(msg)
                self.yaw_tracking_pub.publish(msg_yaw)
                self.rate.sleep()
            except TrajectoryError:
                self.nav_state = NAV_PAUSE
                rospy.loginfo('No feasible trajectory found')
                rospy.loginfo('Entering NAV_PAUSE mode')
                continue


    def pause_navigation(self):
        if self.pause_pos is None or self.pause_start is None:
            self.pause_start = rospy.Time.now()
            self.pause_pos = self.pos

        if (rospy.Time.now() - self.pause_start).to_sec() > 1:
            self.nav_state = NAV_FOLLOW
            self.traj_start = rospy.Time.now()
            rospy.loginfo('Entering NAV_FOLLOW mode')

        msg = build_traj_tracker(self.pause_pos)
        self.traj_tracking_pub.publish(msg)
        self.rate.sleep()


    def wait_local_goal(self):
        print('Waiting for local goal instructions...')
        while (self.local_goal_point is None or self.local_goal_direction is None) and not rospy.is_shutdown():
            self.rate.sleep()
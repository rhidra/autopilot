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

TOLERANCE_FROM_GOAL = .5
IDLE_DURATION = 1

NAV_PAUSE = 1
NAV_FOLLOW = 2

class TrajectorySamplerNode(OctomapNode):
    def setup(self):
        super(TrajectorySamplerNode, self).setup()
        self.rate = rospy.Rate(50)
        self.trajectory_sub = rospy.Subscriber('/autopilot/trajectory/response', MotionPrimitive, self.load_trajectory)
        self.trajectory_pub = rospy.Publisher('/autopilot/trajectory/request', FlatTarget, queue_size=10)
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

        if self.trajectory is None:
            posf, velf = self.pos, self.vel
        else:    
            # Estimate the final position and velocity from the current real world data
            # We could use the trajectory final pose, but it would not consider real world uncertainties
            # Instead, we consider the current acceleration, velocity and position
            # and we model the acceleration as an affine function with a(tf)=[0,0,0].
            # From this assumption, we derive the final estimated velocity and position.
            # t0: start traj; t1=0: now in the traj; tf: end of the traj
            # We know a(t1)=a(0); v(t1)=v(0) and p(t1)=0
            # a(t) = a(t1) - a(t1) / (tf - t1) * (t - t1) with t1=0
            # So we have: 
            #   a(t) = a(0) - a(0) * t / tf
            #   v(t) = v(0) + a(0)*t - a(0) * t*t/(2*tf) => v(tf) = v(0) + a(0)*tf/2
            #   p(t) = p(0) + v(0)*t + a(t)*t*t/2 - a(0)*t*t*t/(6*tf) => p(tf) = p(0) + v(0)*tf + a(0)*tf*tf/3
            # Here tf is actually the duration to reach the end of the traj

            # tf = self.trajectory._tf - (rospy.Time.now() - self.traj_start).to_sec()
            # velf = self.vel + self.acc * tf / 2.
            # posf = self.pos + self.vel * tf + self.acc * tf * tf / 3.
            # This estimation model does not work, so we use the ground truth values instead
            posf = self.trajectory.get_position(self.trajectory._tf)
            velf = self.trajectory.get_velocity(self.trajectory._tf)
        rospy.loginfo('Requesting trajectory with pos0={} and vel0={}...'.format(posf, velf))
        self.trajectory_pub.publish(build_traj_tracker(pos=posf, vel=velf))


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

            if self.dist_from(self.goal_pos, vertical=False) < TOLERANCE_FROM_GOAL:
                rospy.loginfo('Goal attained !')
                rospy.loginfo('Mission done !')
                break

            if self.trajectory is None and self.next_trajectory is None:
                self.rate.sleep()
                continue

            t = (rospy.Time.now() - self.traj_start).to_sec()

            try:
                # Only try to generate a new trajectory before a few milliseconds before reaching the primitive end
                if self.next_trajectory is None and t >= self.trajectory._tf - .1:
                    self.request_trajectory()

                # If no trajectory or if we reached the end, we change trajectory
                if self.trajectory is None or t >= self.trajectory._tf:
                    self.trajectory = self.next_trajectory
                    self.next_trajectory = None
                    self.traj_start = rospy.Time.now()
                    t = 0

                if self.trajectory is None and self.next_trajectory is None:
                    rospy.logerr('No trajectory availaible !')
                    self.rate.sleep()
                    continue
                
                pos = self.trajectory.get_position(t) - self.start_pos
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

        msg = build_traj_tracker(self.pause_pos - self.start_pos)
        self.traj_tracking_pub.publish(msg)
        self.rate.sleep()


    def wait_local_goal(self):
        print('Waiting for local goal instructions...')
        while (self.local_goal_point is None or self.local_goal_direction is None) and not rospy.is_shutdown():
            self.rate.sleep()
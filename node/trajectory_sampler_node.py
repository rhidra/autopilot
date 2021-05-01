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

# Delay before requesting to generate new trajectory before reaching the end
# Should be as small as possible but higher than the running time of the local planner algorithm
TRAJ_GENERATION_DELAY = .02

# ROS node rate (in Hz)
RATE = 500

class TrajectorySamplerNode(OctomapNode):
    def setup(self):
        super(TrajectorySamplerNode, self).setup()
        self.rate = rospy.Rate(RATE)
        self.trajectory_sub = rospy.Subscriber('/autopilot/trajectory/response', MotionPrimitive, self.load_trajectory)
        self.trajectory_pub = rospy.Publisher('/autopilot/trajectory/request', FlatTarget, queue_size=10)
        self.traj_tracking_pub = rospy.Publisher('/reference/flatsetpoint', FlatTarget, queue_size=10)
        self.yaw_tracking_pub = rospy.Publisher('/reference/yaw', Float32, queue_size=10)
        self.trajectory = None
        self.next_trajectory = None
        self.traj_start = rospy.Time.now()
        self.has_requested = False
    

    def load_trajectory(self, msg):
        rospy.loginfo('Trajectory received in {}sec !'.format((rospy.Time.now() - self.start_request).to_sec()))
        self.next_trajectory = buildMotionPrimitiveFromMsg(msg)
        self.has_requested = False


    def request_trajectory(self):
        if self.has_requested:
            return
        self.has_requested = True
        self.start_request = rospy.Time.now()

        if self.trajectory is None:
            # On the first iteration, we can simply give the initial position
            # for the speed, we artificially create a speed from the yaw angle
            posf = self.pos
            init_vel = 0.05
            velf = np.array([init_vel * np.cos(self.yaw), init_vel * np.sin(self.yaw), 0])
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
            # velf[2] = 0 # Force z velocity at 0
            # posf = self.pos + self.vel * tf + self.acc * tf * tf / 3.
            # posf[2] = posfExpected[2]

            posf = self.pos
            posf[2] = self.trajectory.get_position(self.trajectory._tf)[2]
            # velf = self.vel

            # This estimation model does not work, so we use the ground truth values instead
            # posf = self.trajectory.get_position(self.trajectory._tf)
            velf = self.trajectory.get_velocity(self.trajectory._tf)
            # rospy.loginfo('Requesting trajectory \ntf={}\nfrom \tpos={} vel={}\nto \tpos={} vel={}\nexpect\tpos={} vel={}...'.format(tf, self.pos, self.vel, posf, velf, posf_expected, velf_expected))
        self.trajectory_pub.publish(build_traj_tracker(pos=posf, vel=velf))


    def execute_trajectory(self):
        self.wait_local_goal()
        self.wait_for_armed()
        init_z = float(rospy.get_param('/start/z', default=1.))

        # Initialize the UAV to the starting position, with a linear control for the yaw to avoid a sharp move
        msg_yaw = Float32()
        yaw = np.arctan2(self.local_goal_point[1] - self.start_pos[1], self.local_goal_point[0] - self.start_pos[0])
        for i in range(RATE*3):
            self.traj_tracking_pub.publish(build_traj_tracker(pos=[0., 0., init_z]))
            msg_yaw.data = yaw * min(i / (RATE*2.7), 1.)
            self.yaw_tracking_pub.publish(msg_yaw)
            self.rate.sleep()

        self.request_trajectory()

        while not rospy.is_shutdown():
            if self.dist_from(self.goal_pos, vertical=False) < TOLERANCE_FROM_GOAL:
                rospy.loginfo('Goal attained !')
                rospy.loginfo('Mission done !')
                rospy.set_param('/autopilot/done', 1)
                break

            if self.trajectory is None and self.next_trajectory is None:
                self.rate.sleep()
                continue

            t = (rospy.Time.now() - self.traj_start).to_sec()

            try:
                # Only try to generate a new trajectory before a few milliseconds before reaching the primitive end
                if self.next_trajectory is None and t >= self.trajectory._tf - TRAJ_GENERATION_DELAY:
                    self.request_trajectory()

                # If no trajectory or if we reached the end, we change trajectory
                if self.trajectory is None or t >= self.trajectory._tf:
                    self.trajectory = self.next_trajectory
                    self.next_trajectory = None
                    # self.traj_start = rospy.Time.now() - rospy.Duration(secs=TRAJ_GENERATION_DELAY)
                    self.traj_start = self.start_request - rospy.Duration(secs=.06)
                    t = (rospy.Time.now() - self.traj_start).to_sec()

                if self.trajectory is None and self.next_trajectory is None:
                    rospy.logerr('No trajectory available !')
                    if rospy.get_param('/autopilot/done', 0) == 2:
                        break
                    self.rate.sleep()
                    continue
                
                pos = self.trajectory.get_position(t) - self.start_pos
                vel = self.trajectory.get_velocity(t)
                acc = self.trajectory.get_acceleration(t)

                # rospy.loginfo('Pos={} | Vel={} | Acc={}'.format(pos, vel, acc))

                msg = build_traj_tracker(pos, vel, acc)
                msg_yaw = Float32()
                msg_yaw.data = np.arctan2(vel[1], vel[0])
                self.traj_tracking_pub.publish(msg)
                self.yaw_tracking_pub.publish(msg_yaw)
                self.rate.sleep()
            except TrajectoryError:
                rospy.loginfo('No feasible trajectory found')
                continue


    def wait_local_goal(self):
        print('Waiting for local goal instructions...')
        while (self.local_goal_point is None or self.local_goal_direction is None) and not rospy.is_shutdown():
            self.rate.sleep()
    
    def wait_for_armed(self):
        print('Waiting for arming...')
        while not self.state.armed or self.state.mode != 'OFFBOARD':
            self.rate.sleep()
        rospy.sleep(rospy.Duration(secs=1))
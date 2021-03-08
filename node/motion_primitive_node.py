#!/usr/bin/env python
import rospy, math, numpy as np, time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import build_traj_tracker
from motion_primitive import MotionPrimitiveLibrary, TrajectoryError


TOLERANCE_FROM_WAYPOINT = .5
TOLERANCE_FROM_GOAL = 1

# Possible navigation states
NAV_PAUSE = 1
NAV_FOLLOW = 2

IDLE_DURATION = 1

class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        self.tf = 3
        self.mpl = None
        self.trajectory = None
        self.next_trajectory = None
        self.nav_state = NAV_PAUSE
        self.pause_start = None
        self.pause_pos = None
        self.traj_history = []
        self.traj_start = rospy.Time.now()
    

    def follow_local_goal(self):
        self.wait_local_goal()

        for _ in range(100):
            self.traj_tracking_pub.publish(build_traj_tracker())
            self.rate.sleep()

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
            
            t = (rospy.Time.now() - self.traj_start).to_sec()

            try:
                if self.trajectory is None or (self.next_trajectory is None and t >= self.tf / 2.):
                    self.compute_optimal_traj()
                    rospy.loginfo('Selected trajectory with cost: {}'.format(self.next_trajectory.print_cost()))

                if self.trajectory is None or t >= self.tf:
                    self.trajectory = self.next_trajectory
                    self.next_trajectory = None
                    self.traj_start = rospy.Time.now()
                
                pos = self.trajectory.get_position(t)
                vel = self.trajectory.get_velocity(t)

                msg = build_traj_tracker(pos, vel)
                self.traj_tracking_pub.publish(msg)
                rospy.loginfo('Send waypoint: {}'.format(msg))
                self.visualize_local_path(pos=self.pos, vel=self.vel, trajLibrary=self.mpl.trajs, trajSelected=self.trajectory, trajHistory=self.traj_history, tf=self.tf)
                self.traj_history.append(self.trajectory)
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


    def compute_optimal_traj(self):
        start = time.time()
        self.next_trajectory = None
        if self.trajectory is None:
            pos, vel, acc, yaw = self.pos, self.vel, self.acc, self.yaw
        else:
            pos, vel = self.trajectory.get_position(self.tf), self.trajectory.get_velocity(self.tf), 
            acc, yaw = self.trajectory.get_acceleration(self.tf), self.trajectory.get_yaw(self.tf), 
            
        if np.linalg.norm(vel) < 0.2: 
            # When the UAV stopped, we artificially increase its 
            # initial velocity to make move in the right direction
            vel = np.array([.1 * np.cos(yaw), .1 * np.sin(yaw), 0.])
            acc = np.array([0., 0., 0.])

        rospy.loginfo('Initial conditions: ({}; {}; {})'.format(pos, vel, acc))
        self.mpl = MotionPrimitiveLibrary(tf=self.tf)
        self.mpl.generate_traj_library(pos, vel, acc)
        self.mpl.rank_trajectories(self.local_goal_point, self.local_goal_direction, self.get_point_edt)
        self.next_trajectory = self.mpl.get_best_traj()
        rospy.loginfo('Generated a new trajectory in {}sec'.format(time.time() - start))


    def wait_local_goal(self):
        print('Waiting for local goal instructions...')
        while (self.local_goal_point is None or self.local_goal_direction is None) and not rospy.is_shutdown():
            self.rate.sleep()
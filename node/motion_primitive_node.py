#!/usr/bin/env python
import rospy, math, numpy as np, time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import build_position_target
from planning import MotionPrimitiveLibrary, TrajectoryError


TOLERANCE_FROM_WAYPOINT = .5

# Possible navigation states
NAV_IDLE = 1
NAV_RESET = 2
NAV_FOLLOW = 3

IDLE_DURATION = 1

class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        self.tf = 1
        self.mpl = None
        self.trajectory = None
        self.nav_state = NAV_IDLE
        self.idle_start = None
        self.traj_history = []
    

    def follow_local_goal(self):
        self.wait_local_goal()

        print('Init local planner')
        msg = build_position_target(px=0, py=0, pz=2)
        for _ in range(100):
            msg.header.stamp = rospy.Time.now()
            self.position_raw_pub.publish(msg)
            self.rate.sleep()

        self.nav_state = NAV_RESET
        rospy.loginfo('Entering NAV_RESET mode')

        while not rospy.is_shutdown():
            self.try_set_mode('OFFBOARD')
            self.try_set_arm(True)

            if self.nav_state == NAV_RESET:
                self.reset_navigation()
                continue

            if self.nav_state == NAV_IDLE:
                self.idle_navigation()
                continue

            try:
                if self.trajectory is None:
                    self.compute_optimal_traj()
                
                final_pos = self.trajectory.get_position(self.tf)
                final_vel = self.trajectory.get_velocity(self.tf)
                final_acc = self.trajectory.get_acceleration(self.tf)
                final_yaw = self.trajectory.get_yaw(self.tf)

                if self.dist_from(final_pos) < TOLERANCE_FROM_WAYPOINT:
                    self.compute_optimal_traj(final_pos, final_vel, final_acc, final_yaw)
                    rospy.loginfo('Selected trajectory with cost: {}'.format(self.trajectory.print_cost()))
                    
                msg = build_position_target(px=final_pos[0], py=final_pos[1], pz=final_pos[2], yaw=final_yaw)
                self.position_raw_pub.publish(msg)
                self.visualize_local_path(pos=self.pos, vel=self.vel, trajLibrary=self.mpl.trajs, trajSelected=self.trajectory, trajHistory=self.traj_history, tf=self.tf)
                self.traj_history.append(self.trajectory)
                self.rate.sleep()
            except TrajectoryError:
                self.nav_state = NAV_RESET
                rospy.loginfo('No feasible trajectory found')
                rospy.loginfo('Entering NAV_RESET mode')
                continue
    

    def reset_navigation(self):
        yaw = np.arctan2(self.local_goal_point[1] - self.pos[1], self.local_goal_point[0] - self.pos[0])
        if abs(self.yaw - yaw) > .05 or self.pos[2] < 1.9:
            msg = build_position_target(pz=2 if self.pos[2] < 2 else None, v=0, yaw=yaw)
            self.reset_done = None
        else:
            msg = build_position_target(v=0, a=0)
            self.nav_state = NAV_FOLLOW
            rospy.loginfo('Entering NAV_FOLLOW mode')
        self.position_raw_pub.publish(msg)
        self.rate.sleep()


    def idle_navigation(self):
        if self.idle_start is None:
            self.idle_start = time.time()
        elif abs(self.idle_start - time.time()) > IDLE_DURATION:
            self.nav_state = NAV_FOLLOW
            self.idle_start = None
            rospy.loginfo('Entering NAV_FOLLOW mode')
        msg = build_position_target(v=0, a=0)
        self.position_raw_pub.publish(msg)
        self.rate.sleep()


    def compute_optimal_traj(self, pos=None, vel=None, acc=None, yaw=None):
        start = time.time()
        self.trajectory = None
        if pos is None:
            pos, vel, acc, yaw = self.pos, self.vel, self.acc, self.yaw
        self.mpl = MotionPrimitiveLibrary(tf=self.tf)
        self.mpl.generate_traj_library(pos, vel, acc, yaw)
        self.mpl.rank_trajectories(self.local_goal_point, self.local_goal_direction, self.get_point_edt)
        self.trajectory = self.mpl.get_best_traj()
        rospy.loginfo('Generated a new trajectory in {}sec'.format(time.time() - start))


    def wait_local_goal(self):
        print('Waiting for local goal instructions...')
        while (self.local_goal_point is None or self.local_goal_direction is None) and not rospy.is_shutdown():
            self.rate.sleep()
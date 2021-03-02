#!/usr/bin/env python
import rospy, math, numpy as np, time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import build_position_target
from planning import MotionPrimitveLibrary


TOLERANCE_FROM_WAYPOINT = .1

# Possible navigation states
NAV_IDLE = 1
NAV_RESET = 2
NAV_FOLLOW = 3

IDLE_DURATION = 3

class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        self.tf = 1
        self.mpl = None
        self.trajectory = None
        self.nav_state = NAV_IDLE
        self.idle_start = None
    

    def follow_local_goal(self):
        self.wait_local_goal()

        print('Init local planner')
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = 2
        # msg.velocity.x = 0
        # msg.velocity.y = 0
        # msg.velocity.z = 0
        # msg.acceleration_or_force.x = 0
        # msg.acceleration_or_force.y = 0
        # msg.acceleration_or_force.z = 0
        # msg.yaw = 90. * np.pi / 180.
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ

        for _ in range(100):
            msg.header.stamp = rospy.Time.now()
            self.position_raw_pub.publish(msg)
            self.rate.sleep()

        self.nav_state = NAV_RESET

        while not rospy.is_shutdown():
            self.try_set_mode('OFFBOARD')
            self.try_set_arm(True)

            if self.nav_state == NAV_RESET:
                self.reset_navigation()
                self.rate.sleep()
                continue

            if self.nav_state == NAV_IDLE:
                if self.idle_start is None:
                    self.idle_start = time.time()
                elif abs(self.idle_start - time.time()) > IDLE_DURATION:
                    self.nav_state = NAV_FOLLOW
                    self.idle_start = None
                msg = build_position_target(v=0, a=0)
                self.position_raw_pub.publish(msg)
                self.rate.sleep()
                continue

            if self.trajectory is None:
                self.compute_optimal_traj()
            
            final_pos = self.trajectory.get_position(self.tf)
            final_vel = self.trajectory.get_velocity(self.tf)
            final_acc = self.trajectory.get_acceleration(self.tf)

            if self.dist_from(final_pos) < TOLERANCE_FROM_WAYPOINT:
                self.compute_optimal_traj(final_pos, final_vel, final_acc)

            msg.header.stamp = rospy.Time.now()
            msg.position.x = final_pos[0]
            msg.position.y = final_pos[1]
            msg.position.z = final_pos[2]

            self.position_raw_pub.publish(msg)
            self.visualize_local_path(pos=self.pos, vel=self.vel, trajLibrary=self.mpl.trajs, trajSelected=self.trajectory, tf=self.tf)
            self.rate.sleep()
    

    def reset_navigation(self):
        yaw = np.arctan2(self.local_goal[1] - self.pos[1], self.local_goal[0] - self.pos[0])
        if abs(self.yaw - yaw) > .1 or self.pos[2] < 2:
            msg = build_position_target(pz=2 if self.pos[2] < 2 else None, v=0, yaw=yaw)
            self.reset_done = None
        else:
            msg = build_position_target(v=0, a=0)
            self.nav_state = NAV_IDLE
        self.position_raw_pub.publish(msg)


    def compute_optimal_traj(self, pos=None, vel=None, acc=None):
        if pos is None:
            pos, vel, acc = self.pos, self.vel, self.acc
        self.mpl = MotionPrimitveLibrary(tf=self.tf)
        self.mpl.generate_traj_library(pos, vel, acc)
        self.mpl.rank_trajectories(self.local_goal, self.get_point_edt)
        self.trajectory = self.mpl.get_best_traj()


    def wait_local_goal(self):
        print('Waiting for local goal instructions...')
        while self.local_goal is None and not rospy.is_shutdown():
            self.rate.sleep()
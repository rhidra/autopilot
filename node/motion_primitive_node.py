#!/usr/bin/env python
import rospy, math, numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import local_to_global, build_waypoints, fix_path_orientation, remove_start_offset
from planning import MotionPrimitive


TOLERANCE_FROM_WAYPOINT = .1

class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        self.tf = 1
        self.trajs = []
        self.trajectory = None
    
    def follow_local_goal(self):
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

        self.compute_optimal_traj(self.pos, self.vel, self.acc)

        while not rospy.is_shutdown():
            self.try_set_mode('OFFBOARD')
            self.try_set_arm(True)

            final_pos = self.trajectory.get_position(self.tf)
            final_vel = self.trajectory.get_velocity(self.tf)
            final_acc = self.trajectory.get_acceleration(self.tf)
            if self.dist_from(final_pos, sqrt=True) < TOLERANCE_FROM_WAYPOINT:
                self.compute_optimal_traj(final_pos, final_vel, final_acc)

            msg.header.stamp = rospy.Time.now()
            msg.position.x = final_pos[0]
            msg.position.y = final_pos[1]
            msg.position.z = final_pos[2]

            self.position_raw_pub.publish(msg)
            self.visualize_local_path(pos=self.pos, vel=self.vel, trajLibrary=self.trajs, trajSelected=self.trajectory, tf=self.tf)
            self.rate.sleep()

    def dist_from(self, p, sqrt=False):
        sqr = (p[0] - self.local_position.pose.position.x)**2 + \
              (p[1] - self.local_position.pose.position.y)**2 + \
              (p[2] - self.local_position.pose.position.z)**2
        return math.sqrt(sqr) if sqrt else sqr

    def compute_optimal_traj(self, pos=None, vel=None, acc=None):
        if pos is None:
            pos, vel, acc = self.pos, self.vel, self.acc
        print('Generating the trajectory library for pos={}; vel={}; acc={}'.format(pos, vel, acc))
        self.trajs = self.generate_traj_library(pos, vel, acc)
        print('Trajectory library generated !')

        for traj in self.trajs:
            traj.compute_cost(self.local_goal, self.get_point_edt)
        print('Trajectories ranked')

        self.trajectory = min(self.trajs, key=lambda t: t._cost)
        print('Best motion primitive selected')

    def generate_traj_library(self, pos0, vel0, acc0):
        numAngleVariation = 21
        numNormVariation = 10

        theta0 = math.atan2(vel0[1], vel0[0])
        norm0 = np.sqrt(vel0[0]*vel0[0] + vel0[1]*vel0[1])

        trajs = []
        for theta in np.linspace(theta0 - np.pi*.45, theta0 + np.pi*.45, numAngleVariation):
            for norm in np.linspace(np.clip(norm0 - 2, 0, 1e5), norm0 + 4, numNormVariation):
                trajs.append(self.generate_traj(pos0, vel0, acc0, [norm * np.cos(theta), norm * np.sin(theta), vel0[2]]))

        return trajs

    def generate_traj(self, pos0, vel0, acc0, velf):
        traj = MotionPrimitive(pos0, vel0, acc0, [0, 0, -9.81])
        traj.set_goal_velocity(velf)
        traj.set_goal_acceleration([0, 0, 0])
        traj.generate(self.tf)
        return traj

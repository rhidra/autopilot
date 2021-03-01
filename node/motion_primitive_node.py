#!/usr/bin/env python
import rospy, math, numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import local_to_global, build_waypoints, fix_path_orientation, remove_start_offset
from planning import MotionPrimitive


class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        self.tf = 1
    
    def find_trajectory(self):
        print('Generating the trajectory library...')
        trajs = self.generate_traj_library(self.pos, self.vel, self.acc)
        print('Trajectory library generated !')

        for traj in trajs:
            traj.compute_cost(self.local_goal, self.get_point_edt)
        print('Trajectories ranked')

        trajSelected = min(trajs, key=lambda t: t._cost)
        print('Best motion primitive selected')

        while not rospy.is_shutdown():
            self.visualize_local_path(pos=self.pos, vel=self.vel, trajLibrary=trajs, trajSelected=trajSelected, tf=self.tf)
            self.rate.sleep()

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

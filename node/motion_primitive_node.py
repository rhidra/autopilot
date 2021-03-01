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


TOLERANCE_FROM_WAYPOINT = .1

class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        self.tf = 1
        self.trajs = []
        self.trajectory = None
    
    def follow_local_goal(self):
        print('Init local planner')
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 2
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        for i in range(100):
            msg.header.stamp = rospy.Time.now()
            self.position_pub.publish(msg)
            self.rate.sleep()

        self.compute_optimal_traj()
        currentNode = 0

        while not rospy.is_shutdown():
            self.try_set_mode('OFFBOARD')
            self.try_set_arm(True)

            if self.dist_from(self.trajectory[currentNode], sqrt=True) < TOLERANCE_FROM_WAYPOINT:
                if currentNode == self.trajectory.shape[0] - 1:
                    self.compute_optimal_traj()
                    currentNode = 0
                else:
                    currentNode += 1

            msg.header.stamp = rospy.Time.now()

            msg.pose.position.x = self.trajectory[currentNode, 0]
            msg.pose.position.y = self.trajectory[currentNode, 1]
            msg.pose.position.z = self.trajectory[currentNode, 2]

            self.position_pub.publish(msg)
            self.visualize_local_path(pos=self.pos, vel=self.vel, trajLibrary=self.trajs, trajSelected=self.trajectory, tf=self.tf)
            self.rate.sleep()

    def dist_from(self, p, sqrt=False):
        sqr = (p[0] - self.local_position.pose.position.x)**2 + \
              (p[1] - self.local_position.pose.position.y)**2 + \
              (p[2] - self.local_position.pose.position.z)**2
        return math.sqrt(sqr) if sqrt else sqr

    def compute_optimal_traj(self):
        print('Generating the trajectory library...')
        self.trajs = self.generate_traj_library(self.pos, self.vel, self.acc)
        print('Trajectory library generated !')

        for traj in self.trajs:
            traj.compute_cost(self.local_goal, self.get_point_edt)
        print('Trajectories ranked')

        traj = min(self.trajs, key=lambda t: t._cost)
        t = np.linspace(0, self.tf, 10)
        self.trajectory = traj.get_position(t)
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

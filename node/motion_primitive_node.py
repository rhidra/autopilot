#!/usr/bin/env python
import rospy, math, numpy as np, time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from motion_primitive import MotionPrimitiveLibrary, TrajectoryError
from autopilot.msg import MotionPrimitive
from autopilot.srv import LocalGoal, LocalGoalRequest


class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        rospy.wait_for_service('/autopilot/local_goal')
        self.mpl = None
        self.trajectory = None
        self.current_traj = -1
        self.traj_history = []
        self.trajectory_sub = rospy.Subscriber('/autopilot/trajectory/request', Empty, self.send_trajectory)
        self.trajectory_pub = rospy.Publisher('/autopilot/trajectory/response', MotionPrimitive, queue_size=10)
        self.get_local_goal_srv = rospy.ServiceProxy('/autopilot/local_goal', LocalGoal)
        rospy.loginfo('Waiting for trajectory request...')
        self.send_trajectory(None) # Remove this once traj planning works
        rospy.spin()
    
    def send_trajectory(self, _):
        if self.current_traj == -1:
            self.generate_trajectory()
            self.current_traj = 0
        rospy.loginfo('Sending traj = {}'.format(self.current_traj))
        msg = self.traj_history[self.current_traj].toMsg()
        self.trajectory_pub.publish(msg)
        self.visualize_local_path(trajLibrary=self.mpl.trajs, trajSelected=self.traj_history[self.current_traj], trajHistory=self.traj_history, tf=self.tf)
        self.current_traj += 1

    def generate_trajectory(self):
        if self.local_goal_point is None or self.local_goal_direction is None:
            rospy.logerr('Cannot generate a trajectory: No local goal received !')

        rospy.loginfo('*'*30)
        traj = None
        for _ in range(20):
            time.sleep(2)
            try:
                traj = self.compute_optimal_traj(traj)
                self.visualize_local_path(trajLibrary=self.mpl.trajs, trajSelected=traj, trajHistory=self.traj_history, tf=self.tf)
                self.rate.sleep()
                self.traj_history.append(traj)
            except TrajectoryError as e:
                rospy.logerr('Cannot generate a trajectory: No feasible trajectory found')
                rospy.logerr('Traj cost: {}'.format(e.traj.print_cost()))
                break
        rospy.loginfo('Pre-generation done ! Generated {} trajectories'.format(len(self.traj_history)))
    

    def compute_optimal_traj(self, traj_prev=None):
        start = time.time()
        if traj_prev is None:
            traj_prev = self.trajectory

        if traj_prev is None:
            pos, vel, acc, yaw = self.pos, self.vel, self.acc, self.yaw
            vel, acc = np.array([0, 0, 0]), np.array([0, 0, 0])
            local_goal_point, local_goal_direction = self.get_local_goal(pos, vel)
            yaw = np.arctan2(local_goal_point[1] - pos[1], local_goal_point[0] - pos[0])
        else:
            pos, vel = traj_prev.get_position(self.tf), traj_prev.get_velocity(self.tf), 
            acc, yaw = traj_prev.get_acceleration(self.tf), traj_prev.get_yaw(self.tf), 
            local_goal_point, local_goal_direction = self.get_local_goal(pos, vel)
            
        if np.linalg.norm(vel) < 0.2: 
            # When the UAV stopped, we artificially increase its 
            # initial velocity to make move in the right direction
            vel = np.array([.1 * np.cos(yaw), .1 * np.sin(yaw), 0.])
            acc = np.array([0., 0., 0.])

        self.mpl = MotionPrimitiveLibrary(tf=self.tf)
        self.mpl.generate_traj_library(pos, vel, acc, local_goal_point[2])
        self.mpl.rank_trajectories(local_goal_point, local_goal_direction, self.get_point_edt)
        traj = self.mpl.get_best_traj()
        rospy.loginfo('Generated a new trajectory in {}sec'.format(time.time() - start))
        return traj

    def get_local_goal(self, pos, vel=[0, 0, 0]):
        msg = LocalGoalRequest()
        msg.position.x, msg.position.y, msg.position.z = pos[0], pos[1], pos[2]
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vel[0], vel[1], vel[2]
        msg = self.get_local_goal_srv(msg)
        local_goal_pos = np.array([msg.local_goal_position.x, msg.local_goal_position.y, msg.local_goal_position.z])
        local_goal_dir = np.array([msg.local_goal_direction.x, msg.local_goal_direction.y, msg.local_goal_direction.z])
        return local_goal_pos, local_goal_dir

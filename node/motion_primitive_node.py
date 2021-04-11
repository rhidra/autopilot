#!/usr/bin/env python
import rospy, math, numpy as np, time, pickle
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
from controller_msgs.msg import FlatTarget

TOLERANCE_FROM_GOAL = 0.5
class MotionPrimitiveNode(OctomapNode):
    def setup(self):
        super(MotionPrimitiveNode, self).setup()
        rospy.wait_for_service('/autopilot/local_goal')
        self.mpl = None
        self.trajectory = None
        self.current_traj = -1
        self.traj_history = []
        self.trajectory_sub = rospy.Subscriber('/autopilot/trajectory/request', FlatTarget, self.send_trajectory)
        self.trajectory_pub = rospy.Publisher('/autopilot/trajectory/response', MotionPrimitive, queue_size=10)
        self.get_local_goal_srv = rospy.ServiceProxy('/autopilot/local_goal', LocalGoal)
        rospy.loginfo('Waiting for trajectory request...')
        rospy.spin()
    
    def send_trajectory(self, msg):
        pos0 = np.array([msg.position.x, msg.position.y, msg.position.z])
        vel0 = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        if not self.generate_trajectory(pos0, vel0):
            return
        self.current_traj += 1
        rospy.loginfo('Sending traj {}'.format(self.current_traj))
        msg = self.traj_history[self.current_traj].toMsg()
        self.trajectory_pub.publish(msg)
        self.visualize_local_path(trajLibrary=self.mpl.trajs, trajSelected=self.traj_history[self.current_traj], trajHistory=self.traj_history, tf=self.tf)

    def generate_trajectory(self, pos0, vel0):
        if self.local_goal_point is None or self.local_goal_direction is None:
            rospy.logerr('Cannot generate a trajectory: No local goal received !')
            
        rospy.loginfo('*' * 30)
        try:
            traj = self.compute_optimal_traj(pos0, vel0)
            self.traj_history.append(traj)
            return True
        except TrajectoryError as e:
            rospy.logerr('Cannot generate a trajectory: No feasible trajectory found')
            rospy.logerr('Traj cost: {}'.format(e.traj.print_cost()))
            self.visualize_local_path(trajLibrary=self.mpl.trajs, trajSelected=e.traj, trajHistory=self.traj_history, tf=self.tf)
            return False
    

    def compute_optimal_traj(self, pos0=None, vel0=None):
        start = time.time()
        if pos0 is None or vel0 is None:
            pos0, vel0, acc0 = self.pos, self.vel, self.acc
            local_goal_point, local_goal_direction = self.get_local_goal(pos0, vel0)
        else:
            acc0 = np.array([0, 0, 0])
            local_goal_point, local_goal_direction = self.get_local_goal(pos0, vel0)
            
        if np.linalg.norm(vel0) < 0.2: 
            # When the UAV stopped, we artificially increase its 
            # initial velocity to make it move in the right direction
            vel0 = np.array([.1 * np.cos(self.yaw), .1 * np.sin(self.yaw), 0.])

        self.mpl = MotionPrimitiveLibrary(tf=self.tf)
        self.mpl.set_init_state(pos0, vel0, acc0)
        self.mpl.set_local_goal(local_goal_point, local_goal_direction)
        self.mpl.set_edt_function(self.get_point_edt)
        traj = self.mpl.optimize()
        rospy.loginfo('Selected trajectory: {}'.format(traj.print_cost()))
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

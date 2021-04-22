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
import json

# ROS node rate (in Hz)
RATE = 500

class LoggerNode(OctomapNode):
    def setup(self):
        super(LoggerNode, self).setup()
        self.start_node_time = rospy.Time.now()
        self.start_traj_time = rospy.Time.now()

        self.rate = rospy.Rate(RATE)
        self.wp_sub = rospy.Subscriber('/reference/flatsetpoint', FlatTarget, self.wp_cb)
        self.yaw_sub = rospy.Subscriber('/reference/yaw', Float32, self.yaw_cb)
        self.traj_sub = rospy.Subscriber('/autopilot/trajectory/response', MotionPrimitive, self.traj_cb)
        
        self.ref_wpYaw = None
        self.ref_wpPos = None
        self.ref_wpVel = None
        self.ref_wpAcc = None

        self.log_time = []

        self.log_pos = []
        self.log_vel = []
        self.log_acc = []
        self.log_yaw = []

        self.log_wpPos = []
        self.log_wpVel = []
        self.log_wpAcc = []
        self.log_wpYaw = []

        self.trajs = []
    
    def now(self):
        return (rospy.Time.now() - self.start_node_time).to_sec()
    
    def yaw_cb(self, msg):
        self.ref_wpYaw = msg.data

    def wp_cb(self, msg):
        self.ref_wpPos = [msg.position.x + self.start_pos[0], msg.position.y + self.start_pos[1], msg.position.z + self.start_pos[2]]
        self.ref_wpVel = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        self.ref_wpAcc = [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z]

    def traj_cb(self, msg):
        self.trajs.append({
            "pos0": [msg.pos0.x, msg.pos0.y, msg.pos0.z],
            "vel0": [msg.vel0.x, msg.vel0.y, msg.vel0.z],
            "acc0": [msg.acc0.x, msg.acc0.y, msg.acc0.z],
            "alpha": [msg.alpha.x, msg.alpha.y, msg.alpha.z],
            "beta": [msg.beta.x, msg.beta.y, msg.beta.z],
            "gamma": [msg.gamma.x, msg.gamma.y, msg.gamma.z],
            "gravity": [msg.gravity.x, msg.gravity.y, msg.gravity.z],
            "cost": msg.cost,
            "generationTime": msg.generationTime,
            "tf": msg.tf,
        })

    def execute(self):
        self.wait()
        self.start_traj_time = rospy.Time.now()

        while not rospy.is_shutdown():

            self.log_pos.append([self.pos[0], self.pos[1], self.pos[2]])
            self.log_vel.append([self.vel[0], self.vel[1], self.vel[2]])
            self.log_acc.append([self.acc[0], self.acc[1], self.acc[2]])
            self.log_yaw.append(self.yaw)

            if self.ref_wpPos is None or self.ref_wpVel is None or self.ref_wpAcc is None:
                self.log_wpPos.append([self.pos[0], self.pos[1], self.pos[2]])
                self.log_wpVel.append([self.vel[0], self.vel[1], self.vel[2]])
                self.log_wpAcc.append([self.acc[0], self.acc[1], self.acc[2]])
            else:
                self.log_wpPos.append(self.ref_wpPos)
                self.log_wpVel.append(self.ref_wpVel)
                self.log_wpAcc.append(self.ref_wpAcc)
            
            if self.ref_wpYaw is None:
                self.log_wpYaw.append(self.yaw)
            else:
                self.log_wpYaw.append(self.ref_wpYaw)

            self.log_time.append(self.now())

            done = rospy.get_param('/autopilot/done', 0)
            if done == 1: # Success
                rospy.loginfo('Trajectory done successfully !')
                self.writeData(True)
                exit(0)
            elif done == 2: # Failure
                rospy.loginfo('Trajectory done unsuccessfully !')
                self.writeData(False)
                exit(0)
            self.rate.sleep()


    def wait(self):
        print('Waiting until ready...')
        while (self.local_goal_point is None or self.local_goal_direction is None) and not rospy.is_shutdown():
            self.rate.sleep()
        while not self.state.armed or self.state.mode != 'OFFBOARD':
            self.rate.sleep()
        print('Starts logging !')

    def writeData(self, isSuccess):
        totalTime = (rospy.Time.now() - self.start_traj_time).to_sec()

        with open('/home/rhidra/research_data/test_m{}_c{}_t{}.json'.format(self.mapId, self.configId, self.trialId), 'w') as f:
            rospy.loginfo('Writing data')
            data = {
                "trialId": self.trialId,
                "configId": self.configId,
                "mapId": self.mapId,
                "start": [float(rospy.get_param('/start/x', 0)), float(rospy.get_param('/start/y', 0)), float(rospy.get_param('/start/z', 0))],
                "goal": [self.goal_pos[0], self.goal_pos[1], self.goal_pos[2]],
                "success": isSuccess,
                "tf": self.tf,
                "totalTime": totalTime,
                "trajs": self.trajs,

                "time": self.log_time,
                "uav_pos_x": np.array(self.log_pos)[:, 0].tolist(),
                "uav_pos_y": np.array(self.log_pos)[:, 1].tolist(),
                "uav_pos_z": np.array(self.log_pos)[:, 2].tolist(),
                "uav_vel_x": np.array(self.log_vel)[:, 0].tolist(),
                "uav_vel_y": np.array(self.log_vel)[:, 1].tolist(),
                "uav_vel_z": np.array(self.log_vel)[:, 2].tolist(),
                "uav_acc_x": np.array(self.log_acc)[:, 0].tolist(),
                "uav_acc_y": np.array(self.log_acc)[:, 1].tolist(),
                "uav_acc_z": np.array(self.log_acc)[:, 2].tolist(),
                "uav_yaw": self.log_yaw,

                "wp_pos_x": np.array(self.log_wpPos)[:, 0].tolist(),
                "wp_pos_y": np.array(self.log_wpPos)[:, 1].tolist(),
                "wp_pos_z": np.array(self.log_wpPos)[:, 2].tolist(),
                "wp_vel_x": np.array(self.log_wpVel)[:, 0].tolist(),
                "wp_vel_y": np.array(self.log_wpVel)[:, 1].tolist(),
                "wp_vel_z": np.array(self.log_wpVel)[:, 2].tolist(),
                "wp_acc_x": np.array(self.log_wpAcc)[:, 0].tolist(),
                "wp_acc_y": np.array(self.log_wpAcc)[:, 1].tolist(),
                "wp_acc_z": np.array(self.log_wpAcc)[:, 2].tolist(),
            }
            json.dump(data, f)
            rospy.loginfo('Data written ! Exiting...')
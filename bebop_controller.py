#!/usr/bin/env python
import rospy, numpy as np
from simple_pid import PID
from geometry_msgs.msg import Twist, PoseStamped
from geographic_msgs.msg import FlatTarget
from std_msgs.msg import Float32

"""
PID controller for the Parrot Bebop UAV
Uses the simple_pid Python library as a full PID implementation.

Inputs:
    - State (position/yaw) from the OpenVSLAM ROS module
    - Desired position/yaw from the trajectory planning

Output:
    - Linear and angular velocity commands, as described by Bebop autonomy doc
    ref: https://bebop-autonomy.readthedocs.io/en/latest/piloting.html#piloting
"""

# PID parameters
Kp_x = 1
Ki_x = 0.1
Kd_x = 0.05

Kp_y = 1
Ki_y = 0.1
Kd_y = 0.05

Kp_z = 1
Ki_z = 0.1
Kd_z = 0.05

Kp_yaw = 1
Ki_yaw = 0.1
Kd_yaw = 0.05

# Rate of command publishing (in Hz) (must be > 10Hz)
rate = 1 / 0.005

# Rotate a 3D vector by a specific yaw angle
def rotate(v, yaw):
    return [np.cos(yaw)*v[0] - np.sin(yaw)*v[1], np.sin(yaw)*v[0] + np.cos(yaw)*v[1], v[2]]

class BebopController:
    def __init__(self):
        # State vector = [x, y, z, yaw]
        self.state = [0, 0, 0, 0]
        self.last = rospy.Time.now()

        # PID declarations
        self.x = PID(Kp_x, Ki_x, Kd_x, setpoint=0, output_limits=(-1, 1))
        self.y = PID(Kp_y, Ki_y, Kd_y, setpoint=0, output_limits=(-1, 1))
        self.z = PID(Kp_z, Ki_z, Kd_z, setpoint=0, output_limits=(-1, 1))
        self.yaw = PID(Kp_yaw, Ki_yaw, Kd_yaw, setpoint=0, output_limits=(-1, 1))

        # ROS
        rospy.init_node('bebop_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_state = rospy.Subscriber('/openvslam/camera_pose', PoseStamped, self.state_cb)
        self.sub_cmd = rospy.Subscriber('/reference/flatsetpoint', FlatTarget, self.cmd_pos_cb)
        self.sub_cmd = rospy.Subscriber('/reference/yaw', Float32, self.cmd_yaw_cb)
        self.start_pos = np.array([rospy.get_param('/start/x', 0), rospy.get_param('/start/y', 0), rospy.get_param('/start/z', 0)])
        self.rate = rospy.Rate(rate)

    def state_cb(self, msg):
        o = msg.pose.orientation
        self.state = [
            msg.pose.position.x + self.start_pos[0], 
            msg.pose.position.y + self.start_pos[1], 
            msg.pose.position.z + self.start_pos[2],
            np.arctan2(2. * (o.w * o.z + o.x * o.y), o.w * o.w + o.x * o.x - o.y * o.y - o.z * o.z)
        ]

    def cmd_pos_cb(self, msg):
        self.x.setpoint = msg.position.x
        self.y.setpoint = msg.position.y
        self.z.setpoint = msg.position.z

    def cmd_yaw_cb(self, msg):
        self.yaw.setpoint = msg.data
        
    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last).to_sec()

            dx = self.x(self.state[0], dt)
            dy = self.y(self.state[1], dt)
            dz = self.z(self.state[2], dt)
            dyaw = self.yaw(self.state[3], dt)

            # Convert the difference vector from global to local coordinate
            dx, dy, dz = rotate([dx, dy, dz], self.state[3])

            msg = Twist()
            msg.linear.x = dx
            msg.linear.y = dy
            msg.linear.z = dz
            msg.angular.z = dyaw

            self.pub.publish(msg)

            self.last = now
            self.rate.sleep()


if __name__ == '__main__':
    ctrl = BebopController()
    ctrl.run()
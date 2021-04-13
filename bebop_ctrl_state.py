#!/usr/bin/env python
import rospy, numpy as np, sys
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from pynput.keyboard import Listener, Key, KeyCode

rospy.init_node('state_mock', anonymous=True)
pub = rospy.Publisher('/openvslam/camera_pose', PoseStamped, queue_size=10)
rate = rospy.Rate(100)

state = PoseStamped()
state.header.frame_id = 'map'
state.pose.position.x = 0
state.pose.position.y = 0
state.pose.position.z = 0
state.pose.orientation.x = 0
state.pose.orientation.y = 0
state.pose.orientation.z = 0
state.pose.orientation.w = 1

def make_q(s):
    return [s.pose.orientation.w, s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z]

# Convention [w, x, y, z]
def rotate_quat(q, a):
    q_ = [np.cos(a / 2), 0, 0, np.sin(a / 2)]
    return [
        q[0] * q_[0] - q[1] * q_[1] - q[2] * q_[2] - q[3] * q_[3],
        q[0] * q_[1] + q[1] * q_[0] + q[2] * q_[3] - q[3] * q_[2],
        q[0] * q_[2] - q[1] * q_[3] + q[2] * q_[0] + q[3] * q_[1],
        q[0] * q_[3] + q[1] * q_[2] - q[2] * q_[1] + q[3] * q_[0]
    ]

def rotate_state(a):
    q = rotate_quat(make_q(state), a)
    state.pose.orientation.w = q[0]
    state.pose.orientation.x = q[1]
    state.pose.orientation.y = q[2]
    state.pose.orientation.z = q[3]

def on_press(key):
    if key == Key.right:
        state.pose.position.y += .1
    elif key == Key.left:
        state.pose.position.y -= .1
    elif key == Key.up:
        state.pose.position.x += .1
    elif key == Key.down:
        state.pose.position.x -= .1
    elif key == Key.page_up:
        state.pose.position.z += .1
    elif key == Key.page_down:
        state.pose.position.z -= .1
    elif key == KeyCode.from_char('d'):
        rotate_state(-.1)
    elif key == KeyCode.from_char('q'):
        rotate_state(.1)

with Listener(on_press=on_press) as listener:
    while not rospy.is_shutdown():
        sys.stdout.write('\r[{:2.2f}, {:2.2f}, {:2.2f}] x [{:2.2f}, {:2.2f}, {:2.2f}, {:2.2f}]        '
        .format(
            state.pose.position.x, state.pose.position.y, state.pose.position.z, 
            state.pose.orientation.w, state.pose.orientation.x, state.pose.orientation.y, 
            state.pose.orientation.z))
        sys.stdout.flush()

        state.header.stamp = rospy.Time.now()
        pub.publish(state)
        rate.sleep()
    listener.stop()

#!/usr/bin/env python
import rospy, math, numpy as np
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import local_to_global, build_waypoints, fix_path_orientation, remove_start_offset
from autopilot.srv import LocalGoal, LocalGoalResponse

"""
LocalGoalNode

ROS Node which takes a global planner path in and outputs a local goal.
Needs the velocity and position of the robot.
"""

class LocalGoalNode(OctomapNode):
    def setup(self):
        super(LocalGoalNode, self).setup()
        self.goal_point_pub = rospy.Publisher('/autopilot/local_goal/point', Point, queue_size=10) # Local goal extractor
        self.goal_dir_pub = rospy.Publisher('/autopilot/local_goal/direction', Vector3, queue_size=10) # Local goal extractor
        self.local_goal_srv = rospy.Service('/autopilot/local_goal', LocalGoal, self.get_local_goal)
        self.rate = rospy.Rate(1)
        self.rate.sleep()

    def load_local_path(self, path):
        # Visualize the path in Rviz
        self.path = np.array(path)
        self.visualize_global_path(self.path, start=self.path[0], goal=self.path[-1])


    def send_local_goal(self):
        """
        Blocking method which continuously sends the local goal
        relative to the current position and velocity of the robot.
        """
        while not rospy.is_shutdown():
            msg_pt, msg_dir, proj = self.find_local_goal(self.pos, self.vel)

            self.goal_point_pub.publish(msg_pt)
            self.goal_dir_pub.publish(msg_dir)
            self.visualize_global_path(self.path, start=self.path[0], goal=self.path[-1])
            self.rate.sleep()


    def find_local_goal(self, pos, vel):
        msg_pt = Point()
        msg_pt.x = 0
        msg_pt.y = 0
        msg_pt.z = 2

        msg_dir = Vector3()
        msg_dir.x = 0
        msg_dir.y = 0
        msg_dir.z = 0

        # Compute closest path point to the current position
        a, b = self.path[:-1], self.path[1:]
        p = np.repeat(pos.reshape(1,-1), a.shape[0], axis=0)
        d = ptSegmentDist(p, a, b)
        segIdx = np.argmin(d)
        segDist, segA, segB = d[segIdx], a[segIdx], b[segIdx]

        # Solve segment - sphere intersection
        # ref: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
        u = (segB - segA) / np.linalg.norm(segB - segA)
        normSqr = np.dot(segB - segA, segB - segA)
        delta = np.square(np.dot(u, (segA - pos))) - (np.linalg.norm(segA - pos) ** 2 - segDist*segDist)
        if np.isclose(delta, 0) or -1e-2 <= delta < 0:
            d = - np.dot(u, (segA - pos))
            posProj = segA + d * u
        elif delta > 0:
            d1 = - np.dot(u, (segA - pos)) - np.sqrt(delta)
            d2 = - np.dot(u, (segA - pos)) + np.sqrt(delta)
            posProj1, posProj2 = segA + d1 * u, segA + d2 * u
            dot1 = np.dot(segB - segA, posProj1 - segA)
            posProj = posProj1 if 0 <= dot1 <= normSqr or np.isclose(dot1, 0) or np.isclose(dot1, normSqr) else posProj2
        else:
            print('Cannot solve Segment/Sphere intersection during local goal extraction (delta={})'.format(delta))
            return msg_pt, msg_dir

        # Forward projection in the path and snapping to the goal
        if np.linalg.norm(posProj - self.path[-1]) < 1:
            proj = self.path[-1]
        else:
            
            proj = forwardProject(posProj, segIdx, self.path, distance=self.tf * np.clip(3 * np.linalg.norm(vel), .1, 1.5))
            if np.linalg.norm(proj - self.path[-1]) < 1:
                proj = self.path[-1]

        direction = proj - posProj
        direction = direction / np.linalg.norm(direction)

        msg_pt.x = proj[0]
        msg_pt.y = proj[1]
        msg_pt.z = proj[2]

        msg_dir.x = direction[0]
        msg_dir.y = direction[1]
        msg_dir.z = direction[2]

        return msg_pt, msg_dir, proj


    def get_local_goal(self, msg):
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        msg_pt, msg_dir, proj = self.find_local_goal(pos, vel)

        self.visualize_global_path(self.path, start=self.path[0], goal=self.path[-1], point=proj)

        msg = LocalGoalResponse()
        msg.local_goal_position.x, msg.local_goal_position.y, msg.local_goal_position.z = msg_pt.x, msg_pt.y, msg_pt.z
        msg.local_goal_direction.x, msg.local_goal_direction.y, msg.local_goal_direction.z = msg_dir.x, msg_dir.y, msg_dir.z
        return msg


def ptSegmentDist(p, a, b):
    """
    Compute min distance between points p and segments [a, b], in 3D.
    All points are shape (x, 3)
    """
    # normalized tangent vector
    d = (b - a) / np.linalg.norm(b - a, axis=1)[:, None]

    # signed parallel distance components
    s = np.sum((a - p) * d, axis=1)
    t = np.sum((p - b) * d, axis=1)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, np.zeros(len(s))])

    # perpendicular distance component
    c = np.cross(p - a, d)

    return np.hypot(h, np.linalg.norm(c, axis=1))


def forwardProject(initPt, initIdx, path, distance=2):
    """
    Project a point forward on the path.
    If the point is outside the segment, it is projected on the 
    next segment in the path.
    """
    idx = initIdx
    pt = initPt
    while True:
        if idx + 1 >= len(path):
            return path[-1]
        a, b = path[idx], path[idx + 1]
        norm = np.linalg.norm(b - a)
        proj = pt + (b - a) * distance / norm
        diff = np.linalg.norm(a - proj) - norm
        if diff > 0:
            idx += 1
            pt = b
            distance = diff
        else:
            return proj
    raise Exception()
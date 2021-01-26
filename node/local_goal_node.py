#!/usr/bin/env python
import rospy, math, numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, CommandCode, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, Imu
from octomap_node import OctomapNode
from path_utils import local_to_global, build_waypoints, fix_path_orientation, remove_start_offset

"""
LocalGoalNode

ROS Node which takes a global planner path in and outputs a local goal.
Needs the velocity and position of the robot.
"""

class LocalGoalNode(OctomapNode):
    def setup(self):
        super(LocalGoalNode, self).setup()
        self.local_goal_pub = rospy.Publisher('/autopilot/local_goal', PoseStamped, queue_size=10) # Local goal extractor
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
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 2
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        done = False
        while not rospy.is_shutdown() and not done:
            # Compute closest path point to the current position
            a, b = self.path[:-1], self.path[1:]
            p = np.repeat(self.pos.reshape(1,-1), a.shape[0], axis=0)
            d = ptSegmentDist(p, a, b)
            segIdx = np.argmin(d)
            segDist, segA, segB = d[segIdx], a[segIdx], b[segIdx]

            # Solve segment - sphere intersection
            # ref: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
            u = (segB - segA) / np.linalg.norm(segB - segA)
            normSqr = np.dot(segB - segA, segB - segA)
            delta = np.square(np.dot(u, (segA - self.pos))) - (np.linalg.norm(segA - self.pos) ** 2 - segDist*segDist)
            if np.isclose(delta, 0):
                d = - np.dot(u, (segA - self.pos))
                posProj = segA + d * u
            elif delta > 0:
                d1 = - np.dot(u, (segA - self.pos)) - np.sqrt(delta)
                d2 = - np.dot(u, (segA - self.pos)) + np.sqrt(delta)
                posProj1, posProj2 = segA + d1 * u, segA + d2 * u
                dot1 = np.dot(segB - segA, posProj1 - segA)
                posProj = posProj1 if 0 <= dot1 <= normSqr or np.isclose(dot1, 0) or np.isclose(dot1, normSqr) else posProj2
            else:
                raise ValueError('Cannot solve Segment/Sphere intersection during local goal extraction (delta={})'.format(delta))

            # Forward projection in the path and snapping to the goal
            if np.linalg.norm(posProj - self.path[-1]) < 1:
                proj = self.path[-1]
            else:
                proj = forwardProject(posProj, segIdx, self.path, distance=2)#1*np.linalg.norm(self.vel))
                if np.linalg.norm(proj - self.path[-1]) < 1:
                    proj = self.path[-1]

            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = proj[0]
            msg.pose.position.y = proj[1]
            msg.pose.position.z = proj[2]
            
            self.local_goal_pub.publish(msg)
            self.visualize_global_path(self.path, start=self.path[0], goal=self.path[-1], point=proj)
            self.rate.sleep()


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
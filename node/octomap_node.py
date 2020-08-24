#!/usr/bin/env python
import rospy, math, octomap, numpy as np
from visualization_node import VisualizationNode
from octomap_msgs.msg import Octomap


class OctomapNode(VisualizationNode):
    def __init__(self, *args, **kwargs):
        super(OctomapNode, self).__init__(*args, **kwargs)
        self.octree = octomap.OcTree(0.1)
        self.collision_check = 0


    def setup(self):
        super(OctomapNode, self).setup()
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.octomap_cb)
        self.rate.sleep()


    def octomap_cb(self, data):
        rospy.loginfo('Octomap updated !')
        self.octomap = data

        # Read the octomap binary data and load it in the octomap wrapper class
        data = np.array(self.octomap.data, dtype=np.int8).tostring()
        s = '# Octomap OcTree binary file\nid {}\n'.format(self.octomap.id)
        s += 'size 42\nres {}\ndata\n'.format(self.octomap.resolution)
        s += data

        # An error is triggered because a wrong tree size has been specified in the
        # header. We did not find a way to extract the tree size from the octomap msg
        tree = octomap.OcTree(self.octomap.resolution)
        tree.readBinary(s)
        self.octree = tree


    def is_point_occupied(self, point, radius=.5):
        self.collision_check += 1

        node = self.octree.search(point)
        try:
            res = self.octree.isNodeOccupied(node)
        except octomap.NullPointerException:
            # The point is unknown
            res = False
        if res:
            return True
        end = np.array([0., 0., 0.])

        for direction in [np.array([0.,1.,0.]), np.array([0.,-1.,0.]), np.array([1.,0.,0.]), np.array([-1.,0.,0.])]:
            hit = self.octree.castRay(point, direction, end, ignoreUnknownCells=True, maxRange=radius)
            if hit:
                return True
        return False


    def cast_ray(self, origin, dest, radius=0., max_dist=-1, display=False):
        self.collision_check += 1
        
        origin = np.array(origin, dtype=np.double)
        dest = np.array(dest, dtype=np.double)
        direction = dest - origin + 1e-6
        distance = np.linalg.norm(direction)
        direction /= distance
        distance = distance if max_dist == -1 or distance < max_dist else max_dist
        end = dest if max_dist == -1 else origin + direction * max_dist

        hit = self.octree.castRay(origin, direction, end, ignoreUnknownCells=True, maxRange=distance)

        if hit or radius == 0.:
            return hit, end

        # To check the cylinder volume, we cast 4 additional rays
        # axis1 and 2 are in the plane perpendicular to the direction
        if direction[0] == 0 and direction[1] == 0:
            if direction[2] == 0:
                return False, origin
            axis1 = np.array([0., 1., 0.], dtype=np.double)
        else:
            axis1 = np.array([-direction[1], direction[0], 0.], dtype=np.double)           
        axis1 /= np.linalg.norm(axis1)
        axis2 = np.cross(direction, axis1)
        axis2 /= np.linalg.norm(axis2)

        origin1 = origin + axis1 * radius
        origin2 = origin - axis1 * radius
        origin3 = origin + axis2 * radius
        origin4 = origin - axis2 * radius

        if display:
            self.visualize_path(path2=[origin1, origin1 + direction * distance, origin1, 
                                        origin2, origin2 + direction * distance, origin2, 
                                        origin3, origin3 + direction * distance, origin3, 
                                        origin4, origin4 + direction * distance, origin4,
                                        origin, origin + direction * distance])

        for o in [origin1, origin2, origin3, origin4]:
            h = self.octree.castRay(o, direction, end, ignoreUnknownCells=True, maxRange=distance)
            if h:
                return h, end
        
        return hit, end


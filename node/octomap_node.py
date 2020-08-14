#!/usr/bin/env python
import rospy, math, octomap
from visualization_node import VisualizationNode
from octomap_msgs.msg import Octomap


class OctomapNode(VisualizationNode):
    def __init__(self, *args, **kwargs):
        super(OctomapNode, self).__init__(*args, **kwargs)
        self.octree = octomap.OcTree(0.1)


    def setup(self):
        super(OctomapNode, self).setup()
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.octomap_cb)


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


    def cast_ray(self, origin, dest):
        origin = np.array(origin, dtype=np.double)
        dest = np.array(dest, dtype=np.double)
        direction = dest - origin
        end = np.array([0.0, 0.0, 0.0])

        hit = self.octree.castRay(origin, direction, end, ignoreUnknownCells=True, maxRange=np.linalg.norm(direction))
        return hit, end
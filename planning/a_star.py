import numpy as np, math, rospy
from utils import random_position, rand, dist, Node_astar as Node, UAV_THICKNESS
from smoothing import over_sampling, filter_path, bezier

EPSILON_GOAL = .2
EPSILON_NODE = .05
INCREMENT_DISTANCE = .5

children_directions = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [-1, 1, 1], [-1, 0, 1], [-1, -1, 1], [0, -1, 1], [1, -1, 1], 
                                           [1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0], 
                                [0, 0, -1], [1, 0, -1], [1, 1, -1], [0, 1, -1], [-1, 1, -1], [-1, 0, -1], [-1, -1, -1], [0, -1, -1], [1, -1, -1]])


def find_node(s, point):
    l = list(filter(lambda node: dist(node, point) <= EPSILON_NODE, s))
    return l[0] if len(l) > 0 else None


# Return all the children (neighbors) of a specific node
def children(point, ros_node, world_dim, closedset, openset):
    potential_children = point.pos + children_directions * INCREMENT_DISTANCE
    children = []
    for c in potential_children:
        if      world_dim[0] <= c[0] and c[0] <= world_dim[1] \
            and world_dim[2] <= c[1] and c[1] <= world_dim[3] \
            and world_dim[4] <= c[2] and c[2] <= world_dim[5] \
            and not ros_node.cast_ray(point.pos, c, radius=UAV_THICKNESS)[0] \
            and find_node(closedset, c) is None:
                node = find_node(openset, c)
                if node is None:
                    node = Node(c, Node.FREE)
                children.append(node)
    return children


def a_star(ros_node, start, goal, world_dim):
    rospy.loginfo('Computing A* algorithm...')
    
    openset = set()
    closedset = set()

    current = Node(start, Node.FREE)
    openset.add(current)

    i=0
    while openset:
        i = i+1

        current = min(openset, key=lambda o:o.G + o.H)

        if dist(current, goal) < EPSILON_GOAL:
            path = [goal]
            while current.parent:
                path.append(list(current.pos))
                current = current.parent
            path.append(current)
            return path[::-1], i

        openset.remove(current)
        closedset.add(current)

        # Loop through the node's children/siblings
        # From start
        for node in children(current, ros_node, world_dim, closedset, openset):
            if node in openset:
                # Check if we beat the G score
                new_g = current.G + dist(current, node)
                if node.G > new_g:
                    # If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + dist(current, node)
                node.H = dist(node, goal)
                node.parent = current
                openset.add(node)
        
        if i % 50 == 0:
            ros_node.visualize_path(nodes=openset.union(closedset), start=start, goal=goal)
            ros_node.rate.sleep()

    raise ValueError('No Path Found')

def main_a_star(ros_node, start, goal, world_dim):
    assert world_dim[0] <= start[0] and start[0] <= world_dim[1]
    assert world_dim[2] <= start[1] and start[1] <= world_dim[3]
    assert world_dim[4] <= start[2] and start[2] <= world_dim[5]

    path, count = a_star(ros_node, start, goal, world_dim)
    print('Path found !')

    return path, count, count, count
import numpy as np, math, rospy
from utils import random_position, rand, dist, Node
from smoothing import over_sampling, filter_path, bezier

# Radius of closeness for rerouting a node
NEIGHBOR_RADIUS = 1
INCREMENT_DISTANCE = .1

EPSILON = .5
MAX_ITERATIONS = 10000


def project(ros_node, origin, dest, distance=INCREMENT_DISTANCE):
    v = (np.array(dest) - np.array(origin))
    vprime = v * distance / np.linalg.norm(v)
    new = np.array(origin) + vprime
    return None if ros_node.is_point_occupied(new) else [float(new[0]), float(new[1]), float(new[2])]


def line_in_obstacle(ros_node, origin, dest):
    hit, _ = ros_node.cast_ray(origin, dest, radius=.25)
    return hit


def build_path(current):
    rospy.loginfo('Found a path !')
    path = []
    path_len = 0
    while current:
        rospy.loginfo(current)
        path.append(current.pos)
        if current.parent:
            path_len += dist(current, current.parent, sqrt=True)
        current = current.parent
    return path[::-1], int(path_len)


def rrt_star(ros_node, start, goal, world_dim):
    rospy.loginfo('Computing RRT* algorithm...')
    nodes = []
    nodes.append(Node(start, None))

    i=0
    while i < MAX_ITERATIONS:
        i = i+1

        if i%20 == 0:
            x_rand = goal
        else:
            x_rand = random_position(world_dim)

        # Search nearest node to the rand point
        nearest = nodes[0]
        for node in nodes:
            if node.pos == x_rand:
                nearest = None
                break
            if dist(node, x_rand) < dist(nearest, x_rand):
                nearest = node
        if not nearest:
            continue

        # Obtain the new node by projecting the nearest over x_rand
        new = project(ros_node, nearest.pos, x_rand, distance=2)
        if not new:
            continue
        new = x_rand

        # Search the neighbors of the nearest node
        neighbors = filter(lambda node: dist(node, nearest) < NEIGHBOR_RADIUS
                            and not line_in_obstacle(ros_node, node.pos, new), nodes)

        # Select best possible neighbor
        try:
            nearest = min(neighbors, key=lambda node: node.cost)
        except ValueError:
            continue

        new = Node(new, nearest)
        nodes.append(new)

        # Rewiring of the tree
        for node in neighbors:
            if new.cost + dist(new, node) < node.cost:
                node.parent = new
                node.cost = new.cost + dist(new, node)

        if dist(new, goal, sqrt=True) < EPSILON:
            path, path_len = build_path(Node(goal, new))
            return (path, nodes, i, path_len)

        ros_node.visualize_path(nodes=nodes, start=start, goal=goal, point=new.pos)
        ros_node.rate.sleep()

    raise ValueError('No Path Found')


def main_rrt_star(ros_node, start, goal, world_dim):
    path, nodes, count, path_len = rrt_star(ros_node, start, goal, world_dim)

    ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
    for _ in range(50):
        ros_node.rate.sleep()
    # path = over_sampling(path)
    # ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
    # for _ in range(10):
    #     ros_node.rate.sleep()
    # path = filter_path(path, ros_node)
    # ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
    # for _ in range(10):
    #     ros_node.rate.sleep()

    return path, nodes, count, path_len
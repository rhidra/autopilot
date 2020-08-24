import numpy as np, math, rospy, time
from utils import random_position, rand, dist, Node_rrt as Node, UAV_THICKNESS
from smoothing import over_sampling, filter_path, bezier


# Radius of closeness for rerouting a node
NEIGHBOR_RADIUS = 2
INCREMENT_DISTANCE = .7

MAX_ITERATIONS = 100000
MIN_ITERATIONS = 500


def project(ros_node, origin, dest, distance=INCREMENT_DISTANCE):
    v = (np.array(dest) - np.array(origin))
    vprime = v * distance / np.linalg.norm(v)
    new = np.array(origin) + vprime
    return None if ros_node.is_point_occupied(new) else [float(new[0]), float(new[1]), float(new[2])]


def build_path(current):
    path = []
    path_len = 0
    while current:
        path.append(current.pos)
        if current.parent:
            path_len += dist(current, current.parent, sqrt=True)
        current = current.parent
    return path[::-1], int(path_len)


def rrt_star(ros_node, start, goal, world_dim, display=True):
    rospy.loginfo('Computing RRT* algorithm...')
    nodes = []
    nodes.append(Node(start, None))

    i = 0
    goal_node = None
    while i < MAX_ITERATIONS and (goal_node is None or i < MIN_ITERATIONS):
        i = i + 1

        if i % 20 == 0 and goal_node is not None:
            x_rand = goal
        else:
            x_rand = random_position(world_dim)

        # Search nearest node to the rand point
        try:
            nearest = min(nodes, key=lambda node: dist(node, x_rand) if node is not goal_node else 1e1000)
        except ValueError:
            continue

        # Obtain the new node by casting the nearest node. The new node can be against a wall
        _, new = ros_node.cast_ray(nearest.pos, x_rand, radius=UAV_THICKNESS, max_dist=INCREMENT_DISTANCE)

        # Search the neighbors of the nearest node
        radius = min(NEIGHBOR_RADIUS * (math.log(len(nodes))/len(nodes)) ** (1/3.), INCREMENT_DISTANCE)
        neighbors = filter(lambda node: dist(node, nearest) <= NEIGHBOR_RADIUS
                            and not ros_node.cast_ray(node.pos, new, radius=UAV_THICKNESS)[0]
                            and node is not goal_node, nodes)

        # Select best possible neighbor
        try:
            nearest = min(neighbors, key=lambda node: node.cost + dist(node, new))
        except ValueError:
            pass

        new = Node(new, nearest)
        nodes.append(new)

        # Rewiring of the tree
        for node in neighbors:
            if new.cost + dist(new, node) < node.cost:
                node.parent = new
                node.cost = new.cost + dist(new, node)

        if goal_node is None and not ros_node.cast_ray(new.pos, goal, radius=UAV_THICKNESS)[0]:
            # Goal discovered
            goal_node = Node(goal, new)
            nodes.append(goal_node)
        elif goal_node is not None and not ros_node.cast_ray(new.pos, goal, radius=UAV_THICKNESS)[0] and new.cost + dist(new.pos, goal) < goal_node.cost:
            # Goal node rewiring
            goal_node.parent = new
            goal_node.cost = new.cost + dist(new, node)

        if display and i % 50 == 0:
            ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=build_path(goal_node)[0] if goal_node is not None else [])
            ros_node.rate.sleep()

    if goal_node is None:
        raise ValueError('No Path Found')

    path, path_len = build_path(goal_node)
    return (path, nodes, i, path_len)


def main_rrt_star(ros_node, start, goal, world_dim, display=True, with_optim=True):
    assert world_dim[0] <= start[0] and start[0] <= world_dim[1]
    assert world_dim[2] <= start[1] and start[1] <= world_dim[3]
    assert world_dim[4] <= start[2] and start[2] <= world_dim[5]

    start_time = time.time()
    path, nodes, _, _ = rrt_star(ros_node, start, goal, world_dim, display)
    end_time = time.time()

    if not with_optim:
        return path, end_time - start_time

    if display:
        ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
        for _ in range(10):
            ros_node.rate.sleep()

    path = over_sampling(path, max_length=INCREMENT_DISTANCE)

    if display:
        ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
        for _ in range(30):
            ros_node.rate.sleep()
    
    path = filter_path(path, ros_node)

    if display:
        ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
        for _ in range(30):
            ros_node.rate.sleep()
    
    path = over_sampling(path, max_length=INCREMENT_DISTANCE)

    if display:
        ros_node.visualize_path(nodes=nodes, start=start, goal=goal, path=path)
        for _ in range(30):
            ros_node.rate.sleep()

    end_time = time.time()
    return path, end_time - start_time


def main_rrt_star_without_optim(*args, **kwargs):
    return main_rrt_star(*args, with_optim=False, **kwargs)
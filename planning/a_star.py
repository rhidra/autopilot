import numpy as np, math, rospy, time
from utils import random_position, rand, dist, Node_astar as Node, UAV_THICKNESS
from smoothing import over_sampling, filter_path, bezier

EPSILON_NODE = .05 ** 2
INCREMENT_DISTANCE = .5

children_directions = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [-1, 1, 1], [-1, 0, 1], [-1, -1, 1], [0, -1, 1], [1, -1, 1], 
                                           [1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0], 
                                [0, 0,-1], [1, 0,-1], [1, 1,-1], [0, 1,-1], [-1, 1,-1], [-1, 0,-1], [-1, -1,-1], [0, -1,-1], [1, -1,-1]])


# Return all the children (neighbors) of a specific node
def children(node, ros_node, world_dim, grid):
    potential_children = node.pos + children_directions * INCREMENT_DISTANCE
    children = []
    for c in potential_children:
        if world_dim[0] <= c[0] and c[0] <= world_dim[1] and world_dim[2] <= c[1] and c[1] <= world_dim[3] and world_dim[4] <= c[2] and c[2] <= world_dim[5]:
            grid_c = int((c[0] - world_dim[0]) // INCREMENT_DISTANCE), int((c[1] - world_dim[2]) // INCREMENT_DISTANCE), int((c[2] - world_dim[4]) // INCREMENT_DISTANCE)
            grid_c = max(0, grid_c[0]), max(0, grid_c[1]), max(0, grid_c[2])
            grid_c = min(grid_c[0], len(grid)-1), min(grid_c[1], len(grid[0])-1), min(grid_c[2], len(grid[0][0])-1)
            child = grid[grid_c[0]][grid_c[1]][grid_c[2]]

            if child is None:
                child = Node(c, Node.OBSTACLE if ros_node.cast_ray(node.pos, c, radius=UAV_THICKNESS)[0] else Node.FREE)
                grid[grid_c[0]][grid_c[1]][grid_c[2]] = child

            if child.value == Node.OBSTACLE:
                continue

            children.append(child)
    return children


def a_star(ros_node, start, goal, world_dim, grid, display=True):
    rospy.loginfo('Computing A* algorithm...')
    
    openset = set()
    closedset = set()

    current = Node(start, Node.FREE)
    openset.add(current)

    i=0
    while openset:
        i = i+1

        current = min(openset, key=lambda o:o.G + o.H)

        if dist(current, goal, sqrt=False) <= INCREMENT_DISTANCE * INCREMENT_DISTANCE:
            path = [list(goal)]
            while current.parent:
                path.append(list(current.pos))
                current = current.parent
            path.append(list(current.pos))
            return path[::-1], i

        openset.remove(current)
        closedset.add(current)

        # Loop through the node's children/siblings
        # From start
        for node in children(current, ros_node, world_dim, grid):
            if node in closedset:
                continue
            
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

        if display and i % 10 == 0:
            ros_node.visualize_path(nodes=openset.union(closedset), start=start, goal=goal)
            ros_node.rate.sleep()

    raise ValueError('No Path Found')

def main_a_star(ros_node, start, goal, world_dim, display=True):
    assert world_dim[0] <= start[0] and start[0] <= world_dim[1] and world_dim[0] < world_dim[1]
    assert world_dim[2] <= start[1] and start[1] <= world_dim[3] and world_dim[2] < world_dim[3]
    assert world_dim[4] <= start[2] and start[2] <= world_dim[5] and world_dim[4] < world_dim[5]

    start_time = time.time()
    grid_shape = (int((world_dim[1] - world_dim[0]) // INCREMENT_DISTANCE),
                  int((world_dim[3] - world_dim[2]) // INCREMENT_DISTANCE),
                  int((world_dim[5] - world_dim[4]) // INCREMENT_DISTANCE))
    
    grid = [[[None for z in range(grid_shape[2])] for y in range(grid_shape[1])] for x in range(grid_shape[0])]

    path, count = a_star(ros_node, start, goal, world_dim, grid, display)
    end_time = time.time()

    return path, end_time - start_time
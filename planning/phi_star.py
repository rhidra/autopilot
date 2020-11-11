import numpy as np, math, rospy, time
from utils import random_position, rand, dist, Node_astar as Node, UAV_THICKNESS
from smoothing import over_sampling, filter_path, bezier
from decimal import Decimal, ROUND_HALF_UP

EPSILON_NODE = .05 ** 2
INCREMENT_DISTANCE = .4

DEFAULT_FOR_PATH_1 = math.degrees(math.atan2(1. / math.sqrt(2.)))

# JUST FOR 2D
def phi(a,b,c):
    return (180./math.pi) * (-math.atan2(a[1]-b[1], a[0]-b[0]) + math.atan2(c[1]-b[1], c[0]-b[0])

# FOR 3D, return spherical angle between two points A and C, relative to B (vectors BA and BC)
# Phi: angle in plane (x, y)
# Theta: angle in plane (the rotating plane orthogonal to (x,y) going through A, z)
def phi3d(a,b,c):
    phi = (180./math.pi) * (-math.atan2(a[1]-b[1], a[0]-b[0]) + math.atan2(c[1]-b[1], c[0]-b[0]))
    theta = (180./math.pi) * (-math.atan2(a[2]-b[2], math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)) + math.atan2(c[2]-b[2], math.sqrt((c[0]-b[0])**2 + (c[1]-b[1])**2)))
    return phi, theta

children_directions = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [-1, 1, 1], [-1, 0, 1], [-1, -1, 1], [0, -1, 1], [1, -1, 1], 
                                           [1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0], 
                                [0, 0,-1], [1, 0,-1], [1, 1,-1], [0, 1,-1], [-1, 1,-1], [-1, 0,-1], [-1, -1,-1], [0, -1,-1], [1, -1,-1]])


# Return all the children (neighbors) of a specific node
def children(node, ros_node, world_dim, grid):
    potential_children = node.pos + children_directions * INCREMENT_DISTANCE
    children = []
    for c in potential_children:
        if world_dim[0] <= c[0] and c[0] <= world_dim[1] and world_dim[2] <= c[1] and c[1] <= world_dim[3] and world_dim[4] <= c[2] and c[2] <= world_dim[5]:
            # Rounding 4.499999999 => 4.5 and 4.5 => 5
            # With just round(): round(4.5) => 4 and round(3.5) => 4
            # So instead we use Decimal() with ROUND_HALF_UP
            # Then we clamp the coordinates between the grid boundaries
            grid_c = int(Decimal(round((c[0] - world_dim[0]) / INCREMENT_DISTANCE, 3)).to_integral_value(rounding=ROUND_HALF_UP)),\
                     int(Decimal(round((c[1] - world_dim[2]) / INCREMENT_DISTANCE, 3)).to_integral_value(rounding=ROUND_HALF_UP)),\
                     int(Decimal(round((c[2] - world_dim[4]) / INCREMENT_DISTANCE, 3)).to_integral_value(rounding=ROUND_HALF_UP))
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


def theta_star(ros_node, start, goal, world_dim, grid, display=True):
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
            
            if node not in openset:
                node.G = float('inf')
                node.H = dist(node, goal)
                node.parent = None
                openset.add(node)
            
            if current.parent and not ros_node.cast_ray(current.parent.pos, node.pos, radius=UAV_THICKNESS)[0]:
                # If in line of sight, we connect to the parent, it avoid unecessary grid turns
                if current.parent.G + dist(current.parent, node) < node.G:
                    node.G = current.parent.G + dist(current.parent, node)
                    node.parent = current.parent
            else:
                if current.G + dist(current, node) < node.G:
                    node.G = current.G + dist(current, node)
                    node.parent = current

        if display and i % 10 == 0:
            ros_node.visualize_path(nodes=openset.union(closedset), start=start, goal=goal)
            ros_node.rate.sleep()

    raise ValueError('No Path Found')

def main_theta_star(ros_node, start, goal, world_dim, display=True):
    assert world_dim[0] <= start[0] and start[0] <= world_dim[1] and world_dim[0] < world_dim[1]
    assert world_dim[2] <= start[1] and start[1] <= world_dim[3] and world_dim[2] < world_dim[3]
    assert world_dim[4] <= start[2] and start[2] <= world_dim[5] and world_dim[4] < world_dim[5]

    start_time = time.time()
    grid_shape = (int((world_dim[1] - world_dim[0]) // INCREMENT_DISTANCE),
                  int((world_dim[3] - world_dim[2]) // INCREMENT_DISTANCE),
                  int((world_dim[5] - world_dim[4]) // INCREMENT_DISTANCE))
    
    grid = [[[None for z in range(grid_shape[2])] for y in range(grid_shape[1])] for x in range(grid_shape[0])]

    path, count = theta_star(ros_node, start, goal, world_dim, grid, display)
    end_time = time.time()

    return path, end_time - start_time
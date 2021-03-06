import numpy as np, math
from decimal import Decimal, ROUND_HALF_UP

UAV_THICKNESS = .8

class Node:
    def __repr__(self):
        return self.pos.__repr__()

class Node_astar(Node):
    OBSTACLE = 1
    FREE = 0

    def __init__(self, pos, value):
        self.value = value
        self.pos = np.array(pos)
        self.parent = None
        self.H = 0
        self.G = 0

class Node_phistar(Node):
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.reset()

    def reset(self):
        self.parent = None
        self.local = None
        self.H = 0
        self.G = float('inf')
        self.lb_phi = -float('inf')
        self.ub_phi = float('inf')
        self.lb_the = -float('inf')
        self.ub_the = float('inf')


class Node_rrt(Node):
    def __init__(self, pos, parent):
        self.pos = pos
        self.parent = parent
        if parent is not None:
            self.cost = parent.cost + dist(parent, pos)
        else:
            self.cost = 0

class NoPathFound(BaseException):
    pass

def rand(a, b=None, integer=False):
    if integer:
        return np.random.randint(a) if b is None else np.random.randint(b - a) + a
    else:
        return np.random.uniform(0, a) if b is None else np.random.uniform(a, b)


def random_position(world_dim):
    xmin, xmax, ymin, ymax, zmin, zmax = world_dim
    return [rand(xmin, xmax), rand(ymin, ymax), rand(zmin, zmax)]


def dist(p1, p2, sqrt=True, w_z=1.):
    if p1 is None or p2 is None:
        return float('inf')

    p1 = p1.pos if isinstance(p1, Node) else p1
    p2 = p2.pos if isinstance(p2, Node) else p2

    sqr = (p1[0] - p2[0])**2 + (p1[1]-p2[1])**2 + (w_z * (p1[2]-p2[2]))**2
    return math.sqrt(sqr) if sqrt else sqr


# Convert a 3D point in the world to a cell coordinate in a grid 
def pointToCell(pt, world_dim, grid_dim, increment_dist):
    # Rounding 4.499999999 => 4.5 and 4.5 => 5
    # With just round(): round(4.5) => 4 and round(3.5) => 4
    # So instead we use Decimal() with ROUND_HALF_UP
    # Then we clamp the coordinates between the grid boundaries
    cell = int(Decimal(round((pt[0] - world_dim[0]) / increment_dist, 3)).to_integral_value(rounding=ROUND_HALF_UP)),\
           int(Decimal(round((pt[1] - world_dim[2]) / increment_dist, 3)).to_integral_value(rounding=ROUND_HALF_UP)),\
           int(Decimal(round((pt[2] - world_dim[4]) / increment_dist, 3)).to_integral_value(rounding=ROUND_HALF_UP))
    cell = max(0, cell[0]), max(0, cell[1]), max(0, cell[2])
    cell = min(cell[0], grid_dim[0]-1), min(cell[1], grid_dim[1]-1), min(cell[2], grid_dim[2]-1)
    return cell


# ref: www.ambrsoft.com/TrigoCalc/Line3D/LineColinear.htm (Heron formula)
def arePointsAligned(a, b, c):
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = a, b, c
    a = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    b = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2 + (z3 - z1) ** 2)
    c = math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2 + (z3 - z2) ** 2)
    s = (a + b + c) / 2
    A = s * (s - a) * (s - b) * (s - c)
    return isclose(A, 0., abs_tol=1e-3)

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

GOAL_SAFETY_MARGIN = .15
class NonIncrementalPathFinder:
    def __init__(self, func, ros_node, start, goal, world_dim, display=True):
        self.func = func
        self.ros_node = ros_node
        self.world_dim = world_dim

        assert world_dim[0] < world_dim[1] and world_dim[2] < world_dim[3] and world_dim[4] < world_dim[5], 'Uncoherent world dimensions'
        assert world_dim[0] <= start[0] and start[0] <= world_dim[1], 'Start not contained on world dimensions x axis'
        assert world_dim[2] <= start[1] and start[1] <= world_dim[3], 'Start not contained on world dimensions y axis'
        assert world_dim[4] <= start[2] and start[2] <= world_dim[5], 'Start not contained on world dimensions z axis'
        assert world_dim[0] <= goal[0] and goal[0] <= world_dim[1], 'Goal not contained on world dimensions x axis'
        assert world_dim[2] <= goal[1] and goal[1] <= world_dim[3], 'Goal not contained on world dimensions y axis'
        assert world_dim[4] <= goal[2] and goal[2] <= world_dim[5], 'Goal not contained on world dimensions z axis'
        
        if self.ros_node.get_point_edt(start, UAV_THICKNESS) <= GOAL_SAFETY_MARGIN:
            start = self.make_valid_point(start)
        if self.ros_node.get_point_edt(goal, UAV_THICKNESS) <= GOAL_SAFETY_MARGIN:
            goal = self.make_valid_point(goal)
        
        self.start = goal
        self.goal = start
        self.display = display
        self.path = None
    

    # Move the goal or start position to a more appropriate one, further from the obstacles
    def make_valid_point(self, point):
        # Move the goal by this distance
        for d in np.arange(.1, 2.5, .1):
            l = []
            for yaw in np.arange(0, 2 * np.pi, 2*np.pi/100.):
                new_point = np.array([d * np.cos(yaw) + point[0], d * np.sin(yaw) + point[1], point[2]])
                c = self.ros_node.get_point_edt(new_point, UAV_THICKNESS)
                if  c > GOAL_SAFETY_MARGIN and \
                    self.world_dim[0] <= new_point[0] and new_point[0] <= self.world_dim[1] and \
                    self.world_dim[2] <= new_point[1] and new_point[1] <= self.world_dim[3] and \
                    self.world_dim[4] <= new_point[2] and new_point[2] <= self.world_dim[5]:
                        l.append((new_point, c))

            # If there is no valid new goal in this radius, we go to a further one
            if len(l) == 0:
                continue

            try:
                return min(l, key=lambda e: e[1])[0]
            except ValueError:
                continue
        return point
        raise AssertionError('No ideal position found to move the goal or start position')


    def init_graph(self):
        pass

    # For non incremental algorithms, we simply check if the path is still feasible and discard it if not
    def clean_graph(self, bbmin, bbmax):
        if self.path is not None:
            for i in range(len(self.path)-1):
                a, b = self.path[i], self.path[i+1]
                if self.ros_node.cast_ray(a, b, radius=UAV_THICKNESS)[0]:
                    self.path = None
                    return


    def update_graph(self):
        if self.path is not None:
            return self.path, 0.
        path, duration = self.func(self.ros_node, self.start, self.goal, self.world_dim, self.display)
        self.path = path
        return path, duration

    def check_nodes(self):
        self.clean_graph(None, None)
    
    def update_goal(self, pos):
        assert self.world_dim[0] <= pos[0] and pos[0] <= self.world_dim[1], 'Goal not contained on world dimensions x axis'
        assert self.world_dim[2] <= pos[1] and pos[1] <= self.world_dim[3], 'Goal not contained on world dimensions y axis'
        assert self.world_dim[4] <= pos[2] and pos[2] <= self.world_dim[5], 'Goal not contained on world dimensions z axis'
        if self.ros_node.get_point_edt(pos, UAV_THICKNESS) <= GOAL_SAFETY_MARGIN:
            pos = self.make_valid_point(pos)

        self.goal = pos
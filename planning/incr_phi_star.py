import numpy as np, math, sys, time, matplotlib.pyplot as plt
from functools import reduce
from collections import deque
from utils import dist, Node_phistar as Node, UAV_THICKNESS, pointToCell, NoPathFound, arePointsAligned


# Grid resolution
INCREMENT_DISTANCE = .2

# F(s) = G(s) + H_COST_WEIGHT * H(s)
H_COST_WEIGHT = 3

# Max tolerable distance between the goal/start and an obstacle (without the size of the UAV) 
GOAL_SAFETY_MARGIN = .15

# Cost of the heuristic distance in the Z axis
W_Z = 10

class PhiStarPathFinder:
    # Global variables
    openset = set()
    closedset = set()
    grid = []


    def __init__(self, ros_node, start, goal, world_dim, display=True):
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
        
        self.start = start
        self.goal = goal
        self.display = display

    # Return all the children (neighbors) of a specific node
    # crossbar: Only considers the 12 extremities of the vertical and horizontal face from which the point is the corner
    # otherwise considers the 26 possible neighbors in every direction
    def children(self, node, crossbar=True, checkLOS=True):
        # The comments are to disable looking at 3D path. The performances will be improved greatly
        if crossbar:
            # directions = np.array([[0,1,1], [0,1,-1], [0,-1,-1], [0,-1,1], 
            #                     [1,1,0], [-1,1,0], [-1,-1,0], [1,-1,0],
            #                     [1,0,1], [-1,0,1], [-1,0,-1], [1,0,-1]])
            directions = np.array([[1,1,0], [-1,1,0], [-1,-1,0], [1,-1,0]])
        else:
            # directions = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [-1, 1, 1], [-1, 0, 1], [-1, -1, 1], [0, -1, 1], [1, -1, 1], 
            #                                 [1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0], 
            #                     [0, 0,-1], [1, 0,-1], [1, 1,-1], [0, 1,-1], [-1, 1,-1], [-1, 0,-1], [-1, -1,-1], [0, -1,-1], [1, -1,-1]])
            directions = np.array([[1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0]])
        potential_children = node.pos + directions * INCREMENT_DISTANCE
        children = []
        for c in potential_children:
            if self.world_dim[0] <= c[0] <= self.world_dim[1] and self.world_dim[2] <= c[1] <= self.world_dim[3] and self.world_dim[4] <= c[2] <= self.world_dim[5]:
                cell = pointToCell(c, self.world_dim, self.grid.shape, INCREMENT_DISTANCE)
                child = self.grid[cell[0], cell[1], cell[2]]

                if child is None:
                    child = Node(c)
                    self.grid[cell[0], cell[1], cell[2]] = child
                
                if not checkLOS or not self.ros_node.cast_ray(node.pos, child.pos, radius=UAV_THICKNESS)[0]:
                    children.append(child)
        return children


    # Return the spherical angles between the nodes, as in the paper
    def phi(self, a, b, c):
        a, b, c = a.pos, b.pos, c.pos
        p = (180./math.pi) * (-math.atan2(a[1]-b[1], a[0]-b[0]) + math.atan2(c[1]-b[1], c[0]-b[0]))
        if p > 180:
            p = - (180 - (p % 180)) # Set angle between [-180; 180]

        t = (180./math.pi) * (-math.atan2(a[2]-b[2], math.sqrt((a[0]-b[0]) ** 2 + (a[1]-b[1]) ** 2)) + math.atan2(c[2]-b[2], math.sqrt((c[0]-b[0]) ** 2 + (c[1]-b[1]) ** 2)))
        if t > 180:
            t = - (180 - (t % 180)) # Set angle between [-180; 180]
        return p, t


    def pathTie(self, node, current):
        return arePointsAligned(current.parent.pos, current.pos, node.pos)
        

    def updateVertex(self, current, node):
        p, t = self.phi(current, current.parent, node) if current.parent else (None, None)

        if current.parent and not self.ros_node.cast_ray(current.parent.pos, node.pos, radius=UAV_THICKNESS)[0] \
                        and current.lb_phi <= p <= current.ub_phi \
                        and current.lb_the <= t <= current.ub_the \
                        and not self.pathTie(node, current):
            # Path 2
            new_g = current.parent.G + dist(current.parent, node)
            if new_g < node.G:
                node.G = new_g
                node.parent = current.parent
                node.local = current
                neighbors = np.array(list(map(lambda nb: self.phi(node, current.parent, nb), self.children(node, crossbar=True, checkLOS=False))))
                l_phi, h_phi = neighbors[:,0].min(), neighbors[:,0].max()
                l_the, h_the = neighbors[:,1].min(), neighbors[:,1].max()
                delta_phi, delta_the = self.phi(current, current.parent, node)
                node.lb_phi = max(l_phi, current.lb_phi - delta_phi)
                node.ub_phi = min(h_phi, current.ub_phi - delta_phi)
                node.lb_the = max(l_the, current.lb_the - delta_the)
                node.ub_the = min(h_the, current.ub_the - delta_the)
        else:
            # Path 1
            new_g = current.G + dist(current, node)
            if new_g < node.G:
                node.G = new_g
                node.parent = current
                node.local = current
                neighbors = np.array(list(map(lambda nb: self.phi(node, current, nb), self.children(node, crossbar=True, checkLOS=False))))
                node.lb_phi, node.ub_phi = neighbors[:,0].min(), neighbors[:,0].max()
                node.lb_the, node.ub_the = neighbors[:,1].min(), neighbors[:,1].max()


    # Return the path computed by the A* optimized algorithm from the start and goal points
    def find_path(self):
        if len(self.openset) == 0:
            self.openset.add(self.start)

        i = 0
        startTime = time.time()
        while self.openset and min(map(lambda o: o.G + H_COST_WEIGHT * o.H, self.openset)) < self.goal.G + H_COST_WEIGHT * self.goal.H and time.time() - startTime <= 60*2:
            i = i + 1
            current = min(self.openset, key=lambda o: o.G + H_COST_WEIGHT * o.H)

            self.openset.remove(current)
            self.closedset.add(current)

            # Loop through the node's children/siblings
            for node in self.children(current, crossbar=False):
                # If it is already in the closed set, skip it
                if node in self.closedset:
                    continue

                if node not in self.openset:
                    node.reset()
                    node.H = dist(node, self.goal, w_z=W_Z)
                    self.openset.add(node)
                
                self.updateVertex(current, node)
            
            if self.display and i % 10 == 0:
                self.ros_node.visualize_global_path(nodes=self.openset.union(self.closedset), start=self.start.pos, goal=self.goal.pos)
                self.ros_node.rate.sleep()

        if not self.goal.parent:
            raise NoPathFound
        
        path = []
        current = self.goal
        while current.parent:
            path.append(current.pos)
            current = current.parent
        path.append(current.pos)
        return path[::-1]


    def build_graph(self):
        grid_shape = (int((self.world_dim[1] - self.world_dim[0]) // INCREMENT_DISTANCE),
                    int((self.world_dim[3] - self.world_dim[2]) // INCREMENT_DISTANCE),
                    int((self.world_dim[5] - self.world_dim[4]) // INCREMENT_DISTANCE))
        self.grid = np.full(grid_shape, None)
        cells = pointToCell(self.start, self.world_dim, self.grid.shape, INCREMENT_DISTANCE)
        cellg = pointToCell(self.goal, self.world_dim, self.grid.shape, INCREMENT_DISTANCE)

        self.grid[cells[0],cells[1],cells[2]] = Node(self.start)
        self.grid[cellg[0],cellg[1],cellg[2]] = Node(self.goal)
        self.start = self.grid[cells[0],cells[1],cells[2]]
        self.goal = self.grid[cellg[0],cellg[1],cellg[2]]

        self.goal.H, self.start.G, self.start.H = 0, 0, dist(self.start, self.goal)

        self.openset = set()
        self.closedset = set()

        start_time = time.time()
        path = self.find_path()
        duration = abs(time.time() - start_time)

        return path, duration
    

    # Incremental execution of the find path algorithm
    # bbmin and bbmax are the bounding box of a newly added obstacle
    def update_graph(self, bbmin, bbmax):
        cmin = pointToCell(bbmin, self.world_dim, self.grid.shape, INCREMENT_DISTANCE)
        cmax = pointToCell(bbmax, self.world_dim, self.grid.shape, INCREMENT_DISTANCE)
        cmin = min(max(0, cmin[0]-1), self.grid.shape[0]-1), min(max(0, cmin[1]-1), self.grid.shape[1]-1), min(max(0, cmin[2]-1), self.grid.shape[2]-1)
        cmax = min(max(0, cmax[0]+1), self.grid.shape[0]-1), min(max(0, cmax[1]+1), self.grid.shape[1]-1), min(max(0, cmax[2]+1), self.grid.shape[2]-1)

        for i in range(min(cmin[0], cmax[0]), max(cmin[0], cmax[0])+1):
            for j in range(min(cmin[1], cmax[1]), max(cmin[1], cmax[1])+1):
                for k in range(min(cmin[2], cmax[2]), max(cmin[2], cmax[2])+1):
                    if (self.grid[i,j,k] in self.openset or self.grid[i,j,k] in self.closedset) and self.grid[i,j,k] != self.start:
                        self.clearSubtree(self.grid[i,j,k])

        start_time = time.time()
        path = self.find_path()
        duration = abs(time.time() - start_time)

        return path, duration


    def clearSubtree(self, node):
        under, over = deque(), deque()
        under.append(node)

        while under:
            node = under.popleft()
            over.append(node)
            node.reset()

            self.openset.discard(node)
            self.closedset.discard(node)

            for neigh in self.children(node, crossbar=False, checkLOS=False):
                if neigh.local == node:
                    under.append(neigh)
        
        while over:
            node = over.popleft()
            for neigh in self.children(node, crossbar=False, checkLOS=True):
                if neigh in self.closedset:
                    g_old = node.G
                    self.updateVertex(neigh, node)
                    if node.G < g_old:
                        self.openset.add(node)


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
        raise AssertionError('No ideal position found to move the goal or start position')


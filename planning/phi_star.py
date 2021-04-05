import numpy as np, math, sys, time, matplotlib.pyplot as plt
from functools import reduce
from collections import deque
from utils import dist, Node_phistar as Node, UAV_THICKNESS, pointToCell, NoPathFound, arePointsAligned


# Grid resolution
INCREMENT_DISTANCE = .4

# F(s) = G(s) + H_COST_WEIGHT * H(s)
H_COST_WEIGHT = 1.8

# Global variables
openset = set()
closedset = set()
grid = []

# Return all the children (neighbors) of a specific node
# crossbar: Only considers the 12 extremities of the vertical and horizontal face from which the point is the corner
# otherwise considers the 26 possible neighbors in every direction
def children(node, ros_node, world_dim, grid, crossbar=True, checkLOS=True):
    if crossbar:
        directions = np.array([[0,1,1], [0,1,-1], [0,-1,-1], [0,-1,1], 
                               [1,1,0], [-1,1,0], [-1,-1,0], [1,-1,0],
                               [1,0,1], [-1,0,1], [-1,0,-1], [1,0,-1]])
    else:
        directions = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [-1, 1, 1], [-1, 0, 1], [-1, -1, 1], [0, -1, 1], [1, -1, 1], 
                                          [1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0], 
                               [0, 0,-1], [1, 0,-1], [1, 1,-1], [0, 1,-1], [-1, 1,-1], [-1, 0,-1], [-1, -1,-1], [0, -1,-1], [1, -1,-1]])
    potential_children = node.pos + directions * INCREMENT_DISTANCE
    children = []
    for c in potential_children:
        if world_dim[0] <= c[0] <= world_dim[1] and world_dim[2] <= c[1] <= world_dim[3] and world_dim[4] <= c[2] <= world_dim[5]:
            cell = pointToCell(c, world_dim, grid.shape, INCREMENT_DISTANCE)
            child = grid[cell[0], cell[1], cell[2]]

            if child is None:
                child = Node(c)
                grid[cell[0], cell[1], cell[2]] = child
            
            if not checkLOS or not ros_node.cast_ray(node.pos, c, radius=UAV_THICKNESS)[0]:
                children.append(child)
    return children


def phi(a,b,c):
    a, b, c = a.pos, b.pos, c.pos
    p = (180./math.pi) * (-math.atan2(a[1]-b[1], a[0]-b[0]) + math.atan2(c[1]-b[1], c[0]-b[0]))
    if p > 180:
        p = - (180 - (p % 180)) # Set angle between [-180; 180]

    t = (180./math.pi) * (-math.atan2(a[2]-b[2], math.sqrt((a[0]-b[0]) ** 2 + (a[1]-b[1]) ** 2)) + math.atan2(c[2]-b[2], math.sqrt((c[0]-b[0]) ** 2 + (c[1]-b[1]) ** 2)))
    if t > 180:
        t = - (180 - (t % 180)) # Set angle between [-180; 180]
    return p, t


def pathTie(node, current):
    return arePointsAligned(current.parent.pos, current.pos, node.pos)
    

def updateVertex(ros_node, current, node, grid, world_dim):
    p, t = phi(current, current.parent, node) if current.parent else (None, None)

    if current.parent and not ros_node.cast_ray(current.parent.pos, node.pos, radius=UAV_THICKNESS)[0] \
                    and current.lb_phi <= p <= current.ub_phi \
                    and current.lb_the <= t <= current.ub_the \
                    and not pathTie(node, current):
        # Path 2
        new_g = current.parent.G + dist(current.parent, node)
        if new_g < node.G:
            node.G = new_g
            node.parent = current.parent
            node.local = current
            neighbors = np.array(list(map(lambda nb: phi(node, current.parent, nb), children(node, ros_node, world_dim, grid, crossbar=True, checkLOS=False))))
            l_phi, h_phi = neighbors[:,0].min(), neighbors[:,0].max()
            l_the, h_the = neighbors[:,1].min(), neighbors[:,1].max()
            delta_phi, delta_the = phi(current, current.parent, node)
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
            neighbors = np.array(list(map(lambda nb: phi(node, current, nb), children(node, ros_node, world_dim, grid, crossbar=True, checkLOS=False))))
            node.lb_phi, node.ub_phi = neighbors[:,0].min(), neighbors[:,0].max()
            node.lb_the, node.ub_the = neighbors[:,1].min(), neighbors[:,1].max()


# Return the path computed by the A* optimized algorithm from the start and goal points
def find_path(ros_node, start, goal, grid, world_dim, openset=set(), closedset=set(), display=True):
    if len(openset) == 0:
        openset.add(start)

    i = 0
    while openset and min(map(lambda o: o.G + H_COST_WEIGHT * o.H, openset)) < goal.G + H_COST_WEIGHT * goal.H:
        i = i + 1
        current = min(openset, key=lambda o: o.G + H_COST_WEIGHT * o.H)

        openset.remove(current)
        closedset.add(current)

        # Loop through the node's children/siblings
        for node in children(current, ros_node, world_dim, grid, crossbar=False):
            # If it is already in the closed set, skip it
            if node in closedset:
                continue

            if node not in openset:
                node.reset()
                node.H = dist(node, goal)
                openset.add(node)
            
            updateVertex(ros_node, current, node, grid, world_dim)
        
        if display and i % 10 == 0:
            ros_node.visualize_global_path(nodes=openset.union(closedset), start=start.pos, goal=goal.pos)
            ros_node.rate.sleep()

    if not goal.parent:
        raise NoPathFound
    
    path = []
    current = goal
    while current.parent:
        path.append(current.pos)
        current = current.parent
    path.append(current.pos)
    return path[::-1]

"""
def clearSubtree(node, grid, obs, openset, closedset):
    under, over = deque(), deque()
    under.append(node)

    while under:
        node = under.popleft()
        over.append(node)
        node.reset()

        openset.discard(node)
        closedset.discard(node)

        for neigh in children(node, grid, [], crossbar=False, checkLOS=False):
            if neigh.local == node:
                under.append(neigh)
    
    while over:
        node = over.popleft()
        for neigh in children(node, grid, obs, crossbar=False, checkLOS=True):
            if neigh in closedset:
                g_old = node.G
                updateVertex(neigh, node, grid, obs)
                if node.G < g_old:
                    openset.add(node)
"""


def phi_star(ros_node, start, goal, world_dim, display=True):
    grid_shape = (int((world_dim[1] - world_dim[0]) // INCREMENT_DISTANCE),
                  int((world_dim[3] - world_dim[2]) // INCREMENT_DISTANCE),
                  int((world_dim[5] - world_dim[4]) // INCREMENT_DISTANCE))
    global grid
    grid = np.full(grid_shape, None)
    cells = pointToCell(start, world_dim, grid.shape, INCREMENT_DISTANCE)
    cellg = pointToCell(goal, world_dim, grid.shape, INCREMENT_DISTANCE)
    grid[cells[0],cells[1],cells[2]], grid[cellg[0],cellg[1],cellg[2]] = Node(start), Node(goal)
    start, goal = grid[cells[0],cells[1],cells[2]], grid[cellg[0],cellg[1],cellg[2]]

    goal.H, start.G, start.H = 0, 0, dist(start, goal)

    openset = set()
    closedset = set()

    start_time = time.time()

    i = 0
    while True:
        i += 1
        path = find_path(ros_node, start, goal, grid, world_dim, openset, closedset, display)
        duration = abs(time.time() - start_time)
        break

        """
        if DISPLAY_END and WAIT_INPUT:
            blockedCells = plot.waitForInput(grid_obs, lambda: plot.display(start, goal, grid, grid_obs))
        else:
            try:
                blockedCells = next(newBlockedCells)
                updateGridBlockedCells(blockedCells, grid_obs)
            except StopIteration:
                break
              
        t1 = time.time()

        for pt in corners(blockedCells):
            if (grid[pt] in openset or grid[pt] in closedset) and grid[pt] != start:
                clearSubtree(grid[pt], grid, grid_obs, openset, closedset)
        """
    return path, duration


def main_phi_star(ros_node, start, goal, world_dim, display=True):
    assert world_dim[0] <= start[0] and start[0] <= world_dim[1] and world_dim[0] < world_dim[1]
    assert world_dim[2] <= start[1] and start[1] <= world_dim[3] and world_dim[2] < world_dim[3]
    assert world_dim[4] <= start[2] and start[2] <= world_dim[5] and world_dim[4] < world_dim[5]
    
    path, duration = phi_star(ros_node, start, goal, world_dim, display)

    return path, duration

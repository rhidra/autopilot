import numpy as np, math

class Node:
    def __init__(self, pos, parent):
        self.pos = pos
        self.parent = parent
        if parent:
            self.cost = parent.cost + dist(parent, pos)
        else:
            self.cost = 0

    def __repr__(self):
        return self.pos.__repr__()


def rand(a, b=None, integer=False):
    if integer:
        return np.random.randint(a) if b is None else np.random.randint(b - a) + a
    else:
        return np.random.uniform(0, a) if b is None else np.random.uniform(a, b)


def random_position(world_dim):
    xmin, xmax, ymin, ymax, zmin, zmax = world_dim
    return [rand(xmin, xmax), rand(ymin, ymax), rand(zmin, zmax)]


def dist(p1, p2, sqrt=False):
    if p1 is None or p2 is None:
        return math.inf

    p1 = p1.pos if isinstance(p1, Node) else p1
    p2 = p2.pos if isinstance(p2, Node) else p2

    sqr = (p1[0] - p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2
    return math.sqrt(sqr) if sqrt else sqr

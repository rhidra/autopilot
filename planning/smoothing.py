import numpy as np, copy, utils, time

UAV_THICKNESS = .5


"""
over_sampling()
Add points in a path
"""
def over_sampling(path, max_length=1):
    new = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i+1]
        new.append(a)
        last = a
        norm = utils.dist(a, b, sqrt=True) + 1e-6
        dir_x = (b[0]-a[0]) * max_length / norm
        dir_y = (b[1]-a[1]) * max_length / norm
        dir_z = (b[2]-a[2]) * max_length / norm
        while utils.dist(last, b) > max_length * max_length:
            last = [last[0] + dir_x, last[1] + dir_y, last[2] + dir_z]
            new.append(last)
    new.append(path[-1])
    return new


"""
filter()
Remove unnecessary waypoints
"""
def filter_path(path, ros_node):
    new = [path[0]]
    i = 0
    while i < len(path):
        for j, target in reversed(list(enumerate(path))[i:]):
            hit, _ = ros_node.cast_ray(path[i], target, radius=UAV_THICKNESS)
            if not hit:
                new.append(target)
                i = j
                break
        i += 1
    return new


"""
bezier()
Bezier smoothing algorithm.
Smooth the curve by dividing it in small segments.
"""
def bezier(path):
    new = []

    # Divide the curve in small group of 4 points
    for i in range(0, len(path)-3, 3):
        a, b, c, d = path[i], path[i+1], path[i+2], path[i+3]
        for l in np.arange(0,1,.1):
            x = (1-l)**3 * a[0] + 3*l*(1-l)**2 * b[0] + 3*l**2*(1-l) * c[0] + l**3 * d[0]
            y = (1-l)**3 * a[1] + 3*l*(1-l)**2 * b[1] + 3*l**2*(1-l) * c[1] + l**3 * d[1]
            new.append([x,y])
        new.append(d)
    if len(path) % 4 != 0:
        new = new + path[-(len(path)%4):]
    return new
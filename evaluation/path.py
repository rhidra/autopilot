import numpy as np, math, json, rospy

def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)


def evaluate_path(path, processing_time, ros_node, situation='1', algo_name='A*'):
    rospy.loginfo('Computing path stats...')
    relative_length, waypoints_distance = path_length(path)
    rotations = path_rotation(path)
    collision_checks = ros_node.collision_check if hasattr(ros_node, 'collision_check') else 0

    data = dict(
        world='test_zone.world',
        situation=situation,
        algorithm=algo_name,
        path=path,
        relative_length=relative_length,
        processing_time=processing_time,
        waypoints_distance=waypoints_distance,
        rotations=rotations,
        collision_checks=collision_checks,
    )

    rospy.loginfo('Writing path stats to JSON...')
    try:
        with open('results.json', 'r+') as f:
            prev_data = json.load(f)
    except (ValueError, IOError):
        prev_data = []
    
    prev_data.append(data)
    with open('results.json', 'w') as f:
        json.dump(prev_data, f, indent=2, sort_keys=True)


def path_length(path):
    if len(path) < 2:
        return 0.

    straight_path = dist(path[0], path[-1])

    path_length = 0.
    waypoints_distance = []
    for i in range(len(path) - 1):
        path_length += dist(path[i], path[i + 1])
        waypoints_distance.append(dist(path[i], path[i + 1]))
    
    return (path_length / straight_path) - 1., waypoints_distance


def path_rotation(path):
    # Compute the angle of rotation along a path
    angles = []
    for i in range(1, len(path)-1):
        a, b, c = np.array(path[i-1]), np.array(path[i]), np.array(path[i+1])
        vec1, vec2 = b - a, c - b
        angles.append(angle_between(vec1, vec2))
    return angles


def angle_between(v1, v2):
    # Returns the angle between vectors 'v1' and 'v2'
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * 180 / np.pi


def unit_vector(vector):
    # Returns the unit vector of the vector
    return vector / np.linalg.norm(vector) if np.linalg.norm(vector) != 0 else vector
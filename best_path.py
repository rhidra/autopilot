#!/usr/bin/env python
from node import LocalGoalNode
import rospy, numpy as np, sys, getopt
from evaluation import evaluate_path
from planning import main_a_star as a_star, NoPathFound

"""
best_path.py

Utyility used to compute the length of the best possible path in an environment.
Starts an A* global planning algorithm, with a very small resolution grid.
"""


def main():
    node = LocalGoalNode(node_name='global_planner')
    node.setup()

    world_dim = [
        rospy.get_param('/world/x/min', -20), rospy.get_param('/world/x/max', 20),
        rospy.get_param('/world/y/min', -20), rospy.get_param('/world/y/max', 20),
        rospy.get_param('/world/z/min', 0), rospy.get_param('/world/z/max', 4),
    ]

    start_pos = [float(rospy.get_param('/start/x', 0)), float(rospy.get_param('/start/y', 0)), float(rospy.get_param('/start/z', 1))]
    goal = [float(rospy.get_param('/goal/x', 6)), float(rospy.get_param('/goal/y', -7)), float(rospy.get_param('/goal/z', 1))]

    # Global Path planning
    rospy.loginfo('Start global path planning...')

    try:
        path, processing_time = a_star(node, start_pos, goal, world_dim=world_dim, display=False)
        path[-1][0], path[-1][1], path[-1][2] = goal[0], goal[1], goal[2]
    except NoPathFound as e:
        # No path found
        rospy.logerr('No path found')
        rospy.logerr(e)
        rospy.set_param('/autopilot/done', 3)
        exit(1)
    except AssertionError as e:
        # The goal is not valid
        rospy.logerr('Configuration not valid')
        rospy.logerr(e)
        rospy.set_param('/autopilot/done', 4)
        exit(1)
    
    rospy.loginfo('Global path found !')

    d = 0
    for i in range(len(path) - 1):
        d += np.linalg.norm(np.array(path[i]) - np.array(path[i + 1]))
    
    rospy.loginfo('Distance: {}'.format(d))

    with open('/home/rhidra/research_data/best_m{}_c{}_t{}.json'.format(node.mapId, node.configId, node.trialId), 'w') as f:
        f.write(str(d))

if __name__ == '__main__':
    main()
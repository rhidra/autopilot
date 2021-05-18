#!/usr/bin/env python
from node import LocalGoalNode
import rospy, numpy as np, sys, getopt
from evaluation import evaluate_path
from planning import PhiStarPathFinder, NoPathFound

help = """
Usage: python phi_star_planner.py -d

-b, --begin: start coordinates, comma sperated, e.g: --begin=6,-7,1
-e, --end: goal coordinates, comma sperated, e.g: --end=-6,7,1
-l, --local: Computes and sends the local goal continuously once the path is found.
-d, --display: Publish real-time information on Rviz.
"""

def start(start_pos, goal, display=True):
    node = LocalGoalNode(node_name='global_planner')
    node.setup()

    world_dim = [
        rospy.get_param('/world/x/min', -20), rospy.get_param('/world/x/max', 20),
        rospy.get_param('/world/y/min', -20), rospy.get_param('/world/y/max', 20),
        rospy.get_param('/world/z/min', 0), rospy.get_param('/world/z/max', 4),
    ]

    # Global Path planning
    rospy.loginfo('Start global path planning...')

    try:
        solver = PhiStarPathFinder(node, start_pos, goal, world_dim=world_dim, display=display)
        path, processing_time = solver.build_graph()
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
    
    node.load_solver(solver)
    node.load_local_path(path)
    node.send_local_goal()


def main(argv):
    display = False
    start_pos = [float(rospy.get_param('/start/x', 0)), float(rospy.get_param('/start/y', 0)), float(rospy.get_param('/start/z', 1))]
    goal = [float(rospy.get_param('/goal/x', 6)), float(rospy.get_param('/goal/y', -7)), float(rospy.get_param('/goal/z', 1))]

    try:
        opts, _ = getopt.getopt(argv,'hb:e:p:t:sld',['help', 'begin=', 'end=', 'planning=', 'test=', 'stats', 'local', 'display'])
    except getopt.GetoptError:
        print(help)
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print(help)
            sys.exit()
        elif opt in ('-b', '--begin'):
            start_pos = [float(a) for a in arg.split(',')]
        elif opt in ('-e', '--end'):
            goal = [float(a) for a in arg.split(',')]
        elif opt in ('-d', '--display'):
            display = True

    start(start_pos, goal, display=display)


if __name__ == '__main__':
   main(sys.argv[1:])
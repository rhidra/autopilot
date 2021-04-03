#!/usr/bin/env python
from node import LocalGoalNode
import rospy, numpy as np, sys, getopt
from evaluation import evaluate_path
from planning import dummy_path, main_rrt_star, main_a_star, main_rrt_star_without_optim, main_theta_star, main_phi_star

help = """
Usage: python global_planner.py -p <algo> -d

-b, --begin: start coordinates, comma sperated, e.g: --begin=6,-7,1
-e, --end: goal coordinates, comma sperated, e.g: --end=-6,7,1
-p, --planning: (Mandatory) Planning algorithm. 
Can be either: A*, RRT*, RRT_star_without_optim, Theta*, Phi* or dummy
-t, --test: Name of the test. Used in the results JSON file.
-s, --stats: Saves the statistics of the path found in 'results.json'. 
-l, --local: Computes and sends the local goal continuously once the path is found.
-d, --display: Publish real-time information on Rviz.
"""

def start(planning_algo, algo_name, start_pos, goal, situation, save_stats=False, send_local=True, display=True):
    node = LocalGoalNode(node_name='global_planner')
    node.setup()

    # Global Path planning
    rospy.loginfo('Start global path planning...')
    # path, processing_time = planning_algo(node, [0, 0, 1], [-6, -7, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    path, processing_time = planning_algo(node, start_pos, goal, world_dim=[-25, 25, -25, 25, 0, 4], display=display)
    # path, processing_time = planning_algo(node, [0, 0, 1], [0, -7, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    # path, processing_time = planning_algo(node, [0, 0, 1], [0, 7, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    rospy.loginfo('Global path found !')

    node.load_local_path(path)

    if save_stats:
        evaluate_path(path, processing_time, node, situation=situation, algo_name=algo_name)

    if send_local:
        node.send_local_goal()


def main(argv):
    algo = None
    situation = 'test'
    save_stats, send_local, display = False, False, False
    start_pos = [rospy.get_param('/start/x', 0), rospy.get_param('/start/y', 0), rospy.get_param('/start/z', 0) + 1]
    goal = [rospy.get_param('/goal/x', 6), rospy.get_param('/goal/y', -7), rospy.get_param('/goal/z', 1)]

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
        elif opt in ('-p', '--planning'):
            print(arg.lower())
            if arg.lower() == 'dummy':
                algo, algo_name = dummy_path, 'Dummy'
            elif arg.lower() == 'rrt_star':
                algo, algo_name = main_rrt_star, 'RRT*'
            elif arg.lower() == 'rrt_star_without_optim':
                algo, algo_name = main_rrt_star_without_optim, 'RRT* Without Optimization'
            elif arg.lower() == 'a_star':
                algo, algo_name = main_a_star, 'A*'
            elif arg.lower() == 'theta_star':
                algo, algo_name = main_theta_star, 'Theta*'
            elif arg.lower() == 'phi_star':
                algo, algo_name = main_phi_star, 'Phi*'
            else:
                print('This planning algorithm does not exist !\nUse --help to see available algorithms')
                sys.exit()
        elif opt in ('-t', '--test'):
            situation = arg
        elif opt in ('-s', '--stats'):
            save_stats = True
        elif opt in ('-l', '--local'):
            send_local = True
        elif opt in ('-d', '--display'):
            display = True

    if algo is None:
        print('No planning algorithm given with the --planning option !')
        print('Use --help for more info')
        sys.exit()

    start(algo, algo_name, start_pos, goal, situation=situation, save_stats=save_stats, send_local=send_local, display=display)


if __name__ == '__main__':
   main(sys.argv[1:])
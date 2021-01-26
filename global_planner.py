#!/usr/bin/env python
from node import LocalGoalNode
import rospy, numpy as np, sys, getopt
from evaluation import evaluate_path
from planning import dummy_path, main_rrt_star, main_a_star, main_rrt_star_without_optim, main_theta_star, main_phi_star

help = """
Usage: python main.py -p <algo> -d

-p, --planning: (Mandatory) Planning algorithm. 
Can be either: A*, RRT*, RRT_star_without_optim, Theta*, Phi* or dummy
-t, --test: Name of the test. Used in the results JSON file.
-s, --stats: Saves the statistics of the path found in 'results.json'. 
-l, --launch: Execute the planned mission on ROS.
-d, --display: Publish real-time information on Rviz.
"""

def start(planning_algo, algo_name, situation, save_stats=False, launch_mission=True, display=True):
    node = LocalGoalNode(node_name='local_goal_generator')
    node.setup()

    # Global Path planning
    path, processing_time = planning_algo(node, [0, 0, 1], [0, -7, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)

    node.load_local_path(path)

    if save_stats:
        evaluate_path(path, processing_time, node, situation=situation, algo_name=algo_name)

    if launch_mission:
        node.send_local_goal()


def main(argv):
    algo = None
    situation = 'test'
    save_stats, launch_mission, display = False, False, False

    try:
        opts, _ = getopt.getopt(argv,'hp:t:sld',['help', 'planning=', 'test=', 'stats', 'launch', 'display'])
    except getopt.GetoptError:
        print(help)
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print(help)
            sys.exit()
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
        elif opt in ('-l', '--launch'):
            launch_mission = True
        elif opt in ('-d', '--display'):
            display = True

    if algo is None:
        print('No planning algorithm given with the --planning option !')
        print('Use --help for more info')
        sys.exit()

    start(algo, algo_name, situation=situation, save_stats=save_stats, launch_mission=launch_mission, display=display)


if __name__ == '__main__':
   main(sys.argv[1:])
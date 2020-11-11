#!/usr/bin/env python
import sys, getopt, autopilot
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

planning_map = {
    'dummy': {
        'algo': dummy_path,
        'name': 'Dummy',
    },
    'rrt*': {
        'algo': main_rrt_star,
        'name': 'RRT*',
    },
    'rrt_star': {
        'algo': main_rrt_star,
        'name': 'RRT*',
    },
    'rrt_star_without_optim': {
        'algo': main_rrt_star_without_optim,
        'name': 'RRT* Without Optimization',
    },
    'a*': {
        'algo': main_a_star,
        'name': 'A*',
    },
    'a_star': {
        'algo': main_a_star,
        'name': 'A*',
    },
    'theta_star': {
        'algo': main_theta_star,
        'name': 'Theta*',
    },
    'theta*': {
        'algo': main_theta_star,
        'name': 'Theta*',
    },
    'phi*': {
        'algo': main_phi_star,
        'name': 'Phi*',
    },
    'phi_star': {
        'algo': main_phi_star,
        'name': 'Phi*',
    },
}

def main(argv):
    planning_algo = None
    situation = 'test'
    save_stats = False
    launch_mission = False
    display = False

    try:
        opts, args = getopt.getopt(argv,'hp:t:sld',['help', 'planning=', 'test=', 'stats', 'launch', 'display'])
    except getopt.GetoptError:
        print(help)
        sys.exit(2)
    
    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print(help)
            sys.exit()
        elif opt in ('-p', '--planning'):
            try:
                planning_algo = planning_map[arg.lower()]
            except KeyError:
                print('This planning algorithm does not exists !')
                print('Use --help to see available algorithms')
                sys.exit()
        elif opt in ('-t', '--test'):
            situation = arg
        elif opt in ('-s', '--stats'):
            save_stats = True
        elif opt in ('-l', '--launch'):
            launch_mission = True
        elif opt in ('-d', '--display'):
            display = True

    if planning_algo is None:
        print('No planning algorithm given with the --planning option !')
        print('Use --help for more info')
        sys.exit()

    autopilot.start(planning_algo['algo'], planning_algo['name'], situation=situation, save_stats=save_stats, launch_mission=launch_mission, display=display)

if __name__ == '__main__':
   main(sys.argv[1:])

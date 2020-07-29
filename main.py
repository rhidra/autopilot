#!/usr/bin/env python
import sys, getopt, autopilot
from planning import dummy_path, rrt_star

help = """
Usage: python main.py -p <algo>

You need to specify the path planning algorithm with --planning:
RRT, RRT*, A*, dummy (Only for testing purposes)
"""

planning_map = {
    'dummy': dummy_path,
    'rrt*': rrt_star,
    'rrt_star': rrt_star,
}

def main(argv):
    planning_algo = None

    try:
      opts, args = getopt.getopt(argv,'hp:',['help', 'planning='])
    except getopt.GetoptError:
      print(help)
      sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print(help)
            sys.exit()
        elif opt in ('-p', '--planning'):
            try:
                planning_algo = planning_map[arg.lower()]
            except KeyError:
                print('This planning algorithm does not exists !')
                print('Use --help to see available algorithms')
                sys.exit()

    if planning_algo is None:
        print('No planning algorithm given with the --planning option !')
        print('Use --help for more info')
        sys.exit()

    autopilot.start(planning_algo)

if __name__ == '__main__':
   main(sys.argv[1:])

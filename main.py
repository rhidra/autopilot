#!/usr/bin/python3

import sys, getopt

import autopilot
from planning import dummyPath

help = """
Usage: python main.py -p <algo>

You need to specify the path planning algorithm with --planning:
rrt, rrt*, a*, dummy (Only for testing purposes)
"""

planning_map = {
    'dummy': dummyPath,
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

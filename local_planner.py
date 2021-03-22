#!/usr/bin/env python
import sys, getopt
from node import MotionPrimitiveNode

help = """
Usage: python local_planner.py

Trajectory planning ROS node. At each generation, optimize a trajectory
to attain the goal broadcasted at /autopilot/local_goal.
Generate a motion primitive library, compute a cost for each one,
and select one to execute.
"""

def start():
    node = MotionPrimitiveNode(node_name='local_planner')
    node.setup()

def main(argv):
    start()

if __name__ == '__main__':
   main(sys.argv[1:])

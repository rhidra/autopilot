#!/usr/bin/env python3
from planning import dummyPath
from node import MissionNode
import rospy


def start(planning_algo):
    node = MissionNode(node_name='autopilot')
    node.setup()

    # Path planning
    path = planning_algo(None)

    node.load_local_path(path)

    # node.exec_mission()

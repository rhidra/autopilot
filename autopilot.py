#!/usr/bin/env python3
from node import MissionNode
import rospy, numpy as np


def start(planning_algo):
    node = MissionNode(node_name='autopilot')
    node.setup()


    # Path planning
    path, _, _, _ = planning_algo(node, [0, 0, 4], [-3, 6, 7], world_dim=[-100, 100, -100, 100, 0, 50])
    # path, _, _, _ = planning_algo(node, [-4, -3, 7], [0, 0, 4], world_dim=[-100, 100, -100, 100, 0, 50])
    rospy.loginfo(path)

    node.load_local_path(path)

    node.exec_mission()

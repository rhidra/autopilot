#!/usr/bin/env python3
from node import OffboardNode
import rospy, numpy as np


def start(planning_algo):
    node = OffboardNode(node_name='autopilot')
    node.setup()

    # Path planning
    path, _, _, _ = planning_algo(node, [0, 0, 4], [0, -10.5, 4], world_dim=[-20, 20, -10, 10, 0, 5.5])
    # path, _, _, _ = planning_algo(node, [0, -10.5, 4], [0, 0, 4], world_dim=[-20, 20, -10, 10, 0, 5.5])

    # path, _, _, _ = planning_algo(node, [0, 0, 4], [0, 10, 4], world_dim=[-20, 20, -10, 10, 0, 2.8*2])
    # path, _, _, _ = planning_algo(node, [0, -10.5, 4], [0, 0, 4], world_dim=[-20, 20, -10, 10, 0, 2.8*2])

    # path, _, _, _ = planning_algo(node, [0, 0, 4], [-3.5, -3.2, 7], world_dim=[-100, 100, -100, 100, 0, 50])
    # path, _, _, _ = planning_algo(node, [-3.5, -3.2, 7], [0, 0, 4], world_dim=[-100, 100, -100, 100, 0, 50])

    node.load_local_path(path)

    rospy.loginfo('Viz Path:' + str(node.viz_path))
    rospy.loginfo('Path:' + str(node.path))
    node.exec_mission()

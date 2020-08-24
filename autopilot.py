#!/usr/bin/env python3
from node import OffboardNode
import rospy, numpy as np
from evaluation import evaluate_path


def start(planning_algo, algo_name, situation, save_stats=False, launch_mission=True, display=True):
    node = OffboardNode(node_name='autopilot')
    node.setup()

    # Path planning
    # path, _, _, _ = planning_algo(node, [0, 0, 1], [5, 0, 1], world_dim=[-20, 20, -10, 10, 0, 3])
    path, processing_time = planning_algo(node, [0, 0, 1], [-5, -7.5, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    # path, _, _, _ = planning_algo(node, [0, 0, 1], [2, -10.5, 1], world_dim=[-20, 20, -10, 10, 0, 3])
    # path, _, _, _ = planning_algo(node, [-5, -7.5, 1], [0, 0, 1], world_dim=[-20, 20, -10, 10, 0, 3])
    # path, _, _, _ = planning_algo(node, [0, 0, 2], [0, -10.5, 2], world_dim=[-4, 12, -13, 3, 0, 5])

    # path, _, _, _ = planning_algo(node, [0, 0, 4], [0, 10, 4], world_dim=[-20, 20, -10, 10, 0, 2.8*2])
    # path, _, _, _ = planning_algo(node, [0, -10.5, 4], [0, 0, 4], world_dim=[-20, 20, -10, 10, 0, 2.8*2])

    # path, _, _, _ = planning_algo(node, [0, 0, 4], [-3.5, -3.2, 7], world_dim=[-100, 100, -100, 100, 0, 50])
    # path, _, _, _ = planning_algo(node, [-3.5, -3.2, 7], [0, 0, 4], world_dim=[-100, 100, -100, 100, 0, 50])

    node.load_local_path(path)

    if save_stats:
        evaluate_path(path, processing_time, node, situation=situation, algo_name=algo_name)

    if launch_mission:
        node.exec_mission()


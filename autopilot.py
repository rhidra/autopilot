#!/usr/bin/env python3
from node import OffboardNode
import rospy, numpy as np
from evaluation import evaluate_path


def start(planning_algo, algo_name, situation, save_stats=False, launch_mission=True, display=True):
    node = OffboardNode(node_name='autopilot')
    node.setup()

    # Path planning
    path, processing_time = planning_algo(node, [0, 0, 1], [6, 7, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    
    node.load_local_path(path)

    if save_stats:
        evaluate_path(path, processing_time, node, situation=situation, algo_name=algo_name)

    if launch_mission:
        node.exec_mission()


#!/usr/bin/env python3
from node import OffboardNode
import rospy, numpy as np
from evaluation import evaluate_path


def start(planning_algo, algo_name, situation, save_stats=False, launch_mission=True, display=True):
    node = OffboardNode(node_name='autopilot')
    node.setup()

    # Path planning

    ## TEST A
    # path, processing_time = planning_algo(node, [0, 0, 1], [6, 7, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    
    ## TEST B
    # path, processing_time = planning_algo(node, [0, 0, 1], [-5, -7.5, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)

    ## TEST C
    # path, processing_time = planning_algo(node, [1, 7, 1], [6, -7.5, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)

    ## TEST D
    # path, processing_time = planning_algo(node, [0, 0, 1], [-4, 5.5, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)

    ## TEST E
    # path, processing_time = planning_algo(node, [-5, -7.5, 1], [-4, 5.5, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)

    ## TEST F
    # path, processing_time = planning_algo(node, [-5, -7.5, 1], [1.83, 6.83, 4], world_dim=[-20, 20, -10, 10, 0, 5], display=display)

    ## TEST G
    path, processing_time = planning_algo(node, [-7, 8, 1], [10, -3, 1], world_dim=[-20, 20, -10, 10, 0, 3], display=display)
    
    node.load_local_path(path)

    if save_stats:
        evaluate_path(path, processing_time, node, situation=situation, algo_name=algo_name)

    if launch_mission:
        node.exec_mission()


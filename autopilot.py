#!/usr/bin/env python3
from mapping import getWorld, localToGlobal, ORIGIN_LAT, ORIGIN_LON
from planning import dummyPath
from node import MissionNode


def start(planning_algo):
    # Path planning
    path = planning_algo(None)
    print('Local path:', path)

    # Convert the path to global GPS coordinates
    path_global = localToGlobal(ORIGIN_LAT, ORIGIN_LON, path)

    print('Converted path:', path_global)

    # Send the path as a mission to MAVROS
    node = MissionNode(node_name='autopilot')
    node.load_waypoints(path_global)
    node.main()

#!/usr/bin/env python
from node import TrajectorySamplerNode

def main():
    node = TrajectorySamplerNode(node_name='trajectory_sampler', generateEDT=False)
    node.setup()

    node.execute_trajectory()


if __name__ == '__main__':
   main()

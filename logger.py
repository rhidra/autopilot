#!/usr/bin/env python
from node import LoggerNode

def main():
    node = LoggerNode(node_name='logger')
    node.setup()
    if not node.isLogging:
        return
    node.execute()


if __name__ == '__main__':
    main()

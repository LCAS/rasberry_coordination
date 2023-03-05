# -*- coding: utf-8 -*-
#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os

import rclpy
from rclpy.node import Node

global LoggingNode


def do_log(msg):
    LoggingNode.get_logger().info(msg)
    os.system(f'espeak "{msg}"')


class LogTester(Node):
    def __init__(self):
        global LoggingNode
        LoggingNode = self
        print(self)

        super().__init__('node_name')
        do_log('testing...')

    def test(self):
        do_log('more testing...')


def main(args=None):
    rclpy.init(args=args)

    LT = LogTester()
    LT.test()

    LT.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

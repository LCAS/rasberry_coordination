# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.agent_cb = self.create_subscription(
                            msg_type=String,
                            topic='~/input',
                            callback=self.callback,
                            qos_profile=10)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tmap = self.create_subscription(String, '/topological_map_2', self.cb, qos)

    def cb(self, msg):
        print(msg)

    def callback(self, msg):
        self.get_logger().info(msg.data)
        os.system('espeak "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    SP = Speaker()
    rclpy.spin(SP)

    SP.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

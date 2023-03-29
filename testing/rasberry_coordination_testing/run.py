# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from rasberry_coordination_msgs.msg import Configuration, AgentList

class Executor(Node):

    def __init__(self, config):
        super().__init__('executor')
        self.get_logger().info('Node Initialised')

        # Get current state of coordinator
        self.setup = Configuration()
        self.map, self.route_planner = None, None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.setup_sub = self.create_subscription(Configuration, '/coordinator/routing_management/configuration', self.save_setup, qos)

        # Track the current stage of each agent
        self.idleness = dict()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.setup_sub = self.create_subscription(AgentList, '/coordinator/agent_management/fleet_details', self.save_idleness, qos)

        # Modify the axctive map or the active route planner
        self.change_map_pub = self.create_publisher(String, '/topological_map_manager2/switch_topological_map', 1)
        self.change_route_planner_pub = self.create_publisher(String, '/coordinator/routing_management/change_route_planner', 1)


        # Set map and route planner if tests use the same for each
        self.get_logger().info('Setting Global Values')
        if 'global' in config:
            g = config['global']
            self.targets       = g['targets']       if 'targets'       in g else None
            self.map           = g['map']           if 'map'           in g else None
            self.route_planner = g['route_planner'] if 'route_planner' in g else None

        # Execute tests
        self.get_logger().info('Beginning Execution')
        for i, test in enumerate(config['tests']):
            self.get_logger().info('---')
            self.get_logger().info(f'Test {i} has begun')
            self.configure_test_environment(test)


    def save_setup(self, msg): self.setup = msg
    def save_idleness(self, msg):
        for agent in msg.list:
            self.idleness[agent.id] = 'rasberry_coordination_core.base.Idle' in agent.state.stage

    def configure_test_environment(self, test):
        # Identify setup
        self.targets       = test['targets']       if 'targets'       in test else self.targets
        self.map           = test['map']           if 'map'           in test else self.map
        self.route_planner = test['route_planner'] if 'route_planner' in test else self.route_planner

        self.get_logger().info(f'| Map: {self.map}')
        self.get_logger().info(f'| Planner: {self.route_planner}')
        self.get_logger().info(f'| Total Targets:')
        for k,v in self.targets.items():
            self.get_logger().info(f'|   {k} - {len(v)}')


        # Perform Setup
        self.get_logger().info(f'| Declaring Desired Setup')
        if 'map'           in test: self.change_map_pub.publish(String(data=self.map))
        if 'route_planner' in test: self.change_route_planner_pub.publish(String(data=self.route_planner))

        # Wait for coordintor to confirm active map and active route planner
        self.get_logger().info(f'| Waiting for Coordinator to Confirm')
        while self.setup.map != self.map and self.setup.route_planner != self.route_planner:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Begin tests
        self.get_logger().info(f'| Executing Test')
        self.execute()


    def execute(self):
        # Create publisher objects for each agent in the test
        self.get_logger().info(f'| Constructing Publishers')
        publishers = dict()
        for agent_id in self.targets.keys():
            publishers[agent_id] = {
                'move': self.create_publisher(String, f'/{agent_id}/localisation/disable', 1),
                'goal': self.create_publisher(String, f'/{agent_id}/navigation/move_idle', 1)
            }

        # Set their initial positions
        self.get_logger().info(f'| Setting Initial Positions')
        for agent_id in self.targets.keys():
            self.get_logger().info(f'|   {agent_id} - {self.targets[agent_id][0]}')
            publishers[agent_id]['move'].publish(String(data=self.targets[agent_id][0]))
            del self.targets[agent_id][0]
        rclpy.spin_once(self, timeout_sec=0.5)

        # Once the agent is marked as idle, give its next target
        self.get_logger().info(f'| Starting Goals')
        while True:
            for agent_id in self.targets.keys():
                if agent_id not in self.idleness: continue
                if self.idleness[agent_id] and self.targets[agent_id]:
                    self.get_logger().info(f'|   {agent_id} -> {self.targets[agent_id][0]}')
                    del self.idleness[agent_id]
                    publishers[agent_id]['goal'].publish(String(data=self.targets[agent_id][0]))
                    del self.targets[agent_id][0]
            rclpy.spin_once(self, timeout_sec=0.1)




def main(args=None):
    rclpy.init(args=args)

    config_file = sys.argv[1]
    with open(config_file) as f:
        config_data = yaml.safe_load(f)

    Ex = Executor(config_data)
    rclpy.spin(Ex)

    Ex.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

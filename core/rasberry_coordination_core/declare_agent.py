#!/usr/bin/env python3

import sys, os, time
import json, yaml
from pprint import pprint

import rclpy
from rclpy.node import Node

import tf_transformations #https://github.com/DLu/tf_transformations

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from diagnostic_msgs.msg import KeyValue

from rasberry_coordination_msgs.msg import NewAgentConfig, Module


def get_kvp_list(dict, item):
    if item in dict:
        return [KeyValue(k, str(v)) for k, v in dict[item].items()]
    return []


def get_file_path(setup):
    folder = os.getenv('AGENT_SETUP_CONFIG', None)
    setup_file = "%s%s.yaml"%(folder, setup)
    return setup_file


def load_agent_obj(node, agent_id, setup, printer=True):

    # Identify agent and setup filepaths
    setup_file = get_file_path(setup)

    # Load file contents, (fallback on empty file if agent_file not found)
    agent_data = {'agent_id': agent_id.split("/")[-1].split(".")[0]}
    if printer: node.get_logger().warn(f"Launching with agent_data: {agent_data}")


    # Build msg (use yaml.dump to parse further details through to coordinator)
    with open(setup_file) as f:
        setup_data = yaml.safe_load(f)

    pprint(setup_data)
    print("\n")

    agent = NewAgentConfig()
    agent.agent_id = agent_data['agent_id']
    agent.local_properties = get_kvp_list(agent_data, 'local_properties')

    for m in setup_data['modules']:
        m['details'] = m['details'] if 'details' in m else [{'key':'value'}]

    agent.modules = [Module(m['name'], m['interface'], [KeyValue(d.keys()[0], yaml.dump(d.values()[0])) for d in m['details']]) for m in setup_data['modules']]
    print("\n\n")
    return agent


class AgentMonitor(Node):
    def __init__(self):
        super().__init__('add_agent')
        self.pub = self.create_publisher(NewAgentConfig, "/rasberry_coordination/dynamic_fleet/add_agent", 5)
        self.s1 = self.create_subscription(String, "/car/new_agent", self.load_picker, 10)
        self.s2 = self.create_subscription(String, "/sar/new_agent", self.load_tall_controller, 10)
        self.s4 = self.create_subscription(String, "/car_client/get_gps", self.add_car_agent, 10)

    """ Dynamic Fleet """
    def add_car_agent(self, msg):
        details = json.loads(msg.data)
        id = str(details['user'])
        if 'STD_v2' in id:
            self.load(String(id), 'picker', printer=False)

    def load_picker(self, msg): self.load(self, msg, 'picker')
    def load_tall_controller(self, msg): self.load(self, msg, 'tall_controller')
    def load(self, msg, agent_type, printer=True):
        self.get_logger().info(f"Recieved new {msg.data} information: {agent_type}")
        agent = load_agent_obj(agent_id=msg.data, setup=agent_type, printer=printer)
        if printer: print(agent)
        self.pub.publish(agent)

    def spin(self):
        rclpy.spin(self)

class AgentPublisher(Node):

    def __init__(self, agent_id, setup):
        print('\n')
        self.get_logger().info(f"Loading configurations:")
        self.get_logger().info(f"    - agent_file: {agent_id}")
        self.get_logger().info(f"    - setup_file: {setup}")

        # Generate agent details
        self.agent = load_agent_obj(agent_id, setup)
        self.get_logger().info(f"Details of Agent being launched:\n{agemt}\n\n")

        # Create publisher
        self.details_pub = self.create_publisher(NewAgentConfig, "/rasberry_coordination/dynamic_fleet/add_agent", 5)

        # Create a storage point for the robots pose to be saved on shutdown
        self.pose = Pose()
        self.pose_sub = self.create_subscription("/robot_pose", Pose, self.pose_cb)

    def spin(self):
        while self.ok():
            pub.publish(self.agent)
            logmsg(category="null", msg="publishing")
            time.sleep(5)
        self.save()

    def pose_cb(msg):
        self.pose = msg

    def save(msg):
        #On shutdown, save the robots last known location to a file for loading at boot
        filepath = os.getenv('ROBOT_LAST_LOCATION_FOLDER', '')+"%s.sh"%(self.agent.agent_id)
        with open(filepath, 'w') as f:
            o = self.pose.orientation
            rot = tf_transformations.euler_from_quaternion([o.x, o.y, o.w, o.z])[2]
            f.write('export_override ROBOT_POS_A %s\n' % rot)
            f.write('export_override ROBOT_POS_X %s\n' % self.pose.position.x)
            f.write('export_override ROBOT_POS_Y %s\n' % self.pose.position.y)



def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 5:
        manager = AgentMonitor()
        manager.get_logger().info("AddAgent Monitor launched")

    else:
        agent_id = sys.argv[1]
        setup = sys.argv[2]
        manager = AgentPublisher(agent_id, setup)
        manager.get_logger().info("AddAgent Publisher launched")

    manager.spin()

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


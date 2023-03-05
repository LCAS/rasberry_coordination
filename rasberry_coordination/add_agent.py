#!/usr/bin/env python3

import sys, os, time
import json, yaml
from pprint import pprint

import rclpy
import tf_transformations #https://github.com/DLu/tf_transformations

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from diagnostic_msgs.msg import KeyValue

from rasberry_coordination.msg import NewAgentConfig, Module

from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak


def get_kvp_list(dict, item):
    if item in dict:
        return [KeyValue(k, str(v)) for k, v in dict[item].items()]
    return []


def get_file_path(setup):
    folder = os.getenv('AGENT_SETUP_CONFIG', None)
    setup_file = "%s%s.yaml"%(folder, setup)
    return setup_file


def load_agent_obj(agent_id, setup, printer=True):

    # Identify agent and setup filepaths
    setup_file = get_file_path(setup)

    # Load file contents, (fallback on empty file if agent_file not found)
    agent_data = {'agent_id': agent_id.split("/")[-1].split(".")[0]}
    if printer: logmsg(level="warn", category="DRM", msg="Launching with agent_data: %s" % (agent_data))


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
        self.pub = rclpy.create_publisher(NewAgentConfig, "/rasberry_coordination/dynamic_fleet/add_agent", latch=True, queue_size=5)
        self.s1 = rclpy.create_subscriber(String, "/car/new_agent", self.load,  callback_args='picker')
        self.s2 = rclpy.create_subscriber(String, "/sar/new_agent", self.load,  callback_args='tall_controller')
        self.s3 = rclpy.create_subscriber(String, "/car/new_store", self.load,  callback_args='storage')
        self.s4 = rclpy.create_subscriber(String, "/car_client/get_gps", self.add_car_agent)

    """ Dynamic Fleet """
    def add_car_agent(self, msg):
        details = json.loads(msg.data)
        id = str(details['user'])
        if 'STD_v2' in id:
            self.load(String(id), 'picker', printer=False)

    def load(self, msg, agent_type, printer=True):
        logmsg(category="DRM", msg="Recieved new %s information: %s"%(msg.data, agent_type))
        agent = load_agent_obj(agent_id=msg.data, setup=agent_type, printer=printer)
        if printer: print(agent)
        self.pub.publish(agent)

    def spin(self):
        rclpy.spin(self)

class AgentPublisher(Node):

    def __init__(self, agent_id, setup):
        logmsgbreak()
        logmsg(category="DRM", msg="Loading configurations:")
        logmsg(category="DRM", msg="    - agent_file: %s"%agent_id)
        logmsg(category="DRM", msg="    - setup_file: %s"%setup)

        # Generate agent details
        self.agent = load_agent_obj(agent_id, setup)
        logmsg(category="DRM", msg="Details of Agent being launched:\n%s\n\n"%agent)

        # Create publisher
        self.details_pub = rclpy.create_publisher(NewAgentConfig, "/rasberry_coordination/dynamic_fleet/add_agent", latch=False, queue_size=5)

        # Create a storage point for the robots pose to be saved on shutdown
        self.pose = Pose()
        self.pose_sub = rclpy.create_subscriber("/robot_pose", Pose, self.pose_cb)

    def spin(self):
        while not rospy.is_shutdown():
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



def main():
    rclpy.init()

    if len(sys.argv) < 5:
        logmsg(category="DRM", msg="AddAgent Monitor launched")
        manager = AgentMonitor()

    else:
        logmsg(category="DRM", msg="AddAgent Publisher launched")
        agent_id = sys.argv[1]
        setup = sys.argv[2]
        manager = AgentPublisher(agent_id, setup)

    manager.spin()
    manager.destroy_node()


if __name__ == '__main__':
    main()

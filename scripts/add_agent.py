#!/usr/bin/env python

import sys
import rospy
import json
from std_msgs.msg import String
import rasberry_coordination
from rasberry_coordination.msg import NewAgentConfig, Module, KeyValuePair
import rasberry_des.config_utils
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak


def get_kvp_list(dict, item):
    if item in dict:
        return [KeyValuePair(k, str(v)) for k, v in dict[item].items()]
    return []


import rospkg
def get_file_path(agent, setup):
    rc=rospkg.RosPack().get_path('rasberry_coordination')
    agent_file = "%s/config/agent/%s.yaml"%(rc, agent)
    setup_file = "%s/config/setup/%s.yaml"%(rc, setup)
    return agent_file, setup_file


def load_agent_obj(agent_input, setup_input, get_files_from_paths=False, printer=True):

    # Identify agent and setup filepaths
    if get_files_from_paths:
        agent_file, setup_file = get_file_path(agent_input, setup_input)
    else:
        agent_file, setup_file = agent_input, setup_input

    # Load file contents, (fallback on empty file if agent_file not found)
    try:
        agent_data = rasberry_des.config_utils.get_config_data(agent_file)
    except Exception as e:
        if printer: print(e)
        if printer: logmsg(level="warn", category="DRM", msg="File not Loaded: %s" % (agent_file))
        agent_data = {'agent_id': agent_input.split("/")[-1].split(".")[0]}
        if printer: logmsg(level="warn", category="DRM", msg="Launching with agent_data: %s" % (agent_data))
    setup_data = rasberry_des.config_utils.get_config_data(setup_file)

    # Build msg
    agent = NewAgentConfig()
    agent.agent_id = agent_data['agent_id']
    agent.local_properties = get_kvp_list(agent_data, 'local_properties')
    agent.setup.modules = [Module(m['name'], m['role']) for m in setup_data['modules']]
    agent.setup.module_properties = get_kvp_list(setup_data, 'module_properties')
    agent.setup.navigation_properties = get_kvp_list(setup_data, 'navigation_properties')
    agent.setup.visualisation_properties = get_kvp_list(setup_data, 'visualisation_properties')
    return agent


class AgentMonitor():
    def __init__(self):
        self.pub = rospy.Publisher("/rasberry_coordination/dynamic_fleet/add_agent", NewAgentConfig, latch=True, queue_size=5)
        self.s1 = rospy.Subscriber("/car/new_agent", String, self.load,  callback_args='picker')
        self.s2 = rospy.Subscriber("/sar/new_agent", String, self.load,  callback_args='tall_controller')
        self.s3 = rospy.Subscriber("/car/new_store", String, self.load,  callback_args='field_storage')
        self.s4 = rospy.Subscriber('/car_client/get_gps', String, self.add_car_agent)

    """ Dynamic Fleet """
    def add_car_agent(self, msg):
        details = json.loads(msg.data)
        id = str(details['user'])
        if 'STD_v2' in id:
            self.load(String(id), 'picker', printer=False)

    def load(self, msg, agent_type, printer=True):
        logmsg(category="DRM", msg="Recieved new %s information: %s"%(msg.data, agent_type))
        agent = load_agent_obj(agent_input=msg.data, setup_input=agent_type, get_files_from_paths=True, printer=printer)
        if printer: print(agent)
        self.pub.publish(agent)


if __name__ == '__main__':
    # Initialise node
    rospy.init_node("AddAgent", anonymous=True)
    rospy.sleep(1)

    if len(sys.argv) < 5:
        logmsg(category="DRM", msg="AddAgent Monitor launched")
        monitor = AgentMonitor()
        rospy.spin()

    else:
        logmsg(category="DRM", msg="AddAgent Node launched")

        # Collect details
        agent_file = sys.argv[1]
        setup_file = sys.argv[2]

        logmsgbreak()
        logmsg(category="DRM", msg="Loading configurations:")
        logmsg(category="DRM", msg="    - agent_file: %s"%agent_file)
        logmsg(category="DRM", msg="    - setup_file: %s"%setup_file)

        agent = load_agent_obj(agent_file, setup_file)
        logmsg(category="DRM", msg="Details of Agent being launched:\n%s\n\n"%agent)

        # Create publisher
        pub = rospy.Publisher("/rasberry_coordination/dynamic_fleet/add_agent", NewAgentConfig, latch=False, queue_size=5)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            pub.publish(agent)
            logmsg(category="null", msg="publishing")
            rospy.sleep(5)

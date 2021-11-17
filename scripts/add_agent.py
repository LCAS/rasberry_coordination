#!/usr/bin/env python

import sys
import rospy
import rasberry_coordination
from rasberry_coordination.msg import AgentDetails, Module, KeyValuePair
import rasberry_des.config_utils


if __name__ == '__main__':

    # Initialise node
    rospy.init_node("AddAgent", anonymous=True)
    rospy.sleep(1)

    # Collect details
    agent_file = sys.argv[1]
    setup_file = sys.argv[2]

    print("\nLoading configurations:\n%s\n%s\n" % (agent_file, setup_file))

    # Load config file
    agent_data = rasberry_des.config_utils.get_config_data(agent_file)
    setup_data = rasberry_des.config_utils.get_config_data(setup_file)

    # Check item in dict for KVP list generation
    def get_kvp_list(dict, item):
        if item in dict:
            return [KeyValuePair(k, str(v)) for k, v in dict[item].items()]
        return []

    # Build msg
    agent = AgentDetails()
    agent.agent_id = agent_data['agent_id']
    agent.local_properties = get_kvp_list(agent_data, 'local_properties')
    agent.setup.modules = [Module(m['name'], m['role']) for m in setup_data['modules']]
    agent.setup.module_properties =        get_kvp_list(setup_data, 'module_properties')
    agent.setup.navigation_properties =    get_kvp_list(setup_data, 'navigation_properties')
    agent.setup.visualisation_properties = get_kvp_list(setup_data, 'visualisation_properties')

    print("Details of agent being launched:\n%s\n\n"%agent)

    # Create publisher
    # pub = rospy.Publisher("/a", AgentDetails, latch=True, queue_size=5)
    pub = rospy.Publisher("/rasberry_coordination/dynamic_fleet/add_agent", AgentDetails, latch=True, queue_size=5)
    # pub = rospy.Publisher("launch_agent", AgentDetails, latch=True, queue_size=5)
    rospy.sleep(1)

    pub.publish(agent)
    rospy.sleep(1)

    rospy.spin()
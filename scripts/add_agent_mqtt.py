#!/usr/bin/env python

import tf
import sys
import rospy
import os
import json, yaml
from std_msgs.msg import String
import rasberry_coordination
from rasberry_coordination.msg import NewAgentConfig, Module
from diagnostic_msgs.msg import KeyValue
import rasberry_des.config_utils
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak
from pprint import pprint
import rospkg
from geometry_msgs.msg import Pose


class AgentMonitor():
    def __init__(self):
        self.sub = rospy.Subscriber("/rasberry_coordination/dynamic_fleet/add_agent", NewAgentConfig, self.new_agent)

    def load(self, msg, agent_type, printer=True):
        logmsg(category="DRM", msg="Recieved new %s information: %s"%(msg.data, agent_type))
        agent = load_agent_obj(agent_id=msg.data, setup=agent_type, printer=printer)
        if printer: print(agent)
        self.pub.publish(agent)


if __name__ == '__main__':
    # Initialise node
    rospy.init_node("AddAgent", anonymous=True)
    rospy.sleep(1)

    logmsg(category="DRM", msg="AddAgent Monitor launched")
    monitor = AgentMonitor()
    rospy.spin()


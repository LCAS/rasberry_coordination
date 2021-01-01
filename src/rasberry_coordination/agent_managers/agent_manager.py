#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import actionlib
import rospy
import os

from rospy import Subscriber as Sub, Publisher as Pub, get_rostime as Now
from std_msgs.msg import String as Str
from rasberry_coordination.msg import KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg


class AgentManager(object):

    """Initialise class with callback details to apply to agents"""
    def __init__(self, callback_dict):
        self.cb = callback_dict
        self.agent_details = {}
    
    """Add AgentDetails objects"""
    def add_agents(self, agent_id_list):
        for agent_id in agent_id_list:
            self.add_agent(agent_id)

    """Add AgentDetails objects"""
    def remove_agent(self, agent_id):
        self.agent_details[agent_id]._remove()
        self.agent_details.pop(agent_id)

    """Item retrieval objects (potentially slow so don't use unnecessarily)""" #https://stackoverflow.com/questions/12798653/does-setattr-and-getattr-slow-down-the-speed-dramatically
    def get_list(self, list_id): #TODO: swap out to polymorphism
        return [getattr(deets, list_id) for deets in self.agent_details.values()]
    def get(self, agent_id, item):
        return getattr(self.agent_details[agent_id], item)
    def set(self, agent_id, item, value):
        setattr(self.agent_details[agent_id], item, value)

    """Dump Agent Details Callback"""
    def dump_details(self, agent_id="", filename=""):
        if hasattr(agent_id, 'data'):
            agent_id = agent_id.data

        if agent_id in self.agent_details:
            self.agent_details[agent_id]._dump(filename)
        else:
            for agent_id in self.agent_details:
                self.agent_details[agent_id]._dump()

"""Container for all functions related abstractly to the agent"""
class AgentDetails(object):

    """Initialise all fields"""
    def __init__(self, ID, cb):
        self.live_diagnostics_pub = Pub('/rasberry_coordination/agent_monitor/'+ID, KeyValuePair, latch=True, queue_size=5)

        """Callbacks"""
        self.cb = cb

        """Meta Management"""
        self.agent_id = ID

        """Localisation Details"""
        self.previous_node = None
        self.current_node = None
        self.closest_node = None
        self.current_node_sub = Sub(ID+"/current_node", Str, self._current_node_cb)
        self.closest_node_sub = Sub(ID+"/closest_node", Str, self._closest_node_cb)

    """Dump all values for agent into file"""
    def _dump(self, filename=""):

        if filename == "":
            filename='~/'+self.agent_id+'---'+str(Now())+'.txt'
        else:
            filename = '~/'+self.agent_id+'---'+filename+'.txt'

        with open(os.path.expanduser(filename), 'w') as writer:
            writer.write("%s\n\n" % (self.agent_id))

            attr_len = max([len(attr) for attr in dir(self)])

            for attr in dir(self):
                if attr in ['cb'] or attr.startswith('__') or attr.endswith('_cb') or attr.endswith('_sub'):
                    continue
                padding = (attr_len-len(attr)) * ' '
                writer.write("%s%s\t=\t%r\n" % (attr, padding, str(getattr(self, attr))))
        print('write complete -> '+filename)

    """Callback for current node of agent"""
    def _current_node_cb(self, msg):

        if self.current_node != None:
            self.previous_node = self.current_node

        self.current_node = msg.data
        if self.current_node == "none":
            self.current_node = None
        if self.previous_node == "none":
            self.previous_node = None

        if self.cb['update_topo_map']:
            self.cb['update_topo_map']()

    """Callback for closest node from agent"""
    def _closest_node_cb(self, msg):
        self.closest_node = msg.data
        if self.closest_node == "none":
            self.closest_node = None

    """return start node as current node, previous node or closest node"""
    def _get_start_node(self):
        if self.current_node:
            return self.current_node
        elif self.previous_node:
            return self.previous_node
        else:
            return self.closest_node

    """On shutdown"""
    def _remove(self):
        self.current_node_sub.unregister()
        self.closest_node_sub.unregister()

    """Monitoring"""
    def __setattr__(self, key, value):
        if hasattr(self, key):
            val = getattr(self, key)
        else:
            val = "tomato"
        super(AgentDetails, self).__setattr__(key, value)
        if val != "tomato" and val != value:
            self.update_live_diagnostics(key, value)

    def update_live_diagnostics(self, key, value):
        LD = KeyValuePair() #TODO, remove this and just format the object in the publisher
        if key not in ["task_id","task_stage","current_node","previous_node","closest_node"]:
            return

        LD.key = key
        LD.value = str(value)
        if hasattr(self, 'live_diagnostics_pub'):
            self.live_diagnostics_pub.publish(LD)
            # self.live_diagnostics_pub.publish({'key':key,'value':str(value)})

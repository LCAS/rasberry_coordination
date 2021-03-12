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
        """ Initialise management interface for agents.

        :param callback_dict: container for direct callbacks to coordinator for use in subscribers
        """
        self.cb = callback_dict
        self.agent_details = {}

    """Add AgentDetails objects"""
    def add_agents(self, agent_id_list):
        """ Initialise a given list of agents and add to the agent_details collection

        :param agent_id_list: list of agents to instantiate
        :return: None
        """
        for agent_id in agent_id_list:
            self.add_agent(agent_id)


    """Add AgentDetails objects"""
    def add_agent(self, agent_id):
        """" Initialise a given agent and add to the agent_details collection

        :param agent_id: identifier for the agent to initialise
        :return: None
        """
        self.agent_details[agent_id] = AgentDetails(agent_id, self.cb)
    def remove_agent(self, agent_id):
        """" Remove an agent from the agent_details collection.

        :param agent_id: identifier for the agent to remove
        :return: None
        """
        self.agent_details[agent_id]._remove()
        self.agent_details.pop(agent_id)

    """ Get Task Handler """
    def get_task_handler(self, task_id):
        """" Identify the agent handling the given task_id

        :param task_id: identifier to find the associated agent
        :return: Agent managing the given task_id
        """
        handlers = [A for A in self.agent_details.values() if A.task_id == task_id]
        return handlers[0] if handlers else None
    def __getitem__(self, item):
        """" Convience method for accessing agent_details object directly

        :param item: agent_id to find in self.agent_details
        :return: Agent associated to the given item
        """
        if item in self.agent_details:
            return self.agent_details[item]
        else:
            return None
    def get_all_task_handlers(self):
        """" Get list of all agents which have an active task
        :return: list of agents
        """
        return [A for A in self.agent_details.values() if A.task_id]

    """Item retrieval objects (slow so don't use unnecessarily)"""
    def get_list(self, parameter):
        """" Item retrieval using getattr(Agent, parameter)

        :param parameter: agent_id to find in self.agent_details
        :return: list of AgentDetails.parameter
        """
        return [getattr(A, parameter) for A in self.agent_details.values()]
    def get(self, agent_id, item):
        """" Single-item retrieval using getattr(Agent, parameter)

        :param agent_id: identifier for agent
        :param parameter: parameter to find value of
        :return: Agent.parameter
        """
        return getattr(self.agent_details[agent_id], item)
    def set(self, agent_id, parameter, value):
        """" Single-item setting using setattr(Agent, parameter, value)

        :param agent_id: identifier for agent
        :param parameter: parameter to set value of
        :param value: value to set as
        :return: None
        """
        setattr(self.agent_details[agent_id], parameter, value)

    """Dump Agent Details Callback"""
    def dump_details(self, agent_id="", filename=""):
        """" Dump properties for given agent or all agents.
        This method works as both a callback from "rasberry_coordination/X_manager/dump", and called directly.

        :param agent_id: identifier for agent (callback msg)
        :param filename: optional filename description to append to end of filename
        :return: None
        """
        if hasattr(agent_id, 'data'):
            agent_id = agent_id.data

        if agent_id in self.agent_details:
            self.agent_details[agent_id]._dump(filename)
        else:
            for agent_id in self.agent_details:
                self.agent_details[agent_id]._dump()

"""Container for all functions related abstractly to the agent"""
class AgentDetails(object):

    def __repr__(self):
        """ Print readable representation for AgentDetails object depending on agent_id

        :return: String representation of respective class.
        """
        if self.agent_id.startswith("picker"):
            return "Picker(\"%s\"|\"%s\"|\"%s\")" % (self.agent_id, self.current_node, self.task_stage)
        return "Robot(\"%s\"|\"%s\"|\"%s\")" % (self.agent_id, self.current_node, self.task_stage)

    """Initialise all fields"""
    def __init__(self, ID, cb):
        """ Class to define details and interactions for an individual agent.

        :param ID: Unique identifier for agent
        :param cb: Callbacks to direct response from location subscribers
        """
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
        """" Dump properties for agent, excluding __properties, and callback functions.

        :param filename: optional filename description to append to end of filename
        :return: None
        """
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
        """ Callback for "agent_id/current_node"

        If the current_node is not None, copy current_node to previous_node.
        Call callback to update_topo_map (roue_planner).

        :param msg: string containing the new current_node for the agent
        :return: None
        """
        if self.current_node is not None:
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
        """ Callback for "agent_id/closest_node"

        :param msg: string containing the new closest_node for the agent
        :return: None
        """
        self.closest_node = msg.data
        if self.closest_node == "none":
            self.closest_node = None

    """return start node as current node, previous node or closest node"""
    def _get_start_node(self, accuracy=None):  # When between nodes, the current node is None
        """ Identify the agents current location, or if not exists, the next most accurate.

        :param accuracy: bool defining whether to call from previous_node or closest_node if current_node is None
        :return: Waypoint identifier
        """
        if accuracy:
            return self.current_node or self.previous_node or self.closest_node
        else:
            return self.current_node or self.closest_node or self.previous_node

    """On shutdown"""
    def _remove(self):
        """ On removal of robot, unregister callbacks before deletion of AgentDetails object

        :return: None
        """
        self.current_node_sub.unregister()
        self.closest_node_sub.unregister()

    """Monitoring"""
    def __setattr__(self, key, value):
        """ On modification of attribute, if the new value is different, call to update the monitoring window.

        :param key: parameter to modify
        :param value: new value for parameter
        :return: None
        """
        val = None
        if hasattr(self, key):
            val = getattr(self, key)
        super(AgentDetails, self).__setattr__(key, value)
        if val != value:
            self.update_live_diagnostics(key, value)
    def update_live_diagnostics(self, key, value):
        """ Publish call to "/rasberry_coordination/agent_monitor/agent_id" update the monitoring window.
        Publish only if key is in locally defined list.

        :param key: parameter modified
        :param value: new value for parameter
        :return: None
        """

        #TODO: Set this up to read from parameter server
        if key not in ["task_id","task_stage","current_node",
                       "previous_node","closest_node","registered"]:
            return
        if hasattr(self, 'live_diagnostics_pub'): #TODO: find a way to remove this, set like this due to race
            self.live_diagnostics_pub.publish(KeyValuePair(key, str(value)))

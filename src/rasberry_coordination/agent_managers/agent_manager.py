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

    """ Get Task Handler """
    def get_task_handler(self, task_id):
        handlers = [A for A in self.agent_details.values() if A.task_id == task_id]
        return handlers[0] if handlers else None
    def __getitem__(self, item):
        if item in self.agent_details:
            return self.agent_details[item]
        else:
            return None
    def get_all_task_handlers(self):
        return [A for A in self.agent_details.values() if A.task_id]

    """Item retrieval objects (slow so don't use unnecessarily)"""
    def get_list(self, list_id):
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
    def __repr__(self):
        if self.agent_id.startswith("picker"):
            return "Picker(\"%s\"|\"%s\"|\"%s\")" % (self.agent_id, self.current_node, self.task_stage)
        return "Robot(\"%s\"|\"%s\"|\"%s\")" % (self.agent_id, self.current_node, self.task_stage)
    def __del__(self):
        for sub in self.subscribers.values():
            sub.unregister()

    """Debug Tools, for dumping data to file/monitoring node"""
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
    def __setattr__(self, key, value):
        val = None
        if hasattr(self, key):
            val = getattr(self, key)
        super(AgentDetails, self).__setattr__(key, value)
        if val != value:
            self.update_live_diagnostics(key, value)
    def update_live_diagnostics(self, key, value):
        if key not in ["task_id","task_stage","current_node",
                       "previous_node","closest_node","registered"]:
            return
        if hasattr(self, 'live_diagnostics_pub'): #TODO: probably a way to get rid of this
            self.live_diagnostics_pub.publish(KeyValuePair(key, str(value)))

    """Callback for current/closest node of agent"""
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
    def _closest_node_cb(self, msg):
        self.closest_node = msg.data
        if self.closest_node == "none":
            self.closest_node = None

    """return start node as current node, previous node or closest node"""
    def _get_start_node(self):  # When between nodes, the current node is None
        if self.current_node:
            return self.current_node
        elif self.previous_node:
            return self.previous_node
        else:
            return self.closest_node

    """Initialise all fields"""
    def __init__(self, ID, cb, agent_type):
        self.live_diagnostics_pub = Pub('/rasberry_coordination/agent_monitor/'+ID, KeyValuePair, latch=True, queue_size=5)

        """Callbacks"""
        self.cb = cb
        self.subscribers = {}

        """Meta Management"""
        self.agent_id = ID
        self.agent_type = agent_type
        self.registered = True

        """Task Details"""
        self.task_details = {}
        self.task_stage_list = []

        """Localisation Details"""
        self.previous_node = None
        self.current_node = None
        self.closest_node = None
        self.subscribers['current_node'] = Sub(ID+"/current_node", Str, self._current_node_cb)
        self.subscribers['closest_node'] = Sub(ID+"/closest_node", Str, self._closest_node_cb)




    """ Define a given task for the agent """
    def load_idle_task(self):
        switch_dict = {"picker":  TaskDef.idle_picker,
                       "courier": TaskDef.idle_courier,
                       "storage": TaskDef.idle_Storage}
        switch_dict[self.agent_type]()

    def new_task(A, type, details={}):
        switch_dict = {"transportation_request": TaskDef.transportation_request,
                       "transportation_courier": TaskDef.transportation_courier,
                       "transportation_storage": TaskDef.transportation_storage}
        switch_dict[type](A, details)

    """ Convenience function to return active stage and modify task_details """
    def __call__(A, index=0):
        return A.task_stage_list[index]
    def __getitem__(A, key):
        return A.task_details[key] if key in A.task_details else None
    def __setitem__(A, key, val):
        A.task_details[key] = val

    """ Set the flag for stage completion """
    def flag(A, flag):
        A['stage_complete_flag'] = flag

    """ How to respond to task completion """
    def end_stage(A):
        A.task_stage_list.pop(0)





    """ DUMMY METHODS """
    """ DUMMY METHODS """
    """ DUMMY METHODS """
    """ DUMMY METHODS """

    """ Placeholder methods to update task state """
    def picker_pressed_button_with_intent_to_call_courier(self, task_id_made_by_picker_manager):
        self['task_id'] = task_id_made_by_picker_manager
    def picker_pressed_button_with_intent_to_cancel_task(self):
        pass #TODO: add a Stage.cancelled()
    def picker_pressed_button_after_putting_tray_on_robot(self):
        self['picker_has_tray'] = False
    def storage_manager_pressed_button_after_taking_tray_from_robot(self):
        self['storage_has_tray'] = True

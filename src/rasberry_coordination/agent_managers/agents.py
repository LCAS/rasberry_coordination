#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from abc import ABCMeta, abstractmethod
from rospy import Subscriber, Publisher, Time
from std_msgs.msg import String as Str
from rasberry_coordination.msg import KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.robot import Robot as RobotInterface_Old
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef

""" Agent Details """
class AgentManager(object):
    __metaclass__ = ABCMeta  # @abstractmethod

    """ Initialisation """
    def __init__(self, callback_dict):
        self.agent_details = {}
        self.cb = callback_dict
    def add_agents(self, agent_list):
        for agent in agent_list:
            self.add_agent(agent)
    @abstractmethod
    def add_agent(self, agent):
        pass

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None


""" Agent Management """
class AgentDetails(object):
    __metaclass__ = ABCMeta #@abstractmethod
    """ Fields:
    - agent_id, agent_type
    - idle_task_definition, new_task_definition
    - interface
    - task_stage_list, task_details
    - (subs) current_node, closest_node, previous_node
    """

    """ Initialisations """
    @abstractmethod
    def __init__(self, agent_dict, callbacks):
        self.agent_id = agent_dict['agent_id']
        self.cb = callbacks

        #Task Defaults
        self.task_name = None
        self.task_details = {}
        self.task_stage_list = []
        self.idle_task_default = agent_dict['idle_task_default']
        self.new_task_default = agent_dict['new_task_default']
        self.total_tasks = 0

        #Location and Callbacks
        self.has_presence = True #used for routing
        self.subs = {}
        self.current_node = None
        self.previous_node = None
        self.closest_node = None
        if 'initial_location' in agent_dict:
            self.current_node = agent_dict['initial_location']
        self.subs['current_node'] = Subscriber('/%s/current_node'%(self.agent_id), Str, self.current_node_cb)
        self.subs['closest_node'] = Subscriber('/%s/closest_node'%(self.agent_id), Str, self.closest_node_cb)

    def start_idle_task(self, task="default", details={}):
        task = task if task in self.idle_task_definition else self.idle_task_default
        self.idle_task_definition[task](self, details)
    def start_new_task(self, task="default", details={}, task_id=None):
        task = task if task in self.new_task_definition else self.new_task_default
        self.new_task_definition[task](self, details, task_id)

    """ Localisation """
    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data
        if self.cb['update_topo_map']:
            self.cb['update_topo_map']()
        # if self.current_node:
            # logmsg(msg="Agent: %s now at node: %s" % (self.agent_id,self.current_node))

    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data
    def location(self, accurate=False):
        # print("%s |  Current Node: %s" % (self.agent_id, self.current_node))
        # print("%s |  Closest Node: %s" % (self.agent_id, self.closest_node))
        # print("%s | Previous Node: %s" % (self.agent_id, self.previous_node))

        if accurate:
            return self.current_node or self.previous_node or self.closest_node
        return self.current_node or self.closest_node or self.previous_node
    def goal(self):
        return self().target

    """ Conveniences """
    def __call__(A, index=0):
        if len(A.task_stage_list) > index:
            return A.task_stage_list[index]
        else:
            return None
    def __getitem__(A, key):
        return A.task_details[key] if key in A.task_details else None
    def __setitem__(A, key, val):
        A.task_details[key] = val

    """ Standard Task Interactions """
    def start_stage(self):
        self()._start()
        self()._notify_start()
        self().new_stage = False
    def notify(self, state):
        self.interface.publish(state)
    def flag(self, flag):
        self().stage_complete = flag
    def end_stage(self):
        self()._notify_end()
        logmsg(category="stage", id=self.agent_id, msg="Stage %s is over" % self.task_stage_list[0])
        self.task_stage_list.pop(0)

    """ Logging """
    def __repr__(self):
        return self.get_class()
    def get_class(self):
        return str(self.__class__).replace("<class 'rasberry_coordination.agent_managers.store_manager.", "").replace("'>", "")

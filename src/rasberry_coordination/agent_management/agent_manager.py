#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

# from abc import ABCMeta, abstractmethod
from rospy import Subscriber, Publisher, Time
from std_msgs.msg import String as Str
from rasberry_coordination.msg import KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef


""" Agent Details """
class AgentManager(object):

    """ Initialisation """
    def __init__(self, callback_dict):
        self.agent_details = {}
        self.cb = callback_dict
    def add_agents(self, agent_list):
        for agent in agent_list:
            self.add_agent(agent)
    def add_agent(self, agent_dict):
        self.agent_details[agent_dict['agent_id']] = AgentDetails(agent_dict, self.cb)
        self.agent_details[agent_dict['agent_id']].int = len(self.agent_details)

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None



# class PickerAgent(AgentDetails):                  self.tags = {'type':'picker'}
# class StorageDetails(AgentDetails):               self.tags = {'type':'storage'}
# class Courier_RoboticAgent(AgentDetails):         self.tags = {'type':'robot'}
# class UV_RoboticAgent(AgentDetails):              pass
# class DataCollection_RoboticAgent(AgentDetails):  pass


""" Agent Management """
class AgentDetails(object):
    """ Fields:
    - agent_id, agent_type
    - idle_task_definition, new_task_definition
    - interface
    - task_stage_list, task_details
    - (subs) current_node, closest_node, previous_node
    """

    """ Initialisations """
    def __init__(self, agent_dict, callbacks):
        self.agent_id = agent_dict['agent_id']
        self.int = -1
        self.cb = callbacks

        #Task Defaults
        self.task_name = None
        self.task_details = {}
        self.task_contacts = {}
        self.task_stage_list = []
        self.task_buffer = []
        self.total_tasks = 0
        self.interruption = None
        self.properties = agent_dict['setup']['properties']

        # Define interface for each role given #TODO: what about differentiating between Device and App?
        self.roles = []
        self.interfaces = dict()
        for task in agent_dict['setup']['tasks']:
            self.roles += [task['role']]
            interface_name = '%s_%s' % (task['module'], task['role'])
            definition = getattr(InterfaceDef, interface_name)
            self.interfaces[task['module']] = definition(agent=self)

        # Define Default Tasks
        setup = agent_dict['setup']
        if 'idle_task_default' in setup and hasattr(TaskDef, setup['idle_task_default']):
            self.default_idle_task = setup['idle_task_default'] #TODO: remove & define idle task as $module_$role_idle
            # self.default_idle_task_definition = getattr(TaskDef, setup['idle_task_default'])
        # if 'new_task_default' in setup and hasattr(TaskDef, setup['new_task_default']):
        #     self.default_new_task_definition = getattr(TaskDef, setup['new_task_default'])

        #Location and Callbacks
        self.has_presence = True if self.properties['presence'] == 1 else False #used for routing
        self.subs = {}
        self.current_node = None
        self.previous_node = None
        self.closest_node = None
        if 'initial_location' in agent_dict:
            self.current_node = agent_dict['initial_location']
        self.subs['current_node'] = Subscriber('/%s/current_node'%(self.agent_id), Str, self.current_node_cb)
        self.subs['closest_node'] = Subscriber('/%s/closest_node'%(self.agent_id), Str, self.closest_node_cb)


    """ Task Starters """
    # def start_idle_task(self, task=None, details={}):
    #     if not task:
    #         self.default_idle_task_definition(self, details)
    #     else:
    #         getattr(TaskDef, task)(self, details)
    def add_idle_task(self):
        self.add_task(self.default_idle_task)

    def add_task(self, task_name, task_id=None, task_stage_list=[], details={}, contacts={}, index=None):
        """ Called by task stages, used to buffer new tasks for the agent """
        if task_name not in dir(TaskDef): return

        #picker.interface.called(): self.agent.add_task('transportation_request_courier')
        1. #find TaskDef
        2. #task = TaskDef()
        3. #buffer += [task] OR buffer.insert(0, [task])

        task_def = getattr(TaskDef, task_name)
        task = task_def(self, task_id=task_id, details=details, contacts=contacts)
        if not index: self.task_buffer += [task]
        else: self.task_buffer.insert(0, [task])

        logmsg(category="TASK", id=self.agent_id, msg="Buffering %s, task stage list:" % task['name'])
        [logmsg(category="TASK", msg='    - %s'%t) for t in task['stage_list']]

    def start_next_task(self, idx=0):
        if len(self.task_buffer) < 1: self.add_idle_task()
        if len(self.task_buffer) <= idx: return

        task = self.task_buffer.pop(idx)
        TaskDef.load_task(self, task)

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
        # TODO: this is not very good here, the state of an agent being interrupted should be handled abstractly
        if not self.interruption:
            self().stage_complete = flag
    def end_stage(self):
        logmsg(category="stage", id=self.agent_id, msg="Stage %s is over" % self.task_stage_list[0])
        self()._notify_end()
        self()._end()
        self.task_stage_list.pop(0)

    """ Logging """
    def __repr__(self):
        return "%s(%s)" % (self.get_class(), self.agent_id)
    def get_class(self):
        return str(self.__class__).replace("<class 'rasberry_coordination.agent_management.agent_manager.", "").replace("'>", "")

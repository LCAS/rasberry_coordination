#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import copy
from copy import deepcopy
import os
from pprint import pprint
import whiptail
import yaml

from rospy import Subscriber, Publisher, Service, Time

from std_msgs.msg import String as Str, Empty
from std_srvs.srv import Trigger, TriggerResponse

# from diagnostis_msgs.msg import KeyValue
from rasberry_coordination.msg import AgentDetails as AgentDetailsMsg, MarkerDetails, KeyValuePair
from rasberry_coordination.srv import AddAgent, AgentNodePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location, ModuleObj as Module, MapObj as Map
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef

import rasberry_des.config_utils
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch


class AgentManager(object):

    """ Initialisation """
    def __init__(self, callback_dict):
        self.agent_details = {}
        self.new_agent_buffer = dict()

        self.cb = callback_dict
        self.cb['format_agent_marker'] = self.format_agent_marker

        # Setup Connection for Dynamic Fleet
        file_name = 'coordinator-loaded-agents-save-state.yaml'  #logs to $HOME/.ros/coordinator-loaded-agents-save-state.yaml
        if os.path.isfile(file_name):
            if whiptail.Whiptail(title="Agent Management").confirm("Save-state detected, would you like to load it?"):
                with open(file_name) as file:
                    agent_dict = yaml.load(file, Loader=yaml.FullLoader)
                    pprint(agent_dict)
                    self.add_agents(agent_dict)
        self.s = Subscriber('/rasberry_coordination/dynamic_fleet/add_agent', AgentDetailsMsg, self.add_agent_cb)

        self.set_marker_pub = Publisher('/rasberry_coordination/set_marker', MarkerDetails, queue_size=5)
        self.get_markers_sub = Subscriber('/rasberry_coordination/get_markers', Empty, self.get_markers_cb)


    """ Dynamic Fleet """
    def add_agent(self, agent_dict):
        if agent_dict['agent_id'] not in self.agent_details:
            self.new_agent_buffer[agent_dict['agent_id']] = agent_dict
            pub2 = Publisher('/car_client/info/robots', Str, queue_size=1, latch=True)  #TODO: this should not be included here
            pub2.publish(self.simplify())
    def add_agents(self, agent_list):
        for agent in agent_list: self.add_agent(agent)
    def add_agent_from_buffer(self):
        buffer, self.new_agent_buffer = self.new_agent_buffer, dict()
        for agent_dict in buffer.values():
            self.agent_details[agent_dict['agent_id']] = AgentDetails(agent_dict, self.cb)
            self.format_agent_marker(self.agent_details[agent_dict['agent_id']], style='red')
            logmsg(category="null")
    def add_agent_cb(self, msg):
        def kvp_list(msg): return {kvp.key: kvp.value for kvp in msg}
        self.add_agent({'agent_id': msg.agent_id,
                        'local_properties': kvp_list(msg.local_properties),
                        'setup': { 'modules': [{'name':m.name,'role':m.role} for m in msg.setup.modules],
                                   'module_properties': kvp_list(msg.setup.module_properties),
                                   'navigation_properties': kvp_list(msg.setup.navigation_properties),
                                   'visualisation_properties': kvp_list(msg.setup.visualisation_properties)
                                   }})


    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None


    """ Visuals """
    def format_agent_marker(self, agent, style):
        """ Add/modify marker to display in rviz """
        """
           Add Marker: call self.format_agent_marker(agent_id, "")
        Modify Marker: call self.format_agent_marker(agent_id, "red")
        Remove Marker: call self.format_agent_marker(agent_id, "remove")
        """
        marker = MarkerDetails()
        marker.agent_id = agent.agent_id
        marker.type = agent.visualisation_properties['rviz_model']

        #Define marker color ["remove", "red", "green", "blue", "black", "white", ""]
        style = style or agent.visualisation_properties['default_colour']
        marker.optional_color = style

        logmsg(category="rviz", msg="Setting %s %s(%s)" % (marker.type, marker.agent_id, marker.optional_color))

        agent.visualisation_properties['marker'] = marker
        self.set_marker_pub.publish(marker)
    def get_markers_cb(self, empty):
        """ Request from RViz to resend all markers """
        for a in self.agent_details.values():
            if 'marker' in a.visualisation_properties:
                m = a.visualisation_properties['marker']
                self.set_marker_pub.publish(m)

    """ Data """
    def simplify(self):
        # return Str(str({'short': {'logistics':      ['thorvald_014']},
        #                 'tall':  {'uv_treatment':   ['thorvald_002_tall', 'thorvald_030'],
        #                           'data_gathering': ['thorvald_002_tall']}}))

        out = {}
        for a in self.agent_details.values():
            if 'structure_type' not in a.visualisation_properties: continue
            st = a.visualisation_properties['structure_type']
            if st not in out: out[st] = {}
            for t in a.modules:
                if t in ['base', 'health_monitoring']: continue
                if t not in out[st]: out[st][t] = []
                out[st][t] += [a.agent_id]
        return Str(str(out))


class AgentDetails(object):
    """ Fields:
    """

    """ Initialisations """
    def __init__(self, agent_dict, callbacks):
        self.agent_dict = agent_dict

        self.agent_id = agent_dict['agent_id']
        self.local_properties = agent_dict['local_properties']

        self.cb = callbacks

        setup = deepcopy(agent_dict['setup'])
        self.module_properties = setup['module_properties']
        self.navigation_properties = setup['navigation_properties']
        self.visualisation_properties = setup['visualisation_properties']

        lp = agent_dict['local_properties']
        mp = setup['module_properties']
        np = setup['navigation_properties']
        vp = setup['visualisation_properties']

        self.visualisation_properties['default_colour'] = lp['rviz_default_colour'] if 'rviz_default_colour' in lp else ''

        #Subscriptions
        self.subs = {}

        #Navigation Defaults
        self.navigation = dict()

        #Task Defaults
        self.total_tasks = 0
        self.task = Task()
        self.task_buffer = []

        self.interruption = None
        self.registration = False

        # Define interface for each role
        logmsg(category="MODULE", id=self.agent_id, msg="Initialising Module Interfaces:")
        self.modules = {t['name']: Module(agent=self, name=t['name'], role=t['role']) for t in setup['modules']}

        #Location and Callbacks
        initial_location = lp['initial_location'] if 'initial_location' in lp else ''
        has_presence = True if 'has_presence' in np and np['has_presence'] == 'True' else False
        self.location = Location(has_presence=has_presence, initial_location=initial_location)

        #Map
        topic = "/%s/restricted_topological_map_2" % self.agent_id if 'restrictions' in np else None
        self.map_handler = Map(agent=self, topic=topic)


    """ Task Starters """
    def add_idle_tasks(self):
        [self.add_task(task_name=m.idle_task_name) for m in self.modules.values() if m.name != "base"]
        if 'base' in self.modules: self.add_task(task_name=self.modules['base'].idle_task_name)

        if not self.task_buffer:
            logmsg(level="error", category="task", id=self.agent_id, msg="WARNING! Agent has no idle tasks.")
            logmsg(level="error", category="task", msg="    | All agents must be assigned a module.")
            logmsg(level="error", category="task", msg="    | Should each module have an idle task for each agent?")
            logmsg(level="error", category="task", msg="    \ Agent to be assigned a basic idle task.")
            self.add_task(task_name='idle')
    def add_task(self, task_name, task_id=None, task_stage_list=None, details=None, contacts=None, index=None, quiet=False, initiator_id=""):
        """ Called by task stages, used to buffer new tasks for the agent """
        task_stage_list = task_stage_list if task_stage_list else []
        details = details if details else {}
        contacts = contacts if contacts else {}
        # #picker.interface.called(): self.agent.add_task('transportation_request_courier')
        # 1. #find TaskDef
        # 2. #task = TaskDef()
        # 3. #buffer += [task] OR buffer.insert(0, [task])

        # logmsg(category="TASK", id=self.agent_id, msg="Attempting to add task: %s" % task_name)
        # logmsg(category="TASK", msg="    | Adding task: %s" % task_name)

        if task_name not in dir(TaskDef):
            logmsg(category="TASK", msg="    | %s (not found)"%task_name)
            return

        task_def = getattr(TaskDef, task_name)
        task = task_def(self, task_id=task_id, details=details, contacts=contacts, initiator_id=initiator_id)

        if not task:
            logmsg(category="TASK", msg="    | %s (empty)" % task_name)
            return

        task_name = task_name if task.name == task_name else "%s/%s"%(task_name,task.name)

        if not index: self.task_buffer += [task]
        else: self.task_buffer.insert(index, [task])

        if quiet:
            logmsg(category="DTM", msg="    |    | buffering %s to task_buffer[%i]" % (task_name, index or len(self.task_buffer)))
        else:
            logmsg(category="TASK",  msg="    | Buffering %s to task_buffer[%i]:" % (task_name, index or len(self.task_buffer)))
            [logmsg(category="TASK", msg="    |    | %s"%t) for t in task.stage_list]
    def start_next_task(self, idx=0):
        logmsg(category="TASK", id=self.agent_id, msg="Beginning next task", speech=False)

        if len(self.task_buffer) < 1: self.add_idle_tasks()
        if len(self.task_buffer) <= idx: return
        self.task = self.task_buffer.pop(idx)

        logmsg(category="TASK",  msg="- Active task: %s" % self['name'], speech=False)
        logmsg(category="TASK",  msg="  Task details:")
        [logmsg(category="TASK", msg="      | %s" % stage) for stage in self['stage_list']]
    def extend_task(self, task_name, task_id, details):
        task_def = getattr(TaskDef, task_name)
        task = task_def(self, details=details, task_id=task_id)
        self['stage_list'] += task.stage_list


    """ Roles """
    def roles(self):
        return [m.role for m in self.modules.values()]


    """ Navigation """
    def goal(self): return self().target


    """ Conveniences for Active Task """
    def __call__(A, index=0): return A.task.stage_list[index] if len(A.task.stage_list) > index else None
    def __getitem__(A, key):  return A.task[key] if A.task else None
    def __setitem__(A, key, val):    A.task[key] = val
    def simple_agent_id(self):
        return self.agent_id.replace('thorvald','T').replace('picker','P').replace('storage','S')


    """ Standard Task Interactions """
    def start_stage(self):
        self()._start()
        self().new_stage = False
    def end_stage(self):
        from time import sleep; sleep(0.2) #TODO: this can be removed once add task to buffer is removed from async
        logmsg(category="stage", id=self.agent_id, msg="Stage %s is over" % self['stage_list'][0])
        self()._end()
        self['stage_list'].pop(0)
        return None if self['stage_list'] else self['id']


    """ Task Interruption """
    def set_interrupt(self, type, module, task_id, scope, quiet=False):
        self.interruption = (type, module, task_id, scope)
        if quiet:
            logmsg(category="DTM", msg="    - interrupt attached to %s of type: (%s,%s,%s)." % (self.agent_id, type, module, task_id))
        else:
            logmsg(category="DTM", msg="Interrupt attached to %s of type: (%s,%s,%s)." % (self.agent_id, type, module, task_id))


    """ Logging """
    def __repr__(self):
        return "%s(%s)" % (self.get_class(), self.agent_id)
    def get_class(self):
        return str(self.__class__).replace("<class 'rasberry_coordination.agent_management.agent_manager.", "").replace("'>", "")

    """ GC """
    def delete_known_references(self, coordinator):
        import gc

        for m in self.modules.values():
            m.agent = None
            m.interface.agent = None

        coordinator.agent_manager.agent_details.pop(self.agent_id)  # Remove from agent manager
        coordinator.route_finder.planner.agent_details.pop(self.agent_id)  # Remove from route planner
        self.map_handler.agent = None
        coordinator.AllAgentsList.pop(self.agent_id)  # Remove from coordinator

        gc.collect()

        import weakref ; a = weakref.ref(self) ; del self
        logmsg(category="DRM", msg="References remaining: (%s/1)" % len(gc.get_referrers(a())))
        if len(gc.get_referrers(a())) > 1:
            logmsg(level="error", category="DRM", msg="Remove unhandled refs in place or in AgentDetails.delete_known_references()")
        [logmsg(category="DRM", msg="    - (%s) %s" % (i+1, r)) for i, r in enumerate(gc.get_referrers(a()))]
    def __del__(self):
        logmsg(level="warn", category="DRM", id=self.agent_id, msg="Agent handler is deleted.")
        logmsg(level="warn", category="DRM", msg="Here, we must identify and unregister every subscriber")



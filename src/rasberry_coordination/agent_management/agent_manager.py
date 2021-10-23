#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import copy
from pprint import pprint
import yaml

from rospy import Subscriber, Publisher, Service, Time

from std_msgs.msg import String as Str
from std_srvs.srv import Trigger, TriggerResponse

from rasberry_coordination.msg import MarkerDetails, KeyValuePair
from rasberry_coordination.srv import AddAgent, AgentNodePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef, PropertiesDef

from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch


class AgentManager(object):

    """ Initialisation """
    def __init__(self, callback_dict):
        self.agent_details = {}
        self.cb = callback_dict
        self.cb['format_agent_marker'] = self.format_agent_marker
        self.pause_all = False

        # Setup Services for Dynamic Fleet Management
        Service('/rasberry_coordination/dfm/add_agent',    AddAgent, self.add_agent_ros_srv)
        Service('/rasberry_coordination/dfm/remove_agent', AgentNodePair, self.remove_agent_ros_srv)
        Service('/rasberry_coordination/reenable_localisation', Trigger, self.reset_location_srv)

        self.set_marker_pub = Publisher('/rasberry_coordination/set_marker', MarkerDetails, queue_size=5)

    def add_agents(self, agent_list):
        for agent in agent_list: self.add_agent(agent)
    def add_agent(self, agent_dict):
        self.agent_details[agent_dict['agent_id']] = AgentDetails(agent_dict, self.cb)
        # self.agent_details[agent_dict['agent_id']].int = len(self.agent_details)*2 #TODO: remove this once TOC accepts string tasks
        self.format_agent_marker(self.agent_details[agent_dict['agent_id']], style='')

    """ Dynamic Fleet Management """
    def add_agent_ros_srv(self, agent_obj):
        agent_dict = self.agent_dict(agent_obj)
        if self.validate_dict(agent_dict):
            self.add_agent(self.format_dict(agent_dict))
            self.format_agent_marker(agent_dict['agent_id'], "")
            return {'success': 1, 'msg': 'agent added'}
        else:
            return {'success': 0, 'msg': 'agent unable to be added'}
    def remove_agent_ros_srv(self, srv):
        ### option 1. we create a disconnect task
        ### option 2. we create a callback called in task assignment
        # optino 3. we add a condition in start_next_task against a flag
        #    - how do we delete agent though?
        # optino 4. we add a condition in start_next_task to add a disconnect interrupt
        # option 5. we add check in run against task empty and disconnect flag
        # option 6. we attach a new base task called exit_at_node Navigation, Exit
        #    - if passed an invalid waypoint it will stop here it starts or go to base node

        if srv.agent_id in self.agent_details:
            task_location = ""  # TODO: find a persistant reference for this?
            srv.node_id = srv.node_id or task_location or self[srv.agent_id].location(accurate=True)
            self[srv.agent_id].add_task('exit_at_node', contacts={'exit_node':srv.node_id})
            return {'success': 1, 'msg': 'exit_at_node task added for %s'%srv.node_id}
        return {'success': 0, 'msg': 'agent not found'}

    """ New Agent Validation """
    def validate_dict(self, agent_dict): return True #TODO: setup validation system
    def format_dict(self, agent_dict):
        if  agent_dict['initial_location'] == '':
            agent_dict['initial_location'] = None
        return agent_dict
    def agent_dict(self, agent_obj):
        agent_dict = dict().copy()
        agent_dict['agent_id'] = agent_obj.agent_id
        agent_dict['setup'] = dict().copy()
        agent_dict['setup']['tasks'] = [{'module': task.module, 'role': task.role} for task in agent_obj.tasks]
        agent_dict['setup']['properties'] = {prop.key:prop.value for prop in agent_obj.properties}
        agent_dict['setup']['has_presence'] = agent_obj.has_presence
        agent_dict['initial_location'] = agent_obj.initial_location
        return agent_dict

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None

    """ Localisation Ovveride """
    def reset_location_srv(self, req):
        """ Call to reanable automonous localisation for each robot.

        :param node: String containing fake current_node
        :return: None
        """
        r = TriggerResponse()
        r.success = False
        for agent in self.agent_details:
            resp = agent.set_location_srv(AgentNodePairResponse())
        r.success = True
        r.msg = "Localisation resumed for all agents"
        return resp

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
        marker.type = agent.properties['rviz_type']

        #Define marker color ["remove", "red", "green", "blue", "black", "white", ""]
        marker.optional_color = style

        logmsg(category="rviz", msg="Setting %s %s(%s)"%(marker.type,marker.agent_id,marker.optional_color))

        self.set_marker_pub.publish(marker)


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
        # self.int = -1
        self.cb = callbacks
        from copy import deepcopy
        setup = deepcopy(agent_dict['setup'])

        #Subscriptions
        self.subs = {}

        #Navigation Defaults
        self.navigation = dict()

        #Task Defaults
        self.total_tasks = 0
        self.task = Task(default=True)
        self.task_buffer = []

        self.interruption = None
        self.registration = True
        self.properties = setup['properties']

        # Define interface for each role given #TODO: what about differentiating between Device and App?
        self.tasks = setup['tasks']
        self.roles = []
        self.interfaces = dict()
        for task in self.tasks:
            self.roles += [task['role']]
            interface_name = '%s_%s' % (task['module'], task['role'])
            definition = getattr(InterfaceDef, interface_name) #TODO: add catch for module not found
            self.interfaces[task['module']] = definition(agent=self)

        #Location and Callbacks
        self.location = Location(presence=setup['has_presence'], initial_location=agent_dict['initial_location'])

        #Start Task Progression
        self.add_init_task()

    """ Task Starters """
    def add_init_task(self):
        for task in self.tasks:
            self.add_task(task_name='%s_%s_init' % (task['module'], task['role']))
    def add_idle_task(self):
        for task in self.tasks:
            self.add_task(task_name='%s_%s_idle' % (task['module'], task['role']))
        if not self.task_buffer:
            logmsg(level="error", category="task", id=self.agent_id, msg="WARNING! Agent has no idle tasks.")
            logmsg(level="error", category="task",                   msg="    | All agents must be assigned a module.")
            logmsg(level="error", category="task",                   msg="    | Each module must have an idle task for each agent.")
            logmsg(level="error", category="task",                   msg="    \ Agent to be assigned a basic idle task.")
            self.add_task(task_name='idle')
    def add_task(self, task_name, task_id=None, task_stage_list=[], details={}, contacts={}, index=None, quiet=False, initiator_id=""):

        """ Called by task stages, used to buffer new tasks for the agent """
        if task_name not in dir(TaskDef):
            logmsg(level="warn", category="TASK", id=self.agent_id, msg="Task %s not found." % task_name)
            return

        #picker.interface.called(): self.agent.add_task('transportation_request_courier')
        1. #find TaskDef
        2. #task = TaskDef()
        3. #buffer += [task] OR buffer.insert(0, [task])

        task_def = getattr(TaskDef, task_name)
        task = task_def(self, task_id=task_id, details=details, contacts=contacts, initiator_id=initiator_id)
        if task:
            if not index: self.task_buffer += [task]
            else: self.task_buffer.insert(index, [task])

            if quiet:
                logmsg(category="DTM", msg="    | buffering %s to task_buffer[%i]" % (task.name, index or len(self.task_buffer)))
            else:
                logmsg(category="null")
                logmsg(category="TASK", id=self.agent_id, msg="Buffering %s to position %i of task_buffer, task stage list:" % (task.name, index or len(self.task_buffer)))
                [logmsg(category="TASK", msg='    - %s'%t) for t in task.stage_list]
    def start_next_task(self, idx=0):
        # if self.disconnect_on_task_completion: return #TODO: this condition not nescessary anymore
        if len(self.task_buffer) < 1: self.add_idle_task()
        if len(self.task_buffer) <= idx: return
        self.task = self.task_buffer.pop(idx)

        logmsg(category="null")
        logmsg(category="TASK", id=self.agent_id, msg="Active task: %s" % self['name'], speech=False)
        logmsg(category="TASK", msg="Task details:")
        [logmsg(category="TASK", msg="    - %s" % stage) for stage in self['stage_list']]


    """ Navigation """
    def map_cb(self, msg):
        """topological map callback
        """
        self.navigation['tmap'] = yaml.safe_load(msg.data)
        self.navigation['tmap_node_list'] = [node["node"]["name"] for node in self.navigation['tmap']['nodes']]
        self.navigation['tmap_available'] = copy.deepcopy(self.navigation['tmap'])
        self.navigation['available_route_search'] = TopologicalRouteSearch(self.navigation['tmap_available'])
    def is_node_restricted(self, node_id):
        """checks if a given node is in the robot's restricted tmap2
        :param node_id: name of the node, str
        """
        return ('tmap_node_list' in self.navigation and node_id in self.navigation['tmap_node_list'])
    def goal(self): return self().target

    """ Conveniences for Active Task """
    def __call__(A, index=0): return A.task.stage_list[index] if len(A.task.stage_list) > index else None
    def __getitem__(A, key):  return A.task[key] if A.task else None
    def __setitem__(A, key, val):    A.task[key] = val

    """ Standard Task Interactions """
    def start_stage(self):
        self()._start()
        self()._notify_start()
        self().new_stage = False
    def notify(self, state):
        self.interface.publish(state)
    def end_stage(self):
        from time import sleep; sleep(0.2) #TODO: this can be removed once add task to buffer is removed from async
        logmsg(category="stage", id=self.agent_id, msg="Stage %s is over" % self['stage_list'][0])
        self()._notify_end()
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

    """ Disconnection """
    def __del__(self):
        logmsg(level="error", agent=self.agent_id, msg="Identify why this does not get called.", speech=True)
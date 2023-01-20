#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import copy, gc, os, pprint, yaml, json
import whiptail
import traceback

from rospy import Subscriber, Publisher, Service, Time

from std_msgs.msg import String as Str, Empty, Bool
from std_srvs.srv import Trigger, TriggerResponse

from diagnostic_msgs.msg import KeyValue
from rasberry_coordination.msg import NewAgentConfig, MarkerDetails
from rasberry_coordination.msg import Agent, AgentList, AgentRegistration, AgentState, AgentLocation, AgentHealth, AgentRendering
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.agent_management.location_handler import LocationObj as Location
from rasberry_coordination.topomap_management.map_handler import MapObj as Map
from rasberry_coordination.task_management.containers.Module import ModuleObj as Module
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.task_management.__init__ import Interfaces

import rasberry_des.config_utils
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch


class AgentManager(object):

    """ Initialisation """
    def __init__(self, default_agents):
        self.agent_details = {}
        self.new_agent_buffer = dict()

        # Setup Connection for Dynamic Fleet
        file_name = 'coordinator-loaded-agents-save-state.yaml'  #Save to $(rospack find) rather than logs to $HOME/.ros/coordinator-loaded-agents-save-state.yaml
        if False: #os.path.isfile(file_name):
            if whiptail.Whiptail(title="Agent Management").confirm("Save-state detected, would you like to load it?"):
                with open(file_name) as file:
                    agent_dict = yaml.load(file, Loader=yaml.FullLoader)
                    pprint.pprint(agent_dict)
                    self.add_agents(agent_dict)

        # Load each of the default agents
        self.add_agents(default_agents)
        self.s = Subscriber('/rasberry_coordination/dynamic_fleet/add_agent', NewAgentConfig, self.add_agent_cb)

        # Marker Management
        self.cb = dict()
        self.set_marker_pub = Publisher('/rasberry_coordination/set_marker', MarkerDetails, queue_size=5)
        self.get_markers_sub = Subscriber('/rasberry_coordination/get_markers', Empty, self.get_markers_cb)

        # Fleet Monitoring
        self.fleet_pub = Publisher('/rasberry_coordination/fleet_monitoring/fleet', AgentList, latch=True, queue_size=2)
        self.fleet_last = None

    """ Dynamic Fleet """
    def add_agent(self, agent_dict):
        if agent_dict['agent_id'] not in self.agent_details:
            self.new_agent_buffer[agent_dict['agent_id']] = agent_dict
            pub2 = Publisher('/car_client/info/robots', Str, queue_size=1, latch=True)
            #^ TODO: this should not be included here
            pub2.publish(self.simplify())
    def add_agent_from_buffer(self):
        buffer, self.new_agent_buffer = self.new_agent_buffer, dict()
        for agent_dict in buffer.values():
            if agent_dict['agent_id'] not in self.agent_details.keys():
                pprint.pprint(agent_dict)
                self.agent_details[agent_dict['agent_id']] = AgentDetails(agent_dict, self.cb)
                logmsg(category="null")
    def add_agents(self, agent_list):
        for agent in agent_list: self.add_agent(agent)
    def add_agent_cb(self, msg):
        def kvp_list(msg): return {kvp.key: kvp.value for kvp in msg}
        self.add_agent({'agent_id': msg.agent_id,
                        'local_properties': kvp_list(msg.local_properties),
                        'modules': [{'name':m.name,'interface':str(m.interface), 'details':kvp_list(m.details)} for m in msg.modules]})

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None
    def get_agent_list_copy(self):
        return self.agent_details.copy()

    """ Fleet Monitoring """
    def fleet_monitoring(self):
        try:
            lst = [Agent(id=a.agent_id,
                         location=self.get_location(a),
                         registration=self.get_registration(a),
                         state=self.get_state(a),
                         rendering=self.get_rendering(a),
                         health=self.get_health(a)) for a in self.agent_details.values()]
            if self.fleet_last != str(lst):
                self.fleet_pub.publish(AgentList(list=lst))
                self.fleet_last = str(lst)
        except Exception as e:
            print(traceback.format_exc())

    def get_location(self, a):
        return AgentLocation(current_node=a.location.current_node,
                             current_edge=a.location.closest_edge)

    def get_registration(self, a):
        return AgentRegistration(registered=a.registration)

    def get_state(self, a):
        return AgentState(current_task_id=a['id'],
                          current_task=a['name'],
                          stage=a().__repr__(),
                          details=str(a['details']))

    def get_rendering(self, a):
        if 'base' in a.modules and 'rviz' in a.modules['base'].details:
            rviz=a.modules['base'].details['rviz']
            loca=a.local_properties
            col = ''
            col = rviz['colour'] if 'colour' in rviz else col
            col = loca['rviz_default_colour'] if 'rviz_default_colour' in loca else col
            stu = rviz['structure'] if 'structure' in rviz else ''
            return AgentRendering(colour=col, structure=stu)
        return AgentRendering(colour="None", structure="None")

    def get_health(self, a):
        if 'battery_level' in a.local_properties:
            return AgentHealth(battery_estimate=str(a.local_properties['battery_level']))
        return AgentHealth(battery_estimate="None")


    """ RViZ Visuals """
    def get_markers_cb(self, empty):
        """ Request from RViz to resend all markers """
        for a in self.agent_details.values():
            if 'marker' in a.modules['base'].details:
                m = a.modules['base'].details['marker']
                self.set_marker_pub.publish(m)

    """ Data """
    def simplify(self): #TODO: This should make use of the base interface instead
        out = {}
        for a in self.agent_details.values():
            if 'structure_type' not in a.modules['base'].details: continue
            st = a.modules['base'].details['structure_type']
            if st not in out: out[st] = {}
            for t in a.modules:
                if t in ['base', 'health_monitoring']: continue
                if t not in out[st]: out[st][t] = []
                out[st][t] += [a.agent_id]
        return Str(str(out or 'empty'))

class AgentDetails(object):
    """ Fields:
    """

    """ Initialisations """
    def __init__(self, agent_dict, callbacks):
        self.cb = callbacks
        self.agent_dict = agent_dict
        self.agent_id = agent_dict['agent_id']
        self.local_properties = agent_dict['local_properties']

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

        # Define interface for each interface
        logmsg(category="MODULE", id=self.agent_id, msg="Initialising Module Interfaces:")
        self.modules = {t['name']: Module(agent=self, name=t['name'], interface=t['interface'], details=t['details']) for t in agent_dict['modules'] if 'details' in t}

        #Location and Callbacks
        #if 'navigation' in self.modules:
        lp = self.local_properties
        np = self.modules['navigation'].interface.details
        initial_location = lp['initial_location'] if 'initial_location' in lp else ''
        has_presence = True if 'has_presence' in np and np['has_presence'] in [True, 'True'] else False
        self.location = Location(self, has_presence=has_presence, initial_location=initial_location)

        #Map
        topic = "/restricted_topological_map_generators/%s_topological_map_2" % np['restrictions'] if 'restrictions' in np else None
        self.map_handler = Map(agent=self, topic=topic)

        #Visualisers
        self.modules['base'].details['default_colour'] = lp['rviz_default_colour'] if 'rviz_default_colour' in lp else ''
        self.set_marker_pub = Publisher('/rasberry_coordination/set_marker', MarkerDetails, queue_size=5)

        #Debug
        self.speaker_pub = Publisher('/%s/ui/speaker'%self.agent_id, Str, queue_size=1)

        #Final Setup
        for m in self.modules.values(): m.add_init_task()
        self.format_marker(style='red')



    """ Task Starters """
    def add_idle_tasks(self):
        [self.add_task(module=m.name, name='idle') for m in self.modules.values() if m.name != "base"]
        if 'base' in self.modules:
            self.add_task(module='base', name='idle')
        if not self.task_buffer:
            logmsg(level="error", category="task", id=self.agent_id, msg="WARNING! Agent has no idle tasks.")
            logmsg(level="error", category="task", msg="    | All agents must be assigned a module.")
            logmsg(level="error", category="task", msg="    | Should each module have an idle task for each agent?")
            logmsg(level="error", category="task", msg="    \ Agent to be assigned a basic idle task.")
            self.add_task(module='base', name='idle')
    def add_task(self, module, name, task_id=None, task_stage_list=None, details=None, contacts=None, index=None, quiet=False, initiator_id=""):
        """ Called by task stages, used to buffer new tasks for the agent """
        task_stage_list = task_stage_list if task_stage_list else []
        details = details if details else {}
        contacts = contacts if contacts else {}

        if module not in self.modules:
            logmsg(category="TASK", msg="    | module: %s (not valid)"%module)
            return

        if not hasattr(self.modules[module].interface, name):
            logmsg(category="TASK", msg="    | task: %s (not found)"%name)
            return

        task_def = getattr(self.modules[module].interface, name)
        task = task_def(task_id=task_id, details=details, contacts=contacts, initiator_id=initiator_id)

        if not task:
            logmsg(category="TASK", msg="    :    | %s (empty)" % name)
            return

        name = name if task.name == name else "%s/%s"%(name,task.name)

        if not index: self.task_buffer += [task]
        else: self.task_buffer.insert(index, [task])

        if quiet:
            logmsg(category="DTM", msg="    :    | buffering %s to task_buffer[%i]" % (name, index or len(self.task_buffer)))
        else:
            logmsg(category="TASK",  msg="    | Buffering %s to task_buffer[%i]:" % (name, index or len(self.task_buffer)))
            [logmsg(category="TASK", msg="    :    | %s"%t) for t in task.stage_list]

    def start_next_task(self, idx=0):
        logmsg(category="TASK", id=self.agent_id, msg="Beginning next task", speech=False)

        if len(self.task_buffer) < 1: self.add_idle_tasks()
        if len(self.task_buffer) <= idx: return
        self.task = self.task_buffer.pop(idx)

        logmsg(category="TASK",  msg="- Active task: %s" % self['name'], speech=False)
        logmsg(category="TASK",  msg="  Task details:")
        [logmsg(category="TASK", msg="      | %s" % stage) for stage in self['stage_list']]

    def extend_task(self, task_name, task_id, details):
        if module not in self.interfaces:
            logmsg(category="TASK", msg="    | module: %s (not valid)"%task_name)
            return
        if name not in self.interfaces[module]:
            logmsg(category="TASK", msg="    | task: %s (not found)"%task_name)
            return
        task_def = self.interfaces[module][name]
        task = task_def(self, task_id=task_id, details=details)
        self['stage_list'] += task.stage_list


    """ Roles """
    def interfaces(self):
        return [m.interface for m in self.modules.values()]
    def send_car_msg(self, msg):
        interface = [v.interface for k,v in self.modules.items() if issubclass(type(v.interface), Interfaces['base']['StateInterface'])]
        if interface:
            interface[0].notify(msg)


    """ Navigation """
    def goal(self):
        if self().target_agent:
            return self().target_agent.location(accurate=True)
        return self().target


    """ Conveniences for Active Task """
    def __call__(A, index=0):
        if index:
            return A.task.stage_list[index] if len(A.task.stage_list) > index else None
        return A.task.stage_list[0] if A.task.stage_list else None
    def __getitem__(A, key):  return A.task[key] if A.task else None
    def __setitem__(A, key, val):    A.task[key] = val
    def simple_agent_id(self):
        id = self.agent_id.replace('thorvald','T').replace('picker','P').replace('storage','S')
        id = id if not id.startswith('STD_v2_') else "P%s"%id[-4:]
        return id

    """ Standard Task Interactions """
    def start_stage(self):
        self()._start()
        self().new_stage = False
    def end_stage(self):
        logmsg(category="stage", id=self.agent_id, msg="Stage %s is over" % self['stage_list'][0])
        self()._end()
        self['stage_list'].pop(0)
        return None if self['stage_list'] else self['id']


    """ Task Interruption """
    def set_interrupt(self, type, module, task_id, scope, quiet=False):
        if quiet:
            logmsg(category="DTM", msg="    - interrupt attached to %s of type: (%s,%s,%s)." % (self.agent_id, type, module, task_id))
        else:
            logmsg(category="DTM", msg="Interrupt attached to %s of type: (%s,%s,%s)." % (self.agent_id, type, module, task_id))
        self.interruption = (type, module, task_id, scope)


    """ Logging """
    def __repr__(self):
        if 'health_monitoring' in self.modules and self.modules['health_monitoring'].interface.in_auto_mode:
            return "%s(%s)" % (self.get_class(), self.agent_id)
        return "![%s(%s)]" % (self.get_class(), self.agent_id)
    def get_class(self):
        return str(self.__class__).replace("<class 'rasberry_coordination.agent_management.manager.", "").replace("'>", "")
    def speaker(self, msg):
        try:
            self.speaker_pub.publish(Str(msg))
        except:
            logmsg(level="debug", category="AGENT", id=self.agent_id, msg="Speaker pub not set.")


    """ Visuals """
    def format_marker(self, color=None, style=None):
        """
        Add/modify marker to display in rviz

        Create Marker: call self.format_marker("")
        Modify Marker: call self.format_marker("red")
        """
        if 'base' not in self.modules: return
        if 'rviz' not in self.modules['base'].details: return

        rviz = self.modules['base'].details['rviz']
        local = self.local_properties
        local['rviz_default_colour'] = local['rviz_default_colour'] if 'rviz_default_colour' in local else ''
        local['rviz_structure'] = local['rviz_structure'] if 'rviz_structure' in local else ''


        # Construct the marker details
        marker = MarkerDetails()
        marker.id = self.agent_id
        marker.structure = local['structure'] or rviz['structure']
        marker.colour = local['rviz_default_colour'] or rviz['colour'] or ''

        # Set where the location should come from
        if 'tf_source_topic' in rviz:
            marker.tf_source_topic = rviz['tf_source_topic'].replace('~','/%s/'%self.agent_id)
        if 'tf_source_type' in rviz:
            marker.tf_source_type = rviz['tf_source_type']

        # Attach the current pose
        if 'attach_pose' in rviz and rviz['attach_pose'] and self.map_handler.raw_msg:
            print(self.location())
            marker.pose = self.map_handler.get_node_pose(self.location())

        logmsg(category="rviz", msg="Setting %s %s(%s)" % (marker.structure, marker.id, marker.colour))
        self.modules['base'].details['marker'] = marker
        self.set_marker_pub.publish(marker)

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



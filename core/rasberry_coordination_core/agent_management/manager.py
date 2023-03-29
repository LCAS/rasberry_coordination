#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

# Builtins
import os
import copy, gc
import yaml, json
import traceback, pprint

# Messages
from std_msgs.msg import String as Str, Empty, Bool, ColorRGBA as ColourRGBA
from diagnostic_msgs.msg import KeyValue
from rasberry_coordination_msgs.msg import NewAgentConfig, MarkerDetails
from rasberry_coordination_msgs.msg import Agent, AgentList, AgentRegistration, AgentState, AgentLocation, AgentHealth, AgentRendering

# ROS2
from rasberry_coordination_core.node import GlobalNode
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

# Components
from rasberry_coordination_core.agent_management.location_handler import LocationObj as Location
from rasberry_coordination_core.topomap_management.map_handler import MapObj as Map
from rasberry_coordination_core.task_management.containers.Module import ModuleObj as Module
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.task_management.__init__ import Interfaces

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg


class AgentManager(object):

    """ Initialisation """
    def __init__(self, default_agents):
        self.agent_details = {}
        self.new_agent_buffer = dict()

        # CallARobot Info Publisher
        self.car_info_robots_pub = GlobalNode.create_publisher(Str, '/car_client/info/robots', 0)
        # todo: ^ should not be included here

        # Load each of the default agents
        self.add_agents(default_agents)
        topic = '~/agent_management/add_agent'
        self.s = GlobalNode.create_subscription(NewAgentConfig, topic, self.add_agent_cb, 0)

        # Marker Management
        self.cb = dict()
        self.set_marker_pub = GlobalNode.create_publisher(MarkerDetails, '~/agent_management/set_marker', 0)
        self.get_markers_sub = GlobalNode.create_subscription(Empty, '~/agent_management/get_markers', self.get_markers_cb, 0)

        # Fleet Monitoring
        qos = QoSProfile(depth=1,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.fleet_pub = GlobalNode.create_publisher(AgentList, '~/agent_management/fleet_details', qos)
        self.fleet_last = None

    """ Dynamic Fleet """
    def add_agent_cb(self, msg):
        def kvp_list(msg): return {kvp.key: yaml.safe_load(kvp.value) for kvp in msg}
        self.add_agent({'agent_id': msg.agent_id,
                        'local_properties': kvp_list(msg.local_properties),
                        'modules': [{'name':m.name,
                                     'interface':str(m.interface),
                                     'details':kvp_list(m.details)}
                                   for m in msg.modules]})

    def add_agents(self, agent_list):
        for agent in agent_list:
            self.add_agent(agent)

    def add_agent(self, agent_dict):
        if agent_dict['agent_id'] not in self.agent_details and \
           agent_dict['agent_id'] not in self.new_agent_buffer:
            self.new_agent_buffer[agent_dict['agent_id']] = agent_dict
            self.car_info_robots_pub.publish(self.simplify())

    def add_agent_from_buffer(self):
        buffer, self.new_agent_buffer = self.new_agent_buffer, dict()
        for agent_dict in buffer.values():
            if agent_dict['agent_id'] not in self.agent_details.keys():
                pprint.pprint(agent_dict)
                self.agent_details[agent_dict['agent_id']] = AgentDetails(agent_dict, self.cb)
                logmsg(category="null")

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None
    def get_agent_list_copy(self):
        return self.agent_details.copy()

    """ Fleet Monitoring """
    def fleet_monitoring(self):
        try:
            # Publish new version of agent monitoring
            lst = [Agent(id=a.agent_id,
                         location=self.get_location(a),
                         registration=self.get_registration(a),
                         state=self.get_state(a),
                         rendering=self.get_rendering(a),
                         health=self.get_health(a)) for a in self.agent_details.values()]
            if self.fleet_last != str(lst):
                self.fleet_pub.publish(AgentList(list=lst))
                self.fleet_last = str(lst)

            # Publish scheduler setup
            if 'scheduler' in self.agent_details:
                self.agent_details['scheduler'].modules['assignment'].interface.list_schedulable_tasks()

        except Exception as e:
            print(traceback.format_exc())

    def get_location(self, a):
        return AgentLocation(current_node=str(a.location.current_node),
                             current_edge=str(a.location.closest_edge))

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
            col = a.colour or col
            stu = rviz['structure'] if 'structure' in rviz else ''
            return AgentRendering(colour='#%s'%col, structure=stu)
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
        return Str(data=str(out if out else 'empty'))

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
        global Publisher
        self.colour = None
        self.modules['base'].details['default_colour'] = lp['rviz_default_colour'] if 'rviz_default_colour' in lp else ''
        self.set_marker_pub = GlobalNode.create_publisher(MarkerDetails, '~/agent/set_marker', 0)

        #Debug
        self.speaker_pub = GlobalNode.create_publisher(Str, f"/{self.agent_id}/verbal_speaker", 0)

        #Final Setup
        for m in self.modules.values(): m.add_init_task()
        self.format_marker(colour='FF0000')



    """ Task Starters """
    def add_idle_tasks(self):
        [self.add_task(module=m.name, name='idle') for m in self.modules.values() if m.name != "base"]
        if 'base' in self.modules:
            self.add_task(module='base', name='idle')
        if not self.task_buffer:
            logmsg(level="error", category="task", id=self.agent_id, msg="WARNING! Agent has no idle tasks.")
            logmsg(level="error", category="task", msg="   | All agents must be assigned a module.")
            logmsg(level="error", category="task", msg="   | Should each module have an idle task for each agent?")
            logmsg(level="error", category="task", msg="   \ Agent to be assigned a basic idle task.")
            self.add_task(module='base', name='idle')
    def add_task(self, module, name, task_id=None, task_stage_list=None, details=None, contacts=None, index=None, quiet=False, initiator_id=""):
        """ Called by task stages, used to buffer new tasks for the agent """
        task_stage_list = task_stage_list if task_stage_list else []
        details = details if details else {}
        contacts = contacts if contacts else {}

        if module not in self.modules:
            logmsg(category="TASK", msg="   | module: %s (not valid)"%module)
            return

        if not hasattr(self.modules[module].interface, name):
            logmsg(category="TASK", msg="   | task: %s (not found)"%name)
            return

        task_def = getattr(self.modules[module].interface, name)
        task = task_def(task_id=task_id, details=details, contacts=contacts, initiator_id=initiator_id)

        if not task:
            logmsg(category="TASK", msg="   | %s.%s (empty)" % (module, name))
            return

        if not index: self.task_buffer += [task]
        else: self.task_buffer.insert(index, [task])

        if quiet:
            name = name if task.name == name else "%s/%s"%(name,task.name)
            logmsg(category="DTM", msg="   :   | buffering %s to task_buffer[%i]" % (name, index or len(self.task_buffer)))
        else:
            logmsg(category="TASK",  msg="   | [%i] %s.%s:" % (index or len(self.task_buffer), module, task.name))
            [logmsg(category="TASK", msg="   :   | %s"%t) for t in task.stage_list]

    def start_next_task(self, idx=0):
        logmsg(category="TASK", id=self.agent_id, msg="Beginning idle tasks", speech=False)

        if len(self.task_buffer) < 1: self.add_idle_tasks()
        if len(self.task_buffer) <= idx: return
        self.task = self.task_buffer.pop(idx)

        logmsg(category="TASK",  msg="Active task: %s" % self['name'], speech=False)
        [logmsg(category="TASK", msg="   | %s" % stage) for stage in self['stage_list']]

    def extend_task(self, task_name, task_id, details):
        if module not in self.interfaces:
            logmsg(category="TASK", msg="   | module: %s (not valid)"%task_name)
            return
        if name not in self.interfaces[module]:
            logmsg(category="TASK", msg="   | task: %s (not found)"%task_name)
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
        id = self.agent_id.replace('thorvald','T').replace('picker','P').replace('storage','St')
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
            logmsg(category="DTM", msg="   - interrupt attached to %s of type: (%s,%s,%s)." % (self.agent_id, type, module, task_id))
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
    def format_marker(self, colour=None):
        """
        Add/modify marker to display in rviz

        Create Marker: call self.format_marker("")
        Modify Marker: call self.format_marker("FF0000")
        """
        if 'base' not in self.modules: return
        if 'rviz' not in self.modules['base'].details: return

        rviz = self.modules['base'].details['rviz']
        local = self.local_properties
        local['rviz_default_colour'] = local['rviz_default_colour'] if 'rviz_default_colour' in local else ''
        local['rviz_structure'] = local['rviz_structure'] if 'rviz_structure' in local else ''

        # Identify new colour
        self.colour = colour or local['rviz_default_colour'] or rviz['colour'] or ''

        # Construct the marker details
        marker = MarkerDetails()
        marker.id = self.agent_id
        marker.structure = local['rviz_structure'] or rviz['structure']
        marker.colour = ColourRGBA()
        if self.colour:
            marker.colour.r = float(int(self.colour[0:2],16))
            marker.colour.g = float(int(self.colour[2:4],16))
            marker.colour.b = float(int(self.colour[4:6],16))
            marker.colour.a = 1.0

        # Set where the location should come from
        if 'tf_source_topic' in rviz:
            marker.tf_source_topic = rviz['tf_source_topic'].replace('~','/%s/'%self.agent_id)
        if 'tf_source_type' in rviz:
            marker.tf_source_type = rviz['tf_source_type']

        # Attach the current pose
        if 'attach_pose' in rviz and rviz['attach_pose'] and self.map_handler.raw_msg:
            marker.pose = self.map_handler.get_node_pose(self.location())

        logmsg(category="rviz", msg="Setting %s %s(%s)" % (marker.structure, marker.id, str(marker.colour).replace('\n','')))
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
        [logmsg(category="DRM", msg="   - (%s) %s" % (i+1, r)) for i, r in enumerate(gc.get_referrers(a()))]
#    def __del__(self):
#        logmsg(level="warn", category="DRM", id=self.agent_id, msg="Agent handler is deleted.")
#        logmsg(level="warn", category="DRM", msg="Here, we must identify and unregister every subscriber")



#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide, jheselden
# @email: pdasgautham@gmail.com, marc@hanheide.net, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import operator
import Queue
import copy
import os
import csv
import time
import datetime
from datetime import datetime as dt
import threading
import gc
import sys
import weakref
import yaml
from pprint import pprint

import rospy
from rospy import Subscriber, Publisher
import rospkg

from std_msgs.msg import String as Str, Empty
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv

import rasberry_coordination.robot
from rasberry_coordination.msg import MarkerDetails, KeyValuePair
from rasberry_coordination.actions.action_manager import ActionManager
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak, Rasberry_Logger
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location, ModuleObj as Module

#Route Planning
import topological_navigation_msgs.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils
from rasberry_coordination.route_planners.route_planners import RouteFinder

#Task Managment
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef

#Agent Management
from rasberry_coordination.agent_management.agent_manager import AgentManager

class RasberryCoordinator(object):
    """RasberryCoordinator class definition
    """
    def __init__(self, agent_list, base_station_nodes_pool, wait_nodes_pool, planning_format, ns, special_nodes):
        logmsgbreak(total=1)
        print("------------------------------------------")
        logmsg(category="setup", msg='Coordinator initialisation begun')
        logmsgbreak(total=1)


        """ Meta Fields """
        self.ns = ns.strip("/") + "/"
        self.is_parent = True
        self.trigger_fresh_replan = False #ReplanTrigger
        self.last_replan_time = time.time()
        self.force_replan_to_publish = False
        self.log_count = 0
        self.log_routes = True
        self.action_print = True


        """ Initialise System Details: """
        self.special_nodes = special_nodes
        self.default_location = [n for n in self.special_nodes if 'default_location' in n['descriptors']][0]
        self.base_station_nodes_pool = base_station_nodes_pool
        self.wait_nodes_pool = wait_nodes_pool


        """ Initialise Agents: """
        callbacks = {'update_topo_map': None
                     , 'force_replan': self.force_replan
                     , 'trigger_replan': self.trigger_replan #ReplanTrigger
                     , 'default_location': self.default_location } #TODO: redesign for this to be gone
        self.agent_manager = AgentManager(callbacks)
        self.agent_manager.add_agents(agent_list)
        self.AllAgentsList = self.get_all_agents()


        """ Routing Details """
        self.route_finder = RouteFinder(planning_format=planning_format, agent_manager=self.agent_manager)
        self.force_replan_cb = Subscriber('/rasberry_coordination/force_replan', Empty, self.force_replan)


        # """ Communications Setup """
        # self.advertise_services()


        """ TOC Communications """
        self.TOC_Interface = InterfaceDef.TOC_Interface(self)


        """ Internal System Managers """
        self.action_manager = ActionManager(self.agent_manager, self.route_finder, self.special_nodes, self.get_all_agents)


        logmsg(category="setup", msg='Coordinator initialisation complete')
        print("------------------------------------------")
        logmsgbreak(total=1)
        return

    # def advertise_services(self):
    #     """Adverstise ROS services.
    #     Only call at the end of constructor to avoid calls during construction.
    #     If this is a parent class, call from child class
    #     """
    #     # advertise ros services
    #     logmsg(category="null")
    #     logmsg(category="setup", msg="Advertising services:")
    #     for attr in dir(self):
    #         if attr.endswith("_ros_srv"):
    #             service = getattr(self, attr)
    #             rospy.Service(
    #                 self.ns+attr[:-8],
    #                 service.type,
    #                 service
    #             )
    #             logmsg(category="setup", msg="    - %s%s" % (self.ns, attr[:-8]))
    def on_shutdown(self, ):
        """on shutdown cancel all goals
        """
        logmsg(level='warn', msg='Coordinator shutting down, performing shutdown actions')

        logmsg(level='warn', msg='AgentManager shutting down, saving agent_list')
        dict_list = [details.agent_dict for details in self.agent_manager.agent_details.values()]
        with open('coordinator-loaded-agents-save-state.yaml', 'w') as file: yaml.dump(dict_list, file)


    """ Main loop for task progression """
    def run(self, planning_type='fragment_planner'):

        # Remappings to for commonly used functions
        get_agents      = self.get_agents
        offer_service   = self.action_manager.offer_service
        find_routes     = self.route_finder.find_routes
        publish_route   = self.execute_policy_route
        trigger_routing = self.trigger_routing
        interrupt_task  = self.interrupt_task
        def action_print(): return self.action_print

        # Remappings for commonly referenced objects
        A   = get_agents()
        AM  = self.agent_manager
        TOC = self.TOC_Interface
        RL  = Rasberry_Logger(enable_task_logging=True)

        # Functions for simpler standard logging
        def l(idx): return; RL.log_minimal(idx, self.AllAgentsList)
        def lognull(): logmsg(category="null")
        def logbreak(section, condition):
            if any(condition):
                lognull()
                b = '\033[01;04;92m'
                a = '\033[38;5;231m\033[0m'
                logmsg(level="info", category="SECT", id="SECTION", msg="%s%s%s"%(b,section,a))

        # Timeout function for logging TOC updates
        from time import time as Now, sleep
        Ut = Now(); #time_since_TOC_update
        def Update_TOC(A, TOC, Ut):
            if any([a().new_stage for a in A]) or (Now() - Ut > 5):
                TOC.UpdateTaskList();
                return Now();
            return Ut

        # Begin task progression
        l(-1)
        while not rospy.is_shutdown():

            # Add New Agents
            new_agent_buffer = AM.new_agent_buffer
            logbreak("NEW AGENTS", new_agent_buffer)
            if new_agent_buffer: AM.add_agent_from_buffer();                                      """ Add New Agents """

            # Interrupt Stage Execution
            interrupts = [a.interruption for a in A] ; a = None; del A
            logbreak("INTERRUPTS", interrupts)
            if any(interrupts): interrupt_task();                                      """ Interrupt Stage Execution """

            # Update local list of Agents
            A = get_agents()

            # Start Buffered Task
            logbreak("START TASK", [not a['stage_list'] for a in A])
            [a.start_next_task() for a in A if not a['stage_list']]; l(0);                   """ Start Buffered Task """

            # Start Stage
            logbreak("START STAGE", [a().new_stage for a in A])
            [a.start_stage() for a in A if a().new_stage];                                           """ Start Stage """

            # Monitoring
            Ut = Update_TOC(A, TOC, Ut);
            AM.fleet_monitoring()

            # Offer Action Services
            #logbreak("ACTION", [a().action_required if action_print() else False for a in A])
            [offer_service(a) for a in A if a().action_required]; l(2);                            """ Offer Service """

            # Find Routes
            trigger = trigger_routing(A, reset_trigger=False)
            logbreak("ROUTE FIND", [trigger])
            if trigger_routing(A): find_routes();                                                    """ Find Routes """

            # Publish Routes
            logbreak("ROUTE PUBLISH", [a().route_found for a in A])
            [publish_route(a, trigger) for a in A if a().route_found]; l(3);                               """ Publish Routes """

            #Perform Stage-Completion Query
            [a()._query() for a in A]; l(4);                                                               """ Query """

            # End Stage
            logbreak("END", [a().stage_complete for a in A])
            E=[a.end_stage() for a in A if a().stage_complete];                                        """ End Stage """

            # Update TOC
            if any(E): TOC.EndTask(E);                                                   """ Update TOC w/ Completed """

            # Publish Log and Wait
            l(-2);
            rospy.sleep(0.2)

    def get_all_agents(self):
        return self.agent_manager.agent_details.copy() #TODO: is copy needed?
    def get_agents(self):
        self.AllAgentsList = self.get_all_agents()
        return self.AllAgentsList.values()

    """ Interrupt Task (is this old?) """
    def interrupt_task(self):
        from time import sleep; sleep(0.5) #TODO: preventing log overwriting from interrupt attachments
        interrupts = {'pause': self.pause, 'resume': self.resume, 'reset': self.reset, 'disconnect':self.disconnect}
        logmsg(category="null")
        logmsg(category="DTM", msg="Interrupt detected!", speech=False)
        [logmsg(category="DTM", msg="    | %s : %s" % (a.agent_id, a.interruption[0])) for a in self.AllAgentsList.values() if a.interruption]

        A = self.AllAgentsList
        interrupt_ids = [a.agent_id for a in A.values() if a.interruption and a.interruption[0] in interrupts]
        for aid in interrupt_ids: interrupts[A[aid].interruption[0]](A[aid])

    def pause(self, agent):
        """ Agent pausing works as follows:
        1. Put the active stage into a suspended state (so once active again it will be restarted)
        2. Add an additional pause stage which queries self.agent.registration
        """
        agent().suspend() #suspend active stage
        if agent().get_class() != "base.Pause":
            agent['stage_list'].insert(0, StageDef.Pause(agent, self.agent_manager.format_agent_marker)) #add paused stage

        scope = agent.interruption[3]
        agent().pause_state[scope] = True
        logmsg(category="DTM", msg="      | pause trigger ['%s'] set to True" % scope)
        logmsg(category="DTM", msg="      | stage state: %s" % agent().__repr__())

        agent.interruption = None  # reset interruption trigger
        self.agent_manager.format_agent_marker(agent, style='red')
    def resume(self, agent):
        """ Agent unpausing works as follows:
        1. Set the flag to end the pause stage (self.agent.registration)
        """
        logmsg(category="DTM", id=agent.agent_id, msg="Task advancement resumed.")

        scope = agent.interruption[3]
        # agent.registration = True  # enable generic query success condition
        if agent().get_class() == "base.Pause":
            agent().pause_state[scope] = False
            logmsg(category="DTM", msg="      | pause trigger ['%s'] set to False" % scope)
            logmsg(category="DTM", msg="      | stage state: %s" % agent().__repr__())

        agent.interruption = None  # reset interruption trigger
        # self.agent_manager.format_agent_marker(agent, style='')
    def reset(self, agent):
        """ Reset task works as follows:
        If the reset request comes from the initiator:
        - initiator deletes task
        - responder deletes task
        If the reset request comes from the responder:
        - initiator restarts task
        - responder deletes task
        """
        logmsg(category="DTM", msg="Request made to reset task: %s" % agent['id'])
        logmsg(category="DTM", msg="    | Task Details: %s" % agent.task)
        init, resp, tid = agent['initiator_id'], agent['responder_id'], agent['id']

        logmsg(category="DTM", msg="If the request came from TOC, it needs to release both?")

        if agent.agent_id == init:
            TaskDef.release_task(self.agent_manager[init])
            self.unregister(init)
        elif agent.agent_id == resp:
            TaskDef.restart_task(self.agent_manager[init])
            self.unregister(resp)
        self.agent_manager[init].interruption = None  # reset interruption trigger

        if resp and resp in self.agent_manager.agent_details.keys():
            logmsg(category="DTM", msg="    | responder exists: %s" % (resp))
            TaskDef.release_task(self.agent_manager[resp])
            self.agent_manager[resp].interruption = None  # reset interruption trigger
        else:
            logmsg(category="DTM", msg="    | responder does not exist: %s" % (resp))
            print()

        self.TOC_Interface.EndTask([tid])
    def unregister(self, agent_id):
        logmsg(category="DTM", msg="    | unregistering agent: %s"%agent_id)
        self.agent_manager[agent_id].registration = False
        self.agent_manager.format_agent_marker(self.agent_manager[agent_id], style='red')
    def disconnect(self, agent):
        a = weakref.ref(agent) ; del agent
        logmsg(level="error", category="DRM", id=a().agent_id, msg="Agent has been removed from coordinator.")
        logmsg(level="error", category="DRM", msg="    - Coordinator is no longer recieving location data")
        logmsg(level="error", category="DRM", msg="    - Agent is no longer reserving a node in the network")
        logmsg(level="error", category="DRM", msg="    - Ensure agent is moved away")
        a().delete_known_references(self)


    """ Publish route if different from current (move to route planner base? or at least route planning folder) """
    def execute_policy_route(self, agent, trigger=False):

        logmsg(category="route", id=agent.agent_id, msg="Attempting to publish route.")

        """ Publish ExecutePolicyModeGoal if different from current policy """
        policy = strands_navigation_msgs.msg.ExecutePolicyModeGoal()

        """ Define route, if no new route is generated, dont do anything. """
        policy.route.source = agent.route_fragments[0] if agent.route_fragments else None
        policy.route.edge_id = agent.route_edges[0] if agent.route_edges else None

        """ Flag to identify if new route is the same and should not be re-published """
        publish_route = True #assume route is identical
        rationalle_to_publish = ""
        reason_failed_to_publish = ""

        """ Identify key elements in routes. """
        old_node = agent.navigation_interface.execpolicy_goal.route.source
        old_edge = agent.navigation_interface.execpolicy_goal.route.edge_id
        new_node = policy.route.source
        new_edge = policy.route.edge_id

        """ If no new route is generated, dont do anything. """
        if (not new_node) or (not new_edge): return

        """ If old route exists, check against it """
        if old_node:
            publish_route = False  #assume new route is the same, so no need to publish
            reason_failed_to_publish = "Routes are same."

            """ Do lists have different lengths? """
            # old: R========T
            # new:     R====T
            if len(new_edge) > len(old_edge):
                publish_route = True
                rationalle_to_publish = "New route is larger then old route."
                logmsg(category="route", msg="    - new route longer than existing one")
            else:
                publish_route = False
                reason_failed_to_publish = "New shorter route could just be a partially used route"


            """ Compare routes for any differences from target to start. """
            if not publish_route:
                N = new_edge[::-1] #reverse
                O = old_edge[::-1] #reverse
                paired = zip(O[0:len(N)], N)  #trim old route

                for o, n in paired:
                    if o != n:
                        publish_route = True
                        rationalle_to_publish = "New route takes a different route to target."
                        logmsg(category="route", msg="    - new route different from existing route")
                        break
                    else:
                        publish_route = False
                        reason_failed_to_publish = "Old route uses same path as new route."

        """ If publish_route is True, routes are different """
        if publish_route or self.force_replan_to_publish:
            self.force_replan_to_publish = False
            if self.log_routes:
                logmsg(category="route", msg="    - new route generated:\n%s" % policy)
                logmsg(category="route", msg="    - previous route:\n%s" % agent.navigation_interface.execpolicy_goal)

            agent.navigation_interface.cancel_execpolicy_goal()
            agent.navigation_interface.set_execpolicy_goal(policy)
            agent.speaker("Hello. I am about to move. Be careful!")

            agent().route_required = False  # Route has now been published
            logmsg(category="route", id=agent.agent_id, msg="    - route published: %s" % rationalle_to_publish)

            now = str(dt.utcnow())
            path = "%s/routing/filtered_map_%s.prof" % (rospkg.RosPack().get_path('rasberry_coordination'), now.replace(' ','-'))
            with open("output_file.txt", "w") as f_handle:
                yaml.dump(policy, f_handle)

        else:
            logmsg(category="route", id=agent.agent_id, msg="    - route failed to published: %s" % reason_failed_to_publish)

        agent().route_found = False  # Route has now been published


    def force_replan(self, msg=None):
        logmsg(category="route", id="COORDINATOR", msg="Request to force replanning.")
        self.trigger_fresh_replan = True #ReplanTrigger
        self.force_replan_to_publish = True
    def trigger_replan(self):
        logmsg(category="route", id="COORDINATOR", msg="A route has been completed, refreshing routes")
        self.trigger_fresh_replan = True #ReplanTrigger
    def trigger_routing(self, A, reset_trigger=True):
        if self.trigger_fresh_replan:
            logmsg(level="error", category="route", id="COORDINATOR", msg="Replanning due to trigger")
            if reset_trigger:
                self.trigger_fresh_replan = False
        elif any([a().route_required for a in A]):
            logmsg(level="error", category="route", id="COORDINATOR", msg="Replanning due to agent needing a route")
        elif any([a for a in A if a.goal()]) and (time.time() - self.last_replan_time) > 10:
            logmsg(level="error", category="route", id="COORDINATOR", msg="Replanning due to timeout")
            if reset_trigger:
                self.last_replan_time = time.time()
        else:
            return False
        return True

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
import threading
import gc
import sys
import weakref
import yaml
from pprint import pprint

import rospy
from rospy import Subscriber, Publisher

from std_msgs.msg import Empty
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils

import rasberry_coordination.robot
import rasberry_coordination.srv
from rasberry_coordination.msg import MarkerDetails, KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak, Rasberry_Logger
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location, ModuleObj as Module

#Route Planning
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
        self.log_count = 0
        self.log_routes = False
        self.action_print = True

        """ Initialise System Details: """
        self.special_nodes = special_nodes
        self.base_station_nodes_pool = base_station_nodes_pool
        self.wait_nodes_pool = wait_nodes_pool

        """ Initialise Agents: """
        callbacks = {'update_topo_map': None
                     , 'trigger_replan': self.trigger_replan #ReplanTrigger
                    } #TODO: redesign for this to be gone
        self.agent_manager = AgentManager(callbacks)
        self.agent_manager.add_agents(agent_list)
        self.AllAgentsList = self.get_all_agents()


        """ Routing Details """
        self.route_finder = RouteFinder(planning_format=planning_format, agent_manager=self.agent_manager)
        self.replan_trigger_cb = Subscriber('/rasberry_coordination/force_replan', Empty, self.trigger_replan, )

        """ Communications Setup """
        self.advertise_services()

        """ TOC Communications """
        self.TOC_Interface = InterfaceDef.TOC_Interface(self)


        logmsg(category="setup", msg='Coordinator initialisation complete')
        print("------------------------------------------")
        logmsgbreak(total=1)
        return
    def advertise_services(self):
        """Adverstise ROS services.
        Only call at the end of constructor to avoid calls during construction.
        If this is a parent class, call from child class
        """
        # advertise ros services
        logmsg(category="null")
        logmsg(category="setup", msg="Advertising services:")
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(
                    self.ns+attr[:-8],
                    service.type,
                    service
                )
                logmsg(category="setup", msg="    - %s%s" % (self.ns, attr[:-8]))
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
        offer_service   = self.offer_service
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

            # Update TOC
            Ut = Update_TOC(A, TOC, Ut);                                                              """ Update TOC """

            # Start Stage
            logbreak("START STAGE", [a().new_stage for a in A])
            [a.start_stage() for a in A if a().new_stage];                                           """ Start Stage """

            # Offer Action Services
            logbreak("ACTION", [a().action_required if action_print() else False for a in A])
            [offer_service(a) for a in A if a().action_required]; l(2);                            """ Offer Service """

            # Find Routes
            logbreak("ROUTE FIND", [trigger_routing(A,reset_trigger=False)])
            if trigger_routing(A): find_routes();                                                    """ Find Routes """

            # Publish Routes
            logbreak("ROUTE PUBLISH", [a().route_found for a in A])
            [publish_route(a) for a in A if a().route_found]; l(3);                               """ Publish Routes """

            #Perform Stage-Completion Query
            [a()._query() for a in A]; l(4);                                                               """ Query """

            # End Stage
            logbreak("END", [a().stage_complete for a in A])
            E=[a.end_stage() for a in A if a().stage_complete];                                        """ End Stage """

            # Update TOC
            if any(E): TOC.EndTask(E);                                                   """ Update TOC w/ Completed """

            # Publish Log and Wait
            l(-2); rospy.sleep(0.2)

    def get_all_agents(self):
        return self.agent_manager.agent_details.copy() #TODO: is copy needed?
    def get_agents(self):
        self.AllAgentsList = self.get_all_agents()
        return self.AllAgentsList.values()

    """ Services offerd by Coordinator to assist with tasks """
    def offer_service(self, agent):
        action_type =       agent().action['action_type']
        action_style =      agent().action['action_style']

        responses = {'find_agent':self.find_agent,
                     'find_node': self.find_node}

        action_deets = agent().action.copy()
        del action_deets['action_type']
        del action_deets['action_style']
        del action_deets['response_location']

        s, self.action_print = self.action_print, False
        agent().action['response_location'] = responses[action_type](agent)
        self.action_print = s

        if agent().action['response_location']:
            #for better logging but worse code:
            self.action_print = True
            logmsg(category="action", id=agent.agent_id, msg="Perfoming %s(%s) - details: %s" % (action_type, action_style, action_deets))
            responses[action_type](agent)
            logmsg(category="action", msg="Found: %s" % (agent().action['response_location']))

            agent().action_required = False
        else:
            if self.action_print: logmsg(category="action", id=agent.agent_id, msg="Perfoming %s(%s) - details: %s" % (action_type, action_style, action_deets))
            responses[action_type](agent)
            if self.action_print: logmsg(category="action", msg="No response found, continuing in background, will update log when successful.")

            self.action_print = False

    """ Action Category """
    def find_agent(self, agent):
        action_style = agent().action['action_style']

        if 'list' in agent().action:
            agent_list = agent().action['list']
            A = {agent_id:self.AllAgentsList[agent_id] for agent_id in agent_list}
        else:
            agent_type = agent().action['agent_type']
            A = {a.agent_id:a for a in self.AllAgentsList.values() if (a is not agent) and (agent_type in a.roles())}

        responses = {"closest": self.find_closest_agent}  # TODO: ROOM TO EXPAND
        return responses[action_style](agent, A)
    def find_node(self, agent):
        action_style =      agent().action['action_style']


        if 'descriptor' in agent().action:
            if agent().action['action_style'] == "row_ends":
                N = agent().action['descriptor']
                if self.action_print: logmsg(category='action', msg='Finding ends to row: %s'%N)
            elif agent().action['action_style'] == "rows":
                N = agent().action['descriptor']
                if self.action_print: logmsg(category='action', msg='Finding rows in tunnel: %s'%N)
            else:
                descriptor = agent().action['descriptor']
                rl='response_location'
                AExcl = [a for _id,a in self.AllAgentsList.items()  if (_id is not agent.agent_id)]
                if self.action_print: logmsg(category='action', msg='Finding %s unoccupied node to: %s'%(action_style,agent.location()))

                occupied = [a.location.current_node for a in AExcl if a.location.current_node] #Check if node is occupied
                occupied += [a().action[rl] for a in AExcl if rl in a().action and a().action[rl]]  # TackleSharedTarget
                occupied += [a.goal() for a in AExcl if a.goal()]
                if self.action_print: logmsg(category='action', msg='Occupied Nodes: %s'%occupied)

                N2 = {n['id']:n for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)}
                N = [n['id'] for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)]
                if self.action_print: logmsg(category='action', msg='Nodes to Compare Against:')
                if self.action_print: [logmsg(category='action', msg="    - %s: %s"%(n,N2[n])) for n in N2]
        else:
            N = agent().action['list']
            if self.action_print: logmsg(category='action', msg='Nodes to Compare Against:')
            if self.action_print: [logmsg(category='action', msg="    - %s"%n) for n in N]


        responses = {"closest": self.find_closest_node,
                     "row_ends": self.find_row_ends,
                     "rows": self.find_rows}  # ROOM TO EXPAND
        return responses[action_style](agent, N)

    """ Action Style """
    def find_closest_agent(self, agent, agent_list):
        loc = agent.location()
        dist_list = {a.agent_id:self.dist(loc, a.location()) for a in agent_list.values() if a.registration and a.map_handler.is_node_restricted(loc)}
        lst = {a.agent_id:[loc, a.location(), self.dist(loc, a.location()), a.registration, a.map_handler.is_node_restricted(loc)] for a in agent_list.values()}

        if self.action_print: [logmsg(category="action", msg="    | %s location: [ %s | %s | %s ]"%(a.agent_id, a.location.current_node, a.location.closest_node, a.location.previous_node)) for a in agent_list.values()]
        if self.action_print: logmsg(category="action", msg="Finding closest in: %s" % dist_list)
        if self.action_print: logmsg(category="action", msg="Deets: %s" % lst)
        if dist_list:
            return agent_list[min(dist_list, key=dist_list.get)]
        return None
    def find_closest_node(self, agent, node_list):
        """ Find the closest node (via optimal route) to the given agent.

        :param agent: The agent to query against.
        :param node_list: The list of nodes to query.
        :return: The node_id closest to the agent querying against.
        """
        loc = agent.location()
        dist_list = {n:self.dist(loc,n) for n in node_list}
        if self.action_print: logmsg(category="action", msg="Finding closest in:")
        if self.action_print: [logmsg(category='action', msg="    - %s: %s"%(n,dist_list[n])) for n in dist_list]
        if dist_list:
            return min(dist_list, key=dist_list.get)
    def find_row_ends(self, agent, row_id):
        return self.route_finder.planner.get_row_ends(agent, row_id)
    def find_rows(self, agent, tunnel_id):
        return self.route_finder.planner.get_rows(agent, tunnel_id)

    """ Find Distance """
    def dist(self, start_node, goal_node):
        try:
            _,_,route_dists = self.get_path_details(start_node, goal_node)
            return sum(route_dists)
        except: pass
        return None

    """ Interrupt Task """
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
        self.agent_manager.format_agent_marker(agent.agent_id, style='red')
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
        # self.agent_manager.format_agent_marker(agent.agent_id, style='')
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


    """ Publish route if different from current """
    def execute_policy_route(self, agent):

        logmsg(category="route", id=agent.agent_id, msg="Attempting to publish route.")

        """ Publish ExecutePolicyModeGoal if different from current policy """
        policy = strands_navigation_msgs.msg.ExecutePolicyModeGoal()

        """ Define route, if no new route is generated, dont do anything. """
        policy.route.source = agent.route_fragments[0] if agent.route_fragments else None
        policy.route.edge_id = agent.route_edges[0] if agent.route_edges else None

        """ Flag to identify if new route is the same and should not be re-published """
        publish_route = True #assume route is identical

        """ Identify key elements in routes. """
        old_node = agent.navigation_interface.execpolicy_goal.route.source
        old_edge = agent.navigation_interface.execpolicy_goal.route.edge_id
        new_node = policy.route.source
        new_edge = policy.route.edge_id

        """ If no new route is generated, dont do anything. """
        if (not new_node) or (not new_edge): return

        """ If old route exists, check against it """
        if old_node:
            publish_route = False  #assume new route is the same
            reason_failed_to_publish = "Routes are same."

            """ Identify key elements in routes. """
            old_start_edge = old_edge[0]
            old_target_edge = old_node[-1]
            new_start_edge = new_edge[0]
            new_target_edge = new_node[-1]

            """ Do lists have different entrances to the target node? """
            # old: R========T
            # new:          T=====R
            if not publish_route and new_target_edge != old_target_edge:
                publish_route = True #route is different
            if not publish_route:
                reason_failed_to_publish = "Old route comes from same direction as new route."

            """ Do lists have different lengths? """
            # old: R========T
            # new:     R====T
            if not publish_route and len(new_edge) != len(old_edge):
                # If new_route is larger, routes are different
                if len(new_edge) > len(old_edge):
                    publish_route = True #route is different
                else:
                    """ Go backwards from target till smaller route is used up. """
                    # old: R========T
                    # new:     R====T
                    old_edge_crop=[]
                    for i, e in enumerate(list(zip(*(old_start_edge[::-1],new_start_edge[::-1])))):
                        old_edge_crop.append(e[0])
                    old_edge_crop.reverse()
                    old_edge = old_edge_crop
            if not publish_route:
                reason_failed_to_publish = "New route is longer then what remains of the old route."

            """ Do same-sized routes differ? """
            # old: ****R====T
            # new:     R=-_=T
            if not publish_route:
                for i, e in enumerate(list(zip(*(old_edge,new_edge)))):
                    if e[0] != e[1]:
                        logmsg(category="route", msg="    - new route different from existing route")
                        publish_route = True #route is different
                        break
                if not publish_route:
                    reason_failed_to_publish = "Old route uses same path as new route."

        """ If check_route is false, routes are different """
        if publish_route:
            if self.log_routes:
                logmsg(category="route", msg="    - new route generated:\n%s" % policy)
                logmsg(category="route", msg="    - previous route:\n%s" % agent.navigation_interface.execpolicy_goal)

            agent.navigation_interface.cancel_execpolicy_goal()
            agent.navigation_interface.set_execpolicy_goal(policy)

            agent().route_required = False  # Route has now been published
            logmsg(category="route", id=agent.agent_id, msg="    - route published")
        agent().route_found = False  # Route has now been published
        rospy.sleep(1)
    # def new_execute_policy_route(self, agent):
    #     logmsg(category="route", id=agent.agent_id, msg="Attempting to publish route.")
    #
    #     # Define route to send
    #     policy = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
    #     policy.route.source = agent.route_fragments[0] if agent.route_fragments else None
    #     policy.route.edge_id = agent.route_edges[0] if agent.route_edges else None
    #
    #     # Return if new route is empty
    #     if (not policy.route.source) or (not policy.route.edge_id): return
    #
    #     # Identify Old and New routes
    #     old_route_edges = agent.navigation_interface.execpolicy_goal.route.edge_id; old_route_edges.reverse()
    #     new_route_edges = policy.route.edge_id; new_route_edges.reverse()
    #     publish_route = False
    #
    #     # Identify if either is empty
    #     route_contents = [int(not old_route_edges), int(not new_route_edges)];
    #     publish_route = (sum(route_contents) == 1)  #we cant itterate an empty list, but new route might be empty
    #
    #     # Itterate through edges to see if there is a difference
    #     for i in range(1, min([len(old_route_edges), len(new_route_edges)])):
    #         o = old_route_edges[i]
    #         n = new_route_edges[i]
    #         if o != n:
    #             publish_route = True
    #             break
    #
    #     # Publish goal
    #     if publish_route:
    #         agent.navigation_interface.cancel_execpolicy_goal()
    #         agent.navigation_interface.set_execpolicy_goal(policy)
    #         agent().route_required = False
    #         resp = "successful"
    #     else:
    #         resp = "unsuccessful, new route is not significantly different"
    #     logmsg(category="route", msg="    - publish attempt %s" % resp)
    #
    #     agent().route_found = False  # Used to trigger replanning
    #
    #     #Put a delay in route searching, we dont need new attempts every ms
    #     rospy.sleep(1)
    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:

        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_finder.planner.route_search.search_route(start_node, goal_node)
        if route is None:
            if start_node == goal_node:
                logmsg(category="route", msg='start_node %s is goal_node %s' % (start_node, goal_node))
                return ([], [], [0])
            else:
                logmsg(category="route", msg='no route between %s and %s' % (start_node, goal_node))
                return ([], [], [float("inf")])
        route_nodes = route.source
        route_nodes.append(goal_node)
        route_edges = route.edge_id

        for i in range(len(route_nodes) - 1):
            route_distance.append(self.route_finder.planner.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distance)
    def trigger_replan(self, msg=None):
        logmsg(category="route", msg="A route has been completed, refreshing routes")
        self.trigger_fresh_replan = True #ReplanTrigger
    def trigger_routing(self, A, reset_trigger=True):
        """
        route_required => agent is doing navigation task
        route_found => agent has been assigned a route

        # a.Navigation:  REQUIRED=true
        # replan:        TRIGGER=true
        #
        # find\REQUIRED: FOUND=true
        #     |TRIGGER:  TRIGGER=false
        #
        # publish\FOUND: FOUND=false,
        #                REQUIRED=false
        """

        if any([a().route_required for a in A]):
            return True
        elif self.trigger_fresh_replan:
            if reset_trigger:
                logmsg(category="route", msg="Replanning is triggered")
                self.trigger_fresh_replan = False #ReplanTrigger
            return True
        return False

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
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak
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
        logmsg(level="error", category="setup", msg='Issue identified with MQTT:')
        logmsg(level="error", category="setup", msg='    picker01, picker02, storage01 are located on server_uri')
        logmsg(level="error", category="setup", msg='    thorvald_001, thorvald_002 are located on robots_uri')
        logmsg(level="error", category="setup", msg='    details for each agents setup is launched as a latched topic')
        logmsg(level="error", category="setup", msg='    when multisim is started, each agent in turn publishes their info')
        logmsg(level="error", category="setup", msg='    the coordinator recieves each message as it is published')
        logmsg(level="error", category="setup", msg='    if the coordinator is restarted, it reads the collective latched messages')
        logmsg(level="error", category="setup", msg='    (rostopic echo */add_agent)')
        logmsg(level="error", category="setup", msg='    when thorvald_002 info is run on startup, it passes through the mqtt')
        logmsg(level="error", category="setup", msg='    mqtt overrites the last message which was latched for this topic')
        logmsg(level="error", category="setup", msg='    so thorvald_001 is missing from the rostopic echo, and thus not connected')
        logmsg(level="error", category="setup", msg='    ')
        logmsg(level="error", category="setup", msg='Our options:')
        logmsg(level="error", category="setup", msg='    1, run an agent_info manager node on robots_uri to publish a latchd list of all robots')
        logmsg(level="error", category="setup", msg='    2, fix mqtt')
        logmsg(level="error", category="setup", msg='    3, make coordinator query all agent info topics')
        logmsg(level="error", category="setup", msg='    4, make coordinator query all topics searching for new AgentDetails messages')
        logmsg(level="error", category="setup", msg='    5, make details periodically republish')
        logmsgbreak(total=1)
        print("------------------------------------------")
        logmsg(category="setup", msg='Coordinator initialisation begun')
        logmsgbreak(total=1)

        """ Meta Fields """
        self.ns = ns.strip("/") + "/"
        self.is_parent = True
        self.trigger_fresh_replan = False #ReplanTrigger
        self.log_count = 0


        """ Initialise Task Parameters: """ #This should be done within Stages.py programatically
        """
        self.active_tasks = active_tasks
        self.max_load_duration = max_load_duration
        self.max_unload_duration = max_unload_duration
        self.max_task_priority = max_task_priority
        """

        """ Initialise System Details: """
        self.special_nodes = special_nodes
        self.base_station_nodes_pool = base_station_nodes_pool
        self.wait_nodes_pool = wait_nodes_pool

        # Cold storage node is an agnet now, and should be defined as such
        """
        self.use_cold_storage = use_cold_storage
        self.cold_storage_node = cold_storage_node
        """

        """ Initialise Agents: """
        callbacks = {'update_topo_map': None
                     , 'trigger_replan': self.trigger_replan #ReplanTrigger
                    }
        #TODO: redesign for this to be gone

        #Define Agent Manager
        self.agent_manager = AgentManager(callbacks)
        self.agent_manager.add_agents(agent_list)
        self.AllAgentsList = self.get_all_agents()


        """ Routing Details """
        self.route_finder = RouteFinder(planning_format=planning_format, agent_manager=self.agent_manager)
        self.replan_trigger_cb = Subscriber('/rasberry_coordination/force_replan', Empty, self.trigger_replan, )

        """ Communications Setup """
        self.advertise_services()

        # Initialise topics for marker management #Combine these into 1 topic
        # self.marker_add_pub = Publisher('/rasberry_coordination/marker_add', MarkerDetails, queue_size=5)
        # self.marker_remove_pub = Publisher('/rasberry_coordination/marker_remove', MarkerDetails, queue_size=5)

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
        logmsg(level='warn', msg='shutting down all actions')


    """ Main loop for task progression """
    def run(self, planning_type='fragment_planner'):
        # Remappings to simplify function
        AM              = self.agent_manager
        offer_service   = self.offer_service
        l               = self.log_minimal
        find_routes     = self.route_finder.find_routes
        publish_route   = self.execute_policy_route
        get_agents      = self.get_agents
        interrupt_task  = self.interrupt_task
        TOC             = self.TOC_Interface
        trigger_routing = self.trigger_routing
        def lognull(): logmsg(category="null")
        def logbreak(section, condition):
            if any(condition):
                lognull()
                b = '\033[01;04;92m'
                a = '\033[38;5;231m\033[0m'
                logmsg(level="info", category="SECT", id="SECTION", msg="%s%s%s"%(b,section,a))

        A = get_agents()
        self.enable_task_logging = True
        self.task_progression_log = 'task_progression.csv' #logs to $HOME/.ros/task_progression.csv
        self.log_routes = True
        self.action_print = True
        self.timestep = 0
        self.iteration = 0
        self.previous_log_iteration = ""
        self.current_log_iteration = ""

        from time import time as Now, sleep
        Ut = Now(); #time_since_TOC_update
        def Update_TOC(A, TOC, Ut):
            if any([a().new_stage for a in A]) or (Now() - Ut > 5):
                # logbreak("TOC", [a().new_stage for a in A])
                TOC.UpdateTaskList();
                return Now();
            return Ut

        l(-1)
        while not rospy.is_shutdown():

            new_agent_buffer = AM.new_agent_buffer
            logbreak("NEW AGENTS", new_agent_buffer)
            if new_agent_buffer: AM.add_agent_from_buffer();                       """ Add New Agents """

            interrupts = [a.interruption for a in A] ; a = None; del A
            logbreak("INTERRUPTS", interrupts)
            interrupt_task()     if any(interrupts) else None;                     """ Interrupt Stage Execution """

            A = get_agents()

            logbreak("START TASK", [not a['stage_list'] for a in A])
            [a.start_next_task() for a in A if not a['stage_list']];               """ Start Buffered Task """
            l(0);

            Ut = Update_TOC(A, TOC, Ut);                                           """ Update TOC """

            logbreak("START STAGE", [a().new_stage for a in A])
            [a.start_stage()     for a in A if a().new_stage];                     """ Start Stage """

            if self.action_print: logbreak("ACTION", [a().action_required for a in A])
            [offer_service(a)    for a in A if a().action_required];               """ Offer Service """
            l(2)

            logbreak("ROUTE FIND", [trigger_routing(A)])
            if trigger_routing(A): find_routes();                                  """ Find Routes """

            logbreak("ROUTE PUBLISH", [a().route_found for a in A])
            [publish_route(a)    for a in A if a().route_found];                   """ Publish Routes """
            l(3)

            [a()._query()        for a in A];                                      """ Query """
            l(4)

            logbreak("END", [a().stage_complete for a in A])
            E=[a.end_stage()     for a in A if a().stage_complete];                """ End Stage """

            TOC.EndTask(E) if any(E) else None;                                    """ Update TOC with Ended Tasks """

            l(-2) #PUBLISH LOG
            sleep(0.2)

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
        dist_list = {a.agent_id:self.dist(loc, a.location()) for a in agent_list.values() if a.registration and a.is_node_restricted(loc)}
        lst = {a.agent_id:[loc, a.location(), self.dist(loc, a.location()), a.registration, a.is_node_restricted(loc)] for a in agent_list.values()}

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
        _,_,route_dists = self.get_path_details(start_node, goal_node)
        return sum(route_dists)

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
        agent()._suspend() #suspend active stage

        scope = agent.interruption[3]
        if agent().get_class() != "base.Pause":
            agent['stage_list'].insert(0, StageDef.Pause(agent, self.agent_manager.format_agent_marker)) #add paused stage
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
        logmsg(level='warn', category="route", msg="    - check_route label 1")

        """ Identify key elements in routes. """
        old_node = agent.temp_interface.execpolicy_goal.route.source
        old_edge = agent.temp_interface.execpolicy_goal.route.edge_id
        new_node = policy.route.source
        new_edge = policy.route.edge_id
        # logmsg(category="ROB_PY", id=agent.agent_id, msg="    - route to join {0} and {1}"%(old_node, new_node))

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
                logmsg(level='warn', category="route", msg="    - check_route label 2")
                publish_route = True #route is different
            if not publish_route:
                reason_failed_to_publish = "Old route comes from same direction as new route."

            """ Do lists have different lengths? """
            # old: R========T
            # new:     R====T
            if not publish_route and len(new_edge) != len(old_edge):
                logmsg(level='warn', category="route", msg="    - check_route label 3")
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
                logmsg(level='warn', category="route", msg="    - check_route label 4")
                for i, e in enumerate(list(zip(*(old_edge,new_edge)))):
                    if e[0] != e[1]:
                        logmsg(category="route", msg="    - new route different from existing route")
                        publish_route = True #route is different
                        break
                if not publish_route:
                    reason_failed_to_publish = "Old route uses same path as new route."

        """ If check_route is false, routes are different """
        if publish_route:
            logmsg(level='warn', category="route", msg="    - check_route label 5")
            if self.log_routes:
                logmsg(category="route", msg="    - new route generated:\n%s" % policy)
                logmsg(category="route", msg="    - previous route:\n%s" % agent.temp_interface.execpolicy_goal)

            logmsg(category="test", msg="publish route::cancel_execpolicy_goal")
            agent.temp_interface.cancel_execpolicy_goal()
            agent.temp_interface.set_execpolicy_goal(policy)

            agent().route_required = False #ReplanTrigger
            logmsg(level='warn', category="route", id=agent.agent_id, msg="    - route published")
        agent().route_found = False  # ReplanTrigger?
        logmsg(level='warn', category="route", id=agent.agent_id, msg="    - route publish attempt complete")
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
    #     old_route_edges = agent.temp_interface.execpolicy_goal.route.edge_id; old_route_edges.reverse()
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
    #         agent.temp_interface.cancel_execpolicy_goal()
    #         agent.temp_interface.set_execpolicy_goal(policy)
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
    def trigger_routing(self, A):
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
            logmsg(category="route", msg="Replanning is triggered")
            self.trigger_fresh_replan = False #ReplanTrigger
            return True
        return False


    """ Task Stage Logging """
    def log_linebreak(self):
        dash_lengths = [13, 13, 36, 20, 3]
        dashes = []
        dashes += [',|,'.join(['-'*dl for dl in dash_lengths[:3]])]
        dashes += [',|,'.join(['-'*dash_lengths[3] for a in range(len(self.AllAgentsList))])]
        dashes += ['-' *dash_lengths[4]]
        return dashes
    def log_init(self):
        with open(self.task_progression_log, 'w+') as log:
            log.write(' ,'*5+"|,Agents:\n")

        return ['Timestep','Iteration','Stage']+[a.agent_id for a in self.AllAgentsList.values()]
    def log_break(self):
        return ['' for a in range(3+len(self.AllAgentsList))]
    def log_iteration(self):
        return ['%s','%s']+['' for a in range(1 + len(self.AllAgentsList))]

    def log_value(self, detail):
        return ['','']+[a[detail] for a in self.AllAgentsList.values()]
    def log_not_none(self, detail):
        return ['', ''] + [a[detail] is not None for a in self.AllAgentsList.values()]

    def log_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a['stage_list']:
                stages += [a().get_class()]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_new_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a['stage_list']:
                stages += [a().new_stage]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_task(self):
        return ['', ''] + [a['name'] for a in self.AllAgentsList.values()]

    def log_summary(self, detail):
        lst=[]
        for a in self.AllAgentsList.values():
            switch = {'_start':a().new_stage,
                      '_notify_start':a().new_stage,
                      '_action':a().action_required,
                      '_query':a().stage_complete,
                      '_notify_end':a().stage_complete,
                      '_del':a().stage_complete}
            if switch[detail]:
                lst += [a().summary[detail]]
            else:
                lst += ['-']
        return ['','']+lst

    def log_data(self, switches, detail=None, linebreak=False):
        if not self.enable_task_logging:
            return

        switch_group_empty = {'init':self.log_init,
                              'break':self.log_break,
                              'iteration':self.log_iteration, #Make timestep query if time since last post has exceeded T
                              'task':self.log_task,
                              'stage': self.log_stage,
                              'new_stage':self.log_new_stage,
                              'linebreak': self.log_linebreak}
        switch_group_value = {'route':self.log_not_none,          #('route')
                              'action_required':self.log_value,   #('action_required')
                              'route_required':self.log_value,    #('route_required')
                              'stage_complete':self.log_value}    #('stage_complete')
        switch_group_summary = {'_start':self.log_summary,        #('_start')
                                '_notify_start':self.log_summary, #('_notify_start')
                                '_action':self.log_summary,       #('_action')
                                '_query':self.log_summary,        #('_query')
                                '_notify_end':self.log_summary,   #('_notify_end')
                                '_del':self.log_summary}          #('_del')


        for switch in switches:
            details = switch_group_empty[switch]() if switch in switch_group_empty else None
            details = switch_group_value[switch](switch) if switch in switch_group_value else details
            details = switch_group_summary[switch](switch) if switch in switch_group_summary else details

            for idx, item in enumerate(details[2:]):
                if item in [False, None]:
                    details[idx+2] = '-'

            if switch in ["break", "iteration"]:
                details.insert(2,'')
            elif switch in ["init", "linebreak"]:
                pass
            else:
                details.insert(2, switch)
                details += ['']
            details = [str(d) for d in details]
            self.current_log_iteration += "%s\n" % ',|,'.join(details)
    def publish_log(self):
        #TODO: add extra flag to set is ANY log returns a value? or query against empty log?
        if self.enable_task_logging:
            if self.previous_log_iteration != self.current_log_iteration:
                with open(self.task_progression_log, 'a') as log:
                    log.write(self.current_log_iteration % (self.timestep, self.iteration)) #use rospy.Time.now() ?
                    self.iteration += 1
                    logmsgbreak()
                    # logmsg(category="log", msg="Updating log")
                    print('\033[07m------------------------------------------\033[00m')

            self.previous_log_iteration = self.current_log_iteration
            self.current_log_iteration = ""

    def log_minimal(self, idx):
        switch = {-2: self.publish_log,
                  -1:['init'],
                  0: ['linebreak', 'iteration', 'task', 'stage', 'new_stage', 'break'],
                  1: ['_start', '_notify_start', 'break'],
                  2: ['_action', 'break'],
                  3: ['route_required'],
                  4: ['route', 'break'],
                  5: ['_query', 'break', '_notify_end', '_del']}

        if idx == min(switch):
            switch[idx]()
        else:
            self.log_data(switch[idx])
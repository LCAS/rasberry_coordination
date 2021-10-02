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
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak, remove, add, move

#Route Planning
from rasberry_coordination.route_planners.route_planners import RouteFinder

#Task Managment
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef

#Agent Management
from rasberry_coordination.agent_management.agent_manager import AgentManager

class RasberryCoordinator(object):
    """RasberryCoordinator class definition
    """
    def __init__(self, agent_list, base_station_nodes_pool, wait_nodes_pool, planning_type, ns, special_nodes):
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
        # routing_cb = {'publish_task_state': self.publish_task_state,
        #               'send_robot_to_base': self.send_robot_to_base}  # These need to be eventually managed better
        self.route_finder = RouteFinder(planning_type=planning_type, agent_manager=self.agent_manager)
        self.replan_trigger_cb = Subscriber('/rasberry_coordination/force_replan', Empty, self.trigger_replan, )

        """ Communications Setup """
        self.advertise_services()

        # Initialise topics for marker management #Combine these into 1 topic
        self.marker_add_pub = Publisher('/rasberry_coordination/marker_add', MarkerDetails, queue_size=5)
        self.marker_remove_pub = Publisher('/rasberry_coordination/marker_remove', MarkerDetails, queue_size=5)

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
        offer_service   = self.offer_service
        l               = self.log_minimal
        find_routes     = self.route_finder.find_routes
        publish_route   = self.execute_policy_route
        get_agents      = self.get_agents
        interrupt_task  = self.interrupt_task
        TOC             = self.TOC_Interface
        trigger_routing = self.trigger_routing
        def lognull(): logmsg(category="null")
        # def interrupt_all(): return self.agent_manager.pause_all

        A = get_agents()
        self.enable_task_logging = True
        self.task_progression_log = '/home/jheselden/task_progression.csv'
        self.log_routes = True
        self.timestep = 0
        self.iteration = 0
        self.previous_log_iteration = ""
        self.current_log_iteration = ""

        l(-1)
        while not rospy.is_shutdown():
            interrupt_task()    if any([a.interruption for a in A]) else None;     """ Interrupt Task Execution """
            A = get_agents()

            lognull() if any([not a.task_stage_list for a in A]) else None
            [TOC.End(a) for a in A if not a.task_stage_list];                      """ Update TOC with Ended Tasks """  # TODO: Use better condition
            [a.start_next_task() for a in A if not a.task_stage_list];             """ Start Buffered Task """
            l(0);

            lognull() if any([a().new_stage for a in A]) else None
            TOC.Update() if any([a().new_stage for a in A]) else None;             """ Update TOC """ #TODO: Add better conditional
            [a.start_stage()    for a in A if a().new_stage];                      """ Start Stage """
            lognull() if any([a().action_required for a in A]) else None
            [offer_service(a)   for a in A if a().action_required];                """ Offer Service """
            l(2)

            # lognull() if any([a().route_required for a in A]) else None
            if trigger_routing(A): find_routes();                                  """ Find Routes """
            [publish_route(a)  for a in A if a().route_found];                     """ Publish Routes """
            l(3)

            [a()._query()       for a in A];                                       """" Query """
            l(4)

            lognull() if any([a().stage_complete for a in A]) else None
            [a.end_stage()      for a in A if a().stage_complete];                 """ End Stage """
            l(-2) #publish route

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
                     'find_node': self.find_node,
                     'find_agent_from_list': self.find_agent_from_list}  # ROOM TO EXPAND

        action_deets = agent().action.copy()
        del action_deets['action_type']
        del action_deets['action_style']
        del action_deets['response_location']
        logmsg(category="action", id=agent.agent_id, msg="Perfoming %s(%s) - details: %s" % (action_type, action_style, action_deets))

        agent().action['response_location'] = responses[action_type](agent)

        if agent().action['response_location']:
            logmsg(category="action", msg="Found: %s" % (agent().action['response_location']))
            agent().action_required = False

    """ Action Category """
    def find_agent(self, agent):
        action_style = agent().action['action_style']
        agent_type =   agent().action['agent_type']

        A = {a.agent_id:a for a in self.AllAgentsList.values() if (a is not agent) and (agent_type in a.roles)}

        responses = {"closest": self.find_closest_agent}  # TODO: ROOM TO EXPAND
        return responses[action_style](agent, A)
    def find_node(self, agent):
        action_style =      agent().action['action_style']
        descriptor =        agent().action['descriptor']
        rl='response_location'
        AExcl = [a for _id,a in self.AllAgentsList.items()  if (_id is not agent.agent_id)]
        logmsg(category='action', msg='Finding %s unoccupied node to: %s'%(action_style,agent.location()))


        # t1 = [a.current_node for a in AExcl if a.current_node]
        # t2 = [a().action[rl] for a in AExcl if rl in a().action and a().action[rl]]
        # t3 = [a.goal() for a in AExcl if a.goal()]
        # logmsg(level='error', category='action', msg='Physically Occupied Nodes: %s'%t1, speech=True)
        # logmsg(level='error', category='action', msg='Reservered Nodes: %s'%t2, speech=True)
        # logmsg(level='error', category='action', msg='Goal Nodes: %s'%t3, speech=True)

        taken = [a.current_node for a in AExcl if a.current_node] #Check if node is occupied
        taken += [a().action[rl] for a in AExcl if rl in a().action and a().action[rl]]  # TackleSharedTarget
        taken += [a.goal() for a in AExcl if a.goal()]
        logmsg(category='action', msg='Occupied Nodes: %s'%taken)



        N = {n['id']:n for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in taken)}
        logmsg(category='action', msg='Nodes to Compare Against:')
        [logmsg(category='action', msg="    - %s: %s"%(n,N[n])) for n in N]

        responses = {"closest": self.find_closest_node}  # ROOM TO EXPAND
        return responses[action_style](agent, N)
    def find_agent_from_list(self, agent):
        action_style =      agent().action['action_style']
        agent_list =        agent().action['list']

        A = {agent_id:self.AllAgentsList[agent_id] for agent_id in agent_list} #convert list to set()

        responses = {"closest": self.find_closest_agent}  # ROOM TO EXPAND
        return responses[action_style](agent, A)

    """ Action Style """
    def find_closest_agent(self, agent, agent_list):
        """ Find the closest agent (via optimal route) to the given agent.

        :param agent: The agent to query against.
        :param agent_list: The list of agents to query.
        :return: The agent_details object closest to the agent querying against.
        """
        dist_list = {a.agent_id:self.dist(agent.location(),a.location()) for a in agent_list.values() if a.registration}
        logmsg(category="action", msg="Finding closest in: %s" % dist_list)
        if dist_list:
            return agent_list[min(dist_list, key=dist_list.get)]
        else:
            return None
    def find_closest_node(self, agent, node_list):
        """ Find the closest node (via optimal route) to the given agent.

        :param agent: The agent to query against.
        :param node_list: The list of nodes to query.
        :return: The node_id closest to the agent querying against.
        """
        dist_list = {n:self.dist(agent.location(),n) for n in node_list}
        logmsg(category="action", msg="Finding closest in:")
        [logmsg(category='action', msg="    - %s: %s"%(n,dist_list[n])) for n in dist_list]
        return min(dist_list, key=dist_list.get)
    """ Find Distance """
    def dist(self, start_node, goal_node):
        _,_,route_dists = self.get_path_details(start_node, goal_node)
        return sum(route_dists)


    """ Interrupt Task """
    def interrupt_task(self):
        interrupts = {'pause': self.pause_task
            , 'unpause': self.unpause_task
            , 'cancel': self.cancel_task
            , 'toc_pause': self.toc_pause_task
            , 'toc_unpause': self.toc_unpause_task
            , 'toc_cancel': self.toc_cancel_task
            , 'force_cancel_task': self.force_cancel_task
            , 'delete_agent': self.delete_agent
                      }

        rospy.sleep(0.5)  # TODO: find a way to remove this (added for toc_cancel all to process)
        logmsg(category="DTM", msg="Interruption detected!");
        [logmsg(category="DTM", msg="    - %s : %s" % (a.agent_id, a.interruption[0])) for a in
         self.AllAgentsList.values() if a.interruption]
        logmsgbreak(1)

        [interrupts[a.interruption[0]](a) for a in self.AllAgentsList.values() if
         a.interruption and a.interruption[0] in interrupts]
    def pause_task(self, agent):
        logmsg(category="DTM", id=agent.agent_id, msg="Task advancement paused.")
        agent().new_stage = True  # re-enable the _start() call for once unpaused
        agent.task_stage_list.insert(0, StageDef.Pause(agent))
        agent.registration = False  # disable generic query success condition
        if hasattr(agent, 'temp_interface'):
            agent.temp_interface.cancel_execpolicy_goal()  # TODO: how to handle this?
            # TODO: setup an onPause cfunctin similar to oncancel, add all this in there?
        agent.interruption = None  # reset interruption trigger
        self.agent_manager.format_agent_marker(agent.agent_id, style='red')
    def unpause_task(self, agent):
        logmsg(category="DTM", id=agent.agent_id, msg="Task advancement resumed.")
        agent.registration = True  # enable generic query success condition
        agent.interruption = None  # reset interruption trigger
        self.agent_manager.format_agent_marker(agent.agent_id, style='')
    def cancel_task(self, agent, trigger_agent="self", force_release=False):
        logmsg(category="DTM", id=agent.agent_id,
               msg="Cancellation request made for {task:%s} by %s." % (agent['task_id'], trigger_agent))
        logmsg(category="DTM", msg="Cancelling agent and contacts:")

        # Reset agent.interruption so this section is not called again
        module = agent.interruption[1]
        task_id = agent.interruption[2]
        agent.interruption = None

        # Mark robot as inactive to task assignment
        agent.registration = False  # disable generic query success condition

        # Call the on_cancel response function for each agent actively conected to this task
        if module in agent.interfaces:  # TODO: THIS IS BAD, we need to manage base module properly
            for aid, a in [[aid, a] for aid, a in agent.task_contacts.items() if module in agent.interfaces]:  # contacts:
                a.interfaces[module].on_cancel(task_id=task_id, contact_id=agent.agent_id, force_release=force_release)
            agent.interfaces[module].on_cancel(task_id=task_id, contact_id=trigger_agent, force_release=force_release)
            # agent.task_contacts = {}
        else:
            logmsg(level="error", category='DTM', msg="Attempt to cancel base task. Functionality not yet included.")

        self.agent_manager.format_agent_marker(agent.agent_id, style='red')
        logmsgbreak(1)
        pass
    def force_cancel_task(self, agent):
        self.cancel_task(agent, trigger_agent="dfm", force_release=True)
    def delete_agent(self, agent):
        logmsg(level="error", category="DRM", id=agent.agent_id,
               msg="Agent has been removed from the scope of the coordinator.")
        logmsg(level="error", category="DRM", msg="    - Coordinator is no longer recieving location data")
        logmsg(level="error", category="DRM", msg="    - Agent is no longer reserving a node in the network")
        logmsg(level="error", category="DRM", msg="    - Ensure agent is moved away")
        self.agent_manager.agent_details.pop(agent.agent_id)
    def toc_pause_task(self, agent):
        self.pause_task(agent)
    def toc_unpause_task(self, agent):
        self.unpause_task(agent)
    def toc_cancel_task(self, agent):
        self.cancel_task(agent, trigger_agent="toc", force_release=True)
    def add_stages(self, agent, stage_list):
        logmsg(category="DTM", id=agent.agent_id, msg="Adding stages to active task:")
        agent().new_stage = True
        for stage in stage_list:
            logmsg(category="DTM", msg="    - " + str(stage))
            agent.task_stage_list.insert(0, stage)




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
        logmsg(level='warn', category="route", msg="check_route label 1")

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
            publish_route = False  #addume new route is the same

            """ Identify key elements in routes. """
            old_start_edge = old_edge[0]
            old_target_edge = old_node[-1]
            new_start_edge = new_edge[0]
            new_target_edge = new_node[-1]

            """ Do lists have different entrances to the target node? """
            # old: R========T
            # new:          T=====R
            if not publish_route and new_target_edge != old_target_edge:
                logmsg(level='warn', category="route", msg="check_route label 2")
                publish_route = True #route is different

            """ Do lists have different lengths? """
            # old: R========T
            # new:     R====T
            if not publish_route and len(new_edge) != len(old_edge):
                logmsg(level='warn', category="route", msg="check_route label 3")
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

            """ Do same-sized routes differ? """
            # old: ****R====T
            # new:     R=-_=T
            if not publish_route:
                logmsg(level='warn', category="route", msg="check_route label 4")
                for i, e in enumerate(list(zip(*(old_edge,new_edge)))):
                    if e[0] != e[1]:
                        logmsg(category="route", id=agent.agent_id, msg="New route different from existing route")
                        publish_route = True #route is different
                        break

        """ If check_route is false, routes are different """
        if publish_route:
            logmsg(level='warn', category="route", msg="check_route label 5")
            if self.log_routes:
                logmsg(category="rob_py", id=agent.agent_id, msg='New route generated:\n%s' % policy)
                logmsg(category="rob_py", msg='Previous route:\n%s' % agent.temp_interface.execpolicy_goal)

            agent.temp_interface.cancel_execpolicy_goal()
            agent.temp_interface.set_execpolicy_goal(policy)

            agent().route_required = False #ReplanTrigger
            logmsg(level='warn', category="route", id=agent.agent_id, msg="route published")
        agent().route_found = False
        logmsg(level='warn', category="route", id=agent.agent_id, msg="route publish attempt complete")
        rospy.sleep(1)
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
            if a.task_stage_list:
                stages += [a().get_class()]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_new_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a.task_stage_list:
                stages += [a().new_stage]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_task(self):
        return ['', ''] + [a.task_name for a in self.AllAgentsList.values()]

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
                    logmsg(category="log", msg="Updating log")
                    print('------------------------------------------')

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













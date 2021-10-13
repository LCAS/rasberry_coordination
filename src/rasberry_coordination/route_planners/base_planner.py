#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy
import copy
import operator

import strands_navigation_msgs.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils

from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef

from abc import ABCMeta, abstractmethod

class BasePlanner(object):
    __metaclass__ = ABCMeta  # @abstractmethod

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_topo_map = True

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return topological_navigation.tmap_utils.get_node(self.topo_map, node)

    def get_distance_between_adjacent_nodes(self, from_node, to_node):
        """get_distance_between_adjacent_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return topological_navigation.tmap_utils.get_distance_to_node(from_node_obj, to_node_obj)

    def get_edge_distances(self, agent_id):
        """find and fill distances of all edges of a agent's planned route, if at least one edge is there.
        the route must contain the goal_node as the last node in the list.

        Keyword arguments:
            robot_id -- robot_id
        """
        agent = self.agent_details[agent_id]
        agent.route_dists = []
        if len(agent.route_edges) >= 1:
            for i in range (len(agent.route) - 1):
                agent.route_dists.append(self.get_distance_between_adjacent_nodes(agent.route[i], agent.route[i+1]))

    def get_route_distance_to_node(self, agent_id, node_id):
        """get the total distance to a node in a agent's route

        Keyword arguments:
            robot_id -- id of the robot to be checked
            node_id -- node being checked
        """
        dist = 0.0
        adding_ok = False
        agent = self.agent_details[agent_id]
        # robot = self.robot_manager.agent_details[robot_id]
        if len(agent.route_edges) > 1:
            for i in range(len(agent.route)):
                if agent.route[i] == node_id:
                    break
                # add edge_distance only if the source node is not the one we look for
                # also make sure we start adding from current/closest node
                if not adding_ok:
                    adding_ok = agent.route[i] == (agent.current_node or agent.closest_node)
                    # if agent.current_node != None:
                    #     if agent.current_node == agent.route[i]:
                    #         adding_ok = True
                    # elif agent.closest_node != None:
                    #     if agent.closest_node == agent.route[i]:
                    #         adding_ok = True
                if adding_ok:
                    dist += agent.route_dists[i]
        return dist

    def shortest_route_to_node(self, agent_ids, node_id):
        """from a list of robot_ids, find the robot with shortest route distance to a given node
        """
        dists = {}
        for agent_id in agent_ids:
            dists[agent_id] = self.get_route_distance_to_node(agent_id, node_id)

        return sorted(dists.items(), key=operator.itemgetter(1))[0][0]

    def get_agents(self):
        # Filter out agents with no physical presence
        self.agent_details = {a.agent_id: a for a in self.agent_manager.agent_details.values() if a.has_presence}

    def no_route_found(self, agent):
        logmsg(level='error', category='route', id=agent.agent_id, msg='Route not found, executing recovery behaviour:')
        if not 'WaitNode' in str(agent()):
            logmsg(level='error', category='route', msg='    - Adding WaitNode as intermediate target')

            logmsg(category="DTM", id=agent.agent_id, msg="    - Adding stages to active task:")
            agent().new_stage = True
            recovery_stages = [ StageDef.AssignWaitNode(agent), StageDef.NavigateToWaitNode(agent) ]
            recovery_stages.reverse()
            for stage in recovery_stages:
                logmsg(category="DTM", msg="        - " + str(stage))
                agent.task_stage_list.insert(0, stage)

    @abstractmethod
    def __init__(self, agent_manager):
        """ Copy parameters to properties, set update_map callback for agents and initialise map

        Args:
            agent_manager - pointer to coordinator.agent_manager a dictionary of all agent_details objects
        """

        # Filter out agents with no physical presence
        self.agent_manager = agent_manager
        self.agent_details = {a.agent_id: a for a in self.agent_manager.agent_details.values() if a.has_presence}

        """ Download Topological Map """
        self.rec_topo_map = False
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self._map_cb)

        logmsgbreak(total=1)
        logmsg(category="route", msg='Route Planner waiting for Topological map ...')
        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(category="route", msg='Route Planner received Topological map.')
        self.available_topo_map = copy.deepcopy(self.topo_map)  # empty map used to measure routes

        """ Change callback location to modify the FragmentPlanner available topomap """
        for agent in self.agent_details.values():
            agent.cb['update_topo_map'] = self.update_available_topo_map

        """ Setup object to perform route_searching in empty map """
        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

    @abstractmethod
    def find_routes(self):
        self.get_agents()
        pass

    @abstractmethod
    def update_available_topo_map(self, ):
        pass


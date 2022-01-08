#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy
from rospy import Subscriber

import copy
import yaml
import operator

from std_msgs.msg import String
import strands_navigation_msgs.msg
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetNode, get_distance_to_node_tmap2 as GetNodeDist

from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef

from abc import ABCMeta, abstractmethod

class BasePlanner(object):
    __metaclass__ = ABCMeta  # @abstractmethod

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = yaml.safe_load(msg.data)
        self.rec_topo_map = True

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return GetNode(self.topo_map, node)

    def get_row_ends(self, agent, row_id):

        row_nodes = [int(node["node"]["name"].replace("%s-c"%row_id, ''))
                     for node in agent.navigation['tmap']["nodes"]
                     if node["node"]["name"].startswith(row_id)]
        if not row_nodes:
            logmsg(level="error", msg="No row ends found for row_id(%s): %s"%(row_id,row_nodes))

        row_start = "%s-c%s"%(row_id, 0)
        row_end   = "%s-c%s"%(row_id, max(row_nodes))

        return [row_start, row_end]

    def get_rows(self, agent, tunnel_id):
        row_prefix = "%s-r"%tunnel_id

        row_ids = set([int(node["node"]["name"].replace(row_prefix, '').split('-c')[0])
                       for node in agent.navigation['tmap']["nodes"]
                       if node["node"]["name"].startswith(row_prefix)])

        return ["%s%s"%(row_prefix, row_id) for row_id in row_ids]

    def get_distance_between_adjacent_nodes(self, from_node, to_node):
        """get_distance_between_adjacent_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return GetNodeDist(from_node_obj, to_node_obj)

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
                    adding_ok = agent.route[i] == (agent.location.current_node or agent.location.closest_node)
                    # if agent.location.current_node != None:
                    #     if agent.location.current_node == agent.route[i]:
                    #         adding_ok = True
                    # elif agent.location.closest_node != None:
                    #     if agent.location.closest_node == agent.route[i]:
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

    def get_available_optimum_route(self, agent, start_node, goal_node):
        """ find and return a route from start_node to goal_node on the available_tmap
        :param start_node: name of the node from which route should be planned, str
        :param goal_node: name of the node to which route should be planned, str
        :return: route from start_node to goal_node
        """
        print(agent.navigation['available_route_search'])
        print(agent.navigation['available_route_search'].search_route(start_node, goal_node))
        return None or agent.navigation['available_route_search'].search_route(start_node, goal_node)

    def load_route_search(self, agent):
        agent.navigation['available_route_search'] = TopologicalRouteSearch(agent.navigation['tmap_available'])

    # def get_agents(self):
    #     # Filter out agents with no physical presence
    #     self.agent_details = {a.agent_id: a for a in self.agent_manager.agent_details.values() if a.location.has_presence}

    def load_occupied_nodes(self):
        """ get the list of nodes occupied by all agents
        """
        self.occupied_nodes = list(set([a.location(accurate=False) for a in self.agent_manager.agent_details.values() if a.location.has_presence]))

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
                agent['stage_list'].insert(0, stage)

    @abstractmethod
    def __init__(self, agent_manager, heterogeneous_map):
        """ Copy parameters to properties, set update_map callback for agents and initialise map

        Args:
            agent_manager - pointer to coordinator.agent_manager a dictionary of all agent_details objects
        """
        logmsg(category="route", id="PLANNER", msg='Route Planner Initialisation')

        # Filter out agents with no physical presence
        self.agent_manager = agent_manager
        self.agent_details = {a.agent_id: a for a in self.agent_manager.agent_details.values() if a.location.has_presence}

        """ Download Topological Map """
        self.heterogeneous_map = heterogeneous_map
        self.rec_topo_map = False
        Subscriber("topological_map_2", String, self._map_cb)

        logmsg(category="route", msg='    | awaiting topomap')
        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(category="route", msg='    | received topomap')
        self.available_topo_map = copy.deepcopy(self.topo_map)  # empty map used to measure routes

        """ Change callback location to modify the FragmentPlanner available topomap """
        for agent in self.agent_details.values():
            agent.cb['update_topo_map'] = self.update_available_topo_map

        """ Setup object to perform route_searching in empty map """
        self.route_search = TopologicalRouteSearch(self.topo_map)

    @abstractmethod
    def find_routes(self):
        self.agent_details = {a.agent_id: a for a in self.agent_manager.agent_details.values() if a.location.has_presence}
        for agent in self.agent_details.values(): agent.cb['update_topo_map'] = self.update_available_topo_map
        pass

    def update_available_topo_map(self, agent):
        pass


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
from rasberry_coordination.task_management.__init__ import Stages

from abc import ABCMeta, abstractmethod

class BasePlanner(object):
    __metaclass__ = ABCMeta  # @abstractmethod

    def get_row_ends(self, agent, row_id):
        return ["%s-ca" % row_id, "%s-cz" % row_id]

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

    def load_occupied_nodes(self):
        """ get the list of all nodes occupied by agents """
        occ = [a.location(accurate=False) for a in self.agent_manager.agent_details.values() if a.location.has_presence]
        self.occupied_nodes = list(set(occ))
        o={a.agent_id: [a.location(accurate=False), a.location.has_presence] for a in self.agent_manager.agent_details.values() if a.location.has_presence}
        logmsg(category="route", msg="Occupied Nodes: %s"%str(o))

    def no_route_found(self, agent):
        logmsg(level='warn', category='route', id=agent.agent_id, msg='Route not found, executing recovery behaviour:')
        if not 'WaitNode' in str(agent()):
            logmsg(level='warn', category='route', msg='    - Adding WaitNode as intermediate target')
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
        self.occupied_nodes = None
        self.heterogeneous_map = heterogeneous_map

    @abstractmethod
    def find_routes(self):
        self.agent_details = {a.agent_id: a for a in self.agent_manager.agent_details.values() if a.location.has_presence}
        pass

    def update_available_topo_map(self, agent):
        pass


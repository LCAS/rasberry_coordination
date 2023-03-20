#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import copy
import yaml
import operator

from std_msgs.msg import String
import strands_navigation_msgs.msg

from rasberry_coordination_core.utils.logmsg import logmsg, logmsgbreak
from rasberry_coordination_core.task_management.__init__ import Stages

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

    def load_occupied_nodes(self, ret=False):
        """ get the list of all nodes occupied by agents """
        logmsg(category="occupy", msg="Occupation:")

        occ = {a.agent_id: a.modules['navigation'].interface.occupation()
               for a in self.agent_manager.agent_details.values()
               if 'navigation' in a.modules}

        if ret: return occ
        self.occupied_nodes = list(set(sum(occ.values(),[])))

    def no_route_found(self, agent):
        """ process to follow if a route is not found/available """
        logmsg(category='xroute', id=agent.agent_id, msg='Route not found, executing recovery behaviour:')

        # If we are currently executing a wait_node response, do nothing
        #if 'WaitNode' in str(agent()): #TODO: this is where the problem is.... ffs, stages dont have WaitNode int hem anymore?!!?!?
        if 'AssignNode' in str(agent()) or 'NavigateToNode' in str(agent()):
            if agent().association == 'recovery-node_contact_id':
                logmsg(category='xroute', msg='    - recovery behaviour is already active')
                return

        # Set current stage as inactive
        agent().new_stage = True

        # Construct and add WaitNode stages to the active task
        logmsg(category="xroute", msg="    - Adding WaitNode stages to active task:")
        contact = 'recovery_node_contact_id'
        agent['stage_list'] = [
            Stages['assignment']['AssignNode'](agent, contact_id=contact, node_descriptor='wait_node'),
            Stages['navigation']['NavigateToNode'](agent, contact_id=contact)
        ] + agent['stage_list']

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


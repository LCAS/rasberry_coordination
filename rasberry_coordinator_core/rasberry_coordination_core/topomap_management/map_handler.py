# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

# Builtins
from copy import copy, deepcopy
from time import time
import yaml

# Messages
from std_msgs.msg import Bool, String as Str, Empty as Emp
from rasberry_coordination_msgs.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption

# Components
from topological_navigation.route_search2 import TopologicalRouteSearch2
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetNode, get_distance_to_node_tmap2 as GetNodeDist

# ROS2
from rasberry_coordination_core.node import GlobalNode
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg


class MapObj(object):
    """
    Uses:
    - instantiated by Agent
    - .map checked in WaitForMap
    -
    """
    def __init__(self, agent, topic=None):
        self.agent = agent
        self.topic = topic or "/topological_map_2"

        self.raw_msg = None

        # used for sharing occupancy
        self.global_map = None
        self.global_node_list = None

        # used for planning direct routes
        self.empty_map = None
        self.empty_route_search = None
        self.empty_node_list = None

        # used for planning in cluttered workspace
        self.filtered_map = None
        self.filtered_route_search = None
        self.filtered_node_list = None


    def enable_map_monitoring(self):
        # callback are enabled in base.StageDef.WaitForMap._start()
        print('enablin')
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_ALL,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_tmap_sub = GlobalNode.create_subscription(Str, '/topological_map_2', self.global_map_cb, qos)
        self.local_tmap_sub = GlobalNode.create_subscription(Str, self.topic, self.local_map_cb, qos)
        print('subs started!')

    def global_map_cb(self, msg):
        # This is included for each agent as a single global map is needed for an agent to
        # find their neighbouring nodes. In theory, we currently are loading a global map
        # for every agent, we could instead have a single central one to reference.
        # used for sharing occupancy
        print('recieving global')
        t0 = time()

        t1 = time()-t0
        self.global_map = self.load_raw_tmap(msg.data)
        t2 = time()-t0
        self.global_node_list = [node["node"]["name"] for node in self.global_map['nodes']]
        t3 = time()-t0

        tim = (round(t2-t1,2), round(t3-t2,2))
        logmsg(category="TEST", id=self.agent.agent_id, msg="global(%s|%s)"%tim)

    def local_map_cb(self, msg):
        print('recieving local')
        t0 = time()

        # Save copy of message
        t1 = time()-t0
        self.raw_msg = msg.data
        t2 = time()-t0

        # used for planning direct routes
        self.empty_map = self.load_raw_tmap(self.raw_msg)
        t3 = time()-t0
        self.empty_route_search = TopologicalRouteSearch2(self.empty_map)
        t4 = time()-t0
        self.empty_node_list = [node["node"]["name"] for node in self.empty_map['nodes']]
        t5 = time()-t0

        # used for planning in cluttered workspace
        self.filtered_map = deepcopy(self.empty_map)
        #self.filtered_map = self.load_raw_tmap(self.raw_msg)
        t6 = time()-t0
        self.filtered_route_search = TopologicalRouteSearch2(self.filtered_map)
        t7 = time()-t0
        self.filtered_node_list = copy(self.empty_node_list)
        #self.filtered_node_list = [node["node"]["name"] for node in self.filtered_map['nodes']]
        t8 = time()-t0

        # Log timings
        tim = tuple([round(t,2) for t in [t2-t1, t3-t2, t4-t3, t5-t4, t6-t5, t7-t6, t8-t7]])
        logmsg(category="TEST", id=self.agent.agent_id, msg="raw(%s) | empty(%s|%s|%s) | filt(%s|%s|%s)"%tim)

    def start_map_reset(self):
        self.filtered_map = deepcopy(self.empty_map)
        # self.filtered_map = self.load_raw_tmap(self.raw_msg)

    def complete_map_reset(self):
        self.filtered_route_search = TopologicalRouteSearch2(self.filtered_map)
        self.filtered_node_list = [node["node"]["name"] for node in self.filtered_map['nodes']]

    def load_raw_tmap(self, data):
        return yaml.safe_load(data)

    def is_node_restricted(self, node_id):
        """check if given node is in agent's map"""
        if 'restrictions' in self.modules['navigation'].details:
            return (self.empty_node_list and node_id in self.empty_node_list)
        return True

    def simplify(self):
        R = {n.split('-')[0][1:]: {} for n in self.empty_node_list if n.startswith('r') and "-c" in n}
        tall = {}
        short = {}
        for k, v in R.items():
            if '.' in k:
                short[k] = [n.split('-')[1][1:] for n in self.empty_node_list if n.split('-')[0] == ('r' + k ) and '.' in n]
            else:
                tall[k] = [n.split('-')[1][1:] for n in self.empty_node_list if  n.split('-')[0] == ('r' + k) and '.' not in n]
        Map = { 'tall' : tall , 'short' : short }
        return Str(str(Map))


    """ The following are map query tools """
    def is_node(self, node):
        """get node by name"""
        return (node in self.empty_node_list)

    def get_node(self, node):
        """get node by name"""
        return GetNode(self.empty_map, node)

    def get_edge_length(self, from_node, to_node):
        """ get length of edge """
        return GetNodeDist(self.get_node(from_node), self.get_node(to_node))

    def get_edge_distances(self):
        """find edge lengths of route """
        self.agent.route_dists = []
        if not self.agent.route_edges: return
        return [self.get_edge_length(self.agent.route[i], self.agent.route[i+1]) for i in range(len(self.agent.route) - 1)]

    def get_route_length(self, agent, start_node, goal_node):
        """ get length of direct route between nodes """
        if start_node == goal_node: return 0
        route = self.empty_route_search.search_route(start_node, goal_node)
        if route is None: return float("inf")

        route_nodes = route.source
        route_nodes.append(goal_node)

        route_distance = []
        for i in range(len(route_nodes) - 1):
            route_distance.append(self.get_edge_length(route_nodes[i], route_nodes[i + 1]))

        return sum(route_distance)

    def get_node_pose(self, node):
        p, node = Pose(), self.get_node(node)['node']['pose']
        p.position.x = node['position'][0]
        p.position.y = node['position'][1]
        p.position.z = node['position'][2]
        p.orientation.x = node['orientation'][0]
        p.orientation.y = node['orientation'][1]
        p.orientation.z = node['orientation'][2]
        p.orientation.w = node['orientation'][3]
        return p

    def get_node_tf(self, node):
        node = self.get_node(node)['node']['pose']
        pos, ori = node['position'], node['orientation']
        return ((pos['x'], pos['y'], pos['z']), (ori['x'], ori['y'], ori['z'], ori['w']))

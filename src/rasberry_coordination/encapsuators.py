from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy

from std_msgs.msg import Bool, String as Str, Empty as Emp
import strands_executive_msgs.msg

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption

import yaml
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetNode, get_distance_to_node_tmap2 as GetNodeDist
from topological_navigation_msgs.msg import ClosestEdges

class LocationObj(object):

    def __init__(self, agent, has_presence = True, initial_location = None):
        self.agent = agent
        self.has_presence = has_presence
        self.current_node = initial_location
        self.previous_node = None
        self.closest_node = None
        self.closest_edge = None

    def enable_location_monitoring(self, agent_id):
        # callback are enabled in base.StageDef.WaitForLocalisation._start()
        self.current_node_sub = Subscriber('/%s/current_node'    % agent_id, Str, self.current_node_cb)
        self.closest_node_sub = Subscriber('/%s/closest_node'    % agent_id, Str, self.closest_node_cb)
        self.closest_node_sub = Subscriber('/%s/closest_edges'    % agent_id, ClosestEdges, self.closest_edges_cb)
        self.disable_loc = Subscriber('/%s/localisation/disable' % agent_id, Str, self.disable_localisation)
        self.enable_loc  = Subscriber('/%s/localisation/enable'  % agent_id, Emp, self.enable_localisation)

    def __call__(self, accurate=False):
        if accurate:
            return self.current_node or self.previous_node or self.closest_node
        return self.current_node or self.closest_node or self.previous_node

    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data

    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data

    def closest_edges_cb(self, msg):
        self.closest_edge = msg.edge_ids[msg.distances.index(min(msg.distances))]
        #print(self.closest_edge)

    def disable_localisation(self, msg):
        if True: #self.agent.map.is_node(msg): msg in self.agent.empty_node_list!??!?!
            self.current_node_sub.unregister()
            self.closest_node_sub.unregister()
            self.current_node_cb(msg)
            self.closest_node_cb(msg)
        else:
            logmsg(level='warn', agent=self.agent.agent_id, msg="canot fake localisation to node: %s" % str(msg))

    def enable_localisation(self, msg):
        self.previous_node, self.current_node, self.closest_node = None, None, None
        self.current_node_sub = Subscriber(self.picker_id + "/current_node", Str, self.current_node_cb)
        self.closest_node_sub = Subscriber(self.picker_id + "/closest_node", Str, self.closest_node_cb)


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
        self.tmap_sub = Subscriber(self.topic, Str, self.map_cb, queue_size=5)

    def map_cb(self, msg):
        self.raw_msg = msg.data

        # used for planning direct routes
        self.empty_map = self.load_raw_tmap(self.raw_msg)
        self.empty_route_search = TopologicalRouteSearch(self.empty_map)
        self.empty_node_list = [node["node"]["name"] for node in self.empty_map['nodes']]

        # used for planning in cluttered workspace
        self.filtered_map = self.load_raw_tmap(self.raw_msg)
        self.filtered_route_search = TopologicalRouteSearch(self.filtered_map)
        self.filtered_node_list = [node["node"]["name"] for node in self.filtered_map['nodes']]

    def start_map_reset(self):
        self.filtered_map = deepcopy(self.empty_map)
        # self.filtered_map = self.load_raw_tmap(self.raw_msg)

    def complete_map_reset(self):
        self.filtered_route_search = TopologicalRouteSearch(self.filtered_map)
        self.filtered_node_list = [node["node"]["name"] for node in self.filtered_map['nodes']]

    def load_raw_tmap(self, data):
        return yaml.safe_load(data)

    def is_node_restricted(self, node_id):
        """check if given node is in agent's map"""
        if 'restrictions' in self.modules['navigation'].details:
            return (self.empty_node_list and node_id in self.empty_node_list)
        return True

    def simplify(self):
        R = {n.split('-')[0][1:]: {} for n in self.empty_node_list if "-" in n and "WayPoint" not in n and "dock" not in n}
        tall = {}
	short = {}
	for k, v in R.items():
            if '.' in k:
                short[k] = [n.split('-')[1][1:] for n in self.empty_node_list if n.split('-')[0] == ('r' + k ) and '.' in n]
            else:
                tall[k] = [n.split('-')[1][1:] for n in self.empty_node_list if  n.split('-')[0] == ('r' + k) and '.' not in n ]
 
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


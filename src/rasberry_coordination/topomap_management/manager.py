
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
        # Reset the map to default, filtering it as needed
        self.filtered_map = deepcopy(self.empty_map)



    def complete_map_reset(self):
        self.filtered_route_search = TopologicalRouteSearch(self.filtered_map)
        self.filtered_node_list = [node["node"]["name"] for node in self.filtered_map['nodes']]

    def load_raw_tmap(self, data):
        return yaml.safe_load(data)

    def is_node_restricted(self, node_id):
        """check if given node is in agent's map"""
        if 'restrictions' in self.agent.navigation_properties:
            return (self.empty_node_list and node_id in self.empty_node_list)
        return True

    def simplify(self):
        R = {n.split('-')[0][1:]: {} for n in self.empty_node_list if "-" in n and "WayPoint" not in n and "dock" not in n}
        for r in R:
            R[r] = [n.split('-')[1][1:] for n in self.empty_node_list if n.startswith('r' + r)]
        Map = {'tall':  {k:v for k,v in R.items() if '.' not in k},
               'short': {k:v for k,v in R.items() if '.' in k} }
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

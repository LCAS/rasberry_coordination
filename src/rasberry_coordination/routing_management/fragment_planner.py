#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------


import threading
from pprint import pprint

from rasberry_coordination.routing_management.base_planner import BasePlanner
from rasberry_coordination.coordinator_tools import logmsg


class FragmentPlanner(BasePlanner):
    def __init__(self, all_agent_details_pointer, heterogeneous_map):
        """ Copy parameters to properties

        Args:
            all_agent_details_pointer - pointer to coordinator.all_agents_list a dictionary of all agent_details objects
        """
        super(FragmentPlanner, self).__init__(all_agent_details_pointer, heterogeneous_map)
        self.task_lock = threading.Lock()

    def critical_points(self, ):
        """find points where agent's path cross with those of active robots.
        also find active robots which cross paths at these critical points.
        """
        active_agents = [agent.agent_id for agent in self.agent_details.values() if agent.goal()]
        critical_points = {}  # {[route: critical point]} each route which contains a critical point
        critical_agents = {}  # {[critical_point: robot_ids]} all robots touching a critical point

        """for each agent"""
        for agent in self.agent_details.values():
            agent_id = agent.agent_id
            r_outer = agent.route
            critical_points[str(r_outer)] = set([])

            """for each active robot excluding the agent"""
            for robot_id in active_agents:
                if robot_id == agent_id:
                    continue

                robot = self.agent_details[robot_id]

                """if the agent route is not the same as the robot route"""
                r_inner = robot.route
                if r_outer is not r_inner:

                    """add any new critical points to the agents route"""
                    critical_points[str(r_outer)] = \
                        critical_points[str(r_outer)].union(set(r_outer).intersection(set(r_inner)))
                    #TODO: try to simplify this

                    """for each node in the combined set of conflicts"""
                    for conflicted_node_id in set(r_outer).intersection(set(r_inner)):
                        #critical_robots['Waypoint106'] <- [thorvald_001, thorvald_002]

                        """add the active robot to the list of agents involved in the conflict"""
                        if conflicted_node_id not in critical_agents:
                            critical_agents[conflicted_node_id] = [robot_id]
                        elif robot_id not in critical_agents[conflicted_node_id]:
                            critical_agents[conflicted_node_id].append(robot_id)

                        """add the active agent to the list of agents involved in the conflict"""
                        if agent_id in active_agents and agent_id not in critical_agents[conflicted_node_id]:
                            critical_agents[conflicted_node_id].append(agent_id)

        return (critical_points, critical_agents)

    def split_critical_paths(self, ):
        """split robot paths at critical points
        """

        """identify critical points for each route and the robots which are involved"""
        c_points, c_agents = self.critical_points()

        """ remove goal node as critical for robots heading to picker """
        active_agents = [agent.agent_id for agent in self.agent_details.values() if agent.route_edges]
        for agent_id in active_agents:
            agent = self.agent_details[agent_id]
            goal = agent.goal()
            if (goal and goal in c_points[str(agent.route)]):
                c_points[str(agent.route)].remove(goal)

        allowed_cpoints = []  #
        res_routes = {}  #

        """ for each agent populate res_routes with partial routes"""
        for agent in self.agent_details.values():
            agent_id = agent.agent_id
            allowed_to_pass = False
            collective_route = []
            partial_route = []

            """ for each node in the route """
            for node in agent.route:

                """ if node is a critical point in the route """
                if node in c_points[str(agent.route)]:

                    """identify robot closest to the node"""
                    nearest_agent = self.shortest_route_to_node(c_agents[node], node)

                    """
                    each critical vertice can be given to 1 robot thus we give it to the closest robot and
                    prevent it being taken again by adding it to allowed_cpoints,
                    """
                    if (agent_id == nearest_agent and node not in allowed_cpoints):
                        """ if vertice is unassigned, and is best assigned to this robot, assign it so"""
                        """also enable the chosen robot to take the remaining nodes using allowed_to_pass"""
                        partial_route.append(node)
                        allowed_cpoints.append(node)
                        allowed_to_pass = True

                    elif node not in allowed_cpoints and allowed_to_pass:
                        """ if vertice is unassigned, robot has been given permission to take the rest """
                        partial_route.append(node)
                        allowed_cpoints.append(node)

                    else:
                        """ if robot is not the nearest or the robot has not been given permission to take the rest """
                        """ if the robot has an existing route, save it to the collective and reset partial route """
                        if partial_route:
                            collective_route.append(partial_route)
                        partial_route = [node]

                else:
                    """ series of critical nodes has finished, add partial route """
                    partial_route.append(node)

            """ if a partial route exists, append it to the rr"""
            if partial_route:
                collective_route.append(partial_route)
            res_routes[agent_id] = collective_route

        """ for each agent, apply their route fragments """
        for agent in self.agent_details.values():
            if agent.agent_id in res_routes:
                agent.route_fragments = res_routes[agent.agent_id]

        logmsg(category="route", msg="    - All fragments identified")
        for a in self.agent_details.values():
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id,str(a.route_fragments).replace('WayPoint','wp')))

        res_edges = {}
        # split the edges as per the route_fragments
        """ for each active robot """
        for agent_id in active_agents:
            agent = self.agent_details[agent_id]

            """ if the robot has route fragments """
            if agent.route_fragments:

                """ remove goal node from final fragment """
                agent.route_fragments[-1].pop(-1)

                # if start and goal nodes are different, there will be at least one node remaining and an edge
                """ move the last node of all fragments to the start of next fragment """
                for i in range(len(agent.route_fragments) - 1):
                    agent.route_fragments[i+1].insert(0, agent.route_fragments[i][-1])
                    agent.route_fragments[i].pop(-1)

                """ split the edges """
                res_edges[agent_id] = []
                for i in range(len(agent.route_fragments)):
                    res_edges[agent_id].append(agent.route_edges[:len(agent.route_fragments[i])])
                    agent.route_edges = agent.route_edges[len(agent.route_fragments[i]):]
            else:
                agent.route_fragments = []
                res_edges[agent_id] = []

        logmsg(category="route", msg="    - All fragments formatted")
        for a in self.agent_details.values():
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id,str(a.route_fragments).replace('WayPoint','wp')))

        """ for each agent, apply their route edges """
        # self.route_edges = res_edges
        for agent in self.agent_details.values():
            if agent.agent_id in res_edges:
                agent.route_edges = res_edges[agent.agent_id]

        logmsg(category="route", msg="    - All fragment edges formatted")
        for a in self.agent_details.values():
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id,str(a.route_edges).replace('WayPoint','wp')))

    def find_routes(self, ):
        """find_routes - find indiviual paths, find critical points in these paths, and fragment the
        paths at critical points - whenever triggered
        """
        super(FragmentPlanner, self).find_routes()

        logmsg(category="route", id="COORDINATOR", msg="Finding routes for Active agents:")
        [logmsg(category="route", msg="    | %s: %s"%(a.agent_id, a.goal())) for a in self.agent_details.values()]

        # agents with an active nav goal (navigation)
        actives = [a for a in self.agent_details.values() if a.goal()]
        logmsg(category="route", msg="actives --- "+str([a.agent_id for a in actives]))

        # agents without an active goal (idle)
        inactives = [a for a in self.agent_details.values() if not a.goal()]
        logmsg(category="route", msg="inactives - "+str([a.agent_id for a in inactives]))

        # agents which require a route for thier active stage
        need_route = [a for a in self.agent_details.values() if a().route_required]
        logmsg(category="route", msg="Agents requiring routes:")
        for a in need_route: logmsg(category="route", msg="    | {%s: %s}" % (a.agent_id, a()))

        # identify all occupied nodes
        self.load_occupied_nodes()

        # find unblocked routes for all agents which need one
        for agent in actives:
            agent_id = agent.agent_id
            agent().route_found = False

            # get start node and goal node
            start_node = agent.location(accurate=False)
            goal_node  = agent.goal()
            logmsg(category="route", id=agent.agent_id, msg="Finding route for %s: %s -> %s" % (agent_id, start_node, goal_node))

            # if current node is goal node, mark agent as inactive
            if start_node == goal_node: #should _query should have handled this by this point?
                inactives += [agent]
                logmsg(category="route", msg="Agent is at goal_node, adding to inactives")
                continue

            # unblock start and goal nodes, then update map to block other agents
            FragmentPlanner_map_filter.generate_filtered_map(agent, start_node, goal_node, self.occupied_nodes)
            #agent.map_manager.filtering.filter(filter_list=["only_restrictions", "no_nodes_with_presence", "allow_start_and_goal"])

            # generate route from start node to goal node
            route = agent.map_handler.filtered_route_search.search_route(start_node, goal_node)

            # if failed to find route, set robot as inactive and mark navigation as failed
            if route.source == [] and route.edge_id == []:
                logmsg(level="warn", category="route", msg="failed to find route, waiting idle")
                logmsg(level="warn", category="route", msg="modify here for wait_node addition")
                self.no_route_found(agent)
                inactives += [agent]
                agent().route_required = False
                continue

            # add goal_node as it could be a critical point
            route_nodes = route.source + [goal_node]
            route_edges = route.edge_id

            # TODO: is thie needed anymore?
            agent.no_route_found_notification = True

            # save route details
            agent.route = route_nodes
            agent.route_edges = route_edges
            agent.route_dists = agent.map_handler.get_edge_distances()

            # mark route as found
            agent().route_found = True
            logmsg(category="route", msg="Route has been found, marking as such")

        # secure locations for each inactive agent, to make routing not interfere
        for agent in inactives:
            agent.route = [agent.location(accurate=True)]
            agent.route_edges = []
            agent.route_dists = agent.map_handler.get_edge_distances()

        # log each route
        logmsg(category="route", msg="    - All agents assigned routes")
        for a in self.agent_details.values():
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id, str(a.route).replace('WayPoint','wp')))

        # find critical points and fragment routes to avoid critical point collisions
        self.split_critical_paths()


class FragmentPlanner_map_filter(object):

    @classmethod
    def generate_filtered_map(cls, agent, start_node, goal_node, occupied_nodes):
        agent.map_handler.start_map_reset()

        #Block access to nodes occupied by other agents
        cls.block_nodes(agent, occupied_nodes)
        cls.unblock_node(agent, start_node)
        cls.unblock_node(agent, goal_node) # not strictly necesary as pickers and storage agents do not use presence and thus will not be in occupied_nodes

        #Block access to neighbouring rows
        # cls.block_rows(agent, occupied_nodes)
        # cls.unblock_parent_row(agent, start_node)
        # cls.unblock_parent_row(agent, goal_node)

        agent.map_handler.complete_map_reset()

    @classmethod
    def block_nodes(cls, agent, occupied_nodes):
        """remove incoming edges to the list of agent nodes in the available_tmap
        and update the available_route_search object with the new map
        :param agent_nodes: list of nodes occupied by other agents, list
        """
        # # Nothing to do if restrictions are not used
        # if 'restrictions' not in agent.navigation_properties: return
        ocn = occupied_nodes

        for node in agent.map_handler.filtered_map["nodes"]:

            # Remove any edges which go into an occupied node
            node["node"]["edges"] = [e for e in node["node"]["edges"] if e["node"] not in ocn]

            # Remove any edges leaving an occupied node (this will fail unless unblock_node is updated to match)
            #if node["node"]["name"] in ocn: node["node"]["edges"] = []

    @classmethod
    def unblock_node(cls, agent, node_to_unblock):
        """ unblock a node by adding details from unfiltered map """
        nodes_to_append = []
        edges_to_append = []

        # for each edge in network, if edge connects to a node to unblock, add to list
        for node in agent.map_handler.empty_map["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["node"] == node_to_unblock:
                    nodes_to_append.append(node["node"]["name"])
                    edges_to_append.append(edge)

        # for each node in empty map, if node is to be unblocked, add a extra edge
        for node in agent.map_handler.filtered_map["nodes"]:
            if node["node"]["name"] in nodes_to_append:
                ind_to_append = nodes_to_append.index(node["node"]["name"])
                node["node"]["edges"].append(edges_to_append[ind_to_append])

    @classmethod
    def block_rows(cls, agent, occupied_nodes):
        print("blocking %s for %s"%(str(occupied_nodes), agent.agent_id))
        """ block access to each row if the row is occupied """
        #This only works on the assumption that small rows are staggered left of the tall rows
        for node in occupied_nodes:
            name = node["name"]
            if name.startswith('tall_t'):
                #lock: [tall, short, TALL, short, tall]
                _,t,r,_ = name.split('_')
                #lock tall rows:
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])+1 ))
                #lock short rows:
                cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
                cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])+1 ))
                pass
            elif name.startswith('small_t'):
                #lock: [tall, SHORT, tall]
                _,t,r,_ = name.split('_')
                #lock short rows:
                cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
                #lock tall rows:
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
                pass
            elif name.startswith('r'):
                #_,t,r,_ = name.split('_')
                ##lock short rows:
                #cls.block_row_(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
                ##lock tall rows:
                #cls.block_node(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
                #cls.block_node(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
                pass
            else:
                pass

    @classmethod
    def block_row_ends(cls, agent, row_name='small_t1_r2'):
        #Identify all nodes which are related to the row_name given
        nodes_of_interest = [int(n.split('_c')[1]) for n in agent.map_handler.empty_node_list if n.startswith(row_name)]
        if not nodes_of_interest: return

        #Identify the full names of the ends of the row
        row_start = '%s_c%i' % (row_start_id, min(nodes_of_interest))
        row_end = '%s_c%i' % (row_start_id, max(nodes_of_interest))

        #Block the ends of the rows being traversed
        cls.block_nodes(agent, [row_start, row_end])

    @classmethod
    def unblock_parent_row(cls, agent, node_name):
        #Exit early if the node is not in a row
        if node_name.split('_')[0] not in ['tall_t', 'small_t']: return

        #Identify the parent row
        row_name = node_name.split('_c')[0]
        
        #Identify all nodes which are related to the row_name given
        nodes_of_interest = [int(n.split('_c')[1]) for n in agent.map_handler.empty_node_list if n.startswith(row_name)]
        if not nodes_of_interest: return

        #Identify the full names of the ends of the row
        row_start = '%s_c%i' % (row_start_id, min(nodes_of_interest))
        row_end = '%s_c%i' % (row_start_id, max(nodes_of_interest))

        #Block the ends of the rows being traversed
        cls.unblock_node(agent, row_start)
        cls.unblock_node(agent, row_end)


















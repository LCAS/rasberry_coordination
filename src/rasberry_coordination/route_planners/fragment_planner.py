#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy
import copy
import threading

import topological_navigation.route_search

from rasberry_coordination.route_planners.base_planner import BasePlanner
from rasberry_coordination.coordinator_tools import logmsg

class FragmentPlanner(BasePlanner):
    def __init__(self, all_agent_details_pointer):
        """ Copy parameters to properties

        Args:
            all_agent_details_pointer - pointer to coordinator.all_agents_list a dictionary of all agent_details objects
        """
        super(FragmentPlanner, self).__init__(all_agent_details_pointer)
        self.task_lock = threading.Lock()

    def update_available_topo_map(self, ):
        """This function updates the available_topological_map, which is topological_map
        without the edges going into the nodes occupied by the agents. When current node
        of an agent is none, the closest node of the agent is taken.
        """
        agent_nodes = [agent.location(accurate=True) for agent in self.agent_details.values() if agent.has_presence]
        topo_map = copy.deepcopy(self.topo_map)
        for node in topo_map.nodes:
            to_pop = []
            for i in range(len(node.edges)):
                if node.edges[i].node in agent_nodes:
                    to_pop.append(i)
            if to_pop:
                to_pop.reverse()
                for j in to_pop:
                    node.edges.pop(j)

        self.available_topo_map = topo_map

    def unblock_node(self, available_topo_map, node_to_unblock):
        """ unblock a node by adding edges to an occupied node in available_topo_map
        """
        nodes_to_append=[]
        edges_to_append=[]

        """ for each edge in network, if edge connects to a node to unblock, add to list """
        for node in self.topo_map.nodes:
            for edge in node.edges:
                if edge.node == node_to_unblock:
                    nodes_to_append.append(node.name)
                    edges_to_append.append(edge)

        """ for each node in empty map, if node is to be unblocked, add a extra edge """
        for node in available_topo_map.nodes:
            if node.name in nodes_to_append:
                ind_to_append = nodes_to_append.index(node.name)
                node.edges.append(edges_to_append[ind_to_append])

        return available_topo_map

    def critical_points(self, ):
        """find points where agent's path cross with those of active robots.
        also find active robots which cross paths at these critical points.
        """
        active_agents = [agent.agent_id for agent in self.agent_details.values() if agent.goal()]
        critical_points = {}  # {[route: critical point]} each route which contains a critical point
        critical_agents = {}  # {[critical_point: robot_ids]} all robots touching a critical point

        """for each agent"""
        for agent in self.agent_details.values():
        # for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            agent_id = agent.agent_id
            r_outer = agent.route
            critical_points[str(r_outer)] = set([])

            """for each active robot excluding the agent"""
            for robot_id in active_agents:
                if robot_id == agent_id:
                    continue

                robot = self.agent_details[robot_id]
                # robot = self.robot_manager.agent_details[robot_id]

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
            agent_id = agent.agent_id #TODO: test swapping this and above for performance gains
            allowed_to_pass = False
            r = agent.route
            collective_route = []
            partial_route = []

            """ for each node in the route """
            for v in r:

                """ if node is a critical point in the route """
                if v in c_points[str(r)]:

                    """identify robot closest to the node"""
                    nearest_agent = self.shortest_route_to_node(c_agents[v], v)

                    """
                    each critical vertice can be given to 1 robot
                    thus we give it to the closest robot and
                    prevent it being taken again by adding it to
                    allowed_cpoints,
                    """
                    if (agent_id == nearest_agent and v not in allowed_cpoints):
                        """ if vertice is unassigned, and is best assigned to this robot, assign it so"""
                        """also enable the chosen robot to take the remaining nodes using allowed_to_pass"""
                        partial_route.append(v)
                        allowed_cpoints.append(v)
                        allowed_to_pass = True

                    elif v not in allowed_cpoints and allowed_to_pass:
                        """ if vertice is unassigned, robot has been given permission to take the rest """
                        partial_route.append(v)
                        allowed_cpoints.append(v)

                    else:
                        """ if robot is not the nearest or the robot has not been given permission to take the rest """
                        """ if the robot has an existing route, save it to the collective and reset partial route """
                        if partial_route:
                            collective_route.append(partial_route)
                        partial_route = [v]

                else:
                    """ series of critical nodes has finished, add partial route """
                    partial_route.append(v)

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
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id,a.route_fragments))

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
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id,a.route_fragments))

        """ for each agent, apply their route edges """
        # self.route_edges = res_edges
        for agent in self.agent_details.values():
            if agent.agent_id in res_edges:
                agent.route_edges = res_edges[agent.agent_id]

        logmsg(category="route", msg="    - All fragment edges formatted")
        for a in self.agent_details.values():
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id,a.route_edges))

    def find_routes(self, ):
        """find_routes - find indiviual paths, find critical points in these paths, and fragment the
        paths at critical points - whenever triggered
        """
        super(FragmentPlanner, self).find_routes()

        logmsg(category="route", id="routing", msg="Finding routes for Active agents")
        [logmsg(category="route", msg="    - %s:%s"%(a.agent_id, a.goal())) for a in self.agent_details.values()]
        actives =   [a for a in self.agent_details.values() if a.goal()]
        inactives = [a for a in self.agent_details.values() if not a.goal()]
        logmsg(category="route", msg="actives --- "+str([a.agent_id for a in actives]))
        logmsg(category="route", msg="inactives - "+str([a.agent_id for a in inactives]))

        need_route = [a for a in self.agent_details.values() if a().route_required]
        logmsg(category="route", msg="requires route:")
        for a in need_route:
            logmsg(category="route", msg="    - {%s: %s}" % (a.agent_id, a.task_stage_list))

        """find unblocked routes for all agents which need one"""
        for agent in actives:
            agent_id = agent.agent_id
            agent().route_found = False

            """get start node and goal node"""
            start_node = agent.location(accurate=False) #?
            goal_node = agent.goal()

            logmsg(category="route", msg="Finding route for %s: %s -> %s" % (agent_id, start_node, goal_node))

            """if current node is goal node, generate empty route""" #task progression is not to be handled here
            if start_node == goal_node: #should _query should have handled this by this point?
                # this is a moving robot, so must be in a go_to_task stage (picker, storage or base)

                # reset routes and route_edges
                # agent.route = [start_node]
                # agent.route_edges = []
                # self.get_edge_distances(agent_id)
                inactives += [agent]
                continue

            """take copy of empty map"""
            # Each agent has a callback which disconnects their location within the available_topo_map
            # This next portion of code makes a copy of this disconnected map, then opens up the connections from their
            # current location, and to their target location.
            # This is done to prevent robots from routing over one another.
            avail_topo_map = copy.deepcopy(self.available_topo_map)
            avail_topo_map = self.unblock_node(avail_topo_map, start_node)

            """generate route from start node to goal node"""
            avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
            route = None
            if start_node and goal_node:
                route = avail_route_search.search_route(start_node, goal_node)
            route_nodes = []
            route_edges = []


            # moving to wait_node is a different task stage... should this be handled differently?
            """ If failed to find route, set robot as inactive and mark navigation as failed """
            if route is None:
                logmsg(level="warn", category="route", id=agent.agent_id, msg="failed to find route, waiting idle")
                logmsg(level="warn", category="route", msg="modify here for wait_node addition")
                inactives += [agent]
                continue

            route_nodes = route.source + [goal_node] # add goal_node as it could be a critical point
            route_edges = route.edge_id
            agent.no_route_found_notification = True

            """save route details"""
            agent.route = route_nodes
            agent.route_edges = route_edges
            agent().route_found = True #ReplanTrigger #todo: is this really right here?
            logmsg(level="error", category="robot", id=agent.agent_id, msg="Route has been found, marking as such")

            self.get_edge_distances(agent_id)

        """secure locations for each inactive agent, to make routing not interfere"""
        for agent in inactives:
            agent.route = [agent.location(accurate=True)] #consider previous_node before closest_node
            agent.route_edges = []
            self.get_edge_distances(agent.agent_id)

        logmsg(category="route", msg="    - All agents assigned routes")
        for a in self.agent_details.values():
            logmsg(category="route", msg="        - %s:%s" % (a.agent_id, a.route))

        # find critical points and fragment routes to avoid critical point collistions
        self.split_critical_paths()

        rospy.sleep(1)
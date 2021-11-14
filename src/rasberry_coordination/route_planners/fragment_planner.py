#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import copy
import rospy
import threading
import yaml

from rasberry_coordination.route_planners.base_planner import BasePlanner
from rasberry_coordination.coordinator_tools import logmsg

from strands_navigation_msgs.msg import NavRoute

from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

class FragmentPlanner(BasePlanner):
    def __init__(self, all_agent_details_pointer, heterogeneous_map):
        """ Copy parameters to properties

        Args:
            all_agent_details_pointer - pointer to coordinator.all_agents_list a dictionary of all agent_details objects
        """
        super(FragmentPlanner, self).__init__(all_agent_details_pointer, heterogeneous_map)
        self.task_lock = threading.Lock()

    def update_available_tmap(self, agent):
        """remove incoming edges to the list of agent nodes in the available_tmap
        and update the available_route_search object with the new map
        :param agent_nodes: list of nodes occupied by other agents, list
        """
        # Nothing to do if restrictions are not used
        if 'restrictions' not in agent.navigation_properties: return

        available_tmap = copy.deepcopy(agent.navigation['tmap'])

        for node in available_tmap["nodes"]:
            to_pop = []
            for i in range(len(node["node"]["edges"])):
                if node["node"]["edges"][i]["node"] in self.occupied_nodes:
                    to_pop.append(i)
            if to_pop:
                to_pop.reverse()
                for j in to_pop:
                    node["node"]["edges"].pop(j)

        agent.navigation['tmap_available'] = available_tmap
        self.load_route_search(agent)

    def unblock_node(self, agent, node_to_unblock):
        """ unblock a node by adding edges to an occupied node in available_tmap
        copying from tmap
        :param node_to_unblock: name of the node to be unblocked, str
        """
        nodes_to_append = []
        edges_to_append = []

        """ for each edge in network, if edge connects to a node to unblock, add to list """
        for node in agent.navigation['tmap']["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["node"] == node_to_unblock:
                    nodes_to_append.append(node["node"]["name"])
                    edges_to_append.append(edge)

        """ for each node in empty map, if node is to be unblocked, add a extra edge """
        for node in agent.navigation['tmap_available']["nodes"]:
            if node["node"]["name"] in nodes_to_append:
                ind_to_append = nodes_to_append.index(node["node"]["name"])
                node["node"]["edges"].append(edges_to_append[ind_to_append])

        # update the route_search object
        agent.navigation['available_route_search'] = TopologicalRouteSearch(agent.navigation['tmap_available'])

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

        logmsg(category="route", id="COORDINATOR", msg="Finding routes for Active agents:")
        [logmsg(category="route", msg="    | %s: %s"%(a.agent_id, a.goal())) for a in self.agent_details.values()]
        actives =   [a for a in self.agent_details.values() if a.goal()]  # agents with an active nav goal (navigation)
        inactives = [a for a in self.agent_details.values() if not a.goal()]  # agents without an active goal (idle)
        logmsg(category="route", msg="actives --- "+str([a.agent_id for a in actives]))
        logmsg(category="route", msg="inactives - "+str([a.agent_id for a in inactives]))

        need_route = [a for a in self.agent_details.values() if a().route_required]
        logmsg(category="route", msg="Agents requiring routes:")
        for a in need_route:
            logmsg(category="route", msg="    | {%s: %s}" % (a.agent_id, a()))

        """find unblocked routes for all agents which need one"""
        self.load_occupied_nodes()
        for agent in actives:
            agent_id = agent.agent_id
            agent().route_found = False

            """get start node and goal node"""
            start_node = agent.location(accurate=False)
            goal_node  = agent.goal()
            logmsg(category="route", id=agent.agent_id, msg="Finding route for %s: %s -> %s" % (agent_id, start_node, goal_node))

            """if current node is goal node, mark agent as inactive"""
            if start_node == goal_node: #should _query should have handled this by this point?
                inactives += [agent]
                logmsg(category="route", msg="Agent is at goal_node, adding to inactives")
                continue

            """take copy of empty map"""
            self.update_available_tmap(agent)
            self.unblock_node(agent, start_node)
            self.unblock_node(agent, goal_node)  # TODO: is this needed?
            self.load_route_search(agent)

            """generate route from start node to goal node"""
            route = None
            print("label if start_node and goal_node:")
            if start_node and goal_node:
                route = self.get_available_optimum_route(agent, start_node, goal_node)
                # print("label get_route")
                # print(start_node)
                # print(goal_node)
                # print(route)
                # print("")
            route_nodes = []
            route_edges = []

            # xxxx = raw_input('Press enter to continue yo! ')
            # if xxxx == "a":
            #     import pdb; pdb.set_trace()

            """ If failed to find route, set robot as inactive and mark navigation as failed """
            if route is None:
                logmsg(level="warn", category="route", msg="failed to find route, waiting idle")
                logmsg(level="warn", category="route", msg="modify here for wait_node addition")
                self.no_route_found(agent)
                inactives += [agent]
                continue
            # else:
            #     print("label route_none")
            #     print(route)
            #     print("")


            route_nodes = route.source + [goal_node] # add goal_node as it could be a critical point
            route_edges = route.edge_id
            agent.no_route_found_notification = True #TODO: what?

            """save route details"""
            agent.route = route_nodes
            agent.route_edges = route_edges
            agent().route_found = True #ReplanTrigger #todo: is this really right here?
            logmsg(category="route", msg="Route has been found, marking as such")

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

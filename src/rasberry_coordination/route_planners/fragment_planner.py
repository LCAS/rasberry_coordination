#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, jheselden
# @email: pdasgautham@gmail.com, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy
import copy
import threading
import operator

# import strands_executive_msgs.msg
# import strands_executive_msgs.srv
import strands_navigation_msgs.msg
# import strands_navigation_msgs.srv
# import topological_navigation.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils

from rasberry_coordination.coordinator_tools import logmsg

class FragmentPlanner(object):
    def __init__(self, robot_manager_pointer, picker_manager_pointer, callbacks):
        """ Copy parameters to properties """
        self.robot_manager = robot_manager_pointer
        self.picker_manager = picker_manager_pointer
        self.callbacks = callbacks

        """ Change callback location to modify the FragmentPlanner available topomap """
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            agent.cb['update_topo_map'] = self.update_available_topo_map
            agent.cb['get_node'] = self.get_node

        """ Download Topological Map """
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self._map_cb)
        logmsg(msg='FragmentPlanner waiting for Topological map ...')
        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(msg='FragmentPlanner received Topological map ...')
        self.available_topo_map = copy.deepcopy(self.topo_map) #empty map used to measure routes

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_topo_map = True

    def update_available_topo_map(self, ):
        """This function updates the available_topological_map, which is topological_map
        without the edges going into the nodes occupied by the agents. When current node
        of an agent is none, the closest node of the agent is taken.
        """
        topo_map = copy.deepcopy(self.topo_map)
        agent_nodes = []

        """Extract lists of current and closest nodes to the robots and pickers"""
        if not (hasattr(self, 'robot_manager') and hasattr(self, 'picker_manager')):
            return

        curr_nodes = self.robot_manager.get_list('current_node') + self.picker_manager.get_list('current_node')
        clos_nodes = self.robot_manager.get_list('closest_node') + self.picker_manager.get_list('closest_node')


        """For each agent, if they do not have a current_node, extract the closest_node"""
        for i in range(len(curr_nodes)):
            if curr_nodes[i] is None:
                curr_nodes[i] = clos_nodes[i]
            if curr_nodes[i] is not None:
                agent_nodes.append(curr_nodes[i])

        for node in topo_map.nodes:
            to_pop=[]
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

    def get_edge_distances(self, robot_id):
        """find and fill distances of all edges of a robot's planned route, if at least one edge is there.
        the route must contain the goal_node as the last node in the list.

        Keyword arguments:
            robot_id -- robot_id
        """
        robot = self.robot_manager.agent_details[robot_id]
        robot.route_dists = []
        if len(robot.route_edges) >= 1:
            for i in range (len(robot.route) - 1):
                robot.route_dists.append(self.get_distance_between_adjacent_nodes(robot.route[i], robot.route[i+1]))

    def get_route_distance_to_node(self, robot_id, node_id):
        """get the total distance to a node in a robot's route

        Keyword arguments:
            robot_id -- id of the robot to be checked
            node_id -- node being checked
        """
        dist = 0.0
        adding_ok = False
        robot = self.robot_manager.agent_details[robot_id]
        if len(robot.route_edges) > 1:
            for i in range(len(robot.route)):
                if robot.route[i] == node_id:
                    break
                # add edge_distance only if the source node is not the one we look for
                # also make sure we start adding from current/closest node
                if not adding_ok:
                    if robot.current_node is not None:
                        if robot.current_node == robot.route[i]:
                            adding_ok = True
                    elif robot.closest_node is not None:
                        if robot.closest_node == robot.route[i]:
                            adding_ok = True
                if adding_ok:
                    dist += robot.route_dists[i]
        return dist

    def critical_points(self, ):
        """find points where agent's path cross with those of active robots.
        also find active robots which cross paths at these critical points.
        """
        active_robots = self.robot_manager.active_list()
        critical_points = {}  # {[route: critical point]} each route which contains a critical point
        critical_robots = {}  # {[critical_point: robot_ids]} all robots touching a critical point

        """for each agent"""
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            agent_id = agent.agent_id
            r_outer = agent.route
            critical_points[str(r_outer)] = set([])

            """for each active robot excluding the agent"""
            for robot_id in active_robots:
                robot = self.robot_manager.agent_details[robot_id]
                if agent_id == robot_id:
                    continue

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
                        if conflicted_node_id not in critical_robots:
                            critical_robots[conflicted_node_id] = [robot_id]
                        elif robot_id not in critical_robots[conflicted_node_id]:
                            critical_robots[conflicted_node_id].append(robot_id)

                        """add the active agent to the list of agents involved in the conflict"""
                        if agent_id in active_robots and agent_id not in critical_robots[conflicted_node_id]:
                            critical_robots[conflicted_node_id].append(agent_id)

        return (critical_points, critical_robots)

    def shortest_route_to_node(self, robot_ids, node_id):
        """from a list of robot_ids, find the robot with shortest route distance to a given node
        """
        dists = {}
        for robot_id in robot_ids:
            dists[robot_id] = self.get_route_distance_to_node(robot_id, node_id)

        return sorted(dists.items(), key=operator.itemgetter(1))[0][0]

    def split_critical_paths(self, ):
        """split robot paths at critical points
        """

        """identify critical points for each route and the robots which are involved"""
        c_points, c_robots = self.critical_points()

        """ remove start node as critical for robots heading to picker """
        for robot_id in self.robot_manager.active_list():
            robot = self.robot_manager.agent_details[robot_id]
            if robot.task_id:
                goal = robot.goal_node
                if (robot.task_stage == "go_to_picker" and goal in c_points[str(robot.route)]):
                    c_points[str(robot.route)].remove(goal)

        allowed_cpoints = []  #
        res_routes = {}  #

        """ for each agent populate res_routes with partial routes"""
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
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
                    nearest_robot = self.shortest_route_to_node(c_robots[v], v)

                    """
                    each critical vertice can be given to 1 robot
                    thus we give it to the closest robot and
                    prevent it being taken again by adding it to
                    allowed_cpoints,
                    """
                    if (agent_id == nearest_robot and v not in allowed_cpoints):
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
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            if agent.agent_id in res_routes:
                agent.route_fragments = res_routes[agent.agent_id]

        res_edges = {}
        # split the edges as per the route_fragmentsf
        """ for each active robot """
        charging_robots = self.robot_manager.charging_robots()

        for robot_id in set(self.robot_manager.active_list()+charging_robots):
            robot = self.robot_manager.agent_details[robot_id]

            """ if the robot has route fragments """
            if robot.route_fragments:

                """ remove goal node from final fragment """
                robot.route_fragments[-1].pop(-1)

                # if start and goal nodes are different, there will be at least one node remaining and an edge
                """ move the last node of all fragments to the start of next fragment """
                for i in range(len(robot.route_fragments) - 1):
                    robot.route_fragments[i+1].insert(0, robot.route_fragments[i][-1])
                    robot.route_fragments[i].pop(-1)

                """ split the edges """
                res_edges[robot_id] = []
                for i in range(len(robot.route_fragments)):
                    res_edges[robot_id].append(robot.route_edges[:len(robot.route_fragments[i])])
                    robot.route_edges = robot.route_edges[len(robot.route_fragments[i]):]
            else:
                robot.route_fragments = []
                res_edges[robot_id] = []

        """ for each agent, apply their route edges """
        # self.route_edges = res_edges
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            if agent.agent_id in res_edges:
                agent.route_edges = res_edges[agent.agent_id]

    def find_routes(self, ):
        """replan - find indiviual paths, find critical points in these paths, and fragment the
        paths at critical points - whenever triggered
        """

        """find routes for all robots which need one in empty map"""
        for robot in self.robot_manager.agent_details.values():
            robot_id = robot.robot_id

            """for each active robot"""
            if robot.active:

                """if waiting set goal as current node, generate route, and exit"""
                if robot.task_stage in ["wait_loading", "wait_unloading"]:
                    # loading and unloading robots should finish those stages first
                    # put the current node of the idle robots as their route - to avoid other robots planning routes through those nodes
                    robot.route = [robot._get_start_node()]
                    robot.route_edges = []
                    self.get_edge_distances(robot_id)
                    continue

                """get start node and goal node"""
                start_node = robot._get_start_node(accuracy=True)
                goal_node = robot._get_goal_node() #TODO: improve this function

                """if current node is goal node, generate empty route and set task as finished"""
                if start_node == goal_node:

                    # this is a moving robot, so must be in a go_to_task stage (picker, storage or base)
                    task_stage = robot.task_stage
                    robot._finish_task_stage(robot.task_stage)

                    if task_stage == "go_to_picker":
                        robot._reached_picker()
                        self.callbacks['publish_task_state'](robot.task_id, robot_id, "ARRIVED")
                        #logmsg(category="note", msg='publish_task_state callback within fragment planner should be removed')
                    elif task_stage == "go_to_storage":
                        robot._reached_storage()
                    elif task_stage == "go_to_base":
                        self.callbacks['send_robot_to_base'](robot_id)
                        #logmsg(category="note", msg='send_robot_to_base callback within fragment planner should be removed')

                        logmsg(category="list", msg='idle robots: %s' % (str(self.robot_manager.idle_list())))

                    # reset routes and route_edges
                    robot.route = [start_node]
                    robot.route_edges = []
                    self.get_edge_distances(robot_id)
                    continue

                """take copy of empty map"""
                avail_topo_map = copy.deepcopy(self.available_topo_map)

                """unblock means to add an additional edge into and out of any potentially conjected nodes"""
                """unblock nodes for robot starting point"""
                avail_topo_map = self.unblock_node(avail_topo_map, start_node)

                """unblock nodes for picker location so robots can reach them if needed"""
                if robot.task_stage == "go_to_picker":
                    avail_topo_map = self.unblock_node(avail_topo_map, goal_node)


                """generate route from start node to goal node"""
                avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                route = None
                if start_node and goal_node:
                    logmsg(category="robot", id=robot_id, msg='finding route for [start_node: %s | goal_node: %s]' % (start_node, goal_node))
                    route = avail_route_search.search_route(start_node, goal_node)

                route_nodes = []
                route_edges = []

                """if route is not available, replan route to wait node"""
                if (route is None and
                    robot.task_stage == "go_to_storage" and
                    robot.wait_node is not None and
                    robot.wait_node != robot.current_node):
                    logmsg(category="robot", id=robot_id, msg='no route to target %s, moving to wait at %s' % (robot.current_storage, robot.wait_node))
                    goal_node = robot.wait_node
                    avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                    route = avail_route_search.search_route(start_node, goal_node)

                """if still no route to wait node or goal node"""
                if route is None:
                    if robot.no_route_found_notification:
                        logmsg(category="robot", id=robot_id, msg='no route found from %s to %s' % (start_node, goal_node))
                        robot.no_route_found_notification = False
                    if start_node is None:
                        robot._dump(filename='no route found from None')
                    if start_node == "none":
                        robot._dump(filename='no route found from none')

                    #TODO: see how we could improve this by generating wait_node dynamically based on map activity

                else:
                    route_nodes = route.source
                    route_edges = route.edge_id
                    # add goal_node to route_nodes as it could be a critical point
                    route_nodes.append(goal_node)
                    robot.no_route_found_notification = True

                """save route details"""
                robot.route = route_nodes
                robot.route_edges = route_edges

                self.get_edge_distances(robot_id)

            # charging robots
            elif robot.charging:
                if robot.current_node is not robot.charging_node:
                    start_node = robot._get_start_node(accuracy=True)
                    goal_node = robot._get_goal_node() #TODO: improve this function
                    if start_node == goal_node:
                        rospy.loginfo("%s is charging now" %(robot.agent_id))
                        robot.route = [start_node]
                        robot.route_edges = []
                        self.get_edge_distances(robot_id)
                    else:
                        """take copy of empty map"""
                        avail_topo_map = copy.deepcopy(self.available_topo_map)

                        """unblock means to add an additional edge into and out of any potentially conjected nodes"""
                        """unblock nodes for robot starting point"""
                        avail_topo_map = self.unblock_node(avail_topo_map, start_node)

                        """generate route from start node to goal node"""
                        avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                        route = None
                        if start_node and goal_node:
                            logmsg(category="robot", id=robot_id, msg='finding route for [start_node: %s | goal_node: %s]' % (start_node, goal_node))
                            route = avail_route_search.search_route(start_node, goal_node)
                            rospy.loginfo(route)

                        route_nodes = []
                        route_edges = []

                        """if route is not available, replan route to wait node"""
                        if (route is None and
                            robot.wait_node is not None and
                            robot.wait_node != robot.current_node):
                            logmsg(category="robot", id=robot_id, msg='no route to target %s, moving to wait at %s' % (robot.current_storage, robot.wait_node))
                            goal_node = robot.wait_node
                            avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                            route = avail_route_search.search_route(start_node, goal_node)
                            rospy.loginfo(route)

                        """if still no route to wait node or goal node"""
                        if route is None:
                            if robot.no_route_found_notification:
                                logmsg(category="robot", id=robot_id, msg='no route found from %s to %s' % (start_node, goal_node))
                                robot.no_route_found_notification = False
                            if start_node is None:
                                robot._dump(filename='no route found from None')
                            if start_node == "none":
                                robot._dump(filename='no route found from none')

                            #TODO: see how we could improve this by generating wait_node dynamically based on map activity

                        else:
                            route_nodes = route.source
                            route_edges = route.edge_id
                            # add goal_node to route_nodes as it could be a critical point
                            route_nodes.append(goal_node)
                            robot.no_route_found_notification = True

                        """save route details"""
                        robot.route = route_nodes
                        robot.route_edges = route_edges
                        rospy.loginfo(robot.route_edges)

                        self.get_edge_distances(robot_id)
                pass

            else:
                """if robot is inactive, mark current node as route so as to not interfere with robot"""
                robot.route = [robot._get_start_node()] #TODO: sometimes unregistered robot is between 2 nodes
                robot.route_edges = []
                self.get_edge_distances(robot_id)

        """for each picker/virtual picker, mark current position as node to make routing not interfere"""
        for agent in self.picker_manager.agent_details.values():
            if agent.current_node is not None:
                agent.route = [agent.current_node]
            elif agent.previous_node is not None:
                agent.route = [agent.previous_node]
            else:
                agent.route = [agent.closest_node]
            agent.route_edges = []

        # find critical points and fragment routes to avoid critical point collistions
        self.split_critical_paths()

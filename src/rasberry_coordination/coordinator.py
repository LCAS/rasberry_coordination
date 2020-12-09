#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide
# @email: pdasgautham@gmail.com, marc@hanheide.net
# @date:
# ----------------------------------

import Queue
import copy

import rospy

import std_msgs.msg
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils
import thorvald_base.msg

import rasberry_coordination.robot
import rasberry_coordination.srv
from rasberry_coordination.coordinator_tools import logmsg, remove, add, move


class Coordinator(object):
    """Coordinator base class definition
    """
    def __init__(self, robot_ids, picker_ids, virtual_picker_ids,
                 ns="rasberry_coordination",
                 is_parent=False):
        """Initialise the base Coordinator class object.

        Keyword arguments:
            robot_ids -- list of all robots' ids
            picker_ids -- list of all human pickers' ids
            virtual_picker_ids -- list of all DES simulated pickers' ids
            is_parent -- if this is a parent class of another class
        """
        # TODO: this will strip leading slashes as well
        self.ns = ns.strip("/")+"/"

        self.robot_ids = robot_ids
        self.human_picker_ids = picker_ids
        self.virtual_picker_ids = virtual_picker_ids

        self.is_parent = is_parent

        # some basic states - extend as needed
        # 0 - idle
        self.robot_states_str = {0:"Idle"}
        self.robot_states = {robot_id:0 for robot_id in self.robot_ids}

        # time at which the current state of the robots are started
        self.start_time = {robot_id: rospy.get_rostime() for robot_id in self.robot_ids}

        self.idle_robots = self.robot_ids + []  # a copy of robot_ids
        self.active_robots = []  # all robots executing a task

        self.topo_map = None
        self.rec_topo_map = False
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self._map_cb)
        logmsg(msg='coordinator waiting for Topological map ...')
        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(msg='coordinator received Topological map ...')

        # default route search object
        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

        # route planning may be done on available_topo_map avoiding other tracked agents
        # available_topo_map = full_topomap - edges_to_nodes_currently_occupied
        self.available_topo_map = copy.deepcopy(self.topo_map)
        # create a topological_navigation.route_search.TopologicalRouteSearch object
        # with the current self.available_topo_map to plan routes avoiding
        # other agents, when necessary

        # presence agents - robot_ids, picker_ids, virtual_picker_ids from config
        self.presence_agents = copy.deepcopy(self.robot_ids)
        self.presence_agents.extend(self.human_picker_ids)
        self.presence_agents.extend(self.virtual_picker_ids)

        self.current_nodes = {agent_name:"none" for agent_name in self.presence_agents}
        self.prev_current_nodes = {agent_name:"none" for agent_name in self.presence_agents}
        self.closest_nodes = {agent_name:"none" for agent_name in self.presence_agents}
        self.current_node_subs = {agent_name:rospy.Subscriber(agent_name.strip()+"/current_node",
                                                              std_msgs.msg.String,
                                                              self._current_node_cb,
                                                              callback_args=agent_name) for agent_name in self.presence_agents}
        self.closest_node_subs = {agent_name:rospy.Subscriber(agent_name.strip()+"/closest_node",
                                                              std_msgs.msg.String,
                                                              self._closest_node_cb,
                                                              callback_args=agent_name) for agent_name in self.presence_agents}

        # setting a minimum voltage to avoid issues in gazebo simulations
        self.battery_voltage = {robot_id:55.0 for robot_id in self.robot_ids}
        self.battery_data_subs = {robot_id:rospy.Subscriber(robot_id+"/battery_data",
                                                             thorvald_base.msg.BatteryArray,
                                                             self._battery_data_cb,
                                                             callback_args=robot_id) for robot_id in self.robot_ids}

        if not self.is_parent:
            # this should only be called from the child class
            self.advertise_services()

        # don't queue more than 1000 tasks
        self.tasks = Queue.PriorityQueue(maxsize=1000)
        self.last_id = 0

        self.all_task_ids = [] # list of all task_ids
        self.processing_tasks = {} # {task_id:Task_definition}
        self.completed_tasks = {} # {task_id:Task_definition}
        self.cancelled_tasks = {} # {task_id:Task_definition}
        self.failed_tasks = {} # {task_id:Task_definition}

        self.task_robot_id = {} # {task_id:robot_id} to track which robot is assigned to a task
        self.robot_task_id = {robot_id: None for robot_id in self.robot_ids} # current task assigned to a robot

        logmsg(msg='coordinator initialised')

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_topo_map = True

    def _current_node_cb(self, msg, agent_name):
        """callback for current node msgs from presence agents
        """
        if self.current_nodes[agent_name] != "none":
            self.prev_current_nodes[agent_name] = self.current_nodes[agent_name]
        self.current_nodes[agent_name] = msg.data
        self.update_available_topo_map()

    def _closest_node_cb(self, msg, agent_name):
        """callback for closest node msgs from presence agents
        """
        self.closest_nodes[agent_name] = msg.data

    def _battery_data_cb(self, msg, robot_id):
        """callback for battery data msgs from robots
        """
        tot_voltage = 0.0
        count = 0
        for battery_data in msg.battery_data:
            if battery_data.battery_state == -98: # STATUS_ONLINE
                tot_voltage += battery_data.battery_voltage
                count += 1

        if count > 0:
            self.battery_voltage[robot_id] = tot_voltage/count

    def _get_robot_state(self, robot_id):
        """Template method for getting the state of a robot.
        Extend this as needed.

        Keyword arguments:
            robot_id -- robot_id

        Returns:
            (state, goal_node, start_time)
            state -- current state of the robot
            goal_node -- the target node, if the robot has a topological/exec_policy goal
            start_time -- time at which the current state of the robot is started
        """
        if robot_id in self.idle_robots:
            state = "idle"
            goal_node = ""
        else:
            state = ""
            goal_node = ""
        start_time = self.start_time[robot_id]
        return (state, goal_node, start_time)

    def get_robot_state_ros_srv(self, req):
        """get the state of a robot
        """
        resp = rasberry_coordination.srv.RobotStateResponse()
        if req.robot_id in self.robot_ids:
            resp.state, resp.goal_node, resp.start_time = self._get_robot_state(req.robot_id)
        else:
            logmsg(level='error', category="robot", id=req.robot_id, msg='not configured')
        return resp

    get_robot_state_ros_srv.type = rasberry_coordination.srv.RobotState

    def get_robot_states_ros_srv(self, req):
        """get the state of a set of robots
        """
        resp = rasberry_coordination.srv.RobotStatesResponse()
        for robot_id in req.robot_ids:
            if robot_id in self.robot_ids:
                state, goal_node, start_time = self._get_robot_state(robot_id)
                resp.states.append(state)
                resp.goal_nodes.append(goal_node)
                resp.start_times.append(start_time)
            else:
                resp.states.append("")
                resp.goal_nodes.append("")
                resp.start_times.append(rospy.Time())
                logmsg(level='error', category="robot", id=robot_id, msg='not configured')

        return resp

    get_robot_states_ros_srv.type = rasberry_coordination.srv.RobotStates

    def add_task_ros_srv(self, req):
        """Template service definition to add a task into the task execution framework.
        Extend as needed in a child class
        """
        self.last_id += 1
        task_id = self.last_id

        req.task.task_id = task_id
        logmsg(category="task", id=str(req.task.task_id), msg='received task')

        self.tasks.put(
            (task_id, req.task)
        )
        self.all_task_ids.append(task_id)
        return task_id

    add_task_ros_srv.type = strands_executive_msgs.srv.AddTask

    def cancel_task_ros_srv(self, req):
        """Template service definition to cancel a task.
        Extend as needed in a child class
        """
        cancelled = False
        # Two scenarios:
        # 1. task is already being processed
        #    call the task action's cancel topic
        # 2. task is still queued or is in processed (if allocated)
        #    pop the task from the queue and add to cancelled
        if req.task_id in self.all_task_ids:
            if ((req.task_id in self.completed_tasks) or
                  (req.task_id in self.cancelled_tasks)):
                logmsg(level='error', category="task", id=req.task_id, msg='cancelled task is being cancelled, cancel_task_ros_srv cannot be in this condition')

            elif req.task_id in self.processing_tasks:
                # task is being processed. remove it
                move(item=req.task_id, old=self.processing_tasks, new=self.cancelled_tasks)
                # cancel goal of assigned robot and return it to its base
                if req.task_id in self.task_robot_id:
                    robot_id = self.task_robot_id[req.task_id]
                    # call the task_action/cancel for the robot executing this task
                    pass
                logmsg(category="task", id=req.task_id, msg='task is being cancelled')
                cancelled = True

            else:
                # not yet processed. get it out of tasks
                tasks = []
                while not rospy.is_shutdown():
                    try:
                        task_id, task = self.tasks.get(True, 1)
                        if task_id == req.task_id:
                            self.cancelled_tasks[task_id] = task
                            logmsg(category="task", id=req.task_id, msg='cancelling task')

                            break # got it
                        else:
                            # hold on to the other tasks to be readded later
                            tasks.append((task_id, task))
                    except Queue.Empty:
                        break
                # readd popped tasks
                for (task_id, task) in tasks:
                    self.tasks.put((task_id, task))

                cancelled = True

        else:
            # invalid task_id
            logmsg(level='error', category="task", id=req.task_id, msg='cancel_task invoked with invalid task_id')

        return cancelled

    cancel_task_ros_srv.type = strands_executive_msgs.srv.CancelTask

    def all_tasks_info_ros_srv(self, req):
        """Get all tasks grouped into processing, failed, cancelled, unassigned and completed tasks.
        """
        resp = rasberry_coordination.srv.AllTasksInfoResponse()
        for task_id in self.processing_tasks:
            resp.processing_tasks.append(self.processing_tasks[task_id])
        for task_id in self.failed_tasks:
            resp.failed_tasks.append(self.failed_tasks[task_id])
        for task_id in self.cancelled_tasks:
            resp.cancelled_tasks.append(self.cancelled_tasks[task_id])
        for task_id in self.completed_tasks:
            resp.completed_tasks.append(self.completed_tasks[task_id])

        # unassigned tasks
        # retrieve tasks from queue and put them back
        tasks = []
        while not rospy.is_shutdown():
            try:
                task_id, task = self.tasks.get(True, 1)
                tasks.append((task_id, task))
            except Queue.Empty:
                break

        for (task_id, task) in tasks:
            resp.unassigned_tasks.append(task)
            self.tasks.put((task_id, task))

        return resp

    all_tasks_info_ros_srv.type = rasberry_coordination.srv.AllTasksInfo

    def advertise_services(self):
        """Adverstise ROS services.
        Only call at the end of constructor to avoid calls during construction.
        If this is a parent class, call from child class
        """
        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(
                    self.ns+attr[:-8],
                    service.type,
                    service
                )
                logmsg(msg='service advertised: %s' % (attr[:-8]))

    def update_available_topo_map(self, ):
        """This function updates the available_topological_map, which is topological_map
        without the edges going into the nodes occupied by the agents. When current node
        of an agent is none, the closest node of the agent is taken.
        """
        topo_map = copy.deepcopy(self.topo_map)
        agent_nodes = []
        for agent_id in self.presence_agents:
            if self.current_nodes[agent_id] != "none":
                agent_nodes.append(self.current_nodes[agent_id])
            elif self.closest_nodes[agent_id] != "none":
                agent_nodes.append(self.closest_nodes[agent_id])

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

        for node in self.topo_map.nodes:
            for edge in node.edges:
                if edge.node == node_to_unblock:
                    nodes_to_append.append(node.name)
                    edges_to_append.append(edge)

        for node in available_topo_map.nodes:
            if node.name in nodes_to_append:
                ind_to_append = nodes_to_append.index(node.name)
                node.edges.append(edges_to_append[ind_to_append])

        return available_topo_map

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:

        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_search.search_route(start_node, goal_node)
        if route is None:
            logmsg(msg='no route between %s and %s' %(start_node, goal_node))

            return ([], [], [float("inf")])
        route_nodes = route.source
        route_nodes.append(goal_node)
        route_edges = route.edge_id

        for i in range(len(route_nodes) - 1):
            route_distance.append(self.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distance)

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

    def finish_task(self, robot_id):
        """Template method to set the task assigned to the robot as finished.
        Extend in child class.

        Keyword arguments:
            robot_id -- robot_id
        """
        # move task from processing to completed
        task_id = self.robot_task_id[robot_id]
        self.robot_task_id[robot_id] = None
        move(item=task_id, old=self.processing_tasks, new=self.completed_tasks)
        # move robot from active to idle robots
        move(item=robot_id, old=self.active_robots, new=self.idle_robots)

    def set_task_failed(self, task_id):
        """Template method to set task state as failed.
        Extend as needed in a child class
        """
        move(item=task_id, old=self.processing_tasks, new=self.failed_tasks)

    def run(self):
        """Template main loop of the coordinator.
        Extend as needed in a child class
        """
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

            if self.idle_robots and not self.tasks.empty():
                logmsg(msg='unassigned tasks present, number of idle robots: %d' % (len(self.idle_robots)))

            # slow down the loop
            rospy.sleep(5.0)

    def on_shutdown(self, ):
        """Template method on_shutdown.
        Extend as needed in a child class
        """
        logmsg(msg='cancel actions of all active robots')
        for robot_id in self.robot_ids:
            if robot_id in self.active_robots:
                pass

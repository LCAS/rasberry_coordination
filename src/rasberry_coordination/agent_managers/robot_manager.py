#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import actionlib
import copy
import rospy
import yaml

from rospy import Subscriber as Sub, Service as Srv, get_rostime as Now
from std_msgs.msg import String as Str
from thorvald_base.msg import BatteryArray as Battery
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak
from rasberry_coordination.robot import Robot as RobotInterface
from rasberry_coordination.agent_managers.agent_manager import AgentManager, AgentDetails
from rasberry_coordination.srv import RobotState, RobotStates, RobotStateResponse, RobotStatesResponse
from topological_navigation.route_search2 import TopologicalRouteSearch2

class RobotManager(AgentManager):

    """Initialise class with callback details to apply to robots"""
    def __init__(self, callback_dict):
        """ Initialise management interface for robots and services for generic communications

        :param callback_dict: container for direct callbacks to coordinator for use in subscribers
        """
        super(RobotManager, self).__init__(callback_dict)
        self.dump_cb = Sub("rasberry_coordination/robot_manager/dump", Str, self.dump_details)
        Srv("rasberry_coordination/get_robot_state", RobotState, self.get_robot_state_ros_srv)
        Srv("rasberry_coordination/get_robot_states", RobotStates, self.get_robot_states_ros_srv)  # TODO: one not needed

    """Add Robot agents"""
    def add_agents(self, agent_id_list, use_restrictions):
        """ Initialise a given list of agents and add to the agent_details collection

        :param agent_id_list: list of agents to instantiate
        :return: None
        """
        for agent_id in agent_id_list:
            self.add_agent(agent_id, use_restrictions)

    """Add Robot Details Objects"""
    def add_agent(self, agent_id, use_restrictions):
        """ Initialise a given agent and add to the agent_details collection

        :param agent_id: identifier for the agent to initialise
        :return: None
        """
        self.agent_details[agent_id] = RobotDetails(agent_id, self.cb, use_restrictions)

    """Service responses"""
    def get_robot_states_ros_srv(self, req): #TODO: combine these 2 srv into 1?
        """ Service callback for "rasberry_coordination/get_robot_states"

        :param req: list of robots to respond with
        :return: RobotStatesResponse() containing current state for each robot_id (state, start_time, goal_node)
        """
        resp = RobotStatesResponse()
        for robot_id in req.robot_ids:
            resp1 = RobotStateResponse()
            if robot_id in self.agent_details:
                resp1 = self._get_robot_state(robot_id)
            resp.states.append(resp1.state)
            resp.goal_nodes.append(resp1.goal_node)
            resp.start_times.append(resp1.start_time)
        return resp
    def get_robot_state_ros_srv(self, req):
        """ Service callback for "rasberry_coordination/get_robot_state"

        :param req: robot to identify state of
        :return: RobotStateResponse() containing current state for the given robot_id (state, start_time, goal_node)
        """
        resp = RobotStateResponse()
        if req.robot_id in self.agent_details:
            resp = self._get_robot_state(req.robot_id)
        return resp
    def _get_robot_state(self, robot_id):
        """ Method to get information for get_robot_state/s callback

        :param robot_id: robot to identify current state of
        :return: RobotStateResponse() containing current state for the given robot_id (state, start_time, goal_node)
        """
        resp = RobotStateResponse()
        robot = self.agent_details[robot_id]
        resp.goal_node = ""
        if robot.goal_node:
            resp.goal_node = robot.goal_node
        if robot.task_stage is not None:
            resp.state = robot.task_stage
        elif robot.idle:
            resp.state = "idle"
        else:
            resp.state = ""
        resp.start_time = robot.start_time

        return resp

    """ Commonly used list generators """
    def registered_robots(self):
        """ Return dict of registered robots
        :return: {agent_id: True} for each robot if registered
        """
        return {robot.agent_id:robot.registered for robot in self.agent_details.values() if robot.registered}
    def registered_list(self): #TODO: swap out to polymorphism
        """ Return list of registered robots
        :return: [agent_id] for each robot if registered
        """
        return [robot.agent_id for robot in self.agent_details.values() if robot.registered]
    def idle_list(self):
        """ Return list of idle robots
        :return: [agent_id] for each robot if idle
        """
        return [robot.agent_id for robot in self.agent_details.values() if robot.idle is True]
    def moving_list(self):
        """ Return list of moving robots
        :return: [agent_id] for each robot if moving
        """
        return [robot.agent_id for robot in self.agent_details.values() if robot.moving is True]
    def active_list(self):
        """ Return list of active and not paused robots
        :return: [agent_id] for each robot if active and not paused
        """
        return [robot.agent_id for robot in self.agent_details.values() if robot.active and not robot.paused]
    def interruptable_list(self):
        """ Return list of interruptable robots
        :return: [agent_id] for each robot if interruptable
        """
        return [robot.agent_id for robot in self.agent_details.values() if robot.interruptable is True]
    def idle_robots_exist(self):
        """ Iterate across all robots and return 1 if any are found to be idle
        :return: True if any(robots.idle = True)
        """
        for robot in self.agent_details.values():
            if robot.idle:
                return 1
        return 0
    def moving_robots_exist(self):
        """ Iterate across all robots and return 1 if any are found to be moving
        :return: True if any(robots.moving = True)
        """
        #If any robot is moving(not idle), return true
        for robot in self.agent_details.values():
            if not robot.idle: #is moving if NOT idle
                return 1
        return 0
    def available_robots(self):
        """ Return list of registered agents, which are idle or interruptable (all agents able to take on a new task)
        :return: [agent_id] for each robot if registered and idle/interruptable
        """
        return [R.agent_id for R in self.agent_details.values() if (R.idle or R.interruptable) and R.registered]

"""Centralised container for all details pertaining to the robot"""
class RobotDetails(AgentDetails):
    def __init__(robot, ID, cb, use_restrictions):
        """ Class to define details and interactions for an individual robot.

        :param ID: Unique identifie for robot
        :param cb: Callbacks to direct response from location subscribers
        """

        """Initialise Fields in Parent Class"""
        super(RobotDetails, robot).__init__(ID, cb)

        robot.type = None # type of the robot - short or tall
        robot.task_types = None # task capabilities of the robot and the role
        robot.use_restrictions = use_restrictions # use toponav2 restrictions or not

        robot.tmap2 = {}
        robot.tmap2_nodes = []
        robot.available_tmap2 = {}
        robot.available_route_search = TopologicalRouteSearch2({"nodes":{}})
        if robot.use_restrictions:
            robot.tmap2_sub = rospy.Subscriber("/%s/restricted_topological_map_2" %(ID), Str, robot.restricted_tmap2_cb, queue_size=5)
        else:
            robot.tmap2_sub = rospy.Subscriber("/topological_map_2", Str, robot.tmap2_cb, queue_size=5)

        """Detail whether the robot is moving"""
        robot.idle = True
        robot.interruptable = False
        robot.has_toponav_goal = False
        robot.active = False
        robot.moving = False

        """
        interruptable if (has_task and is_going_to_base) or (!has_task)
        moving if (has_toponav_goal?)
        active if (has_task and !is_going_to_base)
        idle if (!has_task and !has_toponav_goal?)
        """

        """             has_task    has_goal    go_base
        moving          x           1           x
        idle            0           0           x
        interruptable   1           x           1
        active          1           x           0
        """

        """Meta Management"""
        robot.robot_id = ID
        robot.robot_interface = RobotInterface(ID)

        """Task Details"""
        robot.task_id = None
        robot.goal_node = None
        robot.task_stage = None
        robot.task_stage_list = []
        robot.tray_loaded = False
        robot.start_time = Now()

        """Task Meta Details"""
        robot.max_task_priority = 255
        robot.admissible_tasks = [] #TODO: add admissible_tasks to robot details in map_config file
        # this would be a good way to manage what robots should take on tasks
        # making use of a simple condtion to check if robot can do task X

        """Goal Definitions"""
        robot.start_node = None
        robot.current_storage = None
        robot.base_station = None
        robot.wait_node = None

        """Route Details"""
        robot.route = []
        robot.route_dists = []
        robot.route_edges = []
        robot.route_fragments = []

        """Notifications"""
        robot.no_route_found_notification = True

        """Registration Details"""
        robot.registered = True
        robot.unregistration_type = None
        robot.disconnect_when_idle = False
        robot.paused = False

    # """On Shutdown"""
    # def _remove(robot): #This shouldnt be required.
    #     super(RobotDetails, robot)._remove()

    """returns True if the robot can do a task"""
    def can_do(robot, task_type):
        """check if the robot can execute a task type depending on task modules
        """
        if robot.task_types is None:
            return False
        elif robot.task_types.__class__ is list:
            if len(robot.task_types) == 0:
                return False
            else:
                for item in robot.task_types:
                    if item["module"] == task_type:
                        return True
                return False

    """tmap2 callback"""
    def tmap2_cb(robot, msg):
        """topological_map_2 callback
        """
        robot.tmap2 = yaml.safe_load(msg.data)
        robot.tmap2_nodes = [node["node"]["name"] for node in robot.tmap2["nodes"]]
        robot.available_tmap2 = copy.deepcopy(robot.tmap2)
        robot.available_route_search = TopologicalRouteSearch2(robot.available_tmap2)

    """restricted_tmap2 callback"""
    def restricted_tmap2_cb(robot, msg):
        """restricted_topological_map_2 callback
        """
        robot.tmap2 = yaml.safe_load(msg.data)
        robot.tmap2_nodes = [node["node"]["name"] for node in robot.tmap2["nodes"]]
        robot.available_tmap2 = copy.deepcopy(robot.tmap2)
        robot.available_route_search = TopologicalRouteSearch2(robot.available_tmap2)

    """check if a node is in the robot's topomap"""
    def is_node_in_topomap(robot, node_name):
        """checks if a given node is in the robot's restricted tmap2

        :param node_name: name of the node, str
        """
        return node_name in robot.tmap2_nodes

    """ remove the given agent nodes and update the available_tmap2"""
    def update_available_tmap2(robot, agent_nodes=[]):
        """remove incoming edges to the list of agent nodes in the available_tmap2
        and update the available_route_search object with the new map

        :param agent_nodes: list of nodes occupied by other agents, list
        """
#        print ("%s updating available map" %(robot.robot_id), agent_nodes)
        available_tmap2 = copy.deepcopy(robot.tmap2)

        for node in available_tmap2["nodes"]:
            to_pop=[]
            for i in range(len(node["node"]["edges"])):
                if node["node"]["edges"][i]["node"] in agent_nodes:
                    to_pop.append(i)
            if to_pop:
                to_pop.reverse()
                for j in to_pop:
                    node["node"]["edges"].pop(j)

        robot.available_tmap2 = available_tmap2
        robot.available_route_search = TopologicalRouteSearch2(robot.available_tmap2)

    def unblock_node(robot, node_to_unblock):
        """ unblock a node by adding edges to an occupied node in available_tmap2
        copying from tmap2

        :param node_to_unblock: name of the node to be unblocked, str
        """
#        print ("%s unblocking %s" %(robot.robot_id, node_to_unblock))
        nodes_to_append=[]
        edges_to_append=[]

        """ for each edge in network, if edge connects to a node to unblock, add to list """
        for node in robot.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["node"] == node_to_unblock:
                    nodes_to_append.append(node["node"]["name"])
                    edges_to_append.append(edge)

        """ for each node in empty map, if node is to be unblocked, add a extra edge """
        for node in robot.available_tmap2["nodes"]:
            if node["node"]["name"] in nodes_to_append:
                ind_to_append = nodes_to_append.index(node["node"]["name"])
                node["node"]["edges"].append(edges_to_append[ind_to_append])

        # update the route_search object
        robot.available_route_search = TopologicalRouteSearch2(robot.available_tmap2)

    """get a route to the goal node from the available tmap2"""
    def get_available_optimum_route(robot, start_node, goal_node):
        """ find and return a route from start_node to goal_node on the available_tmap2

        :param start_node: name of the node from which route should be planned, str
        :param goal_node: name of the node to which route should be planned, str
        :return: route from start_node to goal_node
        """
        route = None
        route = robot.available_route_search.search_route(start_node, goal_node)
        return route

    """return goal node as picker location, storage or base station"""
    def _get_goal_node(robot):
        """ Identify goal node depending on current task stage

        :return: navigation target
        """
        if robot.task_stage == "go_to_picker":
            return robot.goal_node
        elif robot.task_stage == "go_to_storage":
            return robot.current_storage
        elif robot.task_stage == "go_to_base":
            return robot.base_station

    """State Changes"""
    def _set_as_idle(robot):
        """ Set attributes for when robot is idle
        :return: None
        """
        robot.idle = True
        robot.moving = robot.active = False
        robot.interruptable = False  # not needed
    def _begin_task(robot, task_id):
        """ Set attributes for when courier task is begun

        :param task_id: task_id created by picker on task definition
        :return: None
        """
        if robot.interruptable:
            robot.robot_interface.cancel_execpolicy_goal()
            robot._finish_task_stage("")
        robot.task_id = task_id
        robot.task_stage_list = ["go_to_picker", "wait_loading", "go_to_storage",
                                 "wait_unloading", "go_to_base", None]
        robot.idle = False
        robot.active = True
        robot.interruptable = False
        #print("_init_task: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_begin_task: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _reached_picker(robot):
        """ Set attributes for when courier reaches picker

        :return: None
        """
        robot.robot_interface.cancel_execpolicy_goal()
        #robot._finish_task_stage("go_to_picker")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_reached_picker: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _tray_loaded(robot):
        """ Set attributes for when conditions are met for tray to be defined as loaded
        (timeout or notification from picker)

        :return: None
        """
        robot.tray_loaded = True
        #robot._finish_task_stage("wait_loading")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_tray_loaded: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _reached_storage(robot):
        """ Set attributes for when robot has reached the storage

        :return: None
        """
        robot.robot_interface.cancel_execpolicy_goal()
        #robot._finish_task_stage("go_to_storage")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_reached_storage: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _tray_unloaded(robot):
        """ Set attributes for when conditions are met for tray to be defined as unloaded
        (timeout or notification from storage)

        :return: None
        """
        robot.task_id = None
        robot.tray_loaded = False
        robot.interruptable = True
        robot.moving = False
        robot.current_storage = None
        #robot._finish_task_stage("wait_unloading")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_tray_unloaded: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _set_target_base(robot):
        """ Set attributes for when robot has reached the base_station

        :return: None
        """
        robot.idle = False
        robot.active = True
        robot.interruptable = True
        robot._change_task_stage("go_to_base")
    def _end_task(robot):
        """ Set attributes for when robot has reached the end of its task

        :return: None
        """
        robot.task_id = None
        if robot.registered:
            robot.idle = True
        robot.active = False
        robot.interruptable = False
        robot._change_task_stage(None)

    def _finish_route_fragment(robot):
        """ Set attributes for when robot has reached the end of its current route fragment

        :return: None
        """
        robot.moving = False
        robot.route = []
        robot.route_fragments = []
        robot.robot_interface.execpolicy_result = None
    def _finish_task_stage(robot, stage):
        """ Set attributes for when robot has reached the end of a task stage or sub-route fragment.
        If the task stage is a navigation stage, mark froute reagment as complete. Then start the next task stage.

        :return: None
        """

        if robot.task_stage in ["go_to_picker", "go_to_storage", "go_to_base"]:
            robot._finish_route_fragment()
        robot.start_time = Now()
        if len(robot.task_stage_list):
            robot._change_task_stage(stage)
    def _change_task_stage(robot, stage):
        """ Set the given task stage as the current stage

        :return: None
        """
        if robot.task_stage == stage:
            return
        robot.task_stage = stage
        logmsgbreak()
        logmsg(category="robot", id=robot.robot_id, msg='@ stage: %s' % (str(robot.task_stage).upper()))

    """ Registration Controls """
    def _pause_task(robot):
        """ Set robot as paused:
        > mark robot as paused
        > add current task_stage to task_stage_list
        > set current task_stage as "paused"
        > cancel current navigation execpolicy

        :return: None
        """
        robot.unregistration_type = "pause_task"
        robot.paused = True
        robot.task_stage_list.insert(0,robot.task_stage)
        robot.task_stage = "paused"
        #print("_pause_task: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
        robot.robot_interface.cancel_execpolicy_goal()

    def _unpause_robot(robot):
        """ Set robot as unpaused:
        > mark robot as unpaused

        :return: None
        """
        robot.unregistration_type = None
        robot.paused = False
    def _unpause_task(robot):
        """ Set task as unpaused:
        > complete the "paused" task stage and begin the next

        :return: None
        """
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_unpause_task: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _drm_release_task(robot):
        """ Release task as part of unregistration
        > set unregistration_type as "release_task" for use in registration
        > call self._release_task()

        :return: None
        """
        robot.unregistration_type = "release_task"
        robot._release_task()
    def _release_task(robot):
        """ Release task as pat of task cancellation
        > remove navigation execpolicy
        > call self._end_task()

        :return: None
        """
        robot.goal_node = None
        robot._end_task()
        robot.robot_interface.cancel_execpolicy_goal()

        if not robot.disconnect_when_idle:
            robot.idle = False

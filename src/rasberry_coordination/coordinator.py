#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide
# @email: pdasgautham@gmail.com, marc@hanheide.net
# @date:
# ----------------------------------

import Queue
import copy
import yaml

import rospy

import std_msgs.msg
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.msg
import topological_navigation.route_search2
import topological_navigation.tmap_utils
import thorvald_base.msg

import rasberry_coordination.robot
import rasberry_coordination.srv
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails
from rasberry_coordination.coordinator_tools import logmsg, remove, add, move
from rasberry_coordination.agent_managers.robot_manager import RobotManager
from rasberry_coordination.agent_managers.picker_manager import PickerManager


class Coordinator(object):
    """Coordinator base class definition
    """
    def __init__(self,
                 robot_ids,
                 picker_ids,
                 virtual_picker_ids,
                 use_restrictions,
                 ns="rasberry_coordination",
                 is_parent=False):
        """Initialise the base Coordinator class object.

        Keyword arguments:
            robot_ids -- list of all robots' ids
            picker_ids -- list of all human pickers' ids
            virtual_picker_ids -- list of all DES simulated pickers'
            use_restrictions -- to use toponav2 restrictions or not
            is_parent -- if this is a parent class of another class
        """
        # TODO: this will strip leading slashes as well
        self.ns = ns.strip("/")+"/"

        self.human_picker_ids = picker_ids
        self.virtual_picker_ids = virtual_picker_ids

        self.is_parent = is_parent

        self.topo_map = None
        self.rec_topo_map = False
        rospy.Subscriber("topological_map_2", std_msgs.msg.String, self._map_cb)
        logmsg(msg='coordinator waiting for Topological map 2...')
        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(msg='coordinator received Topological map 2...')

        # default route search object
        self.route_search = topological_navigation.route_search2.TopologicalRouteSearch2(self.topo_map)

        # route planning may be done on available_topo_map avoiding other tracked agents
        # available_topo_map = full_topomap - edges_to_nodes_currently_occupied
        self.available_topo_map = copy.deepcopy(self.topo_map)
        # create a topological_navigation.route_search.TopologicalRouteSearch object
        # with the current self.available_topo_map to plan routes avoiding
        # other agents, when necessary

        if not self.is_parent:
            # this should only be called from the child class
            self.advertise_services()

        self.last_id = 0

        self.all_task_ids = [] # lists of all task_ids
        self.completed_tasks = []
        self.cancelled_tasks = []
        self.failed_tasks = []

        self.task_robot_id = {} # {task_id:robot_id} to track which robot is assigned to a task

        """Robot Detail Manage Initialisation"""
        cb_dict = {'task_cancelled': self.task_cancelled, 'task_update': self.task_update}
        self.robot_manager = RobotManager(cb_dict)
        self.robot_manager.add_agents(robot_ids, use_restrictions)
        self.picker_manager = PickerManager(cb_dict)
        self.picker_manager.add_agents(picker_ids + virtual_picker_ids)
        for picker in self.picker_manager.agent_details.values():
            if picker.picker_id in virtual_picker_ids:
                picker.virtual = True
                picker.task_priority = 0
        """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

        self.active_tasks_pub = rospy.Publisher(self.ns + "active_tasks_details",
                                                rasberry_coordination.msg.TasksDetails, latch=True, queue_size=5)
        logmsg(msg='coordinator initialised')

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = yaml.safe_load(msg.data)
        self.rec_topo_map = True

    def cancel_task_ros_srv(self, req):
        """Template service definition to cancel a task.
        Extend as needed in a child class
        """
        task_id = req.task_id

        #Set picker to finish task
        picker = self.picker_manager.get_task_handler(task_id)
        if picker:
            picker.task_finished()

        # Set robot to abandon task
        robot = self.robot_manager.get_task_handler(task_id)
        if robot:
            robot._release_task()
            robot._set_target_base()

        # if task is not active for any picker or robot, it is invalid
        if picker is None and robot is None:
            return False

        self.inform_toc_task_ended(task_id=task_id, reason="task_cancelled")
        return True

    cancel_task_ros_srv.type = strands_executive_msgs.srv.CancelTask

    def inform_toc_active_tasks(self):
        """ publish TasksDetailsList to inform TOC of the current state of all pickers and robots """
        task_list = TasksDetailsList()

        """ loop through tasks owned by pickers """
        for picker in self.picker_manager.get_all_task_handlers():

            """ get the task details """
            task = SingleTaskDetails()
            task.task_id = picker.task_id
            task.state = picker.task_stage

            """ identify connected robot """
            assigned_robot = self.robot_manager.get_task_handler(picker.task_id)
            if assigned_robot:
                task.robot_id = assigned_robot.agent_id
            task.picker_id = picker.agent_id

            """ add task to list """
            task_list.tasks.append(task)

        """ loop through tasks owned by robots """
        for robot in self.robot_manager.get_all_task_handlers():

            """ if task is already added, move on """
            if robot.task_id in [T.task_id for T in task_list.tasks]:
                continue

            """ get the task details """
            task = SingleTaskDetails()
            task.task_id = robot.task_id
            task.state = robot.task_stage

            """ label connected robot """
            task.robot_id = robot.agent_id

            """ add task to list """
            task_list.tasks.append(task)

        """ Convert task state to a toc-recognised state """
        for T in task_list.tasks:
            T.state = self.toc_legacy_responses(T.state)

        self.active_tasks_pub.publish(task_list)

    def inform_toc_task_ended(self, task_id, reason, robot_id=None, picker_id=None):
        """ build a TasksDetailsList object to send to TOC to inform of task end """
        task_list = TasksDetailsList()

        """ get the task details """
        task = SingleTaskDetails()
        task.task_id = task_id
        task.robot_id = str(robot_id)
        task.picker_id = str(picker_id)
        task.state = self.toc_legacy_responses(reason)

        """ add task to list and publish """
        task_list.tasks.append(task)
        self.active_tasks_pub.publish(task_list)

    def toc_legacy_responses(self, state):
        """ as not all task_stage identifiers are TOC-compatible, remap these here """
        return {'CREATED':        'CALLED',
                'ASSIGNED':       'ACCEPT',
                'go_to_picker':   'ACCEPT',

                'ARRIVED':        'ARRIVED',

                'LOADED':         'LOADED',
                'wait_loading':   'LOADED',

                'go_to_storage':  'STORAGE',
                'wait_unloading': 'STORAGE',

                'task_completed': 'DELIVERED',

                'task_cancelled': 'CANCELLED',

                'paused':         'PAUSED',
                'go_to_base':     'None',
                'None':           'None'}[str(state)]

        # picker task stages:
        # "CREATED", "ASSIGNED", "ARRIVED", "LOADED", None

        # robot task stages:
        # go_to_picker, wait_loading, go_to_storage, wait_unloading, go_to_base, None, paused

        # additional task states:
        # task_completed, task_cancelled

    def all_tasks_info_ros_srv(self, req):
        """Get all tasks grouped into processing, failed, cancelled, unassigned and completed tasks.
        """
        resp = rasberry_coordination.srv.AllTasksInfoResponse()
        for task_id in self.failed_tasks: #added in coordinator.set_task_failed
            resp.failed_tasks.append(self.format_task_obj(task_id))
        for task_id in self.cancelled_tasks: #added in coordinator.task_cancelled
            resp.cancelled_tasks.append(self.format_task_obj(task_id))
        for task_id in self.completed_tasks: #added in [rasberry_]coordinator.finish_task
            resp.completed_tasks.append(self.format_task_obj(task_id))
        for P in self.picker_manager.unassigned_tasks(): #queries (picker.task_id and picker.task_stage is "CREATED")
            resp.unassigned_tasks.append(self.picker_manager.format_task_obj(picker_id=P))
        for P in self.picker_manager.assigned_tasks(): #queries (picker.task_id and picker.task_stage is not "CREATED")
            resp.processing_tasks.append(self.picker_manager.format_task_obj(picker_id=P))
        return resp

    all_tasks_info_ros_srv.type = rasberry_coordination.srv.AllTasksInfo

    def format_task_obj(self, task_id):
        """ format strands_executive_msgs.msg.Task object with the given task_id """
        task = strands_executive_msgs.msg.Task
        task.task_id = task_id
        return task

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
        return topological_navigation.tmap_utils.get_node_from_tmap2(self.topo_map, node)

    def get_distance_between_adjacent_nodes(self, from_node, to_node):
        """get_distance_between_adjacent_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return topological_navigation.tmap_utils.get_distance_to_node_tmap2(from_node_obj, to_node_obj)

    def finish_task(self, robot_id):
        """Template method to set the task assigned to the robot as finished.
        Extend in child class.

        Keyword arguments:
            robot_id -- robot_id
        """
        # move task from processing to completed
        robot = self.robot_manager[robot_id]
        task_id = robot.task_id
        robot.task_id = None
        self.completed_tasks.append(task_id)

        # move robot from active to idle robots
        robot.idle = True
        robot.active = False

    def set_task_failed(self, task_id):
        """Template method to set task state as failed.
        Extend as needed in a child class
        """
        self.failed_tasks.append(task_id)

    def task_cancelled(self, task_id):
        """ Callback from picker_manager, on CAR call to cancel task """

        """ Inform the assigned robot if there is one  """
        robot = self.robot_manager.get_task_handler(task_id)
        if robot:
            robot._release_task()
            robot._set_target_base()

        """ Inform TOC """
        self.set_task_failed(task_id)
        self.inform_toc_task_ended(task_id=task_id, reason="task_cancelled")
        self.cancelled_tasks.append(task_id)

    def task_update(self, update, task_id, details):
        """ Abstract base function as placeholder
        till rasberry_coordinator is defined
        """
        pass

    def run(self):
        """Template main loop of the coordinator.
        Extend as needed in a child class
        """
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

            """ if there are unassigned tasks and there robots able to take them on """
            available_robots = self.robot_manager.available_robots()
            unassigned_tasks = self.picker_manager.unassigned_tasks()
            if available_robots and unassigned_tasks:
                logmsg(msg='unassigned tasks present, number of idle robots: %d' % (len(self.robot_manager.idle_list())))

            # slow down the loop
            rospy.sleep(5.0)
            self.inform_toc_active_tasks()

    def on_shutdown(self, ):
        """Template method on_shutdown.
        Extend as needed in a child class
        """
        logmsg(msg='cancel actions of all active robots')
        for robot in self.robot_manager.agent_details.values():
            if robot.active:
                pass

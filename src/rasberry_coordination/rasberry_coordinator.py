#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide, jheselden
# @email: pdasgautham@gmail.com, marc@hanheide.net, jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import operator
import Queue
import copy
import os
import csv
import time
import datetime
import threading

import rospy

import std_srvs.srv
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils

import rasberry_coordination.robot
import rasberry_coordination.msg
import rasberry_coordination.srv
import rasberry_coordination.coordinator
from rasberry_coordination.coordinator_tools import logmsg, remove, add, move
from rasberry_coordination.route_planners.route_planners import RouteFinder


class RasberryCoordinator(rasberry_coordination.coordinator.Coordinator):
    """RasberryCoordinator class definition
    """
    def __init__(self, robot_ids, picker_ids, virtual_picker_ids,
                 local_storages, cold_storage, use_cold_storage,
                 base_stations, wait_nodes,
                 max_task_priorities,
                 admissible_robot_ids, active_tasks,
                 base_station_nodes_pool, wait_nodes_pool,
                 max_load_duration, max_unload_duration,
                 ns="rasberry_coordination"):
        """initialise a RasberryCoordinator object

        Keyword arguments:
            robot_ids -- list of robot_ids
            picker_ids-- list of human picker_ids
            virtual_picker_ids -- list of virtual (DES) picker_ids
            local_storages -- list of local storage nodes
            cold_storage -- cold storage node
            use_cold_storage -- flag to use cold/local storage
            base_stations -- base station nodes for the robots {robot_id:base_node}
            wait_nodes -- waiting nodes (if robot cannot go to storage, it could
                          wait at this node) {robot_id:wait_node}
            max_task_priorities -- max priority of tasks a robot can be assigned to.
                                   this is used to set some robots exclusively for
                                   tasks from human pickers (at higher priority),
                                   while some robots can be assigned to both human
                                   and virtual pickers. (To make demos interesting)
            admissible_robot_ids -- list of robots ids for the robots which are allowed to connect to the system
            active_tasks -- list of tasks which the system is currently performing
            base_station_nodes_pool -- pool defining list of all base stations within the system
            wait_nodes_pool -- pool defining list all waiting nodes
            max_load_duration -- time to wait until the coordinator forces advancement through LOADING stage
            max_unload_duration -- time to wait until the coordinator forces advancement through UNLOADING stage
        """

        self.log_routes = False  # define whether to log route details to console

        super(RasberryCoordinator, self).__init__(robot_ids,
                                                  picker_ids,
                                                  virtual_picker_ids,
                                                  ns=ns,
                                                  is_parent=True)

        self.local_storages = local_storages
        self.cold_storage = cold_storage
        self.use_cold_storage = use_cold_storage

        # Base station initialisation
        self.base_station_nodes_pool = set(base_station_nodes_pool)
        used_base_stations = set(base_stations.values())
        self.available_base_stations = list(self.base_station_nodes_pool.difference(used_base_stations))

        logmsg(msg='base stations in use: %s' % (base_stations.values()))
        logmsg(msg='base stations available: %s' % (str(self.available_base_stations)))

        # Wait node initialisation
        self.wait_nodes_pool = set(wait_nodes_pool)
        used_wait_nodes = set(wait_nodes.values())
        self.available_wait_nodes = list(self.wait_nodes_pool.difference(used_wait_nodes))

        # Initialise conditions for robot registration
        self.admissible_robot_ids = admissible_robot_ids
        self.active_tasks = active_tasks
        logmsg(msg='active tasks: ' + ', '.join(self.active_tasks))

        # Initialise topics for marker management
        self.marker_add_pub = rospy.Publisher('/rasberry_coordination/marker_add',
                                                rasberry_coordination.msg.MarkerDetails, queue_size=5)
        self.marker_remove_pub = rospy.Publisher('/rasberry_coordination/marker_remove',
                                                  rasberry_coordination.msg.MarkerDetails, queue_size=5)

        self.task_lock = threading.Lock()

        logs_dir = os.path.join(os.environ["HOME"], "rasberry_coordination_logs")
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
        dt = datetime.datetime.now()
        self.log_file = open(os.path.join(logs_dir, dt.strftime("%Y%m%d-%H%M%S_coordinator.csv")), "w")
        self.log_headers = ["id", "time", "datetime", "action_type",
                            "task_id", "robot_id", "robot_task_stage",
                            "task_updates", "details", "current_node",
                            "closest_node", "source", "edge_ids"]
        self.log_count = 0
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.log_headers)
        self.log_writer.writeheader()
        self.write_log({"action_type": "starting coordinator"})

        """Robot Detail Manage Initialisation"""
        self.robot_manager.add_agents(robot_ids)
        for robot in self.robot_manager.agent_details.values():
            robot.base_station = base_stations[robot.robot_id]
            robot.wait_node = wait_nodes[robot.robot_id]
            robot.max_task_priority = max_task_priorities[robot.robot_id]

        self.system_paused_robots = []
        self.task_pause = False
        self.trigger_replan = False

        logmsg(msg='robots initialised: ' + ', '.join(self.robot_manager.agent_details.keys()))

        # calling from the child class
        self.advertise_services()

        self.max_load_duration = max_load_duration
        self.max_unload_duration = max_unload_duration

        logmsg(msg='coordinator initialised')

    def tray_loaded_ros_srv(self, req):
        """tray_loaded service
        """
        self.robot_manager[req.robot_id].tray_loaded = True

        self.write_log({"action_type": "tray_loaded_srv",
                        "robot_id": req.robot_id,
                        })
        return []

    tray_loaded_ros_srv.type = rasberry_coordination.srv.TrayLoaded

    def set_robot_reached_picker_ros_srv(self, req):
        """Set a robot reached the picker. finish the go_to_picker stage. robot will go to wait_loading
        """
        logmsg(category='ROBOT', id=req.robot_id, msg='robot has reached picker')

        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_manager.agent_details:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)
        else:
            #Identify robot
            robot = self.robot_manager[req.robot_id]

            if robot.moving and robot.task_stage == "go_to_picker":

                robot._reached_picker()
                self.publish_task_state(robot.task_id, req.robot_id, "ARRIVED")

                resp.success = True
                resp.message = "Robot - %s is set to have reached the picker" %(req.robot_id)

            else:
                resp.success = False
                resp.message = "Robot - %s is not in moving_robots now" %(req.robot_id)

        self.write_log({"action_type": "set_robot_reached_picker_srv",
                        "robot_id": req.robot_id,
                        "details": resp.message,
                        })
        return resp

    set_robot_reached_picker_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def set_robot_reached_storage_ros_srv(self, req):
        """Set a robot reached the storage. finish the go_to_storage stage. robot will go to wait_unloading
        """
        logmsg(category='ROBOT', id=req.robot_id, msg='robot has reached storage')

        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_manager.agent_details:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)
        else:
            robot = self.robot_manager[req.robot_id]

            if robot.moving and robot.task_stage == "go_to_storage":
                robot._reached_storage()
                self.publish_task_state(robot.task_id, req.robot_id, "STORAGE")

                resp.success = True
                resp.message = "Robot - %s is set to have reached the storage" %(req.robot_id)

            else:
                resp.success = False
                resp.message = "Robot - %s is not in moving_robots now" %(req.robot_id)

        self.write_log({"action_type": "set_robot_reached_storage_ros_srv",
                        "robot_id": req.robot_id,
                        "details": resp.message
                        })
        return resp

    set_robot_reached_storage_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def connect_robot_ros_srv(self, req):
        """Add robot to system so the coordinator can see it.
        Reject if the robot_id is not listed in the map config...
        or it's viable tasks are not active system tasks...
        or if the robot is already visible...
        or if there is no base_station available.
        """
        logmsg(category="drm", id=req.agent_id, msg='connecting to system')

        if req.agent_id not in self.admissible_robot_ids:
            logmsg(level="warn", category="drm", id=req.agent_id, msg='connection failed, robot not in admissible_robot_ids')
            logmsg(category="drm", msg='admissible robots: ' + ', '.join(self.admissible_robot_ids))
            return {'success': 0, 'msg': 'robot not in admissible_robot_ids'}

        elif not any([task in self.active_tasks for task in req.tasks]):
            logmsg(level="warn", category="drm", id=req.agent_id, msg='connection failed, given tasks are not active in system')
            logmsg(category="drm", msg='active tasks: ' + ', '.join(self.active_tasks))
            return {'success': 0, 'msg': 'given tasks are not active in system'}

        elif req.agent_id in self.robot_manager.agent_details:
            logmsg(level="warn", category="drm", id=req.agent_id, msg='connection failed, robot already connected')
            logmsg(category="drm", msg='connected robots: ' + ', '.join(self.robot_manager.agent_details.keys()))

            robot = self.robot_manager[req.agent_id]
            if req.register and not robot.registered:
                logmsg(level="warn", category="drm", id=req.agent_id, msg='robot no longer pending to unregister')
                self.register_robot_ros_srv(req)
                self.modify_robot_marker(req.agent_id, color='no_color')
                return {'success': 1, 'msg': 'robot already connected, no longer pending to unregister'}
            else:
                return {'success': 1, 'msg': 'robot already connected'}

        elif len(self.available_base_stations) < 1:
            logmsg(level="warn", category="drm", id=req.agent_id, msg='connection failed, no base stations available')
            logmsg(category="drm", msg='base stations in use: ' + ', '.join(self.robot_manager.get_list('base_station')))
            return {'success': 0, 'msg': 'no base stations available'}

        self.connect_robot(req.agent_id)
        if req.register:
            self.register_robot_ros_srv(req)
            self.modify_robot_marker(req.agent_id, color='no_color')
        else:
            self.modify_robot_marker(req.agent_id, color='red')

        # send success response to service
        return {'success': 1, 'msg': 'robot successfully connected'}

    connect_robot_ros_srv.type = rasberry_coordination.srv.ConnectAgent

    def connect_robot(self, robot_id):
        """ initialise newly connected agent
        """
        self.robot_manager.add_agent(robot_id)
        robot = self.robot_manager[robot_id]
        robot.registered = False

        # Set first available base station as taken
        robot.base_station = remove(self.available_base_stations, self.available_base_stations[-1])
        robot.wait_node = remove(self.available_wait_nodes, self.available_base_stations[-1])

        logmsg(category="drm", id=robot_id, msg='assigned to base station %s' % (robot.base_station))
        logmsg(category="drm", msg='base stations in use: %s' % (str(self.robot_manager.get_list('base_station'))))
        logmsg(category="drm", msg='base stations available: %s' % (str(self.available_base_stations)))
        logmsg(category="drm", id=robot_id, msg='connection complete')

    def register_robot_ros_srv(self, req):
        """Register the robot to enable task assignment.
        Reject if the robot_id is listed as a registered robot...
        but only if it is not planned for unregistration.
        """
        logmsg(category="drm", id=req.agent_id, msg='registering for task allocation')
        robot = self.robot_manager[req.agent_id]

        """ Return fail, if robot is not connected or success, if robot is already registered """
        if not robot:
            return {'success': 0, 'msg': 'registration failed, robot is not connected'}
        elif robot.registered:
            return {'success': 1, 'msg': 'unregistration success, robot is already registered'}


        """ When registering, robot can be in 1 of 4 states """
        if robot.unregistration_type is "pause_task":
            robot._unpause_robot()
            #if not robot.task_id and robot._get_start_node() is not robot.base_station: #check empty task list?
            if not robot.task_stage_list:
                robot._set_target_base()
            self.trigger_replan = True
        elif robot.unregistration_type is "release_task":
            """ if robot has not started moving to base do that """
            """ if robot has not reached base do that """
            """ if robot is at base do nout """

            if robot._get_start_node() is not robot.base_station:
                robot._set_target_base()
                self.trigger_replan = True
        elif robot.unregistration_type is "complete_task":
            if not robot.task_id:
                robot.idle = True # TODO: is this error prone?

        elif robot.unregistration_type is "no_task":
            if not robot.task_id:
                robot.idle = True

        """Resister robot, change visual to appear without color modifier, return success"""
        robot.registered = True
        self.modify_robot_marker(req.agent_id, color='no_color')
        return {'success': 1, 'msg': 'robot has registered'}

    register_robot_ros_srv.type = rasberry_coordination.srv.AgentID

    def unregister_robot_ros_srv(self, req):
        """Based on the action included in the request,
        perform the respective action required to unregister the robot.
        """

        actions = {'complete_task': self.unregister_robot_complete_task_ros_srv,
                   'pause_task': self.unregister_robot_pause_task_ros_srv,
                   'release_task': self.unregister_robot_release_task_ros_srv}
        if req.action in actions:
            return actions[req.action](req)
            # return actions[req.action]({'agent_id': req.agent_id})
        return {'success': 0, 'msg': 'action "%s" not acceptable' % str(req.action)}

    unregister_robot_ros_srv.type = rasberry_coordination.srv.UnregisterAgent

    def unregister_robot_complete_task_ros_srv(self, req):
        """Prevent robot from accepting tasks and set to unregister when its current task completed
        Reject if robot_id is not listed as an registered robot...
        or if the robot is already planned for unregistration.
        """
        logmsg(category="drm", id=req.agent_id, msg='unregistering from task allocation')
        robot = self.robot_manager[req.agent_id]

        """ Return fail, if robot is not connected or success, if robot is already unregistered """
        if not robot:
            return {'success': 0, 'msg': 'unregistration failed, robot is not connected'}
        elif not robot.registered:
            return {'success': 1, 'msg': 'unregistration success, robot is already unregistered'}

        """Remove ability to take on tasks"""
        robot.interruptable = False
        robot.unregistration_type = "no_task"
        if robot.task_id:
            robot.unregistration_type = "finish_task"

        """If robot is attempting to disconnect, allow it"""
        if not robot.disconnect_when_idle:
            robot.idle = False

        """If robot is attempting to unregister, set to appear red"""
        robot.registered = False
        self.modify_robot_marker(req.agent_id, color='red')
        return {'success': 1, 'msg': 'robot has unregistered'}

    unregister_robot_complete_task_ros_srv.type = rasberry_coordination.srv.AgentID

    def unregister_robot_release_task_ros_srv(self, req):
        """Prevent robot from accepting tasks and set to unregister
        and release the active task.
        Return success if robot is already unregistered (regardless of how)
        Return failure if robot does not exist.
        If robt does not have task, call unregister_complete_task.
        """

        logmsg(category="drm", id=req.agent_id, msg='unregistering from task allocation canceling any active tasks')
        robot = self.robot_manager[req.agent_id]

        """ Return fail, if robot is not connected or success, if robot is already unregistered """
        if not robot:
            return {'success': 0, 'msg': 'unregistration failed, robot is not connected'}
        elif not robot.registered:  # TODO: Add additional condition to cancel task if unregistered with 'pause_task'
            return {'success': 1, 'msg': 'unregistration success, robot is already unregistered'}
        elif not robot.task_id:
            return self.unregister_robot_complete_task_ros_srv(req)

        """ Notify relevant picker """
        if robot.task_stage in ["go_to_picker", "wait_loading"]:
            self.picker_manager.get_task_handler(robot.task_id).task_abandonded()

        """ Prevent robot from taking new task """
        robot._drm_release_task()
        robot.registered = False

        """ Set to appear red and return success"""
        self.modify_robot_marker(req.agent_id, color='red')
        return {'success': 1, 'msg': 'robot has unregistered'}

    unregister_robot_release_task_ros_srv.type = rasberry_coordination.srv.AgentID

    def unregister_robot_pause_task_ros_srv(self, req):
        """Prevent robot from accepting tasks and set to unregister
        and pause the active task, cancelling any active route.
        Return success if robot is already unregistered (regardless of how)
        Return failure if robot does not exist.
        If robot does not have task, call unregister_complete_task.
        """
        logmsg(category="drm", id=req.agent_id, msg='unregistering from task allocation pausing active tasks')
        robot = self.robot_manager[req.agent_id]

        """ Return fail, if robot is not connected or success, if robot is already unregistered """
        if not robot:
            return {'success':0, 'msg':'unregistration failed, robot is not connected'}
        elif not robot.registered:
            return {'success': 1, 'msg': 'unregistration success, robot is already unregistered'}
        elif not robot.task_id and robot.task_stage != "go_to_base":
            return self.unregister_robot_complete_task_ros_srv(req)

        """ Set to appear red and return success"""
        robot._pause_task()
        robot.registered = False
        self.modify_robot_marker(req.agent_id, color='red')
        return {'success': 1, 'msg': 'robot has unregistered'}

    unregister_robot_pause_task_ros_srv.type = rasberry_coordination.srv.AgentID

    def disconnect_robot_ros_srv(self, req):
        """Remove robot from system.
        Reject if robot_id is not listed as a visible robot.
        Unregister if required.
        """
        logmsg(category="drm", id=req.agent_id, msg='disconnecting from system')

        """Check if robot exists"""
        if req.agent_id not in self.robot_manager.agent_details:
            logmsg(level="warn", category="drm", id=req.agent_id, msg='disconnection failed, robot is not connected')
            logmsg(category="drm", msg='connected robots: ' + ', '.join(self.robot_manager.agent_details.keys()))
            return {'success': 1, 'msg': 'disconnection success, robot is already not connected'}

        """Identify robot and mark as attempting to disconnect"""
        robot = self.robot_manager[req.agent_id]
        robot.disconnect_when_idle = True

        """Unregister robot if registered"""
        if robot.registered:
            self.unregister_robot_complete_task_ros_srv(req)

        """If the robot is idle, disconnect it"""
        if robot.idle:  # TODO: swap out to query if has task_stage
            self.disconnect_robot(robot.robot_id)
            return {'success': 1, 'msg': 'robot successfully disconnected'}
        return {'success': 1, 'msg': 'robot set to disconnect on task completion'}

    disconnect_robot_ros_srv.type = rasberry_coordination.srv.AgentID

    def disconnect_robot(self, robot_id):
        """remove all record of the robot being a member of the system
        """

        """Identify robot to remove"""
        robot = self.robot_manager[robot_id]

        """Extract any useful information"""
        add(self.available_base_stations, robot.base_station)
        add(self.available_wait_nodes, robot.wait_node)

        logmsg(category="drm", msg='base station %s added to pool' % (robot.base_station))
        logmsg(category="drm", msg='base stations in use: ' + ', '.join(self.robot_manager.get_list('base_station')))
        logmsg(category="drm", msg='base stations available: ' + ', '.join(self.available_base_stations))

        """Remove robot"""
        self.robot_manager.remove_agent(robot_id)

        """Remove robot marker"""
        self.clear_robot_marker(robot_id)

        logmsg(category="drm", id=robot_id, msg="disconnection complete")


    def pause_coordinator_ros_srv(self, req):
        logmsg(category="drm", msg='request made to switch pause status of coordinator to pause=%s'%str(req.data))
        AgentIDRequest = rasberry_coordination.srv.AgentIDRequest

        if req.data:
            self.system_paused_robots = [robot for robot in self.robot_manager.agent_details.values() if robot.registered]
            [self.unregister_robot_pause_task_ros_srv(AgentIDRequest(r.agent_id)) for r in self.system_paused_robots]
            self.task_pause = True
            logmsg(category="drm", msg='coordinator has been paused')
            return {'success': 1, 'message': 'coordinator paused'}
        else:
            [self.register_robot_ros_srv(AgentIDRequest(r.agent_id)) for r in self.system_paused_robots]
            self.task_pause = False
            self.trigger_replan = True
            logmsg(category="drm", msg='coordinator has been unpaused')
            return {'success': 1, 'message': 'coordinator unpaused'}

    pause_coordinator_ros_srv.type = std_srvs.srv.SetBool

    def modify_robot_marker(self, robot_id, color=''):
        """Add/modify marker to display in rviz"""
        marker = rasberry_coordination.msg.MarkerDetails()
        marker.type = 'robot'
        marker.name = robot_id
        if color == 'no_color':
            color = ''
        marker.optional_color = color
        self.marker_add_pub.publish(marker)

    def clear_robot_marker(self, robot_id):
        """Modify rviz marker to reflect removal"""
        marker = rasberry_coordination.msg.MarkerDetails()
        marker.type = "robot"
        marker.name = robot_id
        self.marker_remove_pub.publish(marker)

    def send_robot_to_base(self, robot_id):
        """send robot to base. very specific goal.
        happens after task is finished (unloaded trays) or task failed
        also in replan if goal = start node and task is go_to_base
        """
        logmsg(category="robot", id=robot_id, msg='attempting to send to base station')

        robot = self.robot_manager[robot_id]
        if robot._get_start_node(accuracy=True) == robot.base_station:
            logmsg(category="robot", id=robot_id, msg='already at base station')
            # already at base station. set as idle
            robot._set_as_idle()
            logmsg(category="list", msg='idle robots: %s' % (str(self.robot_manager.idle_list())))
        else:
            logmsg(category="robot", id=robot_id, msg='not at base station, setting as target')
            robot._set_target_base()
            logmsg(category="list", msg='active_interruptable_robots: %s' % (str(self.robot_manager.interruptable_list())))
            self.write_log({"action_type": "robot_update",
                            "robot_task_stage": "go_to_base_start",
                            "robot_id": robot_id,
                            "current_node": robot.current_node,
                            "closest_node": robot.closest_node,
                            })

    def find_closest_robot(self, location, priority):
        """find the robot closest to the task location (picker_node)
        # assign based on priority (0 - virtual pickers only, >=1 real pickers)

        Keyword arguments:
            task - strands_executive_msgs.Task
        """
        goal_node = location
        robot_dists = {}

        for robot_id in self.robot_manager.available_robots():
            robot = self.robot_manager[robot_id]

            # ignore if the robot is not actively accepting tasks
            if not robot.registered:
                continue

            # ignore if the robot's closest_node and current_node is not yet available
            if robot.current_node is None and robot.closest_node is None:
                continue

            # ignore if the task priority is less than the min task priority for the robot
            # lower the value, higher the priority
            if priority > robot.max_task_priority:
                continue

            # use current_node as start_node if available
            if robot.current_node is not None:
                start_node = robot.current_node
            else:
                start_node = robot.closest_node

            if start_node is None or goal_node is None or start_node is None or goal_node is None:
                route_dists = [float("inf")]
            elif start_node != goal_node:
                route_nodes, route_edges, route_dists = self.get_path_details(start_node, goal_node)
            else:
                route_dists = [0]

            if abs(sum(route_dists)) != float("inf"):
                robot_dists[robot_id] = sum(route_dists)

        if len(robot_dists) == 0 or min(robot_dists.values()) == float("inf"):
            return None

        return sorted(robot_dists.items(), key=operator.itemgetter(1))[0][0]

    def assign_tasks(self, ):
        """assign task to idle robots
        high priority tasks are assigned first
        among equal priority tasks, low task_id is assigned first
        """
        trigger_replan = False

        locked = self.task_lock.acquire(False)

        if locked:

            """ get list of unassigned tasks """
            tasks = self.picker_manager.get_unassigned_tasks_ordered()

            """ for each task, assign the most effective robot """
            for task_id in tasks:
                picker = self.picker_manager.get_task_handler(task_id)

                """ identify most promising robot """
                robot_id = self.find_closest_robot(picker.task_location, picker.task_priority) #swap out to use picker.location
                if robot_id is None:
                    continue
                robot = self.robot_manager[robot_id]

                logmsg(category="robot", id=robot_id, msg='assigned task %s'%(task_id))
                logmsg(category="list", msg='interruptable robots: %s'%(str(self.robot_manager.interruptable_list())))
                logmsg(category="list", msg='idle robots: %s'%(str(self.robot_manager.idle_list())))

                """ enable planing for task """
                trigger_replan = True

                """ prepare robot to take task """
                robot._begin_task(task_id)

                # self.processing_tasks[task_id] = picker.task_backup
                robot.goal_node = picker.task_location

                self.update_current_storage(robot_id)

                self.publish_task_state(task_id, robot_id, "ACCEPT")

                self.write_log({"action_type": "robot_update",
                                "robot_task_stage": "go_to_picker_start",
                                "task_id": task_id,
                                "robot_id": robot_id,
                                "details": "to %s" %(picker.task_location),
                                "current_node": robot.current_node,
                                "closest_node": robot.closest_node,
                                })

            self.task_lock.release()

        self.trigger_replan = self.trigger_replan or trigger_replan

    def update_current_storage(self, robot_id):
        """set the current_storage node of the robot. If the robot is idle,
        this is None. If use_cold_storage is set, it is cold_storage node.
        Else, it is the local storage nearest  to the picker_node

        Keyword arguments:
            robot_id -- robot_id
        """
        robot = self.robot_manager[robot_id]
        if not robot.active:
            robot.current_storage = None
        elif self.use_cold_storage:
            robot.current_storage = self.cold_storage
        else:
            # get the closest local storage near the picker location
            # task = self.processing_tasks[robot.task_id]
            picker = self.picker_manager.get_task_handler(robot.task_id)
            picker_node = picker.task_location
            min_dist = float("inf")
            for storage in self.local_storages:
                _, _, dists = self.get_path_details(picker_node, storage)
                if sum(dists) < min_dist:
                    robot.current_storage = storage

    def finish_task(self, robot_id):
        """set the task assigned to the robot as finished whenever trays are unloaded
        """
        robot = self.robot_manager[robot_id]

        # move task from processing to completed
        task_id = robot.task_id
        self.completed_tasks.append(task_id)
        robot.goal_node = None

        # mark task as complete
        robot._tray_unloaded()

        """ Inform TOC of task completion """
        self.inform_toc_task_ended(task_id=task_id, reason="task_completed", robot_id=robot_id)

        self.write_log({"action_type": "task_update",
                        "task_updates": "task_finish",
                        "task_id": task_id,
                        "robot_id": robot_id,
                        })

    def task_update(self, update, task_id, details):
        """ If the pickers location has updated while robot is navigating to them
        this function is called to update the robots goal and trigger a replan.
        """
        if update == "picker_node_update":
            robot = self.robot_manager.get_task_handler(task_id)
            if robot: #and robot.task_stage == "go_to_picker":
                robot.goal_node = details
            self.trigger_replan = True

    # def set_task_failed(self, task_id):
    #     """set task state as failed
    #     """
    #     # remove(self.failed_tasks, task_id)
    #     robot = self.robot_manager.get_task_handler(task_id)
    #     robot.goal_node = None
    #
    #     self.write_log({"action_type": "task_update",
    #                     "task_updates": "task_failed",
    #                     "task_id": task_id,
    #                     "details": "Assigned robot failed to complete task after reaching picker",
    #                     })

    def publish_task_state(self, task_id, robot_id, state):
        """publish the state of task (or picker) in CAR
        """
        self.picker_manager.task_updates(picker_id='',task_id=task_id,robot_id=robot_id,state=state)
        rospy.sleep(0.01)

        self.write_log({"action_type": "car_update",
                        "task_updates": state,
                        "task_id": task_id,
                        "robot_id": robot_id,
                        })

    def readd_task(self, task_id):
        """if robot assigned to a task fails before reaching the picker, the task can be
        reassigned. so readd into the task queue.
        """

        """ Identify handlers for task """
        robot = self.robot_manager.get_task_handler(task_id)
        picker = self.picker_manager.get_task_handler(task_id)

        """ Readd task back to queue if not cancelled """
        # if cancelled, no picker will be found by get task handler as cancelling task removes the task details
        if picker: #TODO: this needs through testing
            logmsg(category="task", id=task_id, msg='picker still requires task completion, task added back to queue')
            logmsg(category="robot", id=robot.agent_id, msg='failed to reach picker')
            robot.goal_node = None

            self.publish_task_state(task_id, "", "ABANDONED")

            self.write_log({"action_type": "task_update",
                            "task_updates": "readd_task",
                            "task_id": task_id,
                            "details": "Assigned robot failed to reach picker, adding the task back into the queue",
                            })

        rospy.sleep(0.01)

    def handle_tasks(self):
        """update task execution progress for all robots
        """
        # trigger replan if there is a new task assignment, a robot waiting completed
        # or task update from a robot
        trigger_replan = False

        for robot_id in self.robot_manager.active_list():
            robot = self.robot_manager[robot_id]
            task_id = robot.task_id

            if robot.moving:
                # topo nav stage
                # if any robot has finished its current goal, remove the finished goal from the robot's route
                if robot.robot_interface.execpolicy_result is None:
                    # task/fragment not finished
                    continue

                # check for robots which are moving, not waiting before a critical point
                if robot.robot_interface.execpolicy_result.success:

                    """ Trigger replan whenever a segment is completed """
                    trigger_replan = True

                    """ Robot has reached goal_node or wait_node """
                    if robot._get_start_node() == robot._get_goal_node():

                        # identify completed stage
                        completed_stage = robot.task_stage

                        # set robot to finish stage
                        if completed_stage in ["go_to_picker", "go_to_storage", "go_to_base"]:
                            logmsg(category="robot", id=robot_id, msg='%s stage is finished' % (robot.task_stage))

                        # set picker to finish the stage
                        if completed_stage == "go_to_picker":
                            robot._reached_picker()
                            self.publish_task_state(task_id, robot_id, "ARRIVED")

                        elif completed_stage == "go_to_storage":
                            robot._reached_storage()
                            self.publish_task_state(task_id, robot_id, "STORAGE")

                        elif completed_stage == "go_to_base":
                            robot._end_task()
                            logmsg(category="list", msg='idle robots: %s' % (str(self.robot_manager.idle_list())))
                            if robot.disconnect_when_idle:
                                self.disconnect_robot(robot_id)
                    elif robot._get_start_node() == robot.wait_node:
                        # finished only a fragment. may have to wait for clearance
                        robot._finish_route_fragment()
                    else:
                        robot._finish_route_fragment()

                else:
                    #if robot is idle and hasnt completed its task stage
                    logmsg(category="robot", id=robot_id, msg='execpolicy_result is not success or None (%s)'%(robot.robot_interface.execpolicy_result))
                    logmsg(category="robot", id=robot_id, msg='current task stage: %s'%(robot.task_stage))

                    #if robot has failed to complete task
                    trigger_replan = True
                    if robot.task_stage == "paused":
                        logmsg(category="robot", id=robot_id, msg='unpausing task at stage %s'%(robot.task_stage_list[0]))
                    else:
                        logmsg(category="robot", id=robot_id, msg='failed to complete task %s at stage %s' % (task_id, robot.task_stage))

                    # robot has either failed to complete its task, or it has paused its task
                    if robot.task_stage == "paused":
                        logmsg(category="robot", id=robot_id, msg='task unpaused')
                        robot._unpause_task()

                    elif robot.task_stage == "go_to_picker":
                        # task is good enough to be assigned to another robot
                        self.readd_task(task_id)
                        self.send_robot_to_base(robot_id)

                    elif robot.task_stage == "go_to_base":
                        # robot failed exececute_policy_mode goal. retry going to base
                        self.send_robot_to_base(robot_id)
                        # another option is to leave the robot out there with a request for help. (# TODO)
                        # in that case, remove robot from active_robots and don't add to idle_robots.

                    else:
                        # set the task as failed as it cannot be readded at this stage
                        if task_id not in self.cancelled_tasks or task_id not in self.failed_tasks :
                            self.set_task_failed(task_id)
                            logmsg(msg='set task as failed')
                        self.send_robot_to_base(robot_id)

            else:

                # robot has either failed to complete its task, or it has paused its task
                if robot.task_stage == "paused":
                    logmsg(category="robot", id=robot_id, msg='task unpaused')
                    robot._unpause_task()

                # wait_loading or wait_unloading
                elif robot.task_stage == "wait_loading":
                    """ If conditions are satisfied, finish waiting """
                    tray_loaded = False

                    """ If wait timeout has expired complete task """  # abandon task? """
                    if rospy.get_rostime() - robot.start_time > self.max_load_duration:
                        tray_loaded = True

                    """ Update task completeness of robot """
                    picker = self.picker_manager.get_task_handler(robot.task_id)
                    if picker and picker.task_stage in ["LOADED"]:
                        tray_loaded = True

                    """ If the robot has been loaded """
                    if tray_loaded:
                        logmsg(category="robot", id=robot_id, msg='%s stage is finished' % (robot.task_stage))
                        self.publish_task_state(task_id, robot_id, "LOADED")    #update picker
                        robot._tray_loaded() #update robot
                        trigger_replan = True

                        """ Release picker from task """
                        if picker:
                            picker.task_finished()

                elif robot.task_stage == "wait_unloading":
                    """ If conditions are satisfied, finish waiting """
                    tray_unloaded = True

                    """ If wait timeout has expired, complete task """
                    if rospy.get_rostime() - robot.start_time > self.max_unload_duration:
                        tray_unloaded = False
                    # else:  # TODO: should we really be including artificial delay in the coordination?
                    #     rospy.sleep(0.5)

                    """ If the robot has been unloaded """
                    if not tray_unloaded:
                        logmsg(category="robot", id=robot_id, msg='%s stage is finished' % (robot.task_stage))
                        self.publish_task_state(task_id, robot_id, "DELIVERED")
                        robot._tray_unloaded()
                        self.inform_toc_task_ended(task_id=task_id, robot_id=robot_id, reason="task_completed")

                        trigger_replan = True

                else:
                    # robot is waiting before a critical point
                    if self.robot_manager.moving_robots_exist():
                        # no other moving robots - replan
                        trigger_replan = True
                    else:
                        # wait for a moving robot to finish its route fragment
                        pass

        self.trigger_replan = self.trigger_replan or trigger_replan

    def set_execute_policy_routes(self, ):
        """find connecting edges for each fragment in route_fragments and set
        the corresponding route object (with source and edge_id)
        """
        for robot_id in self.robot_manager.active_list():
            robot = self.robot_manager[robot_id]
            goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
            if robot.route_fragments:
                goal.route.source = robot.route_fragments[0]
                goal.route.edge_id = robot.route_edges[0]

            if goal != robot.robot_interface.execpolicy_goal:
                same_route = True

                if goal.route.source and robot.robot_interface.execpolicy_goal.route.source:
                    # if there is at least one source node
                    if goal.route.edge_id[-1] != robot.robot_interface.execpolicy_goal.route.edge_id[-1]:
                        # goal edge_ids are different
                        same_route = False
                    else:
                        # loop from new start source node to end source node
                        new_start_node = goal.route.source[0]

                        # look for new_start_node in previous goal
                        idx = 0
                        change_idx = False
                        for i in range(len(robot.robot_interface.execpolicy_goal.route.source)):
                            if new_start_node == robot.robot_interface.execpolicy_goal.route.source[i]:
                                # found current start source node in previous source nodes
                                if len(robot.robot_interface.execpolicy_goal.route.source) - i != len(goal.route.source):
                                    # remaining nodes in the routes are different => different route
                                    same_route = False
                                    break
                                change_idx = True # enable comparing source nodes here onwards

                            if change_idx:
                                if goal.route.source[idx] != robot.robot_interface.execpolicy_goal.route.source[i]:
                                    # different node => different route
                                    same_route = False
                                    break
                                else:
                                    # check next node
                                    idx += 1
                else:
                    # either new or old source nodes are empty
                    same_route = False

                if not same_route:
                    # publish new route only if different
                    if self.log_routes:
                        logmsg(category="robot", id=robot_id, msg='new route %s, previous route was %s' % (goal, robot.robot_interface.execpolicy_goal))

                    self.write_log({"action_type": "robot_update",
                            "details": "new_route",
                            "source": str(goal.route.source),
                            "edge_ids": str(goal.route.edge_id),
                            "robot_id": robot_id,
                            "current_node": robot.current_node,
                            "closest_node": robot.closest_node,
                            })

                    robot.robot_interface.set_execpolicy_goal(goal)

                    """ if a route exists, mark robot as moving"""
                    if goal.route.edge_id:
                        robot.moving = True

    def run(self, planning_type='fragment_planner'):
        """the main loop of the coordinator
        """
        routing_cb = {'publish_task_state': self.publish_task_state,
                      'send_robot_to_base': self.send_robot_to_base}
        self.route_finder = RouteFinder(planning_type=planning_type,
                                        robots=self.robot_manager,
                                        pickers=self.picker_manager,
                                        callbacks=routing_cb)

        #TOC callback timeout
        iterations = 100

        while not rospy.is_shutdown():
            rospy.sleep(0.01)  # TODO: look into methods to remove the artificial delay
            if self.task_pause: continue

            """ update TOC with latest tasks states """
            iterations=iterations-1
            if iterations < 0:
                self.inform_toc_active_tasks()
                iterations = 100

            """ if there are unassigned tasks and there robots able to take them on """
            available_robots = self.robot_manager.available_robots()
            unassigned_tasks = self.picker_manager.unassigned_tasks()
            if available_robots and unassigned_tasks:
                logmsg(category="task", msg='unassigned task present, %s robots available' % (len(available_robots)))
                logmsg(category="list", msg='available robots: %s' % (str(available_robots)))
                # try to assign all tasks
                rospy.sleep(0.2)
                self.assign_tasks()

            """ check progress of active robots """
            self.handle_tasks()

            """ replan if needed """
            if self.trigger_replan:
                self.route_finder.find_routes()

                self.trigger_replan = False

                # assign first fragment of each robot
                self.set_execute_policy_routes()

    def write_log(self, field_vals):
        """write given fileds to the log file
        """
        time_now = time.time()
        dt = datetime.datetime.fromtimestamp(time_now)
        row = {"id": self.log_count,
               "time":time_now,
               "datetime": dt.strftime("%Y%m%d-%H%M%S"),
               }
        row.update(field_vals)
        if not self.log_file.closed:
            self.log_writer.writerow(row)
            self.log_count += 1

    def on_shutdown(self, ):
        """on shutdown cancel all goals
        """
        self.write_log({"action_type":"shutdown coordinator"})
        self.log_file.close()

        logmsg(level='warn', msg='shutting down all actions')
        for robot in self.robot_manager.agent_details.values():
            if robot.active:
                robot.robot_interface.cancel_execpolicy_goal()
                robot.robot_interface.cancel_toponav_goal()

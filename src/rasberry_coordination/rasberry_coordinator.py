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
# import rasberry_coordination.coordinator
from rasberry_coordination.msg import MarkerDetails, KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak, remove, add, move

#Route Planning
from rasberry_coordination.route_planners.route_planners import RouteFinder

#Task Managment
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef

#Agent Management
# from rasberry_coordination.agent_managers.robots import RobotManager
# from rasberry_coordination.agent_managers.humans import CrewManager#, StorageManager #(Since store managed by human)
# from rasberry_coordination.agent_managers.stores import StorageManager
from rasberry_coordination.agent_managers.agents import AgentManager


class RasberryCoordinator():
    """RasberryCoordinator class definition
    """
    def __init__(self, agent_list, base_station_nodes_pool, wait_nodes_pool, planning_type, ns, special_nodes):

        """ Meta Fields """
        self.ns = ns.strip("/") + "/"
        self.is_parent = True
        self.trigger_replan = False
        self.log_count = 0


        """ Initialise Task Parameters: """ #This should be done within Stages.py programatically
        """
        self.active_tasks = active_tasks
        self.max_load_duration = max_load_duration
        self.max_unload_duration = max_unload_duration
        self.max_task_priority = max_task_priority
        """

        """ Initialise System Details: """
        self.special_nodes = special_nodes
        self.base_station_nodes_pool = base_station_nodes_pool
        self.wait_nodes_pool = wait_nodes_pool

        # Cold storage node is an agnet now, and should be defined as such
        """
        self.use_cold_storage = use_cold_storage
        self.cold_storage_node = cold_storage_node
        """

        """ Initialise Agents: """
        callbacks = {'update_topo_map': None, 'task_cancelled': self.task_cancelled} #This should not exist

        #Define Managers
        self.agent_manager = AgentManager(callbacks)
        self.agent_manager.add_agents(agent_list)
        self.AllAgentsList = self.get_all_agents()


        """ Routing Details """
        routing_cb = {'publish_task_state': self.publish_task_state,
                      'send_robot_to_base': self.send_robot_to_base}  # These need to be eventually managed better
        self.route_finder = RouteFinder(planning_type=planning_type,
                                        agents=self.AllAgentsList,
                                        callbacks=routing_cb)


        """ Communications Setup """
        self.advertise_services()

        # Initialise topics for marker management #Combine these into 1 topic
        self.marker_add_pub = rospy.Publisher('/rasberry_coordination/marker_add', MarkerDetails, queue_size=5)
        self.marker_remove_pub = rospy.Publisher('/rasberry_coordination/marker_remove', MarkerDetails, queue_size=5)

        """ TOC Communications """
        self.active_tasks_pub = rospy.Publisher(self.ns + "active_tasks_details",
                                                rasberry_coordination.msg.TasksDetails, latch=True, queue_size=5)


        logmsg(msg='coordinator initialised')
        return

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

    def modify_robot_marker(self, robot_id, color=''):
        # Add/modify marker to display in rviz
        marker = rasberry_coordination.msg.MarkerDetails()
        marker.type = 'robot'
        marker.name = robot_id
        if color == 'no_color':
            color = ''
        marker.optional_color = color
        self.marker_add_pub.publish(marker)

    def clear_robot_marker(self, robot_id):
        # Modify rviz marker to reflect removal
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
        if robot.current_node == robot.base_station:
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
            if robot.current_node == None and robot.closest_node == None:
                continue

            # ignore if the task priority is less than the min task priority for the robot
            # lower the value, higher the priority
            if priority > robot.max_task_priority:
                continue

            # use current_node as start_node if available
            if robot.current_node != None:
                start_node = robot.current_node
            else:
                start_node = robot.closest_node

            if start_node == None or goal_node == None or start_node is None or goal_node is None:
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

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_topo_map = True

    def finish_task(self, robot_id): #TODO: when is this called from?
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
                    if not self.robot_manager.moving_robots_exist():
                        # no other moving robots - replan
                        trigger_replan = True
                    else:
                        # should wait until one of the moving robots to finish its route fragment
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

    # def run(self, planning_type='fragment_planner'):
    #     self.run_new(planning_type)
    # def run_old(self, planning_type='fragment_planner'):
    #     """the main loop of the coordinator
    #     """
    #     routing_cb = {'publish_task_state': self.publish_task_state,
    #                   'send_robot_to_base': self.send_robot_to_base}
    #     self.route_finder = RouteFinder(planning_type=planning_type,
    #                                     robots=self.robot_manager,
    #                                     pickers=self.picker_manager,
    #                                     callbacks=routing_cb)
    #
    #     #TOC callback timeout
    #     iterations = 100
    #
    #     while not rospy.is_shutdown():
    #         rospy.sleep(0.01)  # TODO: look into methods to remove the artificial delay
    #
    #         """ update TOC with latest tasks states """
    #         iterations=iterations-1
    #         if iterations < 0:
    #             self.inform_toc_active_tasks()
    #             iterations = 100
    #
    #         """ if there are unassigned tasks and there robots able to take them on """
    #         available_robots = self.robot_manager.available_robots()
    #         unassigned_tasks = self.picker_manager.unassigned_tasks()
    #         if available_robots and unassigned_tasks:
    #             logmsg(category="task", msg='unassigned task present, %s robots available' % (len(available_robots)))
    #             logmsg(category="list", msg='available robots: %s' % (str(available_robots)))
    #             # try to assign all tasks
    #             rospy.sleep(0.2)
    #             self.assign_tasks()
    #
    #         """ check progress of active robots """
    #         self.handle_tasks()
    #
    #         """ replan if needed """
    #         if self.trigger_replan:
    #             self.route_finder.find_routes()
    #
    #             self.trigger_replan = False
    #
    #             # assign first fragment of each robot
    #             self.set_execute_policy_routes()

    def write_log(self, field_vals):
        """write given fileds to the log file
        """
        return
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
        # self.log_file.close()

        logmsg(level='warn', msg='shutting down all actions')
        # for robot in self.courier_manager.agent_details.values():
        #     if robot.active:
        #         robot.interface.cancel_execpolicy_goal()
        #         robot.interface.cancel_toponav_goal()


    """ Main loop for task progression """
    def run(self, planning_type='fragment_planner'):
        self.run_minimalist()
    def run_bulk(self, planning_type='fragment_planner'):
        """ For details see: rasberry_coordination/src/CoordinatorStructure.md

            Notation Worth Noting:
                - Specific agents are represented by the variable A
                - An active task is identified by __call__, e.g. A()
                - Task details can be identified by __getitem__, e.g. A[key]
                - Task details can be set using __setitem__, e.g. A[key]=val

            Key Concepts:
                - Approach requires no lock as task progression uses linear buffers
        """

        self.AllAgentsList = self.get_all_agents()
        self.enable_task_logging = True
        self.log_routes = False
        self.task_progression_log = '/home/jheselden/task_progression.csv'
        self.timestep = 0
        self.iteration = 0
        self.previous_log_iteration = ""
        self.current_log_iteration = ""
        self.log_data(['init'])

        while not rospy.is_shutdown():

            #TODO: add extra condition to only progress if there are any tasks which require updates?

            """ Get list of all currently connected agents """
            self.AllAgentsList = self.get_all_agents()

            self.log_data(['linebreak','iteration'])
            self.enable_task_logging = True


            """ Default to IDLE if no task present """
            # self.log_data(['task', 'stage'])
            for A in self.AllAgentsList.values():
                A.start_idle_task() if not A.task_stage_list else None
            #self.log_data(['new_stage','stage','break'])
            self.log_data(['task', 'stage', 'new_stage','break'])

            """ Perform stage initialisation and communication """
            self.log_data(['_start','_notify_start','break'])
            # self.log_data(['_start','_notify_start'])
            for A in self.AllAgentsList.values():
                A()._start() if A().new_stage else None
                A()._notify_start() if A().new_stage else None
                A().new_stage = False
            # self.log_data(['break'])

            """ Perform agent-specific request """
            # self.log_data(['coordinator_action_required','_action'])
            for A in self.AllAgentsList.values():
                if A['coordinator_action_required']:
                    logmsg(category="action", id=A.agent_id, msg="%s : %s" % (A.agent_id, A.task_stage_list))
                    print({a.agent_id:a['coordinator_action_required'] for a in self.AllAgentsList.values()})
                self.offer_service(A) if A['coordinator_action_required'] else None
            # self.log_data(['break'])
            self.log_data(['_action','break'])

            """ Perform multi-agent request """
            self.log_data(['replan_required'])
            if any([A['replan_required'] for A in self.AllAgentsList.values()]):
                self.route_finder.find_routes()

            """ Publish new routes """
            for A in [A for A in self.AllAgentsList.values() if A['replan_required']]:
                self.execute_policy_routes(A) #TODO: abstract this for generalisation #A.interface.set_execute_policy_routes()
            self.log_data(['route','break'])

            """ Query if stage completion criteria is met """
            for A in self.AllAgentsList.values():
                A()._query()
            # self.log_data(['_query','stage_complete_flag','break'])
            self.log_data(['_query','break'])

            """ Complete stage where needed """
            self.log_data(['_notify_end','_del'])
            for A in self.AllAgentsList.values():
                if A['stage_complete_flag']:
                    A()._notify_end()
                    A.end_stage()  # calls __del__

            """ Publish log if any updates occured this round """
            if self.enable_task_logging:
                self.publish_log()

    def run_minimalist(self):
        #
        offer_service  = self.offer_service
        l              = self.log_minimal
        find_routes    = self.route_finder.find_routes
        publish_routes = self.execute_policy_routes
        get_agents     = self.get_agents

        A = self.get_all_agents()
        self.enable_task_logging = True
        self.task_progression_log = '/home/jheselden/task_progression.csv'
        self.log_routes = False
        self.timestep = 0
        self.iteration = 0
        self.previous_log_iteration = ""
        self.current_log_iteration = ""

        l(-1)
        while not rospy.is_shutdown():
            A = get_agents()
            logmsg() if any([not a.task_stage_list for a in A]) else None
            [a.start_next_task() for a in A if not a.task_stage_list];             """ Start Buffered Task """
            l(0);
            logmsg() if any([a().new_stage for a in A]) else None
            [a.start_stage()    for a in A if a().new_stage];                      """ Start Stage """
            logmsg() if any([a().action_required for a in A]) else None
            [offer_service(a)   for a in A if a().action_required];                """ Offer Service """
            l(2)

            logmsg() if any([a().route_required for a in A]) else None
            find_routes()       if any([a().route_required for a in A]) else None; """ Find Routes """
            [publish_routes(a)  for a in A if a().route_required];                 """ Publish Routes """
            l(3)
            [a()._query()       for a in A];                                       """" Query """
            l(4)
            logmsg() if any([a().stage_complete for a in A]) else None
            [a.end_stage()      for a in A if a().stage_complete];                 """ End Stage """
            l(-2) #publish route

    def get_all_agents(self):
        # managers = self.agent_managers.values()
        # all_agents = managers[0].agent_details.copy()
        # for manager in managers[1:]:
        #     all_agents.update(manager.agent_details)
        # return all_agents
        return self.agent_manager.agent_details.copy()
    def get_agents(self):
        self.AllAgentsList = self.get_all_agents()
        return self.AllAgentsList.values()

    """ Services offerd by Coordinator to assist with tasks """
    def offer_service(self, agent):
        action_type =       agent().action['action_type']
        action_style =      agent().action['action_style']
        response_location = agent().action['response_location']

        responses = {'find_agent':self.find_agent,
                     'find_node': self.find_node,
                     'find_agent_from_list': self.find_agent_from_list}  # ROOM TO EXPAND

        # logmsg(category="action", id=agent.agent_id, msg="Action: %s" % str(agent().action))

        action_deets = agent().action.copy()
        del action_deets['action_type']
        del action_deets['action_style']
        del action_deets['response_location']
        logmsg(category="action", id=agent.agent_id,
               msg="Perfoming %s(%s_%s) - details: %s" \
                   % (action_type, action_style, response_location, action_deets))


        responses[action_type](agent)

        if agent.task_pointers[response_location]:
            logmsg(category="action", msg="Found %s: %s" % (response_location, agent[response_location]))
            agent().action_required = False

    """ Action Category """
    def find_agent(self, agent):
        action_style =      agent().action['action_style']
        response_location = agent().action['response_location']
        agent_type = agent().action['agent_type']

        A = {a.agent_id:a for a in self.AllAgentsList.values() if (a is not agent) and (agent_type in a.roles)}

        responses = {"closest": self.find_closest_agent}  # ROOM TO EXPAND
        agent.task_pointers[response_location] = responses[action_style](agent, A)

    def find_node(self, agent):
        action_style =      agent().action['action_style']
        response_location = agent().action['response_location']
        descriptor =        agent().action['descriptor']
        # print("\n\n\n")
        logmsg(category='action', msg='Finding %s unoccupied node to: %s'%(action_style,agent.location()))
        taken = [a.current_node for a in self.AllAgentsList.values() if
                 (a.agent_id is not agent.agent_id) and a.current_node is not None] #Check if node is occupied
        logmsg(category='action', msg='Occupied Nodes: %s'%taken)
        # print("\n\n\n-----")
        # print(self.special_nodes)
        # print({n['id']:n for n in self.special_nodes})
        N = {n['id']:n for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in taken)}
        logmsg(category='action', msg='Nodes to Compare Against:')
        [logmsg(category='action', msg="    - %s: %s"%(n,N[n])) for n in N]

        # print("nodeTaken: %s" % taken)
        # print("node2search: %s" % N)
        # print("\n\n\n")
        responses = {"closest": self.find_closest_node}  # ROOM TO EXPAND
        agent.task_pointers[response_location] = responses[action_style](agent, N)
        # print("\n\n\n")

    def find_agent_from_list(self, agent):
        action_style =      agent().action['action_style']
        response_location = agent().action['response_location']
        agent_list =        agent().action['list']

        A = {agent_id:self.AllAgentsList[agent_id] for agent_id in agent_list} #convert list to set()

        responses = {"closest": self.find_closest_agent}  # ROOM TO EXPAND
        agent.task_pointers[response_location] = responses[action_style](agent, A)

    """ Action Style """
    def find_closest_agent(self, agent, agent_list):
        """ Find the closest agent (via optimal route) to the given agent.

        :param agent: The agent to query against.
        :param agent_list: The list of agents to query.
        :return: The agent_details object closest to the agent querying against.
        """
        dist_list = {a.agent_id:self.dist(agent.location(),a.location()) for a in agent_list.values()}
        logmsg(category="action", msg="Finding closest in: %s" % dist_list)
        return agent_list[min(dist_list, key=dist_list.get)]
    def find_closest_node(self, agent, node_list):
        """ Find the closest node (via optimal route) to the given agent.

        :param agent: The agent to query against.
        :param node_list: The list of nodes to query.
        :return: The node_id closest to the agent querying against.
        """
        dist_list = {n:self.dist(agent.location(),n) for n in node_list}
        logmsg(category="action", msg="Finding closest in:")
        [logmsg(category='action', msg="    - %s: %s"%(n,dist_list[n])) for n in dist_list]
        return min(dist_list, key=dist_list.get)

    """ Find Distance """
    def dist(self, start_node, goal_node):
        _,_,route_dists = self.get_path_details(start_node, goal_node)
        return sum(route_dists)


    """ Publish route if different from current """
    def execute_policy_routes(self, agent):
        # print("/\/\/\/\/\/\/\/\/\/\/\/\/")
        # print(agent.agent_id)
        # print(" ")

        """ Publish ExecutePolicyModeGoal if different from current policy """
        policy = strands_navigation_msgs.msg.ExecutePolicyModeGoal()

        """ Define route, if no new route is generated, dont do anything. """
        policy.route.source = agent.route_fragments[0] if agent.route_fragments else None
        policy.route.edge_id = agent.route_edges[0] if agent.route_edges else None
        #
        # if agent.route_fragments and agent.route_edges:
        #     policy.route.source = agent.route_fragments[0]
        #     policy.route.edge_id = agent.route_edges[0]
        # else:
        #     return

        """ Flag to identify if new route is the same and should not be re-published """
        check_route = True

        """ Identify key elements in routes. """
        old_node = agent.temp_interface.execpolicy_goal.route.source
        old_edge = agent.temp_interface.execpolicy_goal.route.edge_id
        new_node = policy.route.source
        new_edge = policy.route.edge_id

        # if not new_node:
        #     print("no new route, why are we here then? just to stop the robot?")
        # else:
        #     print("new route exists, we should make sure it doesnt mess anything up")
        # if not old_node:
        #     print("no existing route, we should just publish whatever we have")
        # else:
        #     print("route exists, we should make sure we dont mess it up")

        """ If no new route is generated, dont do anything. """
        if (not new_node) or (not new_edge):
            return

        """ If old route exists, check against it """
        if old_node:

            """ Identify key elements in routes. """
            old_start_node = old_node[0]
            old_start_edge = old_edge[0]
            old_target_node = old_node[-1]
            old_target_edge = old_node[-1]
            new_start_node = new_node[0]
            new_start_edge = new_edge[0]
            new_target_node = new_node[-1]
            new_target_edge = new_node[-1]

            """ Do lists have different entrances to the target node? """
            # old: R========T
            # new:          T=====R
            if check_route and new_target_edge != old_target_edge:
                check_route = False

            """ Do lists have different lengths? """
            # old: R========T
            # new:     R====T
            if check_route and len(new_edge) != len(old_edge):
                # If new_route is larger, routes are different
                if len(new_edge) > len(old_edge):
                    check_route = False
                else:
                    """ Go backwards from target till smaller route is used up. """
                    # old: R========T
                    # new:     R====T
                    old_edge_crop=[]
                    for i, e in enumerate(list(zip(*(old_start_edge[::-1],new_start_edge[::-1])))):
                        old_edge_crop.append(e[0])
                    old_edge_crop.reverse()
                    old_edge = old_edge_crop

            """ Do same-sized routes differ? """
            # old: ****R====T
            # new:     R=-_=T
            if check_route:
                for i, e in enumerate(list(zip(*(old_edge,new_edge)))):
                    if e[0] != e[1]:
                        logmsg(category="route", id=agent.agent_id, msg="New route different from existing route")
                        check_route = False
                        break

        """ If check_route is false, routes are different """
        if check_route:
            if self.log_routes:
                print(policy)
            agent.temp_interface.set_execpolicy_goal(policy)
            agent().route_required = False
            if self.log_routes:
                logmsg(category="route", id=agent.agent_id,
                       msg='new route %s, previous route was %s' % (policy, agent.temp_interface.execpolicy_goal))

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:

        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_finder.planner.route_search.search_route(start_node, goal_node)
        if route is None:
            if start_node == goal_node:
                logmsg(category="route", msg='start_node %s is goal_node %s' % (start_node, goal_node))
                return ([], [], [0])
            else:
                logmsg(category="route", msg='no route between %s and %s' % (start_node, goal_node))
                return ([], [], [float("inf")])
        route_nodes = route.source
        route_nodes.append(goal_node)
        route_edges = route.edge_id

        for i in range(len(route_nodes) - 1):
            route_distance.append(self.route_finder.planner.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distance)

    """ Task Stage Logging """
    def log_linebreak(self):
        dash_lengths = [13, 13, 36, 20, 3]
        dashes = []
        dashes += [',|,'.join(['-'*dl for dl in dash_lengths[:3]])]
        dashes += [',|,'.join(['-'*dash_lengths[3] for a in range(len(self.AllAgentsList))])]
        dashes += ['-' *dash_lengths[4]]
        return dashes
    def log_init(self):
        with open(self.task_progression_log, 'w+') as log:
            log.write(' ,'*5+"|,Agents:\n")

        return ['Timestep','Iteration','Stage']+[a.agent_id for a in self.AllAgentsList.values()]
    def log_break(self):
        return ['' for a in range(3+len(self.AllAgentsList))]
    def log_iteration(self):
        return ['%s','%s']+['' for a in range(1 + len(self.AllAgentsList))]

    def log_value(self, detail):
        return ['','']+[a[detail] for a in self.AllAgentsList.values()]
    def log_not_none(self, detail):
        return ['', ''] + [a[detail] is not None for a in self.AllAgentsList.values()]

    def log_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a.task_stage_list:
                stages += [a().get_class()]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_new_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a.task_stage_list:
                stages += [a().new_stage]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_task(self):
        return ['', ''] + [a.task_name for a in self.AllAgentsList.values()]

    def log_summary(self, detail):
        lst=[]
        for a in self.AllAgentsList.values():
            switch = {'_start':a().new_stage,
                      '_notify_start':a().new_stage,
                      '_action':a().action_required,
                      '_query':a().stage_complete,
                      '_notify_end':a().stage_complete,
                      '_del':a().stage_complete}
            if switch[detail]:
                lst += [a().summary[detail]]
            else:
                lst += ['-']
        return ['','']+lst

    def log_data(self, switches, detail=None, linebreak=False):
        if not self.enable_task_logging:
            return

        switch_group_empty = {'init':self.log_init,
                              'break':self.log_break,
                              'iteration':self.log_iteration, #Make timestep query if time since last post has exceeded T
                              'task':self.log_task,
                              'stage': self.log_stage,
                              'new_stage':self.log_new_stage,
                              'linebreak': self.log_linebreak}
        switch_group_value = {'route':self.log_not_none,          #('route')
                              'action_required':self.log_value,   #('action_required')
                              'route_required':self.log_value,    #('route_required')
                              'stage_complete':self.log_value}    #('stage_complete')
        switch_group_summary = {'_start':self.log_summary,        #('_start')
                                '_notify_start':self.log_summary, #('_notify_start')
                                '_action':self.log_summary,       #('_action')
                                '_query':self.log_summary,        #('_query')
                                '_notify_end':self.log_summary,   #('_notify_end')
                                '_del':self.log_summary}          #('_del')


        for switch in switches:
            details = switch_group_empty[switch]() if switch in switch_group_empty else None
            details = switch_group_value[switch](switch) if switch in switch_group_value else details
            details = switch_group_summary[switch](switch) if switch in switch_group_summary else details

            for idx, item in enumerate(details[2:]):
                if item in [False, None]:
                    details[idx+2] = '-'

            if switch in ["break", "iteration"]:
                details.insert(2,'')
            elif switch in ["init", "linebreak"]:
                pass
            else:
                details.insert(2, switch)
                details += ['']
            details = [str(d) for d in details]
            self.current_log_iteration += "%s\n" % ',|,'.join(details)
    def publish_log(self):
        #TODO: add extra flag to set is ANY log returns a value? or query against empty log?
        if self.enable_task_logging:
            if self.previous_log_iteration != self.current_log_iteration:
                with open(self.task_progression_log, 'a') as log:
                    log.write(self.current_log_iteration % (self.timestep, self.iteration)) #use rospy.Time.now() ?
                    self.iteration += 1
                    logmsgbreak()
                    logmsg(category="log", msg="Updating log")
                    print('------------------------------------------')

            self.previous_log_iteration = self.current_log_iteration
            self.current_log_iteration = ""

            # if self.iteration >= 2:
            #     quit()

    def log_minimal(self, idx):
        switch = {-2: self.publish_log,
                  -1:['init'],
                  0: ['linebreak', 'iteration', 'task', 'stage', 'new_stage', 'break'],
                  1: ['_start', '_notify_start', 'break'],
                  2: ['_action', 'break'],
                  3: ['route_required'],
                  4: ['route', 'break'],
                  5: ['_query', 'break', '_notify_end', '_del']}

        if idx == min(switch):
            switch[idx]()
        else:
            self.log_data(switch[idx])
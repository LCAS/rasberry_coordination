#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide
# @email: pdasgautham@gmail.com, marc@hanheide.net
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
import rasberry_coordination.coordinator
from rasberry_coordination.coordinator_tools import logmsg, remove, add, move
from rasberry_coordination.robot_manager import RobotManager


class RasberryCoordinator(rasberry_coordination.coordinator.Coordinator):
    """RasberryCoordinator class definition
    """
    def __init__(self, robot_ids, picker_ids, virtual_picker_ids,
                 local_storages, cold_storage, charging_nodes,
                 use_cold_storage,
                 base_stations, wait_nodes,
                 max_task_priorities, low_battery_voltage,
                 admissible_robot_ids, active_tasks,
                 base_station_nodes_pool, wait_nodes_pool,
                 ns="rasberry_coordination"):
        """initialise a RasberryCoordinator object

        Keyword arguments:
            robot_ids -- list of robot_ids
            picker_ids-- list of human picker_ids
            virtual_picker_ids -- list of virtual (DES) picker_ids
            local_storages -- list of local storage nodes
            cold_storage -- cold storage node
            charging_nodes -- list of charging station nodes
            use_cold_storage -- flag to use cold/local storage
            base_stations -- base station nodes for the robots {robot_id:base_node}
            wait_nodes -- waiting nodes (if robot cannot go to storage, it could
                          wait at this node) {robot_id:wait_node}
            max_task_priorities -- max priority of tasks a robot can be assigned to.
                                   this is used to set some robots exclusively for
                                   tasks from human pickers (at higher priority),
                                   while some robots can be assigned to both human
                                   and virtual pickers. (To make demos interesting)
            low_battery_voltage -- voltage at which a robot may set as unfit
            admissible_robot_ids -- list of robots ids for the robots which are allowed to connect to the system
            active_tasks -- list of tasks which the system is currently performing
            base_station_nodes_pool -- pool defining list of all base stations within the system
            wait_nodes_pool -- pool defining list all waiting nodes
        """

        self.log_routes = False  # define whether to print route details to console

        super(RasberryCoordinator, self).__init__(robot_ids,
                                                  picker_ids,
                                                  virtual_picker_ids,
                                                  ns=ns,
                                                  is_parent=True)

        self.local_storages = local_storages
        self.cold_storage = cold_storage
        self.charging_nodes = charging_nodes
        self.use_cold_storage = use_cold_storage

        # Initialise robots
        self.low_battery_voltage = low_battery_voltage

        # Base station initialisation
        self.base_station_nodes_pool = set(base_station_nodes_pool)
        used_base_stations = set(base_stations.values())
        self.available_base_stations = list(self.base_station_nodes_pool.difference(used_base_stations))

        logmsg(msg='base stations in use: ' + ', '.join(base_stations.values()))
        logmsg(msg='base stations available: ' + ', '.join(self.available_base_stations))

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

        # 0 - idle, 1 - transporting_to_picker, 2 - waiting for loading,
        # 3 - waiting for unloading, 4 - transporting to storage, 5- charging
        # 6 - return to base from storage
        self.robot_states_str = {0:"Idle", 1:"Going to picker", 2:"Waiting for loading",
                             3:"Waiting for unloading", 4:"Going to storage",
                             5:"Charging", 6:"Going to base", 9:"Stuck"}

        # robot objects with toponav action clients (on the coord server)
        self.robots = {robot_id: rasberry_coordination.robot.Robot(robot_id) for robot_id in self.robot_ids}

        """Robot Detail Manage Initialisation"""
        self.robot_manager.add_agents(robot_ids)
        for robot in self.robot_manager.agent_details.values():
            robot.base_station = base_stations[robot.robot_id]
            robot.wait_node = wait_nodes[robot.robot_id]
            robot.max_task_priority = max_task_priorities[robot.robot_id]

        # collect_tray_stages = ["go_to_picker", "wait_loading", "go_to_storage", "wait_unloading", "go_to_base"]
        self.task_stages = {robot_id: None for robot_id in self.robot_ids} # keeps track of current stage of the robot

        self.trigger_replan = False
        self.edge_policy_routes = {} # {robot_id: }

        self.moving_robots = [] # all robots which are having an active topo_nav goal (not waiting before critical points for clearance)
        # self.unfit_robots = [] # robots which should be taked away for maintenance. if doing a task now, move to this as soon as it is finished

        # intermediatory lists
        # self.healthy_robots = [] # a list between unfit and idle to avoid idle_robot being modified during task assignments
        # self.unhealthy_robots = [] # a list between idle and unfit to avoid idle_robot being modified during task assignments
        self.force_robot_status_check = False
        self.active_interruptable_robots = [] # all the robots that are executing a task but the task can be interrupted to take on a new one

        logmsg(msg='robots initialised: ' + ', '.join(self.robot_ids))

        # calling from the child class
        self.advertise_services()

        # TaskUpdates msg object for reusing
        self.task_state_msg = rasberry_coordination.msg.TaskUpdates()

        # TODO: these durations should come from a config
        self.max_load_duration = rospy.Duration(secs=60)
        self.max_unload_duration = rospy.Duration(secs=10)

        self.picker_task_updates_pub = rospy.Publisher(self.ns+"task_updates", rasberry_coordination.msg.TaskUpdates, queue_size=5)

        logmsg(msg='coordinator initialised')

    def _get_robot_state(self, robot_id):
        """Extended method to get the state of a robot. Check
        rasberry_coordination.coordinator for more details.

        Keyword arguments:
            robot_id -- robot_id
        """
        state, goal_node, start_time = "", "", rospy.Time()
        robot = self.robot_manager.agent_details[robot_id]

        if self.task_stages[robot_id] is not None:
            state = self.task_stages[robot_id]
            if state == "go_to_picker":
                goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
            elif state == "go_to_storage":
                goal_node = robot.current_storage
            elif state  == "go_to_base":
                goal_node = robot.base_station
            else:
                goal_node = ""
        elif robot_id in self.idle_robots:
            state = "idle"
            goal_node = ""
        elif robot.healthy == False:
            state = "unfit"
            goal_node = ""
        else:
            state = ""
            goal_node = ""
        start_time = self.start_time[robot_id]
        return (state, goal_node, start_time)

    def tray_loaded_ros_srv(self, req):
        """tray_loaded service
        """
        self.robot_manager.set(req.robot_id, 'tray_loaded', True)

        self.write_log({"action_type": "tray_loaded_srv",
                        "robot_id": req.robot_id,
                        })
        return []

    tray_loaded_ros_srv.type = rasberry_coordination.srv.TrayLoaded

    def add_task_ros_srv(self, req):
        """Extended method for adding a task into the task execution framework.
        Check rasberry_coordination.coordinator for more details.
        """
        self.last_id += 1
        task_id = self.last_id

        self.write_log({"action_type": "car_update",
                        "task_updates": "CALLED",
                        "task_id": task_id,
                        "details": "to %s" %(req.task.start_node_id),
                        })

        req.task.task_id = task_id
        logmsg(category="task", id=req.task.task_id, msg='received with target node %s'%(req.task.start_node_id))
        self.tasks.put(
            (task_id, req.task)
        )
        add(self.all_task_ids, task_id)
        self.write_log({"action_type":"add_task_srv",
                        "task_id": task_id,
                        })
        return task_id

    add_task_ros_srv.type = strands_executive_msgs.srv.AddTask

    def cancel_task_ros_srv(self, req):
        """Extended method for cancelling a task from execution.
        Check rasberry_coordination.coordinator for more details.
        """
        cancelled = False
        # Two scenarios:
        # 1. task is already being processed
        #    this also implies the robot has not reached the picker, as
        #    the CAR interface to cancel task won't be available after that.
        #    the topo_nav goal to the robot has to be cancelled
        #    the robot will have to be sent to the base after the cancellation
        # 2. task is still queued or is in processed (if allocated)
        #    pop the task from the queue and add to cancelled
        self.write_log({"action_type": "car_update",
                        "task_updates": "CANCEL",
                        "task_id": req.task_id,
                        })
        for i in range(10):
            locked = self.task_lock.acquire(False)
            if locked:
                break
            rospy.sleep(0.05)

        if locked:
            if req.task_id in self.all_task_ids:
                if ((req.task_id in self.completed_tasks) or
                   (req.task_id in self.cancelled_tasks)):
                    # cannot be here
                    self.write_log({"action_type": "cancel_task_srv",
                                    "task_id": req.task_id,
                                    "details": "fail: cannot be in this condition",
                                    })
    #                raise Exception("cancel_task_ros_srv cannot be in this condition")
                    logmsg(level='error', category="task", id=req.task_id, msg='cancelled task is being cancelled, cancel_task_ros_srv cannot be in this condition')

                elif req.task_id in self.processing_tasks:
                    # task is being processed. remove it
                    move(item=req.task_id, old=self.processing_tasks, new=self.cancelled_tasks)

                    # cancel goal of assigned robot and return it to its base
                    robot_id = ""
                    if req.task_id in self.task_robot_id:
                        robot_id = self.task_robot_id[req.task_id]
                        self.robots[robot_id].cancel_execpolicy_goal()
                        self.send_robot_to_base(robot_id)
                        self.robot_manager.agent_details[robot_id].current_storage = None
                    logmsg(category="task", id=req.task_id, msg='cancelled')
                    logmsg(category="robot", id=robot_id, msg='task canceled and removed')
                    cancelled = True

                    # notify task is being cancelled
                    self.publish_task_state(req.task_id, robot_id, "CANCELLED")

                    self.write_log({"action_type":"cancel_task_srv",
                                    "task_id": req.task_id,
                                    "details": "success",
                                    })
                else:
                    # not yet processed. get it out of tasks
                    tasks = []
                    while not rospy.is_shutdown():
                        try:
                            task_id, task = self.tasks.get(True, 1)
                            if task_id == req.task_id:
                                self.cancelled_tasks[task_id] = task
                                logmsg(category="task", id=task_id, msg='cancelled before assigned to robot')
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

                    # notify task is being cancelled, no robot_id because the task was not yet processing
                    self.publish_task_state(req.task_id, "", "CANCELLED")

                    self.write_log({"action_type":"cancel_task_srv",
                                    "task_id": req.task_id,
                                    "details": "success",
                                    })
            else:
                # invalid task_id
                logmsg(level='error', category="task", id=req.task_id, msg='cancel_task invoked with invalid task_id')
                self.write_log({"action_type": "cancel_task_srv",
                                    "task_id": req.task_id,
                                    "details": "fail: invalid task_id",
                                    })

            self.task_lock.release()
        else:
            logmsg(level='error', category="task", id=req.task_id, msg='cancel service call failed, tasks are being assigned')
            self.write_log({"action_type": "cancel_task_srv",
                                "task_id": req.task_id,
                                "details": "fail: tasks could not be locked. task assignment going on",
                                })

        return cancelled

    cancel_task_ros_srv.type = strands_executive_msgs.srv.CancelTask

    def set_healthy_robot_ros_srv(self, req):
        """Set a robot as healthy if marked as unfit. will then be moved to idle.
        robot must be already set as unfit
        """
        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

        elif not self.robot_manager.get(robot_id, 'healthy'):
            # move to healthy_robots. will be added to idle during next low_battery_check
            self.robot_manager.set(req.robot_id, 'healthy', True)
            # move(item=req.robot_id, old=self.unfit_robots, new=self.healthy_robots)
            self.force_robot_status_check = True
            resp.success = True
            resp.message = "Robot - %s is queued to be marked as healthy" %(req.robot_id)

        else:
            resp.success = False
            resp.message = "Robot - %s is not unfit to be marked as healthy now" %(req.robot_id)

        self.write_log({"action_type": "set_healthy_robot_srv",
                        "robot_id": req.robot_id,
                        "details": resp.message,
                        })
        return resp

    set_healthy_robot_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def set_unhealthy_robot_ros_srv(self, req):
        """Set a robot as unhealthy if marked as unhealthy which will then be moved to unfit.
        robot must be a configured one and is not set as unhealthy or unfit already
        """
        resp = rasberry_coordination.srv.TriggerRobotResponse()

        if req.robot_id not in self.robot_ids:
            """if robot is not setup"""
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)
        elif self.robot_manager.get(req.robot_id, 'healthy'):
            """if robot is set as healthy"""
            self.robot_manager.set(req.robot_id, 'healthy', False)
            req = rasberry_coordination.srv.UnregisterRobot
            req.robot_id = robot.robot_id #TODO: change unregister_robot_ros_srv to take string instead
            self.unregister_robot_ros_srv(req)
            resp.success = True
            resp.message = "Robot - %s is marked as unhealthy" %(req.robot_id)
        else:
            """if robot is unhealthy"""
            resp.success = False
            resp.message = "Robot - %s is already unhealthy" %(req.robot_id)

        self.write_log({"action_type": "set_unhealthy_robot_srv",
                        "robot_id": req.robot_id,
                        "details": resp.message,
                        })
        return resp

    set_unhealthy_robot_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def set_robot_reached_picker_ros_srv(self, req):
        """Set a robot reached the picker. finish the go_to_picker stage. robot will go to wait_loading
        """
        logmsg(category='ROBOT', id=req.robot_id, msg='robot has reached picker')

        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

        elif req.robot_id in self.moving_robots and self.task_stages[req.robot_id] == "go_to_picker":
            self.robots[req.robot_id].cancel_execpolicy_goal()
            self.finish_task_stage(req.robot_id, "go_to_picker")
            task_id = self.robot_task_id[req.robot_id]
            self.publish_task_state(task_id, req.robot_id, "ARRIVED")
            self.write_log({"action_type": "car_update",
                            "task_updates": "ARRIVED",
                            "task_id": task_id,
                            })
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
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

        elif req.robot_id in self.moving_robots and self.task_stages[req.robot_id] == "go_to_storage":
            self.robots[req.robot_id].cancel_execpolicy_goal()
            self.finish_task_stage(req.robot_id, "go_to_storage")
            task_id = self.robot_task_id[req.robot_id]
            self.publish_task_state(task_id, req.robot_id, "STORAGE")
            self.write_log({"action_type": "car_update",
                            "task_updates": "STORAGE",
                            "task_id": task_id,
                            })
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

    def update_task_ros_srv(self, req):
        """update a task information in coordinator. updates come from CAR
        through the serviceproxy in PickerStateMonitor.
        """
        logmsg(category="task", id=req.task.task_id, msg='new task_start_node %s' % req.task.start_node_id)

        self.write_log({"action_type": "task_updates",
                        "task_id": req.task.task_id,
                        "details": "request to update task - %d is received. new task_start_node - %s" %(req.task.task_id, req.task.start_node_id),
                        })
        resp = rasberry_coordination.srv.UpdateTaskResponse()

        # check task_id is in here
        if req.task.task_id in self.completed_tasks:
            resp.success = False
            resp.message = "task %d is already completed" %(req.task.task_id)
            logmsg(category="task", id=req.task.task_id, msg='failed to update task_start_node, task complete')

        elif req.task.task_id in self.failed_tasks:
            resp.success = False
            resp.message = "task %d is already failed" %(req.task.task_id)
            logmsg(category="task", id=req.task.task_id, msg='failed to update task_start_node, already failed')

        elif req.task.task_id in self.cancelled_tasks:
            resp.success = False
            resp.message = "task %d is already cancelled" %(req.task.task_id)
            logmsg(category="task", id=req.task.task_id, msg='failed to update task_start_node, already cancelled')

        elif req.task.task_id in self.processing_tasks:
            # task is being processed
            # get the task_lock
            for i in range(10):
                locked = self.task_lock.acquire(False)
                if locked:
                    self.processing_tasks[req.task.task_id] = req.task
                    resp.success = True
                    resp.message = "successfully updated task %d" %(req.task.task_id)
                    self.trigger_replan = True
                    logmsg(category="task", id=req.task.task_id, msg='task_start_node updated to node %s' % req.task.start_node_id)
                    self.task_lock.release()
                    break

                else:
                    rospy.sleep(0.05)

                    if i == 9:
                        resp.success = False
                        resp.message = "Could not get the task lock"
                        logmsg(category="task", id=req.task.task_id, msg='failed to update task_start_node, unable to get task_lock')

        else:
            # not yet taken up for processing
            # get the task_lock
            for i in range(10):
                locked = self.task_lock.acquire(False)
                if locked:
                    tasks = []
                    while not rospy.is_shutdown():
                        try:
                            task_id, task = self.tasks.get(True, 1)
                            if task_id == req.task.task_id:
                                task = req.task
                                logmsg(category="task", id=req.task.task_id, msg='updating task...')
                                resp.success = True
                                resp.message = "successfully updated task %d" %(req.task.task_id)
                                logmsg(category="task", id=req.task.task_id, msg='task_start_node updated to node %s' % req.task.start_node_id)
                                break # got it
                            else:
                                # hold on to the other tasks to be readded later
                                tasks.append((task_id, task))
                        except Queue.Empty:
                            break
                    # readd popped tasks
                    for (task_id, task) in tasks:
                        # put all retrieved tasks back in the queue
                        self.tasks.put((task_id, task))
                    self.task_lock.release()
                    break

                else:
                    rospy.sleep(0.05)

                    if i == 9:
                        resp.success = False
                        resp.message = "Could not get the task lock"
                        logmsg(category="task", id=req.task.task_id, msg='failed to update task_start_node, unable to get task_lock')

        self.write_log({"action_type": "update_task_ros_srv",
                        "task_id": req.task.task_id,
                        "details": resp.message
                        })
        return resp

    update_task_ros_srv.type = rasberry_coordination.srv.UpdateTask

    def connect_robot_ros_srv(self, req):
        """Add robot to system so the coordinator can see it.
        Reject if the robot_id is not listed in the map config...
        or it's viable tasks are not active system tasks...
        or if the robot is already visible...
        or if there is no base_station available.
        """
        logmsg(level='warn', category="robot", id=req.robot_id, msg='connecting to system')
        self.robot_manager.set(req.robot_id, 'disconnect_when_idle', False)
        resp = rasberry_coordination.srv.ConnectRobotResponse()

        if req.robot_id not in self.admissible_robot_ids:
            logmsg(level="warn", category="robot", id=req.robot_id, msg='connection failed, robot not in admissible_robot_ids')
            logmsg(msg='admissible robots: ' + ', '.join(self.admissible_robot_ids))
            resp.success = 0
            resp.msg = 'robot not in admissible_robot_ids'
            return resp

        elif not any([task in self.active_tasks for task in req.tasks]):
            logmsg(level="warn", category="robot", id=req.robot_id, msg='connection failed, given tasks are not active in system')
            logmsg(msg='active tasks: ' + ', '.join(self.active_tasks))
            resp.success = 0
            resp.msg = 'given tasks are not active in system'
            return resp

        elif req.robot_id in self.robot_ids:
            logmsg(level="warn", category="robot", id=req.robot_id, msg='connection failed, robot already connected')
            logmsg(msg='connected robots: ' + ', '.join(self.robot_ids))

            if req.register and not self.robot_manager.get(req.robot_id, 'registered'):
                logmsg(level="warn", category="robot", id=req.robot_id, msg='robot no longer pending to unregister')
                self.register_robot_ros_srv(req)
                self.modify_robot_marker(req.robot_id, color='no_color')
                resp.success = 1
                resp.msg = 'robot already connected, no longer pending to unregister'
                return resp
            else:
                resp.success = 0
                resp.msg = 'robot already connected'
                return resp

        elif len(self.available_base_stations) < 1:
            logmsg(level="warn", category="robot", id=req.robot_id, msg='connection failed, no base stations available')
            logmsg(msg='base stations in use: ' + ', '.join(self.robot_manager.get_list('base_station')))
            resp.success = 0
            resp.msg = 'no base stations available'
            return resp

        self.connect_robot(req.robot_id)
        if req.register:
            self.register_robot_ros_srv(req)
            self.modify_robot_marker(req.robot_id, color='no_color')
        else:
            self.modify_robot_marker(req.robot_id, color='red')

        # send success response to service
        resp.success = 1
        resp.msg = 'robot successfully connected'
        return resp

    connect_robot_ros_srv.type = rasberry_coordination.srv.ConnectRobot

    def connect_robot(self, robot_id, max_task_priority=255):
        self.robot_manager.add_robot(robot_id)
        robot = self.robot_manager.agent_details[robot_id]

        add(self.robot_ids, robot_id)
        self.robot_states[robot_id] = 0
        self.start_time[robot_id] = rospy.get_rostime()

        # Set first available base station as taken
        robot.base_station = remove(self.available_base_stations, 0)

        logmsg(category="robot", id=robot_id, msg='assigned to base station %s' % (robot.base_station))
        logmsg(msg='base stations in use: ' + ', '.join(self.robot_manager.get_list('base_station')))
        logmsg(msg='base stations available: ' + ', '.join(self.available_base_stations))


        # Set first available wait node as taken
        robot.wait_node = remove(self.available_wait_nodes, 0)

        # Set priority
        add(self.presence_agents, robot_id)

        self.robot_task_id[robot_id] = None  # current task assigned to a robot

        self.robots[robot_id] = rasberry_coordination.robot.Robot(robot_id)
        self.task_stages[robot_id] = None  # keeps track of current stage of the robot

        logmsg(category="robot", id=robot_id, msg='@ stage: %s' % (str(self.task_stages[robot_id]).upper()))

        self.robot_manager.set(robot_id, 'tray_loaded', False)

        logmsg(level='warn', category="robot", id=robot_id, msg='connection complete')

    def register_robot_ros_srv(self, req):
        """Register the robot to enable task assignment.
        Reject if the robot_id is listed as a registered robot...
        but only if it is not planned for unregistration.
        """
        logmsg(level='warn', category="robot", id=req.robot_id, msg='registering for task allocation')
        resp = rasberry_coordination.srv.RegisterRobotResponse()

        if self.robot_manager.get(req.robot_id, 'registered'):
            logmsg(level="warn", category="robot", id=req.robot_id, msg='registration failed, robot already registered')
            logmsg(msg='registered robots: ' + ', '.join(self.robot_manager.get_list('registered')))
            resp.success = 0
            resp.msg = 'registration failed, robot already registered'
            return resp
        elif req.robot_id not in self.robot_ids:
            logmsg(level="warn", category="robot", id=req.robot_id, msg='registration failed, robot is not currently connected')
            logmsg(msg='connected robots: ' + ', '.join(self.robot_ids))
            resp.success = 0
            resp.msg = 'registration failed, robot is not currently connected'
            return resp

        # if not actively completing a task, add to idle robots
        logmsg(category="task", msg='self.robot_task_id: %s' % (str(self.robot_task_id)))
        if (req.robot_id not in self.robot_task_id) or (self.robot_task_id[req.robot_id] is None):
            add(self.idle_robots, req.robot_id)
            logmsg(category="list", msg='idle robots: %s' % (str(self.idle_robots)))

        # set it to appear without a color modifier
        self.robot_manager.set(req.robot_id, 'registered', True)
        self.modify_robot_marker(req.robot_id, color='no_color')
        logmsg(level="warn", category="robot", id=req.robot_id, msg='registration complete')

        # send success response to service
        resp.success = 1
        resp.msg = 'robot has registered'
        return resp

    register_robot_ros_srv.type = rasberry_coordination.srv.RegisterRobot

    def unregister_robot_ros_srv(self, req):
        """Prevent robot from accepting tasks and set to unregister when its current task completed
        Reject if robot_id is not listed as an registered robot...
        or if the robot is already planned for unregistration.
        """
        logmsg(level='warn', category="robot", id=req.robot_id, msg='unregistering from task allocation')
        resp = rasberry_coordination.srv.UnregisterRobotResponse()

        if not self.robot_manager.get(req.robot_id, 'registered'):
            logmsg(level="warn", category="robot", id=req.robot_id, msg='unregistration failed, robot is not registered')
            logmsg(msg='registered robots: ' + ', '.join(self.robot_manager.get_list('registered')))
            resp.success = 0
            resp.msg = 'unregistration failed, robot is not registered'
            return resp

        # remove robot if interruptable
        remove(self.idle_robots, req.robot_id)
        remove(self.active_interruptable_robots, req.robot_id)

        # if robot is attempting to unregister, set it to appear red
        self.robot_manager.set(req.robot_id, 'registered', False)
        self.modify_robot_marker(req.robot_id, color='red')
        logmsg(level="warn", category="robot", id=req.robot_id, msg='unregistration complete')

        # send success response to service
        resp.success = 1
        resp.msg = 'robot has unregistered'
        return resp

    unregister_robot_ros_srv.type = rasberry_coordination.srv.UnregisterRobot

    def disconnect_robot_ros_srv(self, req):
        """Remove robot from system.
        Reject if robot_id is not listed as a visible robot.
        """
        logmsg(level='warn', category="robot", id=req.robot_id, msg='disconnecting from system')
        resp = rasberry_coordination.srv.DisconnectRobotResponse()

        if req.robot_id not in self.robot_ids:
            logmsg(level="warn", category="robot", id=req.robot_id, msg='disconnection failed, robot is not currently connected')
            logmsg(msg='connected robots: ' + ', '.join(self.robot_ids))
            resp.success = 0
            resp.msg = 'disconnection failed, robot is not currently connected'
            return resp

        if self.robot_manager.get(req.robot_id, 'registered'):
            self.unregister_robot_ros_srv(req)

        if req.robot_id in self.idle_robots:
            self.disconnect_robot(req.robot_id)

            # send success response to service
            resp.success = 1
            resp.msg = 'robot successfully disconnected'
            return resp

        self.robot_manager.set(req.robot_id, 'disconnect_when_idle', True)

        # send success response to service
        resp.success = 1
        resp.msg = 'robot set to disconnect on task completion'
        return resp

    disconnect_robot_ros_srv.type = rasberry_coordination.srv.DisconnectRobot

    def disconnect_robot(self, robot_id):
        """remove all record of the robot being a member of the system
        """

        """Identify robot to remove"""
        robot = self.robot_manager.agent_details[robot_id]

        """Extract any useful information"""
        add(self.available_base_stations, robot.base_station)
        add(self.available_wait_nodes, robot.wait_node)
        logmsg(msg='base station %s added to pool' % (robot.base_station))
        logmsg(msg='base stations in use: ' + ', '.join(self.robot_manager.get_list('base_station')))
        logmsg(msg='base stations available: ' + ', '.join(self.available_base_stations))

        """Remove robot"""
        self.robot_manager.remove_robot(robot_id)
        # self.robot_manager.set(robot_id, 'disconnect_when_idle', False) #TODO: not needed?


        self.robots[robot_id].cancel_execpolicy_goal()

        remove(self.robot_ids, robot_id)
        remove(self.active_robots, robot_id)
        remove(self.robot_states, robot_id)
        remove(self.robots, robot_id)

        # localisation
        remove(self.presence_agents, robot_id)

        # task_activity
        remove(self.robot_task_id, robot_id)
        remove(self.task_stages, robot_id)
        remove(self.start_time, robot_id)

        # set robot marker
        self.clear_robot_marker(robot_id)

        logmsg(level='warn', category="robot", id=robot_id, msg="disconnection complete")

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
        """
        logmsg(category="robot", id=robot_id, msg='attempting to send to base station')

        robot = self.robot_manager.agent_details[robot_id]
        if robot.current_node == robot.base_station:
            logmsg(category="robot", id=robot_id, msg='already at base station')
            # already at base station. set as idle
            remove(self.active_robots, robot_id)
            remove(self.active_interruptable_robots, robot_id)
            remove(self.moving_robots, robot_id)
            add(self.idle_robots, robot_id)
            logmsg(category="list", msg='idle robots: %s' % (str(self.idle_robots)))
        else:
            logmsg(category="robot", id=robot_id, msg='not at base station, setting as target')
            # not at base. also not idle. set stage as send to base
            move(item=robot_id, old=self.idle_robots, new=self.active_robots)
            add(self.active_interruptable_robots, robot_id)
            logmsg(category="list", msg='active_interruptable_robots: %s' % (str(self.active_interruptable_robots)))

            self.task_stages[robot_id] = "go_to_base"

            logmsg(category="robot", id=robot_id, msg='@ stage: %s' % (str(self.task_stages[robot_id]).upper()))
            self.write_log({"action_type": "robot_update",
                            "robot_task_stage": "go_to_base_start",
                            "robot_id": robot_id,
                            "current_node": robot.current_node,
                            "closest_node": robot.closest_node,
                            })

    def find_closest_robot(self, task):
        """find the robot closest to the task location (picker_node)
        # assign based on priority (0 - virtual pickers only, >=1 real pickers)

        Keyword arguments:
            task - strands_executive_msgs.Task
        """
        goal_node = task.start_node_id
        robot_dists = {}

        for robot_id in self.idle_robots + self.active_interruptable_robots:
            robot = self.robot_manager.agent_details[robot_id]

            # ignore if the robot is not actively accepting tasks
            if not robot.registered:
                continue

            # ignore if the robot's closest_node and current_node is not yet available
            if robot.current_node == "none" and robot.closest_node == "none":
                continue

            # ignore if the task priority is less than the min task priority for the robot
            # lower the value, higher the priority
            if task.priority > robot.max_task_priorities:
                continue

            # use current_node as start_node if available
            if robot.current_node != "none":
                start_node = robot.current_node
            else:
                start_node = robot.closest_node

            if start_node =="none" or goal_node == "none" or start_node is None or goal_node is None:
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
            for i in range(len(robot.routes)):
                if robot.routes[i] == node_id:
                    break
                # add edge_distance only if the source node is not the one we look for
                # also make sure we start adding from current/closest node
                if not adding_ok:
                    if robot.current_node != "none":
                        if robot.current_node == robot.routes[i]:
                            adding_ok = True
                    elif robot.closest_node != "none":
                        if robot.closest_node == robot.routes[i]:
                            adding_ok = True
                if adding_ok:
                    dist += robot.route_dists[i]
        return dist

    def assign_tasks(self, ):
        """assign task to idle robots
        high priority tasks are assigned first
        among equal priority tasks, low task_id is assigned first
        """
        trigger_replan = False

        locked = self.task_lock.acquire(False)

        if locked:
            # get all tasks from queue
            tasks = []
            while not rospy.is_shutdown():
                try:
                    task_id, task = self.tasks.get(True, 1)
                    tasks.append((task_id, task))
                except Queue.Empty:
                    break

            # high priority tasks first served
            # among equal priority, first came first served
            task_priorities = {}
            for (task_id, task) in tasks:
                if task.priority not in task_priorities:
                    task_priorities[task.priority] = {task_id: task}
                else:
                    task_priorities[task.priority][task_id] = task

            priorities = sorted(task_priorities.keys(), reverse=True) # higher priority first
            for priority in priorities:
                # try to get a robot for each task
                task_ids = sorted(task_priorities[priority].keys()) # lower task_id first
                for task_id in task_ids:
                    task = task_priorities[priority][task_id]
                    # among equal priority, first came first served
                    robot_id = self.find_closest_robot(task)
                    if robot_id is None:
                        continue
                    robot = self.robot_manager.agent_details[robot_id]

                    logmsg(category="robot", id=robot_id, msg='assigned task %s'%(task_id))
                    logmsg(msg='active robots: %s'%(str(self.active_interruptable_robots)))
                    logmsg(msg='idle robots: %s'%(str(self.idle_robots)))

                    # if the robot is still active interrupt previous actions
                    if robot_id in self.active_interruptable_robots:
                        self.robots[robot_id].cancel_execpolicy_goal()
                        self.finish_task_stage(robot_id, self.task_stages[robot_id])
                        remove(self.active_robots, robot_id)
                        remove(self.active_interruptable_robots, robot_id)
                        add(self.idle_robots, robot_id)
                        logmsg(category="list", msg='idle robots: %s' % (str(self.idle_robots)))

                    # trigger replan for any new assignment
                    trigger_replan = True

                    tasks.remove((task_id, task))
                    remove(self.idle_robots, robot_id)
                    add(self.active_robots, robot_id)
                    self.task_stages[robot_id] = "go_to_picker"

                    logmsg(category="robot", id=robot_id, msg='@ stage: %s' % (str(self.task_stages[robot_id]).upper()))
                    self.processing_tasks[task_id] = task
                    self.task_robot_id[task_id] = robot_id
                    self.robot_task_id[robot_id] = task_id
                    self.update_current_storage(robot_id)

                    self.task_state_msg.task_id = task_id
                    self.task_state_msg.robot_id = self.task_robot_id[task_id]
                    self.task_state_msg.state = "ACCEPT"
                    self.picker_task_updates_pub.publish(self.task_state_msg)

                    self.write_log({"action_type": "car_update",
                                    "task_updates": "ACCEPT",
                                    "task_id": task_id,
                                    "robot_id": robot_id,
                                    })

                    self.write_log({"action_type": "robot_update",
                                    "robot_task_stage": "go_to_picker_start",
                                    "task_id": task_id,
                                    "robot_id": robot_id,
                                    "details": "to %s" %(task.start_node_id),
                                    "current_node": robot.current_node,
                                    "closest_node": robot.closest_node,
                                    })

            # putting unassigned tasks back in the queue
            for (task_id, task) in tasks:
                self.tasks.put((task_id, task))

            self.task_lock.release()

        self.trigger_replan = self.trigger_replan or trigger_replan

    def finish_route_fragment(self, robot_id):
        """finish fragment of a route in a task stage
        """
        robot = self.robot_manager.agent_details[robot_id]

        # clear route vis of robot
        remove(self.moving_robots, robot_id)

        # this may be called multiple times when a robot is stuck
        robot.route = []
        robot.route_fragments = []
        self.robots[robot_id].execpolicy_result = None

    def finish_task_stage(self, robot_id, curr_stage=None):
        """finish a stage of the collect tray
        """
        robot = self.robot_manager.agent_details[robot_id]

        next_stage = {"go_to_picker":"wait_loading",
                      "wait_loading":"go_to_storage",
                      "go_to_storage":"wait_unloading",
                      "wait_unloading":"go_to_base",
                      "go_to_base":None}

        # if robot was in transit to task, finish task route
        if curr_stage in ["go_to_picker", "go_to_storage", "go_to_base"]:
            self.finish_route_fragment(robot_id)

        #Identify task_id if it exists
        task_id=""
        if robot_id in self.robot_task_id:
            task_id = self.robot_task_id[robot_id]

        self.write_log({"action_type": "robot_update",
                        "robot_task_stage": curr_stage+"_finish",
                        "task_id": task_id,
                        "robot_id": robot_id,
                        "current_node": robot.current_node,
                        "closest_node": robot.closest_node
                        })

        # set task to next stage in logistics process
        self.task_stages[robot_id] = next_stage[curr_stage]
        self.start_time[robot_id] = rospy.get_rostime()
        if curr_stage != "go_to_base":
            self.write_log({"action_type": "robot_update",
                            "robot_task_stage": next_stage[curr_stage]+"_start",
                            "task_id": task_id,
                            "robot_id": robot_id,
                            "current_node": robot.current_node,
                            "closest_node": robot.closest_node
                            })
        logmsg(category="robot", id=robot_id, msg='@ stage: %s' % (str(self.task_stages[robot_id]).upper()))

    def finish_task(self, robot_id):
        """set the task assigned to the robot as finished whenever trays are unloaded
        """
        robot = self.robot_manager.agent_details[robot_id]
        self.finish_task_stage(robot_id, "wait_unloading")
        # move task from processing to completed
        self.task_stages[robot_id] = "go_to_base"

        logmsg(category="robot", id=robot_id, msg='@ stage: %s' % (str(self.task_stages[robot_id]).upper()))

        task_id = self.robot_task_id[robot_id]
        self.robot_task_id[robot_id] = None
        robot.current_storage = None
        move(item=task_id, old=self.processing_tasks, new=self.completed_tasks)

        # move robot from moving robots to interruptable (i.e. ready to accept a new task)
        move(item=robot_id, old=self.moving_robots, new=self.active_interruptable_robots)

        logmsg(category="robot", id=robot_id, msg='@ stage: %s' % (str(self.task_stages[robot_id]).upper()))

        self.write_log({"action_type": "task_update",
                        "task_updates": "task_finish",
                        "task_id": task_id,
                        "robot_id": robot_id,
                        })

    def set_task_failed(self, task_id):
        """set task state as failed
        """
        move(item=task_id, old=self.processing_tasks, new=self.failed_tasks)

        self.write_log({"action_type": "task_update",
                        "task_updates": "task_failed",
                        "task_id": task_id,
                        "details": "Assigned robot failed to complete task after reaching picker",
                        })

    def publish_task_state(self, task_id, robot_id, state):
        """publish the state of task (or picker) in CAR
        """
        self.task_state_msg.task_id = task_id
        self.task_state_msg.robot_id = robot_id
        self.task_state_msg.state = state
        self.picker_task_updates_pub.publish(self.task_state_msg)
        rospy.sleep(0.01)

    def readd_task(self, task_id):
        """if robot assigned to a task fails before reaching the picker, the task can be
        reassigned. so readd into the task queue.
        """

        #if task_id not in self.task_robot_id:
        #    logmsg(category="task", id=task_id, msg='task not added back to queue')
        #    return
        #else:
        #    robot_id = self.task_robot_id[task_id]
        robot_id = self.task_robot_id[task_id]

        # this task should be readded to the queue if not cancelled by the picker !!!
        if task_id in self.processing_tasks:
            logmsg(category="task", id=task_id, msg='picker still requires task completion, task added back to queue')
            logmsg(category="robot", id=self.task_robot_id[task_id], msg='failed to reach picker')

            remove(self.task_robot_id, task_id)  # remove from the assigned robot
            task = remove(self.processing_tasks, task_id)  # remove from processing tasks

            self.publish_task_state(task_id, "", "ABANDONED")

            self.tasks.put(
                (task_id, task)
            )

            self.write_log({"action_type": "task_update",
                            "task_updates": "readd_task",
                            "task_id": task_id,
                            "details": "Assigned robot failed to reach picker, adding the task back into the queue",
                            })

            self.write_log({"action_type": "car_update",
                            "task_updates": "CALLED",
                            "task_id": task_id,
                            })

        rospy.sleep(0.01)

    def handle_tasks(self):
        """update task execution progress for all robots
        """
        # trigger replan if there is a new task assignment, a robot waiting completed
        # or task update from a robot
        trigger_replan = False

        for robot_id in self.active_robots:
            robot = self.robot_manager.agent_details[robot_id]
            task_id = self.robot_task_id[robot_id]

            if robot_id in self.moving_robots:
                # topo nav stage
                # if any robot has finished its current goal, remove the finished goal from the robot's route
                if self.robots[robot_id].execpolicy_result is None:
                    # task/fragment not finished
                    continue

                # check for robots which are moving, not waiting before a critical point
                if self.robots[robot_id].execpolicy_result.success:

                    logmsg(category="robot", id=robot_id, msg='execpolicy_result.success %s' % (self.task_stages[robot_id]))

                    # trigger replan whenever a segment completion is reported
                    trigger_replan = True
                    # if the robot's route is finished, progress to the next stage of the collect tray process
                    # has it finished the stage?
                    goal_node = None
                    if self.task_stages[robot_id] == "go_to_picker":
                        # goal is picker node as in the task
                        goal_node = self.processing_tasks[task_id].start_node_id
                    elif self.task_stages[robot_id] == "go_to_storage":
                        # goal is storage node
                        goal_node = robot.current_storage
                    elif self.task_stages[robot_id] == "go_to_base":
                        # goal is robot's base node
                        goal_node = robot.base_station

                    if (len(robot.route_fragments) == 1 and
                        robot_id.current_node is not None and
                        robot.current_node == goal_node):
                        # finished the stage
                        if self.task_stages[robot_id] == "go_to_picker":
                            # go_to_picker stage is finished
                            logmsg(category="robot", id=robot_id, msg='go_to_picker stage is finished')
                            self.finish_task_stage(robot_id, "go_to_picker")
                            self.publish_task_state(task_id, robot_id, "ARRIVED")
                            self.write_log({"action_type": "car_update",
                                            "task_updates": "ARRIVED",
                                            "task_id": task_id,
                                            })

                        elif self.task_stages[robot_id] == "go_to_storage":
                            # go_to_storage stage is finished
                            logmsg(category="robot", id=robot_id, msg='go_to_storage stage is finished')
                            self.finish_task_stage(robot_id, "go_to_storage")
                            self.publish_task_state(task_id, robot_id, "STORAGE")
                            self.write_log({"action_type": "car_update",
                                            "task_updates": "STORAGE",
                                            "task_id": task_id,
                                            })

                        elif self.task_stages[robot_id] == "go_to_base":
                            # task is finished
                            logmsg(category="robot", id=robot_id, msg='go_to_base stage is finished')
                            self.finish_task_stage(robot_id, "go_to_base")

                            if robot_id in self.robot_task_id:
                                remove(self.robot_task_id, robot_id)
                            self.task_stages[robot_id] = None

                            # remove robot from any form of active robots
                            remove(self.active_robots, robot_id)
                            remove(self.active_interruptable_robots, robot_id)

                            # add robot to idle, if registered
                            if robot.registered:
                                add(self.idle_robots, robot_id)
                                logmsg(category="list", msg='idle robots: %s' % (str(self.idle_robots)))
                            if robot.disconnect_when_idle:
                                self.disconnect_robot(robot_id)

                    else:
                        # finished only a fragment. may have to wait for clearance
                        self.finish_route_fragment(robot_id)

                else:
                    logmsg(category="robot", id=robot_id, msg='execpolicy_result is not success or None (%s)'%(self.robots[robot_id].execpolicy_result))

                    trigger_replan = True  # trigger replan as robot is being sent back to base
                    # robot failed execution sending to base
                    logmsg(category="task", id=task_id, msg='failed to complete task %s at stage %s'%(task_id, self.task_stages[robot_id]))
                    if self.task_stages[robot_id] == "go_to_picker":
                        # task is good enough to be assigned to another robot
                        self.readd_task(task_id)
                        self.send_robot_to_base(robot_id)
                    elif self.task_stages[robot_id] == "go_to_base":
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
                # wait_loading or wait_unloading
                if self.task_stages[robot_id] == "wait_loading":
                    # if conditions are satisfied, finish waiting
                    # 1. LOADED from CAR
                    # 2. service call from active_compliance
                    # 3. delay (?)
                    finish_waiting = False
                    if robot.tray_loaded:
                        finish_waiting = True
                    elif False:
                        # TODO: active compliance
                        pass
#                    elif rospy.get_rostime() - self.start_time[robot_id] > self.max_load_duration:
#                        # delay
#                        finish_waiting = True
                    else:
                        rospy.sleep(0.5)

                    if finish_waiting:
                        self.publish_task_state(task_id, robot_id, "LOADED")
                        self.write_log({"action_type": "car_update",
                            "task_updates": "LOADED",
                            "task_id": task_id,
                            })

                        self.finish_task_stage(robot_id, "wait_loading")
                        robot.tray_loaded = True
                        trigger_replan = True

                elif self.task_stages[robot_id] == "wait_unloading":
                    # if conditions satisfy, finish waiting
                    # 1. delay
                    if rospy.get_rostime() - self.start_time[robot_id] > self.max_unload_duration:
                        # delay
                        self.publish_task_state(task_id, robot_id, "DELIVERED")
                        self.write_log({"action_type": "car_update",
                            "task_updates": "DELIVERED",
                            "task_id": task_id,
                            })

                        self.finish_task(robot_id)
                        robot.tray_loaded = False
                        trigger_replan = True


                    else:
                        rospy.sleep(0.5)

                else:
                    # robot is waiting before a critical point
                    if not self.moving_robots:
                        # no other moving robots - replan
                        trigger_replan = True
                    else:
                        # should wait until one of the moving robots to finish its route fragment
                        pass
        self.trigger_replan = self.trigger_replan or trigger_replan

    def critical_points(self, ):
        """find points where agent's path cross with those of active robots.
        also find active robots which cross paths at these critical points.
        """
        critical_points = {}
        critical_robots = {} # {critical_point: [robot_ids]} all robots touching a critical point
        #for agent_id in self.presence_agents:
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            agent_id = agent.agent_id
            r_outer = agent.routes
            critical_points[str(r_outer)] = set([])

            # check route of each robot and picker against routes of all robots
            for robot_id in self.active_robots:
                robot = self.robot_manager.agent_details[robot_id]
                if agent_id == robot_id:
                    continue
                r_inner = robot.route
                if r_outer is not r_inner:
                    critical_points[str(r_outer)] = critical_points[str(r_outer)].union(set(r_outer).intersection(set(r_inner)))

                    for node_id in set(r_outer).intersection(set(r_inner)):
                        if node_id not in critical_robots:
                            critical_robots[node_id] = [robot_id]
                        elif robot_id not in critical_robots[node_id]:
                            critical_robots[node_id].append(robot_id)

                        if agent_id in self.active_robots and agent_id not in critical_robots[node_id]:
                            critical_robots[node_id].append(agent_id)

        return (critical_points, critical_robots)

    def shortest_route_to_node(self, robot_ids, node_id):
        """from a list of robot_ids, find the robot with shortest route distance to a given node

        Keyword arguments:

        robot_ids -- []
        node_id --
        """
        dists = {}
        for robot_id in robot_ids:
            dists[robot_id] = self.get_route_distance_to_node(robot_id, node_id)

        return sorted(dists.items(), key=operator.itemgetter(1))[0][0]

    def split_critical_paths(self, ):
        """split robot paths at critical points
        """
        c_points, c_robots = self.critical_points()

        # for robots in go_to_picker mode, if the picker node is in the critical points, remove it
        for robot_id in self.active_robots:
            robot = self.robot_manager.agent_details[robot_id]
            if (self.task_stages[robot_id] == "go_to_picker" and
                self.processing_tasks[self.robot_task_id[robot_id]].start_node_id in c_points[str(robot.routes)]):
                c_points[str(robot.routes)].remove(self.processing_tasks[self.robot_task_id[robot_id]].start_node_id)

#        rospy.loginfo(c_points)

        allowed_cpoints = []
        res_routes = {}

        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            agent_id = agent.agent_id
            allowed_to_pass = False
            r = agent.route
            rr = []
            partial_route = []

            for v in r:
                if v in c_points[str(r)]: # vertice is critical point
                    # allow critical point once for a robot among all agents
                    nearest_robot = self.shortest_route_to_node(c_robots[v], v)
                    if (agent_id == nearest_robot and
                        v not in allowed_cpoints):
                        partial_route.append(v)
                        allowed_cpoints.append(v)
                        allowed_to_pass = True

                    elif v not in allowed_cpoints and allowed_to_pass:
                        # already allowed once, but keep going although not the nearest to cp
                        partial_route.append(v)
                        allowed_cpoints.append(v)

                    else:
                        # neither the nearest or allowed earlier. so wait before cp
                        if partial_route:
                            rr.append(partial_route)
                        partial_route = [v]

                else:
                    partial_route.append(v)

            if partial_route:
                rr.append(partial_route)
            res_routes[agent_id] = rr

        # self.route_fragments = res_routes
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            if agent.agent_id in res_routes:
                agent.route_fragments = res_routes[agent.agent_id]

        res_edges = {}
        # split the edges as per the route_fragments
        for robot_id in self.active_robots:
            robot = self.robot_manager.agent_details[robot_id]

            if robot.route_fragments:
                # remove goal node from last fragment
                # if start and goal nodes are different, there will be at least one node remaining and an edge
                robot.route_fragments[-1].pop(-1)
                # move the last node of all fragments to the start of next fragment
                for i in range(len(robot.route_fragments) - 1):
                    robot.route_fragments[i+1].insert(0, robot.route_fragments[i][-1])
                    robot.route_fragments[i].pop(-1)

                # split the edges
                res_edges[robot_id] = []
                for i in range(len(robot.route_fragments)):
                    res_edges[robot_id].append(robot.route_edges[:len(robot.route_fragments[i])])
                    robot.route_edges = robot.route_edges[len(robot.route_fragments[i]):]
            else:
                robot.route_fragments = []
                res_edges[robot_id] = []

        # self.route_edges = res_edges
        for agent in self.robot_manager.agent_details.values() + self.picker_manager.agent_details.values():
            if agent.agent_id in res_edges:
                agent.route_fragments = res_edges[agent.agent_id]

    def set_execute_policy_routes(self, ):
        """find connecting edges for each fragment in route_fragments and set
        the corresponding route object (with source and edge_id)
        """
        for robot_id in self.active_robots:
            robot = self.robot_manager.agent_details[robot_id]
            goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
            if robot.route_fragments:
                goal.route.source = robot.route_fragments[0]
                goal.route.edge_id = robot.route_edges[0]

            if goal != self.robots[robot_id].execpolicy_goal:
                same_route = True

                if goal.route.source and self.robots[robot_id].execpolicy_goal.route.source:
                    # if there is at least one source node
                    if goal.route.edge_id[-1] != self.robots[robot_id].execpolicy_goal.route.edge_id[-1]:
                        # goal edge_ids are different
                        same_route = False
                    else:
                        # loop from new start source node to end source node
                        new_start_node = goal.route.source[0]

                        # look for new_start_node in previous goal
                        idx = 0
                        change_idx = False
                        for i in range(len(self.robots[robot_id].execpolicy_goal.route.source)):
                            if new_start_node == self.robots[robot_id].execpolicy_goal.route.source[i]:
                                # found current start source node in previous source nodes
                                if len(self.robots[robot_id].execpolicy_goal.route.source) - i != len(goal.route.source):
                                    # remaining nodes in the routes are different => different route
                                    same_route = False
                                    break
                                change_idx = True # enable comparing source nodes here onwards

                            if change_idx:
                                if goal.route.source[idx] != self.robots[robot_id].execpolicy_goal.route.source[i]:
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
                    #logmsg(category="exec", id=robot_id, msg='defining execpolicy_goal')

                    if self.log_routes:
                        logmsg(category="robot", id=robot_id, msg='new route %s, previous route was %s' % (goal, self.robots[robot_id].execpolicy_goal))

                    self.write_log({"action_type": "robot_update",
                            "details": "new_route",
                            "source": str(goal.route.source),
                            "edge_ids": str(goal.route.edge_id),
                            "robot_id": robot_id,
                            "current_node": robot.current_node,
                            "closest_node": robot.closest_node,
                            })

                    self.robots[robot_id].set_execpolicy_goal(goal)
                    if goal.route.edge_id:
                        add(self.moving_robots, robot_id)
                        logmsg(category="list", msg='moving_robots: %s' % (str(self.moving_robots)))

    def set_empty_execpolicy_goal(self, robot_id):
        """for intermediate cancellation, sending another empty goal to preempt
        current goal
        """
        goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()

        logmsg(category="exec", id=robot_id, msg='defining empty execpolicy_goal as %s'%(goal))
        self.robots[robot_id].set_execpolicy_goal(goal)

    def replan(self, ):
        """replan - find indiviual paths, find critical points in these paths, and fragment the
        paths at critical points - whenever triggered
        """

        """find routes for all robots which need one"""
        for robot in self.robot_manager.agent_details.values():
            robot_id = robot.robot_id


            """for each active robot"""
            if robot_id in self.active_robots:


                """if waiting set goal as current node, generate route, and exit"""
                if self.task_stages[robot_id] in ["wait_loading", "wait_unloading"]:
                    # loading and unloading robots should finish those stages first
                    # put the current node of the idle robots as their route - to avoid other robots planning routes through those nodes
                    if robot.current_node != "none":
                        robot.route = [robot.current_node]
                    elif robot.previous_node != "none":
                        robot.route = [robot.previous_node]
                    else:
                        robot.route = [robot.closest_node]
                    robot.route_edges = []
                    self.get_edge_distances(robot_id)
                    continue


                """set start node"""
                if robot.current_node != "none":
                    start_node = robot.current_node
                elif robot.previous_node != "none":
                    start_node = robot.previous_node
                else:
                    start_node = robot.closest_node


                """set goal node as picker location, storage or base station"""
                goal_node = None
                if self.task_stages[robot_id] == "go_to_picker":
                    # goal is picker node as in the task
                    goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
                    # if self.robot_task_id[robot_id] in self.processing_tasks:
                    #     goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
                    # else:  # if task has been cancelled, misatch occured,
                    #     logmsg(level='error', category='robot', id=robot_id, msg='mismatch between processing_tasks and robot_task_id')
                    #     goal_node = start_node
                    #     self.task_stages[robot_id] == "go_to_base"
                    #     continue
                elif self.task_stages[robot_id] == "go_to_storage":
                    # goal is storage node
                    goal_node = robot.current_storage
                elif self.task_stages[robot_id] == "go_to_base":
                    # goal is robot's base node
                    goal_node = robot.base_station
                    logmsg(category="robot", id=robot_id, msg='going to base, target is %s'%(goal_node))



                """if current node is goal node, generate empty route and set task as finished"""
                if start_node == goal_node:
                    # this is a moving robot, so must be in a go_to_task stage (picker, storage or base)
                    if self.task_stages[robot_id] == "go_to_storage":
                        self.finish_task_stage(robot_id, self.task_stages[robot_id])
                    elif self.task_stages[robot_id] == "go_to_picker":
                        self.finish_task_stage(robot_id, self.task_stages[robot_id])
                        task_id = self.robot_task_id[robot_id]
                        self.publish_task_state(task_id, robot_id, "ARRIVED")
                        self.write_log({"action_type": "car_update",
                            "task_updates": "ARRIVED",
                            "task_id": task_id,
                            })
                    elif self.task_stages[robot_id] == "go_to_base":
                        self.finish_task_stage(robot_id, self.task_stages[robot_id])
                        remove(self.moving_robots, robot_id)
                        remove(self.active_robots, robot_id)
                        remove(self.active_interruptable_robots, robot_id)
                        add(self.idle_robots, robot_id)
                        logmsg(category="list", msg='idle robots: %s' % (str(self.idle_robots)))
                    else:
                        self.finish_task_stage(robot_id, self.task_stages[robot_id])
                    # reset routes and route_edges
                    robot.route = [start_node]
                    robot.route_edges = []
                    self.get_edge_distances(robot_id)
                    continue


                """unblock nodes for picker location and robot location"""
                avail_topo_map = copy.deepcopy(self.available_topo_map)
                # make the robot's current node available
                if robot.current_node != "none":
                    avail_topo_map = self.unblock_node(avail_topo_map, robot.current_node)
                # make goal_node available only for a picker_node, in other cases it could be blocked
                if self.task_stages[robot_id] == "go_to_picker":
                    avail_topo_map = self.unblock_node(avail_topo_map, goal_node)


                """generate route from start node to goal node"""
                avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                route = avail_route_search.search_route(start_node, goal_node)
                route_nodes = []
                route_edges = []


                """if route is not available, replan route to wait node"""
                if (route is None and
                    self.task_stages[robot_id] == "go_to_storage" and
                    robot.wait_node != "none" and
                    robot.wait_node != robot.current_node):
                    logmsg(category="robot", id=robot_id, msg='no route to target %s, moving to wait at %s' % (robot.current_storage, robot.wait_node))
                    goal_node = robot.wait_node
                    avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                    route = avail_route_search.search_route(start_node, goal_node)

                """if still no route to wait node or goal node"""
                if route is None:
                    logmsg(category="robot", id=robot_id, msg='no route found from %s to %s' % (start_node, goal_node))
                    #TODO: see how we could improve this by generating wait_node dynamically based on map activity
                else:
                    route_nodes = route.source
                    route_edges = route.edge_id
                    # add goal_node to route_nodes as it could be a critical point
                    route_nodes.append(goal_node)

                """save route details"""
                robot.route = route_nodes
                robot.route_edges = route_edges
                self.get_edge_distances(robot_id)

            else:
                """if robot is inactive, mark current node as route so as to not interfere with robot"""
                # put the current node of the idle robots as their route - to avoid other robots planning routes through those nodes
                if robot.current_node != "none": #TODO: sometimes unregistered robot is between 2 nodes
                    robot.route = [robot.current_node]
                elif robot.previous_node != "none":
                    robot.route = [robot.previous_node]
                else:
                    robot.route = [robot.closest_node]
                robot.route_edges = []
                self.get_edge_distances(robot_id)

        """for each picker/virtual picker, mark current position as node to make routing not interfere"""
        for agent in self.picker_manager.agent_details.values():
            if agent.current_node != "none":
                agent.route = [agent.current_node]
            elif agent.previous_node != "none":
                agent.route = [agent.previous_node]
            else:
                agent.route = [agent.closest_node]
            agent.route_edges = []

        # find critical points and fragment routes to avoid critical point collistions
        for i in range(10):
            locked = self.task_lock.acquire(False)
            if locked:
                break
            rospy.sleep(0.05)
        if locked:
            # restrict finding critical_path as task may get cancelled and moved
            # from processing_tasks
            self.split_critical_paths()
            self.task_lock.release()

    def run(self):
        """the main loop of the coordinator
        """
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

            if self.force_robot_status_check:
                # check robot status
                self.check_robot_status()
                self.force_robot_status_check = False

            available_robots = set(self.idle_robots + self.active_interruptable_robots)

            # if there are tasks present and there are registered robots able to take them on
            if available_robots.intersection(self.robot_manager.get_registered_list()) and not self.tasks.empty():
                logmsg(category="task", msg='unassigned task present, %s idle robots available' % (len(self.idle_robots)))
                logmsg(category="list", msg='idle robots: %s' % (str(self.idle_robots)))
                # check robot status of idle robots before assigning tasks
                self.check_robot_status()
                # try to assign all tasks
                rospy.sleep(0.2)
                self.assign_tasks()

            # check progress of active robots
            self.handle_tasks()

            # replan if needed
            if self.trigger_replan:
                #logmsg(msg='replanning now')
                # check for critical points replan
                self.replan()
                self.trigger_replan = False

                # assign first fragment of each robot
                self.set_execute_policy_routes()

    def check_robot_status(self):
        """Mark robots are unhealthy and pend for unregistration is battery level is low"""
        for robot in self.robot_manager.agent_details.values():
            if robot.battery_voltage < self.low_battery_voltage:
                robot.healthy = False
                req = rasberry_coordination.srv.UnregisterRobot
                req.robot_id = robot.robot_id
                self.unregister_robot_ros_srv(req)
                self.write_log({"action_type": "robot_update",
                                "details": "robot is unfit - low battery",
                                "robot_id": robot.robot_id,
                                "current_node": robot.current_node,
                                "closest_node": robot.closest_node,
                                })

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
        for robot_id in self.robot_ids:
            if robot_id in self.active_robots:
                self.robots[robot_id].cancel_execpolicy_goal()
                self.robots[robot_id].cancel_toponav_goal()

    def update_current_storage(self, robot_id):
        """set the current_storage node of the robot. If the robot is idle,
        this is None. If use_cold_storage is set, it is cold_storage node.
        Else, it is the local storage nearest  to the picker_node

        Keyword arguments:
            robot_id -- robot_id
        """
        robot = self.robot_manager.agent_details[robot_id]
        if robot_id not in self.active_robots:
            robot.current_storage = None
        elif self.use_cold_storage:
            robot.current_storage = self.cold_storage
        else:
            # get the closest local storage near the picker location
            task = self.processing_tasks[self.robot_task_id[robot_id]]
            picker_node = task.start_node_id
            min_dist = float("inf")
            for storage in self.local_storages:
                _, _, dists = self.get_path_details(picker_node, storage)
                if sum(dists) < min_dist:
                    robot.current_storage = storage

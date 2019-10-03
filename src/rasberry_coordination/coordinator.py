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

import std_msgs.msg
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils
import nav_msgs.msg
import geometry_msgs.msg
import thorvald_base.msg

import rasberry_coordination.robot
import rasberry_coordination.srv


class Coordinator:
    """
    """
    def __init__(self, local_storage, charging_node, base_stations, wait_nodes,
                 robot_ids, picker_ids, virtual_picker_ids, max_task_priorities, low_battery_voltage):
        """
        """
        self.ns = "/rasberry_coordination/"

        self.robot_ids = robot_ids
        self.human_picker_ids = picker_ids
        self.virtual_picker_ids = virtual_picker_ids

        # TODO:
        # Assuming only one local storage
        # Assuming each robot has a base station node
        # Assuming one charging node
        self.storage = local_storage
        self.charging_node = charging_node
        self.base_stations = base_stations
        self.wait_nodes = wait_nodes
        self.max_task_priorities = max_task_priorities
        self.low_battery_voltage = low_battery_voltage

        self.task_lock = threading.Lock()

        logs_dir = os.path.join(os.environ["HOME"], "rasberry_coordination_logs")
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
        self.dt = datetime.datetime.now()
        self.log_file = open(os.path.join(logs_dir, self.dt.strftime("%Y%m%d-%H%M%S_coordinator.csv")), "w")
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
        self.robot_states = {robot_id:0 for robot_id in self.robot_ids}
        self.robots = {robot_id: rasberry_coordination.robot.Robot(robot_id) for robot_id in self.robot_ids}

        # collect_tray_stages = ["go_to_picker", "wait_loading", "go_to_storage", "wait_unloading", "got_to_base"]
        self.trigger_replan = False
        self.routes = {robot_id:[] for robot_id in self.robot_ids}
        self.route_dists = {robot_id:[] for robot_id in self.robot_ids}
        self.route_edges = {robot_id:[] for robot_id in self.robot_ids}
        self.route_fragments = {robot_id:[] for robot_id in self.robot_ids}
        self.edge_policy_routes = {} # {robot_id: }
        self.route_publishers = {robot_id:rospy.Publisher("%s/current_route" %(robot_id), nav_msgs.msg.Path, latch=True, queue_size=5) for robot_id in self.robot_ids}
        # publish empty current route
        for robot_id in self.robot_ids:
            self.publish_route(robot_id)

        self.task_stages = {robot_id: None for robot_id in self.robot_ids} # keeps track of current stage of the robot
        self.start_time = {robot_id: rospy.get_rostime() for robot_id in self.robot_ids}

        self.idle_robots = self.robot_ids + [] # a copy of robot_ids
        self.active_robots = [] # all robots executing a task
        self.moving_robots = [] # all robots which are having an active topo_nav goal (not waiting before critical points for clearance)
        self.unfit_robots = [] # robots which should be taked away for maintenance. if doing a task now, move to this as soon as it is finished
        # intermediatory lists
        self.healthy_robots = [] # a list between unfit and idle to avoid idle_robot being modified during task assignments
        self.unhealthy_robots = [] # a list between idle and unfit to avoid idle_robot being modified during task assignments
        self.force_robot_status_check = False

        self.topo_map = None
        self.rec_topo_map = False
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self._map_cb)
        rospy.loginfo("Waiting for Topological map ...")
        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        rospy.loginfo("Received for Topological map ...")
        self.available_topo_map = copy.deepcopy(self.topo_map)

        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

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

        self.battery_voltage = {robot_id:55.0 for robot_id in self.robot_ids}
        self.battery_data_subs = {robot_id:rospy.Subscriber(robot_id+"/battery_data",
                                                             thorvald_base.msg.BatteryArray,
                                                             self._battery_data_cb,
                                                             callback_args=robot_id) for robot_id in self.robot_ids}

        self.max_task_priorities = max_task_priorities

        self.tray_loaded = {robot_id:False for robot_id in self.robot_ids}
        self.tray_unloaded = {robot_id:True for robot_id in self.robot_ids}

        self.advertise_services()
        # don't queue more than 1000 tasks
        self.tasks = Queue.PriorityQueue(maxsize=1000)
        self.last_id = 0

        self.all_task_ids = []
        self.processing_tasks = {}
        self.completed_tasks = {}
        self.cancelled_tasks = {}
        self.failed_tasks = {}

        self.task_robot_id = {}
        self.robot_task_id = {robot_id: None for robot_id in self.robot_ids}
        self.task_state_msg = rasberry_coordination.msg.TaskUpdates()

        # TODO: these durations should come from a config
        self.max_load_duration = rospy.Duration(secs=60)
        self.max_unload_duration = rospy.Duration(secs=10)

        self.picker_task_updates_pub = rospy.Publisher("/rasberry_coordination/task_updates", rasberry_coordination.msg.TaskUpdates, queue_size=5)

        rospy.loginfo("coordinator initialised")

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
        """get the state of a robot

        Keyword arguments:

        robot_id -- robot_id
        """
        state, goal_node, start_time = "", "", rospy.Time()
        if self.task_stages[robot_id] is not None:
            state = self.task_stages[robot_id]
            if state == "go_to_picker":
                goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
            elif state == "go_to_storage":
                goal_node = self.storage
            elif state  == "go_to_base":
                goal_node = self.base_stations[robot_id]
            else:
                goal_node = ""
        elif robot_id in self.idle_robots:
            state = "idle"
            goal_node = ""
        elif robot_id in self.unfit_robots:
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
        self.tray_unloaded[req.robot_id] = False
        self.tray_loaded[req.robot_id] = True

        self.write_log({"action_type": "tray_loaded_srv",
                        "robot_id": req.robot_id,
                        })
        return []

    tray_loaded_ros_srv.type = rasberry_coordination.srv.TrayLoaded

    def get_robot_state_ros_srv(self, req):
        """get the state of a robot
        """
        resp = rasberry_coordination.srv.RobotStateResponse()
        if req.robot_id in self.robot_ids:
            resp.state, resp.goal_node, resp.start_time = self._get_robot_state(req.robot_id)
        else:
            err_msg = "%s is not a among the robots configured" %(req.robot_id)
            rospy.logerr(err_msg)
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
                err_msg = "%s is not a among the robots configured" %(robot_id)
                rospy.logerr(err_msg)
        return resp

    get_robot_states_ros_srv.type = rasberry_coordination.srv.RobotStates

    def add_task_ros_srv(self, req):
        """Adds a task into the task execution framework.
        """
        self.last_id += 1
        task_id = self.last_id

        self.write_log({"action_type": "car_update",
                        "task_updates": "CALLED",
                        "task_id": task_id,
                        "details": "to %s" %(req.task.start_node_id),
                        })

        req.task.task_id = task_id
        rospy.loginfo('received task: %s to %s', req.task.task_id, req.task.start_node_id)
        self.tasks.put(
            (task_id, req.task)
        )
        self.all_task_ids.append(task_id)
        self.write_log({"action_type":"add_task_srv",
                        "task_id": task_id,
                        })
        return task_id

    add_task_ros_srv.type = strands_executive_msgs.srv.AddTask

    def cancel_task_ros_srv(self, req):
        """cancels a task from execution
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

        locked = self.task_lock.acquire(False)

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
                    rospy.logerr("cancel_task_ros_srv cannot be in this condition")

                elif req.task_id in self.processing_tasks:
                    # task is being processed. remove it
                    task = self.processing_tasks.pop(req.task_id)
                    self.cancelled_tasks[req.task_id] = task
                    # cancel goal of assigned robot and return it to its base
                    if req.task_id in self.task_robot_id:
                        robot_id = self.task_robot_id[req.task_id]
                        self.robots[robot_id].cancel_execpolicy_goal()
                        self.send_robot_to_base(robot_id)
                    rospy.loginfo("cancelling task-%d", req.task_id)
                    cancelled = True

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
                                rospy.loginfo("cancelling task-%d", req.task_id)
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

                    self.write_log({"action_type":"cancel_task_srv",
                                    "task_id": req.task_id,
                                    "details": "success",
                                    })
            else:
                # invalid task_id
                rospy.logerr("cancel_task is invoked with invalid task_id")
                self.write_log({"action_type": "cancel_task_srv",
                                    "task_id": req.task_id,
                                    "details": "fail: invalid task_id",
                                    })

            self.task_lock.release()
        else:
            rospy.logerr("tasks are being assigned. cancel service call failed.")
            self.write_log({"action_type": "cancel_task_srv",
                                "task_id": req.task_id,
                                "details": "fail: tasks could not be locked. task assignment going on",
                                })

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

    def set_healthy_robot_ros_srv(self, req):
        """Set a robot as healthy if marked as unfit. will then be moved to idle.
        robot must be already set as unfit
        """
        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

            self.write_log({"action_type": "set_healthy_robot_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not configured",
                            })
            return resp

        if req.robot_id in self.unfit_robots:
            self.unfit_robots.remove(req.robot_id)
            # move to healthy_robots. will be added to idle during next low_battery_check
            self.healthy_robots.append(req.robot_id)
            self.force_robot_status_check = True
            resp.success = True
            resp.message = "Robot - %s is queued to be marked as healthy" %(req.robot_id)

            self.write_log({"action_type": "set_healthy_robot_srv",
                            "robot_id": req.robot_id,
                            "details": "success",
                            })
        else:
            resp.success = False
            resp.message = "Robot - %s is not unfit to be marked as healthy now" %(req.robot_id)

            self.write_log({"action_type": "set_healthy_robot_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not unfit",
                            })
        return resp

    set_healthy_robot_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def set_unhealthy_robot_ros_srv(self, req):
        """Set a robot as unhealthy if marked as unhealthy which will then be moved to unfit.
        robot must be a configured one and is not set as unhealthy or unfit already
        """
        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

            self.write_log({"action_type": "set_unhealthy_robot_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not configured",
                            })
            return resp

        if req.robot_id not in self.unhealthy_robots and req.robot_id not in self.unfit_robots:
            self.unhealthy_robots.append(req.robot_id)
            self.force_robot_status_check = True
            resp.success = True
            resp.message = "Robot - %s is queued to be marked as unhealthy" %(req.robot_id)

            self.write_log({"action_type": "set_unhealthy_robot_srv",
                            "robot_id": req.robot_id,
                            "details": "success",
                            })

        else:
            resp.success = False
            resp.message = "Robot - %s is not healthy to be marked as unhealthy now" %(req.robot_id)

            self.write_log({"action_type": "set_unhealthy_robot_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not healthy",
                            })

        return resp

    set_unhealthy_robot_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def set_robot_reached_picker_ros_srv(self, req):
        """Set a robot reached the storage. finish the go_to_picker stage. robot will go to wait_loading
        """
        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

            self.write_log({"action_type": "set_robot_reached_picker_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not configured",
                            })
            return resp

        if req.robot_id in self.moving_robots and self.task_stages[req.robot_id] == "go_to_picker":
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

            self.write_log({"action_type": "set_robot_reached_picker_srv",
                            "robot_id": req.robot_id,
                            "details": "success",
                            })
        else:
            resp.success = False
            resp.message = "Robot - %s is not in moving_robots now" %(req.robot_id)

            self.write_log({"action_type": "set_robot_reached_picker_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not in moving_robots now",
                            })

        return resp

    set_robot_reached_picker_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def set_robot_reached_storage_ros_srv(self, req):
        """Set a robot reached the storage. finish the go_to_storage stage. robot will go to wait_unloading
        """
        resp = rasberry_coordination.srv.TriggerRobotResponse()
        if req.robot_id not in self.robot_ids:
            resp.success = False
            resp.message = "Robot - %s is not configured" %(req.robot_id)

            self.write_log({"action_type": "set_robot_reached_storage_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not configured",
                            })
            return resp

        if req.robot_id in self.moving_robots and self.task_stages[req.robot_id] == "go_to_storage":
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

            self.write_log({"action_type": "set_robot_reached_storage_srv",
                            "robot_id": req.robot_id,
                            "details": "success",
                            })
        else:
            resp.success = False
            resp.message = "Robot - %s is not in moving_robots now" %(req.robot_id)

            self.write_log({"action_type": "set_robot_reached_storage_srv",
                            "robot_id": req.robot_id,
                            "details": "fail: Robot is not in moving_robots now",
                            })

        return resp

    set_robot_reached_storage_ros_srv.type = rasberry_coordination.srv.TriggerRobot

    def advertise_services(self):
        """Adverstise ROS services.
        Only call at the end of constructor to avoid calls during construction.
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
                rospy.loginfo('advertised %s', attr[:-8])

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

    def send_robot_to_base(self, robot_id):
        """send robot to base. very specific goal.
        happens after task is finished (unloaded trays) or task failed
        """
        rospy.loginfo("sending %s to its base", robot_id)

        if self.current_nodes[robot_id] == self.base_stations[robot_id]:
            rospy.loginfo("%s already at its base station", robot_id)
            # already at base station. set as idle
            if robot_id in self.active_robots:
                self.active_robots.remove(robot_id)
            if robot_id in self.moving_robots:
                self.moving_robots.remove(robot_id)
            if robot_id not in self.idle_robots:
                self.idle_robots.append(robot_id)
        else:
            rospy.loginfo("%s not at its base station. setting goal as base", robot_id)
            # not at base. also not idle. set stage as send to base
            if robot_id in self.idle_robots:
                self.idle_robots.remove(robot_id)
                self.active_robots.append(robot_id)
            self.task_stages[robot_id] = "go_to_base"

            self.write_log({"action_type": "robot_update",
                            "robot_task_stage": "go_to_base_start",
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })

    def find_closest_robot(self, task):
        """find the robot closest to the task location (picker_node)
        # assign based on priority (0 - virtual pickers only, >=1 real pickers)

        Keyword arguments:

        task - strands_executive_msgs.Task
        """
        goal_node = task.start_node_id
        robot_dists = {}

        for robot_id in self.idle_robots:
            # ignore if the robot's closest_node and current_node is not yet available
            if robot_id not in self.closest_nodes and robot_id not in self.current_nodes:
                continue

            # ignore if the task priority is less than the min task priority for the robot
            if task.priority > self.max_task_priorities[robot_id]:
                continue

            # use current_node as start_node if available
            if self.current_nodes[robot_id] != "none":
                start_node = self.current_nodes[robot_id]
            else:
                start_node = self.closest_nodes[robot_id]

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

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:

        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_search.search_route(start_node, goal_node)
        if route is None:
            rospy.loginfo("no route between %s and %s", start_node, goal_node)
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

    def get_edge_distances(self, robot_id):
        """find and fill distances of all edges of a robot's planned route, if at least one edge is there.
        the route must contain the goal_node as the last node in the list.

        Keyword arguments:

        robot_id --
        """
        self.route_dists[robot_id] = []
        if len(self.route_edges[robot_id]) >= 1:
            for i in range (len(self.routes[robot_id]) - 1):
                self.route_dists[robot_id].append(self.get_distance_between_adjacent_nodes(self.routes[robot_id][i], self.routes[robot_id][i+1]))

    def get_route_distance_to_node(self, robot_id, node_id):
        """get the total distance to a node in a robot's route

        Keyword arguments:

        robot_id --
        node_id --
        """
        dist = 0.0
        if len(self.route_edges[robot_id]) > 1:
            for i in range(len(self.routes[robot_id])):
                if self.routes[robot_id][i] == node_id:
                    break
                # add edge_distance only if the source node is not the one we look for
                dist += self.route_dists[robot_id][i]
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
                    rospy.loginfo("selected robot-%s to task %d", robot_id, task_id)

                    # trigger replan for any new assignment
                    trigger_replan = True

                    tasks.remove((task_id, task))
                    self.active_robots.append(robot_id)
                    self.idle_robots.remove(robot_id)
                    self.task_stages[robot_id] = "go_to_picker"

                    self.processing_tasks[task_id] = task
                    self.task_robot_id[task_id] = robot_id
                    self.robot_task_id[robot_id] = task_id

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
                                    "current_node": self.current_nodes[robot_id],
                                    "closest_node": self.closest_nodes[robot_id],
                                    })

            # putting unassigned tasks back in the queue
            for (task_id, task) in tasks:
                self.tasks.put((task_id, task))

            self.task_lock.release()

        self.trigger_replan = self.trigger_replan or trigger_replan

    def finish_route_fragment(self, robot_id):
        """finish fragment of a route in a task stage
        """
        self.publish_route(robot_id)
        if robot_id in self.moving_robots:
            self.moving_robots.remove(robot_id)
        # this may be called multiple times when a robot is stuck
        # so being cautious here
        try:
            self.routes.pop(robot_id)
            self.route_fragments.pop(robot_id)
        except KeyError:
            pass
        self.robots[robot_id].execpolicy_result = None

    def finish_task_stage(self, robot_id, curr_stage=None):
        """finish a stage of the collect tray
        """
        next_stage = {"go_to_picker":"wait_loading",
                      "wait_loading":"go_to_storage",
                      "go_to_storage":"wait_unloading",
                      "wait_unloading":"go_to_base",
                      "go_to_base":None}
        if curr_stage in ["go_to_picker", "go_to_storage", "go_to_base"]:
            self.finish_route_fragment(robot_id)

        self.write_log({"action_type": "robot_update",
                        "robot_task_stage": curr_stage+"_finish",
                        "task_id": self.robot_task_id[robot_id] if self.robot_task_id[robot_id] is not None else "",
                        "robot_id": robot_id,
                        "current_node": self.current_nodes[robot_id],
                        "closest_node": self.closest_nodes[robot_id],
                        })

        rospy.loginfo("setting %s's state from %s to %s", robot_id, curr_stage, next_stage[curr_stage])
        self.task_stages[robot_id] = next_stage[curr_stage]
        self.start_time[robot_id] = rospy.get_rostime()

        if curr_stage != "go_to_base":
            self.write_log({"action_type": "robot_update",
                            "robot_task_stage": next_stage[curr_stage]+"_start",
                            "task_id": self.robot_task_id[robot_id] if self.robot_task_id[robot_id] is not None else "",
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })

    def finish_task(self, robot_id):
        """set the task assigned to the robot as finished whenever trays are unloaded
        """
        self.finish_task_stage(robot_id, "wait_unloading")
        # move task from processing to completed
        self.task_stages[robot_id] = "go_to_base"
        task_id = self.robot_task_id[robot_id]
        self.robot_task_id[robot_id] = None
        self.completed_tasks[task_id] = self.processing_tasks.pop(task_id)
        # move robot from moving robots
        if robot_id in self.moving_robots:
            self.moving_robots.remove(robot_id)

        self.write_log({"action_type": "task_update",
                        "task_updates": "task_finish",
                        "task_id": task_id,
                        "robot_id": robot_id,
                        })

    def set_task_failed(self, task_id):
        """set task state as failed
        """
        task = self.processing_tasks.pop(task_id)
        self.failed_tasks[task_id] = task

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
        robot_id = self.task_robot_id[task_id]
        # this task should be readded to the queue if not cancelled by the picker !!!
        if task_id in self.processing_tasks:
            rospy.loginfo("Assigned robot failed to reach picker, adding the task back into the queue")
            self.task_robot_id.pop(task_id) # remove from the assigned robot
            task = self.processing_tasks.pop(task_id) # remove from processing tasks

            self.publish_task_state(task_id, robot_id, "CALLED")

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
            task_id = self.robot_task_id[robot_id]

            if robot_id in self.moving_robots:
                # topo nav stage
                # if any robot has finished its current goal, remove the finished goal from the robot's route
                if self.robots[robot_id].execpolicy_result is None:
                    # task/fragment not finished
                    continue

                self.publish_route(robot_id)

                # check for robots which are moving, not waiting before a critical point
                if self.robots[robot_id].execpolicy_result.success:
                    # trigger replan whenever a segment comppletion is reported
                    trigger_replan = True
                    # if the robot's route is finished, progress to the next stage of the collect tray process
                    # has it finished the stage?
                    goal_node = None
                    if self.task_stages[robot_id] == "go_to_picker":
                        # goal is picker node as in the task
                        goal_node = self.processing_tasks[task_id].start_node_id
                    elif self.task_stages[robot_id] == "go_to_storage":
                        # goal is storage node
                        goal_node = self.storage
                    elif self.task_stages[robot_id] == "go_to_base":
                        # goal is robot's base node
                        goal_node = self.base_stations[robot_id]

                    if (len(self.route_fragments[robot_id]) == 1 and
                        self.current_nodes[robot_id] is not None and
                        self.current_nodes[robot_id] == goal_node):
                        # finished the stage
                        if self.task_stages[robot_id] == "go_to_picker":
                            # go_to_picker stage is finished
                            self.finish_task_stage(robot_id, "go_to_picker")
                            self.publish_task_state(task_id, robot_id, "ARRIVED")
                            self.write_log({"action_type": "car_update",
                                            "task_updates": "ARRIVED",
                                            "task_id": task_id,
                                            })

                        elif self.task_stages[robot_id] == "go_to_storage":
                            # go_to_storage stage is finished
                            self.finish_task_stage(robot_id, "go_to_storage")
                            self.publish_task_state(task_id, robot_id, "STORAGE")
                            self.write_log({"action_type": "car_update",
                                            "task_updates": "STORAGE",
                                            "task_id": task_id,
                                            })

                        elif self.task_stages[robot_id] == "go_to_base":
                            # task is finished
                            self.finish_task_stage(robot_id, "go_to_base")
                            if robot_id in self.robot_task_id:
                                self.robot_task_id.pop(robot_id)
                            self.task_stages[robot_id] = None
                            # move robot from active to idle
                            self.active_robots.remove(robot_id)
                            self.idle_robots.append(robot_id)

                    else:
                        # finished only a fragment. may have to wait for clearance
                        self.finish_route_fragment(robot_id)

                else:
                    trigger_replan = True # triggger replan as robot is being sent back to base
                    # robot failed execution
                    rospy.loginfo("%s failed to complete task %s at stage %s!!!" , robot_id, task_id, self.task_stages[robot_id])
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
                        self.send_robot_to_base(robot_id)

            else:
                # wait_loading or wait_unloading
                self.publish_route(robot_id)
                if self.task_stages[robot_id] == "wait_loading":
                    # if conditions are satisfied, finish waiting
                    # 1. LOADED from CAR
                    # 2. service call from active_compliance
                    # 3. delay (?)
                    finish_waiting = False
                    if self.tray_loaded[robot_id]:
                        finish_waiting = True
                    elif False:
                        # TODO: active compliance
                        pass
                    elif rospy.get_rostime() - self.start_time[robot_id] > self.max_load_duration:
                        # delay
                        finish_waiting = True
                    else:
                        rospy.sleep(0.5)

                    if finish_waiting:
                        self.publish_task_state(task_id, robot_id, "LOADED")
                        self.write_log({"action_type": "car_update",
                            "task_updates": "LOADED",
                            "task_id": task_id,
                            })

                        self.finish_task_stage(robot_id, "wait_loading")
                        self.tray_loaded[robot_id] = True
                        self.tray_unloaded[robot_id] = False
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
                        self.tray_loaded[robot_id] = False
                        self.tray_unloaded[robot_id] = True
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
        for agent_id in self.presence_agents:
            r_outer = self.routes[agent_id]
            critical_points[str(r_outer)] = set([])

            # check route of agent_id_1 to routes of all robots
            for robot_id in self.active_robots:
                if agent_id == robot_id:
                    continue
                r_inner = self.routes[robot_id]
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
            if (self.task_stages[robot_id] == "go_to_picker" and
                self.processing_tasks[self.robot_task_id[robot_id]].start_node_id in c_points[str(self.routes[robot_id])]):
                c_points[str(self.routes[robot_id])].remove(self.processing_tasks[self.robot_task_id[robot_id]].start_node_id)

#        rospy.loginfo(c_points)

        allowed_cpoints = []
        res_routes = {}
        for agent_id in self.presence_agents:
            allowed_to_pass = False
            r = self.routes[agent_id]
            rr = []
            partial_route = []
            for v in r:
                if v in c_points[str(r)]: # vertice is critical point
                    # allow critical point once for a robot among all agents
                    # once a critical point is passed
                    nearest_robot = self.shortest_route_to_node(c_robots[v], v)
                    if (agent_id == nearest_robot and
                        v not in allowed_cpoints):# and
                        #not allowed_to_pass):
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

        self.route_fragments = res_routes

        res_edges = {}
        # split the edges as per the route_fragments
        for robot_id in self.active_robots:
            if self.route_fragments[robot_id]:
                # remove goal node from last fragment
                # if start and goal nodes are different, there will be at least one node remaining and an edge
                self.route_fragments[robot_id][-1].pop(-1)
                # move the last node of all fragments to the start of next fragment
                for i in range(len(self.route_fragments[robot_id]) - 1):
                    self.route_fragments[robot_id][i+1].insert(0, self.route_fragments[robot_id][i][-1])
                    self.route_fragments[robot_id][i].pop(-1)

                # split the edges
                res_edges[robot_id] = []
                for i in range(len(self.route_fragments[robot_id])):
                    res_edges[robot_id].append(self.route_edges[robot_id][:len(self.route_fragments[robot_id][i])])
                    self.route_edges[robot_id] = self.route_edges[robot_id][len(self.route_fragments[robot_id][i]):]
            else:
                self.route_fragments[robot_id] = []
                res_edges[robot_id] = []

        self.route_edges = res_edges

    def set_execute_policy_routes(self, ):
        """find connecting edges for each fragment in route_fragments and set
        the corresponding route object (with source and edge_id)
        """
        for robot_id in self.active_robots:
            goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
            if self.route_fragments[robot_id]:
                goal.route.source = self.route_fragments[robot_id][0]
                goal.route.edge_id = self.route_edges[robot_id][0]

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
                    rospy.loginfo(robot_id)
                    rospy.loginfo("new_goal")
                    rospy.loginfo(goal)
                    rospy.loginfo("prev_goal")
                    rospy.loginfo(self.robots[robot_id].execpolicy_goal)

                    self.write_log({"action_type": "robot_update",
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })

                    self.write_log({"action_type": "robot_update",
                            "details": "new_route",
                            "source": str(goal.route.source),
                            "edge_ids": str(goal.route.edge_id),
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })

                    self.robots[robot_id].set_execpolicy_goal(goal)
                    self.publish_route(robot_id, goal.route.source, goal.route.edge_id)
                    if goal.route.edge_id and robot_id not in self.moving_robots:
                        self.moving_robots.append(robot_id)

    def set_empty_execpolicy_goal(self, robot_id):
        """for intermediate cancellation, sending another empty goal to preempt
        current goal
        """
        goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
        self.publish_route(robot_id)
        self.robots[robot_id].set_execpolicy_goal(goal)

    def publish_route(self, robot_id, source=[], edge_id=[]):
        """publish route that can be visualised in rviz
        """
        route = nav_msgs.msg.Path()
        route.header.frame_id = "map"
        if source:
            for node in source:
                # if there is any elements in the route
                node_obj = self.get_node(node)
                pose_stamped = geometry_msgs.msg.PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = node_obj.pose.position.x
                pose_stamped.pose.position.y = node_obj.pose.position.y
                route.poses.append(pose_stamped)

            # add the goal node, by looking for the edge in the last source node
            for edge in node_obj.edges:
                if edge.edge_id == edge_id[-1]:
                    node_obj = self.get_node(edge.node)
                    pose_stamped = geometry_msgs.msg.PoseStamped()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose.position.x = node_obj.pose.position.x
                    pose_stamped.pose.position.y = node_obj.pose.position.y
                    route.poses.append(pose_stamped)
                    break

        self.route_publishers[robot_id].publish(route)

    def replan(self, ):
        """replan - find indiviual paths, find critical points in these paths, and fragment the
        paths at critical points - whenever triggered
        """
        for robot_id in self.robot_ids:
            if robot_id in self.active_robots:
                if self.task_stages[robot_id] in ["wait_loading", "wait_unloading"]:
                    # loading and unloading robots should finish those stages first
                    # put the current node of the idle robots as their route - to avoid other robots planning routes through those nodes
                    if self.current_nodes[robot_id] != "none":
                        self.routes[robot_id] = [self.current_nodes[robot_id]]
                    elif self.prev_current_nodes[robot_id] != "none":
                        self.routes[robot_id] = [self.prev_current_nodes[robot_id]]
                    else:
                        self.routes[robot_id] = [self.closest_nodes[robot_id]]
                    self.route_edges[robot_id] = []
                    self.get_edge_distances(robot_id)
                    continue

                if self.current_nodes[robot_id] != "none":
                    start_node = self.current_nodes[robot_id]
                elif self.prev_current_nodes[robot_id] != "none":
                    start_node = self.prev_current_nodes[robot_id]
                else:
                    start_node = self.closest_nodes[robot_id]

                goal_node = None
                if self.task_stages[robot_id] == "go_to_picker":
                    # goal is picker node as in the task
                    goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
                elif self.task_stages[robot_id] == "go_to_storage":
                    # goal is storage node
                    goal_node = self.storage
                elif self.task_stages[robot_id] == "go_to_base":
                    # goal is robot's base node
                    goal_node = self.base_stations[robot_id]

#                rospy.loginfo("start: %s, goal: %s", start_node, goal_node)
                # if current node is goal node, don't search for a path, but set the task stage as finished
                if start_node == goal_node:
                    # this is a moving robot, so must be in a go_to task stage (picker, storage or base)
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
                        if robot_id in self.moving_robots:
                            self.moving_robots.remove(robot_id)
                        self.active_robots.remove(robot_id)
                        self.idle_robots.append(robot_id)
                    else:
                        self.finish_task_stage(robot_id, self.task_stages[robot_id])
                    #reset routes and route_edges
                    self.routes[robot_id] = [start_node]
                    self.route_edges[robot_id] = []
                    self.get_edge_distances(robot_id)
                    continue

                avail_topo_map = copy.deepcopy(self.available_topo_map)
                # make goal_node available only for a picker_node, in other cases it could be blocked
                if self.task_stages[robot_id] == "go_to_picker":
                    avail_topo_map = self.unblock_node(avail_topo_map, goal_node)

                avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                route = avail_route_search.search_route(start_node, goal_node)
                route_nodes = []
                route_edges = []

                # replan to wait_station if in go_to_storage and there is no route
                if (route is None and
                    self.task_stages[robot_id] == "go_to_storage" and
                    self.wait_nodes[robot_id] != "none" and
                    self.wait_nodes[robot_id] != self.current_nodes[robot_id]):
                    rospy.loginfo("%s has no route to %s. will try going to %s to wait there", robot_id, self.storage, self.wait_nodes[robot_id])
                    goal_node = self.wait_nodes[robot_id]
                    avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_topo_map)
                    route = avail_route_search.search_route(start_node, goal_node)

                if route is None:
                    rospy.loginfo("no route between %s and %s", start_node, goal_node)
                else:
                    route_nodes = route.source
                    route_edges = route.edge_id
                    # add goal_node to route_nodes as it could be a critical point
                    route_nodes.append(goal_node)

                self.routes[robot_id] = route_nodes
                self.route_edges[robot_id] = route_edges
                self.get_edge_distances(robot_id)

            else:
                # put the current node of the idle robots as their route - to avoid other robots planning routes through those nodes
                if self.current_nodes[robot_id] != "none":
                    self.routes[robot_id] = [self.current_nodes[robot_id]]
                elif self.prev_current_nodes[robot_id] != "none":
                    self.routes[robot_id] = [self.prev_current_nodes[robot_id]]
                else:
                    self.routes[robot_id] = [self.closest_nodes[robot_id]]
                self.route_edges[robot_id] = []
                self.get_edge_distances(robot_id)

        for agent_id in self.presence_agents:
            if agent_id not in self.robot_ids:
                # all robot_ids have routes are already defined
                if self.current_nodes[agent_id] != "none":
                    self.routes[agent_id] = [self.current_nodes[agent_id]]
                elif self.prev_current_nodes[agent_id] != "none":
                    self.routes[agent_id] = [self.prev_current_nodes[agent_id]]
                else:
                    self.routes[agent_id] = [self.closest_nodes[agent_id]]
                self.route_edges[agent_id] = []
#                self.get_edge_distances(robot_id)

        # find critical points and fragment routes to avoid critical point collistions
        self.split_critical_paths()

    def run(self):
        """the main loop of the coordinator
        """
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            self.trigger_replan = False

            if self.force_robot_status_check:
                # check robot status
                self.check_robot_status()
                self.force_robot_status_check = False

#            rospy.loginfo(self.idle_robots)
            if self.idle_robots and not self.tasks.empty():
                rospy.loginfo("unassigned tasks present. no. idle robots: %d", len(self.idle_robots))
                # check robot status of idle robots before assigning tasks
                self.check_robot_status()
                # try to assign all tasks
                self.assign_tasks()

            # check progress of active robots
            self.handle_tasks()

            # replan if needed
            if self.trigger_replan:
                # check for critical points replan
                self.replan()
                # assign first fragment of each robot
                self.set_execute_policy_routes()

    def check_robot_status(self):
        """check battery level of all idle robots and set as unfit, if low on battery.
        to reset an unfit robot, call service set_healthy
        """
        unfit_robots = []
        for robot_id in self.idle_robots:
            if self.battery_voltage[robot_id] < self.low_battery_voltage:
                unfit_robots.append(robot_id)

        # remove all unfit robots from idle
        for robot_id in unfit_robots:
            self.idle_robots.remove(robot_id)
            self.unfit_robots.append(robot_id)
            self.write_log({"action_type": "robot_update",
                            "details": "robot is unfit - low battery",
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })

        # move healthy robots as idle and safely clear healthy_robots
        to_remove = []
        for robot_id in self.healthy_robots:
            self.idle_robots.append(robot_id)
            to_remove.append(robot_id)
            self.write_log({"action_type": "robot_update",
                            "details": "robot is fit again - from service call",
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })
        for robot_id in to_remove:
            self.healthy_robots.remove(robot_id)

        # move unhealthy robots as unfit and safely clear unhealthy_robots
        to_remove = []
        for robot_id in self.unhealthy_robots:
            self.unfit_robots.append(robot_id)
            self.idle_robots.remove(robot_id)
            to_remove.append(robot_id)
            self.write_log({"action_type": "robot_update",
                            "details": "robot is unfit - from service call",
                            "robot_id": robot_id,
                            "current_node": self.current_nodes[robot_id],
                            "closest_node": self.closest_nodes[robot_id],
                            })
        for robot_id in to_remove:
            self.unhealthy_robots.remove(robot_id)

    def write_log(self, field_vals):
        """
        """
        row = {"id": self.log_count,
               "time":time.time(),
               "datetime": self.dt.strftime("%Y%m%d-%H%M%S"),
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

        print("shutting down all actions")
        for robot_id in self.robot_ids:
            if robot_id in self.active_robots:
                self.robots[robot_id].cancel_execpolicy_goal()
                self.robots[robot_id].cancel_toponav_goal()

if __name__ == '__main__':
    rospy.init_node('simple_task_coordinator', anonymous=True)
    c = Coordinator()
    c.run()
    rospy.spin()

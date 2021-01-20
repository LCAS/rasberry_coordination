#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import actionlib
import rospy

from rospy import Subscriber as Sub, Service as Srv, get_rostime as Now
from std_msgs.msg import String as Str
from thorvald_base.msg import BatteryArray as Battery
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak
from rasberry_coordination.robot import Robot as RobotInterface
from rasberry_coordination.agent_managers.agent_manager import AgentManager, AgentDetails
from rasberry_coordination.srv import RobotState, RobotStates, RobotStateResponse, RobotStatesResponse

class RobotManager(AgentManager):

    """Initialise class with callback details to apply to robots"""
    def __init__(self, callback_dict):
        super(RobotManager, self).__init__(callback_dict)
        self.dump_cb = Sub("rasberry_coordination/robot_manager/dump", Str, self.dump_details)
        Srv("rasberry_coordination/get_robot_state", RobotState, self.get_robot_state_ros_srv)
        Srv("rasberry_coordination/get_robot_states", RobotStates, self.get_robot_states_ros_srv)  # TODO: one not needed

    """Add Robot Details Objects"""
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = RobotDetails(agent_id, self.cb)

    """Service responses"""
    def get_robot_states_ros_srv(self, req): #TODO: combine these 2 srv into 1?
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
        resp = RobotStateResponse()
        if req.robot_id in self.agent_details:
            resp = self._get_robot_state(req.robot_id)
        return resp
    def _get_robot_state(self, robot_id):
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

    """Commonly used actions which require exeptionally high speed"""
    def registered_robots(self): #TODO: swap out to polymorphism
        return {robot.agent_id:robot.registered for robot in self.agent_details.values() if robot.registered}
    def registered_list(self): #TODO: swap out to polymorphism
        return [robot.agent_id for robot in self.agent_details.values() if robot.registered]
    def idle_list(self):
        return [robot.agent_id for robot in self.agent_details.values() if robot.idle is True]
    def moving_list(self):
        return [robot.agent_id for robot in self.agent_details.values() if robot.moving is True]
    def active_list(self):
        return [robot.agent_id for robot in self.agent_details.values() if robot.active and not robot.paused]
    def interruptable_list(self):
        return [robot.agent_id for robot in self.agent_details.values() if robot.interruptable is True]
    def idle_robots_exist(self):
        for robot in self.agent_details.values():
            if robot.idle:
                return 1
        return 0
    def moving_robots_exist(self):
        for robot in self.agent_details.values():
            if robot.idle:
                return 1
        return 0
    def available_robots(self):
        return [R.agent_id for R in self.agent_details.values() if (R.idle or R.interruptable) and R.registered]

"""Centralised container for all details pertaining to the robot"""
class RobotDetails(AgentDetails):
    def __init__(robot, ID, cb):

        """Initialise Fields in Parent Class"""
        super(RobotDetails, robot).__init__(ID, cb)

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

    """On Shutdown"""
    def _remove(robot):
        super(RobotDetails, robot)._remove()

    """return goal node as picker location, storage or base station"""
    def _get_goal_node(robot):
        if robot.task_stage == "go_to_picker":
            return robot.goal_node
        elif robot.task_stage == "go_to_storage":
            return robot.current_storage
        elif robot.task_stage == "go_to_base":
            return robot.base_station

    """State Changes"""
    def _set_as_idle(robot):
        robot.idle = True
        robot.moving = robot.active = False
        robot.interruptable = False  # not needed
    def _begin_task(robot, task_id):
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
        robot.robot_interface.cancel_execpolicy_goal()
        #robot._finish_task_stage("go_to_picker")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_reached_picker: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _tray_loaded(robot):
        robot.tray_loaded = True
        #robot._finish_task_stage("wait_loading")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_tray_loaded: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _reached_storage(robot):
        robot.robot_interface.cancel_execpolicy_goal()
        #robot._finish_task_stage("go_to_storage")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_reached_storage: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _tray_unloaded(robot):
        robot.task_id = None
        robot.tray_loaded = False
        robot.interruptable = True
        robot.moving = False
        robot.current_storage = None
        #robot._finish_task_stage("wait_unloading")
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_tray_unloaded: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _set_target_base(robot):
        robot.idle = False
        robot.active = True
        robot.interruptable = True
        robot._change_task_stage("go_to_base")
    def _end_task(robot):
        robot.task_id = None
        if robot.registered:
            robot.idle = True
        robot.active = False
        robot.interruptable = False
        robot._change_task_stage(None)

    def _finish_route_fragment(robot):
        robot.moving = False
        robot.route = []
        robot.route_fragments = []
        robot.robot_interface.execpolicy_result = None
    def _finish_task_stage(robot, state):
        if robot.task_stage in ["go_to_picker", "go_to_storage", "go_to_base"]:
            robot._finish_route_fragment()
        robot.start_time = Now()
        if len(robot.task_stage_list):
            robot._change_task_stage(state)
    def _change_task_stage(robot, stage): #TODO: these 2 functions are overengineered
        if robot.task_stage == stage:
            return
        robot.task_stage = stage
        logmsgbreak()
        logmsg(category="robot", id=robot.robot_id, msg='@ stage: %s' % (str(robot.task_stage).upper()))

    """ Registration Controls """
    def _pause_task(robot):
        robot.unregistration_type = "pause_task"
        robot.paused = True
        robot.task_stage_list.insert(0,robot.task_stage)
        robot.task_stage = "paused"
        #print("_pause_task: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
        robot.robot_interface.cancel_execpolicy_goal()
    def _unpause_robot(robot):
        robot.unregistration_type = None
        robot.paused = False
    def _unpause_task(robot):
        robot._finish_task_stage(robot.task_stage_list.pop(0))
        #print("_unpause_task: [" + str(robot.task_stage) + "] | "+str(robot.task_stage_list))
    def _drm_release_task(robot):
        robot.unregistration_type = "release_task"
        robot._release_task()
    def _release_task(robot):
        robot.goal_node = None
        robot._end_task()
        robot.robot_interface.cancel_execpolicy_goal()

        if not robot.disconnect_when_idle:
            robot.idle = False

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
        Srv("rasberry_coordination/get_robot_states", RobotStates, self.get_robot_states_ros_srv)

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
        return {deets.agent_id:deets.registered for deets in self.agent_details.values() if deets.registered}
    def registered_list(self): #TODO: swap out to polymorphism
        return [deets.agent_id for deets in self.agent_details.values() if deets.registered]
    def idle_list(self):
        return [deets.agent_id for deets in self.agent_details.values() if deets.idle is True]
    def moving_list(self):
        return [deets.agent_id for deets in self.agent_details.values() if deets.moving is True]
    def active_list(self):
        return [deets.agent_id for deets in self.agent_details.values() if deets.active is True]
    def interruptable_list(self):
        return [deets.agent_id for deets in self.agent_details.values() if deets.interruptable is True]
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
    def __init__(self, ID, cb):

        """Initialise Fields in Parent Class"""
        super(RobotDetails, self).__init__(ID, cb)

        """Detail whether the robot is moving"""
        self.idle = True
        self.interruptable = False
        self.has_toponav_goal = False
        self.active = False
        self.moving = False

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
        self.robot_id = ID
        self.disconnect_when_idle = False
        self.registered = True
        self.robot_interface = RobotInterface(ID)

        """Task Details"""
        self.task_id = None
        self.goal_node = None
        self.task_stage = None
        self.task_stage_list = []
        self.tray_loaded = False
        self.start_time = Now()

        """Task Meta Details"""
        self.max_task_priority = 255
        self.admissible_tasks = [] #TODO: add admissible_tasks to robot details in map_config file
        # this would be a good way to manage what robots should take on tasks
        # making use of a simple condtion to check if robot can do task X

        """Goal Definitions"""
        self.start_node = None
        self.current_storage = None
        self.base_station = None
        self.wait_node = None

        """Route Details"""
        self.route = []
        self.route_dists = []
        self.route_edges = []
        self.route_fragments = []

        """Notifications"""
        self.no_route_found_notification = True

    """On Shutdown"""
    def _remove(self):
        super(RobotDetails, self)._remove()

    """return goal node as picker location, storage or base station"""
    def _get_goal_node(self):
        if self.task_stage == "go_to_picker":
            return self.goal_node
        elif self.task_stage == "go_to_storage":
            return self.current_storage
        elif self.task_stage == "go_to_base":
            return self.base_station

    """State Changes"""
    def _set_as_idle(self):
        self.idle = True
        self.moving = self.active = False
        self.interruptable = False #not needed

    def _begin_task(self, task_id):

        if self.interruptable:
            self.robot_interface.cancel_execpolicy_goal()
            self._finish_task_stage("")

        self.task_id = task_id
        self.task_stage_list = ["start_task", "go_to_picker", "wait_loading", "go_to_storage",
                                "wait_unloading", "go_to_base", None]
        self.idle = False
        self.active = True
        self.interruptable = False
        #self.moving = False

        self._finish_task_stage(self.task_stage_list.pop(0))

    def _end_task(self):
        self.task_id = None

        if self.registered:
            self.idle = True

        self.active = False
        self.interruptable = False
        #self.moving = True

        self._change_task_stage(None)

    def _delivered_tray(self):
        self.task_id = None
        self.tray_loaded = False

        self.interruptable = True
        self.moving = False
        #self.moving = False
        #self.active = True
        #self.idle = False?

        self.current_storage = None

        self._finish_task_stage("wait_unloading")

    def _set_target_base(self):
        self.idle = False
        self.active = True
        self.interruptable = True
        self._change_task_stage("go_to_base")

    def _finish_route_fragment(self):
        self.moving = False

        self.route = []
        self.route_fragments = []
        self.robot_interface.execpolicy_result = None

    def _finish_task_stage(self, state):
        if self.task_stage in ["go_to_picker", "go_to_storage", "go_to_base"]:
            self._finish_route_fragment()
        self.start_time = Now()
        self._change_task_stage(self.task_stage_list.pop(0))

    def _change_task_stage(self, stage):
        if self.task_stage == stage:
            return
        self.task_stage = stage
        logmsgbreak()
        logmsg(category="robot", id=self.robot_id, msg='@ stage: %s' % (str(self.task_stage).upper()))

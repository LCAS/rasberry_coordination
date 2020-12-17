#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import actionlib
import rospy

from rospy import Subscriber as Sub, get_rostime as Now
from std_msgs.msg import String as Str
from thorvald_base.msg import BatteryArray as Battery
from rasberry_coordination.agent_manager import AgentManager, AgentDetails


class RobotManager(AgentManager):

    """Initialise class with callback details to apply to robots"""
    def __init__(self, callback_dict):
        super(RobotManager, self).__init__(callback_dict)
        self.dump_cb = Sub("rasberry_coordination/robot_manager/dump", Str, self.dump_details)

    """Add Robot Details Objects"""
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = RobotDetails(agent_id, self.cb)

    """"""
    def get_registered_list(self): #TODO: swap out to polymorphism
        return {deets.agent_id:deets.registered for deets in self.agent_details.values() if deets.registered is True}

"""Centralised container for all details pertaining to the robot"""
class RobotDetails(AgentDetails):
    def __init__(self, ID, cb):

        """Initialise Fields in Parent Class"""
        super(RobotDetails, self).__init__(ID, cb)
        
        """Detail whether the robot is moving"""
        # self.idle = False
        # self.interruptable = False
        
        """Meta Management"""
        self.robot_id = ID
        self.disconnect_when_idle = False
        self.registered = True

        """Task Details"""
        # self.robot_state = None
        self.tray_loaded = False
        self.start_time = Now()
        self.task_id = None
        self.max_task_priority = 255
        self.admissible_tasks = [] #TODO: add admissible_tasks to robot details in map_config file
        # this would be a good way to manage what robots should take on tasks
        # making use of a simple condtion to check if robot can do task X
        
        """Goal definitions"""
        self.current_storage = None
        self.base_station = None
        self.wait_node = None
        
        """Route details"""
        self.route = []
        self.route_dists = []
        self.route_edges = []
        self.route_fragments = []

    """On shutdown"""
    def _remove(self):
        super(RobotDetails, self)._remove()

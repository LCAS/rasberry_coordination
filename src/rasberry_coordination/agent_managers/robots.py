#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from abc import ABCMeta, abstractmethod
from rospy import Subscriber, Publisher, Time
from std_msgs.msg import String as Str
from rasberry_coordination.msg import KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.robot import Robot as RobotInterface_Old
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef

from rasberry_coordination.agent_managers.agents import AgentManager, AgentDetails
from rasberry_coordination.agent_managers.interfaces import Robot_Interface

""" Agent Details """
# class RobotManager(AgentManager):
#     """ Initialisation """
#     def add_agent(self, agent):
#         switch = {'courier_robot': Courier_RobotAgent,
#                   'uv_robot': UV_RobotAgent,
#                   'data_collection_robot': DataCollection_RobotAgent}
#         self.agent_details[agent['agent_id']] = CourierRobotAgent(agent, self.cb)
#
#
# """ Agent Management """
# class Courier_RoboticAgent(AgentDetails):
#     self.temp_interface = RobotInterface_Old(self.agent_id)
#     self.tags = {'type':'robot'}
#     self.start_idle_task('init_courier')
#     pass
# class UV_RoboticAgent(AgentDetails): pass
# class DataCollection_RoboticAgent(AgentDetails): pass
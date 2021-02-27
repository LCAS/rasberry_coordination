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
from rasberry_coordination.agent_managers.interfaces import CAR_App, UAR_App, CAR_Device, UAR_Device

# """ Agent Details """
# class StorageManager(AgentManager):
#     def add_agent(self, agent):
#         self.agent_details[agent['agent_id']] = StorageDetails(agent, self.cb)


# """ Agent Management """



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

""" Agent Details """
class StorageManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent):
        self.agent_details[agent['agent_id']] = StorageDetails(agent, self.cb)


""" Agent Management """
class StorageDetails(AgentDetails):
    """
    Because the storage details can have multiple pending agents, we need to select which
    """
    def __init__(self, agent_dict, callbacks):
        super(StorageDetails, self).__init__(agent_dict, callbacks)
        self.idle_task_definition = {'default': TaskDef.idle_storage}
        self.new_task_definition = {'default': TaskDef.transportation_storage}
        interfaces = {"car_app": CAR_App,
                      "uar_app": UAR_App,
                      "car_device": CAR_Device,
                      "uar_device": UAR_Device}
        self.interface = interfaces[self.interface_type](agent_id=self.agent_id,
                                                         responses={'UNLOADED': self.unloaded,
                                                                    'OFFLINE': self.offline,
                                                                    'ONLINE': self.online})
        self.tags = {'type': 'storage'}
        self.request_admittance = []
        self.has_presence = False  # used for routing (swap out for physical?)
        pass

    def unloaded(self):
        self['storage_has_tray'] = True

    def offline(self):
        pass

    def online(self):
        pass


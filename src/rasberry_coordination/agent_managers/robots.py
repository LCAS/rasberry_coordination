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
class CourierManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent):
        self.agent_details[agent['agent_id']] = CourierDetails(agent, self.cb)


""" Agent Management """
class CourierDetails(AgentDetails):
    def __init__(self, agent_dict, callbacks):
        print(agent_dict)
        super(CourierDetails, self).__init__(agent_dict, callbacks)


        #Define interface to agent
        interfaces = {"robot_interface":Robot_Interface}
        self.interface = interfaces[self.interface_type](agent_id=self.agent_id,
                                                          responses={'PAUSE'  :self.pause,
                                                                     'UNPAUSE':self.unpause,
                                                                     'RELEASE':self.release})

        #
        self.tags = {'type':'robot'}
        self.temp_interface = RobotInterface_Old(self.agent_id)

        #Start the dfault idle task
        self.start_idle_task('init_courier')
        pass
    def pause(self):
        self.task_stage_list.insert(0, StageDef.Pause(self))
    def unpause(self):
        self.registration = True
    def release(self):
        self.task_stage_list = []


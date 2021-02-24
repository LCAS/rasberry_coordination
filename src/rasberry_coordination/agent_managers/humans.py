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
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef

from rasberry_coordination.agent_managers.agents import AgentManager, AgentDetails
from rasberry_coordination.agent_managers.interfaces import CAR_App, LAR_App, CAR_Device, LAR_Device

""" Agent Details """
class PickerManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent):
        self.agent_details[agent['agent_id']] = PickerDetails(agent, self.cb)

""" Agent Management """
class PickerDetails(AgentDetails):
    def __init__(self, agent_dict, callbacks):
        super(PickerDetails, self).__init__(agent_dict, callbacks)
        self.idle_task_definition = {'default': TaskDef.idle_picker}
        self.new_task_definition = {'default': TaskDef.transportation_request}
        interfaces = {"car_app":CAR_App,
                      "lar_app":LAR_App,
                      "car_device":CAR_Device,
                      "lar_device":LAR_Device}
        self.interface = interfaces[agent_dict['interface_type']](agent_id=self.agent_id,
                                                             responses={'CALLED':self.called,
                                                                        'LOADED':self.loaded,
                                                                        'INIT'  :self.reset})
        self.tags = {'type':'picker'}
        pass
    def called(self):
        logmsg(category="picker", id=self.agent_id, msg="picker has CALLED")
        self.start_new_task(task='transportation_picker', details={'target_agent': self})
        self['start_time'] = Time.now()
    def loaded(self):
        logmsg(category="picker", id=self.agent_id, msg="picker has LOADED")
        self['picker_has_tray'] = False
    def reset(self):
        logmsg(category="picker", id=self.agent_id, msg="picker has INIT")
        pass


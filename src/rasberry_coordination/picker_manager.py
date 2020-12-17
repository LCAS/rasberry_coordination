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
from rasberry_coordination.agent_manager import *


class PickerManager(AgentManager):

    """Initialise class with callback details to apply to pickers"""
    def __init__(self, callback_dict):
        super(PickerManager, self).__init__(callback_dict)
        self.dump_cb = Sub("rasberry_coordination/picker_manager/dump", Str, self.dump_details)

    """Add Picker Details Objects"""
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = PickerDetails(agent_id, self.cb)


"""Centralised container for all details pertaining to the picker"""
class PickerDetails(AgentDetails):
    def __init__(self, ID, cb):

        """Initialise Fields in Parent Class"""
        super(PickerDetails, self).__init__(ID, cb)

        """Meta Management"""
        self.picker_id = ID
        self.registered = True

        """Task Details"""
        # self.picker_state = None
        # self.start_time = Now()
        # self.task_id = None

        """Picker information"""
        self.time_connected = Now()
        self.virtual = False


    """On shutdown"""
    def _remove(self):
        super(PickerDetails, self)._remove()

# File to hold planned core-tasks

from pprint import pprint
from copy import deepcopy
from std_msgs.msg import String as Str
import strands_executive_msgs.msg
import std_msgs.msg
from std_msgs.msg import Bool
import std_srvs.srv
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy
from rasberry_coordination.coordinator_tools import logmsg
import rasberry_coordination.srv
import rasberry_coordination.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption


class StageDef(object):
    """ Active Navigation """
    class FollowAgent(StageBase):
        def _update(self):
            self.agent['target'] = self.agent['following'].location
        def _query(self):
            success_conditions = [self.agent['end_follow']]
            self.agent.flag(any(success_conditions))

    """ SSI Task Auction """
    class AwaitTaskAuction(StageBase):
        def _query(self):
            success_conditions = [self.agent['auction_to_begin'] == True]
            self.agent.flag(any(success_conditions))
    class TaskAuction(StageBase):
        def _start(self):
            self.agent['coordinator_service_required'] = True
        def _query(self):
            success_conditions = [True]
            self.agent.flag(any(success_conditions))

    """ Check Field Change """
    class CheckFieldUpdate(StageBase):
        def __init__(self, agent, field_name):
            super(StageDef.CheckFieldUpdate, self).__init__(agent)
            self.field_name = self.field_name
        def _start(self):
            self.check_value = self.agent[self.field_name]
        def _query(self):
            success_conditions = [self.check_value != self.agent[self.field_name]]
            self.agent.flag(any(success_conditions))

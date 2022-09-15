"""UV Treatment"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location

from rasberry_coordination.task_management.__init__ import StageDef as SDef

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class TaskDef(object):

    """ Tasks """
    @classmethod
    def uv_treatment_treat_edge(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_edge",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToUVStartNode(agent),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToUVEndNode(agent),
                         StageDef.DisableUVLight(agent)
                     ]))
    @classmethod
    def uv_treatment_treat_row(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_row",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindRowEnds(agent, details['row']),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToUVStartNode(agent),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToUVEndNode(agent),
                         StageDef.DisableUVLight(agent)
                     ]))

    """ Control from SAR """
    @classmethod
    def send_uv_treatment(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='uv_treatment',
                    name="send_uv_treatment",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignPhototherapist(agent, details),
                        StageDef.AwaitCompletion(agent),
                    ]))



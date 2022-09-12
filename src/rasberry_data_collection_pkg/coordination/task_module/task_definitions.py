"""Data Collection"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time
from actionlib import SimpleActionClient as SAC
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef

from rasberry_data_collection.msg import RDCCollectDataGoal, RDCCollectDataAction, RDCCollectDataActionGoal, DataCollectionRow

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class TaskDef(object):

    @classmethod
    def data_collection_scanner_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="base_robot_init",
                     details=details,
                     contacts=contacts,
                     initiator_id=agent.agent_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.SetUnregister(agent),
                         StageDef.WaitForDCActionClient(agent),
                         SDef.SetRegister(agent)
                     ]))


    """ Tasks """
    @classmethod
    def data_collection_scan_edge(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='data_collection',
                     name="data_collection_scan_edge",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToDCStartNode(agent),
                         StageDef.PerformDCAction(agent)
                     ]))
    @classmethod
    def data_collection_scan_row(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='data_collection',
                     name="data_collection_scan_row",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindRowEnds(agent, details['row']),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToDCStartNode(agent),
                         StageDef.PerformDCAction(agent)
                     ]))

    """ Control from SAR """
    @classmethod
    def send_data_collection(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='data_collection',
                    name="send_data_collection",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignScanner(agent, details),
                        StageDef.AwaitCompletion(agent),
                    ]))



"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.robot import Robot, VirtualRobot
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass



@classmethod
def move_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    return(Task(id=task_id,
                module='base',
                name="move_idle",
                details=details,
                contacts=contacts,
                initiator_id=agent.agent_id,
                responder_id="",
                stage_list=[
                    StageDef.StartTask(agent, task_id),
                    StageDef.NavigateToTargetNode(agent),
                    StageDef.Idle(agent)
                ]))


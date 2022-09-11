"""Health Monitoring"""

from copy import deepcopy
from std_msgs.msg import String as Str, Bool, Float32
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from thorvald_base.msg import BatteryArray as Battery, ControllerArray
from polytunnel_navigation_actions.msg import RowTraversalHealth

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


@classmethod
def health_monitoring_robot_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):

    # Low battery is added here as new task once idle
    # Critical battery is forced into next task when identified
    if agent.modules['health_monitoring'].interface.battery_low():
        return TaskDef.charge_at_charging_station(agent=agent, task_id=task_id, details=details, contacts=contacts)

@classmethod
def charge_at_charging_station(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    return(Task(id=task_id,
                module='health_monitoring',
                name="charge_at_charging_station",
                details=details,
                contacts=contacts,
                initiator_id=agent.agent_id,
                responder_id="",
                stage_list=[
                    StageDef.StartChargeTask(agent),
                    StageDef.AssignChargeNode(agent),
                    StageDef.NavigateToChargeNode(agent),
                    StageDef.Charge(agent)
                ]))



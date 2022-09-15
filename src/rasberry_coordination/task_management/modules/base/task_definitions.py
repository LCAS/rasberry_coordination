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

from rasberry_coordination.task_management.__init__ import StageDef

#from rasberry_coordination.task_management.modules.base.stage_definitions import StageDef as StageDefBase
#from rasberry_coordination.task_management.modules.base.task_definitions import TaskDef
#from rasberry_coordination.task_management.modules.base.interface_definitions import InterfaceDef
#from rasberry_coordination.task_management.modules.navigation.stage_definitions import StageDef as StageDefNavigation
#from rasberry_coordination.task_management.modules.navigation.task_definitions import TaskDefNavigation
#from rasberry_coordination.task_management.modules.navigation.interface_definitions import InterfaceDef
#-> move robot navigation to here?
#from rasberry_coordination.task_management.modules.assignment.stage_definitions import StageDef as StageDefAssignments
#from rasberry_coordination.task_management.modules.assignment.task_definitions import TaskDef as TaskDefAssignments
#from rasberry_coordination.task_management.modules.assignment.interface_definitions import InterfaceDef


try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass

class TaskDef(object):

    """ Runtime Method for Init Task Definitions """
    @classmethod
    def base_robot_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="base_robot_init",
                     details=details,
                     contacts=contacts,
                     initiator_id=agent.agent_id,
                     responder_id="",
                     stage_list=[
                         StageDef.StartTask(agent, task_id),
                         StageDef.SetUnregister(agent),
                         StageDef.WaitForMap(agent),
                         StageDef.WaitForLocalisation(agent),
                         StageDef.SetRegister(agent)
                     ]))

    @classmethod
    def base_virtual_robot_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="base_virtual_robot_init",
                     details=details,
                     contacts=contacts,
                     initiator_id=agent.agent_id,
                     responder_id="",
                     stage_list=[
                         StageDef.StartTask(agent, task_id),
                         StageDef.SetUnregister(agent),
                         StageDef.WaitForMap(agent),
                         StageDef.EnableVirtualLocalisation(agent),
                         StageDef.WaitForLocalisation(agent),
                         StageDef.SetRegister(agent)
                     ]))

    @classmethod
    def base_human_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="base_human_init",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.SetUnregister(agent),
                        StageDef.WaitForMap(agent),
                        StageDef.SendInfo(agent),
                        StageDef.SetRegister(agent)
                    ]))
    @classmethod
    def base_localised_human_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="base_human_init",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.SetUnregister(agent),
                        StageDef.WaitForMap(agent),
                        StageDef.WaitForLocalisation(agent),
                        StageDef.SendInfo(agent),
                        StageDef.SetRegister(agent)
                    ]))

    """ Idle Tasks """
    @classmethod
    def base_robot_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        if len(agent.task_buffer) == 0:
            return TaskDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def base_virtual_robot_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def base_human_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.idle(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def base_localised_human_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.idle(agent=agent, task_id=task_id, details=details, contacts=contacts)

    """ Runtime Method for Idle Task Definitions """
    @classmethod
    def idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="idle",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.Idle(agent)
                    ]))
    @classmethod
    def wait_at_base(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="wait_at_base",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        # StageDef.Exit(agent)
                        StageDef.AssignBaseNodeIdle(agent),
                        StageDef.NavigateToBaseNodeIdle(agent),
                        StageDef.Idle(agent)
                    ]))
    @classmethod
    def exit_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="exit_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.SetUnregister(agent),
                        StageDef.NavigateToExitNode(agent),
                        StageDef.Exit(agent)
                    ]))

    """ Dynamic Task Management """
    @classmethod
    def release_task(cls, agent):
        logmsg(category="DTM", msg="    | releasing task %s" % (agent['name']))
        agent.task = None
    @classmethod
    def restart_task(cls, agent):
        logmsg(category="DTM", msg="    | restarting task %s" % (agent['name']))
        agent.add_task(task_name=agent['name'], task_id=agent['id'], index=0, quiet=True)
        agent.task = None



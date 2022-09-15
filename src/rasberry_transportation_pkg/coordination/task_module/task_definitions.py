"""Transportation"""

from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.__init__ import StageDef
from rasberry_coordination.robot import Robot as RobotInterface_Old
from rospy import Time, Duration

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass

class TaskDef(object):

    """ Initialisation """
    @classmethod
    def transportation_field_courier_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        if 'load' not in agent.local_properties: agent.local_properties['load'] = 0


    """ Idle Task Stages for Transportation Agents """
    @classmethod
    def transportation_field_courier_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        LP = agent.local_properties
        MP = agent.module_properties

        #If agent is at max capacity deliver load
        agent.local_properties['load'] = int(agent.local_properties['load'])
        if LP['load'] >= int(MP['max_load']):
            return TaskDef.transportation_deliver_load(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            #return TaskDef.transportation_wait_at_head(agent=agent, task_id=task_id, details=details, contacts=contacts)
            pass

    @classmethod
    def transportation_field_storage_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        #If agents are waiting to visit, begin transportation field_storage
        #Otherwise wait idle
        if len(agent.request_admittance) > 0:
            return TaskDef.transportation_field_storage(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            return TaskDef.idle_field_storage_def(agent=agent, task_id=task_id, details=details, contacts=contacts)


    """ Picker Tasks """
    @classmethod
    def transportation_request_field_courier(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_request_field_courier",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignFieldCourier(agent),
                        StageDef.AwaitFieldCourier(agent),
                        StageDef.LoadFieldCourier(agent),
                    ]))
    """ FieldCourier Tasks """
    @classmethod
    def transportation_retrieve_load(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_retrieve_load",
                    details=details,
                    contacts=contacts,
                    initiator_id=initiator_id,
                    responder_id=agent.agent_id,
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.NavigateToPicker(agent),
                        StageDef.Loading(agent)
                    ]))
    @classmethod
    def transportation_wait_at_head(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="wait_at_head",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignHeadNodeIdle(agent),
                        StageDef.NavigateToHeadNodeIdle(agent),
                        SDef.Idle(agent)
                    ]))

    @classmethod
    def transportation_deliver_load(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_deliver_load",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignFieldStorage(agent),
                        SDef.AssignWaitNode(agent),
                        StageDef.AwaitFieldStorageAccess(agent),
                        StageDef.NavigateToFieldStorage(agent),
                        StageDef.Unloading(agent)
                    ]))

    """ Storage Tasks """
    @classmethod
    def idle_field_storage_def(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="idle_field_storage_def",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.IdleFieldStorage(agent)
                    ]))
    @classmethod
    def transportation_field_storage(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_field_storage",
                    details=details,
                    contacts=contacts,
                    initiator_id="",
                    responder_id=agent.agent_id,
                    stage_list=[
                        StageDef.AcceptFieldCourier(agent),
                        StageDef.AwaitFieldCourier(agent),
                        StageDef.UnloadFieldCourier(agent)
                    ]))


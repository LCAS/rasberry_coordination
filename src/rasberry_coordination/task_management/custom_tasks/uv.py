from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef
from rasberry_coordination.robot import Robot as RobotInterface_Old


class InterfaceDef(object):
    class uv_phototherapist(IDef.AgentInterface):
        def __init__(self, agent, sub='%s/lar/get_states', pub='%s/lar/set_states'):
            responses = {}
            super(InterfaceDef.uv_phototherapist, self).__init__(agent, responses, sub=sub, pub=pub)

    class uv_controller(IDef.AgentInterface):
        def __init__(self, agent, sub='%s/lar/get_states', pub='%s/lar/set_states'):
            responses = {'CALLED': self.called, "RESET": self.reset}
            super(InterfaceDef.uv_controller, self).__init__(agent, responses, sub=sub, pub=pub)

        def called(self):
            logmsg(category="UVTask", id=self.agent.agent_id, msg="Request for phototherapist")
            self.agent.add_task(task_name='uv_request_phototherapist')

        def reset(self):
            logmsg(category="UVTask", id=self.agent.agent_id, msg="Reset-task requested")
            if self.agent['id'] and self.agent.task_name=='uv_request_phototherapist':
                self.agent.set_interrupt('reset', 'uv', self.agent['id'], "Task")


    # class uv_scheduler(IDef.AgentInterface): def __init__(self, agent, sub='', pub=''): pass


class TaskDef(object):
    """ Constructors for UV Tasks """

    """ Initialisation Verification """
    @classmethod
    def uv_phototherapist_init(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return TDef.robot_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def uv_controller_init(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return TDef.human_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)


    """ Idle Tasks """
    @classmethod
    def uv_phototherapist_idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return TDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)


    """ Tasks """
    # @classmethod
    # def uv_request_scan_edge(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
    #     return ({'id = task_id,
    #              'module = 'uv',
    #              'name = "uv_request_scan_edge",
    #              'details = deepcopy(details),
    #              'contacts = contacts.copy(),
    #              'initiator_id = agent.agent_id,
    #              'responder_id = "",
    #              'stage_list = [
    #                  SDef.StartTask(agent, task_id),
    #                  StageDef.AssignPhototherapist(agent),
    #                  StageDef.AwaitPhototherapist(agent),
    #                  StageDef.LoadCourier(agent),
    #              ]})
    @classmethod
    def uv_request_scan_row(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return(Task(id = task_id,
                    module = 'uv',
                    name = "uv_request_scan_row",
                    details = deepcopy(details),
                    contacts = contacts.copy(),
                    initiator_id = agent.agent_id,
                    responder_id = "",
                    stage_list = [
                         SDef.StartTask(agent, task_id),  # Get task id and initialise controller details
                         StageDef.IdentifyRowEnds(agent),  # Find points to send for uv-robot nav targets
                         StageDef.AssignPhototherapist(agent),  # Find a uv-robot closest to one of the row ends
                         StageDef.AwaitPhototherapistStart(agent),  # Wait for the robot to get to the row
                         StageDef.MonitorPhototherapistCompletion(agent)  # Wait for the robot to get to the end of the row
                     ]))
    # @classmethod
    # def uv_request_scan_tunnel(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
    #     return ({'id = task_id,
    #              'name = "uv_request_scan_tunnel",
    #              'details = deepcopy(details),
    #              'contacts = contacts.copy(),
    #              'task_module = 'uv',
    #              'initiator_id = agent.agent_id,
    #              'responder_id = "",
    #              'stage_list = [
    #                  SDef.StartTask(agent, task_id),
    #                  StageDef.AssignPhototherapist(agent),
    #                  StageDef.AwaitPhototherapist(agent),
    #              ]})

    @classmethod
    def uv_scan_region(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return(Task(id = task_id,
                    module='uv',
                    name = "uv_scan_region",
                    details = deepcopy(details),
                    contacts = contacts.copy(),
                    initiator_id = initiator_id,
                    responder_id = agent.agent_id,
                    stage_list = [
                         SDef.StartTask(agent, task_id),
                         StageDef.NavigateToRegionNode(agent, details['start_node?']),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToRegionEnd(agent, details['end_node?']),
                         StageDef.DisableUVLight(agent)
                     ]))

    pass

    # @classmethod
    # def uv_treat_edge(cls, agent, task_id=None, details={}, contacts={}):
    #     task_name = "uv_treat_edge"
    #     task_details = deepcopy(details)
    #     task_contacts = contacts.copy()
    #     task_stage_list = [
    #         SDef.StartTask(agent, task_id),
    #         SDef.NavigateToNode(agent),
    #         StageDef.EnableUV(agent),
    #         SDef.NavigateToNode(agent),
    #         StageDef.DisableUV(agent)
    #     ]
    #     return ({'id = task_id,
    #              'name = task_name,
    #              'details = task_details,
    #              'contacts = task_contacts,
    #              'stage_list = task_stage_list})

    # def uv_treat_row(cls, agent, task_id=None, details={}, contacts={}):
    #     task_name = "uv_treat_row"
    #     task_details = deepcopy(details)
    #     task_contacts = contacts.copy()
    #     task_stage_list = [
    #         SDef.StartTask(agent, task_id),
    #         SDef.NavigateToNode(agent),
    #         StageDef.EnableUV(agent),
    #         StageDef.NavigateRow(agent),
    #         StageDef.DisableUV(agent)
    #     ]
    #     return ({'id = task_id,
    #              'name = task_name,
    #              'details = task_details,
    #              'contacts = task_contacts,
    #              'stage_list = task_stage_list})
    #


class StageDef(object):

    class EnableUVLight(SDef.StageBase):
        def _notify_start(self):
            self.agent.interfaces['uv'].notify("ENABLE_LIGHT")  # Request to activate light
        def _query(self):
            success_conditions = [self.agent.interfaces['uv'].state_of_light]
            self._flag(any(success_conditions))

    class DisableUVLight(SDef.StageBase):
        def _notify_start(self):
            self.agent.interfaces['uv'].notify("ENABLE_LIGHT")  # Request to activate light
        def _query(self):
            success_conditions = [not self.agent.interfaces['uv'].state_of_light]
            self._flag(any(success_conditions))

    class NavigateToRegionNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToPicker, self).__init__(agent,  association='picker')

    class AssignCourier(SDef.AssignAgent):
        def _start(self):
            super(StageDef.AssignCourier, self)._start() #defined as default setup
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['agent_type'] = 'phototherapist'
            self.action['response_location'] = None
        def _notify_end(self):
            self.agent.interfaces['uv'].notify("ACCEPT")
        def _end(self):
            super(StageDef.AssignCourier, self)._end()
            self.agent['contacts']['phototherapist'] = self.action['response_location']
            self.agent['contacts']['phototherapist'].add_task(task_name='uv_scan_region',
                                                                task_id=self.agent['id'],
                                                                details={'start_node': self.agent},
                                                                contacts={'phototherapist': self.agent},
                                                                initiator_id=self.agent.agent_id)
            self.agent.responder_id = self.agent['contacts']['phototherapist'].agent_id
























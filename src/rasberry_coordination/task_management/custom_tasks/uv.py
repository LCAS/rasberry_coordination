from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef
from rasberry_coordination.robot import Robot as RobotInterface_Old


class InterfaceDef(object):

    class uv_phototherapist:
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="Publishing: (%s)" % msg)

        def __init__(self, agent):
            self.agent = agent
            # self.sub = Subscriber('/%s/initiate_task/follow'   % agent.agent_id, Str, self.follow)
            self.edge_sub = Subscriber('/%s/initiate_task/edge'     % agent.agent_id, Str, self.edge)
            # self.sub = Subscriber('/%s/initiate_task/row'      % agent.agent_id, Str, self.row)
            # self.sub = Subscriber('/%s/initiate_task/tunnel'   % agent.agent_id, Str, self.tunnel)
            # self.sub = Subscriber('/%s/initiate_task/schedule' % agent.agent_id, Str, self.schedule)

            self.agent.temp_interface = RobotInterface_Old(self.agent.agent_id)

        def edge(self, msg):
            msg.data = "tall-t1-r3-c1_tall-t1-r3-c0"
            logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat edge")
            self.agent.add_task(task_name='uv_treat_edge', details={"nodes": msg.data.split('_')})


    # class uv_controller(IDef.AgentInterface):
    #     def __init__(self, agent, sub='/lar/get_states', pub='/lar/set_states'):
    #
    #         responses = {}
    #         super(InterfaceDef.uv_controller, self).__init__(agent, responses, sub=sub, pub=pub)
    #
    #     def called(self):
    #         logmsg(category="UVTask", id=self.agent.agent_id, msg="Request for phototherapist")
    #         self.agent.add_task(task_name='uv_request_phototherapist')
    #
    #     def reset(self):
    #         logmsg(category="UVTask", id=self.agent.agent_id, msg="Reset-task requested")
    #         if self.agent['id'] and self.agent.task_name=='uv_request_phototherapist':
    #             self.agent.set_interrupt('reset', 'uv', self.agent['id'], "Task")
    #
    #
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
    @classmethod
    def uv_treat_edge(cls, agent, task_id=None, details={'nodes':[]}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='uv',
                     name="uv_treat_edge",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         StageDef.FindUVStartNode(agent, details['nodes']),
                         StageDef.NavigateToUVStartNode(agent),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToUVEndNode(agent),
                         StageDef.DisableUVLight(agent)
                     ]))


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

    class FindUVStartNode(SDef.AssignNode):
        def __init__(self, agent, nodes):
            self.node_list = nodes
            super(StageDef.FindUVStartNode, self).__init__(agent)
        def _start(self):
            super(StageDef.FindUVStartNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['list'] = self.node_list
            self.action['response_location'] = None
        def _end(self):
            super(StageDef.FindUVStartNode, self)._end()
            self.agent['contacts']['start_node'] = self.action['response_location']

            self.node_list.remove(self.action['response_location'])
            self.agent['contacts']['end_node'] = self.node_list[0]

            logmsg(category="stage", msg="UV Task to treat from %s to %s" % (self.agent['contacts']['start_node'], self.agent['contacts']['end_node']))

    class NavigateToUVStartNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToUVStartNode, self).__init__(agent, association='start_node')
    class NavigateToUVEndNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToUVEndNode, self).__init__(agent, association='end_node')

    class EnableUVLight(SDef.StageBase):
        def _notify_start(self):
            self.agent.modules['uv'].interface.notify("ENABLE_LIGHT")  # Request to activate light
            self.agent.cb['format_agent_marker'](self.agent, style='blue')
            self.agent.modules['uv'].interface.state_of_light = True
        def _query(self):
            success_conditions = [self.agent.modules['uv'].interface.state_of_light]
            self._flag(any(success_conditions))
    class DisableUVLight(SDef.StageBase):
        def _notify_start(self):
            self.agent.modules['uv'].interface.notify("ENABLE_LIGHT")  # Request to activate light
            self.agent.cb['format_agent_marker'](self.agent, style='')
            self.agent.modules['uv'].interface.state_of_light = False
        def _query(self):
            success_conditions = [not self.agent.modules['uv'].interface.state_of_light]
            self._flag(any(success_conditions))























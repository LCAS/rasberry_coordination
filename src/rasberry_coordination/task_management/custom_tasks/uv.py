from std_msgs.msg import String as Str
from rasberry_coordination.coordinator_tools import logmsg
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.task_management.base import StageDef as RootDef, InterfaceDef as CommsDef
from rasberry_coordination.robot import Robot as RobotInterface_Old

class InterfaceDef(object):
    class uv_phototherapist(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/lar/get_states', pub='/lar/set_states'):
            responses = {}
            super(InterfaceDef.uv_phototherapist, self).__init__(agent, responses, sub=sub, pub=pub)

    class uv_controller(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/lar/get_states', pub='/lar/set_states'):
            responses = {}
            super(InterfaceDef.uv_controller, self).__init__(agent, responses, sub=sub, pub=pub)


class TaskDef(object):
    """ Initial Task Stages for Transportation Agents """
    pass
    # @classmethod
    # def idle_phototherapist(cls, agent, task_id=None, details={}, contacts={}):
    #     pass
    #
    # @classmethod
    # def idle_controller(cls, agent, task_id=None, details={}, contacts={}):
    #     pass
    #
    # @classmethod
    # def uv_treat_edge(cls, agent, task_id=None, details={}, contacts={}):
    #     task_name = "uv_treat_edge"
    #     task_details = TDef.load_details(details)
    #     task_contacts = contacts.copy()
    #     task_stage_list = [
    #         SDef.StartTask(agent, task_id),
    #         SDef.NavigateToNode(agent),
    #         StageDef.EnableUV(agent),
    #         SDef.NavigateToNode(agent),
    #         StageDef.DisableUV(agent)
    #     ]
    #     return ({'id': task_id,
    #              'name': task_name,
    #              'details': task_details,
    #              'contacts': task_contacts,
    #              'stage_list': task_stage_list})
    #
    # def uv_treat_row(cls, agent, task_id=None, details={}, contacts={}):
    #     task_name = "uv_treat_row"
    #     task_details = TDef.load_details(details)
    #     task_contacts = contacts.copy()
    #     task_stage_list = [
    #         SDef.StartTask(agent, task_id),
    #         SDef.NavigateToNode(agent),
    #         StageDef.EnableUV(agent),
    #         StageDef.NavigateRow(agent),
    #         StageDef.DisableUV(agent)
    #     ]
    #     return ({'id': task_id,
    #              'name': task_name,
    #              'details': task_details,
    #              'contacts': task_contacts,
    #              'stage_list': task_stage_list})
    #

class StageDef(object):
    pass
    # class NavigateRow(Navigation):
    #     """ define a direct route to the end of the row """
    #     def _start(self):
    #         super(StageDef.NavigateToNode, self)._start()
    #         self.target = self.agent.task_contacts[self.association]
from copy import deepcopy
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef


class InterfaceDef(object):

    class uv_treatment_phototherapist(object):
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="Publishing: (%s)" % msg)

        def __init__(self, agent):
            self.agent = agent

            self.light_status = False

            self.sub_edge     = Subscriber('/%s/uv_treatment/initiate_task/edge'     % agent.agent_id, TopoLocation, self.edge)
            self.sub_row      = Subscriber('/%s/uv_treatment/initiate_task/row'      % agent.agent_id, TopoLocation, self.row)
            self.sub_tunnel   = Subscriber('/%s/uv_treatment/initiate_task/tunnel'   % agent.agent_id, TopoLocation, self.tunnel)
            # self.sub_schedule = Subscriber('/%s/initiate_task/schedule' % agent.agent_id, Str, self.schedule)

        def edge(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                # msg.edge_nodes = [0,1]
                nodeA = "%s-t%s-r%s-c%s"%(msg.type, msg.tunnel, msg.row, msg.edge_node[0])
                nodeB = "%s-t%s-r%s-c%s"%(msg.type, msg.tunnel, msg.row, msg.edge_node[1])

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat edge")
                self.agent.add_task(task_name='uv_treatment_treat_edge', details={"nodes": [nodeA, nodeB]})
        def row(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                row = "%s-t%s-r%s"%(msg.type, msg.tunnel, msg.row)

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='uv_treatment_treat_row', details={"row": row})
        def tunnel(self, msg):
            if self.agent.registration:
                msg.type = "tall"
                msg.tunnel = 1
                tunnel = "%s-t%s"%(msg.type, msg.tunnel)

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='uv_treatment_treat_tunnel', details={"tunnel": tunnel})

        def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
        def __setitem__(self, key, val): self.__setattr__(key, val)


class TaskDef(object):
    """ Constructors for UV Tasks """

    """ Tasks """
    @classmethod
    def uv_treatment_treat_edge(cls, agent, task_id=None, details={'nodes':[]}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_edge",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindStartNode(agent, details['nodes']),
                         StageDef.NavigateToUVStartNode(agent),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToUVEndNode(agent),
                         StageDef.DisableUVLight(agent)
                     ]))
    @classmethod
    def uv_treatment_treat_row(cls, agent, task_id=None, details={'row':''}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_row",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
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
    @classmethod
    def uv_treatment_treat_tunnel(cls, agent, task_id=None, details={'tunnel':''}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_tunnel",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         StageDef.FindRowsUV(agent, details['tunnel'])
                     ]))


class StageDef(object):
    class FindRowsUV(SDef.FindRows):
        def __init__(self, agent, tunnel): super(StageDef.FindRowsUV, self).__init__(agent, tunnel, 'uv_treatment_treat_row')

    class NavigateToUVStartNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToUVStartNode, self).__init__(agent, association='start_node')

    class NavigateToUVEndNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToUVEndNode, self).__init__(agent, association='end_node')

    class EnableUVLight(SDef.NotifyTrigger):
        def __init__(self, agent): super(StageDef.EnableUVLight, self).__init__(agent, trigger='light_status', msg="ENABLE_LIGHT", colour='blue')

    class DisableUVLight(SDef.NotifyTrigger):
        def __init__(self, agent): super(StageDef.DisableUVLight, self).__init__(agent, trigger='light_status', msg="DISABLE_LIGHT", colour='')
















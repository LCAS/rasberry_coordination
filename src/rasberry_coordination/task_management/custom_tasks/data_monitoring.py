from copy import deepcopy
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef


class InterfaceDef(object):

    class data_monitoring_scanner(object):
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="Publishing: (%s)" % msg)

        def __init__(self, agent):
            self.agent = agent

            self.camera_status = False

            self.sub_edge     = Subscriber('/%s/data_monitoring/initiate_task/edge'     % agent.agent_id, TopoLocation, self.edge)
            self.sub_row      = Subscriber('/%s/data_monitoring/initiate_task/row'      % agent.agent_id, TopoLocation, self.row)
            self.sub_tunnel   = Subscriber('/%s/data_monitoring/initiate_task/tunnel'   % agent.agent_id, TopoLocation, self.tunnel)
            # self.sub_schedule = Subscriber('/%s/initiate_task/schedule' % agent.agent_id, Str, self.schedule)

        def edge(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                # msg.edge_nodes = [0,1]
                nodeA = "%s-t%s-r%s-c%s"%(msg.type, msg.tunnel, msg.row, msg.edge_node[0])
                nodeB = "%s-t%s-r%s-c%s"%(msg.type, msg.tunnel, msg.row, msg.edge_node[1])

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat edge")
                self.agent.add_task(task_name='data_monitoring_scan_edge', details={"nodes": [nodeA, nodeB]})
        def row(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                row = "%s-t%s-r%s"%(msg.type, msg.tunnel, msg.row)

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='data_monitoring_scan_row', details={"row": row})
        def tunnel(self, msg):
            if self.agent.registration:
                msg.type = "tall"
                msg.tunnel = 1
                tunnel = "%s-t%s"%(msg.type, msg.tunnel)

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='data_monitoring_scan_tunnel', details={"tunnel": tunnel})

        def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
        def __setitem__(self, key, val): self.__setattr__(key, val)


class TaskDef(object):
    """ Constructors for data_monitoring Tasks """

    # """ Initialisation Verification """
    # @classmethod
    # def data_monitoring_scanner_init(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
    #     return TDef.robot_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)

    # """ Idle Tasks """
    # @classmethod
    # def data_monitoring_scanner_idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
    #     return TDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)

    """ Tasks """
    @classmethod
    def data_monitoring_scan_edge(cls, agent, task_id=None, details={'nodes':[]}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='data_monitoring',
                     name="data_monitoring_scan_edge",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindStartNode(agent, details['nodes']),
                         StageDef.NavigateToDMStartNode(agent),
                         StageDef.EnableDMCamera(agent),
                         StageDef.NavigateToDMEndNode(agent),
                         StageDef.DisableDMCamera(agent)
                     ]))
    @classmethod
    def data_monitoring_scan_row(cls, agent, task_id=None, details={'row':''}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='data_monitoring',
                     name="data_monitoring_scan_row",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindRowEnds(agent, details['row']),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToDMStartNode(agent),
                         StageDef.EnableDMCamera(agent),
                         StageDef.NavigateToDMEndNode(agent),
                         StageDef.DisableDMCamera(agent)
                     ]))
    @classmethod
    def data_monitoring_scan_tunnel(cls, agent, task_id=None, details={'tunnel':''}, contacts={}, initiator_id=""):
        return (Task(id=task_id,
                     module='data_monitoring',
                     name="data_monitoring_scan_tunnel",
                     details=deepcopy(details),
                     contacts=contacts.copy(),
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         StageDef.FindRowsDM(agent, details['tunnel'])
                     ]))


class StageDef(object):
    class FindRowsDM(SDef.FindRows):
        def __init__(self, agent, tunnel): super(StageDef.FindRowsDM, self).__init__(agent, tunnel, 'data_monitoring_scan_row')

    class NavigateToDMStartNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToDMStartNode, self).__init__(agent, association='start_node')

    class NavigateToDMEndNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToDMEndNode, self).__init__(agent, association='end_node')

    class EnableDMCamera(SDef.NotifyTrigger):
        def __init__(self, agent): super(StageDef.EnableDMCamera, self).__init__(agent, trigger='camera_status', msg="ENABLE_CAMERA", colour='green')

    class DisableDMCamera(SDef.NotifyTrigger):
        def __init__(self, agent): super(StageDef.DisableDMCamera, self).__init__(agent, trigger='camera_status', msg="DISABLE_CAMERA", colour='')
























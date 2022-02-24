"""Data Collection"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class scanner(object):
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="Publishing: (%s)" % msg)

        def __init__(self, agent):
            self.agent = agent

            self.camera_status = False

            self.sub_edge     = Subscriber('/%s/data_collection/initiate_task/edge'     % agent.agent_id, TopoLocation, self.edge)
            self.sub_row      = Subscriber('/%s/data_collection/initiate_task/row'      % agent.agent_id, TopoLocation, self.row)
            self.sub_tunnel   = Subscriber('/%s/data_collection/initiate_task/tunnel'   % agent.agent_id, TopoLocation, self.tunnel)
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
                self.agent.add_task(task_name='data_collection_scan_edge', details={"row_ends": [nodeA, nodeB]})
        def row(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                row = "%s-t%s-r%s"%(msg.type, msg.tunnel, msg.row)

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='data_collection_scan_row', details={"row": row})
        def tunnel(self, msg):
            if self.agent.registration:
                msg.type = "tall"
                msg.tunnel = 1
                tunnel = "%s-t%s"%(msg.type, msg.tunnel)

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='data_collection_scan_tunnel', details={"tunnel": tunnel})

        def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
        def __setitem__(self, key, val): self.__setattr__(key, val)

    class controller(IDef.RasberryInterfacing_ProtocolManager):
        def sar_BEGUN(self):
            task_scope, details = self.get_task('data_collection')
            task_name = 'send_data_collection'
            if task_name and details: self.agent.add_task(task_name=task_name, details=details)
        def sar_CANCEL(self):
            if self.agent['name'] == 'send_data_collection':
                logmsg(level="error", category="IDef", id=self.agent.agent_id, msg="has task")
                self.agent.set_interrupt('reset', 'data_collection', self.agent['id'], "Task")
        def sar_EMERGENCY_STOP(self):
            if self.agent['name'] == 'send_data_collection':
                self.agent.set_interrupt('pause', 'data_collection', self.agent['id'], "Task")
                if 'phototherapist' in self.agent['contacts'] and 'Pause' not in self.agent['contacts']['scanner']().get_class():
                    self.agent['contacts']['scanner'].set_interrupt('pause', 'data_collection', self.agent['id'], "Task")
        def sar_EMERGENCY_RESUME(self):
            if self.agent['name'] == 'send_data_collection':
                self.agent.set_interrupt('resume', 'data_collection', self.agent['id'], "Task")
                if 'phototherapist' in self.agent['contacts'] and 'Pause' in self.agent['contacts']['scanner']().get_class():
                    self.agent['contacts']['scanner'].set_interrupt('resume', 'data_collection', self.agent['id'], "Task")


class TaskDef(object):
    """ Constructors for data_collection Tasks """

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
                         StageDef.NavigateToDMStartNode(agent),
                         StageDef.EnableDMCamera(agent),
                         StageDef.NavigateToDMEndNode(agent),
                         StageDef.DisableDMCamera(agent)
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
                         StageDef.NavigateToDMStartNode(agent),
                         StageDef.EnableDMCamera(agent),
                         StageDef.NavigateToDMEndNode(agent),
                         StageDef.DisableDMCamera(agent)
                     ]))
    @classmethod
    def data_collection_scan_tunnel(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='data_collection',
                     name="data_collection_scan_tunnel",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         StageDef.FindRowsDM(agent, details['tunnel'])
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


class StageDef(object):

    class FindRowsDM(SDef.FindRows):
        """Used to assign the data_collection_scan_row task to all rows in the given tunnel."""
        def __init__(self, agent, tunnel):
            """Call super to set data_collection_scan_row as task to apply"""
            super(StageDef.FindRowsDM, self).__init__(agent, tunnel, 'data_collection_scan_row')

    class NavigateToDMStartNode(SDef.NavigateToNode):
        """Used to navigate to a given start node"""
        def __init__(self, agent):
            """Call to super to set the navigation target as the node stored in the action association"""
            super(StageDef.NavigateToDMStartNode, self).__init__(agent, association='start_node')
        def _start(self):
            if 'controller' in self.agent['contacts']:
                self.agent['contacts']['controller'].modules['data_collection'].interface.notify("sar_AWAIT_START")
            super(StageDef.NavigateToDMStartNode, self)._start()

    class NavigateToDMEndNode(SDef.NavigateToNode):
        """Used to navigate to a given end node"""
        def __init__(self, agent):
            """Call to super to set the navigation target as the node stored in the action association"""
            super(StageDef.NavigateToDMEndNode, self).__init__(agent, association='end_node')

    class EnableDMCamera(SDef.NotifyTrigger):
        """Used to enable the camera on the robot"""
        def __init__(self, agent):
            """Call to initialise a camera_status message of ENABLE_CAMERA to send on start and set rviz robot to green"""
            super(StageDef.EnableDMCamera, self).__init__(agent, trigger='camera_status', msg="ENABLE_CAMERA", colour='green')
        def _end(self):
            if 'controller' in self.agent['contacts']:
                self.agent['contacts']['controller'].modules['data_collection'].interface.notify("sar_AWAIT_TASK_COMPLETION")

    class DisableDMCamera(SDef.NotifyTrigger):
        """Used to disable the camera on the robot"""
        def __init__(self, agent):
            """Call to initialise a camera_status message of DISABLE_CAMERA to send on start and set rviz robot to clear"""
            super(StageDef.DisableDMCamera, self).__init__(agent, trigger='camera_status', msg="DISABLE_CAMERA", colour='')
        def _end(self):
            if 'controller' in self.agent['contacts']:
                if len([s for s in self.agent['stage_list'][:-1] if 'DisableDMCamera' in s.get_class()]) == 0:
                    # if there is no more stages in stagslit, set flag on controller?
                    self.agent['contacts']['controller']['scanner_completion_flag'] = True

    class AssignScanner(SDef.AssignAgent):
        def __init__(self, agent, details):
            self.details = details
            print(details)
            self.response_task = 'data_collection_scan_'+details['scope']
            self.contacts = {'controller': agent}
            if details['scope']== "edge":
                self.contacts['row_ends'] = details['nodes']
            if details['robot'] != "closest":
                self.action['list'] = [details['robot']]
            super(StageDef.AssignScanner, self).__init__(agent, action_style='closest', agent_type='scanner')
        def _end(self):
            super(StageDef.AssignScanner, self)._end()
            self.agent.modules['data_collection'].interface.notify("sar_AWAIT_START")
            self.agent['contacts']['scanner'].add_task(task_name=self.response_task,
                                                       task_id=self.agent['id'],
                                                       details=self.details,
                                                       contacts=self.contacts,
                                                       initiator_id=self.agent.agent_id)

    class AwaitCompletion(SDef.Idle):
        def _start(self):
            super(StageDef.AwaitCompletion,self)._start()
            self.agent['scanner_completion_flag'] = False
        def _query(self):
            success_conditions = [self.agent['scanner_completion_flag']]
            self.flag(any(success_conditions))
        def _end(self):
            self.agent.modules['data_collection'].interface.notify("sar_COMPLETE")















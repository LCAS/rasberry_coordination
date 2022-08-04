"""Data Collection"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time
from actionlib import SimpleActionClient as SAC
from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.actions.action_manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef

from rasberry_data_collection.msg import RDCCollectDataGoal, RDCCollectDataAction, RDCCollectDataActionGoal, DataCollectionRow

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
            # self.sub_schedule = Subscriber('/%s/initiate_task/schedule' % agent.agent_id, Str, self.schedule)

            self.topo_map = fetch_property('data_collection', 'topological_map')
            self.continuous = fetch_property('data_collection', 'continuous')

            self.action_status = False
            self.action_publisher = SAC('/%s/data_collection/data_collection_server/collect_data' % agent.agent_id, RDCCollectDataAction)

        def edge(self, msg):
            if self.agent.registration:
                # msg.row = 3
                # msg.edge_nodes = [0,1]
                nodeA = "r%s-c%s"%(msg.row, msg.edge_node[0])
                nodeB = "r%s-c%s"%(msg.row, msg.edge_node[1])

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat edge")
                self.agent.add_task(task_name='data_collection_scan_edge', details={"row_ends": [nodeA, nodeB]})
        def row(self, msg):
            if self.agent.registration:
                # msg.row = 3
                row = "r%s"%(msg.row)

                logmsg(category="DMTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='data_collection_scan_row', details={"row": row})

        def publish_action(self, origin, target):
            """
            topological_map: 'tmap_stream.tmap2'/'tmap_70cm.tmap'
            continuous: True
            rows:
            - origin: 'r3-c0'
              end: 'r3-c2'
              orientation: 'front'/'back'/''
              data_config: '{"force_orientation_to_origin":true,"capture_data":true}'
            """
            collection_goal = RDCCollectDataGoal()
            collection_goal.topological_map = "" #self.topo_map
            collection_goal.continuous = self.continuous

            #forward
            row = DataCollectionRow()
            row.origin = origin
            row.end = target
            row.orientation = 'front'
            row.data_config = str({"force_orientation_to_origin": True, "capture_data": True})
            collection_goal.rows.append(row)

            #backward
            row = DataCollectionRow()
            row.origin = target
            row.end = origin
            row.orientation = 'front'
            row.data_config = str({"force_orientation_to_origin": True, "capture_data": True})
            collection_goal.rows.append(row)

            self.action_status = False
            self.action_publisher.send_goal(collection_goal, done_cb=self.action_done_cb)

        def action_done_cb(self, msg, msg2):
            #TODO: make this more robust to failed action server response
            self.action_status = True

        def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
        def __setitem__(self, key, val): self.__setattr__(key, val)

    class controller(IDef.RasberryInterfacing_ProtocolManager):
        def sar_BEGUN(self):
            task_scope, details = self.get_task('data_collection')
            task_name = 'send_data_collection'
            if task_name and details: self.agent.add_task(task_name=task_name, details=details)
        def sar_CANCEL(self):
            if self.agent['name'] == 'send_data_collection':
                logmsg(level="error", category="IDef", id=self.agent.agent_id, msg="already has task")
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

    @classmethod
    def data_collection_scanner_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="base_robot_init",
                     details=details,
                     contacts=contacts,
                     initiator_id=agent.agent_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.SetUnregister(agent),
                         StageDef.WaitForDCActionClient(agent),
                         SDef.SetRegister(agent)
                     ]))


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
                         StageDef.NavigateToDCStartNode(agent),
                         StageDef.PerformDCAction(agent)
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
                         StageDef.NavigateToDCStartNode(agent),
                         StageDef.PerformDCAction(agent)
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

    class WaitForDCActionClient(SDef.StageBase):
        """ f """
        def _query(self):
            """Complete when the agents location is identical to the target location."""
            success_conditions = [True] #[???self.client.wait_for_server()]
            self.flag(any(success_conditions))



    class NavigateToDCStartNode(SDef.NavigateToNode):
        """Used to navigate to a given start node"""
        def __init__(self, agent):
            """Call to super to set the navigation target as the node stored in the action association"""
            super(StageDef.NavigateToDCStartNode, self).__init__(agent, association='start_node')
        def _start(self):
            self.agent.speaker('Navigating to data collection start node at %s' % self.agent['contacts']['start_node'])
            if 'controller' in self.agent['contacts']:
                self.agent['contacts']['controller'].modules['data_collection'].interface.notify("sar_AWAIT_START")
            super(StageDef.NavigateToDCStartNode, self)._start()

    class PerformDCAction(SDef.StageBase):
        """ f """
        def __init__(self, agent):
            """ f """
            super(StageDef.PerformDCAction, self).__init__(agent)
            self.interface = self.agent.modules['data_collection'].interface
        def _start(self):
            """format and publish msg to send to action server"""
            super(StageDef.PerformDCAction, self)._start()
            self.origin = self.agent['contacts']['start_node']
            self.end = self.agent['contacts']['end_node']
            self.agent.speaker('Performing data collection between %s and %s' % (self.origin, self.end))
            self.interface.publish_action(origin=self.origin, target=self.end)
        def _query(self):
            """ f """
            success_conditions = [self.interface.action_status == True]
            self.flag(any(success_conditions))

    class AssignScanner(SDef.ActionResponse):
        """Used to identify the closest scanner."""
        def __init__(self, agent, details):
            """ Mark the details of the associated Action """
            self.details = details
            self.response_task = 'data_collection_scan_'+details['scope']
            self.contacts = {'controller': agent}
            if details['scope'] == "edge": self.contacts['row_ends'] = details['nodes']
            if details['robot'] != "closest": self.action['list'] = [details['robot']]
            super(StageDef.AssignScanner, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='agent_descriptor', descriptor='scanner', style='closest_agent')
            self.contact = 'scanner'
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















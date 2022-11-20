from rospy import Subscriber
from actionlib import SimpleActionClient as SAC
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_data_collection.msg import RDCCollectDataGoal, RDCCollectDataAction, RDCCollectDataActionGoal, DataCollectionRow

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class ScannerVirtual(Interface):

    def __init__(self, agent, details=None):
        self.agent = agent

        self.camera_status = False

        self.sub_edge     = Subscriber('/%s/data_collection/initiate_task'     % agent.agent_id, TopoLocation, self.on_demand_task)
        # self.sub_schedule = Subscriber('/%s/schedule_task' % agent.agent_id, Str, self.schedule) #TODO

        self.topo_map = fetch_property('data_collection', 'topological_map')
        self.continuous = fetch_property('data_collection', 'continuous') == "True"

        # For getting up-status from data collection action server
        self.action_server_status = False
        self.action_server_status_sub = Subscriber('/%s/data_collection/data_collection_server/collect_data/status' % agent.agent_id, GoalStatusArray, self.server_status_cb)

        # For publishing data collection request to action server
        self.action_status = False
        self.action_publisher = SAC('/%s/data_collection/data_collection_server/collect_data' % agent.agent_id, RDCCollectDataAction)

    """
    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return self.wait_at_base()

    def wait_at_base(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_data_collection_pkg',
                    name="wait_at_base",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['assignment']['AssignBaseNodeIdle'](self.agent),
                        Stages['navigation']['NavigateToBaseNodeIdle'](self.agent),
                        Stages['base']['Idle'](self.agent)
                    ]))

    """

    def on_demand_task(self, msg):
        if self.agent.registration:
            if msg.edge_nodes != ['']:
                # msg.row = 3
                # msg.edge_nodes = [0,1]
                nodeA = "r%s-c%s"%(msg.row, msg.edge_nodes[0])
                nodeB = "r%s-c%s"%(msg.row, msg.edge_nodes[1])
                logmsg(category="DCTASK", id=self.agent.agent_id, msg="Request to treat edge")
                self.agent.add_task(module='rasberry_data_collection_pkg', name='scan_edge', contacts={"row_ends": [nodeA, nodeB]})
            else:
                # msg.row = 3
                row = "r%s"%(msg.row)
                logmsg(category="DCTASK", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(module='rasberry_data_collection_pkg', name='scan_row', details={"row": row})

    def server_status_cb(self, msg):
        self.action_server_status = True

    def DCR(origin, end, orientation='front', data_config=None):
        """
        topological_map: 'tmap_stream.tmap2'/'tmap_70cm.tmap'
        continuous: True
        rows:
        - origin: 'r3-c0'
          end: 'r3-c2'
          orientation: 'front' or 'back' or ''
          data_config: '{"force_orientation_to_origin":true,"capture_data":true}'
        """
        row = DataCollectionRow()
        row.origin = origin
        row.end = end
        row.orientation = orientation
        row.data_config = data_config or str({"force_orientation_to_origin": True, "capture_data": True})
        return row

    def publish_action(self, origin, target):
        collection_goal = RDCCollectDataGoal()
        collection_goal.topological_map = "" #self.topo_map
        collection_goal.continuous = self.continuous

        collection_goal.rows.append(DCR(origin, target))
        collection_goal.rows.append(DCR(target, origin))

        #collection_goal.rows.append(DCR(origin, target, 'back'))
        #collection_goal.rows.append(DCR(target, origin, 'back')) #TODO

        self.action_status = False
        self.action_publisher.send_goal(collection_goal, done_cb=self.action_done_cb)

    def action_done_cb(self, msg, msg2):
        #TODO: make this more robust to failed action server response
        self.agent().action_status = True
        print(msg)
        print(msg2)

#    def init(self, task_id=None, details=None, contacts=None, initiator_id=""):
#        return (Task(id=task_id,
#                     module='rasberry_data_collection_pkg',
#                     name="init",
#                     details=details,
#                     contacts=contacts,
#                     initiator_id=self.agent.agent_id,
#                     responder_id="",
#                     stage_list=[
#                         Stages['base']['StartTask'](self.agent, task_id),
#                         Stages['base']['SetUnregister'](self.agent),
#                         Stages['rasberry_data_collection_pkg']['WaitForDCActionClient'](self.agent),
#                         Stages['base']['SetRegister'](self.agent)
#                     ]))

    def scan_edge(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='rasberry_data_collection_pkg',
                     name="scan_edge",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         Stages['base']['StartTask'](self.agent, task_id),
                         Stages['assignment']['FindStartNode'](self.agent),
                         Stages['rasberry_data_collection_pkg']['NavigateToDCStartNode'](self.agent),
                         Stages['rasberry_data_collection_pkg']['NavigateToDCEndNode'](self.agent),
                         Stages['rasberry_data_collection_pkg']['NavigateToDCStartNode'](self.agent)
                     ]))

    def scan_row(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='rasberry_data_collection_pkg',
                     name="scan_row",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         Stages['base']['StartTask'](self.agent, task_id),
                         Stages['assignment']['FindRowEnds'](self.agent, details['row']),
                         Stages['assignment']['FindStartNode'](self.agent),
                         Stages['rasberry_data_collection_pkg']['NavigateToDCStartNode'](self.agent),
                         Stages['rasberry_data_collection_pkg']['NavigateToDCEndNode'](self.agent),
                         Stages['rasberry_data_collection_pkg']['NavigateToDCStartNode'](self.agent)
                     ]))

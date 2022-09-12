"""Data Collection"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time
from actionlib import SimpleActionClient as SAC
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.action_management.manager import ActionDetails
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
            self.continuous = fetch_property('data_collection', 'continuous') == "True"

            self.action_server_status = False
            self.action_server_status_sub = Subscriber('/%s/data_collection/data_collection_server/collect_data/status' % agent.agent_id, GoalStatusArray, self.server_status_cb)

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

        def server_status_cb(self, msg):
            self.action_server_status = True

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



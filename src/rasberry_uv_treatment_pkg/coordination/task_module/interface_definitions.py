"""UV Treatment"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class phototherapist(object):
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="Publishing: (%s)" % msg)

        def __init__(self, agent):
            self.agent = agent

            self.light_status = False

            self.sub_edge     = Subscriber('/%s/uv_treatment/initiate_task/edge'     % agent.agent_id, TopoLocation, self.edge)
            self.sub_row      = Subscriber('/%s/uv_treatment/initiate_task/row'      % agent.agent_id, TopoLocation, self.row)
            # self.sub_schedule = Subscriber('/%s/initiate_task/schedule' % agent.agent_id, Str, self.schedule)

        def edge(self, msg):
            if self.agent.registration:
                # msg.row = 3
                # msg.edge_nodes = [0,1]
                nodeA = "r%s-c%s"%(msg.row, msg.edge_node[0])
                nodeB = "r%s-c%s"%(msg.row, msg.edge_node[1])

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat edge")
                self.agent.add_task(task_name='uv_treatment_treat_edge', contacts={'row_ends': [nodeA, nodeB]})
        def row(self, msg):
            if self.agent.registration:
                # msg.row = 3
                row = "r%s"%(msg.row)

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='uv_treatment_treat_row', details={"row": row})

        def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
        def __setitem__(self, key, val): self.__setattr__(key, val)


    class controller(IDef.RasberryInterfacing_ProtocolManager):
        def sar_BEGUN(self):
            task_scope, details = self.get_task('uv_treatment')
            task_name = 'send_uv_treatment'
            if task_name and details: self.agent.add_task(task_name=task_name, details=details)
        def sar_CANCEL(self):
            if self.agent['name'] == 'send_uv_treatment':
                logmsg(level="error", category="IDef", id=self.agent.agent_id, msg="already has task")
                self.agent.set_interrupt('reset', 'uv_treatment', self.agent['id'], "Task")
        def sar_EMERGENCY_STOP(self):
            if self.agent['name'] == 'send_uv_treatment':
                self.agent.set_interrupt('pause', 'uv_treatment', self.agent['id'], "Task")
                if 'phototherapist' in self.agent['contacts'] and 'Pause' not in self.agent['contacts']['phototherapist']().get_class():
                    self.agent['contacts']['phototherapist'].set_interrupt('pause', 'uv_treatment', self.agent['id'], "Task")
        def sar_EMERGENCY_RESUME(self):
            if self.agent['name'] == 'send_uv_treatment':
                self.agent.set_interrupt('resume', 'uv_treatment', self.agent['id'], "Task")
                if 'phototherapist' in self.agent['contacts'] and 'Pause' in self.agent['contacts']['phototherapist']().get_class():
                    self.agent['contacts']['phototherapist'].set_interrupt('resume', 'uv_treatment', self.agent['id'], "Task")



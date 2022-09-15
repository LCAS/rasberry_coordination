"""Transportation"""

from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.robot import Robot as RobotInterface_Old
from rospy import Time, Duration

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass

class InterfaceDef(object):


    class picker(IDef.RasberryInterfacing_ProtocolManager):
        def car_CALLED(self):
            self.agent.add_task(task_name='rasberry_transportation_request_field_courier')
            self.agent['start_time'] = Time.now()

        def car_LOADED(self):
            self.agent['has_tray'] = False

        def car_CANCEL(self):
            if self.agent['id'] and self.agent['name']=='rasberry_transportation_request_field_courier':
                self.agent.set_interrupt('reset', 'rasberry_transportation', self.agent['id'], "Task")

    class field_courier(IDef.AgentInterface):
        def __init__(self, agent, sub='/r/get_states', pub='/r/set_states'):
            #E.g. If a cancellation request is triggered by picker, we much release
            #E.g. If a cancellation request is triggered by field_storage, we much restart
            self.release_triggers = ['self', 'picker', 'toc']
            self.restart_triggers = ['field_storage']

            responses={'PAUSE':self.pause, 'UNPAUSE':self.unpause, 'RELEASE':self.release}
            super(InterfaceDef.field_courier, self).__init__(agent, responses, sub=sub, pub=pub)

        def pause(self): self.agent.set_interrupt('pause', 'rasberry_transportation', self.agent['id'])
        def unpause(self): self.agent.set_interrupt('unpause', 'rasberry_transportation', self.agent['id'])
        def release(self): self.agent.set_interrupt('reset', 'rasberry_transportation', self.agent['id'])

    class field_storage(IDef.AgentInterface):
        def __init__(self, agent, sub='/uar/get_states', pub='/uar/set_states'):
            self.release_triggers = ['self', 'toc']
            self.restart_triggers = ['thorvald']

            responses={'UNLOADED': self.unloaded}
            from pprint import pprint
            pprint(dir(IDef))
            print("\n")
            pprint(dir(InterfaceDef))
            super(InterfaceDef.rasberry_transportation_field_storage, self).__init__(agent, responses, sub=sub, pub=pub)

            #These need a new home
            self.agent.request_admittance = []
            self.agent.has_presence = False  # used for routing (swap key for physical?)

        def unloaded(self): self.agent['has_tray'] = True



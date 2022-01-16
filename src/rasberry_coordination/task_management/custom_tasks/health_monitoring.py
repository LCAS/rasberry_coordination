"""Health Monitoring"""

from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from thorvald_base.msg import BatteryArray as Battery

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class health_monitoring_robot(IDef.AgentInterface):
        def __init__(self, agent):
            self.agent = agent
            self.battery_data_sub = Subscriber("/%s/dummy_battery_data" % (self.agent.agent_id), Battery, self._battery_data_cb)  # TODO: point this to the correct location

        """ Battery Monitoring """
        def _battery_data_cb(self, msg):
            total_voltage = sum(battery.battery_voltage for battery in msg.battery_data)
            self.agent.local_properties['battery_level'] = total_voltage

            # Add charging task if battery is critical, and agent isnt charging
            if self.battery_critical() and not self.is_charging():
                self.agent.task_buffer = [t for t in self.agent.task_buffer if t.task_name != "charge_at_charging_station"]  # Filter charging task from buffer
                self.agent.add_task(task_name="charge_at_charging_station", index=0)

            # Add charging task if battery is low, agent isnt planning to charge, and stage is idle
            if self.battery_low() and not self.has_charging_task() and self.is_idle():
                self.agent.add_task(task_name="charge_at_charging_station")

        def is_idle(self): return self.agent().get_class() == "base.Idle"
        def is_charging(self): return self.agent['task_name'] is "charge_at_charging_station"
        def has_charging_task(self): return "charge_at_charging_station" in [t.task_name for t in self.agent.task_buffer]+[self.agent['task_name']]
        def battery_critical(self):
            LP = self.agent.local_properties
            CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
            if 'battery_level' in LP and LP['battery_level'] < CRIT: return True
        def battery_low(self):
            LP = self.agent.local_properties
            MIN = fetch_property('health_monitoring', 'min_battery_limit')
            CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
            if 'battery_level' in LP and CRIT < LP['battery_level'] <= MIN: return True


class TaskDef(object):

    @classmethod
    def health_monitoring_robot_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):

        # Low battery is added here as new task once idle
        # Critical battery is forced into next task when identified
        if agent.modules['health_monitoring'].interface.battery_low():
            return TaskDef.charge_at_charging_station(agent=agent, task_id=task_id, details=details, contacts=contacts)

    @classmethod
    def charge_at_charging_station(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='health_monitoring',
                    name="charge_at_charging_station",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartChargeTask(agent),
                        StageDef.AssignChargeNode(agent),
                        StageDef.NavigateToChargeNode(agent),
                        StageDef.Charge(agent)
                    ]))


class StageDef(object):

    class StartChargeTask(SDef.StartTask):
        """Used to Initiate task with the agent set to unregistered"""
        def _start(self):
            """Set registration to false when charging is begun"""
            super(StageDef.StartChargeTask, self)._start()
            self.agent.registration = False

    class AssignChargeNode(SDef.AssignNode):
        """Used to Identify and reserve a charging station"""
        def _start(self):
            """Initiate action to find charging station"""
            super(StageDef.AssignChargeNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['descriptor'] = 'charging_station'
            self.action['response_location'] = None
        def _end(self):
            """Save reserved charging station to contacts"""
            self.agent['contacts']['charging_station'] = self.action['response_location']
            # self.agent.responder_id = self.agent['contacts']['charging_station'] #TODO: if we want this, add another field to TOC (m.location)

    class NavigateToChargeNode(SDef.NavigateToNode):
        """Used to navigate to the assigned charging station"""
        def __init__(self, agent):
            """Identify associated contact as 'charging_station'"""
            super(StageDef.NavigateToChargeNode, self).__init__(agent, association='charging_station')
        def _query(self):
            """Complete navigation if agents location is the target of if battery level is set to be above threshold"""
            LVL = self.agent.local_properties['battery_level']
            MAX = fetch_property('health_monitoring', 'max_battery_limit')
            success_conditions = [self.agent.location(accurate=True) == self.target,
                                  LVL >= MAX]
            self.flag(any(success_conditions))

    class Charge(SDef.StageBase):
        """Used to Pause task progression till battery level is usable"""
        def __repr__(self):
            """Return battery level with class name"""
            return "%s(%s%%)"%(self.get_class(), str(100*self.agent.local_properties['battery_level']).split('.')[0])
        def _query(self):
            """Complete once battery level is safe"""
            LVL = self.agent.local_properties['battery_level']
            MAX = fetch_property('health_monitoring', 'max_battery_limit')
            success_conditions = [LVL >= MAX];
            self.flag(any(success_conditions))
        def _end(self):
            """Enable registration once task is ended"""
            self.agent.registration = True

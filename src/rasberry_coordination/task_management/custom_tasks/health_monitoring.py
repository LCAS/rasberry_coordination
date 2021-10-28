from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef

from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef
from thorvald_base.msg import BatteryArray as Battery


class InterfaceDef(object):

    class health_monitoring_robot(IDef.AgentInterface):
        def __init__(self, agent):
            self.agent = agent
            self.battery_data_sub = Subscriber("/%s/dummy_battery_data" % (self.agent.agent_id), Battery, self._battery_data_cb)  # TODO: point this to the correct location

        """ Battery Monitoring """
        def _battery_data_cb(self, msg):
            total_voltage = sum(battery.battery_voltage for battery in msg.battery_data)
            self.agent.properties['battery_level'] = total_voltage
            if self.battery_critical() and self.agent['task_name'] is not "charge_at_charging_station":  # Only add charging task if battery level critical and active task is not charging
                self.agent.task_buffer = [t for t in self.agent.task_buffer if t.task_name != "charge_at_charging_station"]  # Remove any low-battery tasks in buffer
                self.agent.add_task(task_name="charge_at_charging_station", index=0)  # Add critical battery task to the buffer head

        def battery_critical(self):
            AP = self.agent.properties
            if 'battery_level' in AP and AP['battery_level'] < AP['critical_battery_limit']: return True
        def battery_low(self):
            AP = self.agent.properties
            if 'battery_level' in AP and AP['critical_battery_limit'] < AP['battery_level'] <= AP['min_battery_limit']: return True


class TaskDef(object):

    @classmethod
    def health_monitoring_robot_init(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        agent.properties['critical_battery_limit'] = PDef['health_monitoring']['critical_battery_limit']
        agent.properties['min_battery_limit'] = PDef['health_monitoring']['min_battery_limit']
        agent.properties['max_battery_limit'] = PDef['health_monitoring']['max_battery_limit']

    @classmethod
    def health_monitoring_robot_idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        AP = agent.properties

        # Low battery is added here as new task once idle
        # Critical battery is forced into next task when identified
        if agent.interfaces['health_monitoring'].battery_low():
            return TaskDef.charge_at_charging_station(agent=agent, task_id=task_id, details=details, contacts=contacts)

    @classmethod
    def charge_at_charging_station(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return(Task(id = task_id,
                    module='health_monitoring',
                    name = "charge_at_charging_station",
                    details = deepcopy(details),
                    contacts = contacts.copy(),
                    initiator_id = agent.agent_id,
                    responder_id = "",
                    stage_list = [
                        StageDef.StartChargeTask(agent),
                        StageDef.AssignChargeNode(agent),
                        StageDef.NavigateToChargeNode(agent),
                        StageDef.Charge(agent)
                    ]))


class StageDef(object):

    class StartChargeTask(SDef.StartTask):
        def _start(self):
            super(StageDef.StartChargeTask, self)._start()
            self.agent.registration = False

    class AssignChargeNode(SDef.AssignNode):
        def _start(self):
            super(StageDef.AssignChargeNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['descriptor'] = 'charging_station'
            self.action['response_location'] = None
        def _end(self):
            self.agent['contacts']['charging_station'] = self.action['response_location']
            # self.agent.responder_id = self.agent['contacts']['charging_station'] #TODO: is we want this, add another field to TOC (m.location)

    class NavigateToChargeNode(SDef.NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToChargeNode, self).__init__(agent, association='charging_station')
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target
                                  ,self.agent.properties['battery_level'] >= self.agent.properties['max_battery_limit']]
            self._flag(any(success_conditions))

    class Charge(SDef.StageBase):
        def __repr__(self):
            return "%s(%s%%)"%(self.get_class(), str(100*self.agent.properties['battery_level']).split('.')[0])
        def _query(self):
            success_conditions = [self.agent.properties['battery_level'] >= self.agent.properties['max_battery_limit']];
            self._flag(any(success_conditions))
        def _end(self):
            self.agent.registration = True

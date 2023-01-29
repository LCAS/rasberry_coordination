"""Health Monitoring"""

from copy import deepcopy
from std_msgs.msg import String as Str, Bool, Float32
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.interaction_management.manager import InteractionDetails

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.task_management.__init__ import Stages

from thorvald_base.msg import BatteryArray as Battery, ControllerArray
from polytunnel_navigation_actions.msg import RowTraversalHealth

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass

from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase, StartTask
from rasberry_coordination.task_management.modules.navigation.stage_definitions import NavigateToNode
from rasberry_coordination.task_management.modules.assignment.stage_definitions import InteractionResponse


class StartChargeTask(StartTask):
    """Used to Initiate task with the agent set to unregistered"""
    def _start(self):
        """Set registration to false when charging is begun"""
        super(StartChargeTask, self)._start()
        self.agent.registration = False
        """ Notify of intent to charge """
        LP = self.agent.local_properties
        CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        PERCENTAGE = ((LP['battery_level']-CRIT)/(MAX-CRIT))*100
        self.agent.speaker("My battery level is at %s%%. Charging proceedure initiated."%(str(PERCENTAGE).split('.')[0]))


class NavigateToChargeNode(NavigateToNode):
    def _query(self):
        super(NavigateToChargeNode, self)._query(agent)
        LVL = self.agent.local_properties['battery_level']
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        self.flag(LVL >= MAX)


class Charge(StageBase):
    """Used to Pause task progression till battery level is usable"""
    def __repr__(self):
        """Return battery level with class name"""
        LP = self.agent.local_properties
        CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        PERCENTAGE = ((LP['battery_level']-CRIT)/(MAX-CRIT))*100
        return "%s(%s%%)"%(self.get_class(), str(PERCENTAGE).split('.')[0])
    def _start(self):
        super(Charge, self)._start()
        LP = self.agent.local_properties
        CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        PERCENTAGE = ((LP['battery_level']-CRIT)/(MAX-CRIT))*100
        self.agent.speaker("My battery level is at %s%%, please put me on charge."%str(PERCENTAGE).split('.')[0])
    def _query(self):
        """Complete once battery level is safe"""
        LVL = self.agent.local_properties['battery_level']
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        success_conditions = [LVL >= MAX];
        self.flag(any(success_conditions))
    def _end(self):
        """Enable registration once task is ended"""
        self.agent.registration = True


class AssignRobot(InteractionResponse):
    def __init__(self, agent, details, task_name):
        self.details = details
        self.task_name = task_name
        super(AssignRobot, self).__init__(agent)
        print(self.details)
        grouping = 'agent_list'
        via=self.details['msg'].criteria.viable_agents
        self.interaction=InteractionDetails(type='search',
                                            grouping='agent_list',
                                            list=via,
                                            style='named_agent')
        self.contact = 'robot'

    def _end(self):
        super(AssignRobot, self)._end()
        cont=self.agent['contacts']['robot']
        cont.add_task(module="rasberry_health_monitoring_pkg",
                      name=self.task_name,
                      task_id=self.agent['id'],
                      details=self.details,
                      contacts=None,
                      initiator_id=self.agent.agent_id)


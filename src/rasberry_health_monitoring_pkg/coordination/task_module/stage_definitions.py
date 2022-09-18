"""Health Monitoring"""

from copy import deepcopy
from std_msgs.msg import String as Str, Bool, Float32
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.action_management.manager import ActionDetails

from rasberry_coordination.encapsuators import LocationObj as Location
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.task_management.__init__ import Stages

from thorvald_base.msg import BatteryArray as Battery, ControllerArray
from polytunnel_navigation_actions.msg import RowTraversalHealth

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass



from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase, StartTask
from rasberry_coordination.task_management.modules.navigation.stage_definitions import NavigateToNode
from rasberry_coordination.task_management.modules.assignment.stage_definitions import ActionResponse


class StartChargeTask(StartTask):
    """Used to Initiate task with the agent set to unregistered"""
    def _start(self):
        """Set registration to false when charging is begun"""
        super(StartChargeTask, self)._start()
        self.agent.registration = False

class AssignChargeNode(ActionResponse):
    """Used to identify the closest available charging_station."""
    def __init__(self, agent):
        """ Mark the details of the associated Action """
        super(AssignChargeNode, self).__init__(agent)
        self.action = ActionDetails(type='search', grouping='node_descriptor', descriptor='charging_station', style='closest_node')
        self.contact = 'charging_station'

class NavigateToChargeNode(NavigateToNode):
    """Used to navigate to the assigned charging station"""
    def __init__(self, agent):
        """Identify associated contact as 'charging_station'"""
        super(NavigateToChargeNode, self).__init__(agent, association='charging_station')
    def _start(self):
        super(NavigateToChargeNode, self)._start()
        LP = self.agent.local_properties
        CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        PERCENTAGE = ((LP['battery_level']-CRIT)/(MAX-CRIT))*100
        self.agent.speaker("My battery level is at %s%%. I am going to charge at %s"%(str(PERCENTAGE).split('.')[0], self.target))
    def _query(self):
        """Complete navigation if agents location is the target of if battery level is set to be above threshold"""
        LVL = self.agent.local_properties['battery_level']
        MAX = fetch_property('health_monitoring', 'max_battery_limit')
        success_conditions = [self.agent.location(accurate=True) == self.target,
                              LVL >= MAX]
        self.flag(any(success_conditions))

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

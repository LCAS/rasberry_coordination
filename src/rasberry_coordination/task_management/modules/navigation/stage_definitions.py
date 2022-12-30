"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg

from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.interaction_management.manager import InteractionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import LocationObj as Location, MapObj as Map
from rasberry_coordination.task_management.containers.Module import ModuleObj as Module
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.robot import Robot, DebugRobot

from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase, Idle

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


""" Navigation Controllers """
class Navigation(StageBase):
    """Base task for all Navigation"""
    def __repr__(self):
        """Display class with idle navigation target """
        # if self.target:
        #     return "%s(%s)"%(self.get_class(), self.target.replace('WayPoint','WP'))
        return "%s" % (self.get_class())
    def __init__(self, agent, contact_id, target=None):
        """Identify the location from which the target is identified"""
        super(Navigation, self).__init__(agent)
        self.association = contact_id
        self.target = target
    def _start(self):
        """Flag the agent as requirieng a route"""
        super(Navigation, self)._start()
        self.route_required = True
        logmsg(category="stage", id=self.agent.agent_id, msg="Navigation from %s to %s is begun." % (self.agent.location(accurate=True), self.target))
    def _query(self):
        """Complete when the agents location is identical to the target location."""
        success_conditions = [self.agent.location(accurate=True) == self.target] #TODO: maybe we should query execpolicy success?
        self.flag(any(success_conditions))
    def _end(self):
        """End navigation by refreshing routes for other agents in motion."""
        logmsg(category="stage", id=self.agent.agent_id, msg="Navigation from %s to %s is completed." % (self.agent.location(accurate=True), self.target))
        # agent.modules['navigation'].interface.cancel_execpolicy_goal() # <- this will prevent robot from rotating to align
        self.agent.cb['trigger_replan']()  # ReplanTrigger


class NavigateToAgent(Navigation):
    def _start(self):
        self.target = self.agent['contacts'][self.association].location(accurate=True)
        self.target_agent = self.agent['contacts'][self.association]
        super(NavigateToAgent, self)._start()


class NavigateToNode(Navigation):
    def _start(self):
        self.target = self.target if self.target else self.agent['contacts'][self.association]
        super(NavigateToNode, self)._start()


class NavigateToNodeIdle(NavigateToNode, Idle):
    """ Used to process both stages together """
    pass

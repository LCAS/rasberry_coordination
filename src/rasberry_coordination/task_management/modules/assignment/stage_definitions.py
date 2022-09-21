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
from rasberry_coordination.robot import Robot, VirtualRobot
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase


try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InteractionResponse(StageBase):
    def __init__(self, agent):
        """Enable action"""
        super(InteractionResponse, self).__init__(agent)
        self.interaction_required = True
        self.contact = None
    def _query(self):
        """Complete once action has generated a result"""
        success_conditions = [self.interaction.response != None]
        self.flag(any(success_conditions))
    def _end(self, contact_type='responder_id'):
        """Save interaction response to contacts"""
        super(InteractionResponse, self)._end()
        resp = self.interaction.response
        if self.contact:
            if '_agent' in str(self.interaction.style): #TODO: TypeError: argument of type 'NoneType' is not iterable
                self.agent[contact_type] = self.interaction.response.agent_id
            self.agent['contacts'][self.contact] = self.interaction.response


class SendInfo(InteractionResponse):
    """Used to identify the closest available wait_node."""
    def __init__(self, agent):
        """ Mark the details of the associated Interaction """
        super(SendInfo, self).__init__(agent)
        self.interaction = InteractionDetails(type='info', info='send_info')



""" Node identification Stages"""
class AssignBaseNode(InteractionResponse):
    """Used to identify the closest available base_node."""
    def __init__(self, agent):
        """ Mark the details of the associated Interaction """
        super(AssignBaseNode, self).__init__(agent)
        self.interaction = InteractionDetails(type='search', grouping='node_descriptor', descriptor=self.agent.navigation_properties['wait_node_name'], style='closest_node')
        self.contact = 'base_node'
class AssignBaseNodeIdle(AssignBaseNode):
    """Used to identify the closest available base_node."""
    def _start(self):
        super(AssignBaseNode, self)._start()
        self.accepting_new_tasks = True
    def _query(self):
        """Complete once interaction has generated a result"""
        success_conditions = [self.interaction.response != None,
                              len(self.agent.task_buffer) > 0]
        self.flag(any(success_conditions))

class AssignWaitNode(InteractionResponse):
    """Used to identify the closest available wait_node."""
    def __init__(self, agent):
        """ Mark the details of the associated Interaction """
        super(AssignWaitNode, self).__init__(agent)
        self.interaction = InteractionDetails(type='search', grouping='node_descriptor', descriptor='wait_node', style='closest_node')
        self.contact = 'wait_node'

""" Identification of Navigation Targets """
class FindRowEnds(InteractionResponse):
    """Used to identify the two ends of a given row."""
    def __repr__(self):
        """Display the row id to generate tasks for."""
        return "%s(%s)" % (self.get_class(), self.interaction.descriptor)
    def __init__(self, agent, row):
        """Save the row id of interest"""
        super(FindRowEnds, self).__init__(agent)
        self.interaction = InteractionDetails(type='info', info='find_row_ends', descriptor=row)
        self.contact = 'row_ends'
class FindStartNode(InteractionResponse):
    """Used to identify of two nodes, which one is closest ot the agent."""
    def __repr__(self):
        """Display row ends in the repr"""
        if 'row_ends' in self.agent['contacts'] and self.agent['contacts']['row_ends']:
            return "%s(%s)" % (self.get_class(), self.agent['contacts']['row_ends'])
        return self.get_class()
    def _start(self):
        """Define interaction to find which node of the ends is the closest to start from"""
        super(FindStartNode, self)._start()
        lst = self.agent['contacts']['row_ends']
        self.interaction = InteractionDetails(type='search', grouping='node_list', list=lst, style='closest_node')
    def _end(self):
        """Save the closest and furthest node to be the row start and row end"""
        super(FindStartNode, self)._end()
        self.agent['contacts']['start_node'] = self.interaction.response
        self.agent['contacts']['row_ends'].remove(self.interaction.response)
        self.agent['contacts']['end_node'] = self.agent['contacts']['row_ends'][0]
        logmsg(category="stage", msg="Task to move from %s to %s" % (self.agent['contacts']['start_node'], self.agent['contacts']['end_node']))


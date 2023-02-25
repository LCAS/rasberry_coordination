"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.interaction_management.manager import InteractionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.containers.Module import ModuleObj as Module
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase, Idle


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
        logmsg(category="stage", msg="   | Interaction response found: %s" % (self.interaction.response))
        if self.contact:
            if '_agent' in str(self.interaction.style):
                #TODO: TypeError: argument of type 'NoneType' is not iterable
                self.agent[contact_type] = self.interaction.response.agent_id
            self.agent['contacts'][self.contact] = self.interaction.response

class SendInfo(InteractionResponse):
    def __init__(self, agent):
        """ Mark the details of the associated Interaction """
        super(SendInfo, self).__init__(agent)
        self.interaction = InteractionDetails(type='info', info='send_info')



""" Node identification Stages"""
class AssignNode(InteractionResponse):
    def __repr__(self, **kw):
        return "%s(%s)" % (self.get_class(), self.interaction.descriptor)
    def __init__(self, agent, contact_id, node_descriptor, style='closest_node', **kw):
        super(AssignNode, self).__init__(agent)
        self.interaction = InteractionDetails(type='search', grouping='node_descriptor', descriptor=node_descriptor, style=style)
        self.contact = contact_id
        self.association = contact_id


class AssignNodeIdle(AssignNode, Idle):
    """ Used to process both stages together """
    pass







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


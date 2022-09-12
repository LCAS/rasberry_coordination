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

class StageDef(object):

    class WaitForDCActionClient(SDef.StageBase):
        """ Prevent progression till action server is up and running """
        def _query(self):
            """Complete when the agents location is identical to the target location."""
            success_conditions = [self.agent.modules['data_collection'].interface.action_server_status]
            self.flag(any(success_conditions))



    class NavigateToDCStartNode(SDef.NavigateToNode):
        """Used to navigate to a given start node"""
        def __init__(self, agent):
            """Call to super to set the navigation target as the node stored in the action association"""
            super(StageDef.NavigateToDCStartNode, self).__init__(agent, association='start_node')
        def _start(self):
            self.agent.speaker('Navigating to data collection start node at %s' % self.agent['contacts']['start_node'])
            if 'controller' in self.agent['contacts']:
                self.agent['contacts']['controller'].modules['data_collection'].interface.notify("sar_AWAIT_START")
            super(StageDef.NavigateToDCStartNode, self)._start()

    class PerformDCAction(SDef.StageBase):
        """ f """
        def __init__(self, agent):
            """ f """
            super(StageDef.PerformDCAction, self).__init__(agent)
            self.interface = self.agent.modules['data_collection'].interface
        def _start(self):
            """format and publish msg to send to action server"""
            super(StageDef.PerformDCAction, self)._start()
            self.origin = self.agent['contacts']['start_node']
            self.end = self.agent['contacts']['end_node']
            self.agent.speaker('Performing data collection between %s and %s' % (self.origin, self.end))
            self.interface.publish_action(origin=self.origin, target=self.end)
        def _query(self):
            """ f """
            success_conditions = [self.interface.action_status == True]
            self.flag(any(success_conditions))

    class AssignScanner(SDef.ActionResponse):
        """Used to identify the closest scanner."""
        def __init__(self, agent, details):
            """ Mark the details of the associated Action """
            self.details = details
            self.response_task = 'data_collection_scan_'+details['scope']
            self.contacts = {'controller': agent}
            if details['scope'] == "edge": self.contacts['row_ends'] = details['nodes']
            if details['robot'] != "closest": self.action['list'] = [details['robot']]
            super(StageDef.AssignScanner, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='agent_descriptor', descriptor='scanner', style='closest_agent')
            self.contact = 'scanner'
        def _end(self):
            super(StageDef.AssignScanner, self)._end()
            self.agent.modules['data_collection'].interface.notify("sar_AWAIT_START")
            self.agent['contacts']['scanner'].add_task(task_name=self.response_task,
                                                       task_id=self.agent['id'],
                                                       details=self.details,
                                                       contacts=self.contacts,
                                                       initiator_id=self.agent.agent_id)

    class AwaitCompletion(SDef.Idle):
        def _start(self):
            super(StageDef.AwaitCompletion,self)._start()
            self.agent['scanner_completion_flag'] = False
        def _query(self):
            success_conditions = [self.agent['scanner_completion_flag']]
            self.flag(any(success_conditions))
        def _end(self):
            self.agent.modules['data_collection'].interface.notify("sar_COMPLETE")















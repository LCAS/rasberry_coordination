from rasberry_coordination.interaction_management.manager import InteractionDetails

from rasberry_coordination.task_management.modules.base.stage_definitions import StageBase, Idle
from rasberry_coordination.task_management.modules.navigation.stage_definitions import NavigateToNode
from rasberry_coordination.task_management.modules.assignment.stage_definitions import InteractionResponse

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass




"""
Standard navigation methods for beginning a data collection

"""

class NavigateToDCStartNode(NavigateToNode):
    """Used to navigate to a given start node"""
    def __init__(self, agent):
        """Call to super to set the navigation target as the node stored in the action association"""
        super(NavigateToDCStartNode, self).__init__(agent, association='start_node')
    def _start(self):
        self.agent.speaker('Navigating to data collection start node at %s' % self.agent['contacts']['start_node'])
        if 'controller' in self.agent['contacts']:
            self.agent['contacts']['controller'].modules['rasberry_data_collection_pkg'].interface.notify("sar_AWAIT_START")
        super(NavigateToDCStartNode, self)._start()

""" For virtual agents only (until we get a better interface setup """
class NavigateToDCEndNode(NavigateToNode):
    """Used to navigate to a given end node"""
    def __init__(self, agent):
        """Call to super to set the navigation target as the node stored in the action association"""
        super(NavigateToDCEndNode, self).__init__(agent, association='end_node')





"""
Standard interactions with a communicable action client

"""

class WaitForDCActionClient(StageBase):
    """ Prevent progression till action server is up and running """
    def _query(self):
        """ f """
        success_conditions = [self.agent.modules['rasberry_data_collection_pkg'].interface.action_server_status]
        self.flag(any(success_conditions))

class PerformDCAction(StageBase):
    """ f """
    def _start(self):
        """format and publish msg to send to action server"""
        super(PerformDCAction, self)._start()
        self.action_status = False
        origin = self.agent['contacts']['start_node']
        target = self.agent['contacts']['end_node']
        self.agent.speaker('Performing data collection between %s and %s' % (origin, end))
        interface = self.agent.modules['rasberry_data_collection_pkg'].interface
        interface.publish_action(origin=origin, target=target)
    def _query(self):
        """ f """
        success_conditions = [self.action_status == True]
        self.flag(any(success_conditions))
    def _end(self):
       """ f """
       super(PerformDCAction, self)._end()
       if 'controller' in self.agent['contacts']:
           self.agent['contacts']['controller']['scanner_completion_flag'] = True





"""
Standard control tools for a controller

"""

class AssignScanner(InteractionResponse):
    """Used to identify the closest scanner."""
    def __init__(self, agent, details):
        """ Mark the details of the associated Action """
        self.details = details
        self.response_task = 'scan_'+details['scope']
        self.contacts = {'controller': agent}
        if details['scope'] == "edge": self.contacts['row_ends'] = details['nodes']
        if details['robot'] != "closest": self.action['list'] = [details['robot']]
        super(AssignScanner, self).__init__(agent)
        self.interaction = InteractionDetails(type='search', grouping='agent_descriptor', descriptor='scanner', style='closest_agent')
        self.contact = 'scanner'
    def _end(self):
        super(AssignScanner, self)._end()
        self.agent.modules['rasberry_data_collection_pkg'].interface.notify("sar_AWAIT_START")
        self.agent['contacts']['scanner'].add_task(module="rasberry_data_collection_pkg",
                                                   name=self.response_task,
                                                   task_id=self.agent['id'],
                                                   details=self.details,
                                                   contacts=self.contacts,
                                                   initiator_id=self.agent.agent_id)


class AwaitCompletion(Idle):
    def _start(self):
        super(AwaitCompletion, self)._start()
        self.agent['scanner_completion_flag'] = False
    def _query(self):
        success_conditions = [self.agent['scanner_completion_flag']]
        self.flag(any(success_conditions))
    def _end(self):
        self.agent.modules['rasberry_data_collection_pkg'].interface.notify("sar_COMPLETE")



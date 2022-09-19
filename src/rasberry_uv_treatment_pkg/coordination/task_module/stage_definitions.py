"""UV Treatment"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.interaction_management.manager import InteractionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class NavigateToUVStartNode(NavigateToNode):
    """Used to navigate to a given start node"""
    def __init__(self, agent):
        """Call to super to set the navigation target as the node stored in the action association"""
        super(NavigateToUVStartNode, self).__init__(agent, association='start_node')
    def _start(self):
        if 'controller' in self.agent['contacts']:
            self.agent['contacts']['controller'].modules['uv_treatment'].interface.notify("sar_AWAIT_START")
        super(NavigateToUVStartNode, self)._start()

class NavigateToUVEndNode(NavigateToNode):
    """Used to navigate to a given end node"""
    def __init__(self, agent):
        """Call to super to set the navigation target as the node stored in the action association"""
        super(NavigateToUVEndNode, self).__init__(agent, association='end_node')

class EnableUVLight(NotifyTrigger):
    """Used to enable the UV light on the robot"""
    def __init__(self, agent):
        """Call to initialise a light_status message of ENABLE_LIGHT to send on start and set rviz robot to blue"""
        super(EnableUVLight, self).__init__(agent, trigger='light_status', msg="ENABLE_LIGHT", colour='blue')
    def _end(self):
        if 'controller' in self.agent['contacts']:
            self.agent['contacts']['controller'].modules['uv_treatment'].interface.notify("sar_AWAIT_TASK_COMPLETION")

class DisableUVLight(NotifyTrigger):
    """Used to disable the UV light on the robot"""
    def __init__(self, agent):
        """Call to initialise a light_status message of DISABLE_LIGHT to send on start and set rviz robot to clear"""
        super(DisableUVLight, self).__init__(agent, trigger='light_status', msg="DISABLE_LIGHT", colour='')
    def _end(self):
        if 'controller' in self.agent['contacts']:
            if len([s for s in self.agent['stage_list'][:-1] if 'DisableUVLight' in s.get_class()]) == 0:
                # if there is no more stages in stagslit, set flag on controller?
                self.agent['contacts']['controller']['phototherapist_completion_flag'] = True

class AssignPhototherapist(ActionResponse):
    """Used to identify the closest phototherapist."""
    def __init__(self, agent, details):
        """ Mark the details of the associated Action """
        self.details = details
        self.response_task = 'uv_treatment_treat_'+details['scope']
        self.contacts = {'controller': agent}
        if details['scope'] == "edge": self.contacts['row_ends'] = details['nodes']
        if details['robot'] != "closest": self.interaction['list'] = [details['robot']]
        super(AssignPhototherapist, self).__init__(agent)
        self.interaction = InteractionDetails(type='search', grouping='agent_descriptor', descriptor='phototherapist', style='closest_agent')
        self.contact = 'phototherapist'
    def _end(self):
        super(AssignPhototherapist, self)._end()
        self.agent.modules['uv_treatment'].interface.notify("sar_AWAIT_START")
        self.agent['contacts']['phototherapist'].add_task(task_name=self.response_task,
                                                          task_id=self.agent['id'],
                                                          details=self.details,
                                                          contacts=self.contacts,
                                                          initiator_id=self.agent.agent_id)



class AwaitCompletion(Idle):
    def _start(self):
        super(AwaitCompletion,self)._start()
        self.agent['phototherapist_completion_flag'] = False
    def _query(self):
        success_conditions = [self.agent['phototherapist_completion_flag']]
        self.flag(any(success_conditions))
    def _end(self):
        self.agent.modules['uv_treatment'].interface.notify("sar_COMPLETE")






"""

assignAgent needs ability to assign by name

publish configs to sar

"""

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

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class StageBase(object):
    """Base class for all Stages"""
    def get_class(self):
        """Cleaned class name for explicit class-type queries"""
        return str(self.__class__)\
            .replace("<class '","")\
            .replace("rasberry_coordination.", "")\
            .replace("task_management.","")\
            .replace("modules.","")\
            .replace("coordination.","")\
            .replace("task_module.","")\
            .replace("stage_definitions.","")\
            .replace("'>","")
    def __repr__(self):
        """Simplified representation of class for clean informative logging"""
        return self.get_class()
    def suspend(self):
        """Collection function for all actions to call when a stage is suspended"""
        self.new_stage = True
    def flag(self, completion):
        """Collection function when a stage is to be flagged as completed"""
        # TODO: this is not very good here, the state of an agent being interrupted should be handled abstractly
        if not self.agent.interruption:
            self.stage_complete = completion
    def __init__(self, agent):
        """Class initialisation for populating default values"""
        self.agent = agent
        self.interaction_required = False
        self.route_required = False  #True: trigger a route search
        self.route_found = False  #True: trigger a route publish
        self.stage_complete = False
        self.new_stage = True
        self.target = None
        self.target_agent = None
        self.interaction = None
        self.accepting_new_tasks = False
        self.summaries = {}
    def _start(self):
        """Stage start, called when this is the active stage."""
        logmsg(category="stage", id=self.agent.agent_id, msg="Begun stage %s" % self)
        self.start_time = Time.now()
        self.accepting_new_tasks = False
    def _query(self):
        """Used to define the criteria which ust be met for the stage to be completed"""
        success_conditions = []  # What should be queried to tell if the stage is completed?
        self.flag(any(success_conditions))
    def _end(self):
        """Used to set any fields as the stage is about to be removed"""
        self.stage_complete = False
        pass
class StartTask(StageBase):
    """Called as the first stage of every task, to generate and apply a task_id."""
    def __init__(self, agent, task_id=None):
        """Generate a unique Task ID of {agent_id}_{total_tasks++}."""
        super(StartTask, self).__init__(agent)
        self.task_id = task_id if task_id else "%s_%s" % (self.agent.simple_agent_id(), self.agent.total_tasks)
        self.agent.total_tasks += 1
    def _start(self):
        """Being at the head of the task, once the task is begun the task_id is adopted as the active task_id."""
        self.agent.task_details = {}
        super(StartTask, self)._start()
        self.agent['id'] = self.task_id #Set task_id as active_task_id for agent
        self.agent['start_time'] = Time.now()
        self.agent.format_marker(style='')
    def _query(self):
        """Complete the stage without any condition"""
        self.flag(True)


""" Setup """
class SetUnregister(StageBase):
    """Called to mark the agent as unregistered"""
    def _start(self):
        """Mark agent as unregistered and send a message to rviz to display the agent as red"""
        super(SetUnregister, self)._start()
        self.agent.registration = False
        self.agent.format_marker(style='red')
    def _query(self):
        """Complete the stage without any condition"""
        self.flag(True)
class WaitForMap(StageBase):
    """Called to suspend task progression till a restricted map is recieved"""
    def _start(self):
        """Begin a subscriber to recieve the tmap"""
        super(WaitForMap, self)._start()
        self.agent.map_handler.enable_map_monitoring()
    def _query(self):
        """Complete stage once a tmap is available"""
        success_conditions = [self.agent.map_handler.filtered_node_list]
        self.flag(any(success_conditions))
class EnableVirtualLocalisation(StageBase):
    """Enable localisation for virtual agents"""
    def _start(self):
        """Enable subscribers to virtual localisation"""
        super(EnableVirtualLocalisation, self)._start()
        self.agent.navigation_interface.enable_subscribers()
    def _query(self):
        """Complete the stage without any condition"""
        self.flag(True)
class WaitForLocalisation(StageBase):
    """Called to suspend task progression till a location for the agent is recieved"""
    def _start(self):
        """Enable location monitoring"""
        super(WaitForLocalisation, self)._start()
        self.agent.location.enable_location_monitoring(self.agent.agent_id)
    def _query(self):
        """Complete once location has been identified"""
        success_conditions = [self.agent.location() is not None]
        self.flag(any(success_conditions))
class SetRegister(StageBase):
    """Called to mark the agent as registered"""
    def _start(self):
        """Mark agent as unregistered and send a message to rviz to display the agent without modified colour"""
        super(SetRegister, self)._start()
        self.agent.registration = True
        self.agent.format_marker(style='')
    def _query(self):
        """Complete the stage without any condition"""
        self.flag(True)
    def _end(self):
        super(SetRegister, self)._end()
        self.agent.send_car_msg("REGISTERED")



""" Idle """
class Idle(StageBase):
    """Used to suspend activity until the agent has a task."""
    def __repr__(self):
        """Display class with idle location """
        if self.agent: return "%s(%s)" % (self.get_class(), self.agent.location())
        return self.get_class()
    def _start(self):
        super(Idle, self)._start()
        self.accepting_new_tasks = True
    def _query(self):
        """Complete once task buffer contains another task, or if battery level is low."""
        success_conditions = [len(self.agent.task_buffer) > 0]
        self.flag(any(success_conditions))



""" Communications """
class NotifyTrigger(StageBase):
    """Used to send a message to trigger some response"""
    def __init__(self, agent, trigger, msg, colour):
        """Save initialisation details for message"""
        super(NotifyTrigger, self).__init__(agent)
        self.trigger, self.msg, self.colour = trigger, msg, colour
    def _start(self):
        """Send trigger message"""
        super(NotifyTrigger, self)._start()
        self.interface = self.agent.modules[self.agent['module']].interface
        self.interface.notify(self.msg)
        self.agent.format_marker(style=self.colour)
        self.interface[self.trigger] = True  #PSUEDO
    def _query(self):
        """Wait for flag to be set by message response (or #PSUEDO)"""
        success_conditions = [self.interface[self.trigger]]
        self.flag(any(success_conditions))



""" Meta Stages """
class Pause(StageBase):
    """Inserted on Pause for use as a 3-stage controlled blocker, used on Coordinator, Task, or Agent scopes"""
    def __repr__(self):
        """Display scopes actively contributing to the blocking"""
        return "%s(C%s|T%s|A%s)" % (self.get_class(), int(self.pause_state['c']), int(self.pause_state['t']), int(self.pause_state['a']))
    def __init__(self, agent):
        """Initialise blocking properties"""
        super(Pause, self).__init__(agent)
        logmsg(category="DTM", msg="      | pause init")
        self.agent.registration = False
        self.pause_state = {'c':False, 't':False, 'a':False}
    def _start(self):
        """On start, cancel any active navigation"""
        super(Pause, self)._start()
        if hasattr(self.agent, 'navigation_interface'): self.agent.navigation_interface.cancel_execpolicy_goal()
        #TODO: set an agent function for generic definition of pausing?
    def _query(self):
        """Continue once all blocking stages are False"""
        # success_conditions = [self.agent.registration]
        success_conditions = [not (True in self.pause_state.values())]
        self.flag(any(success_conditions))
    def _end(self):
        """On end, reenable registration"""
        self.agent.registration = True
        self.agent.format_marker(style='')

class Exit(StageBase):
    """Used for controlled removal of agent connections"""
    def _start(self):
        """Unregister agent and cancel any accociated tasks"""
        super(Exit, self)._start()
        self.agent.registration = False
        for task in self.agent.task_buffer:
            self.agent.set_interrupt('reset', task['module'], task['id'], "Task")
    def _query(self):
        """Continue once all tasks are emoved from the buffer"""
        success_conditions = [len(self.agent.task_buffer) == 0]
        self.flag(any(success_conditions))
    def _end(self):
        """Set marker to black and initiate disconnection interruption-"""
        super(Exit, self)._end()
        self.agent.format_marker('black')
        self.agent.set_interrupt('disconnect', 'base', self.agent['id'], "Task")

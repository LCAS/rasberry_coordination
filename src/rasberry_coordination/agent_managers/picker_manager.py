#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import actionlib
import rospy

from rospy import Subscriber as Sub, get_rostime as Now
from std_msgs.msg import String as Str
from std_srvs.srv import Trigger, TriggerResponse
from rasberry_coordination.agent_managers.agent_manager import AgentManager, AgentDetails
from geometry_msgs.msg import PoseStamped
from rasberry_coordination.msg import TasksDetails, TaskUpdates
from rasberry_coordination.srv import SetPickerNode, SetPickerNodeResponse
from rasberry_coordination.srv import ResetPicker, ResetPickerResponse
from rasberry_coordination.coordinator_tools import logmsg

class PickerManager(AgentManager):

    """Initialise class with callback details to apply to pickers"""
    def __init__(self, callback_dict):
        """ Initialise management interface for pickers and services for generic communications

        :param callback_dict: container for direct callbacks to coordinator for use in subscribers
        """
        super(PickerManager, self).__init__(callback_dict)
        self.latest_task_id = 0
        self.dump_cb = Sub("rasberry_coordination/picker_manager/dump", Str, self.dump_details)

        ns = "rasberry_coordination/picker_manager/"

        # Define interaction services with CAR interface
        self.car_event_sub = rospy.Subscriber("/car_client/get_states", Str, self.car_event_cb)

        self.move_picker_srv = rospy.Service(ns+"set_picker_node",
                                             SetPickerNode,
                                             self.set_picker_node_cb)

        self.reset_picker_srv = rospy.Service(ns+"reset_picker_node",
                                              ResetPicker,
                                              self.reset_picker_node_cb)

        self.reset_picker_srv = rospy.Service(ns+"reset_all_picker_nodes",
                                              Trigger,
                                              self.reset_all_picker_nodes_cb)
        #   Define
        # self.task_updates_sub = rospy.Subscriber("rasberry_coordination/task_updates", TaskUpdates, self.task_updates_cb)
        # self.active_tasks_pub = rospy.Publisher("rasberry_coordination/active_tasks_details", TasksDetails, latch=True, queue_size=5)

    """ Set picker position to a custom node (from coordinator perspective) """
    def set_picker_node_cb(self, req):
        """ set_picker_node service callback
        """
        resp = SetPickerNodeResponse()
        resp.success = False
        if req.picker_id in self.agent_details:
            self.agent_details[req.picker_id]._set_picker_node_cb(req.node)
            resp.success = True
        else:
            resp.message = "Given picker_id is not listed"
        return resp

    """ Reset picker position back to default subscribers """
    def reset_picker_node_cb(self, req):
        """ reset_picker_node service callback
        """
        resp = ResetPickerResponse()
        resp.success = False
        if req.picker_id in self.agent_details:
            self.agent_details[req.picker_id]._reset_picker_node_cb()
            resp.success = True
        else:
            resp.message = "Given picker_id is not listed"
        return resp

    """ Reset all picker nodes in one go """
    def reset_all_picker_nodes_cb(self, req):
        """ reset_all_picker_nodes service callback
        """
        resp = TriggerResponse()
        resp.success = True
        for picker_id in self.agent_details:
            self.agent_details[picker_id]._reset_picker_node_cb()
        return resp

    """Add Picker Details Objects"""
    def add_agent(self, agent_id):
        """ Initialise a given agent and add to the agent_details collection

        :param agent_id: identifier for the agent to initialise
        :return: None
        """
        self.agent_details[agent_id] = PickerDetails(agent_id, self.cb)

    """ Picker Manager query functions """
    def get_task(self):
        """ Return list of currently active task_ids

        :return: list of picker.task_id
        """
        return [P.task for P in self.agent_details]
    def format_task_obj(self, picker_id=None, task_id=None):
        """ Create strands_executive_msgs.msg.Task object to send to TOC
        Must have either a picker_id or a task_id given

        :param picker_id: picker to get taks information from
        :param task_id: task to get information about
        :return: strands_executive_msgs.msg.Task detailing task information
        """
        if task_id:
            picker = self.get_task_handler(task_id)
        elif picker_id:
            picker = self.agent_details[picker_id]
        else:
            return None
        task = strands_executive_msgs.msg.Task
        task.task_id = picker.task_id
        task.start_node_id = picker.task_location
        task.action = picker.task_action
        return task

    """ Unassigned tasks queries """
    def assigned_tasks(self):
        """ Create dictionary of assigned tasks
        :return: {task_id: agent_id} for each picker if task is assigned
        """
        return {P.task_id:P.picker_id for P in self.agent_details.values() if P.task_id and P.task_stage is not "CREATED"}
    def unassigned_tasks(self):
        """ Create dictionary of unassigned tasks
        :return: {task_id: agent_id} for each picker if task is unassigned
        """
        return {P.task_id:P.picker_id for P in self.agent_details.values() if P.task_id and P.task_stage is "CREATED"}
    def get_unassigned_tasks_ordered(self):
        """ Create dictionary of unassigned tasks ordered by priority
        :return: {task_id: agent_id} for each picker if task is unassigned ordered by priority
        """
        unassigned_pickers = [P for P in self.agent_details.values() if P.task_stage is "CREATED"]
        picker_priorities = sorted(list(set([P.task_priority for P in unassigned_pickers])), reverse=True)
        tasks = []
        for priority in picker_priorities:
            tasks = tasks + sorted([P.task_id for P in unassigned_pickers if P.task_priority == priority])
        return tasks

    """Communication from coordinator"""
    def task_updates(self, picker_id='', task_id='', robot_id='', state=''):
        """ Update state of task/picker from the coordinator
        Will identify the picker from the task_id if not defined.

        Messages will be further processed if they are one of the following:
        > ACCEPT: an unassigned taks has been accepted by a robot
        > ARRIVED: the robot has arrived at the picker
        > ABANDONED: the robt has been unable to find a route / the robot has cancelled its connection to the task

        :param picker_id: picker to update (optional)
        :param task_id: task_id to update (not required if picker_id given)
        :param robot_id: robot assigned to task
        :param state: new sate of task
        :return: None
        """

        if picker_id == '' and task_id != '':
            picker_task_dict = {picker.task_id: picker.picker_id for picker in self.agent_details.values()}
            #if picker.task_id is not None} #TODO: add this in eventually
            if task_id not in picker_task_dict:#TODO: remove this eventually
                return
            else:
                picker_id = picker_task_dict[task_id]

        #Only allow communications which should come from the coordinator
        if state in ["ACCEPT", "ARRIVED", "ABANDONED"]:#ASSIGNED
            self.update_state(state, picker_id, robot_id)

    """Communication from CAR"""
    def car_event_cb(self, msg):
        """ Update state of task/picker based on response from CAR

        Two messages currently come from the callback, messages contaiing "states" are ignored.
        The message is evauated from a string to contain two properties, the picker_id and the new state.

        Messages will be further processed if they are one of the following:
        > CALLED: a new task is to be crated
        > LOADED: the tray has been loaded onto the robot
        > INIT: a picker has first conected / a taks has been cancelled / a task has completed

        :param msg: evalueted to "{user:picker_id,state:new_state}"
        :return: None
        """

        msg = eval(msg.data)

        # Currently picker_state_monitor publishes the states of each picker
        # this is not needed in the new approach
        if "states" in msg: #TODO: remove this
            return

        # If the given state is valid, update the picker object
        if msg["state"] in ["CALLED", "LOADED", "INIT"]:#CREATED
            self.update_state(new_state=msg["state"], picker_id=msg["user"])

    """ Modify State of Picker """
    def update_state(self, new_state, picker_id, robot_id=''):
        """ Update the current state of the picker to the given new_state.

        If the new_state is CALLED and a task_id exists, return
        If the new_state is CALLED and a task_id does not exist, create a new task_id

        Based on the new_state, set the picker's current state based on the associated response in actions[new_state]

        :param new_state: the new state to assign to the picker
        :param picker_id: the picker to modify
        :param robot_id: n/a
        :return: None
        """

        # Define picker in question
        if picker_id not in self.agent_details:
            logmsg(level="error", category="task", id=picker_id, msg='picker object does not exist')
            return

        picker = self.agent_details[picker_id]

        # Load new task id onto picker
        if new_state == "CALLED":
            if picker.task_id: #task was released by robot
                return
            else: #new task is created
                picker.task_id = self.new_task_id()
                logmsg(category="task", id=picker.task_id, msg='task called by %s' % (picker.agent_id))

        # Every valid action must have a task_id by this point
        if not picker.task_id:
            return

        # Take appropriate action
        actions = {"CALLED":    picker.task_created,     # +new task  #"CREATED
                   "ACCEPT":    picker.task_assigned,    #            #"ASSIGNED"
                   "ARRIVED":   picker.robot_arrived,    #
                   "LOADED":    picker.robot_loaded,     # +task complete
                   "ABANDONED": picker.task_abandonded,  # +task complete
                   "INIT":      picker.task_canceled}    #
        actions[new_state]()
    def new_task_id(self): #TODO: could add a tack lock here for safety?
        """ Increment an internal counter to define new task_id's.

        :return: an unused task_id
        """
        self.latest_task_id = self.latest_task_id+1
        return self.latest_task_id



"""Centralised container for all details pertaining to the picker"""
class PickerDetails(AgentDetails):

    def __init__(self, ID, cb):
        """ Class to define details and interactions for an individual picker.

        :param ID: Unique identifier for picker
        :param cb: Callbacks to direct response from location subscribers
        """

        """Initialise Fields in Parent Class"""
        super(PickerDetails, self).__init__(ID, cb)

        """Meta Management"""
        self.picker_id = ID
        self.registered = True

        """Task Details"""
        self.task_type = None
        self.task_id = None
        self.task_location = None
        self.task_action = None
        self.task_priority = 1
        self.picker_task = False
        self.picker_states = None
        self.task_stage = None
        self.start_time = Now()

        """Picker State Monitor Details""" #TODO: Find out if this is a necessary inclusion
        self.posestamped = None
        self.posestamped_sub = Sub("/%s/posestamped" % ID, PoseStamped, self.picker_posestamped_cb)

        """Picker information"""
        self.time_connected = Now()
        self.virtual = False

        """ State Publisher """
        self.car_state_pub = rospy.Publisher("/car_client/set_states", Str, latch=True, queue_size=5)

        """ Custom named pose_stamped publisher """
        self.posestamped_pub = rospy.Publisher("/%s/pose_stamped" % self.picker_id, PoseStamped, latch=True, queue_size=5)

    def _remove(self):
        """ When shutting down coordinator, unregister posestamped_sub_cb
        :return: None
        """
        super(PickerDetails, self)._remove()
        self.posestamped_sub.unregister()

    """State Monitoring""" #TODO: simplify this and remove repeated lines
    def task_created(self): #called by picker
        """ Called by the picker, define a new task """
        self.task_stage = "CREATED"
        self.task_location = self._get_start_node()
        self.start_time = Now()
    def task_assigned(self): #courtesy call by coordinator
        """ Courtesy call by coordinator, inform picker of robot being assigned """
        self.task_stage = "ASSIGNED"
        self.start_time = Now()
        self.set_picker_state("ACCEPT")
    def robot_arrived(self): #called by coordinator
        """ Called by coordinator, inform picker of robot arrival """
        self.task_stage = "ARRIVED"
        self.start_time = Now()
        self.set_picker_state("ARRIVED")
    def robot_loaded(self): #called by picker
        """ Called by picker, inform coordinator of robot being loaded """
        self.task_stage = "LOADED"
        self.start_time = Now()
        #coordinator queries this task_stage to know when to let robot leave
    def task_finished(self): #called by coordinator
        """ Called by coordinator, inform picker of task completion """
        self.task_id = None
        self.task_stage = None
        self.start_time = Now()
        self.set_picker_state("INIT")
    def task_canceled(self): #called by picker
        """ Called by picker, inform coordinator of task cancellation """
        task_id = self.task_id
        self.task_id = None
        self.task_stage = None
        self.start_time = Now()
        self.set_picker_state("INIT")
        self.cb['task_cancelled'](task_id) #needs to tell coordinator that the robot doesnt need to come anymore
        #this callback can maybe be avoided by making coordinator generate new list of tasks locally
    def task_abandonded(self): #courtesy call by coordinator
        """ Called by coordinator, inform picker that robot has abandoned task """
        self.task_created() #task cancelled by robot, reset to how it was before robot assigned
        self.set_picker_state("CALLED")


    """ Inform picker of given stage """
    def set_picker_state(self, state):  #TODO: replace with KeyValuePair?
        """ Format and pulish message to inform picker of update to current state.

        :param state: new state
        :return: None
        """
        msg = Str()
        msg.data = '{\"user\":\"%s\", \"state\": \"%s\"}' % (self.picker_id, state)
        self.car_state_pub.publish(msg)

    """ Reset Picker Location """
    def _reset_picker_node_cb(self):
        """ reset_picker_node callback
        """
        # unregister any existing subscribers
        # if subscribers are non-existent it should not create any problems
        # https://github.com/ros/ros_comm/blob/11ebad/clients/rospy/src/rospy/topics.py#L176
        self.current_node_sub.unregister()
        self.closest_node_sub.unregister()
        self.posestamped_sub.unregister()

        self.previous_node = None
        self.current_node = None
        self.closest_node = None

        self.current_node_sub = Sub(self.picker_id+"/current_node", Str, self._current_node_cb)
        self.closest_node_sub = Sub(self.picker_id+"/closest_node", Str, self._closest_node_cb)
        self.posestamped_sub = Sub(self.picker_id+"/posestamped", PoseStamped, self.picker_posestamped_cb)

    """ Set Picker Location Pose """
    def _set_picker_node_cb(self, node):
        """ Manual override to take control of picker's current location. (Debugging Tool)
        Callback for "picker_id/move_current_node"

        Unregisters from current_node subscriber and pulishes message to its callback.

        :param node: String containing fake current_node
        :return: None
        """
        self.current_node_sub.unregister()
        self.closest_node_sub.unregister()
        self.posestamped_sub.unregister()

        msg = Str()
        msg.data = node
        self._current_node_cb(msg)
        self._closest_node_cb(msg)
        self._posestamped_cb(msg)

    def _current_node_cb(self, msg):
        """ Callback for "picker_id/current_node"

        If the current_node is different from the message and an associated task exists, update the robot's goal.

        :param msg: string containing the new current_node for the picker
        :return: None
        """
        goal_update = False
        if self.current_node != msg.data:
            logmsg(category="picker", id=self.agent_id, msg='moved to new location %s' % (msg.data))
            goal_update = True #TODO: not necessarially, either could be None
        super(PickerDetails, self)._current_node_cb(msg)
        if goal_update and self.task_id:
            logmsg(category="task", id=self.task_id, msg='new target assigned to robot')
            self.cb['task_update']("picker_node_update", self.task_id, self.current_node)

    def _closest_node_cb(self, msg):
        """ Callback for "picker_id/closest_node

        :param msg: string containing the new closest_node for the picker
        :return: None
        """
        super(PickerDetails, self)._closest_node_cb(msg)

    def _posestamped_cb(self, msg):
        """ Callback for picker_id/posestamped

        :param msg: string containing the new current_node for the picker
        :return: None
        """
        if 'get_node' in self.cb:
            node = self.cb['get_node'](msg.data)
            posestamped_msg = PoseStamped()
            posestamped_msg.pose = node.pose
            self.picker_posestamped_cb(posestamped_msg)

    """ pose_stamped is needed for RViz visualisation"""
    def picker_posestamped_cb(self, msg):
        """ Callback for "picker_id/posestamped"

        :param msg: picker pose
        :return: None
        """
        self.posestamped = msg
        self.posestamped_pub.publish(self.posestamped)


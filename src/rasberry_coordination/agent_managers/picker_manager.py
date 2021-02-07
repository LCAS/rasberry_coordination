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
from rasberry_coordination.agent_managers.agent_manager import AgentManager, AgentDetails
from geometry_msgs.msg import PoseStamped
from rasberry_coordination.msg import TasksDetails, TaskUpdates
from rasberry_coordination.coordinator_tools import logmsg

class PickerManager(AgentManager):

    """Initialise class with callback details to apply to pickers"""
    def __init__(self, callback_dict):
        super(PickerManager, self).__init__(callback_dict)
        self.latest_task_id = 0
        self.dump_cb = Sub("rasberry_coordination/picker_manager/dump", Str, self.dump_details)

        ns = "rasberry_coordination/picker_manager/"

        # Define interaction services with CAR interface
        self.car_event_sub = rospy.Subscriber("/car_client/get_states", Str, self.car_event_cb)

        #   Define
        # self.task_updates_sub = rospy.Subscriber("rasberry_coordination/task_updates", TaskUpdates, self.task_updates_cb)
        # self.active_tasks_pub = rospy.Publisher("rasberry_coordination/active_tasks_details", TasksDetails, latch=True, queue_size=5)


    """Add Picker Details Objects"""
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = PickerDetails(agent_id, self.cb)

    """ Picker Manager query functions """
    def get_task(self):
        return [P.task for P in self.agent_details]

    def format_task_obj(self, picker_id=None, task_id=None): #TODO: remove this
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
        return {P.task_id:P.picker_id for P in self.agent_details.values() if P.task_id and P.task_stage is not "CREATED"}
    def unassigned_tasks(self):
        return {P.task_id:P.picker_id for P in self.agent_details.values() if P.task_id and P.task_stage is "CREATED"}
    def get_unassigned_tasks_ordered(self):
        unassigned_pickers = [P for P in self.agent_details.values() if P.task_stage is "CREATED"]
        picker_priorities = sorted(list(set([P.task_priority for P in unassigned_pickers])), reverse=True)
        tasks = []
        for priority in picker_priorities:
            tasks = tasks + sorted([P.task_id for P in unassigned_pickers if P.task_priority == priority])
        return tasks

    """Communication from coordinator"""
    def task_updates(self, picker_id='', task_id='', robot_id='', state=''):

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
        msg = eval(msg.data)

        # Currently picker_state_monitor publishes the states of each picker
        # this is not needed in the new approach
        if "states" in msg: #TODO: remove this
            return

        # If the given state is valid, update the picker object
        if msg["state"] in ["CALLED", "LOADED", "INIT"]:#CREATED
            self.update_state(new_state=msg["state"], picker_id=msg["user"])

    """
    picker_action CREATE_TASK()  { idle -> wait_accept          | button_1 }
    robot_action  ACCEPT_TASK()  { wait_accept -> wait_arrival  | robot assigned to task }
    robot_action  REACH_PICKER() { wait_arrival -> wait_loading | robot arrives }
    picker_action LOAD_ROBOT()   { wait_loading -> idle         | button_2 }

    picker_action CANCEL_TASK()  { wait_arr... -> idle | button_3 }
    robot_action  ABANDON_TASK() { wait_arr... -> idle | robot ded }
    """
    """ Modify State of Picker """
    def update_state(self, new_state, picker_id, robot_id=''):

        # Define picker in question
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
        self.latest_task_id = self.latest_task_id+1
        return self.latest_task_id



"""Centralised container for all details pertaining to the picker"""
class PickerDetails(AgentDetails):

    def __init__(self, ID, cb):

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

        """ Manual Location Moving """
        self.move_current_node_sub = Sub(ID+"/move_current_node", Str, self._move_current_node_cb)

    def _remove(self):
        super(PickerDetails, self)._remove()
        self.posestamped_sub.unregister()

    """State Monitoring""" #TODO: simplify this and remove repeated lines
    def task_created(self): #called by picker
        self.task_stage = "CREATED"
        self.task_location = self._get_start_node()
        self.start_time = Now()
    def task_assigned(self): #courtesy call by coordinator
        self.task_stage = "ASSIGNED"
        self.start_time = Now()
        self.set_picker_state("ACCEPT")
    def robot_arrived(self): #called by coordinator
        self.task_stage = "ARRIVED"
        self.start_time = Now()
        self.set_picker_state("ARRIVED")
    def robot_loaded(self): #called by picker
        self.task_stage = "LOADED"
        self.start_time = Now()
        #coordinator queries this task_stage to know when to let robot leave
    def task_finished(self): #called by coordinator
        self.task_id = None
        self.task_stage = None
        self.start_time = Now()
        self.set_picker_state("INIT")
    def task_canceled(self): #called by picker
        task_id = self.task_id
        self.task_id = None
        self.task_stage = None
        self.start_time = Now()
        self.set_picker_state("INIT")
        self.cb['task_cancelled'](task_id) #needs to tell coordinator that the robot doesnt need to come anymore
        #this one can maybe be avoided by making coordinator generate new list of tasks locally
    def task_abandonded(self): #courtesy call by coordinator
        self.task_created() #task cancelled by robot, reset to how it was before robot assigned
        self.set_picker_state("CALLED")


    """ Inform picker of given stage """
    def set_picker_state(self, state):  #TODO: replace with KeyValuePair?
        msg = Str()
        msg.data = '{\"user\":\"%s\", \"state\": \"%s\"}' % (self.picker_id, state)
        self.car_state_pub.publish(msg)

    """ Picker Location Pose """
    def _move_current_node_cb(self, msg):
        self.current_node_sub.unregister()
        self._current_node_cb(msg)

    def _current_node_cb(self, msg):
        goal_update = False
        if self.current_node != msg.data:
            logmsg(category="picker", id=self.agent_id, msg='moved to new location %s' % (msg.data))
            goal_update = True #TODO: not necessarially, either could be None
        super(PickerDetails, self)._current_node_cb(msg)
        if goal_update and self.task_id:
            logmsg(category="task", id=self.task_id, msg='new target assigned to robot')
            self.cb['task_update']("picker_node_update", self.task_id, self.current_node)
    def picker_posestamped_cb(self, msg):
        self.posestamped = msg
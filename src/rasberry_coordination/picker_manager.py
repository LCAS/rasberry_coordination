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
from rasberry_coordination.agent_manager import *
from geometry_msgs.msg import PoseStamped
from rasberry_coordination.msg import TasksDetails, TaskUpdates

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

    """Communication from coordinator"""
    def task_updates(self, picker_id='', task_id='', robot_id='', state=''):

        if picker_id == '' and task_id != '':
            picker_task_dict = {picker.task_id:picker.picker_id for picker in self.agent_details.values()}
            picker_id = picker_task_dict[task_id]

        if state in ["CALLED", "ASSIGNED", "ARRIVED", "LOADED", "ABANDONED", "DELIVERED", "CREATED", "ACCEPT"]:
            self.update_state(state, picker_id, robot_id)

    def get_task(self):
        return [picker.task for picker in self.agent_details]




    """Communication from CAR"""
    def car_event_cb(self, msg):
        msg = eval(msg.data)

        # Currently picker_state_monitor publishes the states of each picker
        # this is not needed in the new approach
        if "states" in msg:
            return

        # If the given state is valid, update the picker object
        if msg["state"] in ["CALLED", "LOADED", "INIT"]:
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
        if new_state == "CALLED":#"CREATED"
            picker.task_id = self.new_task_id()

        # Take appropriate action
        actions = {"CALLED":    picker.task_created,   # +new task  #"CREATED
                   "ACCEPT":  picker.task_assigned,    #            #"ASSIGNED"
                   "ARRIVED":   picker.robot_arrived,  #
                   "LOADED":    picker.robot_loaded,   # +task complete
                   "ABANDONED": picker.task_canceled,  # +task complete
                   "DELIVERED": picker.task_canceled}  # +task complete
        actions[new_state]()


    def new_task_id(self):
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
        self.task_id = None
        self.picker_task = False
        self.picker_states = None
        self.task_stage = None
        self.start_time = Now()

        # """Picker State Monitor Details"""
        # self.picker_posestamped = None
        # self.picker_posestamped_sub = Sub("/%s/posestamped" % ID, PoseStamped, self.picker_posestamped_cb)

        """Picker information"""
        self.time_connected = Now()
        self.virtual = False

        """Consistency verificaition"""
        # Srv("rasberry_coordination/picker_manager/get_new_task_id", NewTask, self.get_new_task_id)
        # def get_new_task_id(self, req):
        #     resp = 1
        #     self.max_task_id = self.max_task_id + 1
        #     return resp
        # self.get_new_task_id_srv = rospy.ServiceProxy('rasberry_coordination/picker_manager/get_new_task_id', NewTask)

        """ State Publisher """
        # self.car_state_pub = rospy.Publisher("/car_client/set_states", Str, latch=True, queue_size=5)

    # def _remove(self):
    #     super(PickerDetails, self)._remove()
    #     self.picker_posestamped_sub.unregister()

    """State Monitoring"""
    def task_created(self):
        self.task_stage = "CREATED"
        self.start_time = Now()
        self.set_picker_state("CREATED") #this requires task id (task id managed by picker_manager)
    def task_assigned(self):
        self.task_stage = "ASSIGNED"
        self.start_time = Now()
    def robot_arrived(self):
        self.task_stage = "ARRIVED"
        self.start_time = Now()
    def robot_loaded(self): #at what point is the task completed for the picker??
        self.task_stage = "LOADED"
        self.start_time = Now()
        self.set_picker_state("LOADED") #needs to let coordinator know robot is ready to leave
    def task_canceled(self):
        self.task_id = None
        self.task_stage = None
        self.start_time = Now()
        self.set_picker_state("INIT") #needs to tell coordinator that the robot doesnt need to come anymore
        #this one can maybe be avoided by making coordinator generate new list of tasks locally
    def task_abandonded(self):
        self.task_id = None
        self.task_stage = None
        self.start_time = Now()



    def set_picker_state(self, state):
        msg = Str()
        msg.data = str({"user": self.picker_id, "state": state})
        # self.car_state_pub.publish(msg)

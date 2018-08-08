#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy

import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv
import strands_executive_msgs.msg

import rasberry_coordination.srv


class PickerStateMonitor(object):
    """A class to monitor all pickers' state changes
    """
    def __init__(self, unified=False):
        self.unified = unified
        self.n_pickers = 0
        self.picker_ids = []
        self.picker_states = {}
        self.picker_prev_states = {}
        self.picker_posestamped = {}

        self.picker_posestamped_subs = {}
        self.collect_trays = {}

        self.picker_closest_nodes = {}
        self.picker_closest_node_subs = {}

        self.picker_current_nodes = {}
        self.picker_current_node_subs = {}

        self.picker_task = {} # picker_id: True/False if task is added

        self.car_event_sub = rospy.Subscriber("/car_client/get_states", std_msgs.msg.String, self.car_event_cb)

        self.car_state_pub = rospy.Publisher("/car_client/set_states", std_msgs.msg.String, queue_size=5)

        rospy.wait_for_service("/rasberry_coordination/add_task")
        self.add_task_client = rospy.ServiceProxy("/rasberry_coordination/add_task", strands_executive_msgs.srv.AddTask)

        rospy.wait_for_service("/rasberry_coordination/cancel_task")
        self.cancel_task_client = rospy.ServiceProxy("/rasberry_coordination/cancel_task", rasberry_coordination.srv.CancelTask)

        self.task_picker = {} # tasks by which picker {task_id: picker_id}
        self.task_robot = {} # assigned robots {task_id: robot_id}
        self.tray_loaded = {} # service clients to send loaded status to robots
        self.task_state = {}

        self.task_updates_sub = rospy.Subscriber("/picker_state_monitor/task_updates", rasberry_coordination.msg.TaskUpdates, self.task_updates_cb)
        rospy.loginfo("Picker state monitor initialised")

    def car_event_cb(self, msg):
        """callback function for /car_client/get_states
        """
        msg_data = eval(msg.data)
        if "states" in msg_data:
            # state updates for all users
            for user_id in msg_data["states"]:
                if user_id not in self.picker_ids and user_id.lower().startswith("picker"):
                    self.picker_prev_states[user_id] = msg_data["states"][user_id]
                    self.picker_states[user_id] = msg_data["states"][user_id]
                    self.picker_posestamped[user_id] = None
                    self.picker_posestamped_subs[user_id] = rospy.Subscriber("/%s/posestamped" %(user_id), geometry_msgs.msg.PoseStamped, self.picker_posestamped_cb, callback_args="%s" %(user_id))
                    self.picker_closest_nodes[user_id] = None
                    self.picker_closest_node_subs[user_id] = rospy.Subscriber("/%s/closest_node" %(user_id), std_msgs.msg.String, self.picker_closest_node_cb, callback_args="%s" %(user_id))
                    self.picker_current_nodes[user_id] = None
                    self.picker_current_node_subs[user_id] = rospy.Subscriber("/%s/current_node" %(user_id), std_msgs.msg.String, self.picker_current_node_cb, callback_args="%s" %(user_id))
                    self.picker_task[user_id] = False
                    self.picker_ids.append(user_id)
                    self.n_pickers += 1

                elif user_id in self.picker_ids:
#                    rospy.loginfo(">> %s" %(user_id))
#                    rospy.loginfo(self.picker_prev_states[user_id])
#                    rospy.loginfo(self.picker_states[user_id])
#                    rospy.loginfo(msg_data["states"][user_id])
                    # update state only if the state for this user has been changed
                    if self.picker_states[user_id] != msg_data["states"][user_id]:
                        rospy.loginfo("updating picker states")
                        self.picker_prev_states[user_id] = self.picker_states[user_id]
                        self.picker_states[user_id] = msg_data["states"][user_id]

                if user_id in self.picker_ids and self.picker_states[user_id] == "INIT":
                    # if prev_state was CALLED, cancel the task
                    if self.picker_prev_states[user_id] == "CALLED" or self.picker_prev_states[user_id] == "ACCEPT":
                        try:
#                            assert req.task_id is not None
                            assert self.get_pickers_task(user_id) is not None
                        except:
                            pass
                        else:
                            # TODO: picker cancelling task
                            rospy.loginfo("TODO: picker cancelling task")
#                            self.cancel_task_client(self.get_pickers_task(user_id), self.task_robot[self.get_pickers_task(user_id)])
#                            self.picker_prev_states[user_id] = "INIT"
#                            self.set_picker_state(user_id, "INIT")

                    elif self.picker_prev_states[user_id] == "LOADED":
                        # continue picking.
                        pass

                elif user_id in self.picker_ids and self.picker_states[user_id] == "CALLED" and self.picker_prev_states[user_id] == "INIT":
                    # add a task, track the task with picker, task_id
                    if not self.picker_task[user_id]:
                        task = strands_executive_msgs.msg.Task()
                        task.action = "CollectTray"
                        task.start_node_id = self.picker_closest_nodes[user_id] # this is the picker_node

                        task_id = self.add_task_client(task)
                        self.task_picker[task_id.task_id] = user_id
                        self.picker_task[user_id] = True

                elif user_id in self.picker_ids and self.picker_states[user_id] == "ARRIVED":
                    # this state is set from robot's feedback that it arrived at picker_node to coordinator
                    # no action needed to be taken here
                    pass

                elif user_id in self.picker_ids and self.picker_states[user_id] == "LOADED" and self.picker_prev_states[user_id] == "ARRIVED":
                    # set the tray loaded directly to the corresponding robot (how?)
                    task_id = self.get_pickers_task(user_id)
                    try:
                        assert task_id is not None
                    except:
                        rospy.loginfo("task_id is None in car_event_cb")
                        pass
                    else:
                        robot_id = self.task_robot[task_id]
                        if robot_id not in self.tray_loaded:
                            if self.unified:
                                # there willbe only one robot
                                self.tray_loaded[robot_id] = rospy.ServiceProxy("/tray_loaded", std_srvs.srv.Trigger)
                            else:
                                self.tray_loaded[robot_id] = rospy.ServiceProxy("/%s/tray_loaded" %(robot_id), std_srvs.srv.Trigger)
                        self.tray_loaded[robot_id]()
                        # remove task_id from task_picker
                        self.task_state[task_id] = "LOADED" # this needs to be set here to avoid problems with task_updates_cb
                        self.task_picker.pop(task_id)
                        self.picker_prev_states[user_id] = self.picker_states[user_id]
                        self.set_picker_state(user_id, "INIT")
                        self.picker_task[user_id] = False

                else:
                    pass

        elif "state" in msg_data:
            # resetting state, ARRIVED -> LOADED
            user_id = msg_data["user"]
            if user_id in self.picker_ids:
                if self.picker_states[user_id] != msg_data["state"]:
                    self.picker_prev_states[user_id] = self.picker_states[user_id]
                    self.picker_states[user_id] = msg_data["state"]

    def picker_posestamped_cb(self, msg, picker_id):
        """call back to /picker_id/posestamped topics
        """
        self.picker_posestamped[picker_id] = msg

    def picker_closest_node_cb(self, msg, picker_id):
        """call back to /picker_id/closest_node topics
        """
        self.picker_closest_nodes[picker_id] = msg.data

    def picker_current_node_cb(self, msg, picker_id):
        """call back to /picker_id/current_node topics
        """
        self.picker_current_nodes[picker_id] = msg.data

    def set_picker_state(self, picker_id, state):
        msg = std_msgs.msg.String()
        msg.data = '{\"user\":\"%s\", \"state\": \"%s\"}' %(picker_id, state)
#        rospy.loginfo(msg)
        #rostopic pub /car_client/set_states std_msgs/String "data: '{\"user\":\"picker01\", \"state\": \"ACCEPT\"}'"
        for i in range(5):
            # publish multiple times to make sure it is set
            # TODO: make sure it is set?
            rospy.sleep(0.1)
            self.car_state_pub.publish(msg)

    def task_updates_cb(self, msg):
        """call back for task_updates
        """
        if msg.task_id not in self.task_robot:
            self.task_robot[msg.task_id] = None

        if msg.task_id not in self.task_state:
            self.task_state[msg.task_id] = None

        if msg.state == "ACCEPT" and self.task_state[msg.task_id] != "ACCEPT":
            # a robot has been assigned to do the task
            self.task_state[msg.task_id] = "ACCEPT"
            picker_id = self.task_picker[msg.task_id]
            self.task_robot[msg.task_id] = msg.robot_id
            if msg.robot_id not in self.tray_loaded:
                if self.unified:
                    # there will be only one robot
                    self.tray_loaded[msg.robot_id] = rospy.ServiceProxy("/tray_loaded", std_srvs.srv.Trigger)
                else:
                    self.tray_loaded[msg.robot_id] = rospy.ServiceProxy("/%s/tray_loaded" %(msg.robot_id), std_srvs.srv.Trigger)

            self.picker_prev_states[picker_id] = self.picker_states[picker_id]
            self.set_picker_state(picker_id, "ACCEPT")

        elif msg.state == "ARRIVED" and self.task_state[msg.task_id] != "ARRIVED":
            # robot has arrived at the picker
            self.task_state[msg.task_id] = "ARRIVED"
            picker_id = self.task_picker[msg.task_id]
            self.picker_prev_states[picker_id] = self.picker_states[picker_id]
            self.set_picker_state(picker_id, "ARRIVED")

        elif msg.state == "LOADED" and self.task_state[msg.task_id] != "LOADED":
            # TODO: robot assumed the tray is laoded after max_load_duration
            self.task_state[msg.task_id] = "LOADED"
            picker_id = self.task_picker[msg.task_id]
            if self.picker_states[picker_id] != "LOADED":
                self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                self.set_picker_state(picker_id, "LOADED")

        elif msg.state == "DELIVERED" and self.task_state[msg.task_id] != "DELIVERED":
            # remove task_id from task_robot
            self.task_state[msg.task_id] = "DELIVERED"
            self.task_robot.pop(msg.task_id)


    def get_pickers_task(self, picker_id):
        """get the picker's task.
        """
        # assuming
        for task_id in self.task_picker:
            if task_id in self.task_robot:
                if self.task_picker[task_id] == picker_id and self.task_robot[task_id] is not None:
                    return task_id
        return None

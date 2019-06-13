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
import rasberry_coordination.msg
import rasberry_coordination.srv


class PickerStateMonitor(object):
    """A class to monitor all pickers' state changes
    """
    def __init__(self, virtual_pickers):
        """
        """
        self.n_pickers = 0
        self.picker_ids = []
        self.picker_states = {}
        self.picker_prev_states = {}
        self.picker_posestamped = {}

        self.virtual_pickers = virtual_pickers

        self.picker_posestamped_subs = {}
        self.collect_trays = {}

        self.picker_closest_nodes = {}
        self.picker_closest_node_subs = {}

        self.picker_current_nodes = {}
        self.picker_current_node_subs = {}

        self.picker_task = {} # picker_id: True/False if task is added

        self.car_event_sub = rospy.Subscriber("/car_client/get_states", std_msgs.msg.String, self.car_event_cb)

        self.car_state_pub = rospy.Publisher("/car_client/set_states", std_msgs.msg.String, latch=True, queue_size=5)

        rospy.wait_for_service("/rasberry_coordination/add_task")
        self.add_task_client = rospy.ServiceProxy("/rasberry_coordination/add_task", strands_executive_msgs.srv.AddTask)

        rospy.wait_for_service("/rasberry_coordination/cancel_task")
        self.cancel_task_client = rospy.ServiceProxy("/rasberry_coordination/cancel_task", strands_executive_msgs.srv.CancelTask)

        self.task_picker = {} # tasks by which picker {task_id: picker_id}
        self.task_time = {} # time at which task is added {task_id: time}
        self.task_robot = {} # assigned robots {task_id: robot_id}

        self.tray_loaded = {} # service clients to send loaded status to robots
        self.task_state = {}

        self.task_updates_sub = rospy.Subscriber("/picker_state_monitor/task_updates", rasberry_coordination.msg.TaskUpdates, self.task_updates_cb)
        rospy.loginfo("PickerStateMonitor object is successfully initialised")

    def car_event_cb(self, msg):
        """callback function for /car_client/get_states
        """
        msg_data = eval(msg.data)
        if "states" in msg_data:
            # state updates for all users
            for picker_id in msg_data["states"]:
                if picker_id not in self.picker_ids and picker_id.lower().startswith("picker"):
                    self.picker_prev_states[picker_id] = msg_data["states"][picker_id]
                    self.picker_states[picker_id] = msg_data["states"][picker_id]
                    self.picker_posestamped[picker_id] = None
                    self.picker_posestamped_subs[picker_id] = rospy.Subscriber("/%s/posestamped" %(picker_id), geometry_msgs.msg.PoseStamped, self.picker_posestamped_cb, callback_args="%s" %(picker_id))
                    self.picker_closest_nodes[picker_id] = "none"
                    self.picker_closest_node_subs[picker_id] = rospy.Subscriber("/%s/closest_node" %(picker_id), std_msgs.msg.String, self.picker_closest_node_cb, callback_args="%s" %(picker_id))
                    self.picker_current_nodes[picker_id] = "none"
                    self.picker_current_node_subs[picker_id] = rospy.Subscriber("/%s/current_node" %(picker_id), std_msgs.msg.String, self.picker_current_node_cb, callback_args="%s" %(picker_id))
                    self.picker_task[picker_id] = False
                    self.picker_ids.append(picker_id)
                    self.n_pickers += 1

                elif picker_id in self.picker_ids:
                    # update state only if the state for this user has been changed
                    if self.picker_states[picker_id] != msg_data["states"][picker_id]:
                        rospy.loginfo("updating picker states")
                        self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                        self.picker_states[picker_id] = msg_data["states"][picker_id]

                if picker_id in self.picker_ids and self.picker_states[picker_id] == "INIT":
                    # if prev_state was CALLED, cancel the task
                    if self.picker_prev_states[picker_id] == "CALLED" or self.picker_prev_states[picker_id] == "ACCEPT":
                        task_id = self.get_pickers_task(picker_id)
                        try:
                            assert task_id is not None
                        except:
                            pass
                        else:
                            rospy.loginfo("picker-%s is cancelling task-%d", picker_id, task_id)
                            cancelled = self.cancel_task_client(task_id)
                            if cancelled:
                                # setting previous state as INIT
                                self.picker_prev_states[picker_id] = "INIT"
                                self.set_picker_state(picker_id, "INIT")
                                self.picker_task[picker_id] = False
                                self.task_robot[task_id] = None

                    elif self.picker_prev_states[picker_id] == "LOADED":
                        # continue picking.
                        pass
                    else:
                        # for some inactive pickers it could come here based
                        # on whether he is already in call-a-robot users, but
                        # is a problem for active pickers
                        # TODO: Checking whether active pickers come here
                        pass

                elif picker_id in self.picker_ids and self.picker_states[picker_id] == "CALLED" and self.picker_prev_states[picker_id] == "INIT":
                    # add a task, track the task with picker, task_id
                    if not self.picker_task[picker_id]:
                        if self.picker_closest_nodes[picker_id] == "none":
                            # picker is not localised. do not add a task
                            # reset picker state
                            # prev_state is set to INIT as this is not a cancellation by picker
                            rospy.loginfo("ignoring call as picker %s is not localised", picker_id)
                            self.picker_prev_states[picker_id] = "INIT"
                            self.set_picker_state(picker_id, "INIT")

                        else:
                            task = strands_executive_msgs.msg.Task()
                            task.action = "CollectTray"
                            task.start_node_id = self.picker_closest_nodes[picker_id] # this is the picker_node
                            if picker_id in self.virtual_pickers:
                                task.priority = 0
                            else:
                                task.priority = 1

                            add_task_resp = self.add_task_client(task)
                            self.task_picker[add_task_resp.task_id] = picker_id
                            self.task_time[add_task_resp.task_id] = rospy.get_rostime()
                            self.picker_task[picker_id] = True

                    else:
                        # this may happen as multiple status messages with the same state for one picker
                        # may be received if any other picker changed his state
                        pass
#                        msg = "Picker %s has a callarobot task being processed" %(picker_id)
#                        raise Exception(msg)

                elif picker_id in self.picker_ids and self.picker_states[picker_id] == "ARRIVED":
                    # this state is set from robot's feedback that it arrived at picker_node to coordinator
                    # no action needed to be taken here
                    pass

                elif picker_id in self.picker_ids and self.picker_states[picker_id] == "LOADED" and self.picker_prev_states[picker_id] == "ARRIVED":
                    # set the tray loaded directly to the corresponding robot (how?)
                    task_id = self.get_pickers_task(picker_id)
                    try:
                        assert task_id is not None
                    except:
                        rospy.loginfo("updating tray_loaded status, but picker - %s doesn't have any tasks!!!", picker_id)
                        pass
                    else:
                        robot_id = self.task_robot[task_id]
                        if picker_id not in self.tray_loaded:
                            self.tray_loaded[picker_id] = rospy.ServiceProxy("/rasberry_coordination/tray_loaded",
                                                                             rasberry_coordination.srv.TrayLoaded)
                        req = rasberry_coordination.srv.TrayLoadedRequest()
                        req.picker_id = picker_id
                        req.robot_id = robot_id
                        req.task_id = task_id
                        self.tray_loaded[picker_id](req)

                        # remove task_id from task_picker
                        self.task_state[task_id] = "LOADED" # this needs to be set here to avoid problems with task_updates_cb
                        self.task_picker.pop(task_id)
                        self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                        self.set_picker_state(picker_id, "INIT")
                        self.picker_task[picker_id] = False
                        self.task_robot[task_id] = None # resetting the robot assigned info

                else:
                    pass

        elif "state" in msg_data:
            picker_id = msg_data["user"]
            if picker_id in self.picker_ids:
                # resetting state, ARRIVED -> LOADED
                if self.picker_states[picker_id] != msg_data["state"]:
                    self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                    self.picker_states[picker_id] = msg_data["state"]
            elif picker_id not in self.picker_ids and picker_id.lower().startswith("picker"):
                # new picker - first message could be CALLED
                self.picker_prev_states[picker_id] = "INIT"
                self.picker_states[picker_id] = msg_data["state"]
                self.picker_posestamped[picker_id] = None
                self.picker_posestamped_subs[picker_id] = rospy.Subscriber("/%s/posestamped" %(picker_id), geometry_msgs.msg.PoseStamped, self.picker_posestamped_cb, callback_args="%s" %(picker_id))
                self.picker_closest_nodes[picker_id] = "none"
                self.picker_closest_node_subs[picker_id] = rospy.Subscriber("/%s/closest_node" %(picker_id), std_msgs.msg.String, self.picker_closest_node_cb, callback_args="%s" %(picker_id))
                self.picker_current_nodes[picker_id] = "none"
                self.picker_current_node_subs[picker_id] = rospy.Subscriber("/%s/current_node" %(picker_id), std_msgs.msg.String, self.picker_current_node_cb, callback_args="%s" %(picker_id))
                self.picker_task[picker_id] = False
                self.picker_ids.append(picker_id)
                self.n_pickers += 1

                if self.picker_states[picker_id] == "CALLED" and self.picker_prev_states[picker_id] == "INIT":
                    # add a task, track the task with picker, task_id
                    if not self.picker_task[picker_id]:
                        if self.picker_closest_nodes[picker_id] == "none":
                            # picker is not localised. do not add a task
                            # reset picker state
                            # prev_state is set to INIT as this is not a cancellation by picker
                            rospy.loginfo("ignoring call as picker %s is not localised", picker_id)
                            self.picker_prev_states[picker_id] = "INIT"
                            self.set_picker_state(picker_id, "INIT")

                        else:
                            task = strands_executive_msgs.msg.Task()
                            task.action = "CollectTray"
                            task.start_node_id = self.picker_closest_nodes[picker_id] # this is the picker_node

                            add_task_resp = self.add_task_client(task)
                            self.task_picker[add_task_resp.task_id] = picker_id
                            self.task_time[add_task_resp.task_id] = rospy.get_rostime()
                            self.picker_task[picker_id] = True

                    else:
                        # this shouldn't happen as a task already exists
                        msg = "Picker %s has a callarobot task being processed" %(picker_id)
                        raise Exception(msg)

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
            self.tray_loaded[msg.picker_id] = rospy.ServiceProxy("/rasberry_coordination/tray_loaded", rasberry_coordination.srv.TrayLoaded)

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

        elif msg.state == "CALLED" and self.task_state[msg.task_id] != "CALLED":
            # robot failed to reach picker, task could be reassigned
            self.task_state[msg.task_id] = None
            self.task_robot[msg.task_id] = None
            picker_id = self.task_picker[msg.task_id]
            self.set_picker_state(picker_id, "CALLED")

    def get_pickers_task(self, picker_id):
        """get the picker's task.
        each picker can have only one valid task. the latest task will be
        returned
        """
        picker_task = None
        # for all tasks made by all pickers
        for task_id in self.task_picker:
            if self.task_picker[task_id] == picker_id:
                if picker_task is not None:
                    if self.task_time[task_id] > self.task_time[picker_task]:
                        picker_task = task_id
                else:
                    picker_task = task_id
        return picker_task

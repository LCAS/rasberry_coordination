#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import copy
import os
import datetime
import time
import csv

import rospy

import geometry_msgs.msg
import std_msgs.msg
import strands_executive_msgs.msg

import rasberry_coordination.msg
import rasberry_coordination.srv


class PickerStateMonitor(object):
    """A class to monitor all pickers' state changes
    """
    def __init__(self, picker_ids, virtual_picker_ids):
        """
        """
        # allows only configured pickers to make calls
        self.human_picker_ids = picker_ids
        self.virtual_picker_ids = virtual_picker_ids
        self.picker_ids = copy.deepcopy(picker_ids)
        self.picker_ids.extend(virtual_picker_ids)
        self.n_pickers = len(self.picker_ids)

        logs_dir = os.path.join(os.environ["HOME"], "rasberry_coordination_logs")
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
        self.dt = datetime.datetime.now()
        self.log_file = open(os.path.join(logs_dir, self.dt.strftime("%Y%m%d-%H%M%S_picker_states.csv")), "w")
        self.log_headers = ["id", "time", "datetime", "action_type",
                            "picker_id", "task_id", "picker_status_updates",
                            "task_updates", "details", "current_node",
                            "closest_node", "robot_id"]
        self.log_count = 0
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.log_headers)
        self.log_writer.writeheader()
        self.write_log({"action_type": "starting picker_state_monitor"})

        self.picker_prev_states = {}
        self.picker_states = {}
        self.picker_posestamped = {}

        self.picker_posestamped_subs = {}

        self.picker_closest_nodes = {}
        self.picker_closest_node_subs = {}

        self.picker_current_nodes = {}
        self.picker_current_node_subs = {}

        self.picker_task = {} # picker_id: True/False if task is added
        self.tasks = {} # {task_id:task}

        self.task_picker = {} # tasks by which picker {task_id: picker_id}
        self.task_time = {} # time at which task is added {task_id: time}
        self.task_robot = {} # assigned robots {task_id: robot_id}
        self.task_state = {}

        # service client to send add task requests
        rospy.wait_for_service("/rasberry_coordination/add_task")
        self.add_task_client = rospy.ServiceProxy("/rasberry_coordination/add_task", strands_executive_msgs.srv.AddTask)

        # service client to send cancel task requests
        rospy.wait_for_service("/rasberry_coordination/cancel_task")
        self.cancel_task_client = rospy.ServiceProxy("/rasberry_coordination/cancel_task", strands_executive_msgs.srv.CancelTask)

        # service client to send update task requests
        rospy.wait_for_service("/rasberry_coordination/update_task")
        self.update_task_client = rospy.ServiceProxy("/rasberry_coordination/update_task", rasberry_coordination.srv.UpdateTask)

        # service client to send loaded status to robots
        rospy.wait_for_service("/rasberry_coordination/tray_loaded")
        self.tray_loaded_client = rospy.ServiceProxy("/rasberry_coordination/tray_loaded",
                                                     rasberry_coordination.srv.TrayLoaded)

        for picker_id in self.picker_ids:
            self.picker_prev_states[picker_id] = "INIT"
            self.picker_states[picker_id] = "INIT"
            self.picker_posestamped[picker_id] = None

            self.picker_posestamped_subs[picker_id] = rospy.Subscriber("/%s/posestamped" %(picker_id), geometry_msgs.msg.PoseStamped, self.picker_posestamped_cb, callback_args="%s" %(picker_id))

            self.picker_closest_nodes[picker_id] = "none"
            self.picker_closest_node_subs[picker_id] = rospy.Subscriber("/%s/closest_node" %(picker_id), std_msgs.msg.String, self.picker_closest_node_cb, callback_args="%s" %(picker_id))

            self.picker_current_nodes[picker_id] = "none"
            self.picker_current_node_subs[picker_id] = rospy.Subscriber("/%s/current_node" %(picker_id), std_msgs.msg.String, self.picker_current_node_cb, callback_args="%s" %(picker_id))

            self.picker_task[picker_id] = False

        self.car_event_sub = rospy.Subscriber("/car_client/get_states", std_msgs.msg.String, self.car_event_cb)

        self.car_state_pub = rospy.Publisher("/car_client/set_states", std_msgs.msg.String, latch=True, queue_size=5)

        self.task_updates_sub = rospy.Subscriber("/rasberry_coordination/task_updates", rasberry_coordination.msg.TaskUpdates, self.task_updates_cb)
        rospy.loginfo("PickerStateMonitor object is successfully initialised")

    def car_event_cb(self, msg):
        """callback function for /car_client/get_states
        """
        msg_data = eval(msg.data)
        if "states" in msg_data:
            # state updates for all users
            for picker_id in msg_data["states"]:
                if picker_id not in self.picker_ids:
                    # picker is not configured - reset to INIT if not in INIT
                    if msg_data["states"][picker_id] != "INIT":
                        rospy.logwarn("Picker %s is not in the configured picker_ids", picker_id)
                        self.set_picker_state(picker_id, "INIT")
                    continue

                else:
                    # update state only if the state for this user has been changed
                    if self.picker_states[picker_id] != msg_data["states"][picker_id]:
                        rospy.loginfo("updating picker states")
                        self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                        self.picker_states[picker_id] = msg_data["states"][picker_id]

                if self.picker_states[picker_id] == "INIT" and self.picker_prev_states[picker_id] != "INIT":
                    self.write_log({"action_type": "car_update",
                                    "picker_status_updates": "INIT",
                                    "picker_id": picker_id,
                                    "details": "picker state changed to INIT",
                                    "current_node": self.picker_current_nodes[picker_id],
                                    "closest_node": self.picker_closest_nodes[picker_id],
                                    })

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

                            self.write_log({"action_type": "car_update",
                                            "picker_status_updates": "CANCEL",
                                            "task_id": task_id,
                                            "picker_id": picker_id,
                                            "details": "cancel task request",
                                            "current_node": self.picker_current_nodes[picker_id],
                                            "closest_node": self.picker_closest_nodes[picker_id],
                                            })

                            if cancelled.cancelled:
                                # setting previous state as INIT
                                self.picker_prev_states[picker_id] = "INIT"
                                self.set_picker_state(picker_id, "INIT")
                                self.picker_task[picker_id] = False
                                self.task_robot[task_id] = None

                                self.write_log({"action_type": "car_update",
                                            "picker_status_updates": "%s -> CANCEL" %(self.picker_prev_states[picker_id]),
                                            "task_id": task_id,
                                            "picker_id": picker_id,
                                            "details": "cancel task request is successful",
                                            "current_node": self.picker_current_nodes[picker_id],
                                            "closest_node": self.picker_closest_nodes[picker_id],
                                            })
                            else:
                                # resetting state to prev picker state
                                self.set_picker_state(picker_id, self.picker_prev_states[picker_id])
                                self.write_log({"action_type": "car_update",
                                            "picker_status_updates": "%s -> CANCEL" %(self.picker_prev_states[picker_id]),
                                            "task_id": task_id,
                                            "picker_id": picker_id,
                                            "details": "cancel task request is failed",
                                            "current_node": self.picker_current_nodes[picker_id],
                                            "closest_node": self.picker_closest_nodes[picker_id],
                                            })

                    elif self.picker_prev_states[picker_id] == "LOADED":
                        # continue picking.
                        task_id = self.get_pickers_task(picker_id)
                        self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "LOADED -> INIT",
                                        "task_id": task_id,
                                        "picker_id": picker_id,
                                        "details": "tray is loaded. picker resumes picking",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })

                    else:
                        # for some inactive pickers it could come here based
                        # on whether he is already in call-a-robot users, but
                        # is a problem for active pickers
                        # TODO: Checking whether active pickers come here
                        pass

                elif self.picker_states[picker_id] == "CALLED" and self.picker_prev_states[picker_id] == "INIT":
                    # add a task, track the task with picker, task_id, unless a task is already assigned
                    if not self.picker_task[picker_id]:
                        self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "INIT -> CALLED",
                                        "picker_id": picker_id,
                                        "details": "picker calling for a robot",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })

                        if self.picker_closest_nodes[picker_id] == "none":
                            # picker is not localised. do not add a task
                            # reset picker state
                            # prev_state is set to INIT as this is not a cancellation by picker
                            rospy.logwarn("ignoring call as picker %s is not localised", picker_id)
                            self.picker_prev_states[picker_id] = "INIT"
                            self.set_picker_state(picker_id, "INIT")

                            self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "CALLED -> INIT",
                                        "picker_id": picker_id,
                                        "details": "calling failed as picker is not localised",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })

                        else:
                            task = strands_executive_msgs.msg.Task()
                            task.action = "CollectTray"
                            # task_start_node is the picker_node
                            if self.picker_current_nodes[picker_id] != "none":
                                task.start_node_id = self.picker_current_nodes[picker_id]
                            else:
                                task.start_node_id = self.picker_closest_nodes[picker_id]

                            if picker_id in self.human_picker_ids:
                                task.priority = 2
                            if picker_id in self.virtual_picker_ids:
                                task.priority = 1
                            else:
                                task.priority = 0

                            add_task_resp = self.add_task_client(task)
                            task.task_id = add_task_resp.task_id
                            self.tasks[add_task_resp.task_id] = task
                            self.task_picker[add_task_resp.task_id] = picker_id
                            self.task_time[add_task_resp.task_id] = rospy.get_rostime()
                            self.picker_task[picker_id] = True

                            self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "INIT -> CALLED",
                                        "picker_id": picker_id,
                                        "task_id": add_task_resp.task_id,
                                        "details": "picker's call is added to task queue",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })

                    else:
                        # this may happen as multiple status messages with the same state for one picker
                        # may be received if any other picker changed his state
                        pass
#                        msg = "Picker %s has a callarobot task being processed" %(picker_id)
#                        raise Exception(msg)

                elif self.picker_states[picker_id] == "ARRIVED" and self.picker_prev_states[picker_id] != "ARRIVED":
                    # this state is set from robot's feedback that it arrived at picker_node to coordinator
                    # no action needed to be taken here
                    task_id = self.get_pickers_task(picker_id)
                    try:
                        assert task_id is not None
                    except:
                        rospy.logwarn("updating arrived status, but picker - %s doesn't have any tasks!!!", picker_id)
                        self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "CALLED -> ARRIVED",
                                        "picker_id": picker_id,
                                        "details": "picker's call is added to task queue",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })
                    else:
                        self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "CALLED -> ARRIVED",
                                        "picker_id": picker_id,
                                        "task_id": task_id,
                                        "details": "picker's call is added to task queue",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })
                    pass

                elif self.picker_states[picker_id] == "LOADED" and self.picker_prev_states[picker_id] == "ARRIVED":
                    # set the tray loaded directly to the corresponding robot (how?)
                    task_id = self.get_pickers_task(picker_id)
                    try:
                        assert task_id is not None
                    except:
                        rospy.logwarn("updating tray_loaded status, but picker - %s doesn't have any tasks!!!", picker_id)
                        self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "ARRIVED -> LOADED",
                                        "picker_id": picker_id,
                                        "details": "picker's call is added to task queue",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })
                    else:
                        self.write_log({"action_type": "car_update",
                                        "picker_status_updates": "ARRIVED -> LOADED",
                                        "picker_id": picker_id,
                                        "task_id": task_id,
                                        "details": "picker's call is added to task queue",
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": self.picker_closest_nodes[picker_id],
                                        })

                        robot_id = self.task_robot[task_id]

                        req = rasberry_coordination.srv.TrayLoadedRequest()
                        req.picker_id = picker_id
                        req.robot_id = robot_id
                        req.task_id = task_id
                        self.tray_loaded_client(req)

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
                # resetting state to INIT, ARRIVED -> LOADED
                if self.picker_states[picker_id] != msg_data["state"]:
                    self.write_log({"action_type": "car_update",
                                    "picker_status_updates": "%s -> %s" %(self.picker_states[picker_id], msg_data["state"]),
                                    "picker_id": picker_id,
                                    "details": "state updates",
                                    "current_node": self.picker_current_nodes[picker_id],
                                    "closest_node": self.picker_closest_nodes[picker_id],
                                    })

                    self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                    self.picker_states[picker_id] = msg_data["state"]

    def picker_posestamped_cb(self, msg, picker_id):
        """call back to /picker_id/posestamped topics
        """
        self.picker_posestamped[picker_id] = msg

    def picker_closest_node_cb(self, msg, picker_id):
        """call back to /picker_id/closest_node topics
        """
        old_node = self.picker_closest_nodes[picker_id]
        new_node = msg.data
        self.picker_closest_nodes[picker_id] = msg.data

        if (old_node != new_node and
            old_node != "none" and
            new_node != "none"):
            # a valid change in picker position
            if self.picker_task[picker_id]:
                # has a task being processed
                task_id = self.get_pickers_task(picker_id)
                if task_id is not None:
                    task = self.tasks[task_id]
                    self.write_log({"action_type": "task_update request",
                                    "task_updates": "%s -> %s" %(task.start_node_id, new_node),
                                    "picker_id": picker_id,
                                    "task_id": task_id,
                                    "details": "updating task start_node_id",
                                    "current_node": self.picker_current_nodes[picker_id],
                                    "closest_node": new_node,
                                    })
                    task.start_node_id = new_node
                    req = rasberry_coordination.srv.UpdateTaskRequest()
                    req.task = task
                    resp = self.update_task_client(req)

                    if resp.success:
                        self.write_log({"action_type": "task_update request",
                                        "task_updates": "%s -> %s" %(task.start_node_id, new_node),
                                        "picker_id": picker_id,
                                        "task_id": task_id,
                                        "details": "success: updating task start_node_id. %s" %(resp.message),
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": new_node,
                                        })
                    else:
                        self.write_log({"action_type": "task_update",
                                        "task_updates": "%s -> %s" %(task.start_node_id, new_node),
                                        "picker_id": picker_id,
                                        "task_id": task_id,
                                        "details": "fail: updating task start_node_id. %s" %(resp.message),
                                        "current_node": self.picker_current_nodes[picker_id],
                                        "closest_node": new_node,
                                        })

#        self.picker_closest_nodes[picker_id] = msg.data

    def picker_current_node_cb(self, msg, picker_id):
        """call back to /picker_id/current_node topics
        """
        self.picker_current_nodes[picker_id] = msg.data

    def set_picker_state(self, picker_id, state):
        msg = std_msgs.msg.String()
        msg.data = '{\"user\":\"%s\", \"state\": \"%s\"}' %(picker_id, state)
        self.car_state_pub.publish(msg)

        self.write_log({"action_type": "set_car_state",
                        "picker_status_updates": "%s" %(state),
                        "picker_id": picker_id,
                        "details": "setting picker's state in CAR",
                        })

    def task_updates_cb(self, msg):
        """call back for task_updates
        """
        if msg.task_id not in self.task_robot:
            self.task_robot[msg.task_id] = None

        if msg.task_id not in self.task_state:
            self.task_state[msg.task_id] = None

        if msg.task_id in self.task_picker:
            self.write_log({"action_type": "coordinator_task_updates",
                            "task_updates": "%s -> %s" %(self.task_state[msg.task_id], msg.state),
                            "picker_id": self.task_picker[msg.task_id],
                            "task_id": msg.task_id,
                            "details": "task state changed in coordinator.",
                            "robot_id": msg.robot_id,
                            })
        else:
            self.write_log({"action_type": "coordinator_task_updates",
                            "task_updates": "%s -> %s" %(self.task_state[msg.task_id], msg.state),
                            "task_id": msg.task_id,
                            "details": "task state changed in coordinator.",
                            "robot_id": msg.robot_id,
                            })

        if msg.state == "CALLED" and self.task_state[msg.task_id] != "CALLED":
            # robot failed to reach picker, task could be reassigned
            self.task_state[msg.task_id] = None
            self.task_robot[msg.task_id] = None
            picker_id = self.task_picker[msg.task_id]
            self.set_picker_state(picker_id, "CALLED")

        elif msg.state == "ACCEPT" and self.task_state[msg.task_id] != "ACCEPT":
            # a robot has been assigned to do the task
            self.task_state[msg.task_id] = "ACCEPT"
            picker_id = self.task_picker[msg.task_id]
            self.task_robot[msg.task_id] = msg.robot_id

            self.picker_prev_states[picker_id] = self.picker_states[picker_id]
            self.set_picker_state(picker_id, "ACCEPT")

        elif msg.state == "ARRIVED" and self.task_state[msg.task_id] != "ARRIVED":
            # robot has arrived at the picker
            self.task_state[msg.task_id] = "ARRIVED"
            picker_id = self.task_picker[msg.task_id]
            self.picker_prev_states[picker_id] = self.picker_states[picker_id]
            self.set_picker_state(picker_id, "ARRIVED")

        elif msg.state == "LOADED" and self.task_state[msg.task_id] != "LOADED":
            # tray is loaded
            self.task_state[msg.task_id] = "LOADED"
            picker_id = self.task_picker[msg.task_id]
            if self.picker_states[picker_id] != "LOADED":
                self.picker_prev_states[picker_id] = self.picker_states[picker_id]
                self.set_picker_state(picker_id, "LOADED")

        elif msg.state == "STORAGE" and self.task_state[msg.task_id] != "STORAGE":
            # robot reached storage
            self.task_state[msg.task_id] = "STORAGE"

        elif msg.state == "DELIVERED" and self.task_state[msg.task_id] != "DELIVERED":
            # tray unloading is finished; remove task_id from task_robot
            self.task_state[msg.task_id] = "DELIVERED"
            self.task_robot.pop(msg.task_id)


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

    def write_log(self, field_vals):
        """write log row
        """
        row = {"id": self.log_count,
               "time":time.time(),
               "datetime": self.dt.strftime("%Y%m%d-%H%M%S"),
               }
        row.update(field_vals)
        if not self.log_file.closed:
            self.log_writer.writerow(row)
            self.log_count += 1

    def on_shutdown(self, ):
        """on shutdown cancel all goals
        """
        self.write_log({"action_type":"shutdown picker_state_monitor"})
        self.log_file.close()

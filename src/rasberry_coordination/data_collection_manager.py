#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on

@author: gpdas
"""

import rospy

import rasberry_coordination.srv

class DataCollectionManager(object):
    """
    """
    def __init__(self):
        self.latest_task_id = 10000
        self.assigned_robots = {}
        self.task_priority = {}
        self.task_status = {} # CREATED, ASSIGNED, ABANDONED, COMPLETED
#        self.task_stage = {} # go_to_dc_node, wait_at_dc_node, go_to_base
        self.task_type = {}

    def new_task_id(self): #TODO: could add a tack lock here for safety?
        """ Increment an internal counter to define new task_id's.

        :return: an unused task_id
        """
        self.latest_task_id = self.latest_task_id+1
        return self.latest_task_id

    def get_unassigned_tasks(self):
        """get the list of unassigned tasks
        """
        return [task_id for task_id in self.task_status if self.task_status[task_id] == "CREATED"]

    def get_assigned_tasks(self):
        """get the list of assigned tasks
        """
        return [task_id for task_id in self.task_status if self.task_status[task_id] == "ASSIGNED"]

    def get_abandoned_tasks(self):
        """get the list of abandoned tasks
        """
        return [task_id for task_id in self.task_status if self.task_status[task_id] == "ABANDONED"]

    def get_completed_tasks(self):
        """ get the list of completed tasks
        """
        return [task_id for task_id in self.task_status if self.task_status[task_id] == "COMPLETED"]

    def assign_robot(self, task_id, robot_id):
        """assign a robot to a task - only for logging
        """
        self.assigned_robots[task_id] = robot_id
        self.task_status[task_id] = "ASSIGNED"

    def set_task_finished(self, task_id):
        """set the task as completed
        """
        self.task_status[task_id] = "COMPLETED"

class NodeDataCollectionManager(DataCollectionManager):
    """
    """
    def __init__(self):
        """
        """
        super(NodeDataCollectionManager, self).__init__()
        self.task_nodes = {}
        self.add_node_data_collection_srv = rospy.Service("rasberry_coordination/task_manager/add_node_data_collection_task", rasberry_coordination.srv.AddNodeTask, self.add_new_task_cb)

    def add_new_task_cb(self, req):
        """
        """
        resp = rasberry_coordination.srv.AddNodeTaskResponse()
        task_id = self.new_task_id()
        self.task_nodes[task_id] = req.node_id
        self.task_status[task_id] = "CREATED"
        self.task_priority[task_id] = 1 # TODO: may be add this to the service def
        self.task_type[task_id] = "datacollection"

        rospy.loginfo("new node data collection task added at %s" %(req.node_id))
        resp.success = True
        resp.task_id = str(task_id)
        return resp


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
from thorvald_base.msg import BatteryArray as Battery
from rasberry_coordination.coordinator_tools import logmsg


class RobotManager(object):

    """Initialise class with callback details to apply to robots"""
    def __init__(self, callback_dict):
        self.cb = callback_dict
        self.dump_cb = Sub("robot_manager/dump", Str, self.dump_details)#, ID)
        self.robot_details = {}
    
    """Add Robot Details Objects"""
    def add_robots(self, robot_id_list):
        for robot_id in robot_id_list:
            self.add_robot(robot_id)
    def add_robot(self, robot_id):
        self.robot_details[robot_id] = RobotDetails(robot_id, self.cb)

    """Item retrieval objects (potentially slow so don't use unnecessarily"""
    def get_list(self, list_id):
        return [getattr(deets, list_id) for deets in self.robot_details]
    def get_item(self, robot_id, item):
        return getattr(self.robot_details[robot_id], item)

    """Dump Robot Details Callback"""
    def dump_details(self, msg):
        robot_id = msg.data
        if robot_id in self.robot_details:
            self.robot_details[robot_id].dump()
        else:
            for robot_id in self.robot_details:
                self.robot_details[robot_id].dump()
        
class RobotDetails(object):
    def __init__(self, ID, cb):
        
        """Detail whether the robot is moving"""
        self.idle = False
        self.interruptable = False
        
        """Meta Management"""
        self.robot_id = ID
        self.disconnect_when_idle = False
        self.registered = True
        
        """Localisation Details"""
        self.previous_node = None
        self.current_node = None
        self.closest_node = None
        # self.current_node_sub = Sub(ID+"/current_node", Str, cb['current'], ID)
        # self.closest_node_sub = Sub(ID+"/closest_node", Str, cb['closest'], ID)
        
        """Health Monitoring"""
        self.healthy = True
        self.battery_voltage = 55.0
        # self.battery_data_sub = Sub(ID+"/battery_data", Battery, cb['battery'], ID)

        """Task Details"""
        self.robot_state = None
        self.tray_loaded = False
        self.start_time = Now()
        self.task_id = None
        self.max_task_priority = 255
        self.admissible_tasks = []
        
        """Goal definitions"""
        self.current_storage = None
        self.base_station = None
        self.wait_node = None
        
        """Route details"""
        self.routes = []
        self.route_dists = []
        self.route_edges = []
        self.route_fragments = []
        
        
    """Dump all values for robot into file"""
    def dump(self):
        with open(self.robot_id+'---'+str(Now())+'.txt', 'w') as writer:
            writer.write("%s" % (self.robot_id))
            for attr in dir(self):
                writer.write("%s = %r" % (attr, getattr(self, attr)))


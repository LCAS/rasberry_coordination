#!/usr/bin/env python

import sys

import rospy

import rasberry_coordination.coordinator
import rasberry_coordination.picker_state_monitor

import rasberry_des.config_utils

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print """not enough arguments are passed. correct usage is
        rosrun rasberry_coordination simple_task_executor_node.py config_file.yaml
        """
    else:
        config_file = sys.argv[1]
        config_data = rasberry_des.config_utils.get_config_data(config_file)
        config_keys = rasberry_des.config_utils.get_config_keys(config_file)

        # check for required parameters
        req_params = ["base_station_nodes", "local_storage_node", "charging_station_node", "robot_ids", "max_task_priorities"]

        for key in config_keys:
            if key in req_params:
                req_params.remove(key)

        if len(req_params) != 0:
            raise Exception("not all required keys are set in the config file")
        elif config_data["robot_ids"].__class__ != list:
            raise Exception("robot_ids should be a list in the config file")
        elif len(config_data["robot_ids"]) == 0:
            raise Exception("robot_ids should not be an empty list in the config file")

        _base_stations = config_data["base_station_nodes"] # list of local storage nodes
        local_storage = config_data["local_storage_node"] # list of local storage nodes
        charging_node = config_data["charging_station_node"]
        robot_ids = config_data["robot_ids"]
        _max_task_priorities = config_data["max_task_priorities"]
        virtual_pickers = []
        if "virtual_pickers" in config_data:
            virtual_pickers = config_data["virtual_pickers"]

        if _base_stations.__class__ == str:
            if len(robot_ids) > 1:
                raise Exception("Not enough base stations (1) for %d robots!!!" %(len(robot_ids)))
            base_stations = {robot_id:_base_stations for robot_id in robot_ids}
        elif _base_stations.__class__ == list:
            if len(_base_stations) != len(robot_ids):
                raise Exception("Not enough base stations (%d) for %d robots!!!" %(len(_base_stations), len(robot_ids)))
            base_stations = {robot_ids[i]:_base_stations[i] for i in range(len(robot_ids))}
        elif _max_task_priorities.__class__ == list:
            if len(_max_task_priorities) != len(robot_ids):
                raise Exception("Not enough min task priorities defined (%d) for %d robots!!!" %(len(_max_task_priorities), len(robot_ids)))
            max_task_priorities = {robot_ids[i]:_max_task_priorities[i] for i in range(len(robot_ids))}

        rospy.init_node('simple_task_coordinator', anonymous=False)

        # initialise the coordinator and internally all robots
        coordinator = rasberry_coordination.coordinator.Coordinator(local_storage=local_storage,
                                                          charging_node=charging_node,
                                                          base_stations=base_stations,
                                                          picker_ids=[],
                                                          robot_ids=robot_ids,
                                                          max_task_priorities=max_task_priorities)

        rospy.on_shutdown(coordinator.on_shutdown)
        rospy.sleep(1) # give a second to let everything settle

        # picker_monitor after coordinator
        picker_monitor = rasberry_coordination.picker_state_monitor.PickerStateMonitor(virtual_pickers)

        coordinator.run()

        rospy.spin()

#!/usr/bin/env python

import sys

import rospy

import rasberry_coordination.rasberry_coordinator
import rasberry_coordination.picker_state_monitor

import rasberry_des.config_utils


def validate_field(file, config, key, datatype, mandatory=False):
    """
    Validate the data type for a given field in the config against a given data type
    """
    if key not in config:
        if mandatory:
            raise Exception("Field '%s' is mandatory but missing in %s" % (key, file))
        else:
            return

    typ = config[key].__class__
    acc = str(datatype).replace("<type '", '').replace("'>", "")
    if typ not in datatype:
        raise Exception("Type error for field '%s' in %s, expected any of %s, received %s" % (key, file, acc, typ))


def validate_types(file, config):
    """
    Validate the data type for each known field within the map config following the version 1.1.1 template
    """

    # Meta Fields
    validate_field(file, config, mandatory=True, key='version', datatype=[str])

    # Tasks Fields
    validate_field(file, config, mandatory=True, key='active_tasks', datatype=[list, str])
    validate_field(file, config, mandatory=True, key='max_load_duration', datatype=[int])
    validate_field(file, config, mandatory=True, key='max_unload_duration', datatype=[int])

    # Topology Fields
    validate_field(file, config, mandatory=True, key='base_station_nodes_pool', datatype=[list, str])
    validate_field(file, config, mandatory=True, key='wait_nodes_pool', datatype=[list, str])
    validate_field(file, config, mandatory=True, key='local_storage_nodes', datatype=[list, str])
    validate_field(file, config, mandatory=True, key='use_cold_storage', datatype=[bool])
    validate_field(file, config, mandatory=False, key='cold_storage_node', datatype=[list, str])
    validate_field(file, config, mandatory=True, key='charging_station_nodes', datatype=[list, str])
    # ^ can swap cold_storage_node mandatory boolean to reference config['use_cold_storage']

    # Robot Fields
    validate_field(file, config, mandatory=True, key='admissible_robot_ids', datatype=[list, str])

    # Routing Fields
    validate_field(file, config, mandatory=True, key='planning_type', datatype=[str])

    # Picker Fields
    validate_field(file, config, mandatory=True, key='picker_ids', datatype=[list, str])
    validate_field(file, config, mandatory=False, key='virtual_picker_ids', datatype=[list, str])

    # Initialisation Fields
    validate_field(file, config, mandatory=True, key='spawn_list', datatype=[list])
    for robot in config['spawn_list']:
        if 'default' in robot:
            continue
        validate_field(file, robot, mandatory=True, key='robot_id', datatype=[str])
        validate_field(file, robot, mandatory=False, key='max_task_priority', datatype=[int])
        validate_field(file, robot, mandatory=True, key='base_station_node', datatype=[str])
        validate_field(file, robot, mandatory=False, key='wait_node', datatype=[str])


if __name__ == '__main__':

    if len(sys.argv) < 2:
        usage = "rosrun rasberry_coordination simple_task_executor_node.py config_file.yaml"
        print("Not enough arguments passed. Correct usage is:\n\t"+usage)
        exit()

    config_file = sys.argv[1]
    config_data = rasberry_des.config_utils.get_config_data(config_file)
    config_keys = rasberry_des.config_utils.get_config_keys(config_file)

    # configuration file validation
    template_location = "raspberry_coordination/config/map_config_template.yaml"
    if "version" not in config_data:
        raise Exception('\033[92m'+"Config outdated, update to template: "+template_location+'\033[0m')
    if config_data["version"] != "1.1.1":
        print("Config version: "+config_data["version"])
        raise Exception('\033[92m'+"Config outdated, update to template: "+template_location+'\033[0m')

    # Ensure all required fields are filled with the correct data types
    validate_types(config_file, config_data)

    # Tasks
    active_tasks = config_data["active_tasks"]
    max_load_duration = config_data["max_load_duration"]
    max_unload_duration = config_data["max_unload_duration"]

    # Topology
    base_station_nodes_pool = config_data["base_station_nodes_pool"]
    wait_nodes_pool = config_data["wait_nodes_pool"]
    local_storage_nodes = config_data["local_storage_nodes"]
    use_cold_storage = config_data['use_cold_storage']
    cold_storage_node = None
    if use_cold_storage:
        if "cold_storage_node" not in config_data:
            raise Exception("Cold storage node must be given if use_cold_storage is True.")
        cold_storage_node = config_data["cold_storage_node"]
    charging_station_nodes = config_data["charging_station_nodes"]

    # Routing
    planning_type = config_data["planning_type"]

    # Robots
    admissible_robot_ids = config_data["admissible_robot_ids"]

    # Pickers
    picker_ids = config_data["picker_ids"]
    virtual_picker_ids = []
    if "virtual_picker_ids" in config_data:
        virtual_picker_ids = config_data["virtual_picker_ids"]

    # Robot Initialisation
    robot_ids = []
    base_stations = {}
    wait_nodes = {}
    max_task_priorities = {}
    for robot in config_data['spawn_list']:
        if 'default' in robot:
            continue
        # if wait node omitted copy base station, if "none" leave empty
        if "wait_node" not in robot:
            robot['wait_node'] = robot['base_station_node']
        elif robot['wait_node'].lower() == "none":
            robot['wait_node'] = None

        # if max task priority omitted default at 255
        if "max_task_priority" not in robot:
            robot['max_task_priority'] = 255

        robot_ids.append(robot['robot_id'])
        max_task_priorities[robot['robot_id']] = robot['max_task_priority']
        base_stations[robot['robot_id']] = robot['base_station_node']
        wait_nodes[robot['robot_id']] = robot['wait_node']

    # initialise ROSNode
    rospy.init_node('simple_task_coordinator', anonymous=False)

    use_restrictions = rospy.get_param("~use_restrictions", False)

    # initialise the coordinator and internally all robots
    coordinator = rasberry_coordination.rasberry_coordinator.RasberryCoordinator(
                                                    robot_ids=robot_ids,
                                                    picker_ids=picker_ids, #TODO: rename to admissible_picker_ids
                                                    virtual_picker_ids=virtual_picker_ids,
                                                    local_storages=local_storage_nodes,
                                                    cold_storage=cold_storage_node,
                                                    use_cold_storage=use_cold_storage, #TODO: remove this and query is cold_storage is None
                                                    base_stations=base_stations, #TODO: remove this and initialise with robot details dicts
                                                    wait_nodes=wait_nodes,
                                                    max_task_priorities=max_task_priorities,
                                                    admissible_robot_ids=admissible_robot_ids,
                                                    active_tasks=active_tasks,
                                                    base_station_nodes_pool=base_station_nodes_pool,
                                                    wait_nodes_pool=wait_nodes_pool,
                                                    charging_station_nodes=charging_station_nodes,
                                                    use_restrictions=use_restrictions,
                                                    max_load_duration=rospy.Duration(secs=max_load_duration),
                                                    max_unload_duration=rospy.Duration(secs=max_unload_duration),
                                                    ns="rasberry_coordination")

    rospy.on_shutdown(coordinator.on_shutdown)
    rospy.sleep(1)  # give a second to let everything settle

    # Run the coordinator
    coordinator.run(planning_type=planning_type)

    rospy.spin() #TODO: is this necessary?

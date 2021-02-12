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
    Validate the data type for each known field within the map config following the version 1.1.0 template
    """

    # Meta Fields
    validate_field(file, config, mandatory=True, key='version', datatype=[str])
    validate_field(file, config, mandatory=True, key='use_sim', datatype=[bool])

    # Topology Fields
    for node in config['special_nodes']:
        if 'default' in node:
            continue
        validate_field(file, node, mandatory=True,  key='id', datatype=[str])
        validate_field(file, node, mandatory=True,  key='descriptors', datatype=[list, str])

    # Routing Fields
    validate_field(file, config, mandatory=True, key='planning_type', datatype=[str])

    # Robot Fields
    validate_field(file, config, mandatory=True, key='low_battery_voltage', datatype=[float])

    # Agent Initialisation
    for agent in config['agent_list']:
        if 'default' in agent:
            continue
        validate_field(file, agent, mandatory=True,  key='agent_id', datatype=[str])
        validate_field(file, agent, mandatory=True,  key='agent_type', datatype=[str])
        validate_field(file, agent, mandatory=True,  key='interface_type', datatype=[str])
        validate_field(file, agent, mandatory=False, key='idle_task_default', datatype=[str])
        validate_field(file, agent, mandatory=False, key='new_task_default', datatype=[str])
        validate_field(file, agent, mandatory=False, key='initial_location', datatype=[str])


if __name__ == '__main__':

    if len(sys.argv) < 2:
        usage = "rosrun rasberry_coordination simple_task_executor_node.py config_file.yaml"
        print("Not enough arguments passed. Correct usage is:\n\t"+usage)
        exit()

    config_file = sys.argv[1]
    config_data = rasberry_des.config_utils.get_config_data(config_file)
    config_keys = rasberry_des.config_utils.get_config_keys(config_file)

    # configuration file validation
    VERSION = "1.2.0"
    template_location = "raspberry_coordination/config/map_config_template_%s.yaml" % VERSION
    if "version" not in config_data:
        raise Exception('\033[92m'+"Config outdated, update following: %s\033[0m" % template_location)
    if config_data["version"] != VERSION:
        print(config_file)
        print("Config version: "+config_data["version"])
        raise Exception('\033[92m'+"Config outdated, update following: %s\033[0m" % template_location)

    # Ensure all required fields are filled with the correct data types
    validate_types(config_file, config_data)

    # TOPOLOGY (Node Descriptors)
    base_station_nodes_pool = set()
    wait_nodes_pool = set()
    charging_station_pool = set()
    for node in config_data['special_nodes']:
        if 'default' in node:
            continue
        if 'base_station' in node['descriptors']:
            base_station_nodes_pool.add(node['id'])
        if 'wait_node' in node['descriptors']:
            wait_nodes_pool.add(node['id'])
        if 'charging_station' in node['descriptors']:
            charging_station_pool.add(node['id'])

    # ROUTING
    planning_type = config_data["planning_type"]

    # INITIALISATION
    for agent in config_data['agent_list']:
        if 'default' in agent:
            continue

        # if task_definitions omitted default apply 'default'
        if "idle_task_definition" not in agent:
            agent['idle_task_default'] = 'default'
        if "new_task_definition" not in agent:
            agent['new_task_default'] = 'default'

        # if initial_location omitted default to None
        if "initial_location" not in agent:
            agent['initial_location'] = None

    # Start ROSNode
    rospy.init_node('simple_task_coordinator', anonymous=False)

    # initialise the coordinator and internally all robots
    coordinator = rasberry_coordination.rasberry_coordinator.RasberryCoordinator(
        agent_list=config_data['agent_list'],
        base_station_nodes_pool=base_station_nodes_pool,
        wait_nodes_pool=wait_nodes_pool,
        planning_type=planning_type,
        ns="rasberry_coordination")

    rospy.on_shutdown(coordinator.on_shutdown)
    rospy.sleep(1)  # give a second to let everything settle

    # Run the coordinator
    coordinator.run()

    # rospy.spin() #TODO: is this necessary?

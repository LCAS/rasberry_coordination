#!/usr/bin/env python

import sys
import rospy
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
    Validate the data type for each known field within the map config following the version 1.2.2 template
    """

    # Meta Fields
    validate_field(file, config, mandatory=True, key='version', datatype=[str])
    validate_field(file, config, mandatory=True, key='use_sim', datatype=[bool])
    validate_field(file, config, mandatory=True, key='use_restrictions', datatype=[bool])

    # Topology Fields
    for node in config['special_nodes']:
        if 'default' in node: continue
        validate_field(file, node, mandatory=True,  key='id', datatype=[str])
        validate_field(file, node, mandatory=True,  key='descriptors', datatype=[list, str])

    # Routing Fields
    validate_field(file, config, mandatory=True, key='planning_type', datatype=[str])
    validate_field(file, config, mandatory=True, key='heterogeneous_map', datatype=[bool])

    # Setup Definition
    for setup in [s['setup'] for s in config['agent_setups']]:
        validate_field(file, setup, mandatory=False,  key='tasks', datatype=[list])
        for task in setup['tasks']:
            validate_field(file, task, mandatory=True,  key='module', datatype=[str])
            validate_field(file, task, mandatory=True,  key='role', datatype=[str])
        validate_field(file, setup, mandatory=True,   key='has_presence', datatype=[bool])
        validate_field(file, setup, mandatory=False,  key='properties', datatype=[dict])

    # Agent Initialisation
    for agent in config['agent_list']:
        validate_field(file, agent, mandatory=True,  key='agent_id', datatype=[str])
        validate_field(file, agent, mandatory=True,  key='setup', datatype=[dict])
        validate_field(file, agent, mandatory=False,  key='initial_location', datatype=[str])

    # Agent Initialisation
    for task in config['active_tasks']:
        validate_field(file, task, mandatory=True,  key='module', datatype=[str])
        validate_field(file, agent, mandatory=False,  key='properties', datatype=[dict])


if __name__ == '__main__':

    if len(sys.argv) < 2:
        usage = "rosrun rasberry_coordination simple_task_executor_node.py config_file.yaml"
        print("Not enough arguments passed. Correct usage is:\n\t"+usage)
        exit()

    config_file = sys.argv[1]
    config_data = rasberry_des.config_utils.get_config_data(config_file)
    config_keys = rasberry_des.config_utils.get_config_keys(config_file)

    """ Configuration File Validation """
    VERSION = "1.2.3"
    template_location = "raspberry_coordination/config/map_config_template_%s.yaml" % VERSION
    if "version" not in config_data:
        raise Exception('\033[92m'+"Config outdated, update following: %s\033[0m" % template_location)
    if config_data["version"] != VERSION:
        print(config_file)
        print("Config version: "+config_data["version"])
        raise Exception('\033[92m'+"Config outdated, update following: %s\033[0m" % template_location)

    # Ensure all required fields are filled with the correct data types
    validate_types(config_file, config_data)

    """ TOPOLOGY (Node Descriptors) """
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

    """ ROUTING """
    planning_format = dict()
    planning_format['planning_type'] = config_data["planning_type"]
    planning_format['heterogeneous_map'] = config_data["heterogeneous_map"]

    """ INITIALISATION """
    # load default values for non-mandatory fields
    for agent in config_data['agent_list']:
        if 'default' in agent:
            continue

        # if no active tasks given, inform user
        if len(agent['setup']['tasks']) < 1:
            print("Agent %s connected with 0 available tasks." % agent['agent_id'])

        # Default physical presence to True
        if 'has_presence' not in agent['setup']:
            agent['setup']['has_presence'] = True

        # Default properties to empty dict
        if 'properties' not in agent['setup']:
            agent['setup']['properties'] = dict().copy()

        # if initial_location omitted default to None
        if "initial_location" not in agent:
            agent['initial_location'] = None


    """ ACTIVE TASKS """
    modules_to_load = set([t['module'] for t in config_data['active_tasks']])

    # Start ROSNode
    rospy.init_node('abstract_task_coordinator', anonymous=False)

    # Initialise task manager to store all task and stage definitions in single objects for later access
    import rasberry_coordination.task_management.__init__ as task_init
    task_init.set_properties(config_data['active_tasks'])
    task_init.def_tasks(list(modules_to_load))


    """ COORDINATOR """
    import rasberry_coordination.rasberry_coordinator
    coordinator = rasberry_coordination.rasberry_coordinator.RasberryCoordinator(
        agent_list=config_data['agent_list'],
        base_station_nodes_pool=base_station_nodes_pool,
        wait_nodes_pool=wait_nodes_pool,
        planning_format=planning_format,
        ns="rasberry_coordination",
        special_nodes=config_data['special_nodes'][1:])  # We are not certain of ordering?

    rospy.on_shutdown(coordinator.on_shutdown)
    rospy.sleep(1)  # give a second to let everything settle

    # Run the coordinator
    coordinator.run()

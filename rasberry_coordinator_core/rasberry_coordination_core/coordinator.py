#!/usr/bin/env python3

import os, sys
import yaml
from pprint import pprint

import rospy
import rospkg


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
    """ Validate data types for each field in config, following template version 1.2.4
    """

    # Meta Fields
    validate_field(file, config, mandatory=True, key='version', datatype=[str])

    # Agent Initialisation
    if 'default_agents' in config:
        for agent in config['default_agents']:
            validate_field(file, agent, mandatory=True,  key='agent_id', datatype=[str])
            validate_field(file, agent, mandatory=True,  key='setup', datatype=[str])
            validate_field(file, agent, mandatory=False, key='local_properties', datatype=[dict])

    # Topology Fields
    if 'special_nodes' in config:
        for node in config['special_nodes']:
            validate_field(file, node, mandatory=True,  key='id', datatype=[str])
            validate_field(file, node, mandatory=True,  key='descriptors', datatype=[list, str])

    # Routing Fields
    validate_field(file, config, mandatory=True, key='planning_format', datatype=[dict])
    validate_field(file, config['planning_format'], mandatory=True, key='planning_type', datatype=[str])
    validate_field(file, config['planning_format'], mandatory=True, key='heterogeneous_map', datatype=[bool])

    # Module Initialisation
    for module in config['active_modules']:
        validate_field(file, module, mandatory=True,  key='name', datatype=[str])
        validate_field(file, module, mandatory=False, key='properties', datatype=[dict])


def main(args=None):
    rclpy.init(args=args)

    print("Recommended to set 'force_color_prompt=yes' on line 46 of .bashrc.")
    if len(sys.argv) < 2:
        usage = "rosrun rasberry_coordination abstract_task_executor_node.py $YAML_CONFIG_FILE"
        print("Not enough arguments passed. Correct usage is:\n\t"+usage)
        exit()


    # Load yaml
    config_file = sys.argv[1]
    with open(config_file) as f:
        confid_data = yaml.safe_load(f)

    # Configuration File Validation
    VERSION = "1.2.4"
    template_location = "raspberry_coordination/config/coordinator_templates/template_%s.yaml" % VERSION
    if "version" not in config_data:
        raise Exception('\033[92m'+"Config outdated, update following: %s\033[0m" % template_location)
    if config_data["version"] != VERSION:
        print(config_file)
        print("Config version: "+config_data["version"])
        raise Exception('\033[92m'+"Config outdated, update following: %s\033[0m" % template_location)
    validate_types(config_file, config_data)


    # Agent Initialisation
    config_data['default_agents'] = config_data['default_agents'] or dict()
    config_data['agents'] = []
    for agent in config_data['default_agents']:
        #filepath = "%s/config/setup/" % rospkg.RosPack().get_path('rasberry_coordination')
        filepath = os.getenv('AGENT_SETUP_CONFIG', None)
        setup_file = filepath + "%s.yaml" % agent['setup']
        setup_data = rasberry_des.config_utils.get_config_data(setup_file)
        for m in setup_data['modules']:
            m['details'] = {d.keys()[0]:d.values()[0] for d in m['details']} if 'details' in m else dict()
        agent['local_properties'] = agent['local_properties'] if 'local_properties' in agent else dict()
        config_data['agents'] += [{'agent_id': agent['agent_id'], 'local_properties':agent['local_properties'], 'modules':setup_data['modules']}]

    # Start ROS Node and create global reference to log from any file
    class CoordinatorNodeHandler(Node):
        def __init__(self):
            super().__init__('coordinator')
            global GlobalLogger
            GlobalLogger = self.get_logger()
            global ROS2Node
            ROS2Node = self
    GlobalNode = CoordinatorNodeHandler()


    # Initialise modules for task manager
    import rasberry_coordination.task_management.__init__ as task_init
    task_init.set_properties(config_data['active_modules'])
    task_init.load_custom_modules(list(set([t['name'] for t in config_data['active_modules']])))


    # Create Coordinator
    import rasberry_coordination.rasberry_coordinator
    coordinator = rasberry_coordination.rasberry_coordinator.RasberryCoordinator(
        default_agents=config_data['agents'],
        planning_format=config_data['planning_format'],
        special_nodes=config_data['special_nodes'])

    rospy.on_shutdown(coordinator.on_shutdown)
    rospy.sleep(1)  # give a second to let everything settle


    # Launch Inspector
    from rasberry_coordination.root_inspector import RootInspector
    RootInspector(topic='~root_inspector', root=coordinator)


    # Run the Coordinator
    coordinator.run()


if __name__ == '__main__':
    main()



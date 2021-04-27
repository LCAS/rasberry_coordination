global TaskDef, StageDef, InterfaceDef


def def_tasks(clean_task_list):
    from rasberry_coordination.coordinator_tools import logmsg
    print(" ")

    task_list = ['rasberry_coordination.task_management.custom_tasks.%s' % task for task in clean_task_list]
    if 'rasberry_coordination.task_management.base' not in task_list:
        task_list.insert(0, 'rasberry_coordination.task_management.base')

    logmsg(category="setup", msg="Collecting Interface, Task, and Stage Definitions for modules: ")
    [logmsg(category="setup", msg="    - %s" % module) for module in clean_task_list]

    # For each task, import the TaskDef, StageDef, and InterfaceDef to the respective dictionaries
    task_definitions = dict()
    stage_definitions = dict()
    interface_definitions = dict()
    for task in task_list:
        task_definitions[task] = __import__('%s' % task, globals(), locals(), ['TaskDef'], -1).TaskDef
        stage_definitions[task] = __import__('%s' % task, globals(), locals(), ['StageDef'], -1).StageDef
        interface_definitions[task] = __import__('%s' % task, globals(), locals(), ['InterfaceDef'], -1).InterfaceDef

    # Make these accessible through import
    global TaskDef, StageDef, InterfaceDef
    TaskDef = type('TaskDef', tuple(task_definitions.values()), dict())
    StageDef = type('StageDef', tuple(stage_definitions.values()), dict())
    InterfaceDef = type('InterfaceDef', tuple(interface_definitions.values()), dict())

    # logmsg(msg="Interfaces: ")
    # [logmsg(msg="    - %s" % interface) for interface in dir(InterfaceDef) if not d.startswith('__')]
    # logmsg(msg="Tasks: ")
    # [logmsg(msg="    - %s" % task) for task in dir(TaskDef) if not d.startswith('__')]
    # logmsg(msg="Stages: ")
    # [logmsg(msg="    - %s" % stage) for stage in dir(StageDef) if not d.startswith('__')]

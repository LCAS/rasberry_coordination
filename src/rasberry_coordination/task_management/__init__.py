# from base import StageDef as base_stagedef, TaskDef as base_taskdef
# Custom Tasks
# from tray import StageDef as tray_stagedef, TaskDef as tray_taskdef
# from uv   import StageDef as uv_stagedef,   TaskDef as uv_taskdef

global TaskDef, StageDef


def def_tasks(clean_task_list):
    task_list = ['rasberry_coordination.task_management.custom_tasks.%s' % task for task in clean_task_list]
    if 'rasberry_coordination.task_management.base' not in task_list:
        task_list.insert(0, 'rasberry_coordination.task_management.base')

    print("\n\nCollecting Stage and Task Definitions for each in: %s\n" % clean_task_list)

    # For each task, import the TaskDef and StageDef to the respective dictionaries
    task_definitions = dict()
    stage_definitions = dict()
    for task in task_list:
        # print("\nTasks and Stages defined within: %s" % task)
        # task_definitions[task]  = __import__('transportation.%s'%task, globals(), locals(), [ 'TaskDef'], -1).TaskDef
        task_definitions[task] = __import__('%s' % task, globals(), locals(), ['TaskDef'], -1).TaskDef
        stage_definitions[task] = __import__('%s' % task, globals(), locals(), ['StageDef'], -1).StageDef
        # print("     Tasks: %s" % [d for d in dir(task_definitions[task]) if not d.startswith('__')])
        # print("    Stages: %s" % [d for d in dir(stage_definitions[task]) if not d.startswith('__')])

    # Make these accessible through import
    # print("-------\n\nCompiling to single collection:")
    global TaskDef, StageDef
    TaskDef = type('TaskDef', tuple(task_definitions.values()), dict())
    StageDef = type('StageDef', tuple(stage_definitions.values()), dict())
    # print(" Tasks: %s" % [d for d in dir(TaskDef) if not d.startswith('__')])
    # print("Stages: %s" % [d for d in dir(StageDef) if not d.startswith('__')])
    # print("-------\n\n")


# if 'TaskDef' in globals() and 'StageDef' in globals():
#     print(" Tasks: %s" % [d for d in dir(TaskDef) if not d.startswith('__')])
#     print("Stages: %s" % [d for d in dir(StageDef) if not d.startswith('__')])
# elif 'TaskDef' in locals() and 'StageDef' in locals():
#     print(" Tasks: %s" % [d for d in dir(TaskDef) if not d.startswith('__')])
#     print("Stages: %s" % [d for d in dir(StageDef) if not d.startswith('__')])
# else:
#     print("nope")
# print("\n\n---------------")
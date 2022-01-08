# Agents

## Agent Properties
Along with saving the initial launch config properties, the agent also initialises four other key collections of details. 

In the creation of tasks, each task is initialised with a task_id, this is a unique combination of the agent's identifier, and the **total_tasks** that agent has initiated. The active task is stored in the **task** property, and the remaining pending tasks are stored in the **task_buffer**.
```yaml
total_tasks: used for generating a unique task id (`agent_id`_`total_tasks`)
task: holds the active task module
task_buffer: holds the list of tasks which have been initialised but not begun
```

At any point in the processing of a task, a stage can be interrupted by an outside interference by adding details of the interruption to the **interruption** property.
```yaml
interruption: tuple containing the type of interruption, and the task it effects 
```

If the task progression is paused such as with a request from TOC, or if the agent is being removed from the system, then its ability to accept tasks is removed with the **registration** tag set to false.
```yaml
registration: bool for if the agent is actively accepting new tasks
```

On callback from the first map message, the **navigation property** is populated with the following:
```yaml
navigation: tools for route generation

  'tmap': yaml safe_load of raw msg data
  'tmap_node_list': list of all node names
  
  'tmap_available': copy of new tmap for fresh copies to be made
  'available_route_search': reference to perform a route search with the available tmap
```

Each **module encapsulator** is used to collect all module-related information in a single tidy place so referencing can be simplified. Once the encapsulator is initialised, the agent is given the role initialisation task.
```yaml
modules: Dictionary of modules referenced by module name

  .agent: identify of the agent
  .name: name of the module
  .role: role of the agent within the module

  .interface_name: name of the interface of interest in the form of `name`_`role`

  .interface: initialisation of the interface for the role
  .properties: collection of properties related ot the module

  .init_task_name: name of the initialisation task in the form of `interface_name`_init
  .idle_task_name: name of the module's idle task in the form of `interface_name`_idle
```

The **location encapsulator** is initialised with a property for `has_presence` which identifies if the navigation system should allow other agents to use the route and `initial_location` which is used to preload a location for virtual agents or static agents such as storage locations. The encapsulator offers three functionality of note:

```yaml
location: Location object to encapsulate all current-localisation tools
     (1): Record
     (2): Report
     (3): Simulate
```

On disabling localisation, a new location can be passed in which will act as the basic reference for the `current_node`.


## Shortcuts
To simplify the codebase from repeated calls to child components, some dunder functionality has been overridden. `__call__` has been overridden to identify the active stage in the active task, and `__*item__` have been overridden to get and set a given property in the active task.

For example, `agent().route_required` is used to identify if the active stage requires a route, and `agent['id']` is used to get the task_id for the active task.

```python
def __call__(A, index=0): 
    return A.task.stage_list[index] if len(A.task.stage_list) > index else None

def __getitem__(A, key):
    return A.task[key] if A.task else None

def __setitem__(A, key, val):
    A.task[key] = val
```

To simplify the logging, the `__repr__` dunder has been overridden to return the agent's id cleanly, replacing the full class details with a basic form.

```python
def __repr__(self):
    return "%s(%s)" % (self.get_class(), self.agent_id)
def get_class(self):
    return str(self.__class__).replace("<class 'rasberry_coordination.agent_management.agent_manager.", "").replace("'>", "")
```

## Etc.
Further functionality attributed to the Agent class is discussed in [Ref][Main-Loop.md]

## Launch Config
When agents are launched, certain parameters broken down over two files. These properties are categorised into 5 groups, in the first file, basic properties are detailed, for the agents unique identifier, followed by any properties which are specific to this agent such as their static location, or a physical property such as their current load.

In the second file, are the modules associated to the robot, the navigation parameters, any module-specific properties, and the description of the robot to display in RViZ. 

In the modules list, the modules to run interfaces for on the agent are listed, paired with the agent's role. When loaded, the module properties are saved into the rosparam server on the coordinator, so can be modified at anytime.

### Examples
**Short Robot** used as a transportation courier
```yaml
agent_id: thorvald_002
local_properties:
    load: 0
```
```yaml
modules:
  - name: base
    role: robot
  - name: health_monitoring
    role: robot
  - name: transportation
    role: courier
navigation_properties:
    has_presence: true
    restrictions: inter-row
module_properties:
    max_load: 2
visualisation_properties:
    structure_type: short
    rviz_model: short_robot
```

**Storage Manager** tasked with unloading punnets from a courier:
```yaml
agent_id: storage01
local_properties:
    initial_location: WayPoint67
```
```yaml
modules:
  - name: base
    role: human
  - name: transportation
    role: field_storage
navigation_properties:
    has_presence: false
visualisation_properties:
    rviz_model: human
```
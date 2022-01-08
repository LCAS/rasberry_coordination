#Tasks, Stages, and Interfaces

##Stages Overview
Stages are a fundamental axiom in the ATM Coordinator. 
They serve to offer encapsulation by dividing tasks into common functionality.


###Types of Stages:
**Root Stage**
- the single root of the stage tree, this defines all common functions, stages do not extend beyond this, if stages are believed to require further functions, consider breaking them down further into more stages.

**Core Stages**
- Stages which have abstract functionality to tasks, (e.g. `Navigation`, `Assignment`, `StartTask`, `Idle`)

**Implemented Stages**
- Extends the functionality of Core Stages to offer specialised abstract functionalities (e.g. `NavigateToNode`, `AssignAgent`).

**Context Stages**Tasks, Stages, and Interfaces
- Stages which implement task-specific functionality (e.g. `NavigateToBaseNode`, `TriggerUVLight`).


###Stage Methods:
The root stage is named the `StageBase`, it contains the majority of standard functionality for all stages. Is consists of eight methods, of which three are never overridden, and three are always called back to this base class from child stages. The four core methods are described here, with the remaining four described below:

```python
def __init__(self, agent):
    """Class initialisation for populating default values"""
    #inheritees must call super

def _start(self):
    """Stage start, called when this is the active stage."""
    #inheritees must call super
    
def _query(self):
    """Used to define the criteria which must be met for the stage to be completed"""
    
def _end(self):
    """Used to set any fields as the stage is about to be removed"""
    #inheritees must call super
```

The last four methods can be considered the core functionality for a Stage, these are called as the Stage processes through `ITAASQE`. The stage is initialised to define any initial properties with `__init__`, and added with the rest of its task to the task buffer, the `__init__` method must call first, the Stages `super().__init__` to ensure any other properties up the hierarchy are set. No further activity happens on the stage until after the stage's task passes the head of the task buffer to become the active task. 

Once the stage becomes the head of the task stage list, the `_start` method is called to initialise any properties on the agent such as action functionality (`AssignAgent`), route request details (`Navigation`), topic callback defaults (`WaitForMap`), send messages to an associated agent (`NotifyTrigger`) or just to set any references to make the rest of the stage's code tidier. The `_start` method is called whenever the stage becomes the active stage, which can happen multiple times, for example if the Stage is paused, so this may need to be considered when adding its functionality. This method must also call its super, as there are properties in the base class which must always be set, it is recommended that `super()._start` be called first within the method but this is not required.

The `_query` method is called directly after the `_start` method, and is used to specify a series of conditions which any or all much mme met for the stage to be marked as completed. This stage by standard, always follows the same structure (see below), in which the series of conditions are laid out in a list, and are collapsed into a single boolean value which is sent to the `flag` method. There is no explicit requirement that this is laid out in this format, however at scale, this has been found to be effective at keeping the codebase consistent and tidy. The only explicit requirement is the call to `flag`. This function is called repeatedly once the stage is active, stopping only once the completion flag is set to true. Thus, it is best to limit the processing of this method by simplifying complexity as much as possible, including saving to the Stage any unchanging references during the `_start` method.
```python
def _query(self):
    success_conditions = [conditionA == valueA,
                          conditionB == valueB,
                          conditionC == valueC]
    self.flag(any(success_conditions))
```

The final method, `_end`, is called once `flag` is called with True. This method is used to tidy up any loose ends before the stage is completed, this can include saving action responses (`AssignAgent`), attaching new tasks (`FindRows`), triggering recovery re-planning (`Navigation`), and formatting RViZ markers (`Pause`), but generally the most common uses are saving stage properties to the task and notifying associated agents of stage progression. This method must make a call to `super()._end`, however there is no explicit preference to whether this should be done before or after the method's local code.

To assist with visualisation and tidiness, the remaining methods of `get_class`, `__repr__`, `suspend` and `flag` were included.

```python
def get_class(self):
    """Cleaned class name for explicit class-type queries"""
    #never overridden

def __repr__(self):
    """Simplified representation of class for clean informative logging"""

def suspend(self):
    """Collection function for all actions to call when a stage is suspended"""
    #never overridden

def flag(self, completion):
    """Collection function when a stage is to be flagged as completed"""
    #never overridden
```

The `get_class` method was included to simplify the information presentation in logging, where the full class definition is replaced with just the module name wih the class name, (e.g. below). This simplification helps to improve the readability of the terminal output, thus improve the speed of resolving issues.

```python
# Converted from:
"<class 'rasberry_coordination.task_management.custom_tasks.transportation.AssignFieldCourier'>"
# to:
"transportation.AssignFieldCourier"


# Converted from:
"<class 'rasberry_coordination.task_management.base.NavigateToBaseNode'>"
# to:
"base.NavigateToBaseNode"
```

This is extended on in the `__repr__` method to offer extra information relevant to the class, for instance the `Navigation` task includes the target node, so this can be easily identified in the log, the `Idle` tasks include the node the robot is waiting at, the `Await` tasks identify the associated agent being awaited, etc. While not offering core functional, this assist greatly in understanding the internals of the system operation. 

```python
# StageBase.get_class() returns:
"base.Navigation"
# Navigation.__repr__ converts using:
"base.Navigation(%s)" % self.agent.target
# so returns:
"base.Navigation(WayPoint67)"
```

The `suspend`, and `flag` methods are both wrappers to generalised functionality, so when functionality is late extended, these can offer centralisation. The `flag` includes a wrapper to not allow `_query` to complete if there iis an outstanding interruption. The `suspend` method has a default functionality of setting the new_stage flag if the stage is suspended such as when paused.


## Stage List:
For the full list of stages, see [][]. Each stage in the list is broken down, detailing the Class name, and the list of classes it inherits from, followed by the class description. For each of the five core methods, the doc comments from each stage of inheritance are identified and compiled with a label to the superclass they come from. For this reason it is important that any new stages, or adaptations to existing stages are adequately commented.

```
---------------------------------------------------------------------------
#### NavigateToBaseNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given base_node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call super to set association to base_node

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requiring a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which must be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.
```

This file is an auto-generated file from the doc-comments throughout the StageDef classes. This can be regenerated at any time using: `from rasberry_coordination.task_management import print_stages_md; print_stages_md()`






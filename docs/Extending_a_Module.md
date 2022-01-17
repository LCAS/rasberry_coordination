# Extending a Module
This page will define the process to extending a module, following an example of extending the transportation module to add a new courier-type to transport fruit from a local storage to a cold_storage.

[//]: # (Rename `storage` to `field_storage`)
[//]: # (Rename `courier` to `field_courier`)

## Define a Farm Courier
### 1. Define a Config
Under `$rasberry_coordination/config` create a new file for the setup of the agent `FARM_COURIER.yaml`. 

Being a robot, it must have the modules of `base: robot` and `health_monitoring: robot`. For its new position, it must have the associated module and role, so we add in the `transportation: farmcourier` role.
```yaml
modules:
  - name: base
    role: robot
  - name: health_monitoring
    role: robot
  - name: transportation
    role: farmcourier
```

For the navigation properties, as the agent is a robot, we set `has_presence: true`, and as it does not have explicit navigation limitations we can define `restrictions: none`.
```yaml
navigation_properties:
    has_presence: true
    restrictions: none
```

For the module properties, we can define the total load the farm_courier can handle with `max_load: 20`.
```yaml
module_properties:
    max_load: 20
```

For the visualisation properties, we can define the robot with the standard short_robot details.
```yaml
visualisation_properties:
    structure_type: short
    rviz_model: short_robot
```
### 2. Create an Interface
```python
class transportation_farmcourier(IDef.AgentInterface):
    def __init__(self, agent, sub='/r/get_states', pub='/r/set_states'):
        self.release_triggers = ['self', 'storage', 'toc']
        self.restart_triggers = ['farmstorage']

        responses={'PAUSE':self.pause, 
                   'UNPAUSE':self.unpause, 
                   'RELEASE':self.release}
        super(InterfaceDef.transportation_farmcourier, self).\
            __init__(agent, responses, sub=sub, pub=pub)

    def pause(self): 
        self.agent.set_interrupt('pause', 'transportation', self.agent['id'])

    def unpause(self): 
        self.agent.set_interrupt('unpause', 'transportation', self.agent['id'])

    def release(self): 
        self.agent.set_interrupt('reset', 'transportation', self.agent['id'])
```
### 3. Define Idle Task
```python
@classmethod
def transportation_farmcourier_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    LP = agent.local_properties
    MP = agent.module_properties

    #If agent is at max capacity deliver to cold store
    agent.local_properties['load'] = int(agent.local_properties['load'])
    if LP['load'] >= int(MP['max_load']):
        return TaskDef.transportation_cold_store_delivery(agent=agent, task_id=task_id, details=details, contacts=contacts)
```
### 4. Define Active Tasks 
#### A. Collection Task
```python
@classmethod
def transportation_collect_from_field(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    return(Task(id = task_id,
                module = 'transportation',
                name = "transportation_retrieve_load",
                details = details,
                contacts = contacts,
                initiator_id = initiator_id,
                responder_id = agent.agent_id,
                stage_list = [
                    SDef.StartTask(agent, task_id),

                    # StageDef.NavigateToPicker(agent),
                    # StageDef.Loading(agent)
                    
                    StageDef.AssignStorage(agent),
                    SDef.AssignWaitNode(agent),
                    StageDef.AwaitStoreAccess(agent),
                    StageDef.NavigateToStorage(agent),
                    StageDef.Unloading(agent)
                ]))

```
#### B. Delivery Task
```python
@classmethod
def transportation_deliver_to_cold_storage(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    return(Task(id = task_id,
                module = 'transportation',
                name = "transportation_deliver_load",
                details = details,
                contacts = contacts,
                initiator_id = agent.agent_id,
                responder_id = "",
                stage_list = [
                    SDef.StartTask(agent, task_id),
                    StageDef.AssignStorage(agent),
                    SDef.AssignWaitNode(agent),
                    StageDef.AwaitStoreAccess(agent),
                    StageDef.NavigateToStorage(agent),
                    StageDef.Unloading(agent)
                ]))
```

## Extend Local Storage Functionality
### 1. Extend Config Definitions
```yaml
module_properties:
    max_load: 50
```
### 2. Extend Interface Functionality
added function and response
[//]: # (todo: we might have to rethink our release/restart triggers)
[//]: # (todo: also change has_tray to trays)
[//]: # (todo: define tray as `obj(id:pickerid_traycount, weight, time_collected, etc.)`)
```python
class transportation_storage(IDef.AgentInterface):
    def __init__(self, agent, sub='/uar/get_states', pub='/uar/set_states'):
        self.release_triggers = ['self', 'toc']
        self.restart_triggers = ['thorvald']

        responses={'UNLOADED': self.unloaded,
                   'LOADED': self.loaded}
        super(InterfaceDef.transportation_storage, self).__init__(agent, responses, sub=sub, pub=pub)

        #These need a new home
        self.agent.request_admittance = []
        self.agent.has_presence = False  # used for routing (swap key for physical?)

    def unloaded(self): 
        self.agent['has_tray'] = True
        
    def loaded(self):
        self.agent['has_tray'] = False
```
### 3. Update Idle Task
Currently, the `request_admittance` list holds the agent_id, for our ability to distinguish the required task, we much first identify the role, then select the correct task to use.
```python
@classmethod
def transportation_storage_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    #If agents are waiting to visit, begin transportation storage
    #Otherwise wait idle
    if len(agent.request_admittance) > 0:
        role = agent.agent_manager[agent.request_admittance[0]].interfaces['transportation'].role
        if role == 'farm_courier':
            return TaskDef.transportation_load_farm_courier(agent=agent, task_id=task_id, details=details, contacts=contacts)
        return TaskDef.transportation_unload_field_courier(agent=agent, task_id=task_id, details=details, contacts=contacts)
    else:
        return TaskDef.idle_storage_def(agent=agent, task_id=task_id, details=details, contacts=contacts)
```
### 4. Create an Active Task
With functional similarity to the `transportation_load_field_courier` and `transportation_collect_from_picker` tasks, we do not need to create new stages, and can make use of the field courier stages for accept and await, and the load courier from the collection task.
```python
@classmethod
def transportation_load_farm_courier(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    return(Task(id = task_id,
                module = 'transportation',
                name = "transportation_load_farm_courier",
                details = details,
                contacts = contacts,
                initiator_id = "",
                responder_id = agent.agent_id,
                stage_list = [
                    StageDef.AcceptCourier(agent),
                    StageDef.AwaitCourier(agent),
                    StageDef.LoadCourier(agent)
                ]))
```

## Define a Cold Storage
### 1. Define a Config
```python

```
### 2. Create an Interface
```python

```
### 3. Define Idle Task
```python

```
### 4. Define Active Tasks
```python

```

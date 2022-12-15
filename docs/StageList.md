# Stage List:
This document details the structure and formatting of the stages defined in the StageDef for each module in:


- [Base](#Base)
- [Health Monitoring](#HealthMonitoring)
- [Transportation](#Transportation)
- [UV Treatment](#UVTreatment)
- [Data Collection](#DataCollection)

--------------------------------------------------
<a name="Base"></a>
## Base
Stages defined in the base module:  


-------------------------------------------------
#### NavigateToNode: [`Navigation(.)`, `StageBase(:)`]
Used for navigating to a given node

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle navigation target 

    -> __init__
       | : | Class initialisation for populating default values
       | . | Identify the location from which the target is identified

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Flag the agent as requirieng a route
       |   | Start task by setting the target to a given node

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete when the agents location is identical to the target location.

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       | . | End navigation by refreshing routes for other agents in motion.




-----------------------
#### Pause: [`StageBase(.)`]
Inserted on Pause for use as a 3-stage controlled blocker, used on Coordinator, Task, or Agent scopes

    -> __repr__
       | . | Simplified representation of class for clean informative logging
       |   | Display scopes actively contributing to the blocking

    -> __init__
       | . | Class initialisation for populating default values
       |   | Initialise blocking properties

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | On start, cancel any active navigation

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Continue once all blocking stages are False

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | On end, reenable registration




--------------------------------------------------
#### NavigateToAgent: [`Navigation(.)`, `StageBase(:)`]
Used for navigating to a given agent

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle navigation target 

    -> __init__
       | : | Class initialisation for populating default values
       | . | Identify the location from which the target is identified

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Flag the agent as requirieng a route
       |   | Start task by setting the target to be a defined agent's current location

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete when the agents location is identical to the target location.

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       | . | End navigation by refreshing routes for other agents in motion.




-------------------------------------------------------------------
#### AssignBaseNode: [`AssignNode(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to identify the closest available base_node.

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |    | Start action to find closest base_node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | Save action response to contacts




-------------------------------
#### NotifyTrigger: [`StageBase(.)`]
Used to send a message to trigger some response

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values
       |   | Save initialisation details for message

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Send trigger message

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Wait for flag to be set by message response (os #PSUEDO)

    -> _end
       | . | Used to set any fields as the stage is about to be removed




-------------------------------------------------------------
#### FindRows: [`AssignNode(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to generate a list of row tasks given a tunnel id.

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |    | Display the tunnel id to generate tasks for.

    -> __init__
       | .: | Class initialisation for populating default values
       |    | Save the tunnel details

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |    | Initialise action details to search for rows

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | Begin defined task for each row in action response




----------------------
#### Idle: [`StageBase(.)`]
Used to suspend activity until the agent has a task.

    -> __repr__
       | . | Simplified representation of class for clean informative logging
       |   | Display class with idle location 

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once task buffer contains another task, or if battery level is low.

    -> _end
       | . | Used to set any fields as the stage is about to be removed




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
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




----------------------
#### Exit: [`StageBase(.)`]
Used for controlled removal of agent connections

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Unregister agent and cancel any accociated tasks

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Continue once all tasks are emoved from the buffer

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | Set marker to black and initiate disconnection interruption-




----------------------------------------------
#### AssignAgent: [`Assignment(.)`, `StageBase(:)`]
Handler for stages based around identifying an agent of interest.

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values
       |   | Initialise the agent's type and the action style

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Set flag to perform multi-agent action
       |   | Initiate action details to identify agent

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once action has generated a result

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   |  On completion, save agent contact




-------------------------------------------------------------------
#### AssignWaitNode: [`AssignNode(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to identify the closest availalbe wait_node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |    | Start action to find closest wait_node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | Save action response to contacts




---------------------------------------------
#### AssignNode: [`Assignment(.)`, `StageBase(:)`]
Handler for stages based around itdntifying a node of interest.

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Set flag to perform multi-agent action

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once action has generated a result

    -> _end
       | : | Used to set any fields as the stage is about to be removed




-------------------------------------
#### WaitForLocalisation: [`StageBase(.)`]
Called to suspend task progression till a location for the agent is recieved

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Enable location monitoring

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once location has been identified

    -> _end
       | . | Used to set any fields as the stage is about to be removed




----------------------------
#### Navigation: [`StageBase(.)`]
Base task for all Navigation

    -> __repr__
       | . | Simplified representation of class for clean informative logging
       |   | Display class with idle navigation target 

    -> __init__
       | . | Class initialisation for populating default values
       |   | Identify the location from which the target is identified

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Flag the agent as requirieng a route

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete when the agents location is identical to the target location.

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | End navigation by refreshing routes for other agents in motion.




---------------------------------------------------------------------------
#### NavigateToWaitNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given wait_node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call super to set association to wait_node

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




------------------------------------------------------------------
#### FindStartNode: [`AssignNode(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to identify of two nodes, which one is closest ot the agent.

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |    | Display row ends in the repr

    -> __init__
       | .: | Class initialisation for populating default values

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |    | Start action to find which node of the ends is the closest to start from

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | Save the closest and furthest node to be the row start and row end




-------------------------------------------
#### EnableDebugLocalisation: [`StageBase(.)`]
Enable localisation for debug agents

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Enable subscribers to debug localisation

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete the stage without any condition

    -> _end
       | . | Used to set any fields as the stage is about to be removed




-----------------------------
#### SetRegister: [`StageBase(.)`]
Called to mark the agent as registered

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Mark agent as unregistered and send a message to rviz to display the agent without modified colour

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete the stage without any condition

    -> _end
       | . | Used to set any fields as the stage is about to be removed




----------------------------
#### Assignment: [`StageBase(.)`]
Base task for all Assignments

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Set flag to perform multi-agent action

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once action has generated a result

    -> _end
       | . | Used to set any fields as the stage is about to be removed




--------------------------
#### SendInfo: [`StageBase(.)`]
None

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values
       |   | None

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | None

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once action has generated a result

    -> _end
       | . | Used to set any fields as the stage is about to be removed




----------------------------------------------------------------
#### FindRowEnds: [`AssignNode(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to identify the two ends of a given row.

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |    | Display the id for the row of interest

    -> __init__
       | .: | Class initialisation for populating default values
       |    | Save the row id of interest

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |    | Start action to search for the start and end of the row of interest

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | Save row ends to the contacts dictionary




-------------------------------
#### SetUnregister: [`StageBase(.)`]
Called to mark the agent as unregistered

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Mark agent as unregistered and send a message to rviz to display the agent as red

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete the stage without any condition

    -> _end
       | . | Used to set any fields as the stage is about to be removed




---------------------------------------------------------------------------
#### NavigateToExitNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given exit_node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call super to set association to exit_node

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




-------------
#### StageBase: []
Base class for all Stages

    -> __repr__
       |  | Simplified representation of class for clean informative logging

    -> __init__
       |  | Class initialisation for populating default values

    -> _start
       |  | Stage start, called when this is the active stage.

    -> _query
       |  | Used to define the criteria which ust be met for the stage to be completed

    -> _end
       |  | Used to set any fields as the stage is about to be removed




----------------------------
#### WaitForMap: [`StageBase(.)`]
Called to suspend task progression till a restricted map is recieved

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Begin a subscriber to recieve the tmap

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete stage once a tmap is available

    -> _end
       | . | Used to set any fields as the stage is about to be removed




---------------------------
#### StartTask: [`StageBase(.)`]
Called as the first stage of every task, to generate and apply a task_id.

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values
       |   | Generate a unique Task ID of {agent_id}_{total_tasks++}.

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Being at the head of the task, once the task is begun the task_id is adopted as the active task_id.

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete the stage without any condition

    -> _end
       | . | Used to set any fields as the stage is about to be removed



--------------------------------------------------
<a name="HealthMonitoring"></a>
## Health Monitoring
Stages defined in the modules.health_monitoring module:  


-------------------------------------------------
#### StartChargeTask: [`StartTask(.)`, `StageBase(:)`]
Used to Initiate task with the agent set to unregistered

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values
       | . | Generate a unique Task ID of {agent_id}_{total_tasks++}.

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Being at the head of the task, once the task is begun the task_id is adopted as the active task_id.
       |   | Set registration to false when charging is begun

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete the stage without any condition

    -> _end
       | : | Used to set any fields as the stage is about to be removed




-----------------------------------------------------------------------------
#### NavigateToChargeNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to the assigned charging station

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Identify associated contact as 'charging_station'

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.
       |    | Complete navigation if agents location is the target of if battery level is set to be above threshold

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




------------------------
#### Charge: [`StageBase(.)`]
Used to Pause task progression till battery level is usable

    -> __repr__
       | . | Simplified representation of class for clean informative logging
       |   | Return battery level with class name

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once battery level is safe

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | Enable registration once task is ended




---------------------------------------------------------------------
#### AssignChargeNode: [`AssignNode(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to Identify and reserve a charging station

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |    | Initiate action to find charging station

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | Save reserved charging station to contacts



--------------------------------------------------
<a name="Transportation"></a>
## Transportation
Stages defined in the modules.transportation module:  


-------------------------
#### Loading: [`StageBase(.)`]
Used for awaiting a change-of-state from the picker

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once agents's has_tray flag is true

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | On completion, increment the field_courier's total load by 1




----------------------------------------
#### IdleStorage: [`Idle(.)`, `StageBase(:)`]
Used to Idle a storage agent whilst awaiting a request for admittance

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle location 

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once task buffer contains another task, or if battery level is low.
       |   | Complete once there exists any agents requiring storage

    -> _end
       | : | Used to set any fields as the stage is about to be removed




-------------------------------------
#### TimeoutFlagModifier: [`StageBase(.)`]
Used to idle till timeout or a flag is set

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.
       |   | Define the completion flag and timeout

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once has_tray flag is triggered by interface or timeout completes

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | On Completion, set the field_courier's flag and notify picker




------------------------------------------------------------------------
#### AssignFieldStorage: [`AssignAgent(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to identify a storage location to deliver the load

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values
       |  . | Initialise the agent's type and the action style
       |    | None

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |  . | Initiate action details to identify agent

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  . |  On completion, save agent contact
       |    | On completion, save the storage and add the field_courier's id to the storage's request_admittance list




----------------------------------------------------------------------------
#### AwaitFieldStorageAccess: [`AwaitStoreAccess(.)`, `Idle(:)`, `StageBase(.:)`]
Used to Idle the Field Courier till the field_storage accepts admittance

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle location 
       |  . | Attach id of agent to class name

    -> __init__
       | .: | Class initialisation for populating default values
       |  . | None
       |    | Specify the 

    -> _start
       | .: | Stage start, called when this is the active stage.

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once task buffer contains another task, or if battery level is low.
       |  . | Complete if the field_courier assigned to the storage of interest is this agent

    -> _end
       | .: | Used to set any fields as the stage is about to be removed




---------------------------
#### Unloading: [`StageBase(.)`]
Used for awaiting a change-of-state from the storage

    -> __repr__
       | . | Simplified representation of class for clean informative logging

    -> __init__
       | . | Class initialisation for populating default values

    -> _start
       | . | Stage start, called when this is the active stage.

    -> _query
       | . | Used to define the criteria which ust be met for the stage to be completed
       |   | Complete once agents's has_tray flag is false

    -> _end
       | . | Used to set any fields as the stage is about to be removed
       |   | On completion, reset the field_courier's total load to 0




--------------------------------------------------------------------------------
#### NavigateToFieldStorage: [`NavigateToAgent(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to define the target for the navigation as the field_storage

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Set navigation target as associated field_storage

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to be a defined agent's current location

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




--------------------------------------------------------------
#### UnloadFieldCourier: [`TimeoutFlagModifier(.)`, `StageBase(:)`]
Used to define completion details for when the field_courier can be consideded unloaded

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Define the completion flag and timeout
       |   | Define the flag default as False and the timeout as the tranportation/wait_unloading property

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once has_tray flag is triggered by interface or timeout completes

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       | . | On Completion, set the field_courier's flag and notify picker




----------------------------------------------------------------
#### IdleFieldStorage: [`IdleStorage(.)`, `Idle(:)`, `StageBase(.:)`]
None

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle location 

    -> __init__
       | .: | Class initialisation for populating default values

    -> _start
       | .: | Stage start, called when this is the active stage.

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once task buffer contains another task, or if battery level is low.
       |  . | Complete once there exists any agents requiring storage

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |    | On completion, add an idle field_storage to the end of the buffer




------------------------------------------------------------------------
#### AssignFieldCourier: [`AssignAgent(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to identify an available field_courier to collect a load

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values
       |  . | Initialise the agent's type and the action style
       |    | None

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |  . | Initiate action details to identify agent

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  . |  On completion, save agent contact
       |    |  On completion, notify picker of field_courier acceptance, and assign a retrieve load task to the field_courier




--------------------------------------------------------------------------
#### NavigateToPicker: [`NavigateToAgent(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to define the target for the navigation as the picker

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Set navigation target as associated picker

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to be a defined agent's current location

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




------------------------------------------------------------------------
#### AcceptFieldCourier: [`AssignAgent(.)`, `Assignment(:)`, `StageBase(.:)`]
Used to notify a pending field_courier of admittance

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values
       |  . | Initialise the agent's type and the action style
       |    | None

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |  . | Initiate action details to identify agent
       |    | Initiate action details to identify the closest field_courier from those that request admittance

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  . |  On completion, save agent contact
       |    | On completion, save the action response and remove the field_courier from the request_admittance list




------------------------------------------------------------
#### LoadFieldCourier: [`TimeoutFlagModifier(.)`, `StageBase(:)`]
Used to define completion details for when the field_courier can be consideded loaded

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Define the completion flag and timeout
       |   | Define the flag default as True and the timeout as the tranportation/wait_loading property

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once has_tray flag is triggered by interface or timeout completes

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       | . | On Completion, set the field_courier's flag and notify picker




----------------------------------------------
#### AwaitFieldCourier: [`Idle(.)`, `StageBase(:)`]
Used to idle the agent until a field_courier has arrived

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle location 
       |   | Attach id of agent to class name

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.
       |   | None

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once task buffer contains another task, or if battery level is low.
       |   | Complete once the associated field_courier has arrived at the agents location

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | On completion, notify the picker of ARRIVAL




---------------------------------------------
#### AwaitStoreAccess: [`Idle(.)`, `StageBase(:)`]
Used to idle the field_courier until the storage location has accepted admittance

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle location 
       |   | Attach id of agent to class name

    -> __init__
       | : | Class initialisation for populating default values
       |   | None

    -> _start
       | : | Stage start, called when this is the active stage.

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once task buffer contains another task, or if battery level is low.
       |   | Complete if the field_courier assigned to the storage of interest is this agent

    -> _end
       | : | Used to set any fields as the stage is about to be removed



--------------------------------------------------
<a name="UVTreatment"></a>
## UV Treatment
Stages defined in the modules.uv_treatment module:  


-------------------------------------------------------------------------------
#### FindRowsUV: [`FindRows(.)`, `AssignNode(:)`, `Assignment(.:)`, `StageBase(::)`]
Used to assign the uv_treatment_treat_row task to all rows in the given tunnel.

    -> __repr__
       | :: | Simplified representation of class for clean informative logging
       |  . | Display the tunnel id to generate tasks for.

    -> __init__
       | :: | Class initialisation for populating default values
       |  . | Save the tunnel details
       |    | Call super to set uv_treatment_treat_row as task to apply

    -> _start
       | :: | Stage start, called when this is the active stage.
       | .: | Set flag to perform multi-agent action
       |  . | Initialise action details to search for rows

    -> _query
       | :: | Used to define the criteria which ust be met for the stage to be completed
       | .: | Complete once action has generated a result

    -> _end
       | :: | Used to set any fields as the stage is about to be removed
       |  . | Begin defined task for each row in action response




----------------------------------------------------------------------------
#### NavigateToUVEndNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given end node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call to super to set the navigation target as the node stored in the action association

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




---------------------------------------------------
#### EnableUVLight: [`NotifyTrigger(.)`, `StageBase(:)`]
Used to enable the UV light on the uv robot

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values
       | . | Save initialisation details for message
       |   | Call to initialise a light_status message of ENABLE_LIGHT to send on start and set rviz robot to blue

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Send trigger message

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Wait for flag to be set by message response (os #PSUEDO)

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | None




----------------------------------------------------
#### DisableUVLight: [`NotifyTrigger(.)`, `StageBase(:)`]
Used to disable the UV light on the uv robot

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values
       | . | Save initialisation details for message
       |   | Call to initialise a light_status message of DISABLE_LIGHT to send on start and set rviz robot to clear

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Send trigger message

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Wait for flag to be set by message response (os #PSUEDO)

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | None




------------------------------------------------------------------------------
#### NavigateToUVStartNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given start node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call to super to set the navigation target as the node stored in the action association

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node
       |    | None

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




--------------------------------------------------------------------------
#### AssignPhototherapist: [`AssignAgent(.)`, `Assignment(:)`, `StageBase(.:)`]
None

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values
       |  . | Initialise the agent's type and the action style
       |    | None

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |  . | Initiate action details to identify agent

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  . |  On completion, save agent contact
       |    | None




--------------------------------------------
#### AwaitCompletion: [`Idle(.)`, `StageBase(:)`]
None

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle location 

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.
       |   | None

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once task buffer contains another task, or if battery level is low.
       |   | None

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | None



--------------------------------------------------
<a name="DataCollection"></a>
## Data Collection
Stages defined in the modules.data_collection module:  


-------------------------------------------------------------------------------
#### FindRowsDM: [`FindRows(.)`, `AssignNode(:)`, `Assignment(.:)`, `StageBase(::)`]
Used to assign the data_collection_scan_row task to all rows in the given tunnel.

    -> __repr__
       | :: | Simplified representation of class for clean informative logging
       |  . | Display the tunnel id to generate tasks for.

    -> __init__
       | :: | Class initialisation for populating default values
       |  . | Save the tunnel details
       |    | Call super to set data_collection_scan_row as task to apply

    -> _start
       | :: | Stage start, called when this is the active stage.
       | .: | Set flag to perform multi-agent action
       |  . | Initialise action details to search for rows

    -> _query
       | :: | Used to define the criteria which ust be met for the stage to be completed
       | .: | Complete once action has generated a result

    -> _end
       | :: | Used to set any fields as the stage is about to be removed
       |  . | Begin defined task for each row in action response




------------------------------------------------------------------------------
#### NavigateToDMStartNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given start node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call to super to set the navigation target as the node stored in the action association

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node
       |    | None

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




----------------------------------------------------------------------------
#### NavigateToDMEndNode: [`NavigateToNode(.)`, `Navigation(:)`, `StageBase(.:)`]
Used to navigate to a given end node

    -> __repr__
       | .: | Simplified representation of class for clean informative logging
       |  : | Display class with idle navigation target 

    -> __init__
       | .: | Class initialisation for populating default values
       |  : | Identify the location from which the target is identified
       |    | Call to super to set the navigation target as the node stored in the action association

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Flag the agent as requirieng a route
       |  . | Start task by setting the target to a given node

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete when the agents location is identical to the target location.

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  : | End navigation by refreshing routes for other agents in motion.




-------------------------------------------------------------------
#### AssignScanner: [`AssignAgent(.)`, `Assignment(:)`, `StageBase(.:)`]
None

    -> __repr__
       | .: | Simplified representation of class for clean informative logging

    -> __init__
       | .: | Class initialisation for populating default values
       |  . | Initialise the agent's type and the action style
       |    | None

    -> _start
       | .: | Stage start, called when this is the active stage.
       |  : | Set flag to perform multi-agent action
       |  . | Initiate action details to identify agent

    -> _query
       | .: | Used to define the criteria which ust be met for the stage to be completed
       |  : | Complete once action has generated a result

    -> _end
       | .: | Used to set any fields as the stage is about to be removed
       |  . |  On completion, save agent contact
       |    | None




-----------------------------------------------------
#### DisableDMCamera: [`NotifyTrigger(.)`, `StageBase(:)`]
Used to disable the camera on the robot

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values
       | . | Save initialisation details for message
       |   | Call to initialise a camera_status message of DISABLE_CAMERA to send on start and set rviz robot to clear

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Send trigger message

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Wait for flag to be set by message response (os #PSUEDO)

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | None




----------------------------------------------------
#### EnableDMCamera: [`NotifyTrigger(.)`, `StageBase(:)`]
Used to enable the camera on the robot

    -> __repr__
       | : | Simplified representation of class for clean informative logging

    -> __init__
       | : | Class initialisation for populating default values
       | . | Save initialisation details for message
       |   | Call to initialise a camera_status message of ENABLE_CAMERA to send on start and set rviz robot to green

    -> _start
       | : | Stage start, called when this is the active stage.
       | . | Send trigger message

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Wait for flag to be set by message response (os #PSUEDO)

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | None




--------------------------------------------
#### AwaitCompletion: [`Idle(.)`, `StageBase(:)`]
None

    -> __repr__
       | : | Simplified representation of class for clean informative logging
       | . | Display class with idle location 

    -> __init__
       | : | Class initialisation for populating default values

    -> _start
       | : | Stage start, called when this is the active stage.
       |   | None

    -> _query
       | : | Used to define the criteria which ust be met for the stage to be completed
       | . | Complete once task buffer contains another task, or if battery level is low.
       |   | None

    -> _end
       | : | Used to set any fields as the stage is about to be removed
       |   | None


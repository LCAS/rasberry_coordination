#AgentState.msg / AgentStateList.msg
Sent by the coordinator detailing the task state of all connected agents.

#Interruption.msg
Recieved by the coordinator to interrupt an active task by pausing or cancelling it.

#KeyValuePair.msg
TODO: replace this with diagnostic_msgs/keyvalue.

#MarkerDetails.msg
Sent by coordinator to visualise_agent_markers, detailing how to spawn/modify an agent marker.

#Module.msg
Used by coordinator to standardise the connection of roles in modules.
TODO: this can probably be removed.

#NewAgentConfig.msg / NewAgentConfigSetup.msg
Recieved by coordinator to initialise new agents into the system.

#TaskDetails.msg / TasksDetails.msg
Sent by coordinator to TOC detailling the active tasks state.
This only details the tasks from the perspectives of the initiator. (A logistics robot is listed under the picker's task.

#TopoLocation.msg
Used by coordinator to accept UV and DC tasks directly.

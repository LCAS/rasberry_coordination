# Main Loop
The main functionality of the ATM Coordinator is focused around the Main Loop, this being a synchronous management of all task progression, interruptions, communication, etc.

##Overview
The main loop manages all standard functionality within the coordinator, repeating these 13 steps while active.

```yaml
1: Add New Agents
2: Interrupt Stage Execution
3: Update local list of Agents
4: Start Buffered Task
5: Update TOC
6: Start Stage
7: Offer Action Services
8: Find Routes
9: Publish Routes
10: Perform Stage-Completion Query
11: End Stage
12: Update TOC with Completed
13: Pause
```



### Breakdown:

#### 1: Add New Agents
Requests to add agents, are done asynchronously by adding their details to a buffer. This is handled by the main loop synchronously by instantiating the agents first.

#### 2: Interrupt Stage Execution
Requests to interrupt agents or tasks are handled next, this includes removing any agents form the system, pausing/cancelling tasks or the entire coordinator as a whole.

#### 3: Update local list of Agents
The local list of agents is then updated, taking a copy of the list in the agent manager.

#### 4: Start Buffered Task
For any agents without an active task, the next task in their task buffer is moved to the active task; if the buffer is empty an idle task is applied for each associated module. If there are no idle tasks applied, the default idle task is applied.

#### 5: Update TOC
If the TOC update timer has exceeded the timeout, or any active stages are marked as a new stage, compile the details of all active tasks to publish to TOC.

#### 6: Start Stage
For each active stage, if it is a new stage, call the `start` function.

#### 7: Offer Action Services
Perform cross-agent actions for any agents which require such. This can include identifying a closest agent, or identifying a node of interest.

#### 8: Find Routes
If any agents require routes, of th `trigger_fresh_replan` flag is set, identify routes of any agents which need them using the route planner defined in the coordinator configuration file.

#### 9: Publish Routes
If any saved route differs significantly from the already published one, publish the new one.

#### 10: Perform Stage-Completion Query
Check the active stage for each agent to query if the completion criteria is met, if a stage is completed, it will set the completion flag to True

#### 11: End Stage
For each agent with a completed flag set, the end stage is called to tidy up any loose ends and the stage is removed from the active stage list.

#### 12: Update TOC with Completed
As TOC only removes a task if a COMPLETED message is passed with the task id, this process ensures that is sent for any task with an empty stage list.

#### 13: Pause
Add a reasonably short delay, this ensures the server does not over-process from empty processing.

### Interactions
All interactions with the coordinator from outside sources do so with the use of flags, or by adding tasks to ones own task buffer. For example, if a fruit picker requests a courier to collect some trays, it publishes a message which is received by the `transportation_picker_interface`, and adds a new task (`transportation_request_courier`) to itself. If the picker wishes to acknowledge an arrived courier has taken the trays, it publishes a message which is received by the `transportation_picker_interface` and sets the flag `self.agent['has_tray'] = False`. This flag completes the stage-completion-query for the picker, and ends the stage by setting the associated courier's flag: `self.agent['contacts']['courier']['has_tray'] = ~True~`.

This use of flags ensures outside interference does not compromise the stability of task progression, so at every point the control of task progression is ensured by the main loop.



## Simplified Main Loop Code

`AM`: Agent Manager  
`Ut`: Time of last TOC message  
` E`: List of tasks which have ended

```python
if AM.new_agent_buffer: AM.add_agent_from_buffer()      # Add New Agents
if any([a.interruption for a in A]): interrupt_task()   # Interrupt Stage Execution
A = get_agents()                                        # Update local list of Agents
[a.start_next_task() for a in A if not a['stage_list']] # Start Buffered Task
Ut = Update_TOC(A, TOC, Ut)                             # Update TOC
[a.start_stage() for a in A if a().new_stage]           # Start Stage
[offer_service(a) for a in A if a().action_required]    # Offer Action Services
if trigger_routing(A): find_routes()                    # Find Routes
[publish_route(a) for a in A if a().route_found]        # Publish Routes
[a()._query() for a in A]                               # Perform Stage-Completion Query
E=[a.end_stage() for a in A if a().stage_complete]      # End Stage
if any(E): TOC.EndTask(E)                               # Update TOC
```
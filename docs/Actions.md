#Actions 
Actions can be defined as any act an agent performs which requires information beyond the agent's own properties. Actions can be classified based on the type of information which they require and how they are instigated.

##Single-Agent Actions
Thee require information beyond the agent, but do not require the use of any other agents. This can include actions such as finding if a specific node exists, or getting some information on the structure of the environment such as finding the closest of a list of nodes. Acts which require the use of tools beyond the agent's toolbox also fit into this category.

`from rasberry_coordination.cross_agent_actions.single import SingleAgentAction`

##Dual-Agent Actions
These types of actions require information of other agents such as their location, or registration, but are generally limited to disallow modification of any other agent. These can include identifying an available storage manager, or the closest of a list of nodes, but would not include forcing another agent to swap tasks.

`from rasberry_coordination.cross_agent_actions.dual import DualAgentAction`

##Multi-Agent Actions
These types of directives, unlike the others, allow the direct interaction of other agents, so can include initiating optimisers for route planning or task allocation. These types of tasks would be initiated by a single agent, however has no limitations on the number of agents which can be involved or how much modification can be applied to them.

`from rasberry_coordination.cross_agent_actions.multi import MultiAgentAction`


#Defining New Actions


1. coordinator gives requested tools/info to the stage
2. actions moved to independent classes
3. actions moved to function calls
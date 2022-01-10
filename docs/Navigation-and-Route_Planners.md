# Navigation and Route Planners
[desc on how we can swap for different types]
```yaml
#ROUTING
planning_type: fragment_planner
heterogeneous_map: true
```

Once a second planner is to be added, this will be updated with deeper standardised information.

## Route Planner: `Base-Class`
The base planner defines the tools for use by inherited planners, along with the @abstractmethods for child classes,
working as an interface to the Topological map.
```yaml
1: __init__
2: _map_cb
3: find_routes
4a: get_edge_distances
4b: get_distance_between_adjacent_nodes
4c: get_node
5a: shortest_route_to_node
5b: get_route_distance_to_node
6: get_available_optimum_route
7: load_route_search
8: load_occupied_nodes
9: no_route_found

#
#.a: get_row_ends
#.b: get_rows

#.A: get_agents
```

## Route Planner: `Fragment Planner`
```yaml
1: __init__
2: find_routes
3: update_available_tmap
4: unblock_node
5: split_critical_paths
6: critical_points
```
























































1. block access to occupied nodes by removing their connecting edges from the map
2. identify a list of active and inactive agents based on whether they have a goal

3. find routes for each active agent by unblocking their start and goal, setting as inactive if the route is empty
4. set the route of each inactive agent as their current location

5. identify all route intersections and the agents that own them (excluding goal nodes) for active agents

6. for each agent
   1. for each node in route
      1. if node is critical
         1. find agent closest to conflict
         2. if nearest agent is the owner
            1. mark critical node as assigned
            2. allow agent to take remaining nodes
         3. if node unassigned and agent allowed to take it
            1. mark critical node as assigned
         4. else
            1. if a partial route exists, save to collective route
            2. reset partial route as only node 
      2. if node is not critical, add to the partial route
   2. if a partial route exists, save to collective route
   3. save the collective route to the res_routes 
7. save the res_route to the route fragments for each agent
8. shift each fragment goal/goal_edge to start the following fragment for each agent
9. apply res_edges to each agent









































1. block access to occupied nodes by removing their connecting edges from the map
2. identify a list of active and inactive agents based on whether they have a goal
3. find routes for each active agent by unblocking their start and goal, setting as inactive if the route is empty
4. set the route of each inactive agent as their current location
5. identify route intersections for active agents and the agents that own them
6. remove goals from critical node list
7. for each agent
   1. for each node in route
      1. if node is critical
         1. find agent closest to conflict
         2. if nearest agent is the owner
            1. mark critical node as assigned
            2. allow agent to take remaining nodes
         3. if node unassigned and agent allowed to take it
            1. mark critical node as assigned
         4. else
            1. if a partial route exists, save to collective route
            2. reset partial route as only node 
      2. if node is not critical, add to the partial route
   2. if a partial route exists, save to collective route
   3. save the collective route to the res_routes 
8. save the res_route to the route fragments for each agent
9. shift each fragment goal/goal_edge to start the following fragment for each agent
10. apply res_edges to each agent




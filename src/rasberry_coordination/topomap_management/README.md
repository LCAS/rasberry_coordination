We utilise the map filtewring for the agents currently.


Agents:map_manager keep a copy of the full map
Agents:map_manager filter map locally for route planning
Server:route_planner plans route on agent filtered copy of map
Server:route_planner checks map of agents for getting node information




Agent::Map_Manager::map
Agent::MapManager((MapFilters))map




Coordinator
  - AllAgents
      - Agent
          - Map Manager
              - Map Filter

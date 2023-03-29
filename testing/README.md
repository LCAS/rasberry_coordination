There are 3 types of testing which this package covers; the first is integration testing, this is executed as a github action and is intended to test a series of explicit functionalities. For instance, one test might include the following steps to ensure the system builds without issue:
```
1. Launch docker
2. Download repositories
3. Run colcon build
```

This would be followed by a series of scripts like below, each would test a selection of key functionalities:
```
1. Launch map and coordination servers
2. Set debug-robot position to X
3. Send a move-to-node command to Y
4. Wait for robot to arrive
5. Shutdown servers
```

The second type of testing is robustness testing.
This is to be executed `daily/weekly` through a chron job.
This is intended to show `how a speficied route planner handles across different maps and complexities`.
It will consist of the following steps:
```
1. Load in a specific configuration file
2. Generate a series of TMap's utilising WFC
3. For each TMap:
4.     For each level of compexity:
5.         Launch the map and coordination servers
6.         Set the agent positions to X,Y,Z     (localisation/disable)
7.         Send move-to-node commands to A,B,C  (navigation/move_to_node)
8.         Collect statistics
9.         Send results to WeightsAndBiases
```
#TODO: This needs to be further refined as we go along based on identified weaknesses of the approach.


The third type of testing is `comparison testing`.
This is to be executed `weekly/monthly`.
This is intended to show `how route planners compare to one another`.
It will consist oof the following steps:
```
1. Load in a specific configuration file
2. Generate a TMap's utilising WFC
3. For each installed route planner: (routing_manager/change_planner data:2)
4.     For each level of compexity:
5.         Launch the map and coordination servers
6.         Set the agent positions to X,Y,Z     (localisation/disable data:X)
7.         Send move-to-node commands to A,B,C  (navigation/move_to_node data:A)
8.         Collect statistics
9.         Send results to WeightsAndBiases
```

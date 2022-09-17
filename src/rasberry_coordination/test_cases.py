The coordinator has a defined set of inputs and outputs, to test different portions of code, we can automate this...


For instance '~dtm' takes 3 inputs for type, 3 inputs for scope and a target, when broken cases are removed, we are left with the following:
- Coordinator pause
- Coordinator unpause
- Coordinator reset
- Task pause
- Task unpause
- Task reset
- Agent pause
- Agent unpause
- Agent reset
If we are able to echo the results of the coordinators logmsg to a topic, we can both publish requests and listen for desired responses.
The only thing we cannot do is utilise this for complex scenarios.

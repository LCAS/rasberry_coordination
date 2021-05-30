#!/usr/bin/env python

import rospy
from rospy import ServiceProxy, wait_for_service as WaitService
from rasberry_coordination.msg import KeyValuePair, TaskRole, KeyValuePair
from rasberry_coordination.srv import AddAgent

if __name__ == '__main__':
    rospy.init_node("AddAgent_Interface", anonymous=False)
    add_agent = ServiceProxy("/rasberry_coordination/dfm/add_agent", AddAgent)

    choice0 = int(raw_input("Load default thorvald_002 [0,1]:"))
    if choice0 == 0:
        agent = AddAgent()
        agent.agent_id = raw_input("Enter Agent Identifier: \n$ ")
        agent.has_presence = bool(raw_input("\nDoes node occupation block traversal: \n$ "))
        agent.initial_location = raw_input("\nEnter initial location node if known: \n$ ")

        tasks = []
        n = int(raw_input("\nTotal tasks: \n$ "))
        for i in range(n):
            task = TaskRole()
            task.module = raw_input("Enter module: ")
            task.role = raw_input("Enter role: ")

            if task.module and task.role:
                tasks += [task]
            else:
                break
        agent.tasks = tasks

        KVPs = []
        n = int(raw_input("\nTotal properties: \n$ "))
        for i in range(n):
            KVP = KeyValuePair()
            KVP.key = raw_input("Enter key: ")
            KVP.value = raw_input("Enter value for key(%s): " % KVP.key)

            if KVP.key and KVP.value:
                KVPs += [KVP]
            else:
                break
        agent.properties = KVPs
    else:
        agent = AddAgent()
        agent.agent_id = "thorvald_002"
        agent.has_presence = True
        agent.initial_location = ""

        task = TaskRole()
        task.module = "transportation"
        task.role = "courier"
        agent.tasks = [task]

        kvp1 = KeyValuePair()
        kvp1.key = "max_load"
        kvp1.value = "2"
        kvp2 = KeyValuePair()
        kvp2.key = "load"
        kvp2.value = "1"
        agent.properties = [kvp1, kvp2]

        print(agent)
        print(agent.tasks)

    resp = add_agent(
        agent_id=agent.agent_id,
        tasks=agent.tasks,
        properties=agent.properties,
        has_presence=agent.has_presence,
        initial_location=agent.initial_location
        )

    print("Response: [%s] %s"%(resp.success, resp.msg))

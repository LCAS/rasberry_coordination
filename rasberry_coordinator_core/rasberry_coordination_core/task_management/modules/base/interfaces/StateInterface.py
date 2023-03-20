# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination_core.task_management.modules.base.interfaces.Interface import iFACE as Interface
from rasberry_coordination_core.task_management.__init__ import Stages
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.utils.logmsg import logmsg

from diagnostic_msgs.msg import KeyValue


# Automanaged by rasberry_coordination_core.task_management.__init__.load_modules
# Interface class must be named `iFACE` to be recognised for import
# It will then be identifiable by its Interfaces[<<module>>][<<filename>>]
class iFACE(Interface):
    def __init__(self, agent, details, state_publisher, state_subscriber):
        #setup communication channels
        super(StateInterface, self).__init__(agent, details)
        global Publisher
        self.pub = Publisher(state_publisher, KeyValue, queue_size=5)
        global Subscriber
        self.sub = Subscriber(state_subscriber, KeyValue, self.callback, self.agent.agent_id)

    def callback(self, msg, agent_id):
        #recieve new states from remotes
        if msg.key == agent_id:
            state = msg.value.split('-')[0]
            if "_"+state in dir(self):
                logmsg(category="IDef", id=agent_id, msg="State changed to: %s" % state)
                getattr(self, "_"+state)()

    def notify(self, state):
        #publish state update to remote
        msg = KeyValue(key=self.agent.agent_id, value=state)
        logmsg(category="IDef", msg="Publishing: (%s)" % str(msg).replace('\n',' | '))
        self.pub.publish(msg)



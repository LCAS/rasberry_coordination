# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination_core.task_management import Stages
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.logmsg_utils import logmsg


class iFACE(object):

    def __repr__(self):
        return str(self.__class__).split('\'')[1].split('.')[-1]

    def __init__(self, agent, details=None):
        self.agent = agent
        self.details = details
        self.name = self.__repr__()

    def init(self, task_id=None, details=None, contacts=None, initiator_id=""):
        pass

    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        if len(self.agent.task_buffer) == 0:
            return(Task(id=task_id,
                        module='base',
                        name="idle",
                        details=details,
                        contacts=contacts,
                        initiator_id=self.agent.agent_id,
                        responder_id="",
                        stage_list=[
                            Stages['base']['StartTask'](self.agent, task_id),
                            Stages['base']['Idle'](self.agent)
                        ]))

    def release_task(self, agent):
        logmsg(category="DTM", msg="   | releasing task %s" % (self.agent['name']))
        self.agent.task = None

    def restart_task(self, agent):
        logmsg(category="DTM", msg="   | restarting task %s" % (self.agent['name']))
        self.agent.add_task(module='base', name=agent['name'], task_id=self.agent['id'], index=0, quiet=True)
        self.agent.task = None



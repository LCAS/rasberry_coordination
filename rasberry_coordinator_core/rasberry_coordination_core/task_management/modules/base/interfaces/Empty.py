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
from rasberry_coordination_core.logmsg_utils import logmsg


# Automanaged by rasberry_coordination_core.task_management.__init__.load_modules
# Interface class must be named `iFACE` to be recognised for import
# It will then be identifiable by its Interfaces[<<module>>][<<filename>>]
class iFACE(Interface):
    pass
#    def init(self, task_id=None, details=None, contacts=None, initiator_id=""):
#        return(Task(id=task_id,
#                    module='base',
#                    name="empty_init",
#                    details=details,
#                    contacts=contacts,
#                    initiator_id=self.agent.agent_id,
#                    responder_id="",
#                    stage_list=[
#                        Stages['base']['StartTask'](self.agent, task_id),
#                    ]))

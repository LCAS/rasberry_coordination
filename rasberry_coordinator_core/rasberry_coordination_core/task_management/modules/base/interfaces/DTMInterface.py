# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination_core.task_management.modules.base.interfaces.Interface import iFACE as Interface
from rasberry_coordination_core.task_management import Stages
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.logmsg_utils import logmsg


from diagnostic_msgs.msg import KeyValue

# Automanaged by rasberry_coordination_core.task_management.__init__.load_modules
# Interface class must be named `iFACE` to be recognised for import
# It will then be identifiable by its Interfaces[<<module>>][<<filename>>]
class iFACE(Interface):
    def __init__(self, agent, details, coordinator):
        super(StateInterface, self).__init__(agent, details)
        logmsg(category="SETUP", msg="DTM Initialised")
        self.coordinator = coordinator
        ns = "/rasberry_coordination"

        """ DTM Publishers """
        self.previous_task_list = None
        self.previous_task_list_2 = None
        global Publisher
        self.active_tasks_pub = Publisher('%s/active_tasks_details'%ns, TasksDetailsList, latch=True, queue_size=5)
        self.task_pause_pub = Publisher('%s/pause_state'%ns, Bool, queue_size=5, latch=True)

        """ DTM Dynamic Task Management """
        global Subscriber
        Subscriber('/rasberry_coordination/dtm', Interruption, self.InterruptTask)

        """ Reset the DTM Active Task List """
        self.ResetTaskList()

    """ Short-Definition Convenience Functions """
    def ResetTaskList(self):
        t = TasksDetailsList()
        task = SingleTaskDetails()
        task.task_id = "__RESET__"
        t.tasks.append(task)
        self.publish_task_list(t)
    def UpdateTaskList(self):
        task_list = self.generate_active_tasks_list()
        if self.previous_task_list != task_list and task_list.tasks:
            self.previous_task_list = task_list
            self.publish_task_list(task_list)
            return True
    def EndTask(self, E):
        task_list = self.generate_active_tasks_list()
        task_list.tasks = [t for t in task_list.tasks if t.task_id not in E]
        [task_list.tasks.append(self.generate_completed_task(task_id)) for task_id in E if task_id]
        self.publish_task_list(task_list)

    """ Active Task List Modifers """
    def generate_active_tasks_list(self):
        """ Publish updated list of Active Tasks to DTM """
        task_list = TasksDetailsList()
        for agent in self.coordinator.agent_manager.get_agent_list_copy().values():

            # If a task exists
            if agent['id']:

                # If task is already added, move on
                if agent['id'] in [T.task_id for T in task_list.tasks]: continue

                # Get task details
                task = SingleTaskDetails()
                task.task_id = agent['id']
                task.initiator_id = agent['initiator_id']
                task.responder_id = agent['responder_id']

                # Get state of task from initiator
                i = agent['initiator_id'] or agent.agent_id
                init = self.coordinator.agent_manager.agent_details[i]
                stage_repr = init().__repr__()
                if '(' in stage_repr:
                    stage_content = "(" + stage_repr.split('(')[-1]
                    stage_cls = stage_repr.split('(')[0].split('.')[-1]
                else:
                    stage_content = ""
                    stage_cls = stage_repr.split('.')[-1]
                task.state = stage_cls + stage_content.replace('()','')

                # Add task to list
                task_list.tasks.append(task)
        return task_list
    def generate_completed_task(self, task_id=None):
        task = SingleTaskDetails()
        task.task_id = task_id
        task.state = "CANCELLED"
        return task
    def publish_task_list(self, task_list):
        logmsg(level="info", category="SECT", id="SECTION", msg="\033[01;04;92mDTM\033[38;5;231m\033[0m")
        logmsg(category="DTM",  msg="Active Tasks:")
        [logmsg(category="DTM", msg="   | %s\t  -- %s [%s,%s]" % (t.task_id, t.state.replace("Idle", "\033[01;32mIdle\033[0m"), t.initiator_id, t.responder_id)) for t in task_list.tasks]
        # logmsg(category="null")
        self.active_tasks_pub.publish(task_list)

    """ Dynamic Task Management """
    def InterruptTask(self, m):
        logmsg(category="null")
        logmsg(category="DTM", id="DTM", msg="Interruption made on DTM channels of type: %s" % m.interrupt)
        A = {a.agent_id:a for a in self.coordinator.get_agents()}

        if m.scope in [0, "Coord", "Coordinator"]:
            # Modify all tasks
            logmsg(category="DTM", msg="    - to affect all agents.")
            if m.interrupt == "reset":
                for a in A.values():
                    if a['task_id'] and a.agent_id == a['initiator_id']:
                        logmsg(category="DTM", msg="      | release")
                        a.set_interrupt("reset", a.module, a['task_id'], m.scope, quiet=True)
            else:
                [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id']]


        elif m.scope in [1, "Task"]:
            # Modify all agents on specific task
            logmsg(category="DTM", msg="    - to affect task: %s." % m.target)
            # [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id'] and a['task_id'] == m.target]

            if m.interrupt == "reset":
                for a in A.values():
                    if (a['task_id']) and (a['task_id'] == m.target) and (a.agent_id == a['initiator_id']):
                        logmsg(category="DTM", msg="      | release")
                        a.set_interrupt("reset", a.module, a['task_id'], m.scope, quiet=True)
            else:
                [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id'] and a['task_id'] == m.target]



        elif m.scope in [2, "Agent"]:
            # Modify specific agent's task
            logmsg(category="DTM", msg="    - to affect agent: %s." % m.target)
            A[m.target].set_interrupt(m.interrupt, A[m.target].module, A[m.target]['task_id'], m.scope, quiet=True)

        else:
            print(m)


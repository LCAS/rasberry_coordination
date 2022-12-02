from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages


class Scheduler(Interface):

    def __init__(self, agent, details=None):
        super(self, Scheduler).__init__(agent=agent, details=details)
        self.schedule_sub = Subscriber('/rasberry_coordination/scheduler', TaskMsg, self.schedule_new_task)

    def schedule_new_task(self, msg):
        if msg.credentials.action == 'data_collection'
            self.agent.add_task(module="rasberry_transportation_pkg", name='request_collection')



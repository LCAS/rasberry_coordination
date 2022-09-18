from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages


class Robot(Interface):

    def init(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="init",
                     details=details,
                     contacts=contacts,
                     initiator_id=self.agent.agent_id,
                     responder_id="",
                     stage_list=[
                         Stages['base']['StartTask'](self.agent, task_id),
                         Stages['base']['SetUnregister'](self.agent),
                         Stages['base']['WaitForMap'](self.agent),
                         Stages['base']['WaitForLocalisation'](self.agent),
                         Stages['base']['SetRegister'](self.agent)
                     ]))

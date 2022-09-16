from rasberry_coordination.encapsuators import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.interface import Interface
from rasberry_coordination.task_management import Stages


class human(Interface):
    def __init__(self, agent):
        super(robot, self).__init__(agent)

    @classmethod
    def base_human_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.idle(agent=agent, task_id=task_id, details=details, contacts=contacts)

    @classmethod
    def base_human_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="base_human_init",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.SetUnregister(agent),
                        StageDef.WaitForMap(agent),
                        StageDef.SendInfo(agent),
                        StageDef.SetRegister(agent)
                    ]))

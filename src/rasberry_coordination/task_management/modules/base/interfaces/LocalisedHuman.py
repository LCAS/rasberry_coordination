from rasberry_coordination.encapsuators import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
#from rasberry_coordination.task_management import Stages


class LocalisedHuman(Interface):

    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return Interface.idle(agent=self.agent, task_id=task_id, details=details, contacts=contacts)

    def init(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="init",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages.StartTask(agent, task_id),
                        Stages.SetUnregister(agent),
                        Stages.WaitForMap(agent),
                        Stages.WaitForLocalisation(agent),
                        Stages.SendInfo(agent),
                        Stages.SetRegister(agent)
                    ]))



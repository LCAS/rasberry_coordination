from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages


class Robot(Interface):
    def wait_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="wait_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['assignment']['AssignNode'](self.agent),
                        Stages['navigation']['NavigateToNode'](self.agent),
                        Stages['base']['Idle'](self.agent)
                    ]))

    def exit_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="exit_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['SetUnregister'](self.agent),
                        Stages['navigation']['NavigateToExitNode'](self.agent),
                        Stages['base']['Exit'](self.agent)
                    ]))


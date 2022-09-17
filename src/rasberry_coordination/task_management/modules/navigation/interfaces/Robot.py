from rasberry_coordination.encapsuators import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
#from rasberry_coordination.task_management.__init__ import Stages


class robot(Interface):
    def wait_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="wait_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.AssignNode(agent),
                        StageDef.NavigateToNode(agent),
                        StageDef.Idle(agent)
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
                        StageDef.SetUnregister(agent),
                        StageDef.NavigateToExitNode(agent),
                        StageDef.Exit(agent)
                    ]))


from rasberry_coordination.encapsuators import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
#from rasberry_coordination.task_management import Stages


class Robot(Interface):
    def __init__(self, agent):
        super(robot, self).__init__(agent)
        self.agent.local_properties['load'] = 0

    def idle(self):
        load = self.agent.local_properties['load']
        max_load = self.agent.module_properties['rasberry_transportation']['max_load']
        return self.deliver_load() if int(load) >= int(max_load) else self.wait_at_base()

    def wait_at_base(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation',
                    name="wait_at_base",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.AssignBaseNodeIdle(agent),
                        StageDef.NavigateToBaseNodeIdle(agent),
                        StageDef.Idle(agent)
                    ]))

    def retrieve_load(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation',
                    name='retrieve_load',
                    details=details,
                    contacts=contacts,
                    initiator_id=initiator_id,
                    responder_id=self.agent.agent_id,
                    stage_list=[
                        Stages.base.StartTask(self.agent, task_id),
                        Stages.rasberry_transportation.NavigateToPicker(self.agent),
                        Stages.rasberry_transportation.Loading(self.agent)
                    ]))

    def deliver_load(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation',
                    name='deliver_load',
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id='',
                    stage_list=[
                        Stages.base.StartTask(self.agent, task_id),
                        Stages.rasberry_transportation.AssignFieldStorage(self.agent),
                        Stages.base.AssignWaitNode(self.agent),
                        Stages.rasberry_transportation.AwaitFieldStorageAccess(self.agent),
                        Stages.rasberry_transportation.NavigateToFieldStorage(self.agent),
                        Stages.rasberry_transportation.Unloading(self.agent)
                    ]))

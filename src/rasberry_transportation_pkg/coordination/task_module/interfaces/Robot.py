from rasberry_coordination.task_management.containers.Task import TaskObj as Task


from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages


class Robot(Interface):
    def __init__(self, agent, details):
        super(Robot, self).__init__(agent)
        self.agent.local_properties['load'] = 0

    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        load = self.agent.local_properties['load']
        max_load = self.agent.modules['rasberry_transportation_pkg'].details['max_load']
        return self.deliver_load() if int(load) >= int(max_load) else self.wait_at_base()

    def wait_at_base(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation_pkg',
                    name="wait_at_base",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['assignment']['AssignBaseNodeIdle'](self.agent),
                        Stages['navigation']['NavigateToBaseNodeIdle'](self.agent),
                        Stages['base']['Idle'](self.agent)
                    ]))

    def retrieve_load(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation_pkg',
                    name='retrieve_load',
                    details=details,
                    contacts=contacts,
                    initiator_id=initiator_id,
                    responder_id=self.agent.agent_id,
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['rasberry_transportation_pkg']['NavigateToPicker'](self.agent),
                        Stages['rasberry_transportation_pkg']['Loading'](self.agent)
                    ]))

    def deliver_load(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation_pkg',
                    name='deliver_load',
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id='',
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['rasberry_transportation_pkg']['AssignFieldStorage'](self.agent),
                        Stages['assignment']['AssignWaitNode'](self.agent),
                        Stages['rasberry_transportation_pkg']['AwaitFieldStorageAccess'](self.agent),
                        Stages['rasberry_transportation_pkg']['NavigateToFieldStorage'](self.agent),
                        Stages['rasberry_transportation_pkg']['Unloading'](self.agent)
                    ]))

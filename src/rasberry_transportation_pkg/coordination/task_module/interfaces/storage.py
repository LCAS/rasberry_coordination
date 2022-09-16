from rasberry_coordination.encapsuators import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces import StateInterface
from rasberry_coordination.task_management import Stages


class storage(StateInterface):

    def __init__(self, agent):
        state_publisher = agent.module_properties['rasberry_transportation']['state_publisher']
        state_subscriber = agent.module_properties['rasberry_transportation']['state_subscriber']
        super(storage, self).__init__(agent, state_publisher, state_subscriber)
        self.agent.local_properties['request_admittance'] = []
        self.notify("CONNECTED")

    def _uar_UNLOADED(self):
        self.agent['has_tray'] = True

    def idle(cls, task_id=None, details=None, contacts=None, initiator_id=""):
        #If agents are waiting to visit, accept one, otherwise wait
        if len(self.agent.local_properties['request_admittance']) > 0:
            return self.admit_dropoff(task_id=task_id, details=details, contacts=contacts)
        return self.wait_for_request(task_id=task_id, details=details, contacts=contacts)

    def wait_for_request(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation',
                    name="wait_for_request",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages.rasberry_transportation.StartTask(self.agent, task_id),
                        Stages.rasberry_transportation.IdleFieldStorage(self.agent)
                    ]))

    def admit_dropoff(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation',
                    name="admit_dropoff",
                    details=details,
                    contacts=contacts,
                    initiator_id="",
                    responder_id=self.agent.agent_id,
                    stage_list=[
                        Stages.rasberry_transportation.AcceptFieldCourier(self.agent),
                        Stages.rasberry_transportation.AwaitFieldCourier(self.agent),
                        Stages.rasberry_transportation.UnloadFieldCourier(self.agent)
                    ]))






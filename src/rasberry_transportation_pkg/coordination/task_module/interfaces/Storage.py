from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.StateInterface import StateInterface
from rasberry_coordination.task_management.__init__ import Stages

class Storage(StateInterface):

    def __init__(self, agent, details=None):
        state_publisher = details['state_publisher']
        state_subscriber = details['state_subscriber']
        super(Storage, self).__init__(agent, details, state_publisher, state_subscriber)
        self.details['request_admittance'] = []
        self.notify("CONNECTED")

    def _uar_UNLOADED(self):
        self.agent['has_tray'] = True

    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        #If agents are waiting to visit, accept one, otherwise wait
        if len(self.details['request_admittance']) > 0:
            return self.admit_dropoff(task_id=task_id, details=details, contacts=contacts)
        return self.wait_for_request(task_id=task_id, details=details, contacts=contacts)

    def wait_for_request(self, task_id=None, details=None, contacts=None, initiator_id=""):
        #from pprint import pprint; pprint(Stages)
        return(Task(id=task_id,
                    module=__name__.split('.')[0],
                    name="wait_for_request",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['rasberry_transportation_pkg']['IdleFieldStorage'](self.agent)
                    ]))

    def admit_dropoff(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module=__name__.split('.')[0],
                    name="admit_dropoff",
                    details=details,
                    contacts=contacts,
                    initiator_id="",
                    responder_id=self.agent.agent_id,
                    stage_list=[
                        Stages['rasberry_transportation_pkg']['AcceptFieldCourier'](self.agent),
                        Stages['rasberry_transportation_pkg']['AwaitFieldCourier'](self.agent),
                        Stages['rasberry_transportation_pkg']['UnloadFieldCourier'](self.agent)
                    ]))






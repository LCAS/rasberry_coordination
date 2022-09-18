from  diagnostic_msgs.msg import KeyValue

from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.StateInterface import StateInterface
from rasberry_coordination.task_management.__init__ import Stages


class Picker(StateInterface):

    def __init__(self, agent, details):
        print(details)
        state_publisher = details['state_publisher']
        state_subscriber = details['state_listener']
        super(Picker, self).__init__(agent, details, state_publisher, state_subscriber)
        self.notify("CONNECTED")

    def _car_CALLED(self):
        self.agent.add_task(module="rasberry_transportation", name='request_collection')
        self.agent['start_time'] = Time.now()

    def _car_LOADED(self):
        self.agent['has_tray'] = False

    def _car_CANCEL(self):
        if self.agent['id'] and \
           self.agent['name']=='request_collection' and \
           self.agent['module']=='rasberry_transportation':
            self.agent.set_interrupt('reset', 'rasberry_transportation', self.agent['id'], "Task")

    def loc_cb(self, msg):
        #republish location to car interface
        self.loc_pub.publish(KeyValue(key=self.agent.agent_id, value=msg.data))

    def request_collection(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_transportation',
                    name="request_collection",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages.base.StartTask(self.agent, task_id),
                        Stages.rasberry_transportation.AssignFieldCourier(self.agent),
                        Stages.rasberry_transportation.AwaitFieldCourier(self.agent),
                        Stages.rasberry_transportation.LoadFieldCourier(self.agent),
                    ]))





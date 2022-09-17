
So the best way for it to work generally moving forward...


python_pkg.coordination.task_module
- stage_definitions
- task_definitions
- interface_definitions



each file in coordinator calls
from coordinator.task_management import compile_all_task_modules; # ~= compile_all_task_modules()
from coordinator.task_management import StageDef, TtaskDef, InterfaceDef


StageDef = [<python_pkg>_<stage_name>, ...]
TtaskDef = [<python_pkg>_<task_name>, ...]
InterfaceDef = [<python_pkg>_<interface_name>, ...]

This approach allows the following: 
    - getattr(StageDef, "navigation_NavigateToAgent")
    - TtaskDef.navigation_robot_idle

If we were to generate dictionaries we could use:
    - Interfaces.transportation.robot.tasks.idle
    - Interfaces.transportation.robot.tasks.init
    - Interfaces.transportation.robot.callbacks.pause
    - Interfaces.transportation.tasks.transportation_retrieve_load
    - Interfaces.transportation.stages.NavigateToPicker
    - Interfaces.base.agent.idle



------------------------------------------
class Interface(object):

    @abstractmethod
    def __init__(self, agent): 
        self.tasks = dict()
        self.agent = agent

    def __getitem__(self, item):
        return None if item not in self.tasks else self.tasks[item]

    def __getattribute__(self, item): 
        return self.__getitem__(item)

    @abstractmethod
    def init(self): pass

    @abstractmethod
    def idle(self): pass

------------------------------------------

agent: self.interfaces.add(interface(self, properties[module]))



------------------------------------------
rasberry_TRANSPORTATION.coordination.task_module.interfaces.ROBOT.py

from rasberry_transportation.coordination.task_module.interfaces.ROBOT import robot
from rasberry_transportation.coordination.task_module.interfaces.PICKER import picker
from rasberry_transportation.coordination.task_module.interfaces.STORAGE import storage

from rasberry_coordination.task_management import Interface
from rasberry_coordination.task_management import Stages

class robot(Interface):
    def __init__(self, agent):
        super(robot, self).__init__(agent)
        if 'load' not in self.agent.local_properties:
            self.agent.local_properties['load'] = 0

    @taskmethod
    def idle(self):
        LP = self.agent.local_properties
        MP = self.agent.module_properties
        self.agent.local_properties['load'] = int(self.agent.local_properties['load'])
        if LP['load'] >= int(MP['max_load']):
            return self.transportation_deliver_load()

    @taskmethod
    def retrieve_load(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name='retrieve_load',
                    details=details,
                    contacts=contacts,
                    initiator_id=initiator_id,
                    responder_id=self.agent.agent_id,
                    stage_list=[
                        StageDef.StartTask(self.agent, task_id),
                        StageDef.NavigateToPicker(self.agent),
                        StageDef.Loading(self.agent)
                    ]))

    @taskmethod
    def deliver_load(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name='deliver_load',
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id='',
                    stage_list=[
                        StageDef.StartTask(self.agent, task_id),
                        StageDef.AssignFieldStorage(self.agent),
                        StageDef.AssignWaitNode(self.agent),
                        StageDef.AwaitFieldStorageAccess(self.agent),
                        StageDef.NavigateToFieldStorage(self.agent),
                        StageDef.Unloading(self.agent)
                    ]))





class StateInterface(Interface):
    def __init__(self, agent, properties):
        super(StateInterface, self).__init__(agent, properties)

        state_publisher = agent.module_properties['rasberry_transportation']['state_publisher']
        self.pub = Publisher(state_publisher, KeyValue, queue_size=5)

        state_subscriber = agent.module_properties['rasberry_transportation']['state_subscriber']
        self.sub = Subscriber(state_subscriber, KeyValue, self.callback, self.agent.agent_id)

        self.notify("CONNECTED")

    def callback(self, msg, agent_id):
        #recieve new states from remotes
        if msg.key == agent_id:
            state = msg.value.split('-')[0]
            if state in dir(self):
                logmsg(category="IDef", id=agent_id, msg="State changed to: %s" % state)
                self.msg = msg
                getattr(self, state)()

    def notify(self, state):
        #publish state update to remote
        msg = KeyValue(key=self.agent.agent_id, value=state)
        logmsg(category="IDef", msg="        - Publishing: (%s)" % str(msg).replace('\n',' | '))
        self.pub.publish(msg)




class picker(StateInterface):

    def _car_CALLED(self):
        self.agent.add_task(task_name='rasberry_transportation_request_field_courier')
        self.agent['start_time'] = Time.now()

    def _car_LOADED(self):
        self.agent['has_tray'] = False

    def _car_CANCEL(self):
        if self.agent['id'] and self.agent['name']=='rasberry_transportation_request_field_courier':
            self.agent.set_interrupt('reset', 'rasberry_transportation', self.agent['id'], "Task")

    def loc_cb(self, msg):
        #republish location to car interface
        self.loc_pub.publish(KeyValue(key=self.agent.agent_id, value=msg.data))

    @taskmethod
    def request_collection(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_request_field_courier",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignFieldCourier(agent),
                        StageDef.AwaitFieldCourier(agent),
                        StageDef.LoadFieldCourier(agent),
                    ]))





class storage(Interface):
    def __init__(self, agent, sub='/uar/get_states', pub='/uar/set_states'):
        self.release_triggers = ['self', 'toc']
        self.restart_triggers = ['thorvald']

        responses={'UNLOADED': self.unloaded}
        from pprint import pprint
        pprint(dir(IDef))
        print("\n")
        pprint(dir(InterfaceDef))
        super(InterfaceDef.rasberry_transportation_field_storage, self).__init__(agent, responses, sub=sub, pub=pub)

        #These need a new home
        self.agent.request_admittance = []

    def unloaded(self): 
        self.agent['has_tray'] = True



    @classmethod
    def transportation_field_storage_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        #If agents are waiting to visit, begin transportation field_storage
        #Otherwise wait idle
        if len(agent.request_admittance) > 0:
            return TtaskDef.transportation_field_storage(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            return TtaskDef.wait_for_request(agent=agent, task_id=task_id, details=details, contacts=contacts)


    @classmethod
    def wait_for_request(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="idle_field_storage_def",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.IdleFieldStorage(agent)
                    ]))
    @classmethod
    def admit_dropoff(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_field_storage",
                    details=details,
                    contacts=contacts,
                    initiator_id="",
                    responder_id=agent.agent_id,
                    stage_list=[
                        StageDef.AcceptFieldCourier(agent),
                        StageDef.AwaitFieldCourier(agent),
                        StageDef.UnloadFieldCourier(agent)
                    ]))



-----------------------------------------------



TOC

class base_human(object):
    def __init__(self, agent):
        self.agent = agent

class base_localised_human(base_human):
    pass



















Interfaces:

  BASE:
    robot
      - idle
    human
    localised_human

#  NAV:
#    robot
#      - exit_at_node
#      - wait_at_node

#  TP:
#    robot
#      - wait_at_base
#    picker
#    storage

#-  DC:
#-    robot
#-      - wait_at_dc_base
#-    controller



> coor.task.mod.base.human.human()
Interfaces['base']['Human']()

> coor.task.mod.base.stages.Idle()
Stages['base']['Idle']()

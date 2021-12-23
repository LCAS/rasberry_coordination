from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass
from rasberry_coordination.robot import Robot as RobotInterface_Old

from rospy import Time, Duration


class InterfaceDef(object):

    class transportation_picker(IDef.AgentInterface):
        def __init__(self, agent, sub='/car_client/get_states', pub='/car_client/set_states'):
            # self.release_triggers = ['self', 'toc']
            # self.restart_triggers = ['thorvald']

            responses = {'CALLED': self.called, 'LOADED': self.loaded, 'INIT': self.reset}
            super(InterfaceDef.transportation_picker, self).__init__(agent, responses, sub=sub, pub=pub)

            self.notify("INIT")

        def called(self):
            logmsg(category="TPTask", id=self.agent.agent_id, msg="Request for courier")
            self.agent.add_task(task_name='transportation_request_courier')
            self.agent['start_time'] = Time.now()

        def loaded(self):
            logmsg(msg="picker called loaded")
            self.agent['has_tray'] = False #picker no longer has the tray

        def reset(self):
            logmsg(category="TPTask", id=self.agent.agent_id, msg="Reset-task requested")
            if self.agent['id'] and self.agent['name']=='transportation_request_courier':
                self.agent.set_interrupt('reset', 'transportation', self.agent['id'], "Task")
            else:
                logmsg(level="error", msg="task request cancelled but we only check for if task is active")

        # def on_cancel(self, task_id, contact_id, force_release=False):
        #     old_id = super(InterfaceDef.transportation_picker, self).on_cancel(task_id=task_id, contact_id=contact_id, force_release=force_release)
        #     if old_id == task_id: self.notify("INIT")

    class transportation_courier(IDef.AgentInterface):
        def __init__(self, agent, sub='/r/get_states', pub='/r/set_states'):
            #E.g. If a cancellation request is triggered by picker, we much release
            #E.g. If a cancellation request is triggered by storage, we much restart
            self.release_triggers = ['self', 'picker', 'toc']
            self.restart_triggers = ['storage']

            responses={'PAUSE':self.pause, 'UNPAUSE':self.unpause, 'RELEASE':self.release}
            super(InterfaceDef.transportation_courier, self).__init__(agent, responses, sub=sub, pub=pub)

        def pause(self): self.agent.set_interrupt('pause', 'transportation', self.agent['id'])
        def unpause(self): self.agent.set_interrupt('unpause', 'transportation', self.agent['id'])
        def release(self): self.agent.set_interrupt('reset', 'transportation', self.agent['id'])

    class transportation_storage(IDef.AgentInterface):
        def __init__(self, agent, sub='/uar/get_states', pub='/uar/set_states'):
            self.release_triggers = ['self', 'toc']
            self.restart_triggers = ['thorvald']

            responses={'UNLOADED': self.unloaded}
            super(InterfaceDef.transportation_storage, self).__init__(agent, responses, sub=sub, pub=pub)

            #These need a new home
            self.agent.request_admittance = []
            self.agent.has_presence = False  # used for routing (swap key for physical?)

        def unloaded(self): self.agent['has_tray'] = True

        # def on_cancel(self, task_id, contact_id, force_release=False):
        #     super(InterfaceDef.transportation_storage, self).on_cancel(task_id=task_id, contact_id=contact_id, force_release=force_release)


class TaskDef(object):

    """ Initialisation """
    # @classmethod
    # def transportation_picker_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    #     return TDef.human_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_courier_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        if 'load' not in agent.local_properties: agent.local_properties['load'] = 0


    """ Idle Task Stages for Transportation Agents """
    @classmethod
    def transportation_picker_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        agent.modules['transportation'].interface.notify("INIT")
    @classmethod
    def transportation_courier_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        LP = agent.local_properties
        MP = agent.module_properties

        #If agent is at max capacity deliver load
        agent.local_properties['load'] = int(agent.local_properties['load'])
        if LP['load'] >= int(MP['max_load']):
            return TaskDef.transportation_deliver_load(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_storage_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        #If agents are waiting to visit, begin transportation storage
        #Otherwise wait idle
        if len(agent.request_admittance) > 0:
            return TaskDef.transportation_storage(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            return TaskDef.idle_storage_def(agent=agent, task_id=task_id, details=details, contacts=contacts)


    """ Picker Tasks """
    @classmethod
    def transportation_request_courier(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id = task_id,
                    module = 'transportation',
                    name = "transportation_request_courier",
                    details = details,
                    contacts = contacts,
                    initiator_id = agent.agent_id,
                    responder_id = "",
                    stage_list = [
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignCourier(agent),
                        StageDef.AwaitCourier(agent),
                        StageDef.LoadCourier(agent),
                    ]))

    """ Courier Tasks """
    @classmethod
    def transportation_retrieve_load(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id = task_id,
                    module = 'transportation',
                    name = "transportation_retrieve_load",
                    details = details,
                    contacts = contacts,
                    initiator_id = initiator_id,
                    responder_id = agent.agent_id,
                    stage_list = [
                        SDef.StartTask(agent, task_id),
                        StageDef.NavigateToPicker(agent),
                        StageDef.Loading(agent)
                    ]))
    @classmethod
    def transportation_deliver_load(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id = task_id,
                    module = 'transportation',
                    name = "transportation_deliver_load",
                    details = details,
                    contacts = contacts,
                    initiator_id = agent.agent_id,
                    responder_id = "",
                    stage_list = [
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignStorage(agent),
                        SDef.AssignWaitNode(agent),
                        StageDef.AwaitStoreAccess(agent),
                        StageDef.NavigateToStorage(agent),
                        StageDef.Unloading(agent)
                    ]))

    """ Storage Tasks """
    @classmethod
    def idle_storage_def(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id = task_id,
                    module = 'transportation',
                    name = "idle_storage_def",
                    details = details,
                    contacts = contacts,
                    initiator_id = agent.agent_id,
                    responder_id = "",
                    stage_list = [
                        SDef.StartTask(agent, task_id),
                        StageDef.IdleStorage(agent)
                    ]))
    @classmethod
    def transportation_storage(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id = task_id,
                    module = 'transportation',
                    name = "transportation_storage",
                    details = details,
                    contacts = contacts,
                    initiator_id = "",
                    responder_id = agent.agent_id,
                    stage_list = [
                        StageDef.AcceptCourier(agent),
                        StageDef.AwaitCourier(agent),
                        StageDef.UnloadCourier(agent)
                    ]))


class StageDef(object):

    class IdleStorage(SDef.Idle):
        """Used to Idle a storage agent whilst awaiting a request for admittance"""
        #TODO: change this to idle, and make this condition an interface response?
        def _query(self):
            """Complete once there exists any agents requiring storage"""
            success_conditions = [len(self.agent.request_admittance) > 0] #TODO: this may prove error prone w/ _start
            self.flag(any(success_conditions))
            if any(success_conditions): print("admittance required: %s"%self.agent.request_admittance)
        def _end(self):
            """On completion, add an idle storage to the end of the buffer"""
            self.agent.add_task('transportation_storage_idle')

    """ Assignment-Based Task Stages (involves coordinator) """
    class AssignCourier(SDef.AssignAgent):
        """Used to identify an available courier to collect a load"""
        def _start(self):
            """Initiate action details to identify the closest courier"""
            self.super()._start() #defined as default setup
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['agent_type'] = 'courier'
            self.action['response_location'] = None
        def _end(self):
            """ On completion, notify picker of courier acceptance, and assign a retrieve load task to the courier"""
            self.super()._end()
            self.agent.modules['transportation'].interface.notify("ACCEPT")
            self.agent['contacts']['courier'] = self.action['response_location']
            self.agent['contacts']['courier'].add_task(task_name='transportation_retrieve_load',
                                                         task_id=self.agent['id'],
                                                         details={},
                                                         contacts={'picker': self.agent},
                                                         initiator_id=self.agent.agent_id)
            self.agent['responder_id'] = self.agent['contacts']['courier'].agent_id
    class AssignStorage(SDef.AssignAgent):
        """Used to identify a storage location to deliver the load"""
        def _start(self):
            """Initiate action details to identify the closest storage"""
            self.super()._start()
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['agent_type'] = 'storage'
            self.action['response_location'] = None
        def _end(self):
            """On completion, save the storage and add the courier's id to the storage's request_admittance list"""
            self.super()._end()
            self.agent['contacts']['storage'] = self.action['response_location']
            self.agent['contacts']['storage'].request_admittance.append(self.agent.agent_id)
            self.agent['responder_id'] = self.agent['contacts']['storage'].agent_id
    class AcceptCourier(SDef.AssignAgent):
        """Used to notify a pending courier of admittance"""
        def _start(self):
            """Initiate action details to identify the closest courier from those that request admittance"""
            self.super()._start()
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['list'] = self.agent.request_admittance
            self.action['response_location'] = None
        def _end(self):
            """On completion, save the action response and remove the courier from the request_admittance list"""
            self.super()._end()
            self.agent['contacts']['courier'] = self.action['response_location']
            logmsg(category="stage", msg="Admitted: %s from %s" % (self.agent['contacts']['courier'].agent_id, self.agent.request_admittance))
            logmsg(category="stage", msg="AcceptCourier: stage_complete=%s" % self.stage_complete)
            self.agent.request_admittance.remove(self.agent['contacts']['courier'].agent_id)
            self.agent['initiator_id'] = self.agent['contacts']['courier'].agent_id

    """ Idle Actions for Pending Actions """
    class AwaitCourier(SDef.Idle):   #PICKER + STORAGE
        """Used to idle the agent until a courier has arrived"""
        def __repr__(self):
            """Attach id of agent to class name"""
            if 'courier' in self.agent['contacts']:
                return "%s(%s)"%(self.get_class(), self.agent['contacts']['courier'].agent_id)
            else:
                return "%s()" % (self.get_class())
        def _query(self):
            """Complete once the associated courier has arrived at the agents location"""
            success_conditions = [self.agent['contacts']['courier'].location(accurate=True) == self.agent.location()]
            self.flag(any(success_conditions))
        def _end(self):
            """On completion, notify the picker of ARRIVAL"""
            self.agent.modules['transportation'].interface.notify("ARRIVED")
    class AwaitStoreAccess(SDef.Idle):
        """Used to idle the courier until the storage location has accepted admittance"""
        def __repr__(self):
            """Attach id of agent to class name"""
            if 'storage' in self.agent['contacts']:
                return "%s(%s)"%(self.get_class(), self.agent['contacts']['storage'].agent_id)
            else:
                return "%s()" % (self.get_class())
        # def _start(self):
        #     self.super()._start()
        #     self.route_required = True
        #     Despite moving to a wait node, dont end task on arrival #huh?
        def _query(self):
            """Complete if the courier assigned to the storage of interest is this agent"""
            storage = self.agent['contacts']['storage']
            if 'courier' not in storage['contacts']: return
            courier = storage['contacts']['courier']
            success_conditions = [courier.agent_id == self.agent.agent_id]
            self.flag(any(success_conditions))

    """ Transportation Navigation Subclasses """
    class NavigateToPicker(SDef.NavigateToAgent):
        """Used to define the target for the navigation as the picker"""
        def __init__(self, agent):
            """Set navigation target as associated picker"""
            self.super().__init__(agent,  association='picker')
    class NavigateToStorage(SDef.NavigateToAgent):
        """Used to define the target for the navigation as the storage"""
        def __init__(self, agent):
            """Set navigation target as associated storage"""
            self.super().__init__(agent, association='storage')

    """ Courier Load Modifiers for Picker and Storage """
    class LoadModifier(SDef.StageBase):
        """Used to idle whilst the agent modifes the load of the courier"""
        def __init__(self, agent, end_requirement, wait_timeout=10):
            """Initialise placeholders for a completion flag and a response timeout"""
            self.super().__init__(agent)
            self.end_requirement = end_requirement
            self.wait_timeout = Duration(secs=wait_timeout)
        def _start(self):
            """Set the has_tray flag to match the inverse end_requirement"""
            self.super()._start()
            self.agent['has_tray'] = not self.end_requirement  # local flag
        def _query(self):
            """Complete once agents's has_tray flag is triggered or timeout completes"""
            success_conditions = [Time.now() - self.start_time > self.wait_timeout,
                                  self.agent['has_tray'] == self.end_requirement]
            self.flag(any(success_conditions))
        def _end(self):
            """On Completion, set the associated courier's has_tray flag"""
            self.super()._end()
            self.agent['contacts']['courier']['has_tray'] = not self.end_requirement
    class LoadCourier(LoadModifier): #PICKER
        """Used for loading the courier"""
        def __init__(self, agent):
            """Define the completion flag as False and the timeout as the tranportation/wait_loading property"""
            self.super().__init__(agent, end_requirement=False, wait_timeout=fetch_property('transportation', 'wait_loading'))
        def _end(self):
            """On completion, notify the picker to reset with INIT"""
            self.agent.modules['transportation'].interface.notify("INIT")
    class UnloadCourier(LoadModifier): #STORAGE
        """Used for unloading the courier"""
        def __init__(self, agent):
            """Define the completion flag as True and the timeout as the tranportation/wait_unloading property"""
            self.super().__init__(agent, end_requirement=True, wait_timeout=fetch_property('transportation', 'wait_unloading'))

    """ Loading Modifiers for Courier """
    class Loading(SDef.StageBase):
        """Used for awaiting a change-of-state from the picker"""
        def _query(self):
            """Complete once agents's has_tray flag is true"""
            success_conditions = [self.agent['has_tray'] == True]
            self.flag(any(success_conditions))
        def _end(self):
            """On completion, increment the courier's total load by 1"""
            self.super()._end()
            self.agent.local_properties['load'] += 1
            # logmsg(level='warn', category='STAGE', id=self.agent.agent_id, msg="Total load: %s" % self.agent.properties['load'])
    class Unloading(SDef.StageBase):
        """Used for awaiting a change-of-state from the storage"""
        def _query(self):
            """Complete once agents's has_tray flag is false"""
            success_conditions = [self.agent['has_tray'] == False]
            self.flag(any(success_conditions))
        def _end(self):
            """On completion, reset the courier's total load to 0"""
            self.super()._end()
            self.agent.local_properties['load'] = 0
            # logmsg(level='warn', category='STAGE', id=self.agent.agent_id, msg="Total load: %s" % self.agent.properties['load'])

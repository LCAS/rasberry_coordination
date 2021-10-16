from std_msgs.msg import String as Str
from rasberry_coordination.coordinator_tools import logmsg
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef
from rasberry_coordination.robot import Robot as RobotInterface_Old

from rospy import Time, Duration


class InterfaceDef(object):

    class transportation_picker(IDef.AgentInterface):
        def __init__(self, agent, sub='/car_client/get_states', pub='/car_client/set_states'):
            self.release_triggers = ['self', 'toc']
            self.restart_triggers = ['thorvald']

            responses = {'CALLED': self.called, 'LOADED': self.loaded, 'INIT': self.reset}
            super(InterfaceDef.transportation_picker, self).__init__(agent, responses, sub=sub, pub=pub)

            self.notify("INIT")

        def called(self):
            logmsg(msg="picker called called")
            self.agent.add_task(task_name='transportation_request_courier')
            self.agent['start_time'] = Time.now()

        def loaded(self):
            logmsg(msg="picker called loaded")
            self.agent['has_tray'] = False #picker no longer has the tray

        def reset(self):
            logmsg(msg="picker called reset")
            if self.agent['task_id'] and self.agent.task_name=='transportation_request_courier':
                self.agent.set_interrupt('reset', 'transportation', self.agent['task_id'], "Task")

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

            #TODO: These need a new home (do we need a robot core_task_module?)
            self.agent.temp_interface = RobotInterface_Old(self.agent.agent_id)

        def pause(self): self.agent.set_interrupt('pause', 'transportation', self.agent['task_id'])
        def unpause(self): self.agent.set_interrupt('unpause', 'transportation', self.agent['task_id'])
        def release(self): self.agent.set_interrupt('reset', 'transportation', self.agent['task_id'])

        # def on_cancel(self, task_id, contact_id, force_release=False):
        #     old_id = super(InterfaceDef.transportation_courier, self).on_cancel(task_id=task_id, contact_id=contact_id, force_release=force_release)
        #     if old_id == task_id: self.agent.temp_interface.cancel_execpolicy_goal()

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

    """ Initialisation Verification """
    @classmethod
    def transportation_courier_init(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return TDef.robot_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_picker_init(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return TDef.human_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)


    """ Initial Task Stages for Transportation Agents """
    @classmethod
    def transportation_picker_idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        agent.interfaces['transportation'].notify("INIT")
        return TDef.idle(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_courier_idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        AP = agent.properties

        #If agent can carry more, wait at base
        #Otherwise deliver load
        agent.properties['load'] = int(agent.properties['load'])
        if AP['load'] < int(AP['max_load']):
            return TDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            return TaskDef.transportation_deliver_load(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_storage_idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        #If agents are waiting to visit, begin transportation storage
        #Otherwise wait idle
        if len(agent.request_admittance) > 0:
            return TaskDef.transportation_storage(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            return TaskDef.idle_storage_def(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def idle_storage_def(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "idle_storage",
                'details': TDef.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'transportation',
                'initiator_id': agent.agent_id,
                'responder_id': "",
                'stage_list': [
                    SDef.StartTask(agent, task_id),
                    StageDef.IdleStorage(agent)
                ]})


    """ Picker Logistics Transportation """
    @classmethod
    def transportation_request_courier(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "transportation_request_courier",
                'details': TDef.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'transportation',
                'initiator_id': agent.agent_id,
                'responder_id': "",
                'stage_list': [
                    SDef.StartTask(agent, task_id),
                    StageDef.AssignCourier(agent),
                    StageDef.AwaitCourier(agent),
                    StageDef.LoadCourier(agent),
                ]})
    @classmethod
    def transportation_retrieve_load(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "transportation_retrieve_load",
                'details': TDef.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'transportation',
                'initiator_id': initiator_id,
                'responder_id': agent.agent_id,
                'stage_list': [
                    SDef.StartTask(agent, task_id),
                    StageDef.NavigateToPicker(agent),
                    StageDef.Loading(agent)
                ]})
    @classmethod
    def transportation_deliver_load(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "transportation_deliver_load",
                'details': TDef.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'transportation',
                'initiator_id': agent.agent_id,
                'responder_id': "",
                'stage_list': [
                    SDef.StartTask(agent, task_id),
                    StageDef.AssignStorage(agent),
                    SDef.AssignWaitNode(agent),
                    StageDef.AwaitStoreAccess(agent),
                    StageDef.NavigateToStorage(agent),
                    StageDef.Unloading(agent)
                ]})
    @classmethod
    def transportation_storage(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "transportation_storage",
                'details': TDef.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'transportation',
                'initiator_id': "",
                'responder_id': agent.agent_id,
                'stage_list': [
                    StageDef.AcceptCourier(agent),
                    StageDef.AwaitCourier(agent),
                    StageDef.UnloadCourier(agent)
                ]})


class StageDef(object):

    #TODO: change this to idle, and make this condition an interface response?
    class IdleStorage(SDef.Idle):
        def _query(self):
            success_conditions = [len(self.agent.request_admittance) > 0] #TODO: this may prove error prone w/ _start
            self._flag(any(success_conditions))
            if any(success_conditions): print("admittance required: %s"%self.agent.request_admittance)
        def _end(self):
            self.agent.add_task('transportation_storage_idle')
        def _summary(self):
            super(StageDef.IdleStorage, self)._summary()
            self.summary['_query'] = 'len(store.request_admittance) > 0'
            self.summary['_del'] = 'begin task'


    """ Assignment-Based Task Stages (involves coordinator) """
    class AssignCourier(SDef.AssignAgent):
        def _start(self):
            super(StageDef.AssignCourier, self)._start() #defined as default setup
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['agent_type'] = 'courier'
            self.action['response_location'] = None
        def _notify_end(self):
            self.agent.interfaces['transportation'].notify("ACCEPT")
        def _end(self):
            """ On completion of assign courier, a courier should have been identified.
            As a result of the completion, the couier should be assigned a task, and be
            given details pertaining to its completion. This is done explicitly.
            """
            super(StageDef.AssignCourier, self)._end()
            self.agent.task_contacts['courier'] = self.action['response_location']
            self.agent.task_contacts['courier'].add_task(task_name='transportation_retrieve_load',
                                                         task_id=self.agent['task_id'],
                                                         details={},
                                                         contacts={'picker': self.agent},
                                                         initiator_id=self.agent.agent_id)
            self.agent.responder_id = self.agent.task_contacts['courier'].agent_id

        def _summary(self):
            super(StageDef.AssignCourier, self)._summary()
            self.summary['_start'] = "setup action to find courier"
            self.summary['_action'] = "find closest robot courier"
            self.summary['_del'] = "agent[courier].add_task"
            self.summary['_notify_end'] = "agent -> ACCEPT"
    class AssignStorage(SDef.AssignAgent):
        def _start(self):
            super(StageDef.AssignStorage, self)._start()
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['agent_type'] = 'storage'
            self.action['response_location'] = None
        def _end(self):
            super(StageDef.AssignStorage, self)._end()
            self.agent.task_contacts['storage'] = self.action['response_location']
            self.agent.task_contacts['storage'].request_admittance.append(self.agent.agent_id)
            self.agent.responder_id = self.agent.task_contacts['storage'].agent_id

        def _summary(self):
            super(StageDef.AssignStorage, self)._summary()
            self.summary['_start'] = "setup action to find storage"
            self.summary['_action'] = "find closest local storage"
            self.summary['_del'] = "contact[storage].request_admittance"
    class AcceptCourier(SDef.AssignAgent):
        def _start(self):
            super(StageDef.AcceptCourier, self)._start()
            self.action['action_type'] = 'find_agent_from_list'
            self.action['action_style'] = 'closest'
            self.action['list'] = self.agent.request_admittance
            self.action['response_location'] = None
        def _end(self):
            super(StageDef.AcceptCourier, self)._end()
            self.agent.task_contacts['courier'] = self.action['response_location']
            logmsg(category="stage", msg="Admitted: %s from %s" % (self.agent.task_contacts['courier'].agent_id, self.agent.request_admittance))
            logmsg(category="stage", msg="AcceptCourier: stage_complete=%s" % self.stage_complete)
            self.agent.request_admittance.remove(self.agent.task_contacts['courier'].agent_id)
            self.agent.initiator_id = self.agent.task_contacts['courier'].agent_id
        def _summary(self):
            super(StageDef.AcceptCourier, self)._summary()
            self.summary['_start'] = "setup action to find admittant"
            self.summary['_action'] = "find closest robot request"
            self.summary['_del'] = "contact[courier].request_admittance"

    """ Idle Actions for Pending Actions """
    class AwaitCourier(SDef.Idle):   #PICKER + STORAGE
        def __repr__(self):
            if 'courier' in self.agent.task_contacts:
                return "%s(%s)"%(self.get_class(), self.agent.task_contacts['courier'].agent_id)
            else:
                return "%s()" % (self.get_class())
        def _query(self):
            success_conditions = [self.agent.task_contacts['courier'].location(accurate=True) == self.agent.location()]
            self._flag(any(success_conditions))
        def _notify_end(self):
            self.agent.interfaces['transportation'].notify("ARRIVED")
        def _summary(self):
            super(StageDef.AwaitCourier, self)._summary()
            self.summary['_query'] = "courier @ agent"
            self.summary['_notify_end'] = "agent -> ARRIVED"
    class AwaitStoreAccess(SDef.Idle):
        #While waiting for store access, move to a wait_node
        # (ideally this should be identified by the coordinator and assigned dynamically)
        def __repr__(self):
            if 'storage' in self.agent.task_contacts:
                return "%s(%s)"%(self.get_class(), self.agent.task_contacts['storage'].agent_id)
            else:
                return "%s()" % (self.get_class())
        def _start(self):
            super(StageDef.AwaitStoreAccess, self)._start()
            #self.route_required = True
            # Despite moving to a wait node, dont end task on arrival #huh?
        def _query(self):
            """ If a courier is assigned as the contact to the storage contact, check if it is this stage's owner """
            storage = self.agent.task_contacts['storage']
            if 'courier' not in storage.task_contacts: return
            courier = storage.task_contacts['courier']
            success_conditions = [courier.agent_id == self.agent.agent_id]
            self._flag(any(success_conditions))

    """ Transportation Navigation Subclasses """
    class NavigateToPicker(SDef.NavigateToAgent):
        def __init__(self, agent): super(StageDef.NavigateToPicker, self).__init__(agent,  association='picker')
    class NavigateToStorage(SDef.NavigateToAgent):
        def __init__(self, agent): super(StageDef.NavigateToStorage, self).__init__(agent, association='storage')

    """ Courier Load Modifiers for Picker and Storage """
    class LoadModifier(SDef.StageBase):
        def __init__(self, agent, end_requirement, wait_timeout=10):
            super(StageDef.LoadModifier, self).__init__(agent)
            self.end_requirement = end_requirement
            self.wait_timeout = Duration(secs=wait_timeout)
        def _start(self):
            super(StageDef.LoadModifier, self)._start()
            self.agent['has_tray'] = not self.end_requirement  # local flag
        def _query(self):
            success_conditions = [Time.now() - self.start_time > self.wait_timeout,
                                  self.agent['has_tray'] == self.end_requirement]
            self._flag(any(success_conditions))
        def _end(self):
            super(StageDef.LoadModifier, self)._end()
            self.agent.task_contacts['courier']['has_tray'] = not self.end_requirement
    class LoadCourier(LoadModifier): #PICKER
        def __init__(self, agent):
            super(StageDef.LoadCourier, self).__init__(agent, end_requirement=False, wait_timeout=PDef['transportation']['wait_loading'])
        def _notify_end(self):
            self.agent.interfaces['transportation'].notify("INIT")
    class UnloadCourier(LoadModifier): #STORAGE
        def __init__(self, agent):
            super(StageDef.UnloadCourier, self).__init__(agent, end_requirement=True, wait_timeout=PDef['transportation']['wait_unloading'])

    """ Loading Modifiers for Courier """
    class Loading(SDef.StageBase):
        def _query(self):
            success_conditions = [self.agent['has_tray'] == True]
            self._flag(any(success_conditions))
        def _end(self):
            super(StageDef.Loading, self)._end()
            self.agent.properties['load'] += 1
            # logmsg(level='warn', category='STAGE', id=self.agent.agent_id, msg="Total load: %s" % self.agent.properties['load'])
    class Unloading(SDef.StageBase):
        def _query(self):
            success_conditions = [self.agent['has_tray'] == False]
            self._flag(any(success_conditions))
        def _end(self):
            super(StageDef.Unloading, self)._end()
            self.agent.properties['load'] = 0
            # logmsg(level='warn', category='STAGE', id=self.agent.agent_id, msg="Total load: %s" % self.agent.properties['load'])


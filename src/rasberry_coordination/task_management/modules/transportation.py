"""Transportation"""

from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.actions.action_manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef
from rasberry_coordination.robot import Robot as RobotInterface_Old
from rospy import Time, Duration

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class picker(IDef.RasberryInterfacing_ProtocolManager):
        def car_CALLED(self):
            self.agent.add_task(task_name='transportation_request_field_courier')
            self.agent['start_time'] = Time.now()

        def car_LOADED(self):
            self.agent['has_tray'] = False

        def car_CANCEL(self):
            if self.agent['id'] and self.agent['name']=='transportation_request_field_courier':
                self.agent.set_interrupt('reset', 'transportation', self.agent['id'], "Task")

    class field_courier(IDef.AgentInterface):
        def __init__(self, agent, sub='/r/get_states', pub='/r/set_states'):
            #E.g. If a cancellation request is triggered by picker, we much release
            #E.g. If a cancellation request is triggered by field_storage, we much restart
            self.release_triggers = ['self', 'picker', 'toc']
            self.restart_triggers = ['field_storage']

            responses={'PAUSE':self.pause, 'UNPAUSE':self.unpause, 'RELEASE':self.release}
            super(InterfaceDef.transportation_field_courier, self).__init__(agent, responses, sub=sub, pub=pub)

        def pause(self): self.agent.set_interrupt('pause', 'transportation', self.agent['id'])
        def unpause(self): self.agent.set_interrupt('unpause', 'transportation', self.agent['id'])
        def release(self): self.agent.set_interrupt('reset', 'transportation', self.agent['id'])

    class field_storage(IDef.AgentInterface):
        def __init__(self, agent, sub='/uar/get_states', pub='/uar/set_states'):
            self.release_triggers = ['self', 'toc']
            self.restart_triggers = ['thorvald']

            responses={'UNLOADED': self.unloaded}
            super(InterfaceDef.transportation_field_storage, self).__init__(agent, responses, sub=sub, pub=pub)

            #These need a new home
            self.agent.request_admittance = []
            self.agent.has_presence = False  # used for routing (swap key for physical?)

        def unloaded(self): self.agent['has_tray'] = True



class TaskDef(object):
    """ Initialisation """

    # @classmethod
    # def transportation_picker_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
    #     return TDef.human_localisation(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_field_courier_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        if 'load' not in agent.local_properties: agent.local_properties['load'] = 0


    """ Idle Task Stages for Transportation Agents """
    @classmethod
    def transportation_picker_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        pass
        # agent.modules['transportation'].interface.notify("car_INIT2")
    @classmethod
    def transportation_field_courier_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        LP = agent.local_properties
        MP = agent.module_properties

        #If agent is at max capacity deliver load
        agent.local_properties['load'] = int(agent.local_properties['load'])
        if LP['load'] >= int(MP['max_load']):
            return TaskDef.transportation_deliver_load(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def transportation_field_storage_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        #If agents are waiting to visit, begin transportation field_storage
        #Otherwise wait idle
        if len(agent.request_admittance) > 0:
            return TaskDef.transportation_field_storage(agent=agent, task_id=task_id, details=details, contacts=contacts)
        else:
            return TaskDef.idle_field_storage_def(agent=agent, task_id=task_id, details=details, contacts=contacts)


    """ Picker Tasks """
    @classmethod
    def transportation_request_field_courier(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
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

    """ FieldCourier Tasks """
    @classmethod
    def transportation_retrieve_load(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_retrieve_load",
                    details=details,
                    contacts=contacts,
                    initiator_id=initiator_id,
                    responder_id=agent.agent_id,
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.NavigateToPicker(agent),
                        StageDef.Loading(agent)
                    ]))
    @classmethod
    def transportation_deliver_load(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='transportation',
                    name="transportation_deliver_load",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignFieldStorage(agent),
                        SDef.AssignWaitNode(agent),
                        StageDef.AwaitFieldStorageAccess(agent),
                        StageDef.NavigateToFieldStorage(agent),
                        StageDef.Unloading(agent)
                    ]))

    """ Storage Tasks """
    @classmethod
    def idle_field_storage_def(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
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
    def transportation_field_storage(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
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


class StageDef(object):

    class IdleStorage(SDef.Idle):
        """Used to Idle a storage agent whilst awaiting a request for admittance"""
        #TODO: change this to idle, and make this condition an interface response?
        def _query(self):
            """Complete once there exists any agents requiring storage"""
            success_conditions = [len(self.agent.request_admittance) > 0] #TODO: this may prove error prone w/ _start
            self.flag(any(success_conditions))
            if any(success_conditions): print("admittance required: %s"%self.agent.request_admittance)
    class IdleFieldStorage(IdleStorage):
        pass
        def _end(self):
            """On completion, add an idle field_storage to the end of the buffer"""
            self.agent.add_task('transportation_field_storage_idle')

    """ Assignment-Based Task Stages (involves coordinator) """
    # class AssignFieldCourier(SDef.AssignAgent):
    #     """Used to identify an available field_courier to collect a load"""
    #     def __init__(self, agent):
    #         super(StageDef.AssignFieldCourier, self).__init__(agent, action_style='closest', agent_type='field_courier')
    #     def _end(self):
    #         """ On completion, notify picker of field_courier acceptance, and assign a retrieve load task to the field_courier"""
    #         super(StageDef.AssignFieldCourier, self)._end()
    #         self.agent.modules['transportation'].interface.notify("car_ACCEPT")
    #         self.agent['contacts']['field_courier'].add_task(task_name='transportation_retrieve_load',
    #                                                    task_id=self.agent['id'],
    #                                                    details={},
    #                                                    contacts={'picker': self.agent},
    #                                                    initiator_id=self.agent.agent_id)
    class AssignFieldCourier(SDef.ActionResponse):
        """Used to identify the closest field_courier."""
        def __init__(self, agent):
            """ Mark the details of the associated Action """
            super(StageDef.AssignFieldCourier, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='agent_descriptor', descriptor='field_courier', style='closest_agent')
            self.contact = 'field_courier'
        def _end(self):
            """ On completion, notify picker of field_courier acceptance, and assign a retrieve load task to the field_courier"""
            super(StageDef.AssignFieldCourier, self)._end()

            self.agent.modules['transportation'].interface.notify("car_ACCEPT")
            self.agent['contacts']['field_courier'].add_task(task_name='transportation_retrieve_load',
                                                             task_id=self.agent['id'],
                                                             details={},
                                                             contacts={'picker': self.agent},
                                                             initiator_id=self.agent.agent_id)
    # class AssignFieldStorage(SDef.AssignAgent):
    #     """Used to identify a storage location to deliver the load"""
    #     def __init__(self, agent):
    #         super(StageDef.AssignFieldStorage, self).__init__(agent, action_style='closest', agent_type='field_storage')
    #     def _end(self):
    #         """On completion, save the storage and add the field_courier's id to the storage's request_admittance list"""
    #         super(StageDef.AssignFieldStorage, self)._end()
    #         self.agent['contacts']['field_storage'].request_admittance.append(self.agent.agent_id)
    class AssignFieldStorage(SDef.ActionResponse):
        """Used to identify the closest field_storage."""
        def __init__(self, agent):
            """ Mark the details of the associated Action """
            super(StageDef.AssignFieldStorage, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='agent_descriptor', descriptor='field_storage', style='closest_agent')
            self.contact = 'field_storage'
        def _end(self):
            """ On completion, notify picker of field_courier acceptance, and assign a retrieve load task to the field_courier"""
            super(StageDef.AssignFieldStorage, self)._end()
            self.agent['contacts']['field_storage'].request_admittance.append(self.agent.agent_id)
    # class AcceptFieldCourier(SDef.AssignAgent):
    #     """Used to notify a pending field_courier of admittance"""
    #     def __init__(self, agent):
    #         super(StageDef.AcceptFieldCourier, self).__init__(agent, action_style='closest', agent_type='field_courier')
    #     def _start(self):
    #         """Initiate action details to identify the closest field_courier from those that request admittance"""
    #         super(StageDef.AcceptFieldCourier, self)._start()
    #         self.action['list'] = self.agent.request_admittance
    #     def _end(self):
    #         """On completion, save the action response and remove the field_courier from the request_admittance list"""
    #         super(StageDef.AcceptFieldCourier, self)._end(contact_type='initiator_id')
    #         logmsg(category="stage", msg="Admitted: %s from %s" % (self.agent['contacts']['field_courier'].agent_id, self.agent.request_admittance))
    #         logmsg(category="stage", msg="AcceptFieldCourier: stage_complete=%s" % self.stage_complete)
    #         self.agent.request_admittance.remove(self.agent['contacts']['field_courier'].agent_id)
    class AcceptFieldCourier(SDef.ActionResponse):
        """Used to identify the closest field_storage."""
        def __init__(self, agent):
            """ Mark the details of the associated Action """
            super(StageDef.AcceptFieldCourier, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='agent_list', list=self.agent.request_admittance, style='closest_agent')
            self.contact = 'field_courier'
        def _end(self):
            """ On completion, notify picker of field_courier acceptance, and assign a retrieve load task to the field_courier"""
            super(StageDef.AcceptFieldCourier, self)._end()
            logmsg(category="stage", msg="Admitted: %s from %s" % (self.agent['contacts']['field_courier'].agent_id, self.agent.request_admittance))
            logmsg(category="stage", msg="AcceptFieldCourier: stage_complete=%s" % self.stage_complete)
            self.agent.request_admittance.remove(self.agent['contacts']['field_courier'].agent_id)




    """ Idle for Pending """
    class AwaitFieldCourier(SDef.Idle):
        """Used to idle the agent until a field_courier has arrived"""
        def __repr__(self):
            """Attach id of agent to class name"""
            if 'field_courier' in self.agent['contacts']:
                return "%s(%s)"%(self.get_class(), self.agent['contacts']['field_courier'].agent_id)
            else:
                return "%s()" % (self.get_class())
        def _start(self):
            print("\n"*10)
            logmsg(level="error", category="stage", msg="AwaitFieldCourier started")
            super(StageDef.AwaitFieldCourier, self)._start()
        def _query(self):
            """Complete once the associated field_courier has arrived at the agents location"""
            success_conditions = [self.agent['contacts']['field_courier'].location(accurate=True) == self.agent.location()]
            self.flag(any(success_conditions))
        def _end(self):
            """On completion, notify the picker of ARRIVAL"""
            self.agent.modules['transportation'].interface.notify("car_ARRIVED")
    class AwaitStoreAccess(SDef.Idle):
        """Used to idle the field_courier until the storage location has accepted admittance"""
        def __repr__(self):
            """Attach id of agent to class name"""
            if self.storage_type in self.agent['contacts']:
                return "%s(%s)"%(self.get_class(), self.agent['contacts'][self.storage_type].agent_id)
            return "%s()" % (self.get_class())
        def __init__(self, agent, storage_type):
            super(StageDef.AwaitStoreAccess, self).__init__(agent)
            self.storage_type = storage_type
        def _query(self):
            """Complete if the field_courier assigned to the storage of interest is this agent"""
            storage = self.agent['contacts'][self.storage_type]
            if 'field_courier' not in storage['contacts']: return
            field_courier = storage['contacts']['field_courier']
            success_conditions = [field_courier.agent_id == self.agent.agent_id]
            self.flag(any(success_conditions))
    class AwaitFieldStorageAccess(AwaitStoreAccess):
        """Used to Idle the Field Courier till the field_storage accepts admittance"""
        def __init__(self, agent):
            """Specify the """
            super(StageDef.AwaitFieldStorageAccess, self).__init__(agent, storage_type='field_storage')

    """ Transportation Navigation Subclasses """
    class NavigateToPicker(SDef.NavigateToAgent):
        """Used to define the target for the navigation as the picker"""
        def __init__(self, agent):
            """Set navigation target as associated picker"""
            super(StageDef.NavigateToPicker, self).__init__(agent,  association='picker')
    class NavigateToFieldStorage(SDef.NavigateToAgent):
        """Used to define the target for the navigation as the field_storage"""
        def __init__(self, agent):
            """Set navigation target as associated field_storage"""
            super(StageDef.NavigateToFieldStorage, self).__init__(agent, association='field_storage')

    """ FieldCourier Load Modifiers """
    class TimeoutFlagModifier(SDef.StageBase):
        """Used to idle till timeout or a flag is set"""
        def _start(self, timeout_type, flag, default):
            """Define the completion flag and timeout"""
            super(StageDef.TimeoutFlagModifier, self)._start()
            self.agent[flag] = default
            self.trigger_flag = flag
            self.default = default
            self.timeout = Duration(secs=fetch_property('transportation', timeout_type))
        def _query(self):
            """Complete once has_tray flag is triggered by interface or timeout completes"""
            success_conditions = [Time.now() - self.start_time > self.timeout,
                                  self.agent[self.trigger_flag] != self.default]
            self.flag(any(success_conditions))


        def _end(self):
            """On Completion, set the field_courier's flag and notify picker"""
            super(StageDef.TimeoutFlagModifier, self)._end()
            self.agent['contacts']['field_courier'][self.trigger_flag] = self.default
            self.agent.modules['transportation'].interface.notify("car_COMPLETE")
    class LoadFieldCourier(TimeoutFlagModifier):
        """Used to define completion details for when the field_courier can be consideded loaded"""
        def _start(self):
            """Define the flag default as True and the timeout as the tranportation/wait_loading property"""
            super(StageDef.LoadFieldCourier, self)._start(timeout_type='wait_loading', flag='has_tray', default=True)
    class UnloadFieldCourier(TimeoutFlagModifier):
        """Used to define completion details for when the field_courier can be consideded unloaded"""
        def _start(self):
            """Define the flag default as False and the timeout as the tranportation/wait_unloading property"""
            super(StageDef.UnloadFieldCourier, self)._start(timeout_type='wait_unloading', flag='has_tray', default=False)

    """ Loading Modifiers for Courier """
    class Loading(SDef.StageBase):
        """Used for awaiting a change-of-state from the picker"""
        def _query(self):
            """Complete once agents's has_tray flag is true"""
            success_conditions = [self.agent['has_tray'] == True]
            self.flag(any(success_conditions))
        def _end(self):
            """On completion, increment the field_courier's total load by 1"""
            super(StageDef.Loading, self)._end()
            self.agent.local_properties['load'] += 1
    class Unloading(SDef.StageBase):
        """Used for awaiting a change-of-state from the storage"""
        def _query(self):
            """Complete once agents's has_tray flag is false"""
            success_conditions = [self.agent['has_tray'] == False]
            self.flag(any(success_conditions))
        def _end(self):
            """On completion, reset the field_courier's total load to 0"""
            super(StageDef.Unloading, self)._end()
            self.agent.local_properties['load'] = 0

from std_msgs.msg import String as Str
from rasberry_coordination.coordinator_tools import logmsg
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.task_management.base import StageDef as RootDef, InterfaceDef as CommsDef
from rasberry_coordination.robot import Robot as RobotInterface_Old

from rospy import Time, Duration

class InterfaceDef(object):
    class transportation_picker(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/car_client/get_states', pub='/car_client/set_states'):
            responses = {'CALLED': self.called, 'LOADED': self.loaded, 'INIT': self.reset}
            super(InterfaceDef.transportation_picker, self).__init__(agent, responses, sub=sub, pub=pub)


        def called(self):
            self.agent.start_new_task(task='transportation_request', details={'target_agent': self.agent})
            self.agent['start_time'] = Time.now()
        def loaded(self): self.agent['picker_has_tray'] = False
        def reset(self): pass

    class transportation_storage(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/uar/get_states', pub='/uar/set_states'):
            responses={'UNLOADED': self.unloaded, 'OFFLINE': self.offline, 'ONLINE': self.online}
            super(InterfaceDef.transportation_storage, self).__init__(agent, responses, sub=sub, pub=pub)

            #These need a new home
            self.agent.request_admittance = []
            self.agent.has_presence = False  # used for routing (swap out for physical?)

        def unloaded(self): self.agent['storage_has_tray'] = True
        def offline(self): pass
        def online(self): pass

    class transportation_courier(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/r/get_states', pub='/r/set_states'):
            responses={'PAUSE':self.pause, 'UNPAUSE':self.unpause, 'RELEASE':self.release}
            super(InterfaceDef.transportation_courier, self).__init__(agent, responses, sub=sub, pub=pub)

            #These need a new home
            self.agent.temp_interface = RobotInterface_Old(self.agent.agent_id)
            self.agent.start_idle_task('init_courier')

        def pause(self): self.agent.task_stage_list.insert(0, StageDef.Pause(self))
        def unpause(self): self.agent.registration = True
        def release(self): self.agent.task_stage_list = [] #+ inform associated robots

class TaskDef(object):
    """ Picker Logistics Transportation """
    @classmethod
    def transportation_request(cls, agent, details={}, task_id=None):
        agent.task_name = "transportation_request"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            RootDef.StartTask(agent, task_id),
            StageDef.AssignCourier(agent),
            # robot has accepted task (ACCEPT)
            StageDef.AwaitCourier(agent),
            # robot has arrived (ARRIVED)
            StageDef.LoadCourier(agent),
            # task is marked as complete (INIT)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
    @classmethod
    def transportation_courier(cls, agent, details={}, task_id=None):
        agent.task_name = "transportation_courier"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            RootDef.StartTask(agent, task_id),
            StageDef.NavigateToPicker(agent),
            StageDef.Loading(agent),

            StageDef.AssignStorage(agent),
            RootDef.AssignWaitNode(agent),

            StageDef.AwaitStoreAccess(agent),
            StageDef.NavigateToStorage(agent),
            StageDef.Unloading(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
    @classmethod
    def transportation_storage(cls, agent, details={}, task_id=None):
        agent.task_name = "transportation_storage"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            #-> store.admit_plz > 0
            RootDef.StartTask(agent, task_id),
            # -> True
            StageDef.AcceptCourier(agent),
            # -> store.admitance != None
            StageDef.AwaitCourier(agent),
            # -> courier.location == store.location
            StageDef.UnloadCourier(agent),
            # -> store.has_tray = False
            # StageDef.AwaitCourierExit(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
class StageDef(object):
    """ Assignment-Based Task Stages (involves coordinator) """
    class AssignCourier(RootDef.Assignment):
        def _start(self):
            super(StageDef.AssignCourier, self)._start() #defined as default setup
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'courier'
            self.action['agent_type'] = 'courier'  # local_storage/cold_storage
        def _notify_end(self):
            self.agent.interfaces['transportation_picker'].notify("ACCEPT")
        def __del__(self):
            """ On completion of assign courier, a courier should have been identified.
            As a result of the completion, the couier should be assigned a task, and be
            given details pertaining to its completion. This is done explicitly.
            """
            super(StageDef.AssignCourier, self).__del__()
            self.agent['courier'].start_new_task(details={'target_agent':self.agent},
                                                 task_id=self.agent['task_id'])
        def _summary(self):
            super(StageDef.AssignCourier, self)._summary()
            self.summary['_query'] = "agent[courier] not None"
            self.summary['_action'] = "find closest robot courier"
            self.summary['_del'] = "agent[courier][target]=agent"
            self.summary['_notify_end'] = "agent -> ACCEPT"
    class AssignStorage(RootDef.Assignment):
        def _start(self):
            super(StageDef.AssignStorage, self)._start()
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'storage'

            self.action['agent_type'] = 'storage' #local_storage/cold_storage
        def __del__(self):
            super(StageDef.AssignStorage, self).__del__()
            self.agent['storage'].request_admittance.append(self.agent.agent_id)
    class AcceptCourier(RootDef.Assignment):
        def _start(self):
            super(StageDef.AcceptCourier, self)._start()
            self.action['action_type'] = 'find_agent_from_list'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'courier'

            self.action['list'] = self.agent.request_admittance
        def __del__(self):
            super(StageDef.AcceptCourier, self).__del__()
            logmsg(category="stage", msg="Admitted: %s from %s" % (self.agent['courier'].agent_id, self.agent.request_admittance))
            self.agent.request_admittance.remove(self.agent['courier'].agent_id)
        #On completion: courier being accepted is found @ agent['courier']
        # def __del__(self):
        #     print("\n\n\n")
        #     print("AcceptCourier __del__()")
        #     print(self.agent['courier'].__dict__)
        #     print("\n-----\n")
        #     print(self.agent.__dict__)
        #     print("\n\n\n-----")

    """ Idle Actions for Pending Actions """
    class AwaitCourier(RootDef.Idle): #PICKER + STORAGE
        def _query(self):
            success_conditions = [self.agent['courier'].location() == self.agent.location()]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            if 'transportation_picker' in self.agent.interfaces:
                self.agent.interfaces['transportation_picker'].notify("ARRIVED")
            else:
                self.agent.interfaces['transportation_storage'].notify("ARRIVED")
        def _summary(self):
            super(StageDef.AwaitCourier, self)._summary()
            self.summary['_query'] = "courier @ agent"
            self.summary['_notify_end'] = "agent -> ARRIVED"
    class AwaitStoreAccess(RootDef.Idle):
        #While waiting for store access, move to a wait_node
        # (ideally this should be identified by the coordinator and assigned dynamically)
        def _start(self):
            super(StageDef.AwaitStoreAccess, self)._start()
            self.route_required = True
            # Despite moving to a wait node, dont end task on arrival #huh?

            # print("\n\n\n")
            # print("AwaitStoreAccess _Start()")
            # print(self.agent['storage'].__dict__)
            # print("\n-----\n")
            # print(self.agent.__dict__)
            # print("\n\n\n-----")

        def _query(self):
            courier = self.agent['storage']['courier']
            if not courier:
                return
            success_conditions = [courier.agent_id == self.agent.agent_id]
            self.agent.flag(any(success_conditions))
        # def __del__(self):
        #     print("await_store stage completed")
        #     print(self.agent['storage'].__dict__)

    """ Navigation SubSubclasses """
    class NavigateToPicker(RootDef.NavigateToAgent):
        def __init__(self, agent): super(StageDef.NavigateToPicker, self).__init__(agent,  target_identifier='target_agent')
    class NavigateToStorage(RootDef.NavigateToAgent):
        def __init__(self, agent): super(StageDef.NavigateToStorage, self).__init__(agent, target_identifier='storage')

    """ Loading Modifiers for Picker and Storage """
    class LoadModifier(RootDef.StageBase):
        def __del__(self):
            super(StageDef.LoadModifier, self)._start()
            # print("setting courier['tray_present'] to %s" % not self.end_requirement)
            self.agent['courier']['tray_present'] = not self.end_requirement
    class LoadCourier(LoadModifier): #PICKER
        def __init__(self, agent):
            super(StageDef.LoadCourier, self).__init__(agent)
            self.wait_timeout = Duration(secs=5)
            self.end_requirement = False #flag must be this to end task
            self.agent['picker_has_tray'] = True #local flag
        def _query(self):
            success_conditions = [Time.now() - self.start_time > self.wait_timeout,
                                 self.agent['picker_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.interfaces['transportation_picker'].notify("INIT")
    class UnloadCourier(LoadModifier): #STORAGE
        def __init__(self, agent):
            super(StageDef.UnloadCourier, self).__init__(agent)
            self.wait_timeout = Duration(secs=5)
            self.end_requirement = True #flag must be this to end task
            self.agent['storage_has_tray'] = False  # local flag
        def _query(self):
            success_conditions = [Time.now() - self.start_time > self.wait_timeout,
                                  self.agent['storage_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))

    """ Loading Modifiers for Courier """
    class Loading(RootDef.StageBase):
        def _query(self):
            success_conditions = [self.agent['tray_present']]
            # if any(success_conditions):
            #     print("Loading stage complete")
            self.agent.flag(any(success_conditions))
    class Unloading(RootDef.StageBase):
        def _query(self):
            success_conditions = [not self.agent['tray_present']]
            self.agent.flag(any(success_conditions))

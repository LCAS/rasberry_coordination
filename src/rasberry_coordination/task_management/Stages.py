""" Task Stages:

BaseStage.methods:
    __repr__: #print class
    __init__: #save agent
    __call__: #begin task
    __del__:  #del task_details
    _query:   #check completion
    _target:  #define target
    _notify:  #inform agent

BaseStage.properties:
    agent

Agent.required_properties:
    task_details

Agent.methods:
    load_idle_task()

"""
"""
BaseStage.inheritance:
    StartTask:
    TargetedMovement:
        EdgeUVTreatment:
        RowUVTreatment:
    Await:
        AwaitAgentArrival:
        AwaitAgentDismissal:
        AwaitLoading:
        AwaitUnloading:
    Wait:
        WaitDuration:
        Pause:
"""

"""
All task specific details must be saved in agent.task_details
Exceptions: 
- replan_required (needed to identify across agents)
- location (standard for planning)
- target   (standard for planning)
- ...
- admittance (requied for communication)

Interference with tasks should not be direct between agents.
- advencement achieved by agents with their own _query
- modify agent.tray_present, not agent.task_stage_completed

Methods called within coordinator:
- closest_wait_node()
- closest_storage() #closest_target
- closest_robot()   #closest_target
"""


"""
Rules for Custom Stages:
- all cross-stage details must be stored in Stage.agent.task_details
- all communication must be limited to the Stage._notify* methods
- all overloading of __init__ must include a call to super
- all overloading of _start must include a call to super
- all overloading of __del__ must include a call to super

Notes:
- __init__ is called when the task stages are added to an agent
- __repr__ can be overloaded to include more information
- _start is called once when the stask is at the head of task_stage_list
- _start can be called a second time by setting Stage.task_details['new_stage'] to True
- _query is called on every iteration of Coordinator.run, except when task_progression is paused
- 

(Tim Peters, The Zen of Python):
"Special cases aren't special enough to break the rules." / "Although practicality beats purity."
-> `python -c "import this"`
"""

class StageDef(object):
    class StageBase(object):
        def __repr__(self):
            return str(self.__class__).replace("<class '__main__.","").replace("'>","")
        def __init__(self, agent):
            self.agent = agent
            self.agent['new_stage'] = True
        def _start(self):
            self.agent['new_stage'] = False
        def _notify_start(self):
            pass
        def _query(self):
            success_conditions = []  # What should be queried to tell if the stage is completed?
            self.agent.flag = any(success_conditions)
        def _notify_end(self):
            pass
        def __del__(self):
            self.agent['stage_complete_flag'] = False
            # del self.agent.task_details
            # remove goal node?
            # remove routes?
            # communication with agent?
            pass

    """ Logistics Task Stages """
    class StartTask(StageBase):
        def _query(self):
            self.agent.flag(True)

    """ Idle Placeholder Task """
    class IdleTask(StageBase):
        def _query(self):
            success_conditions = [self.agent.task_id]
            self.agent.flag(any(success_conditions))
    class IdlePicker(IdleTask):
        def __del__(self):
            super(IdlePicker, self)._start()
            self.agent.new_task('transportation_picker', {'target_agent': self.agent})
    class IdleCourier(IdleTask):
        def __del__(self):
            super(IdleCourier, self)._start()
            self.agent.new_task('transportation_courier', {'target_agent': self.agent})
    class IdleStorage(IdleTask):
        def _query(self):
            success_conditions = [len(self.agent.request_admittance) > 0]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            super(IdleStorage, self)._start()
            self.admittance = self.request_admittance[0]
            self.agent.new_task('transportation_storage', {'target_agent': self.agent})

    """ Assignment-Based Task Stages (involves coordinator) """
    class Assignment(StageBase):
        def _start(self, category='find_agent', style='closest', target_type='robot'):
            super(Assignment, self)._start()
            self.agent.task_details.update({'coordinator_action_required':True,
                                            'service_category':category,
                                            'service_type':style,
                                            'service_conditions':('type',target_type)})
        def _query(self):
            success_conditions = [self.agent['recipient'] != None]
            self.agent.flag(any(success_conditions))
    class AssignCourier(Assignment):
        def _start(self):
            super(AssignCourier, self)._start(target_type="robot")#/"courier"?
        def _notify_end(self):
            self.agent.interface.set_state("ACCEPT")
        def __del__(self):
            super(AssignCourier, self)._start()
            agent.task_details['recipient'].new_task("transportation_robot", {'target_agent': self.agent})
            self.agent.task_details['coordinator_action_required'] = False
    class AssignStorage(Assignment):
        def _start(self):
            super(AssignStorage, self)._start(target_type="storage")
        def __del__(self):
            super(AssignStorage, self)._start()
            self.agent.task_details['storage'].request_admittance.append(self.agent)
    class AssignWaitNode(Assignment):
        def _start(self):
            super(AssignWaitNode, self)._start(category='find_node', target_type="wait_node")

    """ Idle Actions for Pending Actions """
    class Idle(StageBase):
        pass
    class AwaitCourier(Idle): #PICKER + STORAGE
        def _query(self):
            courier = self.agent.task_details['recipient']
            success_conditions = [courier.location == courier['target']]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.interface.set_state("ARRIVED")
    class AwaitStoreAccess(Idle):
        #While waiting for store access, move to a wait_node
        # (ideally this should be identified by the coordinator and assigned dynamically)
        def _start(self):
            super(AwaitStoreAccess, self)._start()
            self.agent.replan_required = True # Though moving to a wait node, dont end task on arrival
        def _query(self):
            store = self.agent.task_details['recipient']
            success_conditions = [store.admittance == self.agent.agent_id]
            self.agent.flag(any(success_conditions))

    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        def _start(self):
            super(Navigation, self)._start()
            self.agent.replan_required = True
        def _query(self):
            success_conditions = [self.agent.location ==
                                  self.agent.task_details['target_agent'].location]
            self.agent.flag(any(success_conditions))
    class NavigateToPicker(Navigation):
        pass
    class NavigateToStorage(Navigation):
        pass

    """ Loading Modifiers for Picker and Storage """
    class LoadModifier(StageBase):
        def __del__(self):
            super(LoadModifier, self)._start()
            self.agent.task_details['robot'].task_details['tray_present'] = self.end_requirement
    class LoadCourier(LoadModifier): #PICKER
        def __init__(self, agent):
            super(LoadCourier, self).__init__(agent)
            self.wait_timeout = 25
            self.end_requirement = False #flag must be this to end task
            self.agent.task_details['picker_has_tray'] = True #local flag
        def _query(self):
            success_conditions = [Now() - self.agent.start_time > self.wait_timeout,
                self.agent['picker_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.interface.set_state("INIT")
    class UnloadCourier(LoadModifier): #STORAGE
        def __init__(self, agent):
            super(LoadCourier, self).__init__(agent)
            self.wait_timeout = 50
            self.end_requirement = True #flag must be this to end task
            self.agent.task_details['storage_has_tray'] = False  # local flag
        def _query(self):
            success_conditions = [Now() - self.agent.start_time > self.wait_timeout,
                                  self.agent.task_details['storage_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))


    """ Loading Modifiers for Courier """
    class Loading(StageBase):
        def _query(self):
            success_conditions = [self.agent.task_details['tray_is_with_courier']]
            self.agent.flag(any(success_conditions))
    class Unloading(StageBase):
        def _query(self):
            success_conditions = [not self.agent.task_details['tray_is_with_courier']]
            self.agent.flag(any(success_conditions))




















"""
    if True:
        "" Meta Stages ""
        class Pause(StageBase):
            def query(self):
                success_conditions = [self.agent.registration]
                self.agent.flag(any(success_conditions))

        "" Move to Target Stages ""
        class TargetedMovement(StageBase):
            def __repr__(self):
                cls_name = super(StageDef.TargetedMovement, self).__repr__()
                return "%s(%s)"%(cls_name,self.target)
            def __init__(self, agent, target):
                super(StageDef.TargetedMovement, self).__init__(agent)
                self.target = target
            def set_nav_target(self):
                self.agent.target = self.target
            def __del__(self):
                self.agent.target = None
        class Navigation(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class EdgeUVTreatment(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class RowUVTreatment(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class EdgeDataCollection(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class RowDataCollection(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))

        "" Wait for Time Expiry Stages "
        class AwaitLoad(StageBase):
            def query(self):
                success_conditions = [self.agent.tray_loaded]
                self.agent.flag(any(success_conditions))
        class AwaitUnload(StageBase):
            def query(self):
                success_conditions = [not self.agent.tray_loaded]
                self.agent.flag(any(success_conditions))
                
                

    # "" Access Request for Storage "" #?
    # class RequestAccess(StageBase):
    #     def __call__(self):
    #         self.target = self.agent.wait_node
    #         self.agent.replan_required = True
    #     def _query(self):
    #         success_conditions = [len(self.agent.request_admittance) > 0]
    #         self.agent.flag(any(success_conditions))
    #     def __del__(self):
    #         self.agent.admittance = self.agent.request_admittance.pop(0)

                

        #"""  """
        class Wait(StageBase):
            def __init__(self, agent, timeout):
                super(StageDef.Wait, self).__init__(agent)
                self.wait_timeout = timeout
        class LoadCourier(Wait):
            def query(self):
                success_conditions = [Now() - self.agent.start_time > self.wait_timeout]
                self.agent.flag(any(success_conditions))
        class UnloadCourier(Wait):
            def query(self):
                success_conditions = [Now() - self.agent.start_time > self.wait_timeout]
                self.agent.flag(any(success_conditions))

        class Assign(StageBase):
            def __call__(self):
                self.agent.storage = closest_storage_location()
                self.agent.storage.new_task('store', {'robot': self.agent})
"""
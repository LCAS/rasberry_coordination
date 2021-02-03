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
    Non-Task-Based Attributes:
        - location
        - task_details #not directly accessed
    Non-Task-Based Methods:
        - .flag()
        - .new_task()   # 
        - .set_status() #communication
    Values which must persist beyond the end of task
        - store.request_admittance
        

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
        #we need to consider this fully
        def _update(self):
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
        def _start(self):
            self.agent.task_details = {}
        def _query(self):
            success_conditions = [self.agent.task_details]
            self.agent.flag(any(success_conditions))
    class IdlePicker(IdleTask):  #TODO: for picker task to start, any value must be assigned into task_details (TASK_ID, start_time would be beter though?)
        def __del__(self):
            self.agent.new_task('transportation_picker', {'target_agent': self.agent}) #self.agent doesnt exist...
    class IdleCourier(IdleTask):
        def __del__(self):
            self.agent.new_task(self.agent['task_type'], {'target_agent': self.agent['target']})??? #ADD ABILITY TO FOLLOW PICKER
    class IdleStorage(IdleTask):
        def _query(self):
            success_conditions = [len(self.agent.request_admittance) > 0] #TODO: this may prove error prone w/ _start
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.admittance = self.agent.request_admittance[0]
            self.agent.new_task('transportation_storage', {'target_agent': self.agent})


    """ Assignment-Based Task Stages (involves coordinator) """
    class Assignment(StageBase):
        def _start(self, category='find_agent', style='closest', target_type='robot', response='courier'):
            super(Assignment, self)._start()
            self.agent['coordinator_action_required'] = True
            self.agent['service_category'] = category
            self.agent['service_type'] = style
            self.agent['service_conditions'] = {'key':'type','val':target_type} #TODO: this is badly done
            self.agent['response_location'] = response
        def _query(self):
            success_conditions = [self.agent['recipient'] != None]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.agent['coordinator_action_required'] = False
    class AssignCourier(Assignment):
        def _start(self):
            super(AssignCourier, self)._start(response='courier',target_type="robot")
        def _notify_end(self):
            self.agent.set_state("ACCEPT")
        def __del__(self):
            super(AssignCourier, self).__del__()
            self.agent['courier']['target_agent'] = self.agent #picker introduces themselves (which starts courier task)
    class AssignStorage(Assignment):
        def _start(self):
            super(AssignStorage, self)._start(response='storage', target_type="storage")
        def __del__(self):
            super(AssignStorage, self).__del__()
            self.agent['storage'].request_admittance.append(self.agent)
    class AssignWaitNode(Assignment):
        def _start(self):
            super(AssignWaitNode, self)._start(response='wait_node', category='find_node', target_type="wait_node")


    """ Idle Actions for Pending Actions """
    class Idle(StageBase):
        pass
    class AwaitCourier(Idle): #PICKER + STORAGE
        def _query(self):
            courier = self.agent['recipient']
            success_conditions = [courier.location == courier['target']]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.set_state("ARRIVED")
    class AwaitStoreAccess(Idle):
        #While waiting for store access, move to a wait_node
        # (ideally this should be identified by the coordinator and assigned dynamically)
        def _start(self):
            super(AwaitStoreAccess, self)._start()
            self.agent['replan_required'] = True # Though moving to a wait node, dont end task on arrival
        def _query(self):
            store = self.agent['recipient']
            success_conditions = [store['admittance'] == self.agent.agent_id]
            self.agent.flag(any(success_conditions))


    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        def _start(self):
            super(Navigation, self)._start()
            self.agent['replan_required'] = True
        def _query(self):
            success_conditions = [self.agent.location ==
                                  self.agent['target_agent'].location]
            self.agent.flag(any(success_conditions))
    class NavigateToPicker(Navigation):
        pass
    class NavigateToStorage(Navigation):
        pass
    class FollowAgent(StageBase):
        def _update(self):
            self.agent['target'] = self.agent['following'].location
        def _query(self):
            success_conditions = [self.agent['end_follow']]
            self.agent.flag(any(success_conditions))

    """ Loading Modifiers for Picker and Storage """
    class LoadModifier(StageBase):
        def __del__(self):
            super(LoadModifier, self)._start()
            self.agent['robot']['tray_present'] = self.end_requirement
    class LoadCourier(LoadModifier): #PICKER
        def __init__(self, agent):
            super(LoadCourier, self).__init__(agent)
            self.wait_timeout = 25
            self.end_requirement = False #flag must be this to end task
            self.agent['picker_has_tray'] = True #local flag
        def _query(self):
            success_conditions = [Now() - self.agent['start_time'] > self.wait_timeout,
                                 self.agent['picker_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.set_state("INIT")
    class UnloadCourier(LoadModifier): #STORAGE
        def __init__(self, agent):
            super(LoadCourier, self).__init__(agent)
            self.wait_timeout = 50
            self.end_requirement = True #flag must be this to end task
            self.agent['storage_has_tray'] = False  # local flag
        def _query(self):
            success_conditions = [Now() - self.agent['start_time'] > self.wait_timeout,
                                  self.agent['storage_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))


    """ Loading Modifiers for Courier """
    class Loading(StageBase):
        def _query(self):
            success_conditions = [self.agent['tray_is_with_courier']]
            self.agent.flag(any(success_conditions))
    class Unloading(StageBase):
        def _query(self):
            success_conditions = [not self.agent['tray_is_with_courier']]
            self.agent.flag(any(success_conditions))


    """ SSI Task Auction """
    class AwaitTaskAuction(StageBase):
        def _query(self):
            success_conditions = [self.agent['auction_to_begin'] == True]
            self.agent.flag(any(success_conditions))
    class TaskAuction(StageBase):
        def _start(self):
            self.agent['coordinator_service_required'] = True
        def _query(self):
            success_conditions = [True]
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
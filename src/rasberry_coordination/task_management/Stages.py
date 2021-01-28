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
- ???
- admittnce (requied for communication)
Interference with tasks should not be direct between agents.
- advencement achieved by agents with their own _query
- modify agent.tray_present, not agent.task_stage_completed
Methods called within coordinator:
- closest_wait_node()
- closest_storage() #closest_target
- closest_robot()   #closest_target
"""

class StageDef(object):
    class StageBase(object):
        def __repr__(self):
            return str(self.__class__).replace("<class '__main__.","").replace("'>","")
        def __init__(self, agent):
            self.agent = agent
        def __call__(self):
            pass
        def _query(self):
            success_conditions = []  # What should be queried to tell if the stage is completed?
            self.agent.flag = any(success_conditions)
        def _target(self):
            self.agent.task_details['target'] = self.agent.current_node
            #self.agent.target = self.agent.location
        def __del__(self):
            del self.agent.task_details
            # remove goal node?
            # remove routes?
            # communication with agent?
            pass

    """ Logistics Task Stages """
    class StartTask(StageBase):
        def _query(self):
            self.agent.flag(True)
    class Idle(StageBase):
        pass

    class Assignment(StageBase):
        def __call__(self):
            self.agent.task_details['recipient'] = COORDINATOR.closest_target(self.agent)
            self.agent.task_details['recipient'].new_transportation_task({'target_agent': self.agent})
        def _query(self):
            success_conditions = [self.agent.task_details['recipient'] != None]
            self.agent.flag(any(success_conditions))
            #if not success_conditions:
            #    self()
    class AssignCourier(Assignment):
        def _notify(self):
            self.agent.interface.set_state("ASSIGNED")
    class AssignStorage(Assignment):
        pass
    """
    Coordinator.ClosestTarget(agent):
        if str(agent.stage) == AssignCourier:
            return Coordinator.closest_picker(agent)
        elif str(agent.stage) == AssignStorage:
            return Coordinator.closest_storage(agent)
        else:
            return None
    CourierAgent.new_transportation_task(details):
        self.new_task('transportation_courier', {'target_agent': self.agent})
    StorageAgent.new_transportation_task(details):
        self.new_task('transportation_storage', {'target_agent': self.agent})
    """

    class RequestAccess(StageBase):
        def __call__(self):
            self.target = self.agent.wait_node
            self.agent.replan_required = True
        def _query(self):
            success_conditions = [len(self.agent.request_admittance) > 0]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.agent.admittance = self.agent.request_admittance.pop(0)

    class AwaitStoreAccess(StageBase):
        #While waiting for store access, move to a wait_node
        # (ideally this should be identified by the coordinator and assigned dynamically)
        def __init__(self, agent):
            super(StageDef.AwaitStoreAccess, self).__init__(agent)
            self.agent.task_details['wait_node'] = COORDINATOR.closest_wait_node(self.agent)
            self.target = self.agent.task_details['wait_node']
        def __call__(self):
            self.agent.replan_required = True
            # Though moving to a wait node, dont end task on arrival
        def _query(self):
            store = self.agent.task_details['recipient']
            success_conditions = [store.admittance == self.agent.agent_id]
            self.agent.flag(any(success_conditions))
    class AwaitCourier(StageBase): #PICKER + STORAGE
        def _query(self):
            courier = self.agent.task_details['recipient']
            success_conditions = [courier.location == courier.task_details['target']]
            self.agent.flag(any(success_conditions))

    class Navigation(StageBase): #ROBOT
        def __init__(self, agent, target):
            super(StageDef.Navigation, self).__init__(agent)
            self.target = target
        def __call__(self):
            self.agent.replan_required = True
        def _query(self):
            success_conditions = [self.agent.location == self.agent.task_details['target']]
            self.agent.flag(any(success_conditions))

    class LoadModifier(StageBase):
        def _query(self):
            success_conditions = [Now() - self.agent.start_time > self.wait_timeout]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.agent.task_details['robot'].task_details['tray_present'] = self.set_tray
    class LoadCourier(LoadModifier): #PICKER
        def __call__(self):
            self.set_tray = True
            self.wait_timeout = 25
            self.agent.interface.set_state("LOADING")
    class UnloadCourier(LoadModifier): #STORAGE
        def __call__(self):
            self.set_tray = False
            self.wait_timeout = 50
            self.agent.interface.set_state("UNLOADING")

    class Loading(StageBase):
        def _query(self):
            success_conditions = [self.agent.task_details['tray_present']]
            self.agent.flag(any(success_conditions))
    class Unloading(StageBase):
        def _query(self):
            success_conditions = [not self.agent.task_details['tray_present']]
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